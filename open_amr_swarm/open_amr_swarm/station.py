"""Arm station model (pure Python, no ROS): pallet stock, the bay handshake and faults. Used by station_agent.

A station is a fixed arm cell with one bay the robots stand on and `len(pallets)` staged pallet poses.
  receiving  depalletizes: pallet -> robot deck. A box leaves its pallet when the arm picks it; an emptied pallet is
             replaced by a full one `swap_s` later (next truck unloaded by the dock crew).
  outbound   palletizes: robot deck -> pallet. A box counts on the pallet once placed; a full pallet waits `swap_s`
             for pickup (forklift/truck), then an empty one is staged in its place.
The arm finishes one pallet before starting the next (the emptiest non-empty / the fullest non-full), so the other
pose is the buffer that keeps it working while a pallet is swapped.
Handshake: the robot waiting at the bay (status AT_STATION, its task) is served once; `served` names the last task
completed there, which is what the robot waits for. A fault interrupts a transfer in progress: the box goes back on
the pallet (receiving) or stays on the robot (outbound) and the transfer restarts once the fault clears.
"""
from dataclasses import dataclass

IDLE, BUSY, FAULT, STARVED = 0, 1, 2, 3
STATE_NAMES = {IDLE: 'idle', BUSY: 'busy', FAULT: 'fault', STARVED: 'starved'}


@dataclass
class StationParams:
    role: str                  # 'receiving' | 'outbound'
    capacity: int = 24         # boxes per full pallet: EUR pallet 1.2 x 0.8 m, 3 x 2 boxes of 0.35 m per layer, 4 layers
    cycle_s: float = 8.0       # one transfer: arm pick/place (~6 s, UR10e-class palletizing) + deck conveyor (0.35 m / 0.2 m/s)
    swap_s: float = 180.0      # pallet replaced (receiving) / picked up and replaced (outbound) this long after empty / full


@dataclass
class Started:
    robot: str
    task: str
    pallet: int
    slot: int


class Station:
    def __init__(self, p, pallets):
        assert p.role in ('receiving', 'outbound')
        self.p = p
        self.pallets = [max(0, min(p.capacity, int(n))) for n in pallets]
        self.swap_at = [None] * len(self.pallets)
        self.state, self.robot, self.task, self.served = IDLE, '', '', ''
        self.active, self.slot, self.done_at = None, None, 0.0
        self.transfers = 0
        self.log = []                 # (time, text) of notable events, drained by the caller
        self._started = False         # pallets that start spent get their swap timer on the first step

    @property
    def receiving(self):
        return self.p.role == 'receiving'

    def _spent(self, n):
        return n == 0 if self.receiving else n >= self.p.capacity

    def available(self):
        """Boxes the arm can hand out (receiving) or free slots it can fill (outbound), pallets being swapped excluded."""
        if self.receiving:
            return sum(n for i, n in enumerate(self.pallets) if self.swap_at[i] is None)
        return sum(self.p.capacity - n for i, n in enumerate(self.pallets) if self.swap_at[i] is None)

    def _pick_pallet(self):
        ok = [i for i, n in enumerate(self.pallets) if self.swap_at[i] is None and not self._spent(n)]
        if not ok:
            return None
        # finish one pallet before starting the next: emptiest (receiving) / fullest (outbound), then lower index
        key = (lambda i: (self.pallets[i], i)) if self.receiving else (lambda i: (-self.pallets[i], i))
        return min(ok, key=key)

    def step(self, now, at_bay, fault):
        """Advance to `now`. at_bay = (robot_id, task_id) of the robot waiting at the bay, or None.
        Returns Started when a transfer begins this step, else None."""
        self._swaps(now)
        if fault:
            if self.state == BUSY:
                self._abort(now, 'fault')
            if self.state != FAULT:
                self.log.append((now, 'FAULT: out of service'))
            self.state = FAULT
            return None
        if self.state == FAULT:
            self.log.append((now, 'fault cleared, back in service'))
            self.state = IDLE
        if self.state == BUSY:
            if at_bay is None or at_bay != (self.robot, self.task):
                self._abort(now, 'robot left the bay')
            elif now >= self.done_at:
                self._complete(now)
            else:
                return None
        if at_bay is not None and at_bay[1] and at_bay[1] != self.served:
            a = self._pick_pallet()
            if a is not None:
                if self.receiving:
                    slot = self._take(now, a)                  # the top box
                else:
                    slot = self.pallets[a]                     # next free slot
                self.state, (self.robot, self.task) = BUSY, at_bay
                self.active, self.slot, self.done_at = a, slot, now + self.p.cycle_s
                return Started(self.robot, self.task, a, slot)
        self.state = IDLE if self.available() > 0 else STARVED
        return None

    def _swaps(self, now):
        if not self._started:
            self._started = True
            self.swap_at = [now + self.p.swap_s if self._spent(n) else None for n in self.pallets]
        for i, t in enumerate(self.swap_at):
            if t is not None and now >= t:
                self.pallets[i] = self.p.capacity if self.receiving else 0
                self.swap_at[i] = None
                self.log.append((now, f'pallet {i} {"restocked (full)" if self.receiving else "shipped, empty pallet staged"}'))

    def _take(self, now, a):
        """Receiving: the top carton of pallet a leaves it (picked). Returns its slot."""
        self.pallets[a] -= 1
        if self.pallets[a] == 0:
            self.swap_at[a] = now + self.p.swap_s
            self.log.append((now, f'pallet {a} empty, next one due in {self.p.swap_s:.0f} s'))
        return self.pallets[a]

    def _put(self, now, a):
        """A carton lands on pallet a (outbound: stacked; receiving: put back)."""
        if self.swap_at[a] is not None and self.receiving and self.pallets[a] == 0:
            self.swap_at[a] = None
        self.pallets[a] += 1
        if not self.receiving and self.pallets[a] >= self.p.capacity:
            self.swap_at[a] = now + self.p.swap_s
            self.log.append((now, f'pallet {a} full, pickup in {self.p.swap_s:.0f} s'))

    def _complete(self, now):
        if not self.receiving:
            self._put(now, self.active)
        self.served, self.transfers = self.task, self.transfers + 1
        self.state, self.robot, self.task, self.active, self.slot = IDLE, '', '', None, None

    def _abort(self, now, why):
        if self.receiving:                                    # the box goes back on its pallet
            self._put(now, self.active)
        self.log.append((now, f'transfer for {self.task} ({self.robot}) interrupted: {why}'))
        self.state, self.robot, self.task, self.active, self.slot = IDLE, '', '', None, None


# ---------------------------------------------------------------------------------------------------------------------
# A station whose arm is a real cell (open_amr_arm_cell cell_controller): transfers end on the cell's events, not a timer.
PREPARE, DEPALLETIZE, PALLETIZE = 0, 1, 2                     # open_amr_msgs/ArmTransfer kinds
C_NONE, C_PALLET, C_TOOL, C_DECK, C_LOST = 0, 1, 2, 3, 4       # ArmTransfer.Result.CARTON_*: where the carton ended
JOB_NAMES = {PREPARE: 'prepare', DEPALLETIZE: 'depalletize', PALLETIZE: 'palletize'}


@dataclass
class Job:
    kind: int
    pallet: int
    slot: int
    robot: str = ''
    task: str = ''
    held: bool = False               # DEPALLETIZE of a pre-picked carton


class CellStation(Station):
    """Stock and handshake of a station run by a cell controller. step() returns the next Job to send; the agent
    reports how it went: deck_clear() (the arm left the deck zone: the robot is served and may leave), finished(ok,
    carton), cancelled().
    receiving  pre-picks the next carton while no robot is waiting (PREPARE: the carton leaves its pallet and waits on
               the tool over the deck zone), so a robot at the bay waits only for the hand-over (DEPALLETIZE).
    outbound   PALLETIZE when a robot is at the bay; the carton counts on its pallet when the cell reports it there,
               after the robot has already left.
    A failed job leaves its carton where the cell says: back on the pallet (counted), on the robot's deck (the robot
    stays and is served again after the reset), on the tool or lost (a person takes it away: counted in `lost`)."""

    def __init__(self, p, pallets):
        super().__init__(p, pallets)
        self.job = None                  # the Job the cell is running
        self.held = None                 # (pallet, slot) of a pre-picked carton on the tool
        self.deck_pending = None         # (robot, task): a carton reached its deck, but the arm faulted before clearing
        self.lost = 0

    def available(self):
        return super().available() + (1 if self.held else 0)

    def step(self, now, at_bay, fault):
        self._swaps(now)
        if fault:
            if self.state != FAULT:
                self.log.append((now, 'FAULT: out of service'))
            self.state = FAULT
            return None
        if self.state == FAULT:
            self.log.append((now, 'fault cleared, back in service'))
            self.state = IDLE
            if self.deck_pending:                             # its carton is on the deck: served now the arm is clear
                self.served, self.transfers, self.deck_pending = self.deck_pending[1], self.transfers + 1, None
        if self.job is not None:
            return None
        waiting = at_bay if at_bay is not None and at_bay[1] and at_bay[1] != self.served else None
        if waiting:
            if self.receiving and self.held:
                (a, k), self.held = self.held, None
                return self._busy(Job(DEPALLETIZE, a, k, *waiting, held=True))
            a = self._pick_pallet()
            if a is not None:
                if self.receiving:
                    return self._busy(Job(DEPALLETIZE, a, self._take(now, a), *waiting))
                return self._busy(Job(PALLETIZE, a, self.pallets[a], *waiting))
        elif self.receiving and self.held is None:
            a = self._pick_pallet()
            if a is not None:
                self.job = Job(PREPARE, a, self._take(now, a))
                self.state = IDLE
                return self.job
        self.state = IDLE if self.available() > 0 else STARVED
        return None

    def _busy(self, job):
        self.job, self.state, self.robot, self.task = job, BUSY, job.robot, job.task
        self.active, self.slot = job.pallet, job.slot
        return job

    def deck_clear(self, now):
        j = self.job
        if j is not None and j.kind != PREPARE and self.served != j.task:
            self.served, self.transfers = j.task, self.transfers + 1

    def finished(self, now, ok, carton, why=''):
        j, self.job = self.job, None
        if j is None:
            return
        if ok:
            if j.kind == PREPARE:
                self.held = (j.pallet, j.slot)
            elif j.kind == PALLETIZE:
                self._put(now, j.pallet)
        else:
            self.log.append((now, f'{JOB_NAMES[j.kind]} pallet {j.pallet} slot {j.slot}'
                                  f'{" for " + j.task if j.task else ""} failed: {why}'))
            if carton == C_PALLET or (carton == C_NONE and j.kind != PALLETIZE):
                self._put(now, j.pallet)                     # stacked (outbound) / never picked (receiving)
            elif carton == C_DECK and j.kind == DEPALLETIZE and self.served != j.task:
                self.deck_pending = (j.robot, j.task)
            elif carton in (C_TOOL, C_LOST):
                self.lost += 1
                self.log.append((now, f'carton of {JOB_NAMES[j.kind]} pallet {j.pallet} slot {j.slot} '
                                      f'{"on the tool" if carton == C_TOOL else "lost"}: a person removes it'))
            self.state = FAULT
        self.robot, self.task, self.active, self.slot = '', '', None, None
        if self.state == BUSY:
            self.state = IDLE

    def cancelled(self, now):
        """The cell dropped a job before entering the deck zone (its robot left): a pre-picked carton stays held."""
        j, self.job = self.job, None
        if j is not None and j.kind == DEPALLETIZE:
            self.held = (j.pallet, j.slot)
        self.robot, self.task, self.active, self.slot = '', '', None, None
        self.state = IDLE

    def tool_cleared(self, now):
        """After a reset the gripper is empty: a held carton was taken off by a person."""
        if self.held:
            self.lost += 1
            self.log.append((now, f'pre-picked carton (pallet {self.held[0]} slot {self.held[1]}) taken off the tool'))
            self.held = None
