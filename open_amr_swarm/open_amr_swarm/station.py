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
        if not self._started:
            self._started = True
            self.swap_at = [now + self.p.swap_s if self._spent(n) else None for n in self.pallets]
        for i, t in enumerate(self.swap_at):
            if t is not None and now >= t:
                self.pallets[i] = self.p.capacity if self.receiving else 0
                self.swap_at[i] = None
                self.log.append((now, f'pallet {i} {"restocked (full)" if self.receiving else "shipped, empty pallet staged"}'))
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
                    self.pallets[a] -= 1
                    slot = self.pallets[a]                     # the top box
                    if self.pallets[a] == 0:
                        self.swap_at[a] = now + self.p.swap_s
                        self.log.append((now, f'pallet {a} empty, next one due in {self.p.swap_s:.0f} s'))
                else:
                    slot = self.pallets[a]                     # next free slot
                self.state, (self.robot, self.task) = BUSY, at_bay
                self.active, self.slot, self.done_at = a, slot, now + self.p.cycle_s
                return Started(self.robot, self.task, a, slot)
        self.state = IDLE if self.available() > 0 else STARVED
        return None

    def _complete(self, now):
        if not self.receiving:
            a = self.active
            self.pallets[a] += 1
            if self.pallets[a] >= self.p.capacity:
                self.swap_at[a] = now + self.p.swap_s
                self.log.append((now, f'pallet {a} full, pickup in {self.p.swap_s:.0f} s'))
        self.served, self.transfers = self.task, self.transfers + 1
        self.state, self.robot, self.task, self.active, self.slot = IDLE, '', '', None, None

    def _abort(self, now, why):
        if self.receiving:                                    # the box goes back on its pallet
            a = self.active
            if self.swap_at[a] is not None and self.pallets[a] == 0:
                self.swap_at[a] = None
            self.pallets[a] += 1
        self.log.append((now, f'transfer for {self.task} ({self.robot}) interrupted: {why}'))
        self.state, self.robot, self.task, self.active, self.slot = IDLE, '', '', None, None
