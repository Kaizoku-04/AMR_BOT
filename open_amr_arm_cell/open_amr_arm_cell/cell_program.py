"""The cell's motion program (pure Python, no ROS): one depalletize / palletize cycle as a list of steps. The reach study
plans and validates exactly these steps and writes the taught points; the cell executor runs exactly these steps. One
definition, so what was validated is what runs.

Deck zone interlock (cell_geometry.deck_zone): the arm keeps out of the volume over the bay unless a robot is docked
and stopped there. A cycle is split by two markers:
  DOCK     the arm waits here until the interlock is made (robot at the bay, AT_STATION, stopped); from here on the
           planning scene holds the docked robot instead of the keep-out volume
  UNDOCK   the arm is out of the zone again: the robot may leave (the station reports the task served); the keep-out
           volume is back in the scene for the rest of the cycle
  depalletize  home -PTP-> above slot -LIN-> slot (locate, grip) -LIN-> lift -PTP-> hold over the zone   [PREPARE ends]
               DOCK -LIN-> above deck -LIN(slow)-> deck (release) -LIN-> clear of the zone  UNDOCK -PTP-> home
  palletize    home -PTP-> wait over the zone  DOCK -LIN-> deck (locate, grip) -LIN-> hold height  UNDOCK
               -PTP-> above slot -LIN-> approach -LIN(slow)-> slot (release) -LIN-> above slot -PTP-> home
Poses are TCP poses in the cell frame, suction face down: (x, y, z, yaw_deg). Taught joint configurations (PTP goals)
come from the taught pattern: slot_above, deck_hold (depalletize: carton held over the zone; palletize: empty gripper
waiting over it); t['via'] / t['hold_high'] pick the heights (heights()).
"""
from dataclasses import dataclass

HOME = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
CONTACT = 0.005           # planned contact stops this short: the foam pad compresses 10-20 mm to seal, and a released
                          # carton drops this far (exact contact would read as a collision)

PTP, LIN, GRIP, RELEASE, DOCK, UNDOCK = 'ptp', 'lin', 'grip', 'release', 'dock', 'undock'


@dataclass
class Step:
    kind: str
    name: str
    q: list = None            # PTP goal (joint configuration)
    pose: tuple = None        # LIN goal (x, y, z, yaw_deg), cell frame
    place: bool = False       # LIN at the slow placing speed (the last centimetres onto a slot or the deck)
    locate: tuple = None      # GRIP target located before the step that goes down to it: ('pallet', i, k) | ('deck',)
    where: str = ''           # GRIP / RELEASE: where the carton is taken from / put down


def heights(cell, pallet, slot, via, hold_high=False):
    """(slot top z, deck top z, carry height at the slot, carry height over the deck) for one cycle. A via cycle
    carries the carton at the transit height over the pallets; over the deck it is held just above the zone, so the
    robot waits for the shortest descent — unless the swing down to that hold would sweep the carton through a full
    pallet (hold_high: held at the transit height; the reach study decides per slot)."""
    (_, _, sz), _ = cell.slot_cell(pallet, slot)
    (_, _, dz), _ = cell.deck_cell()
    zt = cell.transit_height()
    return sz, dz, (zt if via else cell.lift_height(sz)), (zt if via and hold_high else cell.deck_hold_height())


def taught_poses(cell, depal, pallet, slot, grip_yaw, deck_yaw, via, hold_high=False):
    """TCP poses of the two taught points: (slot_above, deck_hold)."""
    (sx, sy, sz), _ = cell.slot_cell(pallet, slot)
    (dx, dy, _), _ = cell.deck_cell()
    _, _, z_slot, z_deck = heights(cell, pallet, slot, via, hold_high)
    a = cell.p.approach
    slot_above = (sx, sy, sz + a if depal else z_slot, grip_yaw)
    deck_hold = (dx, dy, z_deck if depal else cell.deck_clear_height(), deck_yaw)
    return slot_above, deck_hold


def shifted(pose, offset):
    """Pose moved by a located offset (dx, dy, dyaw_deg) in the cell frame."""
    if offset is None:
        return pose
    x, y, z, yaw = pose
    return (x + offset[0], y + offset[1], z, yaw + offset[2])


def depalletize(cell, pallet, slot, t, offset=None):
    """Steps of one depalletizing cycle. t = taught entry (slot_above, deck_hold, grip_yaw, deck_yaw, via);
    offset = where the located carton really is relative to its slot (dx, dy, dyaw), None = nominal."""
    (sx, sy, sz), _ = cell.slot_cell(pallet, slot)
    (dx, dy, dz), _ = cell.deck_cell()
    _, _, z_slot, _ = heights(cell, pallet, slot, t['via'], t.get('hold_high', False))
    gy, dyaw, a = t['grip_yaw'], t['deck_yaw'], cell.p.approach
    return [
        Step(PTP, 'to slot', q=t['slot_above']),
        Step(LIN, 'down to slot', pose=shifted((sx, sy, sz + CONTACT, gy), offset), locate=('pallet', pallet, slot)),
        Step(GRIP, 'grip', where=f'pallet {pallet} slot {slot}'),
        Step(LIN, 'lift', pose=shifted((sx, sy, z_slot, gy), offset)),
        Step(PTP, 'to deck hold', q=t['deck_hold']),
        Step(DOCK, 'dock'),
        Step(LIN, 'down to deck', pose=(dx, dy, dz + a, dyaw)),
        Step(LIN, 'place on deck', pose=(dx, dy, dz + CONTACT, dyaw), place=True),
        Step(RELEASE, 'release', where='deck'),
        Step(LIN, 'clear deck', pose=(dx, dy, cell.deck_clear_height(), dyaw)),
        Step(UNDOCK, 'undock'),
        Step(PTP, 'home', q=HOME),
    ]


def palletize(cell, pallet, slot, t, offset=None):
    """Steps of one palletizing cycle; offset = where the located carton on the deck really is (dx, dy, dyaw)."""
    (sx, sy, sz), _ = cell.slot_cell(pallet, slot)
    (dx, dy, dz), _ = cell.deck_cell()
    _, _, _, z_deck = heights(cell, pallet, slot, t['via'], t.get('hold_high', False))
    gy, dyaw, a = t['grip_yaw'], t['deck_yaw'], cell.p.approach
    return [
        Step(PTP, 'to deck wait', q=t['deck_hold']),
        Step(DOCK, 'dock'),
        Step(LIN, 'down to deck', pose=shifted((dx, dy, dz + CONTACT, dyaw), offset), locate=('deck',)),
        Step(GRIP, 'grip', where='deck'),
        Step(LIN, 'up from deck', pose=shifted((dx, dy, z_deck, dyaw), offset)),
        Step(UNDOCK, 'undock'),
        Step(PTP, 'to slot', q=t['slot_above']),
        Step(LIN, 'down to slot', pose=(sx, sy, sz + a, gy)),
        Step(LIN, 'place in slot', pose=(sx, sy, sz + CONTACT, gy), place=True),
        Step(RELEASE, 'release', where=f'pallet {pallet} slot {slot}'),
        Step(LIN, 'retreat', pose=(sx, sy, sz + a, gy)),
        Step(PTP, 'home', q=HOME),
    ]


def program(cell, depal, pallet, slot, t, offset=None):
    return (depalletize if depal else palletize)(cell, pallet, slot, t, offset)
