"""Where a carton really is, relative to where the pallet pattern says it is (the cell controller's `box_locator`).

The taught pattern knows where a carton *should* be. Before the arm goes down to grip one, the cell asks a locator for
the carton's real position and corrects the grip by the offset — or stops with a fault if the carton is too far off to
be the one expected (a collapsed stack, a carton pushed across the deck), so a person can look.

  NominalLocator     trusts the pattern (offset 0): palletizing onto the cell's own stacks, or a cell with no camera.
  DetectionLocator   matches the carton against detections on a PoseArray topic (carton centres, world frame): in the
                     simulator the true carton poses (open_amr_sim.py publishes them), on a real cell the output of a
                     3D camera's carton detection (depalletizing mixed truck pallets needs one, Isaac Sim Setup
                     "Making the arms really move"). Only detections newer than the request count.
                     The timeout runs on the node's clock: sim time in the simulator (use_sim_time), wall time on a
                     real cell. In wall time, 1.0 s at real-time factor 0.22 was 0.22 s of sim time, about two of the
                     sim's 10 Hz detection periods: a fleet shift lost the receiving cell for 120 s to "no carton
                     detections for 1.0 s" (c60_real4, 2026-11-13).
  'deck_frame'       target ('deck_frame',): where the docked robot's deck really is, from a second PoseArray (deck-centre
                     poses, world frame: in the simulator open_amr_sim.py publishes /<arm>/deck/detected; on a real cell a
                     fiducial on the AMR deck seen by the cell camera). DEPALLETIZE places the carton there, not at the
                     nominal bay pose (ADR-048): the robot's docking offset (~10 mm, 1.4 deg) would otherwise become the
                     carton's offset against the deck rails. Beyond the limits the cell faults rather than places.
Offsets are (dx, dy, dyaw_deg) in the cell frame; a square carton's yaw is taken modulo 90 deg, the deck's modulo 180.
"""
import math
import threading
import time


WALL_LIMIT_S = 60.0


class LocateFault(Exception):
    pass


class NominalLocator:
    def locate(self, target, nominal, timeout=1.0):
        return (0.0, 0.0, 0.0)

    def locate_deck(self, nominal, timeout=1.0):
        return (0.0, 0.0, 0.0)

    def detected(self):
        return []


def match(detections, nominal, max_dist, max_dyaw, search=0.15, height_tol=0.08):
    """Offset (dx, dy, dyaw) of the detection nearest to `nominal` (x, y, z, yaw_deg; carton centre, same frame).
    Raises LocateFault if none is within `search` (and `height_tol` vertically), or if the nearest is further off
    than max_dist / max_dyaw."""
    nx, ny, nz, nyaw = nominal
    best = None
    for x, y, z, yaw in detections:
        d = math.hypot(x - nx, y - ny)
        if d <= search and abs(z - nz) <= height_tol and (best is None or d < best[0]):
            best = (d, x - nx, y - ny, (yaw - nyaw + 45.0) % 90.0 - 45.0)
    if best is None:
        raise LocateFault('no carton found')
    d, dx, dy, dyaw = best
    if d > max_dist or abs(dyaw) > max_dyaw:
        raise LocateFault(f'carton out of place by {d * 1000:.0f} mm / {dyaw:+.1f} deg '
                          f'(correctable: {max_dist * 1000:.0f} mm / {max_dyaw:.0f} deg)')
    return dx, dy, dyaw


def match_deck(decks, nominal, max_dist, max_dyaw, search=0.30):
    """Offset (dx, dy, dyaw) of the deck detection nearest to `nominal` (x, y, yaw_deg; deck centre, cell frame). Raises
    LocateFault if none is within `search`, or if the nearest is further off than max_dist / max_dyaw."""
    nx, ny, nyaw = nominal
    best = None
    for x, y, yaw in decks:
        d = math.hypot(x - nx, y - ny)
        if d <= search and (best is None or d < best[0]):
            best = (d, x - nx, y - ny, (yaw - nyaw + 90.0) % 180.0 - 90.0)
    if best is None:
        raise LocateFault('no deck found')
    d, dx, dy, dyaw = best
    if d > max_dist or abs(dyaw) > max_dyaw:
        raise LocateFault(f'deck out of place by {d * 1000:.0f} mm / {dyaw:+.1f} deg '
                          f'(correctable: {max_dist * 1000:.0f} mm / {max_dyaw:.0f} deg)')
    return dx, dy, dyaw


class DetectionLocator:
    """Detections from a geometry_msgs/PoseArray of carton centres in the world frame; `cell` (cell_geometry.Cell at
    its arm pose) converts them into the cell frame. Limits per target kind: {'pallet': (m, deg), 'deck': (m, deg),
    'deck_frame': (m, deg)}; deck_topic = the PoseArray of deck-centre poses (optional)."""

    def __init__(self, node, topic, cell, limits, deck_topic=''):
        from geometry_msgs.msg import PoseArray
        self.cell, self.limits, self.clock = cell, limits, node.get_clock()
        self.lock, self.cartons, self.stamp = threading.Lock(), [], 0.0
        self.decks, self.deck_stamp = [], 0.0
        node.create_subscription(PoseArray, topic, self.on_detections, 1)
        if deck_topic:
            node.create_subscription(PoseArray, deck_topic, self.on_decks, 1)

    def now(self):
        return self.clock.now().nanoseconds * 1e-9

    def on_detections(self, m):
        ax, ay, ayaw = self.cell.arm_pose
        c, s = math.cos(math.radians(ayaw)), math.sin(math.radians(ayaw))
        out = []
        for p in m.poses:
            dx, dy = p.position.x - ax, p.position.y - ay
            yaw = 2 * math.degrees(math.atan2(p.orientation.z, p.orientation.w))   # cartons stand upright
            out.append((c * dx + s * dy, -s * dx + c * dy, p.position.z, yaw - ayaw))
        with self.lock:
            self.cartons, self.stamp = out, self.now()

    def on_decks(self, m):
        ax, ay, ayaw = self.cell.arm_pose
        c, s = math.cos(math.radians(ayaw)), math.sin(math.radians(ayaw))
        out = []
        for p in m.poses:
            dx, dy = p.position.x - ax, p.position.y - ay
            yaw = 2 * math.degrees(math.atan2(p.orientation.z, p.orientation.w))
            out.append((c * dx + s * dy, -s * dx + c * dy, yaw - ayaw))
        with self.lock:
            self.decks, self.deck_stamp = out, self.now()

    def detected(self):
        """The latest carton detections (x, y, z, yaw_deg), cell frame."""
        with self.lock:
            return list(self.cartons)

    def locate(self, target, nominal, timeout=1.0):
        """target = ('pallet', i, k) | ('deck',); nominal = carton centre (x, y, z, yaw_deg), cell frame."""
        if target[0] == 'deck_frame':
            return self.locate_deck(nominal, timeout)
        cartons = self._wait('carton', 'cartons', 'stamp', timeout)
        max_dist, max_dyaw = self.limits[target[0]]
        return match(cartons, nominal, max_dist, max_dyaw)

    def locate_deck(self, nominal, timeout=1.0):
        """Offset (dx, dy, dyaw_deg) of the deck from `nominal` = its pose (x, y, yaw_deg), cell frame."""
        decks = self._wait('deck', 'decks', 'deck_stamp', timeout)
        max_dist, max_dyaw = self.limits['deck_frame']
        return match_deck(decks, nominal, max_dist, max_dyaw)

    def _wait(self, what, field, stamp, timeout):
        """The first detections of `field` newer than now (sim clock), or LocateFault after `timeout`."""
        t0, w0 = self.now(), time.monotonic()
        while True:
            with self.lock:
                if getattr(self, stamp) > t0:
                    return list(getattr(self, field))
            if self.now() - t0 > timeout:
                raise LocateFault(f'no {what} detections for {timeout:.1f} s')
            if time.monotonic() - w0 > WALL_LIMIT_S:    # the clock itself stopped (no /clock): don't wait forever
                raise LocateFault(f'no {what} detections and no clock for {WALL_LIMIT_S:.0f} s')
            time.sleep(0.01)
