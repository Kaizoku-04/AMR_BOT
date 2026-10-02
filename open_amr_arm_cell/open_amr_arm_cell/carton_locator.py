"""Where a carton really is, relative to where the pallet pattern says it is (the cell controller's `box_locator`).

The taught pattern knows where a carton *should* be. Before the arm goes down to grip one, the cell asks a locator for
the carton's real position and corrects the grip by the offset — or stops with a fault if the carton is too far off to
be the one expected (a collapsed stack, a carton pushed across the deck), so a person can look.

  NominalLocator     trusts the pattern (offset 0): palletizing onto the cell's own stacks, or a cell with no camera.
  DetectionLocator   matches the carton against detections on a PoseArray topic (carton centres, world frame): in the
                     simulator the true carton poses (open_amr_sim.py publishes them), on a real cell the output of a
                     3D camera's carton detection (depalletizing mixed truck pallets needs one, Isaac Sim Setup
                     "Making the arms really move"). Only detections newer than the request count.
Offsets are (dx, dy, dyaw_deg) in the cell frame; a square carton's yaw is taken modulo 90 deg.
"""
import math
import threading
import time


class LocateFault(Exception):
    pass


class NominalLocator:
    def locate(self, target, nominal, timeout=1.0):
        return (0.0, 0.0, 0.0)


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


class DetectionLocator:
    """Detections from a geometry_msgs/PoseArray of carton centres in the world frame; `cell` (cell_geometry.Cell at
    its arm pose) converts them into the cell frame. Limits per target kind: {'pallet': (m, deg), 'deck': (m, deg)}."""

    def __init__(self, node, topic, cell, limits):
        from geometry_msgs.msg import PoseArray
        self.cell, self.limits = cell, limits
        self.lock, self.cartons, self.stamp = threading.Lock(), [], 0.0
        node.create_subscription(PoseArray, topic, self.on_detections, 1)

    def on_detections(self, m):
        ax, ay, ayaw = self.cell.arm_pose
        c, s = math.cos(math.radians(ayaw)), math.sin(math.radians(ayaw))
        out = []
        for p in m.poses:
            dx, dy = p.position.x - ax, p.position.y - ay
            yaw = 2 * math.degrees(math.atan2(p.orientation.z, p.orientation.w))   # cartons stand upright
            out.append((c * dx + s * dy, -s * dx + c * dy, p.position.z, yaw - ayaw))
        with self.lock:
            self.cartons, self.stamp = out, time.monotonic()

    def locate(self, target, nominal, timeout=1.0):
        """target = ('pallet', i, k) | ('deck',); nominal = carton centre (x, y, z, yaw_deg), cell frame."""
        t0 = time.monotonic()
        while True:
            with self.lock:
                if self.stamp > t0:
                    cartons = list(self.cartons)
                    break
            if time.monotonic() - t0 > timeout:
                raise LocateFault(f'no carton detections for {timeout:.1f} s')
            time.sleep(0.01)
        max_dist, max_dyaw = self.limits[target[0]]
        return match(cartons, nominal, max_dist, max_dyaw)
