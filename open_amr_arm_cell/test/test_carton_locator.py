"""Unit tests for the carton locator's matching (open_amr_arm_cell/carton_locator.py)."""
import pytest

from open_amr_arm_cell.carton_locator import LocateFault, match


def test_nearest_detection_gives_the_offset():
    nominal = (1.0, 2.0, 0.5, 30.0)
    dets = [(1.36, 2.0, 0.5, 30.0), (1.02, 1.99, 0.5, 31.5), (1.0, 2.0, 0.25, 30.0)]   # neighbour, ours, one below
    dx, dy, dyaw = match(dets, nominal, 0.03, 5.0)
    assert dx == pytest.approx(0.02) and dy == pytest.approx(-0.01) and dyaw == pytest.approx(1.5)


def test_square_carton_yaw_is_modulo_90():
    assert match([(1.0, 2.0, 0.5, 30.0 + 90.0 - 2.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)[2] == pytest.approx(-2.0)


def test_out_of_place_and_missing_cartons_fault():
    with pytest.raises(LocateFault, match='out of place'):
        match([(1.06, 2.0, 0.5, 30.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)
    with pytest.raises(LocateFault, match='out of place'):
        match([(1.0, 2.0, 0.5, 38.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)
    with pytest.raises(LocateFault, match='no carton'):
        match([(1.0, 2.0, 0.25, 30.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)     # only the layer below


class _Clock:
    """A sim clock the test advances (rclpy Clock.now().nanoseconds)."""

    def __init__(self):
        self.t = 100.0

    def now(self):
        return type('T', (), {'nanoseconds': int(self.t * 1e9)})()


class _Node:
    def __init__(self):
        self.clock = _Clock()

    def get_clock(self):
        return self.clock

    def create_subscription(self, *a):
        pass


def _locator():
    from open_amr_arm_cell.carton_locator import DetectionLocator
    cell = type('C', (), {'arm_pose': (0.0, 0.0, 0.0)})()
    node = _Node()
    return node, DetectionLocator(node, '/arm/cartons/detected', cell, {'pallet': (0.03, 5.0)})


def _detections(*xyz):
    from geometry_msgs.msg import Pose, PoseArray
    m = PoseArray()
    for x, y, z in xyz:
        p = Pose()
        p.position.x, p.position.y, p.position.z, p.orientation.w = x, y, z, 1.0
        m.poses.append(p)
    return m


def test_timeout_is_sim_time_not_wall_time():
    """c60_real4: at real-time factor 0.22 the next 10 Hz detection came > 1.0 s of wall time after the request, but
    within 0.3 s of sim time: 'no carton detections for 1.0 s' faulted the receiving cell (fails with time.monotonic)."""
    import threading
    import time
    node, loc = _locator()
    out = {}
    th = threading.Thread(target=lambda: out.setdefault('r', _try(loc)))
    th.start()
    for _ in range(12):                      # 1.2 s of wall time, 0.24 s of sim time (rtf 0.2)
        time.sleep(0.1)
        node.clock.t += 0.02
    loc.on_detections(_detections((1.0, 2.0, 0.5)))
    th.join(timeout=5.0)
    assert out.get('r') == pytest.approx((0.0, 0.0, 0.0)), out


def test_no_detection_for_the_timeout_in_sim_time_still_faults():
    import threading
    import time
    node, loc = _locator()
    out = {}
    th = threading.Thread(target=lambda: out.setdefault('r', _try(loc)))
    th.start()
    for _ in range(6):
        time.sleep(0.05)
        node.clock.t += 0.25                  # 1.5 s of sim time, no detection
    th.join(timeout=5.0)
    assert isinstance(out.get('r'), LocateFault) and 'no carton detections' in str(out['r'])


def _try(loc):
    try:
        return loc.locate(('pallet', 0, 0), (1.0, 2.0, 0.5, 0.0))
    except LocateFault as e:
        return e
