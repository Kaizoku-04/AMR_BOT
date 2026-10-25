"""bay_marker_detector's odometry history: the pose at a scan's time from the odometry stream.

Chain 49 (2026-10-25, real-input fleet shift): 601 of 4578 consecutive `odometry/filtered` messages carried stamps
< 1 us apart (two EKF outputs on one sim-clock tick). Extrapolating 33 ms past such a pair divided by ~0 and put a
0.002 mm step at 80 m / 187 deg, so tracking predicted the marker kilometres away and docking failed (error 904)."""
import math
import os
import sys
from collections import deque
from types import SimpleNamespace

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from open_amr_docking.bay_marker_detector import BayMarkerDetector  # noqa: E402


def odom_msg(t, x, yaw=0.0):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=int(t), nanosec=round((t % 1) * 1e9))),
                           pose=SimpleNamespace(pose=SimpleNamespace(
                               position=SimpleNamespace(x=x, y=0.0),
                               orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)))))


def detector():
    d = SimpleNamespace(odom=deque(maxlen=300), get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: None))
    d.on_odom = lambda m: BayMarkerDetector.on_odom(d, m)
    d.odom_at = lambda t: BayMarkerDetector.odom_at(d, t)
    return d


def test_extrapolation_past_a_repeated_stamp_stays_bounded():
    d = detector()
    v = 0.1                                                   # m/s, a docking approach
    for k in range(10):
        d.on_odom(odom_msg(100.0 + 0.033 * k, v * 0.033 * k, 0.001 * k))
    t = 100.0 + 0.033 * 9
    d.on_odom(odom_msg(t + 1e-7, v * 0.033 * 9 + 2e-6, 0.009 + 1e-7))   # the EKF's second output on the same tick
    x, y, yaw = d.odom_at(t + 0.017)                           # a scan 17 ms later
    assert abs(x - v * (0.033 * 9 + 0.017)) < 0.005           # where the robot is (old code: tens of metres)
    assert abs(math.remainder(yaw - 0.009, math.tau)) < math.radians(1.0)


def test_interpolation_between_regular_samples_unchanged():
    d = detector()
    for k in range(5):
        d.on_odom(odom_msg(10.0 + 0.02 * k, 0.002 * k))
    x, _, _ = d.odom_at(10.05)
    assert abs(x - 0.005) < 1e-9
