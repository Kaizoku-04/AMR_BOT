"""V-marker detection (open_amr_docking/v_marker.py) on synthetic scans."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from open_amr_docking.v_marker import detect, template_scan  # noqa: E402

W, D = 0.30, 0.06


@pytest.mark.parametrize('dist,lat,yaw_deg', [(1.3, 0.0, 0.0), (1.2, 0.08, 5.0), (0.9, -0.06, -8.0),
                                              (0.32, 0.0, 0.0), (0.35, 0.01, 2.0)])
def test_pose_from_staging_to_docked(dist, lat, yaw_deg):
    true = (dist, lat, math.radians(yaw_deg))                 # marker frame in the lidar frame
    pts = template_scan(true, W, D)
    pred = (true[0] + 0.12, true[1] - 0.10, true[2] + math.radians(6))   # localization error
    pose, info = detect(pts, pred, W, D)
    assert pose is not None, info
    assert math.hypot(pose[0] - true[0], pose[1] - true[1]) < 0.001
    assert abs(math.degrees(math.remainder(pose[2] - true[2], 2 * math.pi))) < 0.1


def test_noisy_lidar_single_scans_and_their_average():
    """5 mm range noise (RPLidar class at 1 m): single scans are good to a few mm; the docking server averages the
    detections it gets (filter_coef), so the mean over 20 scans must be within 3 mm."""
    rng = np.random.default_rng(3)
    true = (1.0, 0.03, math.radians(3.0))
    found = []
    for k in range(20):
        pts = template_scan(true, W, D, noise=0.005, rng=rng)
        pose, info = detect(pts, (1.05, 0.0, 0.0), W, D, line_tol=0.0125, max_rms=0.008)
        if pose is not None:
            found.append(pose)
    assert len(found) >= 12                                   # >= 6 Hz of detections from a 10 Hz lidar
    errs = [math.hypot(x - true[0], y - true[1]) for x, y, _ in found]
    assert np.mean(errs) < 0.008
    mx, my = np.mean([f[0] for f in found]), np.mean([f[1] for f in found])
    assert math.hypot(mx - true[0], my - true[1]) < 0.003


def test_a_flat_wall_is_not_a_marker():
    pts = np.array([[1.0, y] for y in np.arange(-0.4, 0.4, 0.008)])
    pose, why = detect(pts, (1.0, 0.0, 0.0), W, D)
    assert pose is None
