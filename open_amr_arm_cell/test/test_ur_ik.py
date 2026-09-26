"""Closed-form UR10e IK: every solution reproduces the pose; the solver finds the configuration it came from."""
import math
import random

import numpy as np

from open_amr_arm_cell.ur_ik import closest, fk, ik


def test_zero_pose_matches_ur_reference():
    # UR10e at all-zero joints: flange at (-(a2+a3)... in DH base), i.e. (1.18425, 0.2907, 0.06085) in base_link
    assert np.allclose(fk([0.0] * 6)[:3, 3], [1.18425, 0.2907, 0.06085], atol=1e-6)


def test_all_solutions_reproduce_the_pose_and_include_the_source():
    rng = random.Random(1)
    for _ in range(300):
        q = [rng.uniform(-math.pi, math.pi) for _ in range(6)]
        if abs(math.sin(q[4])) < 0.05:
            continue
        T = fk(q)
        sols = ik(T)
        assert sols, q
        for s in sols:
            assert np.allclose(fk(s), T, atol=1e-6)
        assert min(max(abs(math.atan2(math.sin(a - b), math.cos(a - b))) for a, b in zip(s, q)) for s in sols) < 1e-6


def test_closest_respects_limits():
    q = [0.3, -1.2, 1.5, -1.9, -1.57, 2.9]
    s = closest(ik(fk(q)), [0.3, -1.2, 1.5, -1.9, -1.57, 3.5])
    assert abs(s[5] - 2.9) < 1e-6
    lim = [(-2 * math.pi, 2 * math.pi)] * 2 + [(-math.pi, math.pi)] + [(-2 * math.pi, 2 * math.pi)] * 3
    s = closest(ik(fk(q)), q, lim)
    assert max(abs(a - b) for a, b in zip(s, q)) < 1e-6
