"""Localization watch: tell a lost AMCL pose from AMCL's ordinary corrections (2026-11-21, stall reproducer B5r3).

AMCL publishes map -> odom; odometry (odom -> base_footprint) is continuous. Where the robot would be if an older
map -> odom still held, minus where it is believed now, is how far AMCL has moved the robot's estimate since then
without the robot moving there. Ordinary corrections stay at centimetres (fleet shifts: AMCL error 0.11 m mean,
0.27 m max). In B5r3 the robot backed out of a stall pinned on a rack upright, pivoting 85 deg, and AMCL walked
the estimate 4 m away in 20 s with nothing to bring it back (recovery_alpha 0): every path the agent sent was then
rejected as 'Resulting plan has 0 poses in it', recoveries ran blind on the wrong pose, and the robot stood the
shift out. A correction beyond `jump_m` within `window_s` is not something a robot may drive on.

Poses are (x, y, yaw) in 2D.
"""
import math
from collections import deque


def compose(a, b):
    """Pose b given in frame a, expressed in a's parent frame (a o b)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


class LocWatch:
    """Largest map -> odom correction at the robot's position over the last `window_s` seconds."""

    def __init__(self, window_s=20.0, jump_m=1.0):
        self.window_s, self.jump_m = window_s, jump_m
        self.hist = deque()                  # (t, map->odom)

    def reset(self):
        self.hist.clear()

    def update(self, t, map_odom, odom_base):
        """Add a sample; return the largest correction (m) since the oldest sample still inside the window."""
        while self.hist and t - self.hist[0][0] > self.window_s:
            self.hist.popleft()
        now = compose(map_odom, odom_base)
        worst = 0.0
        for _, mo in self.hist:
            then = compose(mo, odom_base)
            worst = max(worst, math.hypot(then[0] - now[0], then[1] - now[1]))
        self.hist.append((t, map_odom))
        return worst

    def lost(self, correction):
        return self.jump_m > 0.0 and correction > self.jump_m
