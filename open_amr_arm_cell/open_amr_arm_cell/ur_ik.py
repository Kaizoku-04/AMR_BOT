"""Closed-form inverse kinematics of Universal Robots e-series arms (all 8 solutions), deterministic.

MoveIt's KDL solver restarts from random seeds, so the same pose can come back as different arm configurations from
run to run — a taught pallet pattern must not. Taught points are computed here and chosen by a fixed rule (closest
to a reference configuration); MoveIt/Pilz then only plans between them.

Frames as in ur_description: base_link (REP-103) = UR's DH `base` rotated by pi about z; tool0 = DH flange frame.
Standard UR solution (Hawkins, "Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms", 2013).
"""
import math

import numpy as np

UR10E = dict(d1=0.1807, a2=-0.6127, a3=-0.57155, d4=0.17415, d5=0.11985, d6=0.11655)
BASE_LINK_TO_BASE = np.diag([-1.0, -1.0, 1.0, 1.0])            # Rz(pi)


def _dh(p):
    d = [p['d1'], 0.0, 0.0, p['d4'], p['d5'], p['d6']]
    a = [0.0, p['a2'], p['a3'], 0.0, 0.0, 0.0]
    al = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]
    return d, a, al


def fk(q, p=UR10E):
    """tool0 in base_link (4x4)."""
    d, a, al = _dh(p)
    T = BASE_LINK_TO_BASE.copy()
    for i in range(6):
        c, s, ca, sa = math.cos(q[i]), math.sin(q[i]), math.cos(al[i]), math.sin(al[i])
        T = T @ np.array([[c, -s * ca, s * sa, a[i] * c], [s, c * ca, -c * sa, a[i] * s],
                          [0.0, sa, ca, d[i]], [0.0, 0.0, 0.0, 1.0]])
    return T


def ik(T_base_link_tool0, p=UR10E, wrist_eps=1e-6):
    """All joint solutions (list of 6-lists, angles in (-pi, pi]) that put tool0 at the given pose."""
    a2, a3, d4, d6 = p['a2'], p['a3'], p['d4'], p['d6']
    T = BASE_LINK_TO_BASE @ np.asarray(T_base_link_tool0, dtype=float)          # -> DH base frame
    out = []
    p05 = T @ np.array([0.0, 0.0, -d6, 1.0])
    r = math.hypot(p05[0], p05[1])
    if r < abs(d4):
        return out
    psi, phi = math.atan2(p05[1], p05[0]), math.acos(d4 / r)
    for t1 in (psi + phi + math.pi / 2, psi - phi + math.pi / 2):
        s1, c1 = math.sin(t1), math.cos(t1)
        c5 = (T[0, 3] * s1 - T[1, 3] * c1 - d4) / d6
        if abs(c5) > 1.0 + 1e-9:
            continue
        a5 = math.acos(max(-1.0, min(1.0, c5)))
        for t5 in (a5, -a5):
            s5 = math.sin(t5)
            if abs(s5) < wrist_eps:
                continue                                                   # wrist singularity: t6 undefined
            t6 = math.atan2((-T[0, 1] * s1 + T[1, 1] * c1) / s5, (T[0, 0] * s1 - T[1, 0] * c1) / s5)
            # frame 1 -> frame 4 origin, planar 2R for t2, t3
            T01 = _a(0, t1, p)
            T45 = _a(4, t5, p)
            T56 = _a(5, t6, p)
            T14 = np.linalg.inv(T01) @ T @ np.linalg.inv(T45 @ T56)
            px, py = T14[0, 3], T14[1, 3]
            c3 = (px * px + py * py - a2 * a2 - a3 * a3) / (2 * a2 * a3)
            if abs(c3) > 1.0 + 1e-9:
                continue
            a3a = math.acos(max(-1.0, min(1.0, c3)))
            for t3 in (a3a, -a3a):
                t2 = math.atan2(py, px) - math.atan2(a3 * math.sin(t3), a2 + a3 * math.cos(t3))
                T13 = _a(1, t2, p) @ _a(2, t3, p)
                T34 = np.linalg.inv(T13) @ T14
                t4 = math.atan2(T34[1, 0], T34[0, 0])
                out.append([_wrap(v) for v in (t1, t2, t3, t4, t5, t6)])
    return out


def _a(i, th, p):
    d, a, al = _dh(p)
    c, s, ca, sa = math.cos(th), math.sin(th), math.cos(al[i]), math.sin(al[i])
    return np.array([[c, -s * ca, s * sa, a[i] * c], [s, c * ca, -c * sa, a[i] * s],
                     [0.0, sa, ca, d[i]], [0.0, 0.0, 0.0, 1.0]])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def closest(solutions, ref, limits=None):
    """The solution nearest to `ref` (each joint may be shifted by 2 pi within `limits` [(lo, hi)] to get closer)."""
    best, best_d = None, math.inf
    for q in solutions:
        q2 = []
        for j, (v, r0) in enumerate(zip(q, ref)):
            cands = [v + k * 2 * math.pi for k in (-1, 0, 1)]
            if limits is not None:
                lo, hi = limits[j]
                cands = [c for c in cands if lo <= c <= hi]
            if not cands:
                break
            q2.append(min(cands, key=lambda c: abs(c - r0)))
        else:
            dist = sum((a - b) ** 2 for a, b in zip(q2, ref))
            if dist < best_d:
                best, best_d = q2, dist
    return best
