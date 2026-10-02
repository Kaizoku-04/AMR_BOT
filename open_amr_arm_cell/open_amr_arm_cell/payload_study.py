"""Payload study of the cell (deployment check): the force the vacuum pad must hold on the carton through every carrying
move of the taught pattern, against the gripper's datasheet. Each cycle is planned exactly as the cell runs it (the
reach study's planner and program, production speeds); every carrying trajectory is sampled at 1 ms along the
controller's own interpolation (quintic per segment from positions / velocities / accelerations) and the carton's
centre of mass is pushed through forward kinematics:
    F = m (a - g)          force the pad applies to the carton (world frame)
    pull  = F . n          along the pad normal n (out of the pad, into the carton): what the vacuum must hold
    shear = |F - (F.n) n|  in the pad's plane: what friction under the foam must hold
Reported per mode and worst over the pattern, with the margin to the ratings (`--pull-rating`, `--shear-rating`;
defaults: Schmalz ROB-SET FXCB UR O20, open foam: lift capacity 350 N 'horizontal' = pad horizontal / load
perpendicular, 80 N 'vertical' = load along the pad, Schmalz datasheet 'Handling Sets FXCB').

    ros2 run open_amr_arm_cell payload_study [--slots 0,11,23] [--mass 8.0] [--out payload.yaml]
"""
import argparse
import os
import sys

import numpy as np
import rclpy
import yaml

from . import ur_ik
from .cell_geometry import Cell, CellParams
from .cell_motion import load_pattern
from .reach_study import JOINTS, Study, _exit

G = np.array([0.0, 0.0, -9.81])


def quintic(p0, v0, a0, p1, v1, a1, T, t):
    """Position at t of the quintic through (p0, v0, a0) at 0 and (p1, v1, a1) at T (per joint, vectorised)."""
    T2, T3, T4, T5 = T * T, T ** 3, T ** 4, T ** 5
    c0, c1, c2 = p0, v0, a0 / 2
    c3 = (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3)
    c4 = (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 * T4)
    c5 = (12 * (p1 - p0) - (6 * v1 + 6 * v0) * T - (a0 - a1) * T2) / (2 * T5)
    return c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5


def carton_states(jt, p: CellParams, dt=0.001):
    """(times, carton CoG positions [N,3], pad normals [N,3]) along a joint trajectory."""
    idx = [jt.joint_names.index(j) for j in JOINTS]
    pts = jt.points
    tt = [pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 for pt in pts]
    arr = lambda pt, f: np.array([getattr(pt, f)[i] if len(getattr(pt, f)) else 0.0 for i in idx])
    ur = ur_ik.params(p.ur_type)
    ts, cog, nrm = [], [], []
    for a, b, ta, tb in zip(pts, pts[1:], tt, tt[1:]):
        T = tb - ta
        if T <= 0:
            continue
        for t in np.arange(0.0, T, dt):
            q = quintic(arr(a, 'positions'), arr(a, 'velocities'), arr(a, 'accelerations'),
                        arr(b, 'positions'), arr(b, 'velocities'), arr(b, 'accelerations'), T, t)
            M = ur_ik.fk(q, ur)
            n = M[:3, 2]                                    # tool z = out of the pad, into the carton
            tcp = M[:3, 3] + n * p.tool_length + np.array([0.0, 0.0, p.pedestal_height])
            ts.append(ta + t)
            cog.append(tcp + n * p.box[2] / 2)
            nrm.append(n)
    return np.array(ts), np.array(cog), np.array(nrm)


def loads(jt, p, mass):
    t, x, n = carton_states(jt, p)
    if len(t) < 5:
        return 0.0, 0.0, 0.0
    dt = np.diff(t).mean()
    a = np.zeros_like(x)
    a[1:-1] = (x[2:] - 2 * x[1:-1] + x[:-2]) / dt ** 2
    F = mass * (a - G)                                      # force the pad applies to the carton
    pull = np.einsum('ij,ij->i', F, n)                      # positive = pad pulling the carton towards itself...
    shear = np.linalg.norm(F - pull[:, None] * n, axis=1)
    acc = np.linalg.norm(a, axis=1)
    return float(np.max(np.abs(pull[1:-1]))), float(np.max(shear[1:-1])), float(np.max(acc[1:-1]))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--layout', default=os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')),
                                                    'sim', 'configs', 'warehouse_layout.yaml'))
    ap.add_argument('--slots', default='', help='default: all')
    ap.add_argument('--mass', type=float, default=8.0)
    ap.add_argument('--pull-rating', type=float, default=350.0)
    ap.add_argument('--shear-rating', type=float, default=80.0)
    ap.add_argument('--out', default='')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    p = CellParams.from_layout(yaml.safe_load(open(a.layout)))
    cell = Cell((0.0, 0.0, 0.0), p)
    pattern = load_pattern()['pattern']
    st = Study(cell)
    slots = [int(s) for s in a.slots.split(',')] if a.slots else list(range(p.capacity))
    rows = []
    for depal in (True, False):
        mode = 'depalletize' if depal else 'palletize'
        for pallet in (0, 1):
            for k in slots:
                e = pattern.get(f'{mode}/{pallet}/{k}')
                if e is None:
                    continue
                st.set_scene(pallet, k + 1 if depal else k)
                st.record = []
                r = st.sequence(pallet, k, e['taught']['grip_yaw'], e['taught']['deck_yaw'], depal, e['via'],
                                e['taught'].get('hold_high', False))
                rec, st.record = st.record, None
                if isinstance(r, str):
                    print(f'{mode}/{pallet}/{k}: plan failed at {r}', flush=True)
                    continue
                worst = [0.0, 0.0, 0.0]
                for carrying, jt in rec:
                    if carrying:
                        for i, v in enumerate(loads(jt, p, a.mass)):
                            worst[i] = max(worst[i], v)
                rows.append(dict(cycle=f'{mode}/{pallet}/{k}', pull_N=round(worst[0], 1), shear_N=round(worst[1], 1),
                                 acc_ms2=round(worst[2], 2)))
                print(f'{mode}/{pallet}/{k}: pad pull {worst[0]:6.1f} N, shear {worst[1]:6.1f} N, carton accel '
                      f'{worst[2]:5.2f} m/s^2', flush=True)
    pull = max(r['pull_N'] for r in rows)
    shear = max(r['shear_N'] for r in rows)
    summary = dict(cycles=len(rows), mass_kg=a.mass, pull_max_N=pull, shear_max_N=shear,
                   acc_max_ms2=max(r['acc_ms2'] for r in rows),
                   pull_margin=round(a.pull_rating / pull, 2), shear_margin=round(a.shear_rating / shear, 2),
                   ratings=dict(pull_N=a.pull_rating, shear_N=a.shear_rating))
    print(f'payload study: {summary}', flush=True)
    if a.out:
        yaml.safe_dump(dict(summary=summary, cycles=rows), open(a.out, 'w'), sort_keys=False)
    return _exit(0)


if __name__ == '__main__':
    sys.exit(main())
