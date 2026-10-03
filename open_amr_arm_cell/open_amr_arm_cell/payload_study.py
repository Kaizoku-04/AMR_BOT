"""Payload study of the cell (deployment check): the force the vacuum pad must hold on the carton through every carrying
move of the taught pattern, against the gripper's datasheet. Each cycle is planned exactly as the cell runs it (the
reach study's planner and program, production speeds); along every carrying trajectory the carton's centre-of-mass
acceleration comes from the planned joint velocities / accelerations (a = J qdd + dJ/dt qd):
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


def carton_point(q, p: CellParams, ur):
    """Carton centre of mass (cell frame) and the pad normal for joint configuration q."""
    M = ur_ik.fk(q, ur)
    n = M[:3, 2]                                            # tool z = out of the pad, into the carton
    tcp = M[:3, 3] + n * p.tool_length + np.array([0.0, 0.0, p.pedestal_height])
    return tcp + n * p.box[2] / 2, n


def carton_accel(q, qd, qdd, p, ur, eps=1e-6, h=1e-4):
    """Carton CoG acceleration from the joint state: a = J qdd + (dJ/dt) qd, Jacobians by finite differences (no
    second differences of positions: sampled trajectories made those blow up)."""
    def J(qq):
        x0, _ = carton_point(qq, p, ur)
        cols = []
        for i in range(6):
            dq = np.array(qq, float)
            dq[i] += eps
            cols.append((carton_point(dq, p, ur)[0] - x0) / eps)
        return np.column_stack(cols)
    J0 = J(q)
    Jh = J(np.asarray(q) + np.asarray(qd) * h)
    return J0 @ qdd + ((Jh - J0) / h) @ qd


def loads(jt, p, mass):
    """Worst pad pull / shear (N) and carton acceleration (m/s^2) over the trajectory's points (Pilz trapezoidal
    profiles: the joint accelerations are piecewise constant between points, so the points carry the extremes)."""
    ur = ur_ik.params(p.ur_type)
    idx = [jt.joint_names.index(j) for j in JOINTS]
    worst = [0.0, 0.0, 0.0]
    for pt in jt.points:
        if not (len(pt.velocities) and len(pt.accelerations)):
            continue
        q = np.array([pt.positions[i] for i in idx])
        qd = np.array([pt.velocities[i] for i in idx])
        qdd = np.array([pt.accelerations[i] for i in idx])
        a = carton_accel(q, qd, qdd, p, ur)
        _, n = carton_point(q, p, ur)
        F = mass * (a - G)                                  # force the pad applies to the carton
        pull = float(F @ n)
        shear = float(np.linalg.norm(F - pull * n))
        worst = [max(worst[0], abs(pull)), max(worst[1], shear), max(worst[2], float(np.linalg.norm(a)))]
    return tuple(worst)


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
