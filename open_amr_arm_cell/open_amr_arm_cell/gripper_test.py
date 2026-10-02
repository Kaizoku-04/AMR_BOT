"""Gripper test of a cell (Phase 4b gate): every carton of a full pallet goes pallet 0 -> AMR deck -> pallet 1 with the
vacuum over UR I/O, at the cell's production speeds (PTP 100 %, LIN at the approach tool speed; moveit_cpp.yaml) —
all 48 slots, both directions. Pass: no cell fault (no part lost, none
missed) and every carton ends in its slot on pallet 1 (true poses from the simulator, /<arm_id>/test_boxes) within
15 mm and 3 deg.

    ros2 run open_amr_arm_cell gripper_test --arm-id arm_receiving [--cycles 24] [--speed 1.0] [--out report.yaml]
Needs open_amr_sim.py --arms --arm-test and the cell's control stack (arm_cell.launch.py hardware:=topic).
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
import rclpy.qos
import yaml
from geometry_msgs.msg import PoseArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from .cell_geometry import Cell, CellParams
from .cell_motion import CellFault, CellMotion, VacuumIO, load_pattern
from .reach_study import _exit, moveit_configs, moveit_py

POS_TOL_M, YAW_TOL_DEG = 0.015, 3.0


class Watch(Node):
    def __init__(self, arm_id):
        super().__init__('gripper_test_io', namespace=arm_id,
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.lock, self.boxes = threading.Lock(), None
        self.create_subscription(PoseArray, f'/{arm_id}/test_boxes', self.on_boxes, 1)

    def on_boxes(self, m):
        with self.lock:
            self.boxes = [(p.position.x, p.position.y, p.position.z,
                           2 * math.degrees(math.atan2(p.orientation.z, p.orientation.w))) for p in m.poses]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--arm-id', default='arm_receiving')
    ap.add_argument('--cycles', type=int, default=24)
    ap.add_argument('--speed', type=float, default=0.0,
                    help='override velocity and acceleration scaling of every move (0 = the production speeds)')
    ap.add_argument('--layout', default=os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')),
                                                    'sim', 'configs', 'warehouse_layout.yaml'))
    ap.add_argument('--out', default='')
    ap.add_argument('--diag', action='store_true', help='snapshot each carton just before release (slower)')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    layout = yaml.safe_load(open(a.layout))
    p = CellParams.from_layout(layout)
    arm_pose = next(layout[d]['arm']['pose'] for d in ('receiving', 'outbound') if layout[d]['arm']['id'] == a.arm_id)
    world_cell = Cell(tuple(arm_pose), p)          # where the cartons really are
    cell = Cell((0.0, 0.0, 0.0), p)               # the planner's frame
    cfg = moveit_configs(p, hardware='topic', execution=True).to_dict()
    cfg['use_sim_time'] = True
    moveit = moveit_py('gripper_test', cfg, name_space=a.arm_id)
    watch = Watch(a.arm_id)
    io = VacuumIO(watch, a.arm_id)
    from .tracking_test import Recorder
    rec = Recorder(a.arm_id)                      # command / state streams, dumped on a fault
    ex = SingleThreadedExecutor()
    ex.add_node(watch)
    ex.add_node(rec)
    threading.Thread(target=ex.spin, daemon=True).start()
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    fjt = ActionClient(watch, FollowJointTrajectory, f'/{a.arm_id}/joint_trajectory_controller/follow_joint_trajectory')
    if not (fjt.wait_for_server(timeout_sec=60.0) and io.ready(60.0)):
        print('trajectory controller or set_io service not available', flush=True)
        return _exit(2)
    t0 = time.time()
    while watch.boxes is None and time.time() - t0 < 30:
        time.sleep(0.1)
    if watch.boxes is None:
        print(f'no /{a.arm_id}/test_boxes: run the simulator with --arms --arm-test', flush=True)
        return _exit(2)
    time.sleep(2.0)

    motion = CellMotion(moveit, io, cell, load_pattern(), vel_scale=a.speed or None, acc_scale=a.speed or None)
    n = p.capacity
    motion.set_scene(stock=[n, 0], docked=False, deck_box=False)   # a stand-in AMR is always docked: no interlock
    held = {}

    def snap_before(where):
        if where.startswith('pallet 1'):
            time.sleep(0.6)
            with watch.lock:
                held[where] = list(watch.boxes)
    if a.diag:
        motion.before_release = snap_before
    motion.home()
    cycles, fault, landed = [], None, []
    for c in range(min(a.cycles, n)):
        k, slot = n - 1 - c, c                    # top box of pallet 0 -> next free slot of pallet 1
        try:
            t0 = time.monotonic()
            motion.start(True, 0, k)                 # one depalletizing and one palletizing program, run through
            motion.run()
            t1 = time.monotonic()
            motion.start(False, 1, slot)
            motion.run()
            t2 = time.monotonic()
        except CellFault as e:
            fault = f'cycle {c} (pallet 0 slot {k} -> pallet 1 slot {slot}): {e}'
            print(f'FAULT {fault}', flush=True)
            if a.out:
                import pickle
                with rec.lock:
                    t_end = rec.hw[-1][0] if rec.hw else 0.0
                    pickle.dump(dict(cmds=[x for x in rec.cmds if x[0] > t_end - 6], hw=[x for x in rec.hw if x[0] > t_end - 6],
                                     fault=fault), open(a.out + '.fault.pkl', 'wb'))
            break
        time.sleep(0.6)                           # a fresh /test_boxes (2 Hz)
        with watch.lock:
            snap = list(watch.boxes)
        (ex, ey, ez), _ = world_cell.slot_world(1, slot)
        bx, by, bz, _ = snap[k]
        landing = math.dist((bx, by, bz), (ex, ey, ez - p.box[2] / 2))
        # cartons placed earlier: moved since they landed?
        moved = []
        for c2, s0 in enumerate(landed):
            b = snap[n - 1 - c2]
            d = math.dist(b[:3], s0[:3])
            if d > 0.002:
                moved.append((f'box {n - 1 - c2} (slot {c2})', round(d * 1000, 1)))
            landed[c2] = b
        landed.append(snap[k])
        cycles.append(dict(cycle=c, depal_s=round(t1 - t0, 2), pal_s=round(t2 - t1, 2),
                           landing_mm=round(landing * 1000, 1), others_moved=moved))
        if a.diag:
            (_, _), pyaw = world_cell.pallet_world(1)[:2], world_cell.pallet_world(1)[2]
            cu, su = math.cos(math.radians(pyaw)), math.sin(math.radians(pyaw))
            def rel(b):                               # error in the pallet frame (long, short, up) mm and yaw deg
                dx, dy, dz = b[0] - ex, b[1] - ey, b[2] - (ez - p.box[2] / 2)
                return (round(1000 * (cu * dx + su * dy), 1), round(1000 * (-su * dx + cu * dy), 1), round(1000 * dz, 1),
                        round((b[3] - pyaw + 45.0) % 90.0 - 45.0, 2))
            pre = held.get(f'pallet 1 slot {slot}')
            print(f'   slot {slot}: held before release {rel(pre[k]) if pre else "?"} -> landed {rel(snap[k])} '
                  f'(long, short, up mm; yaw deg)', flush=True)
        if landing > 0.005 or moved:
            print(f'   landing {landing * 1000:.1f} mm; moved since landing: {moved}', flush=True)
        print(f'cycle {c:2d}: pallet 0 slot {k:2d} -> deck -> pallet 1 slot {slot:2d} | depalletize {t1 - t0:5.2f} s '
              f'palletize {t2 - t1:5.2f} s (wall)', flush=True)
    try:
        motion.home()
    except CellFault as e:
        fault = fault or f'home: {e}'
    time.sleep(1.5)
    with watch.lock:
        boxes = list(watch.boxes)
    errors = []
    for c in range(len(cycles)):
        k, slot = n - 1 - c, c
        (x, y, z), yaw = world_cell.slot_world(1, slot)
        bx, by, bz, byaw = boxes[k]
        pos = math.dist((bx, by, bz), (x, y, z - p.box[2] / 2))
        dyaw = abs((byaw - yaw + 45.0) % 90.0 - 45.0)       # square footprint: any multiple of 90 deg fits
        errors.append(dict(box=k, slot=slot, pos_mm=round(pos * 1000, 1), yaw_deg=round(dyaw, 2)))
    bad = [e for e in errors if e['pos_mm'] > POS_TOL_M * 1000 or e['yaw_deg'] > YAW_TOL_DEG]
    summary = dict(cycles=len(cycles), requested=min(a.cycles, n), fault=fault, misplaced=len(bad),
                   pos_mm_max=max((e['pos_mm'] for e in errors), default=None),
                   yaw_deg_max=max((e['yaw_deg'] for e in errors), default=None),
                   depal_s_mean=round(sum(c['depal_s'] for c in cycles) / max(len(cycles), 1), 2),
                   pal_s_mean=round(sum(c['pal_s'] for c in cycles) / max(len(cycles), 1), 2), speed=a.speed)
    summary['gate'] = 'PASS' if (fault is None and not bad and len(cycles) == summary['requested']) else 'FAIL'
    print(f'gripper test {a.arm_id}: {summary}', flush=True)
    for e in bad:
        print(f'  misplaced: {e}', flush=True)
    if a.out:
        yaml.safe_dump(dict(summary=summary, cycles=cycles, placement=errors), open(a.out, 'w'), sort_keys=False)
    return _exit(0 if summary['gate'] == 'PASS' else 1)


if __name__ == '__main__':
    sys.exit(main())
