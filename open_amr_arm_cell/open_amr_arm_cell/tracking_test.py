"""Tracking test of a cell's arm (Phase 4b gate): N Pilz moves planned by MoveIt, executed through the cell's
joint_trajectory_controller on its hardware (Isaac through the arm bridge, or mock), with the arm's actual joint
states recorded on sim time.

Moves alternate between PTP transfers among taught points (above slots / above the deck, from the reach study) and LIN
approach / lift moves (down and up along the tool axis), i.e. what the cell does all day. Per move:
  tracking      TCP position / orientation difference between what ros2_control commanded the hardware
                (/<arm_id>/isaac_joint_commands) and where the simulated arm was (/<arm_id>/isaac_joint_states) at the
                same sim time, over the whole move. Gate: < 1 mm and < 0.5 deg (hardware:=topic only).
  final error   TCP error against the move's goal once the controller reports it done and the arm settled (0.3 s)
                — the accuracy a box is put down with. Gate: < 1 mm and < 0.5 deg.
  lag           time shift that best aligns the actual joint trajectory with the commanded one.

    ros2 run open_amr_arm_cell tracking_test --arm-id arm_receiving --moves 100 [--out report.yaml]
Needs the cell's control stack (arm_cell.launch.py) and, for hardware:=topic, the simulator (open_amr_sim.py --arms).
"""
import argparse
import math
import os
import random
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.qos
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from . import ur_ik
from .cell_geometry import Cell, CellParams
from .reach_study import HOME, JOINTS, _exit, moveit_configs, moveit_py, taught_ik, tool_down

GATE_POS_M, GATE_ROT_DEG = 0.001, 0.5


def tcp(q, p: CellParams):
    """TCP pose (4x4) in the cell frame for joint positions q."""
    T = ur_ik.fk(q)
    T[2, 3] += p.pedestal_height
    T[:3, 3] += T[:3, 2] * p.tool_length
    return T


def rot_err_deg(A, B):
    R = A[:3, :3].T @ B[:3, :3]
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))))


class Recorder(Node):
    def __init__(self, arm_id, sim_time=True):
        super().__init__('tracking_recorder', namespace=arm_id,
                         parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=sim_time)])
        self.lock, self.samples, self.cmds, self.hw = threading.Lock(), [], [], []
        self.create_subscription(JointState, 'joint_states', self.on_js, 50)
        # the hardware layer (sim): commands in, actual states out, both stamped with sim time
        be = rclpy.qos.QoSProfile(depth=200, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)  # matches both
        self.create_subscription(JointState, 'isaac_joint_commands', lambda m: self.store(m, self.cmds), be)
        self.create_subscription(JointState, 'isaac_joint_states', lambda m: self.store(m, self.hw), be)

    def store(self, m, where):
        idx = {n: i for i, n in enumerate(m.name)}
        if all(j in idx for j in JOINTS) and all(math.isfinite(m.position[idx[j]]) for j in JOINTS):
            with self.lock:
                where.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, [m.position[idx[j]] for j in JOINTS]))

    def tracking(self, t0, t1, p):
        """Max TCP (position m, rotation deg) difference between commanded and actual in [t0, t1], and the lag
        (s) that best aligns them; None without hardware-layer topics."""
        with self.lock:
            cmds = [c for c in self.cmds if t0 - 0.1 <= c[0] <= t1 + 0.1]
            hw = [h for h in self.hw if t0 <= h[0] <= t1]
        if len(cmds) < 2 or not hw:
            return None
        ct = [c[0] for c in cmds]

        def cmd_at(t):
            k = max(1, min(len(ct) - 1, int(np.searchsorted(ct, t))))
            (ta, qa), (tb, qb) = cmds[k - 1], cmds[k]
            f = max(0.0, min(1.0, (t - ta) / max(tb - ta, 1e-9)))
            return [a + f * (b - a) for a, b in zip(qa, qb)]
        pos, rot = 0.0, 0.0
        for t, q in hw:
            Tc, Ta = tcp(cmd_at(t), p), tcp(q, p)
            pos = max(pos, float(np.linalg.norm(Tc[:3, 3] - Ta[:3, 3])))
            rot = max(rot, rot_err_deg(Tc, Ta))
        lag = min(np.arange(0.0, 0.1, 0.002), key=lambda L: max(
            max(abs(u - v) for u, v in zip(q, cmd_at(t - L))) for t, q in hw))
        return pos, rot, float(lag)

    def on_js(self, m):
        idx = {n: i for i, n in enumerate(m.name)}
        if not all(j in idx for j in JOINTS):
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self.lock:
            self.samples.append((t, [m.position[idx[j]] for j in JOINTS]))

    def latest(self):
        with self.lock:
            return self.samples[-1] if self.samples else None

    def window(self, t0, t1):
        with self.lock:
            return [s for s in self.samples if t0 <= s[0] <= t1]

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def interp(traj, t):
    """Planned joint positions at time t (s from trajectory start); traj = [(t, q)]."""
    if t <= traj[0][0]:
        return traj[0][1]
    for (ta, qa), (tb, qb) in zip(traj, traj[1:]):
        if t <= tb:
            f = (t - ta) / max(tb - ta, 1e-9)
            return [a + f * (b - a) for a, b in zip(qa, qb)]
    return traj[-1][1]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--arm-id', default='arm_receiving')
    ap.add_argument('--moves', type=int, default=100)
    ap.add_argument('--seed', type=int, default=4)
    ap.add_argument('--layout', default=os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')),
                                                    'sim', 'configs', 'warehouse_layout.yaml'))
    ap.add_argument('--out', default='')
    ap.add_argument('--dump', default='', help='write the raw command / state streams of every move here (npz)')
    ap.add_argument('--wall-time', action='store_true', help='no /clock (mock hardware without a simulator)')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    from moveit.planning import PlanRequestParameters
    from moveit.core.robot_state import RobotState

    p = CellParams.from_layout(yaml.safe_load(open(a.layout)))
    cell = Cell((0.0, 0.0, 0.0), p)
    cfg = moveit_configs(p, hardware='topic', execution=True).to_dict()
    cfg['use_sim_time'] = not a.wall_time
    if a.wall_time:
        cfg = moveit_configs(p, hardware='mock', execution=True).to_dict()
        cfg['use_sim_time'] = False
    moveit = moveit_py('tracking_test', cfg, name_space=a.arm_id)
    arm = moveit.get_planning_component('arm')
    ptp, lin = PlanRequestParameters(moveit, 'ptp'), PlanRequestParameters(moveit, 'lin')
    model = moveit.get_robot_model()
    rec = Recorder(a.arm_id, sim_time=not a.wall_time)
    ex = SingleThreadedExecutor()
    ex.add_node(rec)
    threading.Thread(target=ex.spin, daemon=True).start()
    t_wait = time.time()
    while rec.latest() is None and time.time() - t_wait < 60:
        time.sleep(0.1)
    if rec.latest() is None:
        print(f'no /{a.arm_id}/joint_states: is the cell stack (and the simulator) running?', flush=True)
        return _exit(2)
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    fjt = ActionClient(rec, FollowJointTrajectory, f'/{a.arm_id}/joint_trajectory_controller/follow_joint_trajectory')
    if not fjt.wait_for_server(timeout_sec=60.0):
        print('trajectory controller action server not available', flush=True)
        return _exit(2)
    time.sleep(2.0)                                   # MoveIt's own action client connects too

    # targets: taught points (above slots / deck) + LIN down/up along the tool axis
    rng = random.Random(a.seed)
    points = []
    for i in (0, 1):
        for k in range(0, p.capacity, 5):
            (x, y, z), yaw = cell.slot_cell(i, k)
            points.append((x, y, cell.transit_height(), yaw))
    (dx, dy, dz), dyaw = cell.deck_cell()
    points.append((dx, dy, dz + p.approach, dyaw))

    def plan_exec(goal_q=None, goal_pose=None, use_lin=False):
        arm.set_start_state_to_current_state()
        if goal_q is not None:
            s = RobotState(model)
            s.joint_positions = dict(zip(JOINTS, goal_q))
            s.update()
            arm.set_goal_state(robot_state=s)
        else:
            from geometry_msgs.msg import PoseStamped
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose = goal_pose
            arm.set_goal_state(pose_stamped_msg=ps, pose_link='tcp')
        res = arm.plan(single_plan_parameters=lin if use_lin else ptp)
        if not res:
            return None
        jt = res.trajectory.get_robot_trajectory_msg().joint_trajectory
        idx = [jt.joint_names.index(j) for j in JOINTS]
        traj = [(pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9, [pt.positions[i] for i in idx])
                for pt in jt.points]
        t0 = rec.now_s()
        ok = moveit.execute(res.trajectory, controllers=[])
        t1 = rec.now_s()
        time.sleep(0.3 * 1.5)                       # settle (sim time runs at RTF < 1)
        return traj, t0, t1, ok

    results, raw = [], []
    plan_exec(goal_q=HOME)
    for n in range(a.moves):
        if n % 2 == 0:
            x, y, z, yaw = rng.choice(points)
            seed = list(HOME)
            seed[0] = math.atan2(y, x)
            goal = taught_ik(tool_down(x, y, z, yaw), seed, p)
            kind, r = 'PTP', plan_exec(goal_q=goal) if goal is not None else None
            goal_T = tcp(goal, p) if goal is not None else None
        else:
            Tn = tcp(rec.latest()[1], p)
            dz = -rng.uniform(0.1, 0.35) if Tn[2, 3] > 0.9 else rng.uniform(0.1, 0.35)
            goal_T = Tn.copy()
            goal_T[2, 3] += dz
            yaw = math.degrees(math.atan2(Tn[1, 0], Tn[0, 0]))
            kind, r = 'LIN', plan_exec(goal_pose=tool_down(goal_T[0, 3], goal_T[1, 3], goal_T[2, 3], yaw), use_lin=True)
        if r is None:
            results.append(dict(move=n, kind=kind, planned=False))
            continue
        traj, t0, t1, ok = r
        q_end = rec.latest()[1]
        T_end = tcp(q_end, p)
        pos_err = float(np.linalg.norm(T_end[:3, 3] - goal_T[:3, 3]))
        rot_err = rot_err_deg(T_end, goal_T)
        tr = rec.tracking(t0, t1, p)
        if a.dump:
            with rec.lock:
                raw.append(dict(move=n, kind=kind, t0=t0, t1=t1,
                                cmds=[c for c in rec.cmds if t0 - 0.2 <= c[0] <= t1 + 0.2],
                                hw=[h for h in rec.hw if t0 - 0.2 <= h[0] <= t1 + 0.2]))
        results.append(dict(move=n, kind=kind, planned=True, executed=bool(ok), duration_s=round(traj[-1][0], 3),
                            final_mm=round(pos_err * 1000, 3), final_deg=round(rot_err, 3),
                            track_mm=round(tr[0] * 1000, 3) if tr else None, track_deg=round(tr[1], 3) if tr else None,
                            lag_ms=round(tr[2] * 1000, 1) if tr else None))
        print(f'move {n:3d} {kind} {traj[-1][0]:4.1f} s: final {pos_err * 1000:6.3f} mm {rot_err:6.3f} deg | tracking '
              + (f'{tr[0] * 1000:6.3f} mm {tr[1]:6.3f} deg, lag {tr[2] * 1000:4.1f} ms' if tr else 'n/a (no hardware topics)')
              + f' | {"ok" if ok else "EXECUTION FAILED"}', flush=True)

    done = [r for r in results if r.get('planned')]
    summary = dict(moves=len(results), planned=len(done), executed=sum(r['executed'] for r in done),
                   final_mm_max=max((r['final_mm'] for r in done), default=None),
                   final_deg_max=max((r['final_deg'] for r in done), default=None),
                   track_mm_max=max((r['track_mm'] for r in done if r['track_mm'] is not None), default=None),
                   track_deg_max=max((r['track_deg'] for r in done if r['track_deg'] is not None), default=None),
                   lag_ms_mean=round(sum(r['lag_ms'] or 0.0 for r in done) / max(len(done), 1), 1))
    tracked = summary['track_mm_max'] is not None
    passed = (summary['planned'] == a.moves and summary['executed'] == a.moves and
              summary['final_mm_max'] < GATE_POS_M * 1000 and summary['final_deg_max'] < GATE_ROT_DEG and
              (not tracked or (summary['track_mm_max'] < GATE_POS_M * 1000 and summary['track_deg_max'] < GATE_ROT_DEG)))
    summary['gate'] = 'PASS' if passed else 'FAIL'
    print(f'tracking test {a.arm_id}: {summary}', flush=True)
    if a.dump:
        import pickle
        pickle.dump(raw, open(a.dump, 'wb'))
    if a.out:
        yaml.safe_dump(dict(summary=summary, moves=results), open(a.out, 'w'), sort_keys=False)
    return _exit(0 if passed else 1)


if __name__ == '__main__':
    sys.exit(main())
