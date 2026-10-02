"""Reach study of the palletizing cell (Phase 4b gate): plan every pick / place of the pallet pattern with MoveIt 2 +
Pilz (the planner the cell runs), with collision checking against the pedestal, both pallets and their boxes, the
docked AMR and the floor, carrying the box attached to the gripper.

For each pallet slot, both directions, the cell's own motion program (cell_program.py: depalletize / palletize, with
the deck zone interlock — the keep-out volume over the bay is in the scene until DOCK, the docked robot from DOCK to
UNDOCK, the keep-out volume again after UNDOCK). Scene per slot = the worst case of the pattern: the slot's pallet
holds the boxes below it (and the picked one), the other pallet is full. The four gripper orientations (box square
footprint) are tried, then the same with a via point at the transit height; the fastest feasible one is kept. A slot
passes only if every step plans and passes the dense collision check.

Writes a taught pattern (joint configurations above each slot / the deck, gripper yaw, segment times) that the cell
controller uses as PTP goals, as a UR palletizer's taught waypoints.

    ros2 run open_amr_arm_cell reach_study --layout $OPENAMR_ROOT/sim/configs/warehouse_layout.yaml \\
        --out config/taught_pattern.yaml
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive

from .cell_geometry import Cell, CellParams
from . import cell_program as prog
from .cell_program import CONTACT, HOME  # noqa: F401 (re-exported)
from . import ur_ik

JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
LIMITS = [(-2 * math.pi, 2 * math.pi)] * 2 + [(-math.pi, math.pi)] + [(-2 * math.pi, 2 * math.pi)] * 3   # ur_description
AMR = (0.806, 0.64, 0.334)          # AMR body up to the deck surface (collision outline, Robot Parameters)
VACUUM_S = 0.3                      # vacuum build-up / release (typical area gripper with a vacuum switch)


def moveit_configs(p: CellParams, hardware='none', execution=False):
    from moveit_configs_utils import MoveItConfigsBuilder
    b = MoveItConfigsBuilder('arm_cell', package_name='open_amr_arm_cell')
    b = b.robot_description(file_path='urdf/arm_cell.urdf.xacro',
                            mappings={'ur_type': p.ur_type,
                                      'pedestal_height': str(p.pedestal_height), 'pedestal_size': str(p.pedestal_size),
                                      'tool_length': str(p.tool_length), 'hardware': hardware})
    b = b.robot_description_semantic(file_path='srdf/arm_cell.srdf.xacro')
    b = b.robot_description_kinematics(file_path='config/kinematics.yaml')
    b = b.joint_limits(file_path='config/joint_limits.yaml')
    b = b.pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
    b = b.planning_pipelines(pipelines=['pilz_industrial_motion_planner'],
                             default_planning_pipeline='pilz_industrial_motion_planner')
    b = b.moveit_cpp(file_path='config/moveit_cpp.yaml')
    if execution:                                   # execute through the cell's FollowJointTrajectory controller
        b = b.trajectory_execution(file_path='config/moveit_controllers.yaml', moveit_manage_controllers=False)
    return b.to_moveit_configs()


def tool_down(x, y, z, yaw_deg):
    """TCP pose, suction face down (tcp z = -world z), tcp x along yaw."""
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    h = math.radians(yaw_deg) / 2.0            # q = Rz(yaw) * Rx(pi)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = math.cos(h), math.sin(h), 0.0, 0.0
    return p


def moveit_py(node_name, cfg, name_space=''):
    """MoveItPy in a namespace (one per cell: /<arm_id>). Its config_dict path writes the parameters under the bare
    node name, which doesn't match the namespaced node (MoveIt then finds no robot_description_semantic): write them
    under the wildcard key instead."""
    import tempfile
    from moveit.planning import MoveItPy
    if not name_space:
        return MoveItPy(node_name=node_name, config_dict=cfg)
    f = tempfile.NamedTemporaryFile('w', prefix=f'moveit_{name_space}_', suffix='.yaml', delete=False)
    yaml.safe_dump({'/**': {'ros__parameters': cfg}}, f)
    f.close()
    return MoveItPy(node_name=node_name, name_space=name_space, launch_params_filepaths=[f.name])


DENSE_STEP_RAD = 0.01                # dense path check: max joint step between checked states
TRANSFER_MARGIN = 0.02              # m: PTP transfers keep the carried carton this far from everything


def dense_collision(psm, jt, step=DENSE_STEP_RAD):
    """First colliding state of a joint trajectory, checked densely (interpolated between its waypoints so no joint
    moves more than `step` between checks), with whatever is attached to the tool; None if the path is clear.
    MoveIt's ValidateSolution checks only the waypoints (Pilz samples ~0.1 s apart): at production speed the carried
    carton moved ~20 cm between checks and grazed a stacked carton in the sim (gripper test, 2026-09-26)."""
    names = list(jt.joint_names)
    pts = [[pt.positions[names.index(j)] for j in JOINTS] for pt in jt.points]
    with psm.read_write() as scene:
        state = scene.current_state                      # the scene's own state holds the attached bodies
        saved = dict(state.joint_positions)
        try:
            for a, b in zip(pts, pts[1:]):
                n = max(1, int(max(abs(x - y) for x, y in zip(a, b)) / step) + 1)
                for i in range(1, n + 1):
                    q = [x + (y - x) * i / n for x, y in zip(a, b)]
                    state.joint_positions = dict(zip(JOINTS, q))
                    state.update()
                    if scene.is_state_colliding(joint_model_group_name='arm'):
                        return q
        finally:
            state.joint_positions = saved
            state.update()
    return None


def taught_ik(pose, ref, p: CellParams):
    """Taught point for a TCP pose (cell frame): closed-form IK (ur_ik), the solution nearest `ref` within UR's joint
    limits — deterministic, unlike KDL's random restarts (the same study gave different configurations and 91 vs 92
    feasible cycles run to run). None if unreachable."""
    x, y, z, w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z - p.pedestal_height]   # cell -> base_link
    T[:3, 3] -= R[:, 2] * p.tool_length                                                  # tcp -> tool0
    return ur_ik.closest(ur_ik.ik(T, ur_ik.params(p.ur_type)), ref, LIMITS)


def box_object(name, size, xyz, yaw_deg=0.0, op=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = 'world'
    co.id = name
    co.operation = op
    if op == CollisionObject.ADD:
        sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        pose.orientation.z, pose.orientation.w = math.sin(math.radians(yaw_deg) / 2), math.cos(math.radians(yaw_deg) / 2)
        co.primitives, co.primitive_poses = [sp], [pose]
    return co


def zone_objects(cell, docked, deck_box=False):
    """Planning-scene objects of the bay: the docked robot (+ the carton on its deck) or the keep-out volume."""
    p = cell.p
    rm = lambda n: box_object(n, (), (), op=CollisionObject.REMOVE)
    if not docked:
        x, y, sx, sy, top = cell.deck_zone()
        return [rm('amr'), rm('deck_box'), rm('deck_rail_0'), rm('deck_rail_1'),
                box_object('deck_zone', (sx, sy, top), (x, y, top / 2))]
    (dx, dy, dz), dyaw = cell.deck_cell()
    rails = [box_object(f'deck_rail_{i}', r[3:], r[:3]) for i, r in enumerate(cell.rails_cell())]
    return [rm('deck_zone'), box_object('amr', AMR, (p.deck_distance, 0.0, AMR[2] / 2))] + rails + [
            box_object('deck_box', p.box, (dx, dy, dz - p.box[2] / 2), dyaw) if deck_box else rm('deck_box')]


class Study:
    def __init__(self, cell: Cell):
        from moveit.planning import MoveItPy, PlanRequestParameters
        self.cell, self.p = cell, cell.p
        cfg = moveit_configs(self.p).to_dict()
        # offline: no robot publishes joint states, every plan gets its start state explicitly
        cfg['planning_scene_monitor_options']['wait_for_initial_state_timeout'] = 0.0
        self.moveit = MoveItPy(node_name='reach_study', config_dict=cfg)
        self.arm = self.moveit.get_planning_component('arm')
        self.psm = self.moveit.get_planning_scene_monitor()
        self.model = self.moveit.get_robot_model()
        self.ptp = PlanRequestParameters(self.moveit, 'ptp')
        self.lin = PlanRequestParameters(self.moveit, 'lin')
        self.lin_place = PlanRequestParameters(self.moveit, 'lin_place')
        self.objects = set()
        self.carrying = False
        self.record = None

    def self_check(self):
        """The study is only as good as its collision checking: (1) a move only the carried box collides on must
        fail (a fresh start state once dropped attached bodies and it passed); (2) taught points are deterministic.
        Returns a list of failures."""
        fails = []
        self.clear()
        start = self.ik(tool_down(0.9, 0.0, 1.0, 0.0), HOME)
        self.apply([box_object('wall', (0.6, 0.6, 0.02), (0.9, 0.0, 0.70))])
        goal = tool_down(0.9, 0.0, 0.84, 0.0)
        if self.plan(start, goal_pose=goal, lin=True) is None:
            fails.append('free move over the wall did not plan')
        self.attach(True)
        if self.plan(start, goal_pose=goal, lin=True) is not None:
            fails.append('carried box NOT collision-checked: a move into the wall planned')
        self.attach(False)
        self.clear()
        # (3) the dense path check sees what waypoint checking misses: a post only the mid-swing tool passes
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        qa, qb = list(HOME), list(HOME)
        qa[0], qb[0] = -1.0, 1.0
        jt = JointTrajectory(joint_names=JOINTS, points=[JointTrajectoryPoint(positions=qa),
                                                         JointTrajectoryPoint(positions=qb)])
        T = ur_ik.fk(HOME, ur_ik.params(self.p.ur_type))
        self.apply([box_object('post', (0.08, 0.08, 0.08), (T[0, 3], T[1, 3], T[2, 3] + self.p.pedestal_height
                                                              + self.p.tool_length * T[2, 2]))])
        with self.psm.read_only() as scene:
            ends = [scene.is_state_colliding(robot_state=self.state(q), joint_model_group_name='arm') for q in (qa, qb)]
        if any(ends) or dense_collision(self.psm, jt) is None:
            fails.append('dense path check does not catch a collision between waypoints')
        self.clear()
        (x, y, z), yaw = self.cell.slot_cell(0, 7)
        seed = list(HOME)
        seed[0] = math.atan2(y, x)
        a, b = self.ik(tool_down(x, y, z + 0.1, yaw), seed), self.ik(tool_down(x, y, z + 0.1, yaw), seed)
        if a is None or a != b:
            fails.append(f'taught point not deterministic: {a} vs {b}')
        return fails

    # ------------------------------------------------------------ scene
    def apply(self, objs):
        with self.psm.read_write() as scene:
            for co in objs:
                scene.apply_collision_object(co)
                (self.objects.add if co.operation == CollisionObject.ADD else self.objects.discard)(co.id)

    def clear(self):
        self.apply([box_object(n, (), (), op=CollisionObject.REMOVE) for n in list(self.objects)])

    def attach(self, attach, pad=0.0):
        """Attach the carried box to the gripper (the TCP is its top centre) or detach it. Detaching makes MoveIt put
        the box back into the world where it hangs (touching the pad): that copy is removed here, the caller adds the
        box where it lands."""
        aco = AttachedCollisionObject()
        aco.link_name = 'tcp'
        aco.touch_links = ['tcp', 'vacuum_gripper', 'wrist_3_link']
        b = self.p.box
        co = box_object('carried', (b[0] + 2 * pad, b[1] + 2 * pad, b[2] + pad), (0.0, 0.0, (b[2] + pad) / 2)) if attach \
            else box_object('carried', (), (), op=CollisionObject.REMOVE)
        self.carrying = attach
        co.header.frame_id = 'tcp'
        aco.object = co
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object(aco)
            if not attach:
                scene.apply_collision_object(box_object('carried', (), (), op=CollisionObject.REMOVE))

    def set_scene(self, pallet, filled):
        """pallet `pallet` holds slots 0..filled-1, the other one is full; no robot docked (keep-out volume over the
        bay)."""
        self.clear()
        objs = [box_object('floor', (6.0, 6.0, 0.02), (0.0, 0.0, -0.011))]
        for i in (0, 1):
            (cx, cy), yaw = self.cell.pallet_cell(i)
            ps = self.p.pallet_size
            objs.append(box_object(f'pallet_{i}', ps, (cx, cy, ps[2] / 2), yaw))
            for k in range(filled if i == pallet else self.p.capacity):
                (x, y, z), byaw = self.cell.slot_cell(i, k)
                objs.append(box_object(f'box_{i}_{k}', self.p.box, (x, y, z - self.p.box[2] / 2), byaw))
        self.apply(objs)
        self.dock(False)

    def dock(self, docked, deck_box=False):
        """Robot docked (its outline, a carton on its deck if deck_box) or not (the deck zone's keep-out volume)."""
        self.apply(zone_objects(self.cell, docked, deck_box))

    # ------------------------------------------------------------ planning
    def state(self, q):
        from moveit.core.robot_state import RobotState
        s = RobotState(self.model)
        s.joint_positions = dict(zip(JOINTS, q))
        s.update()
        return s

    def ik(self, pose, ref):
        return taught_ik(pose, ref, self.p)

    def plan(self, start, goal_q=None, goal_pose=None, lin=False, place=False):
        """(end joint config, duration s) or None."""
        # start from the scene's own current state (it holds the attached box); a fresh RobotState from joint values
        # carries no attached bodies, and the carried box silently drops out of collision checking (found by
        # test_attached_box_is_checked, 2026-09-26)
        with self.psm.read_write() as scene:
            scene.current_state.joint_positions = dict(zip(JOINTS, start))
            scene.current_state.update()
        self.arm.set_start_state_to_current_state()
        if goal_q is not None:
            self.arm.set_goal_state(robot_state=self.state(goal_q))
        else:
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose = goal_pose
            self.arm.set_goal_state(pose_stamped_msg=ps, pose_link='tcp')
        res = self.arm.plan(single_plan_parameters=self.lin_place if place else self.lin if lin else self.ptp)
        if not res:
            return None
        # dense check; PTP transfers with the carried carton padded by TRANSFER_MARGIN
        if not lin and self.carrying:
            self.attach(True, pad=TRANSFER_MARGIN)
        hit = dense_collision(self.psm, res.trajectory.get_robot_trajectory_msg().joint_trajectory)
        if not lin and self.carrying:
            self.attach(True)
        if hit is not None:
            return None
        msg = res.trajectory.get_robot_trajectory_msg().joint_trajectory
        if self.record is not None:                   # payload study: keep the trajectory and whether it carries
            self.record.append((self.carrying, msg))
        last = msg.points[-1]
        idx = [msg.joint_names.index(j) for j in JOINTS]
        d = last.time_from_start
        return [last.positions[i] for i in idx], d.sec + d.nanosec * 1e-9

    def sequence(self, pallet, slot, yaw_grip, yaw_deck, depal, via=False, hold_high=False):
        """Plan one full cycle of the cell's program. Returns dict(times, deck_s, taught) or the name of the failing
        step. via: carry the box across at the transit height (clear of a full pallet) instead of swinging straight
        from the lift point."""
        (sx, sy, _), _ = self.cell.slot_cell(pallet, slot)
        (dx, dy, dz), _ = self.cell.deck_cell()
        p_slot, p_deck = prog.taught_poses(self.cell, depal, pallet, slot, yaw_grip, yaw_deck, via, hold_high)
        seed = list(HOME)
        seed[0] = math.atan2(sy, sx)
        q_slot = self.ik(tool_down(*p_slot), seed)
        if q_slot is None:
            return 'ik above slot'
        seed_d = list(HOME)
        seed_d[0] = math.atan2(dy, dx)
        q_deck = self.ik(tool_down(*p_deck), seed_d)
        if q_deck is None:
            return 'ik deck hold'
        t = dict(slot_above=q_slot, deck_hold=q_deck, grip_yaw=yaw_grip, deck_yaw=yaw_deck, via=via, hold_high=hold_high)
        times, q, docked_at, deck_s = {}, list(HOME), None, 0.0
        try:
            for st in prog.program(self.cell, depal, pallet, slot, t):
                if st.kind == prog.DOCK:
                    self.dock(True, deck_box=not depal)
                    docked_at = sum(times.values())
                elif st.kind == prog.UNDOCK:
                    deck_s = sum(times.values()) - docked_at
                    self.dock(False)
                elif st.kind == prog.GRIP:
                    self.apply([box_object('deck_box' if st.where == 'deck' else f'box_{pallet}_{slot}', (), (),
                                           op=CollisionObject.REMOVE)])
                    self.attach(True)
                    times[st.name] = VACUUM_S
                elif st.kind == prog.RELEASE:           # the released box lands where it was put down
                    self.attach(False)
                    if st.where == 'deck':
                        self.apply([box_object('deck_box', self.p.box, (dx, dy, dz - self.p.box[2] / 2), yaw_deck)])
                    else:
                        (bx, by, bz), _ = self.cell.slot_cell(pallet, slot)
                        self.apply([box_object(f'box_{pallet}_{slot}', self.p.box, (bx, by, bz - self.p.box[2] / 2),
                                               yaw_grip)])
                    times[st.name] = VACUUM_S
                else:
                    r = self.plan(q, goal_q=st.q) if st.kind == prog.PTP else \
                        self.plan(q, goal_pose=tool_down(*st.pose), lin=True, place=st.place)
                    if r is None:
                        return st.name
                    q, times[st.name] = r
        finally:
            self.attach(False)
        return dict(times=times, via=via, deck_s=round(deck_s, 2),
                    taught=dict(slot_above=[round(v, 5) for v in q_slot], deck_hold=[round(v, 5) for v in q_deck],
                                grip_yaw=yaw_grip, deck_yaw=yaw_deck, hold_high=hold_high))

    def slot(self, pallet, slot, depal):
        """Best (fastest) feasible gripper orientation for a slot, or the failures per orientation."""
        _, box_yaw = self.cell.slot_cell(pallet, slot)
        best, fails = None, []
        # preference: direct swing; else a via point at the transit height with the carton still held low over the
        # deck (shortest robot wait); else held at the transit height. Within a group the fastest cycle wins.
        groups = [(False, False), (True, False), (True, True)]
        for (via, high), m in [(g, m) for g in groups for m in range(4)]:
            if best is not None and (via, high) != (best['via'], best['taught']['hold_high']):
                break                                    # an earlier (preferred) group already works
            yaw_grip = box_yaw + 90.0 * m
            # the box turns with the gripper: on the deck it must end square to the AMR (the deck conveyor pushes it
            # into a bin), i.e. at a multiple of 90 deg from the AMR's heading (180 in the cell frame); take the one
            # closest to the grip orientation so wrist 3 turns least. (Keeping the grip orientation put boxes on the
            # deck at the pallet's angle and on the other pallet 30 deg off its pattern: gripper test, 2026-09-26.)
            yaw_deck = 180.0 + 90.0 * round((yaw_grip - 180.0) / 90.0)
            filled = slot + 1 if depal else slot
            self.set_scene(pallet, filled)
            r = self.sequence(pallet, slot, yaw_grip, yaw_deck, depal, via, high)
            if isinstance(r, str):
                fails.append(f'{int(yaw_grip) % 360}deg{" via" if via else ""}{" high" if high else ""}: {r}')
                continue
            r['cycle_s'] = round(sum(r['times'].values()), 2)
            if best is None or r['cycle_s'] < best['cycle_s']:
                best = r
        return best, fails


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--layout', default=os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')),
                                                    'sim', 'configs', 'warehouse_layout.yaml'))
    ap.add_argument('--out', default='taught_pattern.yaml')
    ap.add_argument('--slots', default='', help='e.g. 0,5,23 (default: all)')
    ap.add_argument('--check', action='store_true', help='only the self-check (collision checking, determinism)')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    layout = yaml.safe_load(open(a.layout))
    p = CellParams.from_layout(layout)
    cell = Cell((0.0, 0.0, 0.0), p)                  # cell frame: both cells are identical in it
    st = Study(cell)
    fails = st.self_check()
    print(f'self-check: {"; ".join(fails) if fails else "OK (carried box collision-checked, paths checked densely, taught points deterministic)"}',
          flush=True)
    if a.check or fails:
        return _exit(1 if fails else 0)
    slots = [int(s) for s in a.slots.split(',')] if a.slots else list(range(p.capacity))
    out = dict(cell=dict(vars(p)), pattern={}, failures={})
    t0, n_ok, n = time.time(), 0, 0
    for depal in (True, False):
        mode = 'depalletize' if depal else 'palletize'
        for pallet in (0, 1):
            for k in slots:
                n += 1
                best, fails = st.slot(pallet, k, depal)
                key = f'{mode}/{pallet}/{k}'
                if best is None:
                    out['failures'][key] = fails
                    print(f'FAIL {key}: {"; ".join(fails)}', flush=True)
                else:
                    n_ok += 1
                    out['pattern'][key] = best
                    print(f'ok   {key}: cycle {best["cycle_s"]:.2f} s, robot waits {best["deck_s"]:.2f} s (grip {int(best["taught"]["grip_yaw"]) % 360} deg'
                          f'{", via transit height" if best["via"] else ""}{", held high" if best["taught"]["hold_high"] else ""})',
                          flush=True)
    cycles = [v['cycle_s'] for v in out['pattern'].values()]
    decks = [v['deck_s'] for v in out['pattern'].values()]
    out['summary'] = dict(passed=n_ok, total=n, cycle_mean_s=round(sum(cycles) / max(len(cycles), 1), 2),
                          cycle_max_s=max(cycles, default=0.0), deck_mean_s=round(sum(decks) / max(len(decks), 1), 2),
                          deck_max_s=max(decks, default=0.0), wall_s=round(time.time() - t0, 1))
    for k in ('pallet_size', 'box', 'pattern', 'amr_footprint'):
        out['cell'][k] = list(out['cell'][k])
    with open(a.out, 'w') as f:
        f.write(f'# Taught pallet pattern of the {p.ur_type.upper()} cell, generated by: ros2 run open_amr_arm_cell reach_study '
                '--out config/taught_pattern.yaml\n# (cell frame; both cells identical; steps = cell_program.py). Do not '
                'edit by hand: change warehouse_layout.yaml arm_cell and re-run the study.\n')
        yaml.safe_dump(out, f, sort_keys=False)
    print(f'reach study: {n_ok}/{n} slot cycles feasible, cycle mean {out["summary"]["cycle_mean_s"]} s, '
          f'max {out["summary"]["cycle_max_s"]} s; robot at the bay {out["summary"]["deck_mean_s"]} s mean, '
          f'{out["summary"]["deck_max_s"]} s max -> {a.out}', flush=True)
    return _exit(0 if n_ok == n else 1)


def _exit(code):
    """End the process without running destructors: MoveItPy's (moveit_py 2.12.4) segfaults at teardown whatever the
    order (shutdown() first or not, before or after rclpy.shutdown()), after all results are written. Long-running
    nodes (the cell controller) never destroy it."""
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == '__main__':
    sys.exit(main())
