"""Motion executor of a palletizing cell: runs the taught pattern (config/taught_pattern.yaml, from the reach study)
through MoveIt 2 + Pilz on the cell's trajectory controller, with the vacuum over UR digital I/O
(io_and_status_controller: set_io tool output 16, io_states tool input 16 = part present). The same code drives the
simulated cell (open_amr_sim.py --arms) and a real UR30 (ur_robot_driver).

Phases (so the cell controller can pre-pick and never keeps a robot waiting for more than the handover):
  depalletizing  pick(pallet, slot)       home -> above the slot -> down -> vacuum -> part present -> lift -> above deck
                 place_on_deck()          down -> vacuum off -> part released -> up (clear of the deck) -> home
  palletizing    pick_from_deck()         home -> above the deck -> down -> vacuum -> part present -> up (clear)
                 place(pallet, slot)      above the slot -> down -> vacuum off -> released -> up -> home
  Every cycle starts and ends at home: exactly the moves the reach study planned and validated.
  home()
Every move is planned with Pilz against the planning scene the executor keeps (pallets with their current stock,
the docked AMR, the carried box attached to the tool), and validated for collisions before it runs.
Faults raise CellFault: plan rejected, execution failed (controller abort, protective stop), no part after vacuum on,
part lost while carrying, part still held after release.
"""
import os
import threading
import time

import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

from .cell_geometry import Cell, CellParams
from .reach_study import AMR, CONTACT, HOME, JOINTS, TRANSFER_MARGIN, box_object, dense_collision, tool_down

VACUUM_OUT, PART_PRESENT_IN = 16, 16


class CellFault(Exception):
    """A cell fault the station must report (FAULT) and a person / the controller must clear."""


def load_pattern(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('open_amr_arm_cell'), 'config', 'taught_pattern.yaml')
    return yaml.safe_load(open(path))


class VacuumIO:
    """The cell's tool I/O over ur_robot_driver's io_and_status_controller interface (sim: arm_bridge.py)."""

    def __init__(self, node, arm_id):
        from ur_msgs.msg import IOStates
        from ur_msgs.srv import SetIO
        self.SetIO = SetIO
        self.cli = node.create_client(SetIO, f'/{arm_id}/io_and_status_controller/set_io')
        self.lock, self.part_present, self.stamp = threading.Lock(), None, 0.0
        node.create_subscription(IOStates, f'/{arm_id}/io_and_status_controller/io_states', self.on_io, 10)

    def on_io(self, m):
        for d in m.digital_in_states:
            if d.pin == PART_PRESENT_IN:
                with self.lock:
                    self.part_present, self.stamp = bool(d.state), time.monotonic()

    def ready(self, timeout=30.0):
        return self.cli.wait_for_service(timeout_sec=timeout)

    def vacuum(self, on):
        req = self.SetIO.Request(fun=self.SetIO.Request.FUN_SET_DIGITAL_OUT, pin=VACUUM_OUT,
                                 state=float(self.SetIO.Request.STATE_ON if on else self.SetIO.Request.STATE_OFF))
        fut = self.cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.005)
        if not fut.done() or not fut.result().success:
            raise CellFault(f'set_io vacuum {"on" if on else "off"} failed')

    def wait_part(self, present, timeout):
        """Wait for the vacuum switch to read `present` (a fresh reading); False on timeout."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.lock:
                if self.part_present is present and self.stamp > t0:
                    return True
            time.sleep(0.01)
        return False

    def holding(self):
        with self.lock:
            return bool(self.part_present)


class CellMotion:
    def __init__(self, moveit, io, cell: Cell, pattern, vel_scale=None, acc_scale=None, logger=print):
        """vel_scale / acc_scale override the configured scaling (moveit_cpp.yaml ptp 0.8/0.5, lin 0.5/0.5) for
        every move; 1.0 = the arm's full speed (UR's joint velocity limits, UR MoveIt's acceleration limits)."""
        from moveit.planning import PlanRequestParameters
        self.moveit, self.io, self.cell, self.p = moveit, io, cell, cell.p
        self.pattern = pattern['pattern']
        self.arm = moveit.get_planning_component('arm')
        self.psm = moveit.get_planning_scene_monitor()
        self.ptp, self.lin = PlanRequestParameters(moveit, 'ptp'), PlanRequestParameters(moveit, 'lin')
        self.lin_place = PlanRequestParameters(moveit, 'lin_place')
        for prm in (self.ptp, self.lin):
            if vel_scale is not None:
                prm.max_velocity_scaling_factor = float(vel_scale)
            if acc_scale is not None:
                prm.max_acceleration_scaling_factor = float(acc_scale)
        self.log = logger
        self.times = {}                    # segment -> last duration (s)
        self.carrying = None               # 'pallet' / 'deck' source of the box on the tool, or None
        self.stock = [0, 0]
        self.amr = False
        self.deck_box = False
        self.before_release = None         # diagnostics hook: called with the place name just before vacuum off

    # ------------------------------------------------------------------ planning scene (mirrors the real cell)
    def _apply(self, objs):
        with self.psm.read_write() as scene:
            for co in objs:
                scene.apply_collision_object(co)

    def set_scene(self, stock=None, amr=None, deck_box=None):
        """Pallet stock (boxes per pallet), AMR docked or not, a box on its deck or not."""
        self.stock = list(stock) if stock is not None else self.stock
        self.amr = self.amr if amr is None else amr
        self.deck_box = self.deck_box if deck_box is None else deck_box
        objs = [box_object('floor', (6.0, 6.0, 0.02), (0.0, 0.0, -0.011))]
        for i in (0, 1):
            (cx, cy), yaw = self.cell.pallet_cell(i)
            ps = self.p.pallet_size
            objs.append(box_object(f'pallet_{i}', ps, (cx, cy, ps[2] / 2), yaw))
            for k in range(self.p.capacity):
                (x, y, z), byaw = self.cell.slot_cell(i, k)
                objs.append(box_object(f'box_{i}_{k}', self.p.box, (x, y, z - self.p.box[2] / 2), byaw)
                            if k < self.stock[i] else box_object(f'box_{i}_{k}', (), (), op=CollisionObject.REMOVE))
        objs.append(box_object('amr', AMR, (self.p.deck_distance, 0.0, AMR[2] / 2)) if self.amr else
                    box_object('amr', (), (), op=CollisionObject.REMOVE))
        (dx, dy, dz), _ = self.cell.deck_cell()
        objs.append(box_object('deck_box', self.p.box, (dx, dy, dz - self.p.box[2] / 2), 180.0) if self.deck_box else
                    box_object('deck_box', (), (), op=CollisionObject.REMOVE))
        self._apply(objs)

    def _attach(self, on, pad=0.0):
        aco = AttachedCollisionObject()
        aco.link_name = 'tcp'
        aco.touch_links = ['tcp', 'vacuum_gripper', 'wrist_3_link']
        b = self.p.box
        co = box_object('carried', (b[0] + 2 * pad, b[1] + 2 * pad, b[2] + pad), (0.0, 0.0, (b[2] + pad) / 2)) if on \
            else box_object('carried', (), (), op=CollisionObject.REMOVE)
        co.header.frame_id = 'tcp'
        aco.object = co
        with self.psm.read_write() as scene:
            scene.process_attached_collision_object(aco)
            if not on:
                scene.apply_collision_object(box_object('carried', (), (), op=CollisionObject.REMOVE))

    # ------------------------------------------------------------------ motion
    def _move(self, name, goal_q=None, goal_pose=None, lin=False, place=False):
        from moveit.core.robot_state import RobotState
        self.arm.set_start_state_to_current_state()
        if goal_q is not None:
            s = RobotState(self.moveit.get_robot_model())
            s.joint_positions = dict(zip(JOINTS, goal_q))
            s.update()
            self.arm.set_goal_state(robot_state=s)
        else:
            ps = PoseStamped()
            ps.header.frame_id = 'world'
            ps.pose = goal_pose
            self.arm.set_goal_state(pose_stamped_msg=ps, pose_link='tcp')
        t0 = time.monotonic()
        res = self.arm.plan(single_plan_parameters=self.lin_place if place else self.lin if lin else self.ptp)
        if not res:
            raise CellFault(f'plan rejected: {name}')
        # every trajectory is checked densely before it runs (MoveIt checks only its waypoints), PTP transfers with
        # the carried carton padded by TRANSFER_MARGIN
        if not lin and self.carrying:
            self._attach(True, pad=TRANSFER_MARGIN)
        hit = dense_collision(self.psm, res.trajectory.get_robot_trajectory_msg().joint_trajectory)
        if not lin and self.carrying:
            self._attach(True)
        if hit is not None:
            raise CellFault(f'plan rejected: {name} collides between waypoints')
        status = self.moveit.execute(res.trajectory, controllers=[])
        if not status:
            raise CellFault(f'execution failed: {name} ({getattr(status, "status", status)})')
        jt = res.trajectory.get_robot_trajectory_msg().joint_trajectory
        end = dict(zip(jt.joint_names, jt.points[-1].positions))
        self._settle([end[j] for j in JOINTS], name)
        self.times[name] = time.monotonic() - t0
        if self.carrying and not self.io.holding():
            raise CellFault(f'vacuum lost: part dropped during {name}')

    def _settle(self, target, name, tol=0.001, timeout=2.0):
        """Wait until the measured joints have reached the move's end (within `tol` rad): the next plan must start
        from where the arm really is, not from a state a few ms stale."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.psm.read_only() as scene:
                q = scene.current_state.joint_positions
            if max(abs(q[j] - v) for j, v in zip(JOINTS, target)) <= tol:
                return
            time.sleep(0.01)
        raise CellFault(f'arm did not settle after {name} (off by > {tol} rad for {timeout} s)')

    def _grip(self, where):
        self.io.vacuum(True)
        if not self.io.wait_part(True, 1.5):
            self.io.vacuum(False)
            raise CellFault(f'no part at {where}: vacuum switch stayed off')
        self._attach(True)
        self.carrying = where

    def _release(self, where):
        if self.before_release:
            self.before_release(where)
        self.io.vacuum(False)
        if not self.io.wait_part(False, 1.5):
            raise CellFault(f'part still held after release at {where}')
        self._attach(False)
        self.carrying = None

    def _taught(self, mode, pallet, slot):
        key = f'{mode}/{pallet}/{slot}'
        if key not in self.pattern:
            raise CellFault(f'no taught motion for {key}')
        return self.pattern[key]

    def _geometry(self, pallet, slot, t):
        (sx, sy, sz), _ = self.cell.slot_cell(pallet, slot)
        (dx, dy, dz), _ = self.cell.deck_cell()
        tt = t['taught']
        via = t['via']
        zt = self.cell.transit_height()
        return sx, sy, sz, dx, dy, dz, tt, via, zt

    def pick(self, pallet, slot):
        """Depalletizing, part 1: take the top box of `pallet` (slot = its count - 1) and hold it above the deck."""
        t = self._taught('depalletize', pallet, slot)
        sx, sy, sz, dx, dy, dz, tt, via, zt = self._geometry(pallet, slot, t)
        a = self.p.approach
        self._move('to slot', goal_q=tt['slot_above'])
        self._move('down to slot', goal_pose=tool_down(sx, sy, sz + CONTACT, tt['grip_yaw']), lin=True)
        self.set_scene(stock=[n - 1 if i == pallet else n for i, n in enumerate(self.stock)])   # the box is ours now
        self._grip(f'pallet {pallet} slot {slot}')
        self._move('lift', goal_pose=tool_down(sx, sy, zt if via else self.cell.lift_height(sz), tt['grip_yaw']),
                   lin=True)
        self._move('to deck', goal_q=tt['deck_above'])
        self._deck_yaw, self._deck_carry = tt['deck_yaw'], zt if via else dz + a

    def place_on_deck(self):
        """Depalletizing, part 2 (the AMR is docked and stopped): put the held box on the deck and clear it."""
        (dx, dy, dz), _ = self.cell.deck_cell()
        a = self.p.approach
        self._move('down to deck', goal_pose=tool_down(dx, dy, dz + CONTACT, self._deck_yaw), lin=True, place=True)
        self._release('deck')
        self.set_scene(deck_box=True)
        self._move('retreat', goal_pose=tool_down(dx, dy, dz + a, self._deck_yaw), lin=True)
        self.home()                    # every taught cycle starts and ends at home (the moves the reach study validated)

    def pick_from_deck(self, pallet, slot):
        """Palletizing, part 1 (the AMR is docked and stopped): take the box off the deck, up clear of it."""
        t = self._taught('palletize', pallet, slot)
        sx, sy, sz, dx, dy, dz, tt, via, zt = self._geometry(pallet, slot, t)
        a = self.p.approach
        self._move('to deck', goal_q=tt['deck_above'])
        self._move('down to deck', goal_pose=tool_down(dx, dy, dz + CONTACT, tt['deck_yaw']), lin=True)
        self.set_scene(deck_box=False)
        self._grip('deck')
        self._move('up from deck', goal_pose=tool_down(dx, dy, zt if via else dz + a, tt['deck_yaw']), lin=True)
        self._pal = (pallet, slot)

    def place(self, pallet, slot):
        """Palletizing, part 2: put the held box into `slot` of `pallet` (slot = its count)."""
        t = self._taught('palletize', pallet, slot)
        sx, sy, sz, dx, dy, dz, tt, via, zt = self._geometry(pallet, slot, t)
        a = self.p.approach
        self._move('to slot', goal_q=tt['slot_above'])
        self._move('down to slot', goal_pose=tool_down(sx, sy, sz + CONTACT, tt['grip_yaw']), lin=True, place=True)
        self._release(f'pallet {pallet} slot {slot}')
        self.set_scene(stock=[n + 1 if i == pallet else n for i, n in enumerate(self.stock)])
        self._move('retreat', goal_pose=tool_down(sx, sy, zt if via else sz + a, tt['grip_yaw']), lin=True)
        self.home()

    def home(self):
        self._move('home', goal_q=HOME)

    def recover(self):
        """After a fault: vacuum off, detach, back to the scene the controller will set again."""
        try:
            self.io.vacuum(False)
        except CellFault:
            pass
        self._attach(False)
        self.carrying = None


def make_cell(layout_path=None):
    if layout_path is None:
        layout_path = os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')), 'sim',
                                   'configs', 'warehouse_layout.yaml')
    return Cell((0.0, 0.0, 0.0), CellParams.from_layout(yaml.safe_load(open(layout_path))))


__all__ = ['CellFault', 'CellMotion', 'VacuumIO', 'load_pattern', 'make_cell']
