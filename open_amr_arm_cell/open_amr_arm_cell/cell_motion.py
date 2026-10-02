"""Motion executor of a palletizing cell: runs the cell's motion program (cell_program.py, the steps the reach study
validated, with the taught points of config/taught_pattern.yaml) through MoveIt 2 + Pilz on the cell's trajectory
controller, with the vacuum over UR digital I/O (io_and_status_controller: set_io tool output 16, io_states tool input
16 = part present). The same code drives the simulated cell (open_amr_sim.py --arms) and a real UR30 (ur_robot_driver).

A cycle runs as one program with two markers (cell_program.py): at DOCK the executor calls `on_dock` (the cell
controller waits there for the bay interlock), at UNDOCK `on_undock` (the robot may leave). `start()` + `run(until=DOCK)`
pre-picks a carton and holds it over the deck zone; a later `run()` finishes the cycle.
Every move is planned with Pilz against the planning scene the executor keeps (pallets with their current stock, the
keep-out volume over the bay or the docked robot, the carried carton attached to the tool), checked densely for
collisions before it runs, and watched while it runs by `guard(in_zone)`: a non-empty answer (protective stop, robot
moved under the arm) stops the trajectory at once.
Faults raise CellFault (plan rejected, collision between waypoints, execution failed or stopped, no part after vacuum
on, part lost while carrying, part still held after release, carton not where expected); `carton` tells where the
carton of the cycle is then.
"""
import math
import os
import threading
import time

import yaml
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject

from . import cell_program as prog
from .carton_locator import LocateFault, NominalLocator
from .cell_geometry import Cell, CellParams
from .reach_study import JOINTS, TRANSFER_MARGIN, box_object, dense_collision, tool_down, zone_objects

VACUUM_OUT, PART_PRESENT_IN = 16, 16
# where the carton of the current cycle is
C_NONE, C_PALLET, C_TOOL, C_DECK, C_LOST = 'none', 'pallet', 'tool', 'deck', 'lost'
VACUUM_LOST = 'vacuum switch lost the part'


class CellFault(Exception):
    """A cell fault the station must report (FAULT) and a person must clear (cell reset)."""


def load_pattern(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('open_amr_arm_cell'), 'config', 'taught_pattern.yaml')
    return yaml.safe_load(open(path))


class VacuumIO:
    """The cell's tool I/O over ur_robot_driver's io_and_status_controller interface (sim: arm_bridge.py), and the
    payload (set_payload: a real UR's dynamics and force limits must know a carton hangs on the tool)."""

    def __init__(self, node, arm_id, echo=False):
        """echo: part present = the vacuum output's state (URSim, which has no vacuum switch to simulate)."""
        from ur_msgs.msg import IOStates
        self.echo = echo
        from ur_msgs.srv import SetIO, SetPayload
        self.SetIO, self.SetPayload = SetIO, SetPayload
        self.cli = node.create_client(SetIO, f'/{arm_id}/io_and_status_controller/set_io')
        self.payload_cli = node.create_client(SetPayload, f'/{arm_id}/io_and_status_controller/set_payload')
        self.lock, self.part_present, self.stamp = threading.Lock(), None, 0.0
        node.create_subscription(IOStates, f'/{arm_id}/io_and_status_controller/io_states', self.on_io, 10)

    def on_io(self, m):
        for d in (m.digital_out_states if self.echo else m.digital_in_states):
            if d.pin == (VACUUM_OUT if self.echo else PART_PRESENT_IN):
                with self.lock:
                    self.part_present, self.stamp = bool(d.state), time.monotonic()

    def ready(self, timeout=30.0):
        return self.cli.wait_for_service(timeout_sec=timeout)

    def _call(self, cli, req, what):
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.005)
        if not fut.done() or not fut.result().success:
            raise CellFault(f'{what} failed')

    def vacuum(self, on):
        self._call(self.cli, self.SetIO.Request(fun=self.SetIO.Request.FUN_SET_DIGITAL_OUT, pin=VACUUM_OUT, state=float(
            self.SetIO.Request.STATE_ON if on else self.SetIO.Request.STATE_OFF)), f'set_io vacuum {"on" if on else "off"}')

    def payload(self, mass, cog_z):
        """Tool payload (kg, centre of gravity along the tool axis from the flange, m); skipped without set_payload."""
        if not self.payload_cli.service_is_ready():
            return
        req = self.SetPayload.Request(mass=float(mass))
        req.center_of_gravity.z = float(cog_z)
        self._call(self.payload_cli, req, 'set_payload')

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
    def __init__(self, moveit, io, cell: Cell, pattern, locator=None, vel_scale=None, acc_scale=None, logger=print,
                 gripper_mass=2.0, carton_mass=8.0):
        """vel_scale / acc_scale override the configured scaling (moveit_cpp.yaml: the production speeds) for every
        move; 1.0 = the arm's full speed (UR's joint velocity limits, UR MoveIt's acceleration limits)."""
        from moveit.planning import PlanRequestParameters
        self.moveit, self.io, self.cell, self.p = moveit, io, cell, cell.p
        self.pattern = pattern['pattern']
        self.locator = locator or NominalLocator()
        self.arm = moveit.get_planning_component('arm')
        self.psm = moveit.get_planning_scene_monitor()
        self.tem = moveit.get_trajectory_execution_manager()
        self.ptp, self.lin = PlanRequestParameters(moveit, 'ptp'), PlanRequestParameters(moveit, 'lin')
        self.lin_place = PlanRequestParameters(moveit, 'lin_place')
        for prm in (self.ptp, self.lin):
            if vel_scale is not None:
                prm.max_velocity_scaling_factor = float(vel_scale)
            if acc_scale is not None:
                prm.max_acceleration_scaling_factor = float(acc_scale)
        self.log = logger
        self.gripper_mass, self.carton_mass = gripper_mass, carton_mass
        self.guard = lambda in_zone: None        # -> reason to stop now, or None (set by the cell controller)
        self.times = {}                          # step -> last duration (s)
        self.step_log = []                       # (step, plan s, execute s, planned s, settle s) of the last run()
        self.carrying = None                     # where the carton on the tool came from, or None
        self.carton = C_NONE                     # where the carton of the current cycle is
        self.stock, self.docked, self.deck_box = [0, 0], False, False
        self.seen = {}                           # (pallet, slot) -> detected carton centre (x, y, z, yaw): the scene
                                                 # uses these instead of the pattern (cleared with every new stock)
        self.cycle = None                        # (depal, pallet, slot, taught entry) of the program being run
        self.steps, self.pc = [], 0              # its steps and the next one to run
        self.before_release = None               # diagnostics hook: called with the place name just before vacuum off

    # ------------------------------------------------------------------ planning scene (mirrors the real cell)
    def _apply(self, objs):
        with self.psm.read_write() as scene:
            for co in objs:
                scene.apply_collision_object(co)

    def set_scene(self, stock=None, docked=None, deck_box=None, fresh=False):
        """Pallet stock (cartons per pallet), robot docked (its outline, a carton on its deck if deck_box) or not (the
        keep-out volume over the bay). fresh: a new transfer — forget the detected carton poses of the last one."""
        if fresh:
            self.seen = {}
        self.stock = list(stock) if stock is not None else self.stock
        self.docked = self.docked if docked is None else docked
        self.deck_box = self.deck_box if deck_box is None else deck_box
        objs = [box_object('floor', (6.0, 6.0, 0.02), (0.0, 0.0, -0.011))]
        for i in (0, 1):
            (cx, cy), yaw = self.cell.pallet_cell(i)
            ps = self.p.pallet_size
            objs.append(box_object(f'pallet_{i}', ps, (cx, cy, ps[2] / 2), yaw))
            for k in range(self.p.capacity):
                (x, y, z), byaw = self.cell.slot_cell(i, k)
                cx, cy, cz, cyaw = self.seen.get((i, k), (x, y, z - self.p.box[2] / 2, byaw))
                objs.append(box_object(f'box_{i}_{k}', self.p.box, (cx, cy, cz), cyaw)
                            if k < self.stock[i] else box_object(f'box_{i}_{k}', (), (), op=CollisionObject.REMOVE))
        self._apply(objs + zone_objects(self.cell, self.docked, self.deck_box))

    def clear_bay(self):
        """Neither robot nor keep-out volume (fault recovery, after a person has checked the bay)."""
        self._apply([box_object(n, (), (), op=CollisionObject.REMOVE) for n in ('amr', 'deck_box', 'deck_zone',
                                                                                  'deck_rail_0', 'deck_rail_1')])

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
    def _move(self, name, goal_q=None, goal_pose=None, lin=False, place=False, in_zone=False):
        from moveit.core.robot_state import RobotState
        why = self.guard(in_zone)
        if why:
            raise CellFault(f'{why} (before {name})')
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
        t_plan = time.monotonic()
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
        stopped, done = [], threading.Event()

        def watch():                             # stops the arm the moment the guard (or the vacuum switch) says so
            while not done.is_set():
                r = self.guard(in_zone) or (self.carrying and not self.io.holding() and VACUUM_LOST)
                if r:
                    stopped.append(r)
                    self.tem.stop_execution()
                    return
                done.wait(0.02)
        w = threading.Thread(target=watch, daemon=True)
        w.start()
        try:
            status = self.moveit.execute(res.trajectory, controllers=[])
        finally:
            done.set()
            w.join()
        if stopped and stopped[0] == VACUUM_LOST:
            self._dropped(name)
        if stopped:
            raise CellFault(f'stopped during {name}: {stopped[0]}')
        if not status:
            # the trajectory controller can abort a few ms before the driver's safety-mode message arrives: a safety
            # stop must be reported as one, not as a failed execution
            t1 = time.monotonic()
            while time.monotonic() - t1 < 0.5:
                why = self.guard(in_zone)
                if why:
                    raise CellFault(f'stopped during {name}: {why}')
                time.sleep(0.02)
            raise CellFault(f'execution failed: {name} ({getattr(status, "status", status)})')
        t_exec = time.monotonic()
        jt = res.trajectory.get_robot_trajectory_msg().joint_trajectory
        end = dict(zip(jt.joint_names, jt.points[-1].positions))
        self._settle([end[j] for j in JOINTS], name)
        t_end = time.monotonic()
        self.times[name] = t_end - t0
        d = jt.points[-1].time_from_start
        self.step_log.append((name, round(t_plan - t0, 2), round(t_exec - t_plan, 2), round(d.sec + d.nanosec * 1e-9, 2),
                              round(t_end - t_exec, 2)))
        if self.carrying and not self.io.holding():
            self._dropped(name)

    def _dropped(self, name):
        self.carton, self.carrying = C_LOST, None
        self._attach(False)
        raise CellFault(f'vacuum lost: carton dropped during {name}')

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
            raise CellFault(f'no carton at {where}: vacuum switch stayed off')
        self._attach(True)
        self.carrying, self.carton = where, C_TOOL
        m, g = self.carton_mass, self.gripper_mass       # carton centre half a carton below the pad face
        self.io.payload(m + g, (g * 0.5 * self.p.tool_length + m * (self.p.tool_length + self.p.box[2] / 2)) / (m + g))

    def _release(self, where):
        if self.before_release:
            self.before_release(where)
        self.io.vacuum(False)
        if not self.io.wait_part(False, 1.5):
            raise CellFault(f'carton still held after release at {where}')
        self._attach(False)
        self.carrying = None
        self.io.payload(self.gripper_mass, 0.5 * self.p.tool_length)

    # ------------------------------------------------------------------ program
    def taught(self, depal, pallet, slot):
        key = f'{"depalletize" if depal else "palletize"}/{pallet}/{slot}'
        if key not in self.pattern:
            raise CellFault(f'no taught motion for {key}')
        e = self.pattern[key]
        return dict(e['taught'], via=e['via'])

    def start(self, depal, pallet, slot):
        """Load one cycle's program (the next run() starts it)."""
        self.cycle = (depal, pallet, slot, self.taught(depal, pallet, slot))
        self.steps, self.pc = prog.program(self.cell, depal, pallet, slot, self.cycle[3]), 0
        self.carton = C_PALLET if depal else C_DECK

    @property
    def at_dock(self):
        """A program is loaded and waits at its DOCK marker (a depalletizing pre-pick holds its carton there)."""
        return bool(self.steps) and self.pc < len(self.steps) and self.steps[self.pc].kind == prog.DOCK

    def holding_for(self, pallet, slot):
        return self.at_dock and self.cycle[0] and tuple(self.cycle[1:3]) == (pallet, slot) and self.carrying is not None

    def _locate(self, step):
        """Correct the program by where the carton really is (the steps from here on are rebuilt with the offset)."""
        depal, pallet, slot, t = self.cycle
        if step.locate[0] == 'pallet':
            (x, y, z), yaw = self.cell.slot_cell(pallet, slot)
            where = f'pallet {pallet} slot {slot}'
        else:
            (x, y, z), _ = self.cell.deck_cell()
            yaw, where = t['deck_yaw'], 'the deck'
        try:
            off = self.locator.locate(step.locate, (x, y, z - self.p.box[2] / 2, yaw))
        except LocateFault as e:
            raise CellFault(f'{e} at {where}')
        if step.locate[0] == 'pallet':
            self._scene_from_detections(pallet)
        if any(abs(v) > 1e-4 for v in off):
            self.log(f'carton at {where} located {off[0] * 1000:+.0f} / {off[1] * 1000:+.0f} mm, {off[2]:+.1f} deg '
                     'off the pattern: grip corrected')
        self.steps = prog.program(self.cell, depal, pallet, slot, t, offset=off)

    def _scene_from_detections(self, pallet):
        """Plan against what the camera sees: the cartons on `pallet` go into the scene at their detected poses (a
        stack nudged a few mm — by a stop, a dropped carton — made a lift 'collide' with neighbours drawn at their
        pattern slots, fleet run 2026-10-02)."""
        dets = self.locator.detected()
        if not dets:
            return
        changed = False
        for k in range(self.stock[pallet]):
            (x, y, z), yaw = self.cell.slot_cell(pallet, k)
            nz = z - self.p.box[2] / 2
            near = min(dets, key=lambda d: (d[0] - x) ** 2 + (d[1] - y) ** 2 + (d[2] - nz) ** 2)
            if math.dist(near[:3], (x, y, nz)) < 0.06:
                dyaw = (near[3] - yaw + 45.0) % 90.0 - 45.0
                self.seen[(pallet, k)] = (near[0], near[1], near[2], yaw + dyaw)
                changed = True
        if changed:
            self.set_scene()

    def run(self, until=None, on_dock=None, on_undock=None, on_step=None):
        """Run the loaded program from where it stands; stop before a step of kind `until` (e.g. prog.DOCK: pre-pick).
        on_dock() blocks until the interlock is made; on_undock() is told the arm is clear of the deck zone;
        on_step(name) before every step."""
        depal, pallet, slot, _ = self.cycle
        self.step_log = []
        while self.pc < len(self.steps):
            st = self.steps[self.pc]
            if until is not None and st.kind == until:
                return
            if on_step:
                on_step(st.name)
            if st.locate:
                self._locate(st)
                st = self.steps[self.pc]
            if st.kind == prog.DOCK:
                if on_dock:
                    on_dock()
                self.set_scene(docked=True, deck_box=not depal)
            elif st.kind == prog.UNDOCK:
                self.set_scene(docked=False, deck_box=False)       # the robot leaves (with the carton, depalletizing)
                if on_undock:
                    on_undock()
            elif st.kind == prog.GRIP:
                if st.where == 'deck':
                    self.set_scene(deck_box=False)
                else:
                    self.set_scene(stock=[n - 1 if i == pallet else n for i, n in enumerate(self.stock)])
                self._grip(st.where)
            elif st.kind == prog.RELEASE:
                self._release(st.where)
                if st.where == 'deck':
                    self.carton = C_DECK
                    self.set_scene(deck_box=True)
                else:
                    self.carton = C_PALLET
                    self.set_scene(stock=[n + 1 if i == pallet else n for i, n in enumerate(self.stock)])
            elif st.kind == prog.PTP:
                self._move(st.name, goal_q=st.q, in_zone=self.docked)
            else:
                self._move(st.name, goal_pose=tool_down(*st.pose), lin=True, place=st.place, in_zone=self.docked)
            self.pc += 1
        self.abandon()

    def home(self):
        self._move('home', goal_q=prog.HOME, in_zone=self.docked)

    def abandon(self):
        """Forget the loaded program."""
        self.steps, self.pc, self.cycle = [], 0, None

    def recover(self, bay_clear=True):
        """After a fault, once a person has cleared the cell (no carton on the tool): vacuum off, nothing attached,
        program dropped, home. bay_clear: no robot at the bay — plan without the keep-out volume (the arm may have
        stopped inside it), which is back in the scene once the arm is home."""
        if self.io.holding():
            raise CellFault('a carton is still on the gripper: take it off (release) before the reset')
        self.io.vacuum(False)
        self._attach(False)
        self.carrying, self.carton = None, C_NONE
        self.abandon()
        if bay_clear:
            self.docked = False
            self.clear_bay()
        self.home()
        self.set_scene(docked=self.docked)


def make_cell(layout_path=None, arm_id=None):
    """The cell in its own frame (arm_id None) or placed in the warehouse (the arm's pose from the layout)."""
    if layout_path is None:
        layout_path = os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')), 'sim',
                                   'configs', 'warehouse_layout.yaml')
    layout = yaml.safe_load(open(layout_path))
    pose = (0.0, 0.0, 0.0) if arm_id is None else \
        tuple(next(layout[d]['arm']['pose'] for d in ('receiving', 'outbound') if layout[d]['arm']['id'] == arm_id))
    return Cell(pose, CellParams.from_layout(layout))


__all__ = ['CellFault', 'CellMotion', 'VacuumIO', 'load_pattern', 'make_cell']
