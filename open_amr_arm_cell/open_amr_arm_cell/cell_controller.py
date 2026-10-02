"""cell_controller: one per palletizing cell (namespace /<arm_id>), the same node for the simulated and the real UR30.

Serves ArmTransfer at /<arm_id>/cell/transfer for the cell's station_agent (which owns stock and pattern bookkeeping)
and runs each transfer with the cell executor (cell_motion.py: MoveIt 2 + Pilz, vacuum over UR I/O, the motion program
the reach study validated).

Interlocks (software; a real cell adds safety-rated hardware, see below):
  - deck zone: the arm enters the keep-out volume over the bay only while the transfer's robot reports, in its own
    heartbeat (/swarm/state), AT_STATION, docked to the bay's marker (its docking server: millimetres), stopped, and
    at this bay (pose within bay_tol: the localization estimate) — and no other robot reports a pose in the zone. Until then the volume is a collision object, so no planned motion can enter it.
  - while the arm is in the zone, that robot moving (speed, or pose drifting from where it docked, or leaving
    AT_STATION) or another robot entering stops the arm at once (trajectory execution stopped) and faults the cell.
    A heartbeat that goes stale does not stop the arm: a silent robot is a stopped robot (Phase 4a), and the real
    guarantee is the bay's safety sensor.
  - the arm's safety mode (io_and_status_controller/safety_mode: protective / safeguard / emergency stop) stops any
    move and the cell reports STOPPED; a transfer that was running ends in a fault.
  - the robot is released (feedback 'deck_clear', the station then reports the task served) only after the vacuum
    switch confirmed the release and the arm has left the zone.
Faults latch (ArmCellState FAULT): a person clears the cell and calls /<arm_id>/cell/reset (std_srvs/Trigger), which
refuses while a carton is on the gripper (/<arm_id>/cell/release_carton turns the vacuum off with someone holding it) or
the arm is safety-stopped, and otherwise lifts the gripper out of the deck zone and sends the arm home.
Heartbeat: ArmCellState on /<arm_id>/cell/state, 5 Hz.
Carton locator: `detections_topic` (geometry_msgs/PoseArray of carton centres, world frame: the simulator's ground
truth, or a 3D camera's detection on a real cell); empty = trust the pattern.

Deployment (not software): a safety-rated presence sensor / light curtain over the bay wired to the UR's safety I/O
(safeguard stop when someone or something unexpected enters; ISO 10218-2:2025), UR safety planes keeping the arm out
of the bay volume in reduced mode, AMR safety per ISO 3691-4.

    ros2 run open_amr_arm_cell cell_controller --ros-args -r __ns:=/arm_receiving -p arm_id:=arm_receiving \
        -p use_sim_time:=true [-p detections_topic:=/arm_receiving/test_boxes]
"""
import math
import os
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from open_amr_msgs.action import ArmTransfer
from open_amr_msgs.msg import ArmCellState, RobotState

from . import cell_program as prog
from .carton_locator import DetectionLocator
from .cell_motion import C_DECK, C_LOST, C_NONE, C_PALLET, C_TOOL, CellFault, CellMotion, VacuumIO, load_pattern, \
    make_cell
from .reach_study import moveit_configs, moveit_py, tool_down

STATE_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CARTON = {C_NONE: ArmTransfer.Result.CARTON_NONE, C_PALLET: ArmTransfer.Result.CARTON_PALLET,
          C_TOOL: ArmTransfer.Result.CARTON_TOOL, C_DECK: ArmTransfer.Result.CARTON_DECK,
          C_LOST: ArmTransfer.Result.CARTON_LOST}
SAFETY_NAMES = {1: 'normal', 2: 'reduced', 3: 'protective stop', 4: 'recovery', 5: 'safeguard stop',
                6: 'system emergency stop', 7: 'robot emergency stop', 8: 'violation', 9: 'fault',
                12: 'automatic mode safeguard stop', 13: 'three-position enabling stop'}


class Cancelled(Exception):
    pass


class CellController(Node):
    def __init__(self):
        super().__init__('cell_controller')
        dp = self.declare_parameter
        self.arm_id = dp('arm_id', 'arm_receiving').value
        layout = dp('layout', '').value or None
        # the robot's own docking (lidar marker, Nav2 docking server) is what positions it to millimetres; its reported
        # pose is a localization estimate (~0.15 m) and only tells which bay it is at
        self.require_docked = dp('require_docked', True).value
        self.bay_tol = dp('bay_tol_m', 0.30).value               # reported pose vs bay pose: at this bay
        self.bay_tol_yaw = dp('bay_tol_deg', 10.0).value
        self.stop_speed = dp('stop_speed', 0.02).value           # m/s: counts as stopped (interlock made)
        self.move_speed = dp('move_speed', 0.05).value           # m/s: counts as moving (interlock broken)
        self.move_dist = dp('move_dist_m', 0.05).value           # drift from the docked pose that breaks it
        self.hb_timeout = dp('heartbeat_timeout_s', 1.0).value   # a heartbeat older than this can't make the interlock
        self.cell = make_cell(layout, self.arm_id)               # placed in the warehouse (bay pose, locator frame)
        self.local = make_cell(layout)                           # the planner's frame
        bx, by, byaw = self.cell.bay_pose()
        self.bay = (bx, by, math.radians(byaw))
        zx, zy, sx, sy, _ = self.cell.deck_zone()
        self.zone_world = (self.cell.to_world(zx, zy)[:2], sx, sy, math.radians(self.cell.arm_pose[2]))

        self.lock = threading.Lock()
        self.robots = {}                  # robot_id -> (RobotState, receive monotonic time)
        self.safety_mode = 1
        self.state, self.phase, self.fault, self.cycles, self.faults = ArmCellState.STARTING, '', '', 0, 0
        self.busy = False
        self.zone_robot = None            # (robot_id, (x, y) where it docked) while the arm may be in the zone

        cfg = moveit_configs(self.local.p, hardware='topic', execution=True).to_dict()
        cfg['use_sim_time'] = self.get_parameter('use_sim_time').value
        self.moveit = moveit_py('cell_controller_moveit', cfg, name_space=self.arm_id)
        self.io = VacuumIO(self, self.arm_id)
        topic = dp('detections_topic', '').value
        lim = {'pallet': (dp('locate_pallet_mm', 30.0).value / 1000, dp('locate_pallet_deg', 5.0).value),
               'deck': (dp('locate_deck_mm', 100.0).value / 1000, dp('locate_deck_deg', 10.0).value)}
        locator = DetectionLocator(self, topic, self.cell, lim) if topic else None
        self.motion = CellMotion(self.moveit, self.io, self.local, load_pattern(dp('pattern', '').value or None),
                                 locator=locator, logger=lambda s: self.get_logger().info(f'{self.arm_id}: {s}'),
                                 gripper_mass=dp('gripper_mass', 2.0).value, carton_mass=dp('carton_mass', 8.0).value)
        self.motion.guard = self.guard

        from ur_dashboard_msgs.msg import SafetyMode
        cb = MutuallyExclusiveCallbackGroup()
        self.create_subscription(RobotState, '/swarm/state', self.on_robot, STATE_QOS, callback_group=cb)
        self.create_subscription(SafetyMode, f'/{self.arm_id}/io_and_status_controller/safety_mode',
                                 self.on_safety, LATCHED, callback_group=cb)
        self.pub = self.create_publisher(ArmCellState, f'/{self.arm_id}/cell/state', 10)
        self.create_timer(0.2, self.publish, callback_group=cb)
        work = ReentrantCallbackGroup()
        self.server = ActionServer(self, ArmTransfer, f'/{self.arm_id}/cell/transfer', self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=work)
        self.create_service(Trigger, f'/{self.arm_id}/cell/reset', self.on_reset, callback_group=work)
        self.create_service(Trigger, f'/{self.arm_id}/cell/release_carton', self.on_release, callback_group=work)
        threading.Thread(target=self.startup, daemon=True).start()

    def log(self, s, warn=False):
        # one call site per severity: rclpy refuses a call site whose severity changes between calls
        if warn:
            self.get_logger().warn(f'{self.arm_id}: {s}')
        else:
            self.get_logger().info(f'{self.arm_id}: {s}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def startup(self):
        if not self.io.ready(120.0):
            self.set_fault('no set_io service (arm driver / io_and_status_controller down)')
            return
        time.sleep(1.0)
        for attempt in range(5):
            try:
                self.motion.set_scene(stock=[0, 0], docked=False)
                if self.io.holding():
                    raise CellFault('a carton is on the gripper at start-up: take it off and reset')
                self.motion.home()
                break
            except CellFault as e:
                # right after start-up the measured joints can still be settling (MoveIt then refuses a trajectory
                # whose start deviates > 0.01 rad from them, seen 2026-10-02): try again before calling it a fault
                if 'carton' in str(e) or attempt == 4:
                    self.set_fault(f'start-up: {e}')
                    return
                self.log(f'start-up home: {e}; again in 2 s')
                time.sleep(2.0)
        self.state = ArmCellState.READY
        self.log('ready (home, deck zone guarded)')

    # ------------------------------------------------------------------ inputs
    def on_robot(self, m):
        with self.lock:
            self.robots[m.robot_id] = (m, time.monotonic())

    def on_safety(self, m):
        if m.mode != self.safety_mode:
            self.log(f'arm safety mode: {SAFETY_NAMES.get(m.mode, m.mode)}', warn=m.mode not in (1, 2))
        self.safety_mode = m.mode

    def safety_stopped(self):
        return self.safety_mode not in (1, 2)

    # ------------------------------------------------------------------ interlock
    def in_zone(self, x, y):
        (cx, cy), sx, sy, yaw = self.zone_world
        c, s = math.cos(yaw), math.sin(yaw)
        u, v = c * (x - cx) + s * (y - cy), -s * (x - cx) + c * (y - cy)
        return abs(u) <= sx / 2 and abs(v) <= sy / 2

    def interlock(self, rid):
        """None if robot `rid` is docked and stopped at the bay and nobody else is in the zone, else why not."""
        t = time.monotonic()
        with self.lock:
            robots = dict(self.robots)
        if rid not in robots or t - robots[rid][1] > self.hb_timeout:
            return f'no heartbeat from {rid}'
        m = robots[rid][0]
        if m.status != RobotState.AT_STATION:
            return f'{rid} not at a station (status {m.status})'
        if self.require_docked and not m.docked:
            return f'{rid} not docked to the bay marker'
        if m.speed > self.stop_speed:
            return f'{rid} moving ({m.speed:.2f} m/s)'
        d = math.hypot(m.pose.x - self.bay[0], m.pose.y - self.bay[1])
        dyaw = abs((m.pose.theta - self.bay[2] + math.pi) % (2 * math.pi) - math.pi)
        if d > self.bay_tol or math.degrees(dyaw) > self.bay_tol_yaw:
            return f'{rid} not docked ({d:.2f} m, {math.degrees(dyaw):.0f} deg from the bay pose)'
        other = self.intruder(robots, rid, t)
        return f'{other} in the deck zone' if other else None

    def intruder(self, robots, rid, t):
        return next((r for r, (m, rx) in sorted(robots.items())
                     if r != rid and t - rx < self.hb_timeout and self.in_zone(m.pose.x, m.pose.y)), None)

    def guard(self, in_zone):
        """Reason to stop the arm now (cell_motion calls it before and during every move), or None."""
        if self.safety_stopped():
            return f'arm {SAFETY_NAMES.get(self.safety_mode, self.safety_mode)}'
        if not in_zone or self.zone_robot is None:
            return None
        rid, (x0, y0) = self.zone_robot
        t = time.monotonic()
        with self.lock:
            robots = dict(self.robots)
        m, rx = robots.get(rid, (None, 0.0))
        if m is not None and t - rx <= self.hb_timeout:      # stale: a silent robot is a stopped robot
            if m.status != RobotState.AT_STATION or (self.require_docked and not m.docked):
                return f'interlock: {rid} left the station / undocked under the arm (status {m.status})'
            if m.speed > self.move_speed or math.hypot(m.pose.x - x0, m.pose.y - y0) > self.move_dist:
                return f'interlock: {rid} moved under the arm ({m.speed:.2f} m/s, ' \
                       f'{math.hypot(m.pose.x - x0, m.pose.y - y0) * 1000:.0f} mm)'
        other = self.intruder(robots, rid, t)
        return f'interlock: {other} entered the deck zone' if other else None

    # ------------------------------------------------------------------ transfers
    def on_goal(self, goal):
        why = 'busy' if self.busy else self.fault and f'faulted: {self.fault}' or \
            (self.state == ArmCellState.STARTING and 'starting') or \
            (self.safety_stopped() and f'arm {SAFETY_NAMES.get(self.safety_mode)}') or None
        if goal.kind not in (ArmTransfer.Goal.PREPARE, ArmTransfer.Goal.DEPALLETIZE, ArmTransfer.Goal.PALLETIZE):
            why = f'unknown kind {goal.kind}'
        if why:
            self.log(f'transfer {goal.task_id or "(prepare)"} rejected: {why}', warn=True)
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def feedback(self, gh, phase):
        self.phase = phase
        gh.publish_feedback(ArmTransfer.Feedback(phase=phase))

    def execute(self, gh):
        g = gh.request
        kind = {0: 'prepare', 1: 'depalletize', 2: 'palletize'}[g.kind]
        t0, marks = self.now(), {}
        res = ArmTransfer.Result()
        self.state = ArmCellState.BUSY
        self.log(f'{kind} pallet {g.pallet} slot {g.slot}' + (f' for {g.robot_id} ({g.task_id})' if g.robot_id else ''))

        def on_dock():
            marks['wait'] = self.now()
            self.feedback(gh, 'waiting_interlock')
            last = None
            while True:
                if gh.is_cancel_requested:
                    raise Cancelled()
                why = self.interlock(g.robot_id)
                if why is None:
                    break
                if why != last:
                    self.log(f'waiting for the interlock: {why}')
                    last = why
                time.sleep(0.05)
            with self.lock:
                m = self.robots[g.robot_id][0]
            self.zone_robot = (g.robot_id, (m.pose.x, m.pose.y))
            marks['dock'] = self.now()
            self.feedback(gh, 'in_deck_zone')

        def on_undock():
            self.zone_robot = None
            marks['clear'] = self.now()
            self.feedback(gh, 'deck_clear')

        steps = {'to deck hold': 'holding', 'to deck wait': 'holding', 'to slot': 'picking' if g.kind != 2 else 'placing',
                 'home': 'home'}
        try:
            m = self.motion
            if g.kind == ArmTransfer.Goal.DEPALLETIZE and m.holding_for(g.pallet, g.slot):
                pass                                            # pre-picked: continue from the DOCK marker
            else:
                if m.carrying is not None or self.io.holding():
                    raise CellFault(f'asked to {kind} pallet {g.pallet} slot {g.slot} while holding a carton from '
                                    f'{m.carrying or "?"}')
                m.set_scene(stock=list(g.stock), docked=False, deck_box=False, fresh=True)
                m.start(g.kind != ArmTransfer.Goal.PALLETIZE, g.pallet, g.slot)
            m.run(until=prog.DOCK if g.kind == ArmTransfer.Goal.PREPARE else None, on_dock=on_dock,
                  on_undock=on_undock, on_step=lambda s: s in steps and self.feedback(gh, steps[s]))
            res.success, res.carton = True, CARTON[m.carton if g.kind != ArmTransfer.Goal.PREPARE else C_TOOL]
            self.state = ArmCellState.HOLDING if g.kind == ArmTransfer.Goal.PREPARE else ArmCellState.READY
            if g.kind != ArmTransfer.Goal.PREPARE:
                self.cycles += 1
            gh.succeed()
        except Cancelled:
            # only ever while waiting for the interlock, outside the zone: a pre-picked carton stays on the tool
            self.zone_robot = None
            res.carton = CARTON[C_TOOL if self.motion.carrying else self.motion.carton]
            if self.motion.carrying is None:
                self.motion.abandon()
            self.state = ArmCellState.HOLDING if self.motion.carrying else ArmCellState.READY
            self.log(f'{kind} for {g.robot_id} cancelled while waiting for the interlock')
            gh.canceled()
        except CellFault as e:
            self.zone_robot = None
            res.fault, res.carton = str(e), CARTON[self.motion.carton]
            self.set_fault(f'{kind} pallet {g.pallet} slot {g.slot}: {e}', carton=self.motion.carton)
            gh.abort()
        except Exception as e:                                   # never leave the action hanging
            self.zone_robot = None
            res.fault, res.carton = f'internal error: {e!r}', CARTON[self.motion.carton]
            self.set_fault(res.fault)
            gh.abort()
        finally:
            self.busy, self.phase = False, ''
        t1 = self.now()
        res.duration_s = float(t1 - t0)
        res.wait_interlock_s = float(marks['dock'] - marks['wait']) if 'dock' in marks else 0.0
        res.deck_s = float(marks['clear'] - marks['dock']) if 'clear' in marks and 'dock' in marks else 0.0
        if res.success:
            self.log(f'{kind} done in {res.duration_s:.1f} s' + (f', robot at the bay {res.deck_s:.1f} s after the '
                     f'interlock (waited {res.wait_interlock_s:.1f} s for it)' if 'dock' in marks else ''))
        return res

    def set_fault(self, text, carton=None):
        self.fault, self.state = text, ArmCellState.FAULT
        self.faults += 1
        self.log(f'FAULT: {text}' + (f' (carton: {carton})' if carton else ''), warn=True)

    # ------------------------------------------------------------------ operator
    def on_release(self, req, resp):
        if self.busy:
            resp.success, resp.message = False, 'a transfer is running'
            return resp
        try:
            self.io.vacuum(False)
            self.motion._attach(False)
            self.motion.carrying = None
            if self.motion.at_dock:
                self.motion.abandon()
            if self.state == ArmCellState.HOLDING:
                self.state = ArmCellState.READY
            resp.success, resp.message = True, 'vacuum off: carton released'
            self.log('carton released by the operator')
        except CellFault as e:
            resp.success, resp.message = False, str(e)
        return resp

    def on_reset(self, req, resp):
        if self.busy:
            resp.success, resp.message = False, 'a transfer is running'
            return resp
        if self.safety_stopped():
            resp.success, resp.message = False, f'clear the arm\'s {SAFETY_NAMES.get(self.safety_mode)} first'
            return resp
        self.busy = True
        try:
            # a robot still docked and stopped at the bay (e.g. a loaded one waiting) stays in the scene; otherwise
            # the bay is planned as clear (the arm may have stopped inside the keep-out volume)
            docked = next((r for r in sorted(self.robots) if self.interlock(r) is None), None)
            if self.io.holding():
                raise CellFault('a carton is still on the gripper: take it off (release_carton) before the reset')
            self.motion.io.vacuum(False)
            self.motion._attach(False)
            self.motion.carrying = None
            self.motion.abandon()
            self.motion.docked = docked is not None
            if docked is None:
                self.motion.clear_bay()
            else:
                self.motion.set_scene(docked=True, deck_box=True)    # assume a carton on its deck: conservative
            self.lift_clear()
            self.motion.home()
            self.motion.set_scene(docked=False, deck_box=False)
            self.fault, self.state = '', ArmCellState.READY
            resp.success, resp.message = True, 'cell reset: arm home' + (f' ({docked} at the bay)' if docked else '')
            self.log(resp.message)
        except CellFault as e:
            resp.success, resp.message = False, f'reset failed: {e}'
            self.log(resp.message, warn=True)
        finally:
            self.busy = False
        return resp

    def lift_clear(self):
        """If the gripper stopped low over the bay, go straight up out of the zone first (a carton may stand on the
        deck below it that the planning scene doesn't know about)."""
        with self.motion.psm.read_only() as scene:
            T = scene.current_state.get_global_link_transform('tcp')
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        zx, zy, sx, sy, _ = self.local.deck_zone()
        if abs(x - zx) <= sx / 2 and abs(y - zy) <= sy / 2 and z < self.local.deck_clear_height():
            yaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
            self.motion._move('lift clear', goal_pose=tool_down(x, y, self.local.deck_clear_height(), yaw), lin=True,
                              in_zone=self.motion.docked)

    # ------------------------------------------------------------------ heartbeat
    def publish(self):
        state = self.state
        if self.safety_stopped() and state != ArmCellState.STARTING:
            state = ArmCellState.STOPPED          # the stop first; a latched fault shows again once it is cleared
        at_bay = next((r for r in sorted(self.robots) if self.interlock(r) is None), '')
        self.pub.publish(ArmCellState(
            stamp=self.get_clock().now().to_msg(), arm_id=self.arm_id, state=state, phase=self.phase,
            fault=self.fault, carton_on_tool=self.io.holding(), safety_mode=int(self.safety_mode),
            interlock=bool(at_bay), robot_at_bay=at_bay, cycles=self.cycles, faults=self.faults))


def main():
    rclpy.init(args=sys.argv)
    node = CellController()
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        sys.stdout.flush()
        os._exit(0)          # moveit_py 2.12.4 segfaults when MoveItPy is destroyed (reach_study._exit)
