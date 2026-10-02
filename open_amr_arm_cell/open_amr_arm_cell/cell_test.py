"""Cell controller scenario test (Phase 4b gate): the cell_controller of one cell in Isaac Sim (physical 8 kg cartons,
a stand-in AMR deck at the bay), driven the way the swarm drives it. This test plays the station (ArmTransfer goals,
stock bookkeeping), the robot (its /swarm/state heartbeat: away, arriving, docked, leaving), the operator (release,
reset, removing a carton) and the fault injector (simulator hooks), and checks every outcome.

  locate      a carton pushed 20 mm / 2 deg on its pallet is gripped where it is and stacked in its slot; one pushed
              60 mm faults ('out of place', still on the pallet); pushed back, reset, it is picked
  normal      N cycles: pre-pick with no robot (held over the zone) -> robot docks -> carton onto its deck -> robot
              leaves at 'deck_clear' -> robot back loaded -> carton off the deck onto pallet 1 -> robot leaves
  interlock   the arm waits over the zone while the robot is still moving in, and while a second robot is in the bay
  moved       the robot moves while the arm is placing in the zone: arm stops, fault, carton still on the tool; reset
              refused until the operator releases it (it drops onto the deck); reset lifts clear and goes home; the
              dropped carton is then located on the deck and palletized
  drop        vacuum lost while carrying: arm stops, fault 'vacuum lost', carton lost; operator removes it, reset
  estop       protective stop during a move: arm stops, cell STOPPED, fault; a goal is rejected; reset refused until
              the stop is cleared
  station     the real station_agent (cell mode, receiving) serves 3 robot visits through the cell: pre-picks, serves
              at 'deck_clear', the robot waits only for the hand-over
Throughout: the arm (TCP, and the carried carton) never enters the deck zone unless the robot is docked and stopped
(checked independently from the joint states, 100 Hz), and every stacked carton ends within 15 mm / 3 deg of its slot.

    ros2 run open_amr_arm_cell cell_test --arm-id arm_receiving [--normal 4] [--out report.yaml]
Needs open_amr_sim.py --arms --arm-test, the cell's control stack and its cell_controller (tools/tests/isaac_arm_test.sh
--cell runs everything).
"""
import argparse
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Pose2D, PoseArray
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from open_amr_msgs.action import ArmTransfer
from open_amr_msgs.msg import ArmCellState, RobotState, StationState

from . import ur_ik
from .cell_motion import make_cell
from .reach_study import JOINTS

R = ArmTransfer.Result
G = ArmTransfer.Goal
POS_TOL_M, YAW_TOL_DEG = 0.015, 3.0
STATE_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)


class Bench(Node):
    def __init__(self, arm_id, layout):
        super().__init__('cell_test', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.arm = arm_id
        self.cell = make_cell(layout, arm_id)          # world frame
        self.local = make_cell(layout)                 # cell frame
        self.p = self.cell.p
        self.ur = ur_ik.params(self.p.ur_type)
        self.lock = threading.Lock()
        self.boxes = None                              # true carton poses (x, y, z, yaw_deg), index = slot on pallet 0
        self.cell_state = None
        self.robots = {}                               # robot_id -> dict(status, pose, speed, task, last_node)
        self.docked = set()                            # robots this test has docked and stopped (ground truth)
        self.intrusions, self.zone_samples, self.tcp = [], 0, None
        self.excused = False                           # the arm stopped in the zone by a fault, until the reset
        self.station, self.station_rx = None, 0.0
        self.create_subscription(PoseArray, f'/{arm_id}/test_boxes', self.on_boxes, 1)
        self.create_subscription(ArmCellState, f'/{arm_id}/cell/state', lambda m: setattr(self, 'cell_state', m), 10)
        self.create_subscription(JointState, f'/{arm_id}/joint_states', self.on_joints, 50)
        self.create_subscription(StationState, '/swarm/stations', self.on_station, STATE_QOS)
        self.pub_state = self.create_publisher(RobotState, '/swarm/state', STATE_QOS)
        self.pub_inject = self.create_publisher(String, f'/{arm_id}/sim/inject', 10)
        self.pub_cartons = self.create_publisher(String, f'/{arm_id}/sim/cartons', 10)
        self.client = ActionClient(self, ArmTransfer, f'/{arm_id}/cell/transfer')
        self.reset_cli = self.create_client(Trigger, f'/{arm_id}/cell/reset')
        self.release_cli = self.create_client(Trigger, f'/{arm_id}/cell/release_carton')
        self.seq = 0
        self.create_timer(0.2, self.heartbeat)
        bx, by, byaw = self.cell.bay_pose()
        self.bay = (bx, by, math.radians(byaw))
        _, _, _, _, self.zone_top = self.local.deck_zone()

    # ------------------------------------------------------------------ inputs
    def on_boxes(self, m):
        with self.lock:
            self.boxes = [(p.position.x, p.position.y, p.position.z,
                           2 * math.degrees(math.atan2(p.orientation.z, p.orientation.w))) for p in m.poses]

    def on_station(self, m):
        if m.station_id == self.arm:
            self.station, self.station_rx = m, time.time()

    def on_joints(self, m):
        """Independent deck-zone check: TCP (and the carried carton's lowest corners) vs the zone, cell frame."""
        if not all(j in m.name for j in JOINTS):
            return
        q = [m.position[m.name.index(j)] for j in JOINTS]
        T = ur_ik.fk(q, self.ur)
        x, y, z = T[:3, 3] + T[:3, 2] * self.p.tool_length
        z += self.p.pedestal_height
        self.tcp = (x, y, z)
        carrying = bool(self.cell_state and self.cell_state.carton_on_tool)
        low = z - (self.p.box[2] if carrying else 0.0)
        reach = (math.hypot(*self.p.box[:2]) / 2) if carrying else 0.0
        zx, zy, sx, sy, top = self.local.deck_zone()
        inside = abs(x - zx) <= sx / 2 + reach and abs(y - zy) <= sy / 2 + reach and low < top
        self.zone_samples += 1
        if inside and not self.docked and not self.excused:
            t = self.get_clock().now().nanoseconds * 1e-9
            if not self.intrusions or t - self.intrusions[-1][0] > 1.0:
                print(f'   !! arm in the deck zone with no robot docked: TCP ({x:.3f}, {y:.3f}, {z:.3f}), lowest '
                      f'{low:.3f} < {top:.3f}', flush=True)
            self.intrusions.append((t, round(x, 3), round(y, 3), round(low, 3)))

    # ------------------------------------------------------------------ the robot(s)
    def heartbeat(self):
        for rid, r in list(self.robots.items()):
            self.seq += 1
            x, y, th = r['pose']
            self.pub_state.publish(RobotState(robot_id=rid, seq=self.seq, status=r['status'],
                                              pose=Pose2D(x=x, y=y, theta=th), speed=float(r['speed']),
                                              task_id=r.get('task', ''), last_node=r.get('node', 0),
                                              stamp=self.get_clock().now().to_msg()))

    def robot(self, rid, where, task='', node=0):
        """where: 'away' | 'arriving' (moving in, 1 m short) | 'docked' (AT_STATION, stopped at the bay) |
        'in_bay' (stopped in the bay without being served: a second robot) | 'moved' (docked, then moving)."""
        bx, by, th = self.bay
        back = (bx - 1.0 * math.cos(th), by - 1.0 * math.sin(th))
        r = {'away': dict(status=RobotState.TO_DROP, pose=(bx - 6 * math.cos(th), by - 6 * math.sin(th), th), speed=0.8),
             'arriving': dict(status=RobotState.TO_PICK, pose=(*back, th), speed=0.35),
             'docked': dict(status=RobotState.AT_STATION, pose=(bx, by, th), speed=0.0),
             'in_bay': dict(status=RobotState.YIELDING, pose=(bx + 0.1, by, th), speed=0.0),
             'moved': dict(status=RobotState.AT_STATION, pose=(bx - 0.12 * math.cos(th), by - 0.12 * math.sin(th), th),
                           speed=0.3)}[where]
        r.update(task=task, node=node)
        self.robots[rid] = r
        (self.docked.add if where == 'docked' else self.docked.discard)(rid)
        if where == 'moved':
            self.docked.add(rid)            # the arm is allowed in until it reacts (the test checks that it does)

    def gone(self, rid):
        self.robots.pop(rid, None)
        self.docked.discard(rid)

    # ------------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, s):
        """Sim seconds."""
        t0 = self.now()
        while self.now() - t0 < s:
            time.sleep(0.02)

    def poses(self):
        with self.lock:
            return list(self.boxes) if self.boxes else None

    def fresh_poses(self):
        time.sleep(0.3)
        return self.poses()

    def inject(self, what):
        self.pub_inject.publish(String(data=what))

    def carton_cmd(self, what):
        self.pub_cartons.publish(String(data=what))

    def call(self, cli, timeout=120.0):
        if not cli.wait_for_service(timeout_sec=10.0):
            return False, 'service not available'
        fut = cli.call_async(Trigger.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        r = fut.result()
        return (r.success, r.message) if r else (False, 'timeout')

    def transfer(self, kind, pallet, slot, stock, robot='', task='', on_phase=None, timeout=300.0):
        """Send one ArmTransfer and wait. Returns (accepted, result, status, phases)."""
        phases = []

        def fb(m):
            phases.append(m.feedback.phase)
            if on_phase:
                on_phase(m.feedback.phase)
        g = G(kind=kind, pallet=pallet, slot=slot, stock=list(stock), robot_id=robot, task_id=task)
        fut = self.client.send_goal_async(g, feedback_callback=fb)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 30:
            time.sleep(0.02)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, None, None, phases
        rf = gh.get_result_async()
        while not rf.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        if not rf.done():
            return True, None, 'timeout', phases
        return True, rf.result().result, rf.result().status, phases

    def wait_cell(self, states, timeout=120.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cell_state is not None and self.cell_state.state in states:
                return True
            time.sleep(0.05)
        return False

    def placement(self, k, slot):
        """(pos error m, yaw error deg) of carton k (index = its slot on pallet 0) in `slot` of pallet 1."""
        b = self.fresh_poses()[k]
        (x, y, z), yaw = self.cell.slot_world(1, slot)
        return math.dist(b[:3], (x, y, z - self.p.box[2] / 2)), abs((b[3] - yaw + 45.0) % 90.0 - 45.0)

    def outward(self, k):
        """Unit vector (world) from pallet 0's centre to carton k: pushing it this way moves it away from the
        cartons still beside it."""
        px, py, _ = self.cell.pallet_world(0)
        b = self.poses()[k]
        d = math.hypot(b[0] - px, b[1] - py) or 1.0
        return (b[0] - px) / d, (b[1] - py) / d


class Run:
    """The scenarios, in order, on one cell: pallet 0 full, pallet 1 empty."""

    def __init__(self, b: Bench, a):
        self.b, self.a = b, a
        self.stock = [b.p.capacity, 0]                 # what the station would believe
        self.placed = []                               # (carton index, pallet 1 slot)
        self.results, self.failures, self.deck_s = [], [], []

    def check(self, ok, what):
        print(f'   {"ok  " if ok else "FAIL"} {what}', flush=True)
        self.results.append(dict(check=what, ok=bool(ok)))
        if not ok:
            self.failures.append(what)
        return ok

    def top(self):
        return self.stock[0] - 1

    # ------------------------------------------------------------------ building blocks
    def prepare(self):
        k = self.top()
        acc, r, st, ph = self.b.transfer(G.PREPARE, 0, k, self.stock)
        ok = acc and r is not None and r.success and r.carton == R.CARTON_TOOL and 'holding' in ph
        if ok:
            self.stock[0] -= 1
        return ok, k, r

    def depalletize(self, k, robot='amr_t', task='t', leave=True, on_phase=None):
        """The robot docks, the held carton k goes onto its deck; it leaves at deck_clear."""
        b = self.b

        def phase(p):
            if p == 'deck_clear' and leave:
                b.robot(robot, 'away')
            if on_phase:
                on_phase(p)
        acc, r, st, ph = b.transfer(G.DEPALLETIZE, 0, k, self.stock, robot, task, phase)
        if r is not None and r.success:
            self.deck_s.append(('depalletize', round(r.deck_s, 2)))
        return acc, r, ph

    def palletize(self, k, robot='amr_t', task='t', on_phase=None):
        """The robot comes back loaded (carton k still on the stand-in deck) and docks; carton k goes to pallet 1."""
        b = self.b
        slot = self.stock[1]
        b.robot(robot, 'docked', task)

        def phase(p):
            if p == 'deck_clear':
                b.robot(robot, 'away')
            if on_phase:
                on_phase(p)
        acc, r, st, ph = b.transfer(G.PALLETIZE, 1, slot, self.stock, robot, task, phase)
        if r is not None and r.success:
            self.stock[1] += 1
            self.placed.append((k, slot))
            self.deck_s.append(('palletize', round(r.deck_s, 2)))
            e, ey = b.placement(k, slot)
            self.check(e <= POS_TOL_M and ey <= YAW_TOL_DEG, f'carton {k} in pallet 1 slot {slot}: {e * 1000:.1f} mm, '
                       f'{ey:.2f} deg')
        return acc, r, ph

    def cycle(self, name):
        ok, k, r = self.prepare()
        if not self.check(ok, f'{name}: pre-pick pallet 0 slot {k} with no robot at the bay, held over the zone'):
            return False
        self.b.robot('amr_t', 'docked', f'{name}_in')
        acc, r, ph = self.depalletize(k, task=f'{name}_in')
        if not self.check(r is not None and r.success and r.carton == R.CARTON_DECK and 'deck_clear' in ph,
                          f'{name}: carton {k} onto the deck, robot released at deck_clear'
                          f' ({r.deck_s:.1f} s at the bay)' if r else f'{name}: depalletize'):
            return False
        acc, r, ph = self.palletize(k, task=f'{name}_out')
        return self.check(r is not None and r.success and r.carton == R.CARTON_PALLET,
                          f'{name}: carton {k} off the deck onto pallet 1 ({r.deck_s:.1f} s at the bay)' if r
                          else f'{name}: palletize')

    def reset(self, expect=True, what='reset'):
        ok, msg = self.b.call(self.b.reset_cli)
        self.check(ok == expect, f'{what}: {"done" if ok else "refused"} ({msg})')
        return ok

    # ------------------------------------------------------------------ scenarios
    def locate(self):
        print('== locate: cartons pushed off their pattern position', flush=True)
        b = self.b
        k = self.top()
        ux, uy = b.outward(k)
        b.carton_cmd(f'nudge {k} {0.020 * ux:.4f} {0.020 * uy:.4f} 2.0')
        b.sleep(1.0)
        if not self.cycle('locate20'):
            return
        k = self.top()
        ux, uy = b.outward(k)
        b.carton_cmd(f'nudge {k} {0.060 * ux:.4f} {0.060 * uy:.4f} 0.0')
        b.sleep(1.0)
        acc, r, st, ph = b.transfer(G.PREPARE, 0, k, self.stock)
        self.check(r is not None and not r.success and 'out of place' in r.fault and r.carton == R.CARTON_PALLET,
                   f'carton {k} pushed 60 mm: fault "{r.fault if r else "?"}", carton still on the pallet')
        self.check(b.wait_cell([ArmCellState.FAULT], 10), 'cell state FAULT')
        b.carton_cmd(f'nudge {k} {-0.058 * ux:.4f} {-0.058 * uy:.4f} 0.0')   # the operator squares it up again
        b.sleep(1.0)
        if self.reset(what='reset after squaring the carton up'):
            self.check(b.wait_cell([ArmCellState.READY], 30), 'cell READY after the reset')
            self.cycle('locate60')

    def normal(self, n):
        print(f'== normal: {n} cycles', flush=True)
        for c in range(n):
            if not self.cycle(f'normal{c}'):
                return

    def interlock(self):
        print('== interlock: robot still moving in; a second robot in the bay', flush=True)
        b = self.b
        ok, k, r = self.prepare()
        if not self.check(ok, f'pre-pick pallet 0 slot {k}'):
            return
        b.robot('amr_t', 'arriving', 'il_in')
        b.robot('amr_x', 'in_bay')
        t_go = {}

        def release_later():
            b.sleep(4.0)
            b.robot('amr_t', 'docked', 'il_in')          # docked, but amr_x is still in the zone
            b.sleep(3.0)
            t_go['t'] = b.now()
            b.gone('amr_x')
        threading.Thread(target=release_later, daemon=True).start()
        acc, r, ph = self.depalletize(k, task='il_in')
        ok = r is not None and r.success and r.wait_interlock_s >= 6.5
        self.check(ok, f'arm waited {r.wait_interlock_s if r else -1:.1f} s over the zone (robot moving 4 s, second '
                   f'robot in the bay 3 s more), then placed')
        self.palletize(k, task='il_out')

    def moved(self):
        print('== moved: the robot moves while the arm places onto its deck', flush=True)
        b = self.b
        ok, k, r = self.prepare()
        if not self.check(ok, f'pre-pick pallet 0 slot {k}'):
            return
        b.robot('amr_t', 'docked', 'mv_in')
        (_, _, dz), _ = self.b.local.deck_cell()
        trigger = dz + b.p.approach + 0.02             # TCP below this: the slow placing move has begun

        def on_phase(p):
            if p == 'in_deck_zone':
                def watch():
                    t0 = time.time()
                    while time.time() - t0 < 30 and (b.tcp is None or b.tcp[2] > trigger):
                        time.sleep(0.005)
                    b.robot('amr_t', 'moved', 'mv_in')
                threading.Thread(target=watch, daemon=True).start()
        acc, r, ph = self.depalletize(k, task='mv_in', leave=False, on_phase=on_phase)
        self.check(r is not None and not r.success and 'interlock' in r.fault and r.carton == R.CARTON_TOOL,
                   f'arm stopped: "{r.fault if r else "?"}", carton still on the tool')
        b.excused = True                               # stopped in the zone; the robot backs off
        b.robot('amr_t', 'away')
        b.sleep(1.0)
        self.reset(expect=False, what='reset with the carton on the gripper')
        ok, msg = b.call(b.release_cli)
        self.check(ok, f'operator releases the carton onto the deck ({msg})')
        b.sleep(2.0)
        ok = self.reset(what='reset (lift clear of the zone, home)')
        b.excused = False
        if ok:
            # the carton fell onto the deck: the robot comes back for it and the arm finds it there
            acc, r, ph = self.palletize(k, task='mv_out')
            self.check(r is not None and r.success, 'the dropped carton located on the deck and palletized')

    def drop(self):
        print('== drop: vacuum lost while carrying', flush=True)
        b = self.b
        ok, k, r = self.prepare()
        if not self.check(ok, f'pre-pick pallet 0 slot {k}'):
            return
        b.robot('amr_t', 'docked', 'dr_in')
        acc, r, ph = self.depalletize(k, task='dr_in')
        if not self.check(r is not None and r.success, f'carton {k} onto the deck'):
            return
        slot = self.stock[1]
        b.robot('amr_t', 'docked', 'dr_out')

        def phase(p):
            if p == 'deck_clear':
                b.robot('amr_t', 'away')
                b.inject('drop')                       # carton over the deck at the hold height
        acc, r, st, ph = b.transfer(G.PALLETIZE, 1, slot, self.stock, 'amr_t', 'dr_out', phase)
        self.check(r is not None and not r.success and 'vacuum lost' in r.fault and r.carton == R.CARTON_LOST,
                   f'fault "{r.fault if r else "?"}", carton reported lost')
        b.sleep(2.0)
        b.carton_cmd(f'remove {k}')                    # the operator takes it away
        b.sleep(1.0)
        self.reset(what='reset after removing the dropped carton')

    def estop(self):
        print('== estop: protective stop during a move', flush=True)
        b = self.b
        ok, k, r = self.prepare()
        if not self.check(ok, f'pre-pick pallet 0 slot {k}'):
            return
        b.robot('amr_t', 'docked', 'es_in')
        acc, r, ph = self.depalletize(k, task='es_in')
        if not self.check(r is not None and r.success, f'carton {k} onto the deck'):
            return

        def phase(p):
            if p == 'home':
                def later():
                    b.sleep(0.3)
                    b.inject('protective_stop')
                threading.Thread(target=later, daemon=True).start()
        slot = self.stock[1]
        b.robot('amr_t', 'docked', 'es_out')
        done = {}

        def ph2(p):
            if p == 'deck_clear':
                b.robot('amr_t', 'away')
            phase(p)
        acc, r, st, ph = b.transfer(G.PALLETIZE, 1, slot, self.stock, 'amr_t', 'es_out', ph2)
        self.check(r is not None and not r.success and 'protective stop' in r.fault and r.carton == R.CARTON_PALLET,
                   f'fault "{r.fault if r else "?"}", carton already on pallet 1')
        if r is not None and r.carton == R.CARTON_PALLET:
            self.stock[1] += 1
            self.placed.append((k, slot))
        self.check(b.wait_cell([ArmCellState.STOPPED], 10), 'cell state STOPPED while the stop is active')
        acc, r, st, ph = b.transfer(G.PREPARE, 0, self.top(), self.stock)
        self.check(not acc, 'a new transfer is rejected while the cell is stopped')
        self.reset(expect=False, what='reset during the protective stop')
        b.inject('clear_stop')
        b.sleep(1.0)
        done['reset'] = self.reset(what='reset after the stop is cleared')
        self.check(b.wait_cell([ArmCellState.READY], 30), 'cell READY')

    def station(self, visits, graph):
        print(f'== station: station_agent (cell mode) serves {visits} robot visits', flush=True)
        b = self.b
        from open_amr_swarm.lane_graph import LaneGraph
        bay = LaneGraph(graph).id('receiving_bay')
        stock = list(self.stock)
        cmd = ['ros2', 'run', 'open_amr_swarm', 'station_agent', '--ros-args', '-r', '__node:=station_cell_test',
               '-p', f'station_id:={b.arm}', '-p', 'role:=receiving', '-p', 'bay:=receiving_bay', '-p', f'graph:={graph}',
               '-p', f'pallets:=[{stock[0]}, 0]', '-p', 'cell:=true', '-p', 'use_sim_time:=true']
        proc = subprocess.Popen(cmd, stdout=open(os.path.join(self.a.log_dir, 'station.log'), 'w'),
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            t0 = time.time()
            while (b.station is None or b.cell_state is None or b.cell_state.state != ArmCellState.HOLDING) and \
                    time.time() - t0 < 120:
                time.sleep(0.1)
            self.check(b.cell_state is not None and b.cell_state.state == ArmCellState.HOLDING,
                       'station pre-picked a carton before any robot came')
            for v in range(visits):
                k = b.station.pallets[0] - 1 if b.cell_state.state != ArmCellState.HOLDING else b.station.pallets[0]
                task = f'st_{v}'
                b.robot('amr_s', 'docked', task, node=bay)
                t_dock = b.now()
                t1 = time.time()
                while (b.station is None or b.station.served_task != task) and time.time() - t1 < 120 and \
                        time.time() - b.station_rx < 3.0 and proc.poll() is None:
                    time.sleep(0.05)
                if proc.poll() is not None or time.time() - b.station_rx >= 3.0:
                    self.check(False, f'station_agent alive (exit code {proc.poll()}, see station.log)')
                    break
                served = b.station is not None and b.station.served_task == task
                wait = b.now() - t_dock
                self.check(served, f'visit {v}: served after {wait:.1f} s at the bay')
                self.deck_s.append(('station visit', round(wait, 2)))
                b.robot('amr_s', 'away', task)
                b.sleep(1.5)
                b.carton_cmd(f'remove {k}')            # it drove off with the carton
                b.sleep(1.0)
                while b.cell_state.state != ArmCellState.HOLDING and time.time() - t1 < 180:
                    time.sleep(0.1)                     # the next pre-pick
            self.stock[0] = b.station.pallets[0] + (1 if b.cell_state.state == ArmCellState.HOLDING else 0)
        finally:
            os.killpg(proc.pid, 2)
            proc.wait(timeout=10)
            b.gone('amr_s')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--arm-id', default='arm_receiving')
    ap.add_argument('--normal', type=int, default=4)
    ap.add_argument('--visits', type=int, default=3)
    ap.add_argument('--scenarios', default='locate,normal,interlock,moved,drop,estop,station')
    root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
    ap.add_argument('--layout', default=os.path.join(root, 'sim', 'configs', 'warehouse_layout.yaml'))
    ap.add_argument('--graph', default=os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.geojson'))
    ap.add_argument('--out', default='')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    a.log_dir = os.path.dirname(os.path.abspath(a.out)) if a.out else '/tmp'
    rclpy.init(args=ros_args)
    b = Bench(a.arm_id, a.layout)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(b)
    threading.Thread(target=ex.spin, daemon=True).start()
    run = Run(b, a)
    t0 = time.time()
    while (b.boxes is None or b.cell_state is None or b.cell_state.state != ArmCellState.READY) and time.time() - t0 < 300:
        time.sleep(0.2)
    if b.cell_state is None or b.cell_state.state != ArmCellState.READY:
        print(f'cell controller not READY ({b.cell_state.fault if b.cell_state else "no /cell/state"})', flush=True)
        os._exit(2)
    print(f'cell READY; cartons {len(b.boxes)}', flush=True)
    t_sim0 = b.now()
    for s in a.scenarios.split(','):
        if s == 'normal':
            run.normal(a.normal)
        elif s == 'station':
            run.station(a.visits, a.graph)
        else:
            getattr(run, s)()
    # every stacked carton still in its slot at the end
    for k, slot in run.placed:
        e, ey = b.placement(k, slot)
        run.check(e <= POS_TOL_M and ey <= YAW_TOL_DEG, f'end: carton {k} in pallet 1 slot {slot}: {e * 1000:.1f} mm, '
                  f'{ey:.2f} deg')
    run.check(not b.intrusions, f'deck zone: {len(b.intrusions)} samples with the arm inside and no robot docked '
              f'(of {b.zone_samples})')
    decks = [d for _, d in run.deck_s]
    summary = dict(checks=len(run.results), failed=len(run.failures), stacked=len(run.placed),
                   robot_at_bay_s=dict(mean=round(float(np.mean(decks)), 2) if decks else None,
                                       max=max(decks, default=None)),
                   sim_s=round(b.now() - t_sim0, 1), wall_s=round(time.time() - t0, 1))
    summary['gate'] = 'PASS' if not run.failures else 'FAIL'
    print(f'cell test {a.arm_id}: {summary}', flush=True)
    for f in run.failures:
        print(f'  failed: {f}', flush=True)
    if a.out:
        yaml.safe_dump(dict(summary=summary, checks=run.results, robot_at_bay=run.deck_s, intrusions=b.intrusions[:50]),
                       open(a.out, 'w'), sort_keys=False)
    sys.stdout.flush()
    os._exit(0 if summary['gate'] == 'PASS' else 1)


if __name__ == '__main__':
    sys.exit(main())
