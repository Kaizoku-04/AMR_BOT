"""Driver-in-the-loop test of a cell (Phase 4b): the same cell_controller as in the simulator, against UR's own controller
software (URSim, Docker) through the real ur_robot_driver — scaled_joint_trajectory_controller, io_and_status_controller,
dashboard client — in real time. What the simulator can't check: the driver's execution path, the speed slider, pause /
resume, a stopped program, the I/O and payload services of a real UR.

  cycles   N pre-pick / hand-over / palletize cycles at full speed (real-time durations)
  slider   the pendant's speed slider at 50 %: the transfer completes, slower (scaled controller, no duration abort)
  hold     speed slider 0 % for 3 s mid-transfer, back to 100 %: the transfer completes (pendant pause / play needs the
           URCap program, not headless mode)
  stop     the program stopped mid-transfer: the cell faults; program resent, cell reset: work continues

URSim has no vacuum switch: the cell controller runs with part_present_echo (part present = the vacuum output read
back), and there are no cartons — this tests the arm's control path, not the handling (the Isaac tests do that).

    ros2 run open_amr_arm_cell driver_test --arm-id arm_receiving [--cycles 3] [--out report.yaml]
tools/tests/ursim_cell_test.sh runs everything (URSim, the driver stack, the cell controller, this test).
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from geometry_msgs.msg import Pose2D
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from open_amr_msgs.action import ArmTransfer
from open_amr_msgs.msg import ArmCellState, RobotState

from .cell_motion import make_cell

G = ArmTransfer.Goal


class Bench(Node):
    def __init__(self, arm, layout):
        super().__init__('driver_test')
        self.arm = arm
        self.cell_state, self.scaling, self.robot_mode = None, None, None
        self.cell = make_cell(layout, arm)
        bx, by, byaw = self.cell.bay_pose()
        self.bay = (bx, by, math.radians(byaw))
        self.create_subscription(ArmCellState, f'/{arm}/cell/state', lambda m: setattr(self, 'cell_state', m), 10)
        self.create_subscription(Float64, f'/{arm}/speed_scaling_state_broadcaster/speed_scaling',
                                 lambda m: setattr(self, 'scaling', m.data), 10)
        from ur_dashboard_msgs.msg import RobotMode
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.create_subscription(RobotMode, f'/{arm}/io_and_status_controller/robot_mode',
                                 lambda m: setattr(self, 'robot_mode', m.mode),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(RobotState, '/swarm/state', 10)
        self.create_timer(0.2, self.heartbeat)
        self.client = ActionClient(self, ArmTransfer, f'/{arm}/cell/transfer')
        self.dash = {n: self.create_client(Trigger, f'/{arm}/dashboard_client/{n}')
                     for n in ('power_on', 'brake_release', 'pause', 'play', 'stop')}
        self.resend = self.create_client(Trigger, f'/{arm}/io_and_status_controller/resend_robot_program')
        self.reset = self.create_client(Trigger, f'/{arm}/cell/reset')
        self.release = self.create_client(Trigger, f'/{arm}/cell/release_carton')
        from ur_msgs.srv import SetSpeedSliderFraction
        self.SetSpeed = SetSpeedSliderFraction
        self.slider = self.create_client(SetSpeedSliderFraction, f'/{arm}/io_and_status_controller/set_speed_slider')
        self.seq = 0

    def heartbeat(self):                     # a robot docked and stopped at the bay
        self.seq += 1
        x, y, th = self.bay
        self.pub.publish(RobotState(robot_id='amr_t', seq=self.seq, status=RobotState.AT_STATION, docked=True,
                                    pose=Pose2D(x=x, y=y, theta=th), speed=0.0, task_id='dt'))

    def call(self, cli, req=None, timeout=30.0):
        if not cli.wait_for_service(timeout_sec=10.0):
            return False, 'no service'
        f = cli.call_async(req or Trigger.Request())
        t0 = time.time()
        while not f.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        r = f.result()
        return (bool(r.success), getattr(r, 'message', '')) if r else (False, 'timeout')

    def transfer(self, kind, pallet, slot, stock, on_phase=None, timeout=180.0):
        phases = []

        def fb(m):
            phases.append(m.feedback.phase)
            if on_phase:
                on_phase(m.feedback.phase)
        f = self.client.send_goal_async(G(kind=kind, pallet=pallet, slot=slot, stock=list(stock), robot_id='amr_t',
                                          task_id='dt'), feedback_callback=fb)
        t0 = time.time()
        while not f.done() and time.time() - t0 < 30:
            time.sleep(0.02)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None, 0.0, phases
        r = gh.get_result_async()
        while not r.done() and time.time() - t0 < timeout:
            time.sleep(0.05)
        return (r.result().result if r.done() else None), time.time() - t0, phases

    def wait_cell(self, states, timeout=120.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.cell_state is not None and self.cell_state.state in states:
                return True
            time.sleep(0.1)
        return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--arm-id', default='arm_receiving')
    ap.add_argument('--cycles', type=int, default=3)
    ap.add_argument('--scenarios', default='cycles,slider,pause,stop')     # (pause = the hold above)
    ap.add_argument('--layout', default=os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')),
                                                    'sim', 'configs', 'warehouse_layout.yaml'))
    ap.add_argument('--out', default='')
    a, ros_args = ap.parse_known_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=ros_args)
    b = Bench(a.arm_id, a.layout)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(b)
    threading.Thread(target=ex.spin, daemon=True).start()
    results, fails = [], []

    def check(ok, what):
        print(f'   {"ok  " if ok else "FAIL"} {what}', flush=True)
        results.append(dict(check=what, ok=bool(ok)))
        if not ok:
            fails.append(what)
        return ok

    # bring the robot up as an operator would: power on, release the brakes, (re)send the External Control program
    print('== bring-up', flush=True)
    for step in ('power_on', 'brake_release'):
        ok, msg = b.call(b.dash[step])
        check(ok, f'dashboard {step} ({msg})')
        time.sleep(3.0)
    t0 = time.time()
    while b.robot_mode != 7 and time.time() - t0 < 60:          # RobotMode.RUNNING
        time.sleep(0.5)
    check(b.robot_mode == 7, f'robot mode RUNNING ({b.robot_mode})')
    ok, msg = b.call(b.resend)
    check(ok, f'External Control program sent ({msg})')
    if not b.wait_cell([ArmCellState.READY, ArmCellState.FAULT], 120):
        check(False, 'cell controller state')
        os._exit(2)
    if b.cell_state.state == ArmCellState.FAULT:                  # it started before the robot ran: reset once
        ok, msg = b.call(b.reset, timeout=120)
        check(ok, f'cell reset after bring-up ({msg})')
    check(b.wait_cell([ArmCellState.READY], 60), 'cell READY (home on the real controller)')
    stock = [24, 0]
    times = []

    def cycle(tag, on_phase=None):
        k, slot = stock[0] - 1, stock[1]
        r, d1, _ = b.transfer(G.PREPARE, 0, k, stock, on_phase)
        if not check(r is not None and r.success, f'{tag}: pre-pick pallet 0 slot {k} ({d1:.1f} s)'
                     + (f' {r.fault}' if r is not None and not r.success else '')):
            return None
        stock[0] -= 1
        r, d2, _ = b.transfer(G.DEPALLETIZE, 0, k, stock)
        if not check(r is not None and r.success, f'{tag}: onto the deck ({d2:.1f} s, robot {r.deck_s if r else 0:.1f} s)'):
            return None
        r, d3, _ = b.transfer(G.PALLETIZE, 1, slot, stock)
        if not check(r is not None and r.success, f'{tag}: off the deck onto pallet 1 ({d3:.1f} s, robot '
                     f'{r.deck_s if r else 0:.1f} s)'):
            return None
        stock[1] += 1
        return d1, d2, d3

    for s in a.scenarios.split(','):
        if b.cell_state is not None and b.cell_state.state == ArmCellState.FAULT:   # the operator, between scenarios
            if b.cell_state.carton_on_tool:
                b.call(b.release)
            b.call(b.resend)
            time.sleep(2.0)
            ok, msg = b.call(b.reset, timeout=120)
            print(f'   (operator reset before {s}: {msg})', flush=True)
        if s == 'cycles':
            print(f'== cycles: {a.cycles} at full speed (real time)', flush=True)
            for c in range(a.cycles):
                t = cycle(f'cycle {c}')
                if t:
                    times.append(t)
        elif s == 'slider':
            print('== slider: speed slider 50 %', flush=True)
            req = b.SetSpeed.Request()
            req.speed_slider_fraction = 0.5
            ok, _ = b.call(b.slider, req)
            check(ok, 'speed slider set to 50 %')
            time.sleep(1.0)
            check(b.scaling is not None and abs(b.scaling - 50.0) < 5.0, f'driver reports speed scaling {b.scaling} %')
            t = cycle('at 50 %')
            if t and times:
                full = [sum(x) for x in times]
                check(sum(t) > 1.3 * (sum(full) / len(full)), f'cycle {sum(t):.1f} s at 50 % vs {sum(full) / len(full):.1f} s')
            req.speed_slider_fraction = 1.0
            b.call(b.slider, req)
            time.sleep(1.0)
        elif s == 'pause':
            # a hold mid-transfer: the speed slider to 0 and back (the scaled controller freezes the trajectory's
            # time). Pendant pause / play needs the URCap External Control program — not headless mode, where the
            # dashboard's 'play' has no program to resume (found here: the transfer hung, 2026-10-02)
            print('== hold: speed slider 1 % (the driver minimum) for 3 s mid-transfer, then 100 %', flush=True)
            peak = {}

            def hold_on(p):
                if p == 'picking':
                    def later():
                        time.sleep(0.5)
                        req = b.SetSpeed.Request()
                        # turned down over 1 s, as an operator turns the slider: a jump from 100 % to 1 % mid-swing
                        # left the arm 0.202 rad behind its slowed reference -> the scaled controller's 0.2 rad path
                        # tolerance aborted the move (2026-10-02). 0.01 = the driver's minimum (its .srv doc says 0)
                        for f in (0.7, 0.4, 0.2, 0.1, 0.05, 0.01):
                            req.speed_slider_fraction = f
                            b.call(b.slider, req)
                            time.sleep(0.2)
                        time.sleep(3.0)
                        peak['scaling'] = b.scaling
                        for f in (0.1, 0.4, 1.0):
                            req.speed_slider_fraction = f
                            b.call(b.slider, req)
                            time.sleep(0.2)
                    threading.Thread(target=later, daemon=True).start()
            k = stock[0] - 1
            r, d, _ = b.transfer(G.PREPARE, 0, k, stock, hold_on)
            check(peak.get('scaling') is not None and peak['scaling'] <= 2.0, f'robot held (speed scaling '
                  f'{peak.get("scaling")} %)')
            check(r is not None and r.success, f'pre-pick completed across the hold ({d:.1f} s)'
                  + (f': {r.fault}' if r is not None and not r.success else ''))
            if r is not None and r.success:
                stock[0] -= 1
                r, _, _ = b.transfer(G.DEPALLETIZE, 0, k, stock)
                r2, _, _ = b.transfer(G.PALLETIZE, 1, stock[1], stock)
                if r2 is not None and r2.success:
                    stock[1] += 1
        elif s == 'stop':
            print('== stop: program stopped mid-transfer', flush=True)

            def stop_on(p):
                if p == 'picking':
                    def later():
                        time.sleep(0.5)
                        b.call(b.dash['stop'])
                    threading.Thread(target=later, daemon=True).start()
            k = stock[0] - 1
            r, d, _ = b.transfer(G.PREPARE, 0, k, stock, stop_on)
            check(r is not None and not r.success, f'stopped program -> transfer failed: "{r.fault if r else "?"}"')
            check(b.wait_cell([ArmCellState.FAULT, ArmCellState.STOPPED], 10), 'cell out of service')
            ok, msg = b.call(b.resend)
            check(ok, f'program resent ({msg})')
            time.sleep(2.0)
            if b.cell_state is not None and b.cell_state.carton_on_tool:     # the operator takes it off
                ok, msg = b.call(b.release)
                check(ok, f'carton taken off the gripper ({msg})')
            ok, msg = b.call(b.reset, timeout=120)
            check(ok, f'cell reset ({msg})')
            check(b.wait_cell([ArmCellState.READY], 30), 'cell READY')
            t = cycle('after the stop')
    full = [sum(x) for x in times]
    summary = dict(checks=len(results), failed=len(fails),
                   cycle_s=dict(mean=round(sum(full) / len(full), 1) if full else None, max=round(max(full), 1) if full else None),
                   gate='PASS' if not fails else 'FAIL')
    print(f'driver test {a.arm_id}: {summary}', flush=True)
    for f in fails:
        print(f'  failed: {f}', flush=True)
    if a.out:
        yaml.safe_dump(dict(summary=summary, checks=results, cycles=times), open(a.out, 'w'), sort_keys=False)
    os._exit(0 if not fails else 1)


if __name__ == '__main__':
    sys.exit(main())
