"""station_agent: one per arm station (environment side, fixed cell), no coordinator.

Heartbeat StationState on /swarm/stations at 5 Hz: state (idle / busy / fault / starved), the robot being served,
pallet stock, and `served_task` — the robot at the bay leaves when it sees its own task there. The station learns
who is waiting from the robots' heartbeats (/swarm/state: status AT_STATION on the bay node), so the handshake
needs no extra messages and survives message loss. Every transfer is announced on /swarm/transfers when it starts
(the simulator animates it). Stock, pallet swaps and fault handling: station.py.
Fault injection for tests: fault_at_s / fault_for_s (sim seconds after the station starts).
cell:=true — the arm is a real cell (open_amr_arm_cell cell_controller at /<station_id>): transfers are ArmTransfer goals
(station.CellStation: receiving pre-picks the next carton while nobody waits, the robot is served when the arm reports
'deck_clear'); the station is in FAULT while the cell is faulted, stopped, starting or silent. A job whose robot leaves
the bay before the arm entered the deck zone is cancelled.
Runs until killed; uses sim time.
"""
import time as wallclock

import rclpy
import rclpy.executors
from rclpy.node import Node

from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from open_amr_msgs.action import ArmTransfer
from open_amr_msgs.msg import ArmCellState, RobotState, StationState, Transfer

from .agent import EVENT_QOS, STATE_QOS
from .lane_graph import LaneGraph
from .station import C_NONE, DEPALLETIZE, FAULT, JOB_NAMES, PREPARE, STARVED, STATE_NAMES, CellStation, Station, \
    StationParams


class StationAgent(Node):
    def __init__(self):
        super().__init__('station_agent')
        dp = self.declare_parameter
        self.sid = dp('station_id', 'arm_receiving').value
        role = dp('role', 'receiving').value
        graph = LaneGraph(dp('graph', '').value)
        self.bay_name = dp('bay', 'receiving_bay').value
        self.bay = graph.id(self.bay_name)
        p = StationParams(role=role, capacity=dp('capacity', 24).value, cycle_s=dp('cycle_s', 8.0).value,
                          swap_s=dp('swap_s', 180.0).value)
        pallets = [int(n) for n in dp('pallets', [p.capacity if role == 'receiving' else 0] * 2).value]
        self.cell = dp('cell', False).value
        self.st = (CellStation if self.cell else Station)(p, pallets)
        if self.cell:
            self.deck_nominal = dp('deck_s_nominal', 3.0).value      # announced transfer time (box visuals)
            self.cell_timeout = dp('cell_timeout_s', 2.0).value
            self.cell_state, self.cell_rx = None, 0.0
            self.goal, self.sending, self.phase, self.left_since = None, False, '', None
            self.client = ActionClient(self, ArmTransfer, f'/{self.sid}/cell/transfer')
            self.create_subscription(ArmCellState, f'/{self.sid}/cell/state', self.on_cell, 10)
        self.fault_at, self.fault_for = dp('fault_at_s', 0.0).value, dp('fault_for_s', 0.0).value
        self.peer_timeout = dp('peer_timeout_s', 3.0).value
        self.robots = {}                 # robot_id -> (RobotState, receive wall time)
        self.t0 = None
        self.pub_state = self.create_publisher(StationState, '/swarm/stations', STATE_QOS)
        self.pub_transfer = self.create_publisher(Transfer, '/swarm/transfers', EVENT_QOS)
        self.create_subscription(RobotState, '/swarm/state', self.on_state, STATE_QOS)
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'{self.sid}: {role} station at {self.bay_name}, pallets {pallets} of {p.capacity}, '
                               f'cycle {p.cycle_s:.1f} s'
                               + (f', fault injected at {self.fault_at:.0f} s for {self.fault_for:.0f} s' if self.fault_for else ''))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cell(self, m):
        self.cell_state, self.cell_rx = m, wallclock.monotonic()

    def cell_down(self):
        """Why the cell can't work now (None: in service)."""
        c = self.cell_state
        if c is None or wallclock.monotonic() - self.cell_rx > self.cell_timeout:
            return 'cell controller silent'
        if c.state in (ArmCellState.READY, ArmCellState.BUSY, ArmCellState.HOLDING):
            return None
        return {ArmCellState.STARTING: 'cell starting', ArmCellState.STOPPED: 'arm safety stop'}.get(
            c.state, f'cell fault: {c.fault}')

    # ---------------------------------------------------------------- cell jobs (ArmTransfer)
    def send(self, job):
        self.phase, self.left_since = '', None
        g = ArmTransfer.Goal(kind=job.kind, pallet=job.pallet, slot=job.slot, stock=[int(n) for n in self.st.pallets],
                             robot_id=job.robot, task_id=job.task)
        if job.kind == PREPARE:              # the station already took the carton off its count: the scene must hold it
            g.stock[job.pallet] += 1
        elif job.kind == DEPALLETIZE and not job.held:
            g.stock[job.pallet] += 1
        self.goal, self.sending = None, True
        fut = self.client.send_goal_async(g, feedback_callback=self.on_feedback)
        fut.add_done_callback(lambda f, j=job: self.on_accepted(f, j))

    def on_accepted(self, fut, job):
        gh = fut.result()
        self.sending = False
        if not gh.accepted:
            self.get_logger().warn(f'{self.sid}: cell rejected {JOB_NAMES[job.kind]} pallet {job.pallet} slot {job.slot}')
            if job.kind == DEPALLETIZE and job.held:
                self.st.cancelled(self.now())
            else:
                self.st.finished(self.now(), False, C_NONE, 'rejected by the cell')
            return
        self.goal = gh
        gh.get_result_async().add_done_callback(lambda f, j=job: self.on_result(f, j))

    def on_feedback(self, m):
        self.phase = m.feedback.phase
        if self.phase == 'deck_clear':
            self.st.deck_clear(self.now())

    def on_result(self, fut, job):
        r, status = fut.result().result, fut.result().status
        self.goal, now = None, self.now()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.st.finished(now, True, r.carton)
            if job.kind != PREPARE:
                self.get_logger().info(f'{self.sid}: {JOB_NAMES[job.kind]} for {job.robot} done: robot at the bay '
                                       f'{r.deck_s:.1f} s after docking, arm {r.duration_s:.1f} s')
        elif status == GoalStatus.STATUS_CANCELED:
            self.st.cancelled(now)
        else:
            self.st.finished(now, False, r.carton, r.fault or f'status {status}')

    def cell_tick(self, now, at_bay, fault):
        if fault is None and self.cell_state is not None and self.cell_state.state == ArmCellState.READY \
                and self.st.held and self.st.job is None:
            self.st.tool_cleared(now)                        # reset with the gripper empty: a person took it off
        job = self.st.step(now, at_bay, fault is not None)
        if job is not None:
            self.send(job)
            if job.kind != PREPARE:
                self.pub_transfer.publish(Transfer(
                    stamp=self.get_clock().now().to_msg(), task_id=job.task, robot_id=job.robot, station_id=self.sid,
                    kind=Transfer.PALLET_TO_ROBOT if self.st.receiving else Transfer.ROBOT_TO_PALLET,
                    pallet=job.pallet, slot=job.slot, duration_s=float(self.deck_nominal)))
                self.get_logger().info(f'{self.sid}: serving {job.robot} ({job.task}), pallet {job.pallet} slot '
                                       f'{job.slot}{" (pre-picked)" if job.held else ""}, stock {self.st.pallets}')
            return
        j = self.st.job
        # (a ClientGoalHandle must not be compared with ==: its __eq__ reads the other side's goal_id)
        if j is not None and j.kind != PREPARE and self.goal is not None and not self.sending and \
                self.phase in ('', 'picking', 'holding', 'waiting_interlock') and at_bay != (j.robot, j.task):
            self.left_since = self.left_since or now
            if now - self.left_since > 2.0:                  # the robot is gone before the arm came near it
                self.get_logger().warn(f'{self.sid}: {j.robot} left the bay: cancelling {JOB_NAMES[j.kind]}')
                self.goal.cancel_goal_async()
                self.left_since = float('inf')
        elif j is not None:
            self.left_since = None

    def on_state(self, m):
        self.robots[m.robot_id] = (m, wallclock.monotonic())

    def at_bay(self):
        t = wallclock.monotonic()
        here = sorted((m.robot_id, m.task_id) for m, rx in self.robots.values()
                      if t - rx < self.peer_timeout and m.last_node == self.bay and m.status == RobotState.AT_STATION
                      and m.task_id)
        if len(here) > 1:
            self.get_logger().warn(f'{self.sid}: {len(here)} robots report the bay: {here}', throttle_duration_sec=10.0)
        return here[0] if here else None

    def tick(self):
        now = self.now()
        if now <= 0:
            return                       # no /clock yet
        if self.t0 is None:
            self.t0 = now
        el = now - self.t0
        fault = self.fault_for > 0 and self.fault_at <= el < self.fault_at + self.fault_for
        prev = self.st.state
        if self.cell:
            why = 'fault injected' if fault else self.cell_down()
            if why and prev != FAULT:
                self.get_logger().warn(f'{self.sid}: out of service: {why}')
            self.cell_tick(now, self.at_bay(), why)
            started = None
        else:
            started = self.st.step(now, self.at_bay(), fault)
        for _, text in self.st.log:
            self.get_logger().info(f'{self.sid}: {text}')
        self.st.log.clear()
        if self.cell:
            if prev != self.st.state and self.st.state == STARVED:
                self.get_logger().info(f'{self.sid}: starved (pallets {self.st.pallets})')
        elif started:
            recv = self.st.receiving
            self.pub_transfer.publish(Transfer(
                stamp=self.get_clock().now().to_msg(), kind=Transfer.PALLET_TO_ROBOT if recv else Transfer.ROBOT_TO_PALLET,
                task_id=started.task, robot_id=started.robot, station_id=self.sid, pallet=started.pallet,
                slot=started.slot, duration_s=float(self.st.p.cycle_s)))
            self.get_logger().info(f'{self.sid}: serving {started.robot} ({started.task}), pallet {started.pallet} '
                                   f'slot {started.slot}, stock {self.st.pallets}')
        elif prev != self.st.state and self.st.state == STARVED:
            self.get_logger().info(f'{self.sid}: starved (pallets {self.st.pallets})')
        self.publish()

    def publish(self):
        s = self.st
        self.pub_state.publish(StationState(
            stamp=self.get_clock().now().to_msg(), station_id=self.sid, bay_node=self.bay_name, state=s.state,
            claimed_by=s.robot, task_id=s.task, served_task=s.served, boxes_available=s.available(),
            pallets=[int(n) for n in s.pallets], capacity=s.p.capacity, transfers=s.transfers,
            cycle_s=float(s.p.cycle_s)))


def main():
    rclpy.init()
    node = StationAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(f'{node.sid}: {node.st.transfers} transfers, final state '
                               f'{STATE_NAMES[node.st.state]}, pallets {node.st.pallets}')
