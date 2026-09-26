"""station_agent: one per arm station (environment side, fixed cell), no coordinator.

Heartbeat StationState on /swarm/stations at 5 Hz: state (idle / busy / fault / starved), the robot being served,
pallet stock, and `served_task` — the robot at the bay leaves when it sees its own task there. The station learns
who is waiting from the robots' heartbeats (/swarm/state: status AT_STATION on the bay node), so the handshake
needs no extra messages and survives message loss. Every transfer is announced on /swarm/transfers when it starts
(the simulator animates it). Stock, pallet swaps and fault handling: station.py.
Fault injection for tests: fault_at_s / fault_for_s (sim seconds after the station starts).
Runs until killed; uses sim time.
"""
import time as wallclock

import rclpy
import rclpy.executors
from rclpy.node import Node

from open_amr_msgs.msg import RobotState, StationState, Transfer

from .agent import EVENT_QOS, STATE_QOS
from .lane_graph import LaneGraph
from .station import STARVED, STATE_NAMES, Station, StationParams


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
        self.st = Station(p, pallets)
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
        started = self.st.step(now, self.at_bay(), fault)
        for _, text in self.st.log:
            self.get_logger().info(f'{self.sid}: {text}')
        self.st.log.clear()
        if started:
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
