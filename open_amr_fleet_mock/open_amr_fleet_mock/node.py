"""mock_fleet: the mock fleet (fleet.py) on the real /swarm/* topics, with the real messages and QoS, driving /clock.

    ros2 run open_amr_fleet_mock mock_fleet --ros-args -p graph:=$OPENAMR_ROOT/sim/worlds/warehouse_mission_graph.geojson \
        -p robots:=6 -p rtf:=20.0 -p scenario:=<file.yaml>
Everything else (the WMS, dashboards) runs with use_sim_time:=true and sees N robots and two arm stations.
  rtf        sim seconds per wall second (0 = as fast as the CPU allows); the sim advances in dt steps (0.1 s), each
             published on /clock, so a shift of thousands of orders takes minutes.
  scenario   YAML of timed failures (fleet.py module doc); seed, drop_rate, chargers, battery_scale as parameters.
  inject     a parameter used as a service: `ros2 param set /mock_fleet inject "silent amr_3 60"` applies a command at
             once (the set fails with the reason if the command is bad).
Bridging: what the mock robots and stations publish goes out on DDS; /swarm/tasks, /swarm/operator and the claims
other nodes publish (the WMS's CANCEL / re-sends) come in. The fleet's own claims, echoed back by DDS, are ignored.
"""
import os
import time as wallclock

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from rosgraph_msgs.msg import Clock

from open_amr_msgs.msg import Bid, Claim, Inventory, OperatorCommand, RobotState, StationState, Task, Transfer
from open_amr_swarm.agent import EVENT_QOS, OPERATOR_QOS, STATE_QOS
from open_amr_swarm.lane_graph import LaneGraph

from .fleet import Fleet, load_scenario
from .robot import to_time

OUT = {'/swarm/state': (RobotState, STATE_QOS), '/swarm/stations': (StationState, STATE_QOS),
       '/swarm/bids': (Bid, EVENT_QOS), '/swarm/claims': (Claim, EVENT_QOS), '/swarm/transfers': (Transfer, EVENT_QOS)}


class MockFleetNode(Node):
    def __init__(self):
        super().__init__('mock_fleet')
        dp = self.declare_parameter
        root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
        gpath = dp('graph', os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.geojson')).value
        events, kw = [], {}
        scen = dp('scenario', '').value
        if scen:
            events, kw = load_scenario(scen)
        chargers = [c for c in dp('chargers', kw.pop('chargers', ['charge_0', 'charge_1'])).value if c]
        self.fleet = Fleet(LaneGraph(gpath), robots=dp('robots', kw.pop('robots', 6)).value,
                           seed=dp('seed', kw.pop('seed', 1)).value, chargers=chargers or None,
                           battery_scale=dp('battery_scale', float(kw.pop('battery_scale', 20.0))).value,
                           drop_rate=dp('drop_rate', float(kw.pop('drop_rate', 0.0))).value, scenario=events, **kw)
        self.rtf = dp('rtf', 20.0).value
        self.dt = dp('dt', 0.1).value
        self.clock_on = dp('publish_clock', True).value
        dp('inject', '')
        self.add_on_set_parameters_callback(self.on_params)
        self.ours = {r.rid for r in self.fleet.robots}
        self.pubs = {t: self.create_publisher(c, t, q) for t, (c, q) in OUT.items()}
        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.fleet.bus.taps.append(self.out)
        self.create_subscription(Task, '/swarm/tasks', lambda m: self.inp('/swarm/tasks', m), EVENT_QOS)
        self.create_subscription(Claim, '/swarm/claims', self.on_claim, EVENT_QOS)
        self.create_subscription(OperatorCommand, '/swarm/operator', lambda m: self.inp('/swarm/operator', m),
                                 OPERATOR_QOS)
        self.now = 1.0
        self.w0, self.s0 = wallclock.monotonic(), self.now
        self.create_timer(0.02, self.tick, clock=rclpy.clock.Clock())   # wall-clock timer: we are the sim clock
        self.get_logger().info(f'mock fleet: {len(self.fleet.robots)} robots, 2 stations, rtf {self.rtf or "max"}, '
                               f'{len(events)} scenario events, drop rate {self.fleet.bus.drop_rate}')

    def on_params(self, params):
        for p in params:
            if p.name == 'inject' and p.value:
                try:
                    text = self.fleet.inject_line(p.value)
                except ValueError as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().warn(text)
        return SetParametersResult(successful=True)

    def out(self, topic, sender, msg):
        if topic in self.pubs and sender != 'ros':
            self.pubs[topic].publish(msg)

    def inp(self, topic, msg):
        self.fleet.bus.publish(topic, 'ros', msg)

    def on_claim(self, m):
        if (not m.by and m.robot_id in self.ours) or m.by in self.ours:
            return                                 # our own claim, echoed by DDS
        self.inp('/swarm/claims', m)

    def tick(self):
        if self.rtf > 0:
            target = self.s0 + (wallclock.monotonic() - self.w0) * self.rtf
        else:
            target = self.now + 50 * self.dt
        while self.now + self.dt <= target:
            self.fleet.step(self.now, self.dt)
            self.fleet.bus.flush()
            self.now = round(self.now + self.dt, 6)
            if self.clock_on:
                self.pub_clock.publish(Clock(clock=to_time(self.now)))
        for t, text in self.fleet.drain_logs():
            self.get_logger().info(f'[{t:.1f}] {text}')


def main():
    rclpy.init()
    node = MockFleetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
