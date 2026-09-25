"""battery_sim: stands in for the robot's BMS in simulation (one per robot, in its namespace).

Publishes sensor_msgs/BatteryState on `battery_state` at 2 Hz — the same interface a real BMS driver would give the
swarm agent, so nothing above it knows it's simulated. Energy model and numbers: energy.py (BatteryParams).
Charging contacts close when the robot stands still on a charger: within `contact_tol_m` of a lane-graph node of
kind `charger` (map pose from TF) and facing into the dock (the charger's incoming lane heading) within 30 deg.
Speed and turn rate come from `odom`. Uses sim time.
"""
import math

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState

from .energy import BatteryParams, step_soc
from .lane_graph import LaneGraph


class BatterySim(Node):
    def __init__(self):
        super().__init__('battery_sim')
        dp = self.declare_parameter
        graph = LaneGraph(dp('graph', '').value)
        self.p = BatteryParams(capacity_wh=dp('capacity_wh', 1200.0).value, charge_w=dp('charge_w', 900.0).value,
                               time_scale=dp('time_scale', 1.0).value)
        self.soc = dp('initial_soc', 1.0).value
        self.tol = dp('contact_tol_m', 0.3).value
        # charger pose + the heading a robot has when it's in the dock (along its incoming lane)
        self.docks = []
        in_service = [c for c in dp('chargers', ['']).value if c]      # default: every charger in the graph
        for c in ([graph.id(n) for n in in_service] or graph.of_kind('charger')):
            inc = [a for a, succ in graph.succ.items() if c in succ]
            self.docks.append((graph.pos[c], graph.heading(inc[0], c) if inc else None))
        self.v = self.w = 0.0
        self.pose, self.contact, self.last_t = None, False, None
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'battery sim: {self.p.capacity_wh:.0f} Wh, start {100 * self.soc:.0f} %, '
                               f'time x{self.p.time_scale:g}, {len(self.docks)} chargers')

    def on_odom(self, m):
        lin = m.twist.twist.linear          # magnitude only: independent of the frame the twist is expressed in
        self.v, self.w = math.hypot(lin.x, lin.y), m.twist.twist.angular.z

    def on_charger(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return self.contact
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if abs(self.v) > 0.03 or abs(self.w) > 0.1:
            return False
        return any(math.hypot(cx - x, cy - y) < self.tol and
                   (hd is None or abs(math.remainder(yaw - hd, math.tau)) < math.radians(30))
                   for (cx, cy), hd in self.docks)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0:
            return
        dt = 0.0 if self.last_t is None else min(now - self.last_t, 2.0)
        self.last_t = now
        contact = self.on_charger()
        if contact != self.contact:
            self.get_logger().info(f"charger contacts {'closed' if contact else 'open'} at {100 * self.soc:.1f} %")
            self.contact = contact
        self.soc, amps = step_soc(self.soc, dt, self.v, self.w, contact, self.p)
        m = BatteryState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.voltage = float(self.p.nominal_v + 2.4 * (self.soc - 0.5))       # LiFePO4 8S: flat 22.8..25.2 V
        m.current = float(amps)
        cap_ah = self.p.capacity_wh / self.p.nominal_v
        m.charge, m.capacity, m.design_capacity = float(self.soc * cap_ah), float(cap_ah), float(cap_ah)
        m.percentage = float(self.soc)
        m.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_FULL if contact and self.soc >= 0.999 else
                                 BatteryState.POWER_SUPPLY_STATUS_CHARGING if contact else
                                 BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        m.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        m.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        m.present = True
        m.location = 'main'
        self.pub.publish(m)


def main():
    rclpy.init()
    node = BatterySim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
