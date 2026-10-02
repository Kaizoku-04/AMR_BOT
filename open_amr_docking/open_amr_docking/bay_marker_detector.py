"""bay_marker_detector: the robot's lidar finds the V-marker of the dock it approaches and publishes its pose for the
Nav2 docking server (SimpleNonChargingDock with use_external_detection_pose, topic `detected_dock_pose`).

For every scan: the markers of the dock database (sim/worlds/warehouse_mission_docks.yaml `markers:`) within
`activation_range` and inside the lidar's field of view are predicted in the lidar frame from TF (map -> lidar, i.e.
the robot's localization), and the V is fitted to the scan around the prediction (v_marker.detect). The pose
published is the marker frame (apex, x into the marker = the docked heading) in `output_frame` (odom), transformed
with the lidar's pose at the scan's time and stamped with it; the dock plugin turns it into the docking pose (external_detection_translation_x = -to_dock).
Nothing is published when no V is found (the docking server times out on a lost detection).

    ros2 run open_amr_docking bay_marker_detector --ros-args -r __ns:=/amr_0 -p docks_file:=.../docks.yaml
"""
import math
from collections import deque

import numpy as np
import rclpy
import rclpy.time
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .v_marker import detect


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class BayMarkerDetector(Node):
    def __init__(self):
        super().__init__('bay_marker_detector')
        dp = self.declare_parameter
        db = yaml.safe_load(open(dp('docks_file', '').value))
        self.markers = db.get('markers', {})
        self.map_frame = dp('map_frame', 'map').value
        # published in the docking server's fixed frame, transformed with the lidar pose *at the scan's time*: given in
        # the lidar frame, the server transforms it with the latest TF, so while the robot moves the dock lands
        # speed x latency too far ahead; its smoothing filter kept that, and the robot stopped 5-15 mm past the bay
        # (dock test, 2026-10-02)
        self.out_frame = 'odom'
        # the pose at the scan's time comes from the odometry stream (60 Hz) interpolated here, plus the static
        # base -> lidar mount: rclpy's TF listener fell 0.2-0.7 s behind ~300 TF messages/s on the loaded fleet
        # machine (C++ listeners keep up), so lookups at the scan time failed or extrapolated
        self.odom = deque(maxlen=120)
        self.mount = None                       # (x, y, yaw) of the lidar in base_footprint
        self.base_frame = dp('base_frame', 'base_footprint').value
        self.create_subscription(Odometry, 'odom', self.on_odom, 50)
        self.range = dp('activation_range', 2.5).value
        self.fov = math.radians(dp('fov_deg', 120.0).value) / 2
        self.line_tol = dp('line_tol', 0.008).value
        self.max_rms = dp('max_rms', 0.006).value
        self.tf = Buffer()
        TransformListener(self.tf, self, spin_thread=True)   # own thread: lookups below wait for TF
        self.pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.found, self.missed, self.last_why = 0, 0, ''
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'{len(self.markers)} markers: {", ".join(self.markers)}')

    def report(self):
        if self.found or self.missed:
            self.get_logger().info(f'markers: {self.found} detections, {self.missed} scans near one without'
                                   + (f' (last: {self.last_why})' if self.missed else ''))
        self.found = self.missed = 0

    def on_odom(self, m):
        p = m.pose.pose
        self.odom.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x, p.position.y,
                          yaw_of(p.orientation)))

    def odom_at(self, t):
        """Odometry pose (x, y, yaw) at time t: interpolated, or extrapolated up to 0.1 s past the last sample."""
        o = list(self.odom)
        if not o or t > o[-1][0] + 0.1 or t < o[0][0]:
            return None
        if t > o[-1][0] and len(o) >= 2:                  # the scan came in before its odometry: extrapolate
            a, b = o[-2], o[-1]
            f = (t - a[0]) / max(b[0] - a[0], 1e-9)
            return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
                    a[3] + f * math.remainder(b[3] - a[3], math.tau))
        for a, b in zip(o, o[1:]):
            if a[0] <= t <= b[0]:
                f = (t - a[0]) / max(b[0] - a[0], 1e-9)
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]),
                        a[3] + f * math.remainder(b[3] - a[3], math.tau))
        return o[-1][1:]

    def on_scan(self, scan):
        if self.mount is None:
            try:
                m = self.tf.lookup_transform(self.base_frame, scan.header.frame_id, rclpy.time.Time())
                self.mount = (m.transform.translation.x, m.transform.translation.y, yaw_of(m.transform.rotation))
            except Exception:
                return
        # the prediction only has to put the region of interest within ~0.3 m: the latest map -> lidar transform will
        # do (at the scan's own stamp the lookup mostly fails: AMCL publishes map -> odom slower than the lidar scans)
        try:
            t = self.tf.lookup_transform(scan.header.frame_id, self.map_frame, rclpy.time.Time())
        except Exception as e:
            self.tf_fail = getattr(self, 'tf_fail', 0) + 1
            if self.tf_fail % 100 == 1:
                self.get_logger().warn(f'no map -> {scan.header.frame_id} transform: {e}')
            return
        tx, ty, tyaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        c, s = math.cos(tyaw), math.sin(tyaw)
        best = None
        for name, m in self.markers.items():
            mx, my, myaw = m['pose']
            lx, ly = tx + c * mx - s * my, ty + s * mx + c * my       # marker apex in the lidar frame
            d = math.hypot(lx, ly)
            if d < self.range and abs(math.atan2(ly, lx)) < self.fov and (best is None or d < best[0]):
                best = (d, name, (lx, ly, myaw + tyaw), m)
        if best is None:
            return
        _, name, pred, m = best
        r = np.asarray(scan.ranges, float)
        a = scan.angle_min + scan.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
        pts = np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
        pose, info = detect(pts, pred, m['width'], m['depth'], line_tol=self.line_tol, max_rms=self.max_rms)
        if pose is None:
            self.missed += 1
            self.last_why = f'{name}: {info}'
            return
        ts = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        b = self.odom_at(ts)
        if b is None:
            self.missed += 1
            self.last_why = 'no odometry at the scan time'
            return
        bx, by, byaw = b                                   # base in odom, then the lidar mount, then the marker
        mx, my, myaw = self.mount
        lx, ly, lyaw = bx + math.cos(byaw) * mx - math.sin(byaw) * my, by + math.sin(byaw) * mx + math.cos(byaw) * my, \
            byaw + myaw
        co, so = math.cos(lyaw), math.sin(lyaw)
        x, y, yaw = lx + co * pose[0] - so * pose[1], ly + so * pose[0] + co * pose[1], pose[2] + lyaw
        self.found += 1
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = scan.header.stamp, self.out_frame
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BayMarkerDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
