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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
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
        self.odom = deque(maxlen=300)          # 5 s at 60 Hz
        self.mount = None                       # (x, y, yaw) of the lidar in base_footprint
        self.base_frame = dp('base_frame', 'base_footprint').value
        # odometry on its own thread: behind a slow scan fit in one thread, it lagged the (newest-only) scans by more
        # than the extrapolation allowance — "no odometry at the scan time" for up to a third of them (Isaac 2026-10-03)
        self.create_subscription(Odometry, 'odom', self.on_odom, 50, callback_group=MutuallyExclusiveCallbackGroup())
        self.range = dp('activation_range', 2.5).value
        self.fov = math.radians(dp('fov_deg', 120.0).value) / 2
        self.line_tol = dp('line_tol', 0.008).value
        self.max_rms = dp('max_rms', 0.006).value
        self.tf = Buffer()
        TransformListener(self.tf, self, spin_thread=True)   # own thread: lookups below wait for TF
        self.pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)
        # only the newest scan: a 33 Hz safety scanner's 1,350-point scans outpace this Python fit on a loaded PC, and a
        # queue of old scans ran past the odometry window ("no odometry at the scan time" for 40 % of the scans,
        # docking aborted, Isaac 2026-10-03). The docking server needs ~10 Hz of fresh detections, not every scan.
        self.create_subscription(LaserScan, 'scan', self.on_scan, QoSProfile(
            depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.found, self.missed, self.last_why = 0, 0, ''
        # scans dropped before a fit (not counted in `missed`; a dock test lost the detection for > 1 s while this
        # reported 0 misses, 2026-10-05): tracking without odometry at the scan time, marker outside the sector of
        # the map-based prediction, no map -> lidar transform. Plus the longest gap between detections while
        # tracking and the longest processing lag (receive - scan stamp).
        self.skip = {'track_no_odom': 0, 'out_of_sector': 0, 'no_tf': 0}
        self.max_gap, self.max_lag = 0.0, 0.0
        # scan callbacks themselves: how many ran and the longest pause between two (sim time) — the docking server
        # declared a detection lost (> 1 s) while every processed scan had produced a detection, i.e. no callback ran
        # (2026-10-05): scans not arriving, or this node's executor starved
        self.scans, self.max_cb_gap, self.last_cb = 0, 0.0, None
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'{len(self.markers)} markers: {", ".join(self.markers)}')

    def report(self):
        if self.found or self.missed:
            skips = ', '.join(f'{k} {v}' for k, v in self.skip.items() if v)
            self.get_logger().info(f'markers: {self.found} detections, {self.missed} scans near one without'
                                   + (f' (last: {self.last_why})' if self.missed else '')
                                   + f' | longest gap {self.max_gap:.2f} s, lag {self.max_lag * 1000:.0f} ms'
                                   + f' | {self.scans} scans, longest pause between scans {self.max_cb_gap:.2f} s'
                                   + (f' | skipped: {skips}' if skips else ''))
        self.found = self.missed = 0
        self.skip = {k: 0 for k in self.skip}
        self.max_gap = self.max_lag = 0.0
        self.scans, self.max_cb_gap = 0, 0.0

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
        ts = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        self.max_lag = max(self.max_lag, now - ts)
        self.scans += 1
        if self.last_cb is not None:
            self.max_cb_gap = max(self.max_cb_gap, now - self.last_cb)
        self.last_cb = now
        best = self.track(ts)
        if best is not None:
            return self.fit(scan, ts, best)
        last = getattr(self, 'last_det', None)
        if last is not None and 0.0 <= ts - last[0] < 1.0:
            self.skip['track_no_odom'] += 1                 # tracking, but no odometry at the scan time
        # the prediction only has to put the region of interest within ~0.3 m: the latest map -> lidar transform will
        # do (at the scan's own stamp the lookup mostly fails: AMCL publishes map -> odom slower than the lidar scans)
        try:
            t = self.tf.lookup_transform(scan.header.frame_id, self.map_frame, rclpy.time.Time())
        except Exception as e:
            self.tf_fail = getattr(self, 'tf_fail', 0) + 1
            self.skip['no_tf'] += 1
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
            if last is not None and ts - last[0] < 5.0:     # was docking a moment ago: count it
                self.skip['out_of_sector'] += 1
            return
        return self.fit(scan, ts, best)

    def lidar_in_odom(self, ts):
        """(x, y, yaw) of the lidar in odom at time ts, or None."""
        b = self.odom_at(ts)
        if b is None:
            return None
        bx, by, byaw = b
        mx, my, myaw = self.mount
        return (bx + math.cos(byaw) * mx - math.sin(byaw) * my, by + math.sin(byaw) * mx + math.cos(byaw) * my,
                byaw + myaw)

    def track(self, ts):
        """Once a marker is detected, predict it in the next scans from that detection and odometry (good to mm
        over a second), not from the map pose: near the bay the marker is 0.15-0.3 m from a nose-mounted scanner and
        AMCL's normal 0.1-0.2 m error swung the map-based prediction out of the sector — scans were dropped
        silently and docking aborted on a lost detection (Isaac 2026-10-03)."""
        last = getattr(self, 'last_det', None)
        if last is None or not 0.0 <= ts - last[0] < 1.0:
            return None
        lp = self.lidar_in_odom(ts)
        if lp is None:
            return None
        _, X, Y, Yaw, name = last
        c, s = math.cos(-lp[2]), math.sin(-lp[2])
        px, py = c * (X - lp[0]) - s * (Y - lp[1]), s * (X - lp[0]) + c * (Y - lp[1])
        return (math.hypot(px, py), name, (px, py, Yaw - lp[2]), self.markers[name])

    def fit(self, scan, ts, best):
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
        lp = self.lidar_in_odom(ts)                        # base in odom, then the lidar mount, then the marker
        if lp is None:
            self.missed += 1
            self.last_why = 'no odometry at the scan time'
            return
        lx, ly, lyaw = lp
        co, so = math.cos(lyaw), math.sin(lyaw)
        x, y, yaw = lx + co * pose[0] - so * pose[1], ly + so * pose[0] + co * pose[1], pose[2] + lyaw
        self.found += 1
        prev = getattr(self, 'last_det', None)
        if prev is not None and ts - prev[0] < 5.0:
            self.max_gap = max(self.max_gap, ts - prev[0])
        self.last_det = (ts, x, y, yaw, name)
        msg = PoseStamped()
        msg.header.stamp, msg.header.frame_id = scan.header.stamp, self.out_frame
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BayMarkerDetector()
    try:
        ex = MultiThreadedExecutor(num_threads=2)
        ex.add_node(node)
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
