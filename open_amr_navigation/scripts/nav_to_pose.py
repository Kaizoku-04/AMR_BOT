#!/usr/bin/env python3
"""
ROS 2 node for navigating to a goal pose and publishing status updates.

This script creates a ROS 2 node that:
    - Subscribes to a goal pose
    - Navigates the robot to the goal pose
    - Publishes the estimated time of arrival
    - Publishes the goal status
    - Cancels the goal if a stop signal is received

Subscription Topics:
    goal_pose/goal (geometry_msgs/PoseStamped): The desired goal pose
    stop/navigation/go_to_goal_pose (std_msgs/Bool): Signal to stop navigation
    cmd_vel (geometry_msgs/Twist): Velocity command

Publishing Topics:
    goal_pose/eta (std_msgs/String): Estimated time of arrival in seconds
    goal_pose/status (std_msgs/String): Goal pose status

Topic names are relative and Nav2 is addressed in the node's namespace, so one
instance per robot works (e.g. `--ros-args -r __ns:=/amr_3`).

:author: Mohannad Rababah
:date: Mars 30, 2026
"""

import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, PoseStamped

COSTMAP_CLEARING_PERIOD = 0.5  # seconds without forward progress before clearing costmaps
FEEDBACK_PERIOD = 0.1  # seconds between feedback polls


class GoToGoalPose(Node):
    """Navigate to goal poses and publish ETA / status.

    The goal callback runs in its own callback group and blocks while the goal is
    active; stop requests and velocity updates are handled concurrently in a
    reentrant group. Shared state is guarded by a lock / event instead of globals.
    """

    def __init__(self):
        """Constructor."""
        super().__init__('go_to_goal_pose')

        self._lock = threading.Lock()
        self._nav_in_progress = False
        self._moving_forward = True
        self._stop_requested = threading.Event()

        goal_group = MutuallyExclusiveCallbackGroup()
        monitor_group = ReentrantCallbackGroup()

        self.publisher_eta = self.create_publisher(String, 'goal_pose/eta', 10)
        self.publisher_status = self.create_publisher(String, 'goal_pose/status', 10)

        self.create_subscription(
            PoseStamped, 'goal_pose/goal', self.go_to_goal_pose, 10,
            callback_group=goal_group)
        self.create_subscription(
            Bool, 'stop/navigation/go_to_goal_pose', self.set_stop_navigation, 10,
            callback_group=monitor_group)
        self.create_subscription(
            Twist, 'cmd_vel', self.get_current_velocity, 1,
            callback_group=monitor_group)

        self.last_clear_time = self.get_clock().now()

        # Nav2 lives in the same namespace as this node
        self.navigator = BasicNavigator(namespace=self.get_namespace().strip('/'))
        self.navigator.waitUntilNav2Active()

    def set_stop_navigation(self, msg):
        """Request cancellation of the active goal."""
        with self._lock:
            active = self._nav_in_progress
        if active and msg.data:
            self._stop_requested.set()
            self.get_logger().info('Navigation cancellation request received by ROS 2...')

    def get_current_velocity(self, msg):
        """Track whether the robot is making forward progress."""
        with self._lock:
            self._moving_forward = msg.linear.x > 0.0

    def _publish_status(self, text):
        msg_status = String()
        msg_status.data = text
        self.publisher_status.publish(msg_status)

    def go_to_goal_pose(self, msg):
        """Go to goal pose."""
        self._stop_requested.clear()
        with self._lock:
            self._nav_in_progress = True

        # Clear all costmaps before sending to a goal
        self.navigator.clearAllCostmaps()
        self.navigator.goToPose(msg)

        while rclpy.ok() and not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            if feedback:
                eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                msg_eta = String()
                msg_eta.data = f"{eta:.0f}"
                self.publisher_eta.publish(msg_eta)
                self._publish_status("IN_PROGRESS")

                # If we are making no forward progress, clear all costmaps periodically
                now = self.get_clock().now()
                with self._lock:
                    moving_forward = self._moving_forward
                if (not moving_forward and
                        (now - self.last_clear_time).nanoseconds * 1e-9 > COSTMAP_CLEARING_PERIOD):
                    self.navigator.clearAllCostmaps()
                    self.last_clear_time = now

            # Wait for the next poll, waking immediately if a stop is requested
            if self._stop_requested.wait(FEEDBACK_PERIOD):
                self.navigator.cancelTask()
                self._stop_requested.clear()
                self.get_logger().info('Navigation cancellation request fulfilled...')

        with self._lock:
            self._nav_in_progress = False

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Successfully reached the goal!')
            self._publish_status("SUCCEEDED")
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
            self._publish_status("CANCELED")
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
            self._publish_status("FAILED")
        else:
            self.get_logger().info('Goal has an invalid return status!')
            self._publish_status("INVALID")


def main(args=None):
    """Main function to initialize and run the ROS 2 node."""
    rclpy.init(args=args)

    try:
        go_to_goal_pose = GoToGoalPose()
        executor = MultiThreadedExecutor()
        executor.add_node(go_to_goal_pose)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            go_to_goal_pose.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
