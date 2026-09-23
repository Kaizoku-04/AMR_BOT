#!/usr/bin/env python3
"""
Launch ROS 2 controllers for the OpenAMR robot.

This script creates a launch description that starts the necessary controllers
for operating the OpenAMR robot in a specific sequence.

Launched Controllers:
    1. Joint State Broadcaster: Publishes joint states to /joint_states
    2. Diff Drive Controller: Controls the robot's diff drive movements via ~/cmd_vel

author: Mohannad Rababah
date: Mars 30, 2026
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.

    The controller_manager `spawner` waits for the controller manager to come up
    (instead of a fixed start-up delay), then the diff drive controller is started
    once the joint state broadcaster is active.

    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager = LaunchConfiguration('controller_manager')
    timeout = LaunchConfiguration('controller_manager_timeout')

    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller,
                       '--controller-manager', controller_manager,
                       '--controller-manager-timeout', timeout],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')

    start_joint_state_broadcaster_cmd = spawner('joint_state_broadcaster')
    start_diff_drive_controller_cmd = spawner('diff_drive_controller')

    # Start the diff drive controller only after the joint state broadcaster is active
    load_diff_drive_after_jsb_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_diff_drive_controller_cmd]))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('controller_manager', default_value='/controller_manager',
                              description='Controller manager node (namespaced for multi-robot)'),
        DeclareLaunchArgument('controller_manager_timeout', default_value='120',
                              description='Seconds to wait for the controller manager'),
        start_joint_state_broadcaster_cmd,
        load_diff_drive_after_jsb_cmd,
    ])
