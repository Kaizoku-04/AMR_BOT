#!/usr/bin/env python3
"""
Launch file for diff_parameters_node.

This launch file launches the diff_parameters_node with configurable parameters.

:author: Mohannad Rababah
:date: Mars 30, 2026
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Generate a launch description for the diff_parameters_node.

    :return: A LaunchDescription object containing the actions to execute.
    :rtype: LaunchDescription
    """

    # Constants for paths to different files and folders
    package_name_system_tests = 'open_amr_system_tests'

    diff_params_path = 'config/diff_parameters.yaml'

    # Set the path to different files and folders
    pkg_share_system_tests = FindPackageShare(
        package=package_name_system_tests).find(package_name_system_tests)

    diff_params_path = os.path.join(pkg_share_system_tests, diff_params_path)

    # Launch configuration variables
    diff_params = LaunchConfiguration('diff_params')

    # Declare the launch arguments
    declare_diff_params_cmd = DeclareLaunchArgument(
        name='diff_params',
        default_value=diff_params_path,
        description='Full path to parameters for the parameters file.')

    # Launch the diff_parameters_node
    start_diff_parameters_node_cmd = Node(
        package=package_name_system_tests,
        executable='diff_parameters_node',
        parameters=[diff_params],
        output='screen')

    # Create the launch description and populate it
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_diff_params_cmd)

    # Add actions to the launch description
    ld.add_action(start_diff_parameters_node_cmd)

    return ld
