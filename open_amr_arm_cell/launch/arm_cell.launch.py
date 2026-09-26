"""One UR10e cell's control stack, in the cell's namespace (/<arm_id>): robot_state_publisher, ros2_control
(controller_manager + joint_state_broadcaster + joint_trajectory_controller) on the chosen hardware.

    ros2 launch open_amr_arm_cell arm_cell.launch.py arm_id:=arm_receiving hardware:=topic   # Isaac (sim time)
    ros2 launch open_amr_arm_cell arm_cell.launch.py arm_id:=arm_receiving hardware:=mock    # no simulator
hardware:=topic needs the simulator's arm bridge (OpenAMR sim/scripts/open_amr_sim.py --arms): it subscribes to
/<arm_id>/isaac_joint_commands and publishes /<arm_id>/isaac_joint_states. A real cell runs ur_robot_driver instead.
TF is namespaced (/<arm_id>/tf) like the AMRs'.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def spawn(context):
    lc = lambda k: LaunchConfiguration(k).perform(context)
    share = get_package_share_directory('open_amr_arm_cell')
    root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
    layout = yaml.safe_load(open(os.path.join(root, 'sim', 'configs', 'warehouse_layout.yaml')))
    cell = layout.get('arm_cell', {})
    arm, sim = lc('arm_id'), lc('use_sim_time') == 'true'
    desc = xacro.process_file(os.path.join(share, 'urdf', 'arm_cell.urdf.xacro'), mappings={
        'hardware': lc('hardware'), 'arm_id': arm, 'pedestal_height': str(cell.get('pedestal_height', 1.1)),
        'pedestal_size': str(cell.get('pedestal_size', 0.5)), 'tool_length': str(cell.get('tool_length', 0.2))}).toxml()
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    ctrl = os.path.join(share, 'config', 'ros2_controllers.yaml')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=arm, output='screen',
             parameters=[{'robot_description': desc, 'use_sim_time': sim}], remappings=tf),
        Node(package='controller_manager', executable='ros2_control_node', namespace=arm, output='screen',
             parameters=[ctrl, {'use_sim_time': sim}] + (
                 [] if lc('hardware') == 'topic' else [os.path.join(share, 'config', 'ros2_controllers_mock.yaml')]),
             remappings=tf + [('~/robot_description', 'robot_description')]),
        Node(package='controller_manager', executable='spawner', namespace=arm, output='screen',
             arguments=['joint_state_broadcaster', 'joint_trajectory_controller', '--controller-manager',
                        f'/{arm}/controller_manager', '--controller-manager-timeout', '120'],
             parameters=[{'use_sim_time': sim}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_id', default_value='arm_receiving'),
        DeclareLaunchArgument('hardware', default_value='topic', description='topic (Isaac) | mock'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=spawn),
    ])
