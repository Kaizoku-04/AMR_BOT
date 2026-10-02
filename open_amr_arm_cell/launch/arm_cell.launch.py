"""One UR30 cell's control stack, in the cell's namespace (/<arm_id>): robot_state_publisher, ros2_control
(controller_manager + joint_state_broadcaster + joint_trajectory_controller) on the chosen hardware.

    ros2 launch open_amr_arm_cell arm_cell.launch.py arm_id:=arm_receiving hardware:=topic   # Isaac (sim time)
    ros2 launch open_amr_arm_cell arm_cell.launch.py arm_id:=arm_receiving hardware:=mock    # no simulator
    ros2 launch open_amr_arm_cell arm_cell.launch.py arm_id:=arm_receiving hardware:=ur robot_ip:=192.168.56.101 \
        use_sim_time:=false                                                                  # real UR30 / URSim
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


def ur_controllers(arm):
    """ur_robot_driver's controller config for a namespaced cell: keys as /** wildcards, no tf_prefix."""
    import tempfile
    share = get_package_share_directory('ur_robot_driver')
    text = open(os.path.join(share, 'config', 'ur_controllers.yaml')).read().replace('$(var tf_prefix)', '')
    data = yaml.safe_load(text)
    data['controller_manager']['ros__parameters']['update_rate'] = 500          # e-series (ur30_update_rate.yaml)
    out = {f'/**/{k}': v for k, v in data.items()}
    f = tempfile.NamedTemporaryFile('w', suffix=f'_{arm}_ur_controllers.yaml', delete=False)
    yaml.safe_dump(out, f)
    return f.name


def spawn(context):
    lc = lambda k: LaunchConfiguration(k).perform(context)
    share = get_package_share_directory('open_amr_arm_cell')
    root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
    layout = yaml.safe_load(open(os.path.join(root, 'sim', 'configs', 'warehouse_layout.yaml')))
    cell = layout.get('arm_cell', {})
    arm, sim = lc('arm_id'), lc('use_sim_time') == 'true'
    hw = lc('hardware')
    ports = {} if hw != 'ur' else {k: str(int(lc('port_base')) + i) for i, k in enumerate(
        ('reverse_port', 'script_sender_port', 'trajectory_port', 'script_command_port'))}
    desc = xacro.process_file(os.path.join(share, 'urdf', 'arm_cell.urdf.xacro'), mappings={
        **ports, 'robot_ip': lc('robot_ip'),
        'hardware': hw, 'arm_id': arm, 'ur_type': cell.get('ur_type', 'ur30'),
        'pedestal_height': str(cell.get('pedestal_height', 1.1)),
        'pedestal_size': str(cell.get('pedestal_size', 0.5)), 'tool_length': str(cell.get('tool_length', 0.2))}).toxml()
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    ctrl = os.path.join(share, 'config', 'ros2_controllers.yaml')
    if hw == 'ur':                       # the real arm / URSim: ur_robot_driver's hardware interface and controllers
        return [
            Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=arm, output='screen',
                 parameters=[{'robot_description': desc, 'use_sim_time': sim}], remappings=tf),
            Node(package='controller_manager', executable='ros2_control_node', namespace=arm, output='screen',
                 parameters=[ur_controllers(arm), {'use_sim_time': sim}],
                 remappings=tf + [('~/robot_description', 'robot_description')]),
            Node(package='ur_robot_driver', executable='dashboard_client', name='dashboard_client', namespace=arm,
                 output='screen', parameters=[{'robot_ip': lc('robot_ip')}]),
            Node(package='ur_robot_driver', executable='controller_stopper_node', name='controller_stopper',
                 namespace=arm, output='screen',
                 parameters=[{'headless_mode': True, 'joint_controller_active': True,
                              'consistent_controllers': ['io_and_status_controller', 'speed_scaling_state_broadcaster',
                                                         'joint_state_broadcaster']}]),
            Node(package='controller_manager', executable='spawner', namespace=arm, output='screen',
                 arguments=['joint_state_broadcaster', 'io_and_status_controller', 'speed_scaling_state_broadcaster',
                            'scaled_joint_trajectory_controller', '--controller-manager', f'/{arm}/controller_manager',
                            '--controller-manager-timeout', '120'],
                 parameters=[{'use_sim_time': sim}]),
        ]
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
        DeclareLaunchArgument('hardware', default_value='topic', description='topic (Isaac) | mock | ur (real / URSim)'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.56.101', description='hardware:=ur'),
        DeclareLaunchArgument('port_base', default_value='50001',
                              description='hardware:=ur: first of the 4 driver ports (one block per cell on a PC)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=spawn),
    ])
