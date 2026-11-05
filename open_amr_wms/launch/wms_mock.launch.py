"""open_amr_wms against the mock fleet, on /clock: ros2 launch open_amr_wms wms_mock.launch.py rtf:=20 db:=/tmp/wms.db"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn(context):
    lc = lambda k: LaunchConfiguration(k).perform(context)
    root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
    graph = lc('graph') or os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.geojson')
    n = int(lc('robots'))
    return [Node(package='open_amr_fleet_mock', executable='mock_fleet', name='mock_fleet', output='screen',
                 parameters=[{'graph': graph, 'robots': n, 'rtf': float(lc('rtf')), 'scenario': lc('scenario')}]),
            Node(package='open_amr_wms', executable='wms', name='wms', output='screen',
                 parameters=[{'graph': graph, 'db': lc('db'), 'open_tasks': n + 2, 'use_sim_time': True}])]


def generate_launch_description():
    args = [('robots', '6'), ('rtf', '20.0'), ('scenario', ''), ('db', '/tmp/wms.db'), ('graph', '')]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] + [OpaqueFunction(function=spawn)])
