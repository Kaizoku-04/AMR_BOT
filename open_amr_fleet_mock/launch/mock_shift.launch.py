"""A whole shift without Isaac: the mock fleet (6 robots + 2 arm stations, driving /clock) and mission_generator.

    ros2 launch open_amr_fleet_mock mock_shift.launch.py rtf:=20 duration_s:=3600 scenario:=<file.yaml>
wms:=mission_generator (default) | none (start your own WMS with use_sim_time:=true).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawn(context):
    lc = lambda k: LaunchConfiguration(k).perform(context)
    root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
    graph = lc('graph') or os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.geojson')
    nodes = graph.replace('.geojson', '.yaml')
    n = int(lc('robots'))
    acts = [Node(package='open_amr_fleet_mock', executable='mock_fleet', name='mock_fleet', output='screen',
                 parameters=[{'graph': graph, 'robots': n, 'rtf': float(lc('rtf')), 'seed': int(lc('seed')),
                              'scenario': lc('scenario'), 'drop_rate': float(lc('drop_rate'))}])]
    if lc('wms') == 'mission_generator':
        acts.append(Node(package='open_amr_swarm', executable='mission_generator', name='mission_generator',
                         output='screen',
                         parameters=[{'mode': 'mission', 'graph_nodes': nodes, 'graph': graph,
                                      'duration_s': float(lc('duration_s')), 'open_tasks': n + 2,
                                      'out_dir': lc('out_dir'), 'use_sim_time': True}]))
    return acts


def generate_launch_description():
    args = [('robots', '6'), ('rtf', '20.0'), ('seed', '1'), ('scenario', ''), ('drop_rate', '0.0'),
            ('wms', 'mission_generator'), ('duration_s', '0'), ('out_dir', '/tmp/swarm_metrics'), ('graph', '')]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] + [OpaqueFunction(function=spawn)])
