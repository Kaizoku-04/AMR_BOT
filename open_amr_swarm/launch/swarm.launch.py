"""Swarm layer for N robots: one swarm_agent per robot (namespace amr_i, namespaced TF) + the mission generator.

    ros2 launch open_amr_swarm swarm.launch.py robots:=6 mode:=mission duration_s:=600
Needs the robots' Nav2 stacks running (OpenAMR sim/ros/open_amr_nav2_isaac.launch.py namespace:=amr_i, lanes on)
and the lane graph from OpenAMR sim/scripts/generate_route_graph.py; files are found via $OPENAMR_ROOT
(set by OpenAMR tools/setup/openamr_env.sh) unless `graph`/`graph_nodes` are given.
battery:=true adds a simulated BMS per robot (battery_sim -> /amr_i/battery_state); battery_time_scale compresses
battery time for demos (20 = an 8 h shift in 24 min); battery_start = comma-separated start charges, default a
spread 90 % .. 40 % (a fleet mid-shift, so robots don't all need a charger at once).
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
    nodes = lc('graph_nodes') or os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.yaml')
    n = int(lc('robots'))
    scale = float(lc('battery_time_scale'))
    start = [float(v) for v in lc('battery_start').split(',') if v.strip()] or \
        [round(0.9 - 0.5 * i / max(n - 1, 1), 2) for i in range(n)]
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    actions = [Node(package='open_amr_swarm', executable='swarm_agent', namespace=f'amr_{i}', name='swarm_agent',
                    output='screen',
                    parameters=[{'robot_id': f'amr_{i}', 'graph': graph, 'drain_prior_per_h': 0.15 * scale,
                                 'use_sim_time': True}],
                    remappings=tf)
               for i in range(n)]
    if lc('battery') == 'true':
        actions += [Node(package='open_amr_swarm', executable='battery_sim', namespace=f'amr_{i}', name='battery_sim',
                         output='screen',
                         parameters=[{'graph': graph, 'time_scale': scale, 'initial_soc': start[i % len(start)],
                                      'use_sim_time': True}],
                         remappings=tf)
                    for i in range(n)]
    if lc('mode') != 'none':
        actions.append(Node(package='open_amr_swarm', executable='mission_generator', name='mission_generator',
                            output='screen',
                            parameters=[{'mode': lc('mode'), 'graph_nodes': nodes, 'duration_s': float(lc('duration_s')),
                                         'open_tasks': int(lc('open_tasks') or n + 2), 'out_dir': lc('out_dir'),
                                         'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='6'),
        DeclareLaunchArgument('mode', default_value='mission', description='mission | goto_random | none'),
        DeclareLaunchArgument('duration_s', default_value='0', description='sim seconds, 0 = run forever'),
        DeclareLaunchArgument('open_tasks', default_value='', description='default robots + 2: nobody idles for lack of work'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/swarm_metrics'),
        DeclareLaunchArgument('battery', default_value='true', description='simulated BMS per robot'),
        DeclareLaunchArgument('battery_time_scale', default_value='1.0'),
        DeclareLaunchArgument('battery_start', default_value='', description='e.g. 0.9,0.5,...; default spread 90..40 %'),
        DeclareLaunchArgument('graph', default_value=''),
        DeclareLaunchArgument('graph_nodes', default_value=''),
        OpaqueFunction(function=spawn),
    ])
