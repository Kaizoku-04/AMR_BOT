"""Swarm layer for N robots: one swarm_agent per robot (namespace amr_i, namespaced TF) + the mission generator.

    ros2 launch open_amr_swarm swarm.launch.py robots:=6 mode:=mission duration_s:=600
Needs the robots' Nav2 stacks running (OpenAMR sim/ros/open_amr_nav2_isaac.launch.py namespace:=amr_i, lanes on)
and the lane graph from OpenAMR sim/scripts/generate_route_graph.py; files are found via $OPENAMR_ROOT
(set by OpenAMR tools/setup/openamr_env.sh) unless `graph`/`graph_nodes` are given.
battery:=true adds a simulated BMS per robot (battery_sim -> /amr_i/battery_state); battery_time_scale compresses
battery time for demos (20 = an 8 h shift in 24 min); battery_start = comma-separated start charges, default a
spread 90 % .. 40 % (a fleet mid-shift, so robots don't all need a charger at once).
chargers:=charge_0,charge_1,charge_2 puts only those chargers in service (the rest are dead docks): fleets with
fewer chargers than robots park idle robots on the parking row and hand chargers to robots low on battery.
mode:=mission also starts a station_agent per arm (arm_receiving at receiving_bay, arm_outbound at outbound_bay):
receiving_pallets / outbound_pallets = boxes on each staged pallet at start (default a dock mid-shift: receiving 8 and 24
boxes, outbound 16 and 0, so a pallet runs empty / fills up early in a run), pallet_swap_s = truck restock / pickup
time, arm_cycle_s = one transfer; fault:=arm_receiving:300:120 takes that arm out of service at 300 s for 120 s.
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
    chargers = [c.strip() for c in lc('chargers').split(',') if c.strip()] or ['']
    actions = [Node(package='open_amr_swarm', executable='swarm_agent', namespace=f'amr_{i}', name='swarm_agent',
                    output='screen',
                    parameters=[{'robot_id': f'amr_{i}', 'graph': graph, 'drain_prior_per_h': 0.15 * scale,
                                 'chargers': chargers, 'fleet': [f'amr_{k}' for k in range(n)],
                                 'use_sim_time': True}],
                    remappings=tf)
               for i in range(n)]
    if lc('battery') == 'true':
        actions += [Node(package='open_amr_swarm', executable='battery_sim', namespace=f'amr_{i}', name='battery_sim',
                         output='screen',
                         parameters=[{'graph': graph, 'time_scale': scale, 'initial_soc': start[i % len(start)],
                                      'chargers': chargers, 'use_sim_time': True}],
                         remappings=tf)
                    for i in range(n)]
    if lc('mode') == 'mission':
        fault = lc('fault').split(':') if lc('fault') else ['', '0', '0']
        for sid, role, bay, pallets in (('arm_receiving', 'receiving', 'receiving_bay', lc('receiving_pallets')),
                                        ('arm_outbound', 'outbound', 'outbound_bay', lc('outbound_pallets'))):
            f = fault[0] == sid
            actions.append(Node(package='open_amr_swarm', executable='station_agent', name=f'station_{sid}',
                                output='screen',
                                parameters=[{'station_id': sid, 'role': role, 'bay': bay, 'graph': graph,
                                             'pallets': [int(v) for v in pallets.split(',')],
                                             'cycle_s': float(lc('arm_cycle_s')), 'swap_s': float(lc('pallet_swap_s')),
                                             'fault_at_s': float(fault[1]) if f else 0.0,
                                             'fault_for_s': float(fault[2]) if f else 0.0, 'use_sim_time': True}]))
    if lc('mode') != 'none':
        actions.append(Node(package='open_amr_swarm', executable='mission_generator', name='mission_generator',
                            output='screen',
                            parameters=[{'mode': lc('mode'), 'graph_nodes': nodes, 'graph': graph,
                                         'duration_s': float(lc('duration_s')),
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
        DeclareLaunchArgument('chargers', default_value='', description='chargers in service, e.g. charge_0,charge_1; default all'),
        DeclareLaunchArgument('receiving_pallets', default_value='8,24'),
        DeclareLaunchArgument('outbound_pallets', default_value='16,0'),
        DeclareLaunchArgument('pallet_swap_s', default_value='180'),
        DeclareLaunchArgument('arm_cycle_s', default_value='8.0'),
        DeclareLaunchArgument('fault', default_value='', description='station:start_s:duration_s, e.g. arm_receiving:300:120'),
        DeclareLaunchArgument('graph', default_value=''),
        DeclareLaunchArgument('graph_nodes', default_value=''),
        OpaqueFunction(function=spawn),
    ])
