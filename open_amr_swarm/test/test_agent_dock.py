"""Docking failures: back to the staging node and dock again; after 3 attempts STUCK until an operator acknowledges
(fleet run 2026-10-03: a robot retried 79 times from a pose 1.8 m off the bay, the marker out of view)."""
import json

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import OperatorCommand, RobotState
from open_amr_swarm.agent import SwarmAgent


@pytest.fixture
def agent(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(6)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(5)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_0', '-p', 'fleet:=["amr_0", "amr_1"]'])
    a = SwarmAgent()
    a.route, a.docking, a.leg, a.next_idx = [0, 1, 2, 3, 4], True, None, 4   # at the staging node n3, bay n4
    a.dock_sent = []
    a.dock_client.server_is_ready = lambda: True
    a.dock_client.send_goal_async = lambda g: a.dock_sent.append(g) or type('F', (), {'add_done_callback': lambda s, cb: None})()
    yield a
    a.destroy_node()
    rclpy.shutdown()


def test_failed_dock_goes_back_to_staging_then_docks_again(agent):
    agent.dock_failed('n4', 'error 904')
    assert agent.dock_state == 'restaging' and agent.next_idx == 3      # drive to the staging node n3 again
    assert agent.status != RobotState.STUCK
    agent.next_idx = 4                                                  # ... reached it: the leg logic docks
    agent.start_dock()
    assert len(agent.dock_sent) == 1 and agent.dock_state == 'sending'


def test_three_failed_attempts_stop_and_wait_for_an_operator(agent):
    for k in range(3):
        agent.dock_failed('n4', 'error 904')
    assert agent.status == RobotState.STUCK and agent.dock_state == 'gave_up'
    agent.next_idx = 4
    for _ in range(5):
        agent.start_dock()
    assert agent.dock_sent == []                                        # no more attempts on its own
    agent.on_operator(OperatorCommand(command=OperatorCommand.ACK_FAULT, target='amr_1', stamp=TimeMsg(sec=1)))
    assert agent.dock_state == 'gave_up'                                # someone else's acknowledgement
    agent.on_operator(OperatorCommand(command=OperatorCommand.ACK_FAULT, target='amr_0', stamp=TimeMsg(sec=1)))
    assert agent.dock_state == 'restaging' and agent.dock_fails == 0 and agent.status != RobotState.STUCK


def test_inaccurate_dock_redocks_from_where_the_undock_left_it(agent):
    agent.dock_failed('n4', 'inaccurate dock, re-docking')              # the undock already backed out to staging
    assert agent.dock_state == 'retry' and agent.next_idx == 4
