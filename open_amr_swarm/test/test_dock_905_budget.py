"""A docking controller failure (905, FAILED_TO_CONTROL) is retried without using up one of the 3 docking attempts.

Real-input fleet shifts c51-c55 (2026-10-25): every 905 came with the docking server's 'Could not get transform'
(the controller's odom -> base_footprint lookup at now() timing out under load), none with a collision, and three of
them in a row left a robot STUCK at its bay until an operator came (1-2 robots a shift). The old rule counted them as
attempts: 3 x 905 -> STUCK."""
import json

import pytest
import rclpy

from open_amr_msgs.msg import RobotState
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
    a.route, a.docking, a.leg, a.next_idx = [0, 1, 2, 3, 4], True, None, 4
    a.pose = (3.0, 0.0, 0.0)                                # at the staging node: a failure restages
    yield a
    a.destroy_node()
    rclpy.shutdown()


def test_control_failures_do_not_use_up_attempts(agent):
    for k in range(5):
        agent.dock_failed('n4', 'error 905', 905)
        assert agent.status != RobotState.STUCK and agent.dock_fails == 0 and agent.control_fails == k + 1
        assert agent.dock_state == 'restaging'
    agent.dock_failed('n4', 'error 905', 905)              # the budget is spent: this one counts
    assert agent.dock_fails == 1 and agent.status != RobotState.STUCK


def test_detection_failures_still_give_up_after_three(agent):
    for _ in range(3):
        agent.dock_failed('n4', 'error 904', 904)
    assert agent.status == RobotState.STUCK and agent.dock_state == 'gave_up'


def test_a_failure_without_a_code_counts(agent):
    agent.dock_failed('n4', 'goal rejected')
    assert agent.dock_fails == 1 and agent.control_fails == 0
