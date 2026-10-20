"""A carton shifted on the deck: the agent stops and asks the deck to re-centre it. A request lost under load (DDS) is
sent again while the 4 s window is still open, instead of the robot going STUCK for a person to clear."""
import json

import pytest
import rclpy

from open_amr_msgs.msg import RobotState
from open_amr_swarm.agent import SwarmAgent


@pytest.fixture
def agent(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(3)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(2)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_0', '-p', 'fleet:=["amr_0"]',
                     '-p', 'deck_sensor:=true'])
    a = SwarmAgent()
    a.t = 10.0
    a.now = lambda: a.t
    a.loaded, a.phase, a.deck_load = True, 'driving', 2
    a.recenter_calls = 0
    a.recenter_cli.service_is_ready = lambda: True

    def call(req):
        a.recenter_calls += 1
    a.recenter_cli.call_async = call
    yield a
    a.destroy_node()
    rclpy.shutdown()


def test_lost_recenter_request_is_sent_again(agent):
    agent.watch_load()
    assert agent.phase == 'recentring' and agent.recenter_calls == 1
    agent.t += 1.0
    agent.watch_load()
    assert agent.recenter_calls == 1                    # the jog takes 1 s: not yet
    agent.t += 1.0
    agent.watch_load()
    assert agent.recenter_calls == 2                    # still shifted at 2 s: the first request was lost
    agent.deck_load = 1
    agent.t += 0.5
    agent.watch_load()
    assert agent.phase == 'driving' and agent.status != RobotState.STUCK


def test_carton_that_will_not_recentre_still_stops_the_robot(agent):
    agent.watch_load()
    for _ in range(9):
        agent.t += 0.5
        agent.watch_load()
    assert agent.phase == 'load_fault' and agent.status == RobotState.STUCK
