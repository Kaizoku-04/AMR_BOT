"""Docked at a bay with AMCL 0.42 m off (fleet run c41_base, 2026-10-21): the cell's interlock compares the reported pose
with the bay pose (0.30 m), so the arm waited all shift for a robot that was docked to the millimetre. The agent puts
AMCL back on the bay's pose (lane graph + the marker-measured dock error), and keeps checking while it stays docked."""
import json
import math

import pytest
import rclpy

from open_amr_swarm.agent import SwarmAgent


@pytest.fixture
def agent(tmp_path):
    nodes = [('receiving_bay', 'bay', (-14.0, 7.0)), ('receiving_queue', 'queue', (-11.0, 7.0)),
             ('charge_0', 'charger', (0.0, 0.0))]
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': n, 'kind': k}},
              'geometry': {'type': 'Point', 'coordinates': list(xy)}} for i, (n, k, xy) in enumerate(nodes)]
    feats += [{'type': 'Feature', 'properties': {'id': 100, 'startid': 1, 'endid': 0, 'metadata': {'kind': 'street'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[-11, 7], [-14, 7]]]}}]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_1', '-p', 'fleet:=["amr_1"]'])
    a = SwarmAgent()
    a.t = 100.0
    a.now = lambda: a.t
    a.docked, a.dock_name = True, 'receiving_bay'
    a.pose = (-14.415, 6.978, math.radians(-179.1))         # c41_base amr_1: AMCL 0.42 m behind the truth
    a.dock_error = lambda: (0.002, -0.001, math.radians(0.5))
    a.sent = []
    a.pub_initialpose.publish = a.sent.append
    yield a
    a.destroy_node()
    rclpy.shutdown()


def test_docked_robot_with_amcl_off_is_put_back_on_the_bay(agent):
    agent.check_bay_localization()
    assert len(agent.sent) == 1
    p = agent.sent[0].pose.pose
    assert agent.sent[0].header.frame_id == 'map'
    assert abs(p.position.x - (-14.002)) < 0.003 and abs(p.position.y - 7.0) < 0.003


def test_reset_is_repeated_while_amcl_stays_off(agent):
    agent.check_bay_localization()
    agent.t += 1.0
    agent.check_bay_localization()
    assert len(agent.sent) == 1                              # not every tick
    agent.t += 5.0
    agent.check_bay_localization()
    assert len(agent.sent) == 2                              # still off after 5 s (a lost message): sent again


def test_no_reset_when_amcl_agrees(agent):
    agent.pose = (-14.03, 7.02, math.radians(-179.1))
    agent.check_bay_localization()
    assert agent.sent == []


def test_no_reset_when_the_dock_is_suspect(agent):
    agent.pose = (-12.5, 7.0, math.radians(-179.1))         # 1.5 m: the dock or the map is wrong, not AMCL
    agent.check_bay_localization()
    assert agent.sent == []


def test_only_at_a_bay_and_only_docked(agent):
    agent.dock_name = 'charge_0'
    agent.check_bay_localization()
    agent.dock_name, agent.docked = 'receiving_bay', False
    agent.t += 10.0
    agent.check_bay_localization()
    assert agent.sent == []
