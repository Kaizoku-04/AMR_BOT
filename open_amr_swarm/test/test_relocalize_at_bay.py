"""Docked at a bay with AMCL 0.42 m off (fleet run c41_base, 2026-10-21): the cell's interlock compares the reported pose
with the bay pose (0.30 m), so the arm waited all shift for a robot docked to the millimetre. The agent puts AMCL back
on the bay's pose (lane graph + the marker-measured dock error) BEFORE it reports docked: a reset after `docked` moved
the reported pose 0.12 m under a descending arm and the cell faulted (c41_base rerun, the same day)."""
import json
import math

import pytest
import rclpy

from open_amr_swarm.agent import SwarmAgent

BAY = (-14.0, 7.0)
AMCL_OFF = (-14.415, 6.978, math.radians(-179.1))          # c41_base amr_1: AMCL 0.42 m behind the truth


@pytest.fixture
def agent(tmp_path):
    nodes = [('receiving_bay', 'bay', BAY), ('receiving_queue', 'queue', (-11.0, 7.0)), ('charge_0', 'charger', (0, 0))]
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
    a.docking, a.dock_name, a.dock_state, a.dock_t0, a.dock_retries = True, 'receiving_bay', 'settling', 90.0, 0
    a.pose, a.speed_est = AMCL_OFF, 0.0
    a.dock_error = lambda: (0.002, -0.001, math.radians(0.5))
    a.sent = []
    a.pub_initialpose.publish = a.sent.append
    yield a
    a.destroy_node()
    rclpy.shutdown()


def amcl_takes(a):
    """AMCL applied the last reset: the next TF puts the robot where the reset said."""
    p = a.sent[-1].pose.pose
    a.pose = (p.position.x, p.position.y, a.pose[2])


def test_reset_comes_before_docked(agent):
    agent.check_dock('receiving_bay')
    assert len(agent.sent) == 1 and not agent.docked       # not docked yet: the cell must not latch the old pose
    p = agent.sent[0].pose.pose
    assert agent.sent[0].header.frame_id == 'map'
    assert abs(p.position.x - (-14.002)) < 0.003 and abs(p.position.y - 7.0) < 0.003
    amcl_takes(agent)
    agent.t += 0.2
    assert agent.final_reached(*agent.pose[:2])
    assert agent.docked
    d = math.hypot(agent.pose[0] - BAY[0], agent.pose[1] - BAY[1])
    assert d < 0.01                                          # what the cell sees at `docked`: the bay pose
    n = len(agent.sent)
    for _ in range(50):                                      # docked under the arm: never moved again
        agent.t += 0.2
        agent.final_reached(*agent.pose[:2])
    assert len(agent.sent) == n


def test_lost_reset_is_resent_then_gives_up_reporting_docked(agent):
    agent.check_dock('receiving_bay')
    agent.t += 1.0
    agent.final_reached(*agent.pose[:2])
    assert len(agent.sent) == 1                              # in flight
    agent.t += 1.0
    agent.final_reached(*agent.pose[:2])
    assert len(agent.sent) == 2 and not agent.docked        # lost: sent again
    agent.t += 2.0
    agent.final_reached(*agent.pose[:2])
    agent.t += 2.0
    agent.final_reached(*agent.pose[:2])
    assert len(agent.sent) == 3 and agent.docked            # 3 resets, AMCL never moved: docked, the cell shows why


def test_docked_at_once_when_amcl_agrees(agent):
    agent.pose = (-14.03, 7.02, math.radians(-179.1))
    agent.check_dock('receiving_bay')
    assert agent.docked and agent.sent == []


def test_waits_for_standstill_after_the_jump(agent):
    agent.check_dock('receiving_bay')
    amcl_takes(agent)
    agent.speed_est = 0.17                                   # the reset's jump seen as speed for one tick
    agent.t += 0.2
    agent.final_reached(*agent.pose[:2])
    assert not agent.docked
    agent.speed_est = 0.0
    agent.t += 0.2
    agent.final_reached(*agent.pose[:2])
    assert agent.docked and len(agent.sent) == 1


def test_no_reset_when_the_dock_is_suspect_or_not_a_bay(agent):
    agent.pose = (-12.5, 7.0, math.radians(-179.1))         # 1.5 m: the dock or the map is wrong, not AMCL
    agent.check_dock('receiving_bay')
    assert agent.docked and agent.sent == []
    agent.docked, agent.dock_state, agent.dock_name = False, 'settling', 'charge_0'
    agent.pose = (0.4, 0.0, 0.0)
    agent.check_dock('charge_0')
    assert agent.docked and agent.sent == []
