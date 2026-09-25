"""Unit tests for the lane-graph reservation rules (open_amr_swarm/traffic.py)."""
import json

import pytest

from open_amr_swarm.lane_graph import LaneGraph
from open_amr_swarm.traffic import Peer, TrafficParams, plan_reservations, wait_chain


@pytest.fixture
def line(tmp_path):
    """Nodes 0..5 on a straight one-way lane, 2 m apart along x."""
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}'}},
              'geometry': {'type': 'Point', 'coordinates': [2.0 * i, 0.0]}} for i in range(6)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[2.0 * i, 0], [2.0 * i + 2, 0]]]}}
              for i in range(5)]
    p = tmp_path / 'g.geojson'
    p.write_text(json.dumps({'features': feats}))
    return LaneGraph(str(p))


ROUTE = [0, 1, 2, 3, 4, 5]
P = TrafficParams(horizon_m=4.0, max_nodes=5)


def test_free_road_reserves_up_to_horizon(line):
    me = Peer('amr_0', 0.0, 0.0)
    res, blocked = plan_reservations(line, ROUTE, 1, me, [], held=[], p=P)
    assert res == [0, 1, 2] and not blocked          # 4 m ahead = nodes 1, 2 (+ node 0 still under the tail)


def test_first_come_blocks_before_held_node(line):
    me = Peer('amr_0', 0.0, 0.0)
    other = Peer('amr_1', 20.0, 5.0, reserved=[2])
    res, blocked = plan_reservations(line, ROUTE, 1, me, [other], held=[], p=P)
    assert res == [0, 1] and blocked


def test_race_lower_id_wins_when_nobody_committed(line):
    a = Peer('amr_0', 0.0, 0.0, reserved=[1, 2])
    b = Peer('amr_1', 4.0, 3.0, reserved=[2])       # a robot on a crossing lane, 3 m from node 2
    res_a, blocked_a = plan_reservations(line, ROUTE, 1, a, [b], held=[1, 2], p=P)
    assert 2 in res_a and not blocked_a
    res_b, blocked_b = plan_reservations(line, [2], 0, b, [a], held=[2], p=P)
    assert res_b == [] and blocked_b


def test_committed_robot_keeps_node_against_higher_priority(line):
    a = Peer('amr_0', 0.0, 0.0, reserved=[1, 2])
    b = Peer('amr_5', 4.0, 0.6, reserved=[2])       # 0.6 m from node 2: committed
    res_b, _ = plan_reservations(line, [2], 0, b, [a], held=[2], p=P)
    assert res_b == [2]
    res_a, blocked_a = plan_reservations(line, ROUTE, 1, a, [b], held=[1, 2], p=P)
    assert 2 not in res_a and blocked_a


def test_node_behind_released_after_release_dist(line):
    me = Peer('amr_0', 3.2, 0.0)                     # 1.2 m past node 1
    res, _ = plan_reservations(line, ROUTE, 2, me, [], held=[1, 2], p=P)
    assert 1 not in res and res[0] == 2


def test_robot_standing_on_node_blocks_even_without_reservation(line):
    me = Peer('amr_0', 0.0, 0.0)
    parked = Peer('amr_1', 4.1, 0.1, reserved=[])
    res, blocked = plan_reservations(line, ROUTE, 1, me, [parked], held=[], p=P)
    assert res == [0, 1] and blocked


def test_ageing_priority_beats_lower_id(line):
    a = Peer('amr_0', 0.0, 0.0, reserved=[1, 2], wait_s=0.0)
    b = Peer('amr_3', 4.0, 3.0, reserved=[2], wait_s=25.0)   # waited 2+ buckets
    res_a, blocked_a = plan_reservations(line, ROUTE, 1, a, [b], held=[1, 2], p=P)
    assert 2 not in res_a and blocked_a


def test_agents_near_a_node_do_not_block_it_by_standing(line):
    """An agent standing close to a node (but holding other nodes) blocks nothing: its reservations cover it."""
    me = Peer('amr_0', 0.0, 0.0)
    near = Peer('amr_1', 4.3, 0.6, reserved=[9])     # 0.66 m from node 2, holds another node
    res, blocked = plan_reservations(line, ROUTE, 1, me, [near], held=[], p=P)
    assert 2 in res and not blocked


def test_wait_chain_finds_cycle():
    chain, cycle = wait_chain('amr_1', {'amr_1': 'amr_2', 'amr_2': 'amr_3', 'amr_3': 'amr_1'})
    assert set(cycle) == {'amr_1', 'amr_2', 'amr_3'}
    chain, cycle = wait_chain('amr_1', {'amr_1': 'amr_2', 'amr_2': None})
    assert chain == ['amr_1', 'amr_2'] and cycle == []
    chain, cycle = wait_chain('amr_0', {'amr_0': 'amr_1', 'amr_1': 'amr_2', 'amr_2': 'amr_1'})
    assert cycle == ['amr_1', 'amr_2']               # I'm behind a cycle, not in it
