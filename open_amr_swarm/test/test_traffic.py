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


def test_no_far_node_beyond_horizon_plus_overreach(tmp_path):
    """A node 6.6 m ahead isn't held while a nearer one is (a stuck robot must not block a merge far away)."""
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}'}},
              'geometry': {'type': 'Point', 'coordinates': [x, 0.0]}} for i, x in enumerate([0.0, 0.5, 7.1])]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[0, 0], [1, 0]]]}} for i in range(2)]
    p = tmp_path / 'far.geojson'
    p.write_text(json.dumps({'features': feats}))
    g = LaneGraph(str(p))
    res, blocked = plan_reservations(g, [0, 1, 2], 1, Peer('amr_0', 0.0, 0.0), [], held=[], p=P)
    assert res == [0, 1] and not blocked
    res, _ = plan_reservations(g, [0, 1, 2], 2, Peer('amr_0', 0.6, 0.0), [], held=[], p=P)
    assert 2 in res                                  # once it's the very next node it is held, however far


@pytest.fixture
def dock(tmp_path):
    """A bay at x=0 (kind bay) with its queue lane behind it, nodes 1 m apart: q2 (-3) -> q1 (-2) -> q0 (-1) -> bay."""
    nodes = [(0, 'bay', 0.0, 'bay'), (1, 'q0', -1.0, 'queue'), (2, 'q1', -2.0, 'queue'), (3, 'q2', -3.0, 'queue')]
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': nm, 'kind': k}},
              'geometry': {'type': 'Point', 'coordinates': [x, 0.0]}} for i, nm, x, k in nodes]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + a, 'startid': a, 'endid': b},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[-a, 0], [-b, 0]]]}}
              for a, b in ((3, 2), (2, 1), (1, 0))]
    p = tmp_path / 'dock.geojson'
    p.write_text(json.dumps({'features': feats}))
    return LaneGraph(str(p))


P6 = TrafficParams(horizon_m=4.0, max_nodes=5, turn_clearance=1.5)
DOCK_ROUTE = [3, 2, 1, 0]


def test_conflicts_pair_turn_nodes_with_close_neighbours(dock):
    assert dock.conflicts(0, 1.5) == [1]             # bay <-> q0 (1.0 m); q1 is 2.0 m away
    assert dock.conflicts(1, 1.5) == [0]
    assert dock.conflicts(2, 1.5) == []              # queue nodes 1 m apart don't conflict: neither turns in place


def test_robot_in_bay_keeps_the_next_one_out_of_its_turning_room(dock):
    """Fleet run 2026-10-19: the bay robot could not turn out while the next robot stood 1.0 m behind it."""
    docked = Peer('amr_2', 0.0, 0.0, reserved=[0])
    me = Peer('amr_0', -2.0, 0.0)                    # standing on q1
    res, blocked = plan_reservations(dock, DOCK_ROUTE, 2, me, [docked], held=[2], p=P6)
    assert res == [2] and blocked                    # waits on q1, q0 stays free
    res, blocked = plan_reservations(dock, DOCK_ROUTE, 2, me, [docked], held=[2], p=TrafficParams(turn_clearance=0.0))
    assert 1 in res                                  # without rule 6 it moved up to q0 (the deadlock)


def test_turning_room_frees_once_the_bay_is_released(dock):
    left = Peer('amr_2', 0.0, 2.2, reserved=[9])     # gone north to its exit node, bay released
    res, blocked = plan_reservations(dock, DOCK_ROUTE, 2, Peer('amr_0', -2.0, 0.0), [left], held=[2], p=P6)
    assert res == [2, 1, 0] and not blocked


def test_docking_robot_holds_staging_and_bay_together(dock):
    """Rule 6 only counts peers: the robot that docks holds q0 (its staging node) and the bay at once."""
    res, blocked = plan_reservations(dock, DOCK_ROUTE, 3, Peer('amr_0', -1.0, 0.0), [], held=[1], p=P6)
    assert res == [1, 0] and not blocked
