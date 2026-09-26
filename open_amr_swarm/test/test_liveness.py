"""Unit tests for the fail-safe rules (open_amr_swarm/liveness.py) and the stale-view reservation rule."""
import json

import pytest

from open_amr_swarm.lane_graph import LaneGraph
from open_amr_swarm.liveness import (LivenessParams, Pending, acked, announcer, ghost_hold, outranks, quorum,
                                     reclaimable)
from open_amr_swarm.traffic import Peer, TrafficParams, plan_reservations

FLEET = [f'amr_{i}' for i in range(6)]


@pytest.fixture
def line(tmp_path):
    """Nodes 0..7 on a straight one-way lane, 1 m apart along x."""
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(8)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(7)]
    p = tmp_path / 'g.geojson'
    p.write_text(json.dumps({'features': feats}))
    return LaneGraph(str(p))


# ---------------------------------------------------------------- quorum
def test_quorum_majority_of_fleet():
    assert quorum('amr_0', FLEET[1:], FLEET)                       # hears everyone
    assert quorum('amr_0', FLEET[1:3], FLEET[:4])                  # 3 of 4
    assert quorum('amr_0', ['amr_1', 'amr_2', 'amr_3'], FLEET)     # 4 of 6
    assert not quorum('amr_0', ['amr_1', 'amr_2'], FLEET)          # 3 of 6: a split in half stops both halves
    assert not quorum('amr_0', [], FLEET)                          # cut off: minority of one


def test_quorum_released_robots_leave_the_count():
    heard = ['amr_1', 'amr_2']
    assert not quorum('amr_0', heard, FLEET)
    assert quorum('amr_0', heard, FLEET, released={'amr_3', 'amr_4'})          # 3 of 4
    # a released robot that is heard again counts again (on both sides of the fraction)
    assert quorum('amr_0', heard + ['amr_3'], FLEET, released={'amr_3', 'amr_4'})


def test_quorum_two_robot_fleet_needs_both():
    assert quorum('amr_0', ['amr_1'], ['amr_0', 'amr_1'])
    assert not quorum('amr_0', [], ['amr_0', 'amr_1'])
    assert quorum('amr_0', [], ['amr_0', 'amr_1'], released={'amr_1'})


# ---------------------------------------------------------------- silent peer = still there
def test_ghost_holds_reservation_route_margin_and_its_node(line):
    # reserved 2..3, remaining route 3..7, stands at x=2.4: + route nodes up to 2 m past node 3 (4, 5), + nearest (2)
    held = ghost_hold(line, [2, 3], [3, 4, 5, 6, 7], 2.4, 0.0, 2.0)
    assert held == [2, 3, 4, 5]
    # at least one node past the reservation even with no margin
    assert ghost_hold(line, [2, 3], [3, 4, 5], 2.4, 0.0, 0.0) == [2, 3, 4]
    # idle (no route): its node only
    assert ghost_hold(line, [6], [], 6.1, 0.0, 2.0) == [6]
    # no reservations at all (never heard driving): the node it stands nearest
    assert ghost_hold(line, [], [], 4.2, 0.1, 2.0) == [4]


def test_ghost_blocks_followers(line):
    held = ghost_hold(line, [3, 4], [4, 5, 6], 3.5, 0.0, 2.0)
    ghost = Peer('amr_1', 3.5, 0.0, held, 0.0)
    me = Peer('amr_0', 0.0, 0.0)
    res, blocked = plan_reservations(line, list(range(8)), 1, me, [ghost], held=[], p=TrafficParams(horizon_m=6.0, max_nodes=8))
    assert res == [0, 1, 2] and blocked


def test_stale_view_adds_no_nodes(line):
    me = Peer('amr_0', 0.0, 0.0)
    p = TrafficParams(horizon_m=4.0, max_nodes=5)
    res, blocked = plan_reservations(line, list(range(8)), 1, me, [], held=[0, 1, 2], p=p, extend=False)
    assert res == [0, 1, 2] and blocked                  # keeps what it holds, nothing new
    res, blocked = plan_reservations(line, list(range(8)), 1, me, [], held=[0, 1, 2], p=p)
    assert res == [0, 1, 2, 3, 4] and not blocked        # fresh view: extends to the horizon


# ---------------------------------------------------------------- re-auction
def test_reclaim_rules():
    p = LivenessParams(reclaim_silent_s=20.0)
    assert reclaimable(25.0, False, 't0001', False, p)
    assert not reclaimable(10.0, False, 't0001', False, p)          # not silent long enough
    assert reclaimable(1.0, True, 't0001', False, p)                # an operator released it: at once
    assert not reclaimable(99.0, True, 't0001', True, p)            # box on the deck: never
    assert not reclaimable(99.0, False, '', False, p)               # no task


def test_announcer_is_lowest_live_id():
    assert announcer('amr_3', ['amr_1', 'amr_4']) == 'amr_1'
    assert announcer('amr_0', ['amr_1', 'amr_4']) == 'amr_0'


def test_outranks_loaded_then_round_then_id():
    assert outranks((True, 0, 'amr_5'), (False, 3, 'amr_0'))       # the box is on its deck
    assert outranks((False, 2, 'amr_5'), (False, 1, 'amr_0'))      # re-auctioned (newer round)
    assert outranks((False, 1, 'amr_0'), (False, 1, 'amr_5'))      # same round: lower id
    assert not outranks((False, 1, 'amr_5'), (False, 1, 'amr_0'))


# ---------------------------------------------------------------- lost acknowledgements
def test_pending_tokens():
    q = Pending()
    t1 = q.send(0.0)
    assert not q.overdue(2.0, 3.0) and q.overdue(3.5, 3.0)
    t2 = q.send(3.5, retry=True)                     # resent after a lost ack
    assert q.tries == 2
    assert not q.ack(t1)                             # the late answer to the first one is stale
    assert q.ack(t2) and not q.overdue(99.0, 3.0)
    assert not q.ack(t2)                             # answered once
    t3 = q.send(10.0)
    q.clear()                                        # abandoned (robot stopped)
    assert not q.current(t3) and not q.ack(t3) and not q.overdue(99.0, 3.0)


def test_trapped_behind_a_silent_robot(tmp_path):
    """A loop street 0-1-2-3-0 with a one-way aisle 1 -> 10 -> 11 -> 12 -> 3: a robot stopped on 11 traps 10."""
    pts = {0: (0, 0), 1: (4, 0), 2: (4, 4), 3: (0, 4), 10: (2, 1), 11: (2, 2), 12: (2, 3)}
    edges = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 10), (10, 11), (11, 12), (12, 3)]
    feats = [{'type': 'Feature', 'properties': {'id': n, 'metadata': {'name': f'n{n}'}},
              'geometry': {'type': 'Point', 'coordinates': list(xy)}} for n, xy in pts.items()]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': a, 'endid': b},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[pts[a], pts[b]]]}} for i, (a, b) in enumerate(edges)]
    p = tmp_path / 'loop.geojson'
    p.write_text(json.dumps({'features': feats}))
    g = LaneGraph(str(p))
    assert g.trapped(set()) == set()
    assert g.trapped({11}) == {10}               # 12 can still drive out; 10 can only drive into the silent robot
    assert g.trapped({2}) == set()               # the aisle is the way round
    assert g.cut_off({11}) == {10, 12}           # 10 has no way out, 12 no way in
    assert g.cut_off({2}) == set() and g.cut_off(set()) == set()


def test_acked_needs_every_heard_peer():
    from types import SimpleNamespace as S
    a = S(heard_ids=['amr_0', 'amr_2'], heard_seq=[12, 40])
    b = S(heard_ids=['amr_0'], heard_seq=[10])
    assert acked(10, 'amr_0', [a, b])
    assert not acked(11, 'amr_0', [a, b])            # amr_b hasn't received heartbeat 11 yet
    assert not acked(5, 'amr_0', [S(heard_ids=[], heard_seq=[])])   # a peer that never heard me
    assert not acked(None, 'amr_0', [a])             # never published
    assert acked(3, 'amr_0', [])                     # alone
