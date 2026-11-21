"""Old tasks first (2026-11-21). (1) A robot orders open tasks by their first offer, not by Task.stamp, which every
re-offer renews (the WMS re-offers an unclaimed task every reoffer_s = 4 s): before, a task waiting for minutes sorted
behind one offered a second ago. (2) bid_strict_order: a robot bids only on the first task in that order it can do,
waiting for that task's round result or re-offer; before, a freed robot bid on every open round and took whichever
closed first (c65_cpufix t0013: offered t=218, never claimed in 900 s, while amr_5 and amr_3 bid on it eligibly and won
newer tasks in the same moment). A task whose round closed more than strict_wait_s ago is skipped, and one the robot
can't do never holds it back."""
import json

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import Task
from open_amr_swarm.agent import SwarmAgent


@pytest.fixture
def agent(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(8)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(7)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_0'])
    a = SwarmAgent()
    a.sent, a.lines, a.bid_on, a.clock = [], [], [], [0.0]
    a.send = lambda pub, msg: a.sent.append(msg)
    a.pose, a.last_node, a.reserved, a.synced = (0.0, 0.0, 0.0), 0, [0], True
    a.now = lambda: a.clock[0]
    a.get_logger = lambda: type('L', (), {'info': lambda self, m: a.lines.append(m),
                                          'warn': lambda self, m: None, 'error': lambda self, m: None,
                                          'debug': lambda self, m: None})()
    a.estimate = lambda start, goal, cb: a.bid_on.append(a.graph.name[goal])
    yield a
    a.destroy_node()
    rclpy.shutdown()


def offer(a, tid, origin, stamp, rnd=0):
    a.on_task(Task(task_id=tid, type=Task.GOTO, origin_id=origin, round=rnd, priority=1,
                   stamp=TimeMsg(sec=int(stamp), nanosec=int(round((stamp % 1) * 1e9)))))


def bid_once(a, now):
    a.clock[0], a.bid_pending, a.bid_on = now, False, []
    a.auction()
    return a.bid_on


def test_a_reoffered_old_task_sorts_before_a_newer_one(agent):
    offer(agent, 't_old', 'n3', 100.0)            # first offered at 100 ...
    offer(agent, 't_new', 'n5', 148.0)            # ... t_new at 148 ...
    offer(agent, 't_old', 'n3', 150.0, rnd=12)    # ... t_old re-offered at 150 (stamp renewed)
    assert [t.task_id for t in agent.pending_tasks()] == ['t_old', 't_new']


def test_strict_order_is_the_default(agent):
    """ADR-049: on unless a launch says bid_strict_order:=false (the fixture sets no such parameter)."""
    assert agent.strict_order is True


def test_off_bids_on_any_open_round(agent):
    agent.strict_order = False
    offer(agent, 't_old', 'n3', 100.0)
    offer(agent, 't_old', 'n3', 149.0, rnd=12)    # its round closed at 150.5
    offer(agent, 't_new', 'n5', 150.8)            # open until 152.3
    assert bid_once(agent, 151.0) == ['n5']


def test_strict_waits_for_the_old_task_instead_of_bidding_on_a_newer_one(agent):
    agent.strict_order = True
    offer(agent, 't_old', 'n3', 100.0)
    offer(agent, 't_old', 'n3', 149.0, rnd=12)
    offer(agent, 't_new', 'n5', 150.8)
    assert bid_once(agent, 151.0) == []           # waits for t_old's re-offer
    offer(agent, 't_old', 'n3', 153.0, rnd=13)    # the WMS re-offers it
    assert bid_once(agent, 153.2) == ['n3']


def test_strict_waits_for_its_own_open_bid(agent):
    agent.strict_order = True
    offer(agent, 't_old', 'n3', 100.0)
    offer(agent, 't_new', 'n5', 100.5)
    agent.clock[0] = 100.6
    agent.my_bids[('t_old', 0)] = 5.0             # bid sent, round open until 101.5
    assert bid_once(agent, 100.7) == []


def test_strict_skips_a_task_the_wms_stopped_offering(agent):
    agent.strict_order = True
    offer(agent, 't_old', 'n3', 100.0)            # round closed at 101.5, never re-offered
    offer(agent, 't_new', 'n5', 110.0)
    assert bid_once(agent, 110.2) == ['n5']       # 110.2 - 100 > 1.5 + 6


def test_strict_a_task_it_cannot_do_does_not_hold_it_back(agent):
    agent.strict_order = True
    offer(agent, 't_old', 'n3', 100.0)
    offer(agent, 't_old', 'n3', 149.0, rnd=12)
    offer(agent, 't_new', 'n5', 150.8)
    real = agent.bid_block                        # a per-task block (battery need, its arm out of service, cut off)
    agent.bid_block = lambda t: ('arm_down:n3', 'arm out of service at n3') if t.task_id == 't_old' else real(t)
    assert bid_once(agent, 151.0) == ['n5']
