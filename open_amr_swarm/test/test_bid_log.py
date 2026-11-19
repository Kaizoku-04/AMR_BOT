"""[bid] lines in swarm.log (2026-11-19): one INFO line per task when a robot's verdict or reason changes, so "why did
no robot take t0007 for 200 s" can be answered. Fed straight into a SwarmAgent (no sim), as test_operator_hold.py does."""
import json

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import OperatorCommand as OC
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
    a.sent, a.lines = [], []
    a.send = lambda pub, msg: a.sent.append(msg)
    a.pose, a.last_node, a.reserved, a.synced = (0.0, 0.0, 0.0), 0, [0], True
    a.get_logger = lambda: type('L', (), {'info': lambda self, m: a.lines.append(m),
                                          'warn': lambda self, m: None, 'error': lambda self, m: None,
                                          'debug': lambda self, m: None})()
    yield a
    a.destroy_node()
    rclpy.shutdown()


def task(a, tid, rnd=0):
    t = Task(task_id=tid, type=Task.GOTO, origin_id='n3', round=rnd, stamp=TimeMsg(sec=1000))
    a.tasks[tid] = t
    return t


def bids(a):
    return [m for m in a.lines if m.startswith('[bid]')]


def test_paused_logged_once_not_every_round(agent):
    task(agent, 't0001')
    agent.on_operator(OC(command=OC.PAUSE, issued_by='laith', reason='t', stamp=TimeMsg(sec=10)))
    for _ in range(5):
        agent.log_eligibility()
    assert bids(agent) == ['[bid] amr_0 t0001 round 0 cost=- eligible=no reason=paused']


def test_reason_change_logs_again(agent):
    task(agent, 't0001')
    agent.on_operator(OC(command=OC.PAUSE, issued_by='laith', reason='t', stamp=TimeMsg(sec=10)))
    agent.log_eligibility()
    agent.on_operator(OC(command=OC.OUT_OF_SERVICE, target='amr_0', issued_by='laith', reason='t', stamp=TimeMsg(sec=11)))
    agent.log_eligibility()
    assert [m.split('reason=')[1] for m in bids(agent)] == ['paused', 'out of service']


def test_busy_names_the_task(agent):
    agent.task = Task(task_id='t0003')
    task(agent, 't0007')
    agent.log_eligibility()
    assert bids(agent)[0].endswith('t0007 round 0 cost=- eligible=no reason=busy with t0003')


def test_low_battery_says_soc_need_and_margin(agent):
    agent.bms, agent.battery = True, 0.13
    agent.must_charge = False
    agent.drain.rate = 0.01                              # fraction of charge per s while working
    task(agent, 't0002')
    agent.log_eligibility()
    line = bids(agent)[0]
    assert 'eligible=no reason=battery: soc 13 % - need' in line and 'x1.3 margin' in line and 'reserve 12 %' in line
    agent.battery = 0.125                                  # same reason, other numbers: not logged again
    agent.log_eligibility()
    assert len(bids(agent)) == 1


def test_charging_reason(agent):
    agent.bms, agent.battery, agent.must_charge = True, 0.25, True
    task(agent, 't0002')
    agent.log_eligibility()
    assert 'reason=charging: soc 25 % below 30 %' in bids(agent)[0]


def test_eligible_line_has_the_cost_and_is_logged_once(agent):
    t = task(agent, 't0004')
    agent.send_bid(t, 12.34)
    t2 = task(agent, 't0004', rnd=1)
    agent.send_bid(t2, 12.9)                               # next round, still eligible: no new line
    assert bids(agent) == ['[bid] amr_0 t0004 round 0 cost=12.3 eligible=yes']


def test_verdict_flip_logs_both_ways_and_forgets_finished_tasks(agent):
    t = task(agent, 't0004')
    agent.send_bid(t, 5.0)
    agent.on_operator(OC(command=OC.PAUSE, issued_by='laith', reason='t', stamp=TimeMsg(sec=10)))
    agent.log_eligibility()
    assert bids(agent)[-1].endswith('eligible=no reason=paused')
    agent.claims['t0004'] = 'amr_1'
    agent.log_eligibility()
    assert 't0004' not in agent.bid_verdict


def _state(a, kind):
    a.bms, a.battery, a.must_charge = True, 1.0, False
    a.drain.rate = 0.01
    if kind == 'out_of_service':
        a.out_of_service = True
    elif kind == 'paused':
        a.paused_me = True
    elif kind == 'busy':
        a.task = Task(task_id='t0009')
    elif kind == 'phase':
        a.phase = 'docking'
    elif kind == 'charging':
        a.battery, a.must_charge = 0.25, True
    elif kind == 'battery':
        a.battery = 0.13
    elif kind == 'arm_down':
        a.handshake = True
        a.station_down = lambda bay: True
        a.graph.kind[a.graph.id('n3')] = 'bay'
    elif kind == 'blocked':
        a.cut_off = lambda ghosts=None: [{a.graph.id('n3')}]


@pytest.mark.parametrize('kind', ['eligible', 'out_of_service', 'paused', 'busy', 'phase', 'charging', 'battery',
                                  'arm_down', 'blocked'])
def test_bid_block_and_the_real_auction_agree(agent, kind):
    """Guard: auction() starts a bid for a task exactly when bid_block() says None (they must never drift)."""
    t = task(agent, 't0001')
    t.stamp = agent.get_clock().now().to_msg()             # the round is still open
    _state(agent, kind)
    started = []
    agent.estimate = lambda start, goal, cb: started.append(goal)
    agent.auction()
    assert bool(started) == (agent.bid_block(t) is None)
    assert bool(started) == (kind == 'eligible')
    if kind != 'eligible':
        assert any('eligible=no' in m for m in bids(agent))
