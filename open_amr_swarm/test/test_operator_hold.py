"""Operator PAUSE / RESUME and OUT_OF_SERVICE / IN_SERVICE (OperatorCommand, Phase 6, 2026-11-05): what a robot does
with them. Fed straight into a SwarmAgent (no sim), as test_agent_liveness.py does."""
import json

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import Claim, OperatorCommand, RobotState, Task
from open_amr_swarm.agent import SwarmAgent

OC = OperatorCommand


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
    a.sent = []
    a.send = lambda pub, msg: a.sent.append(msg)
    a.pose, a.last_node, a.reserved, a.synced = (0.0, 0.0, 0.0), 0, [0], True
    yield a
    a.destroy_node()
    rclpy.shutdown()


def cmd(a, c, target=''):
    a.on_operator(OC(command=c, target=target, issued_by='laith', reason='test', stamp=TimeMsg(sec=10)))


def beat(a):
    a.publish_state()
    return a.sent[-1]


def test_fleet_pause_stops_new_work_and_is_reported(agent):
    assert agent.available()
    cmd(agent, OC.PAUSE)
    assert not agent.available() and beat(agent).status == RobotState.PAUSED
    cmd(agent, OC.RESUME)
    assert agent.available() and beat(agent).status == RobotState.IDLE


def test_a_pause_for_another_robot_changes_nothing(agent):
    cmd(agent, OC.PAUSE, 'amr_3')
    cmd(agent, OC.OUT_OF_SERVICE, 'amr_3')
    assert agent.available() and beat(agent).status == RobotState.IDLE and not beat(agent).out_of_service


def test_robot_pause_is_cleared_by_its_resume_or_a_fleet_resume(agent):
    cmd(agent, OC.PAUSE, 'amr_0')
    cmd(agent, OC.RESUME, 'amr_5')
    assert agent.paused()
    cmd(agent, OC.RESUME)
    assert not agent.paused()
    cmd(agent, OC.PAUSE)
    cmd(agent, OC.PAUSE, 'amr_0')
    cmd(agent, OC.RESUME, 'amr_0')
    assert agent.paused()                         # the fleet pause still holds


def test_out_of_service_takes_no_work_but_keeps_moving(agent):
    cmd(agent, OC.OUT_OF_SERVICE, 'amr_0')
    hb = beat(agent)
    assert not agent.available() and hb.out_of_service and hb.status == RobotState.IDLE and not agent.paused()
    cmd(agent, OC.IN_SERVICE, 'amr_0')
    assert agent.available() and not beat(agent).out_of_service


def test_a_long_pause_gives_an_empty_task_back_like_a_stuck_robot(agent, monkeypatch):
    agent.task = Task(task_id='t7', round=2, type=Task.GOTO, origin_id='n5')
    agent.loaded = False
    cmd(agent, OC.PAUSE)
    t = [100.0]
    monkeypatch.setattr(agent, 'now', lambda: t[0])
    agent.check_stuck()
    t[0] = 100.0 + agent.lp.reclaim_stuck_s + 1
    agent.check_stuck()
    rel = [m for m in agent.sent if isinstance(m, Claim) and m.action == Claim.RELEASE]
    assert rel and rel[0].resource_id == 't7' and agent.task is None


def test_a_paused_robot_with_a_box_keeps_its_task(agent, monkeypatch):
    agent.task = Task(task_id='t8', round=0, type=Task.FETCH_FROM_BIN, origin_id='n5', dest_id='n6')
    agent.loaded = True
    cmd(agent, OC.PAUSE)
    t = [100.0]
    monkeypatch.setattr(agent, 'now', lambda: t[0])
    agent.check_stuck()
    t[0] = 1000.0
    agent.check_stuck()
    assert agent.task is not None and not [m for m in agent.sent if isinstance(m, Claim)]


def test_commands_replayed_in_order_leave_the_last_state(agent):
    for c in (OC.PAUSE, OC.RESUME, OC.PAUSE, OC.RESUME):
        cmd(agent, c)
    assert not agent.paused()
