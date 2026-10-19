"""Docking failures: back to the staging node and dock again; after 3 attempts STUCK until an operator acknowledges
(fleet run 2026-10-03: a robot retried 79 times from a pose 1.8 m off the bay, the marker out of view). An inaccurate
dock backs out of the bay (undock) and docks again; a blocked back-out is retried, not counted. A back-out always aims at
the bay's staging node: from partway out it backs up the rest instead of undocking (fleet run 2026-10-19, chain 33)."""
import json

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import OperatorCommand, RobotState
from open_amr_swarm.agent import SwarmAgent, backout_plan


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
    a.route, a.docking, a.leg, a.next_idx = [0, 1, 2, 3, 4], True, None, 4   # at the staging node n3, bay n4
    a.dock_sent = []
    a.dock_client.server_is_ready = lambda: True
    a.dock_client.send_goal_async = lambda g: a.dock_sent.append(g) or type('F', (), {'add_done_callback': lambda s, cb: None})()
    yield a
    a.destroy_node()
    rclpy.shutdown()


def test_failed_dock_goes_back_to_staging_then_docks_again(agent):
    agent.dock_failed('n4', 'error 904')
    assert agent.dock_state == 'restaging' and agent.next_idx == 3      # drive to the staging node n3 again
    assert agent.status != RobotState.STUCK
    agent.next_idx = 4                                                  # ... reached it: the leg logic docks
    agent.start_dock()
    assert len(agent.dock_sent) == 1 and agent.dock_state == 'sending'


def test_three_failed_attempts_stop_and_wait_for_an_operator(agent):
    for k in range(3):
        agent.dock_failed('n4', 'error 904')
    assert agent.status == RobotState.STUCK and agent.dock_state == 'gave_up'
    agent.next_idx = 4
    for _ in range(5):
        agent.start_dock()
    assert agent.dock_sent == []                                        # no more attempts on its own
    agent.on_operator(OperatorCommand(command=OperatorCommand.ACK_FAULT, target='amr_1', stamp=TimeMsg(sec=1)))
    assert agent.dock_state == 'gave_up'                                # someone else's acknowledgement
    agent.on_operator(OperatorCommand(command=OperatorCommand.ACK_FAULT, target='amr_0', stamp=TimeMsg(sec=1)))
    assert agent.dock_state == 'restaging' and agent.dock_fails == 0 and agent.status != RobotState.STUCK


class _Res:
    def __init__(self, ok):
        from action_msgs.msg import GoalStatus
        self.status = GoalStatus.STATUS_SUCCEEDED if ok else GoalStatus.STATUS_ABORTED
        self.result = type('R', (), {'success': ok, 'error_code': 0 if ok else 999})()


def _undock(agent, ok):
    """Finish the pending undock goal: succeeded, or aborted (back-out blocked)."""
    agent.on_undock_done(type('F', (), {'result': lambda s: _Res(ok)})(), 'n4')


@pytest.fixture
def bay_agent(agent):
    agent.undock_sent = []
    agent.undock_client.send_goal_async = lambda g: agent.undock_sent.append(g) or \
        type('F', (), {'add_done_callback': lambda s, cb: None})()
    agent.backup_sent = []
    agent.backup_client.send_goal_async = lambda g: agent.backup_sent.append(g) or \
        type('F', (), {'add_done_callback': lambda s, cb: None})()
    agent.dock_error = lambda: (0.001, 0.003, 0.05)                     # 3 mm, 2.9 deg: out of tolerance
    agent.pose = (4.0, 0.0, 0.0)                                        # in the bay n4
    return agent


def test_inaccurate_dock_backs_out_then_docks_again(bay_agent):
    a = bay_agent
    a.check_dock('n4')
    assert len(a.undock_sent) == 1 and a.dock_state == 'undocking' and a.dock_fails == 1
    _undock(a, True)                                                    # backed out to the staging pose
    assert a.dock_state == 'retry' and a.next_idx == 4 and a.status != RobotState.STUCK
    a.dock_retry_at = 0.0
    a.start_dock()
    assert len(a.dock_sent) == 1 and a.dock_state == 'sending'


def test_blocked_back_out_is_retried_not_counted(bay_agent):
    """Fleet run 2026-10-03: the undock aborted on the robot queued behind; each abort counted as a docking attempt and
    a 'dock' from inside the bay succeeded at once with the same error -> STUCK in 20 s."""
    a = bay_agent
    a.check_dock('n4')
    for _ in range(5):
        _undock(a, False)
        assert a.dock_state == 'backout_wait' and a.dock_fails == 1 and a.status != RobotState.STUCK
        a.dock_retry_at = 0.0
        a.start_dock()                                                  # retries the undock, never a dock
        assert a.dock_state == 'undocking' and a.dock_sent == []
    assert len(a.undock_sent) == 6
    a.backout_t0 = a.now() - a.backout_patience - 1.0                   # blocked for longer than the patience
    _undock(a, False)
    assert a.status == RobotState.STUCK and a.dock_state == 'gave_up'


def test_operator_ack_in_the_bay_backs_out(bay_agent):
    a = bay_agent
    for _ in range(2):
        a.check_dock('n4'); _undock(a, True)
    a.check_dock('n4')
    assert a.status == RobotState.STUCK and a.dock_state == 'gave_up' and len(a.undock_sent) == 2
    a.on_operator(OperatorCommand(command=OperatorCommand.ACK_FAULT, target='amr_0', stamp=TimeMsg(sec=1)))
    assert a.dock_state == 'undocking' and len(a.undock_sent) == 3 and a.dock_fails == 0   # not a forward path


def test_backout_plan_aims_at_the_staging_node():
    bay, staging = (4.0, 0.0), (3.0, 0.0)
    assert backout_plan((4.0, 0.0, 0.0), bay, staging, 0.12, 0.05) == ('undock', 1.0)       # in the bay
    how, d = backout_plan((3.7, 0.02, 0.0), bay, staging, 0.12, 0.05)                      # 0.3 m out
    assert how == 'backup' and d == pytest.approx(0.7)
    assert backout_plan((3.08, 0.0, 0.0), bay, staging, 0.12, 0.05) == ('done', 0.0)       # at staging
    how, d = backout_plan((-13.8, 7.29, 3.14), (-14.0, 7.0), (-13.0, 7.0), 0.12, 0.05)       # chain 33, amr_1
    assert how == 'backup' and d == pytest.approx(0.8)


def test_dock_aborted_short_of_the_bay_backs_up_the_rest(bay_agent):
    """Chain 33, amr_1: the dock aborted 0.2 m short of receiving_bay (a late TF sample); the undock from there aimed
    1.2 m out, into amr_0's nose at queue_1 (13 attempts, 905 'Collision detected', STUCK)."""
    a = bay_agent
    a.pose = (3.8, 0.0, 0.0)
    a.dock_failed('n4', 'error 905')
    assert a.undock_sent == [] and len(a.backup_sent) == 1 and a.dock_state == 'undocking'
    g = a.backup_sent[0]
    assert g.target.x == pytest.approx(0.8) and g.speed == pytest.approx(a.backout_speed)
    _undock(a, True)
    assert a.dock_state == 'retry' and a.status != RobotState.STUCK


def test_retried_back_out_never_aims_further_out(bay_agent):
    """Chain 33, amr_0: a back-out stopped after ~0.55 m (a late TF sample); each retry undocked 1.0 m from there and
    hit the robot at queue_1 at the same pose for 60 s. The retry backs up the 0.45 m left instead."""
    a = bay_agent
    a.check_dock('n4')
    assert len(a.undock_sent) == 1
    a.pose = (3.45, 0.0, 0.0)                                           # stopped partway
    _undock(a, False)
    assert a.dock_state == 'backout_wait'
    a.dock_retry_at = 0.0
    a.start_dock()
    assert len(a.undock_sent) == 1 and len(a.backup_sent) == 1
    assert a.backup_sent[0].target.x == pytest.approx(0.45)
    a.pose = (3.0, 0.0, 0.0)
    _undock(a, True)
    assert a.dock_state == 'retry'


def test_back_out_from_the_staging_node_docks_again_at_once(bay_agent):
    a = bay_agent
    a.pose = (3.05, 0.0, 0.0)
    a.back_out('n4', 'operator acknowledged')
    assert a.undock_sent == [] and a.backup_sent == [] and a.dock_state == 'retry'
