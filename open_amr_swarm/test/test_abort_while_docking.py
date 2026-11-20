"""A task dropped while its robot docks (WMS CANCEL, a peer's outranking claim or heartbeat, a release on my behalf,
give_back): every bay goal is cancelled, and a robot standing in the bay backs out to the staging pose before it takes
new work (review 2026-11-05 finding 1; the first test is the review's reproducer, which failed on 4206a0a)."""
import json

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import Claim, OperatorCommand, RobotState, Task
from open_amr_swarm.agent import SwarmAgent


class Handle:
    def __init__(self):
        self.cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


class _Res:
    def __init__(self, ok):
        from action_msgs.msg import GoalStatus
        self.status = GoalStatus.STATUS_SUCCEEDED if ok else GoalStatus.STATUS_ABORTED
        self.result = type('R', (), {'success': ok, 'error_code': 0 if ok else 999})()


@pytest.fixture
def agent(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'bay' if i == 7 else 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(8)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(7)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_0', '-p', 'dock_bays:=true'])
    a = SwarmAgent()
    a.sent = []
    a.send = lambda pub, msg: a.sent.append(msg)
    a.pose, a.last_node, a.reserved, a.synced = (6.5, 0.0, 0.0), 6, [6, 7], True
    a.route, a.docking, a.next_idx = [5, 6, 7], True, 2                 # staging node n6, bay n7
    a.task = Task(task_id='t1', round=0, type=Task.FETCH_FROM_STATION, origin_id='n7', dest_id='n2')
    a.loaded, a.phase = False, 'driving'
    a.undock_sent, a.backup_sent = [], []
    fut = type('F', (), {'add_done_callback': lambda s, cb: None})()
    a.undock_client.send_goal_async = lambda g: a.undock_sent.append(g) or fut
    a.backup_client.send_goal_async = lambda g: a.backup_sent.append(g) or fut
    yield a
    a.destroy_node()
    rclpy.shutdown()


def cancel(a):
    a.on_claim(Claim(resource_type=Claim.TASK, resource_id='t1', robot_id='', action=Claim.CANCEL, by='wms',
                     stamp=TimeMsg(sec=5)))


def test_wms_cancel_while_docking_cancels_the_dock_goal(agent):
    h = Handle()
    agent.dock_handle, agent.dock_state = h, 'docking'
    cancel(agent)
    assert agent.task is None                 # the task is dropped ...
    assert h.cancelled, 'DockRobot goal still running after the task was dropped'   # ... and so is the dock goal
    assert agent.dock_handle is None and agent.dock_state != 'docking'


def test_robot_in_the_bay_backs_out_before_taking_new_work(agent):
    agent.pose = (7.0, 0.0, 0.0)                                        # nose-in at the pedestal
    agent.dock_handle, agent.dock_state = Handle(), 'docking'
    cancel(agent)
    assert agent.phase == 'bay_leave' and not agent.available()
    assert len(agent.undock_sent) + len(agent.backup_sent) == 1         # one back-out goal to the staging pose
    agent.on_undock_done(type('F', (), {'result': lambda s: _Res(True)})(), 'n7')
    assert agent.phase == 'idle' and agent.status == RobotState.IDLE and agent.dock_state is None
    assert agent.available()


def test_blocked_back_out_after_a_drop_is_retried_from_the_tick(agent):
    agent.pose = (7.0, 0.0, 0.0)
    agent.dock_state = 'settling'
    cancel(agent)
    agent.backout_t0 = agent.now()
    agent.on_undock_done(type('F', (), {'result': lambda s: _Res(False)})(), 'n7')
    assert agent.dock_state == 'backout_wait' and agent.phase == 'bay_leave'
    n = len(agent.undock_sent) + len(agent.backup_sent)
    agent.dock_retry_at = agent.now() - 1.0
    agent.update_pose = lambda: True                                    # no TF here; the pose is set
    agent.check_peers = lambda: True
    agent.tick()
    assert len(agent.undock_sent) + len(agent.backup_sent) == n + 1


def test_late_acceptance_of_a_dock_goal_after_the_drop_is_cancelled(agent):
    tok = agent.dock_req.send(0.0)                                      # sent, acceptance not yet back
    agent.dock_state = 'sending'
    cancel(agent)
    h = Handle()
    h.accepted = True
    agent.on_dock_handle(type('F', (), {'result': lambda s: h})(), 'n7', tok)
    assert h.cancelled and agent.dock_handle is None


def test_drop_at_the_staging_node_goes_idle_without_backing_out(agent):
    agent.dock_state = 'restaging'                                      # docking leg, but not in the bay (6.5 m)
    cancel(agent)
    assert agent.phase == 'idle' and agent.undock_sent == [] and agent.backup_sent == []


def test_drop_without_docking_changes_nothing_else(agent):
    agent.docking, agent.dock_state, agent.route, agent.next_idx = False, None, [1, 2, 3], 1
    cancel(agent)
    assert agent.phase == 'idle' and agent.undock_sent == [] and agent.backup_sent == []


def test_drop_after_giving_up_in_the_bay_waits_for_the_operator(agent):
    agent.pose = (7.0, 0.0, 0.0)
    agent.dock_state, agent.status = 'gave_up', RobotState.STUCK
    agent.give_back('no progress for 300 s')
    assert agent.task is None and agent.status == RobotState.STUCK and agent.phase == 'bay_leave'
    assert agent.undock_sent == [] and agent.backup_sent == [] and not agent.available()
    agent.on_operator(OperatorCommand(command=OperatorCommand.ACK_FAULT, target='amr_0', stamp=TimeMsg(sec=1)))
    assert len(agent.undock_sent) + len(agent.backup_sent) == 1
    agent.on_undock_done(type('F', (), {'result': lambda s: _Res(True)})(), 'n7')
    assert agent.phase == 'idle' and agent.status == RobotState.IDLE


def test_drop_in_the_bay_while_paused_backs_out_only_after_the_resume(agent):
    """c61_ops (2026-11-14): paused by the operator, an empty robot gives its task back after 120 s (ADR-047) and
    backed out of the bay at once, 1.0 m while the fleet was paused. It waits for the RESUME now."""
    agent.pose = (7.0, 0.0, 0.0)
    agent.dock_state, agent.docked = 'docked', True
    agent.on_operator(OperatorCommand(command=OperatorCommand.PAUSE, target='', stamp=TimeMsg(sec=1)))
    agent.give_back('no progress for 121 s')
    assert agent.task is None and agent.phase == 'bay_leave' and not agent.available()
    agent.update_pose = lambda: True
    agent.check_peers = lambda: True
    agent.tick()
    assert agent.undock_sent == [] and agent.backup_sent == [], 'backed out of the bay while paused'
    agent.on_operator(OperatorCommand(command=OperatorCommand.RESUME, target='', stamp=TimeMsg(sec=2)))
    agent.tick()
    assert len(agent.undock_sent) + len(agent.backup_sent) == 1
