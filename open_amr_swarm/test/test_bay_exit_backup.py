"""Leaving an arm bay (chain 41, 2026-10-23): the ~90 deg turn at the bay passes the arm pedestal by 2-5 cm, and a robot
that crept 4-5 cm forward in the turn (MPPI alone, or the RotationShim handing the turn to MPPI after a late TF sample)
was held by collision_monitor for the rest of the shift. The agent now backs straight out bay_exit_backup_m along the
docking line first and plans the leg from the bay's exit, with the lead-in from where it stopped."""
import json

import pytest
import rclpy
from action_msgs.msg import GoalStatus

from open_amr_msgs.msg import RobotState
from open_amr_swarm.agent import Leg, SwarmAgent

BAY, STAGING, EXIT, AISLE = (-14.0, 7.0), (-13.0, 7.0), (-14.0, 9.2), (-10.0, 9.2)


def _future():
    return type('F', (), {'add_done_callback': lambda s, cb: None})()


@pytest.fixture
def agent(tmp_path):
    nodes = [('receiving_queue_0', 'queue', STAGING), ('receiving_bay', 'bay', BAY),
             ('receiving_exit_0', 'dock_exit', EXIT), ('street_0', 'street', AISLE), ('aisle_0_bay_0', 'aisle_bay', (-10, 7))]
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': n, 'kind': k}},
              'geometry': {'type': 'Point', 'coordinates': list(xy)}} for i, (n, k, xy) in enumerate(nodes)]
    for eid, (s, e) in enumerate([(0, 1), (1, 2), (2, 3), (3, 4), (4, 3)]):
        feats.append({'type': 'Feature', 'properties': {'id': 100 + eid, 'startid': s, 'endid': e, 'metadata': {}},
                      'geometry': {'type': 'MultiLineString', 'coordinates': [[list(nodes[s][2]), list(nodes[e][2])]]}})
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_2', '-p', 'fleet:=["amr_2"]'])
    a = SwarmAgent()
    a.t = 100.0
    a.now = lambda: a.t
    a.last_node, a.pose, a.phase = 1, (BAY[0], BAY[1], 3.1416), 'dwelling'    # docked, nose at the pedestal
    a.leg = Leg(1, 30.0, RobotState.TO_PICK, station=True)
    a.legs = [Leg(4, 5.0, RobotState.TO_DROP)]
    a.backup_sent, a.routes = [], []
    a.backup_client.server_is_ready = lambda: True
    a.backup_client.send_goal_async = lambda g: a.backup_sent.append(g) or _future()
    a.route_client.wait_for_server = lambda timeout_sec=None: True
    a.route_client.send_goal_async = lambda g: a.routes.append(g) or _future()
    yield a
    a.destroy_node()
    rclpy.shutdown()


def _finish_backup(a, ok, error_code=0):
    h = object()
    a.exitbk_handle = h
    res = type('R', (), {'result': type('Res', (), {'error_code': error_code})(),
                         'status': GoalStatus.STATUS_SUCCEEDED if ok else GoalStatus.STATUS_ABORTED})()
    a.on_exitbk_done(type('F', (), {'result': lambda s: res})(), h)


def test_leaving_an_arm_bay_backs_out_before_routing(agent):
    a = agent
    a.leave_bay()
    assert a.phase == 'bay_exit' and a.routes == []
    assert len(a.backup_sent) == 1 and a.backup_sent[0].target.x == pytest.approx(0.20)


def test_after_the_back_up_the_leg_is_planned_from_the_bay_exit(agent):
    """Negative control in the next test: from the bay node itself the route starts at the bay, as before."""
    a = agent
    a.leave_bay()
    a.pose = (BAY[0] + 0.20, BAY[1], 3.1416)                             # backed out along the docking line
    _finish_backup(a, True)
    assert len(a.routes) == 1 and a.routes[0].start_id == 2 and a.routes[0].goal_id == 4
    assert a.phase == 'routing'


def test_switched_off_it_turns_at_the_bay_as_before(agent):
    a = agent
    a.bay_exit_backup = 0.0
    a.leave_bay()
    assert a.backup_sent == [] and len(a.routes) == 1 and a.routes[0].start_id == 1


def test_a_resent_goal_backs_only_the_distance_left(agent):
    """A lost acknowledgement is resent from the robot's pose: it never backs further than bay_exit_backup_m."""
    a = agent
    a.leave_bay()
    a.pose = (BAY[0] + 0.12, BAY[1], 3.1416)
    a.leave_bay(retry=True)
    assert len(a.backup_sent) == 2 and a.backup_sent[1].target.x == pytest.approx(0.08)
    a.pose = (BAY[0] + 0.18, BAY[1], 3.1416)                              # within 3 cm: no more backing
    a.leave_bay(retry=True)
    assert len(a.backup_sent) == 2 and len(a.routes) == 1 and a.routes[0].start_id == 2


def test_a_blocked_back_up_still_leaves(agent):
    """Something behind the robot (a person in the docking line): leave from where it stopped, never wait in the bay."""
    a = agent
    a.leave_bay()
    _finish_backup(a, False, error_code=712)                             # BackUp aborted at once, still at the bay
    assert len(a.routes) == 1 and a.routes[0].start_id == 1


def test_three_unanswered_sends_leave_without_backing(agent):
    a = agent
    a.leave_bay()
    for _ in range(2):
        a.leave_bay(retry=True)
    assert len(a.backup_sent) == 3
    a.leave_bay(retry=True)
    assert len(a.backup_sent) == 3 and len(a.routes) == 1


def test_bins_are_left_without_backing(agent):
    a = agent
    a.last_node, a.pose = 4, (-10.0, 7.0, 0.0)                         # an aisle_bay (a storage bin)
    a.legs = [Leg(3, 0.0, RobotState.TO_DROP)]
    a.leave_bay()
    assert a.backup_sent == [] and len(a.routes) == 1 and a.routes[0].start_id == 4


def test_a_late_result_after_the_robot_moved_on_is_ignored(agent):
    a = agent
    a.leave_bay()
    a.pose = (BAY[0] + 0.20, BAY[1], 3.1416)
    _finish_backup(a, True)
    _finish_backup(a, True)                                              # the re-polled result, answered twice
    assert len(a.routes) == 1
