"""Chain 64 (2026-11-20): amr_2 stood 0.44 m off aisle_0's axis, nose corner against the rack, from sim 328 to the end of
the shift. Nav2 aborted 60 times ('Failed to make progress'), the agent sent the same path again every time, gave
t0004 back twice after 120 s and won it again within 2 s (the robot next to a task is its cheapest bidder), and the
operator's ACK was a no-op. Now: a bounded straight back-up + re-plan after repeated aborts with nobody near, STUCK only
after recover_max of them; a task given back for no progress is not bid on again (cooldown), and no task at all before
the robot has moved 0.3 m or recovered."""
import json
import math
import time as wallclock

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose2D, PoseStamped
from nav_msgs.msg import Path

from open_amr_msgs.msg import Claim, OperatorCommand, RobotState, Task
from open_amr_swarm.agent import Leg, SwarmAgent


def _future():
    return type('F', (), {'add_done_callback': lambda s, cb: None})()


@pytest.fixture
def agent(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(8)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(7)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_2', '-p', 'fleet:=["amr_2", "amr_1"]'])
    a = SwarmAgent()
    a.t = 1000.0
    a.now = lambda: a.t
    a.sent, a.backups, a.routes, a.lines = [], [], [], []
    a.send = lambda pub, msg: a.sent.append(msg)
    a.synced = True
    a.backup_client.server_is_ready = lambda: True
    a.backup_client.send_goal_async = lambda g: a.backups.append(g) or _future()
    a.route_client.wait_for_server = lambda timeout_sec=None: True
    a.route_client.send_goal_async = lambda g: a.routes.append(g) or _future()
    a.follow_client.send_goal_async = lambda g: _future()
    a.clear_local.service_is_ready = lambda: False
    a.odom_pose = lambda: None
    a.confirm = lambda: set(range(8))                     # every peer has acknowledged every node
    yield a
    a.destroy_node()
    rclpy.shutdown()


def drive_to_bay(a, x=3.44, y=-0.2, fails=3):
    """Driving the last edge n2 -> n4 (the pick node), the robot jammed short of it, the controller aborted `fails` times."""
    a.route, a.last_node, a.next_idx = [2, 3, 4], 3, 2
    a.path = Path(poses=[PoseStamped() for _ in range(3)])
    for i, p in enumerate(a.path.poses):
        p.pose.position.x = 2.0 + i
    a.node_idx = [0, 1, 2]
    a.stop_at = set()
    a.pose, a.reserved = (x, y, 1.9), [3, 4]
    a.leg = Leg(4, 4.0, RobotState.TO_PICK, final_precise=True)
    a.legs, a.phase, a.status = [], 'driving', RobotState.TO_PICK
    a.follow_end, a.follow_state, a.leg_fail = 4, GoalStatus.STATUS_ABORTED, fails
    a.task = Task(task_id='t0004', type=Task.GOTO, origin_id='n4')


def peer_at(a, x, y):
    m = type('S', (), {'robot_id': 'amr_1', 'pose': Pose2D(x=x, y=y), 'reserved_nodes': [], 'wait_s': 0.0})()
    a.peers['amr_1'] = (m, wallclock.monotonic())


def test_fourth_abort_backs_up_and_replans(agent):
    a = agent
    drive_to_bay(a)
    a.drive()
    assert a.phase == 'recovering' and a.leg_fail == 4
    assert len(a.backups) == 1 and a.backups[0].target.x == pytest.approx(0.5) and a.backups[0].speed == pytest.approx(0.15)
    assert a.follow_end is None and a.recoveries == {4: 1}
    a.recov_handle = h = object()                                  # BackUp succeeded: plan the leg again from here
    res = type('R', (), {'result': type('Res', (), {'error_code': 0})(), 'status': GoalStatus.STATUS_SUCCEEDED})()
    a.on_recov_result(type('F', (), {'result': lambda s: res})(), h)
    assert a.phase == 'retry' and [l.target for l in a.legs] == [4] and a.leg is None
    a.tick_retry = a.next_leg(first=True)
    assert a.phase == 'routing' and a.routes[-1].goal_id == 4 and a.leg_fail == 0


def test_a_blocked_back_up_still_replans_and_counts(agent):
    a = agent
    drive_to_bay(a)
    a.drive()
    a.recov_handle = h = object()
    res = type('R', (), {'result': type('Res', (), {'error_code': 705})(), 'status': GoalStatus.STATUS_ABORTED})()
    a.on_recov_result(type('F', (), {'result': lambda s: res})(), h)
    assert a.phase == 'retry' and a.recoveries == {4: 1}


def test_not_with_a_robot_within_2_5_m_and_stuck_as_before(agent):
    """Negative control: a peer near may be the cause (the traffic rules handle that), so no back-up; the 9th abort is
    STUCK exactly as on main."""
    a = agent
    drive_to_bay(a, fails=3)
    peer_at(a, 3.44, 2.2)
    a.drive()
    assert a.backups == [] and a.phase == 'driving' and a.status != RobotState.STUCK
    for _ in range(5):
        a.follow_state = GoalStatus.STATUS_ABORTED
        a.drive()
    assert a.leg_fail == 9 and a.status == RobotState.STUCK and a.backups == []


def test_bounded_then_stuck(agent):
    a = agent
    for n in (1, 2):
        drive_to_bay(a, fails=3)
        a.recoveries = {4: n - 1}
        a.drive()
        assert a.phase == 'recovering' and a.recoveries == {4: n}
        a.recovery_done('test')
    drive_to_bay(a, fails=3)
    a.recoveries = {4: 2}
    a.drive()
    assert a.phase == 'driving' and a.status != RobotState.STUCK       # recover_max used up: back to the old behaviour
    for _ in range(5):
        a.follow_state = GoalStatus.STATUS_ABORTED
        a.drive()
    assert a.status == RobotState.STUCK and len(a.backups) == 2


def test_switched_off(agent):
    a = agent
    a.recover_backup = 0.0
    drive_to_bay(a)
    a.drive()
    assert a.backups == [] and a.phase == 'driving'


def test_operator_ack_of_a_stuck_controller_gives_fresh_recoveries(agent):
    a = agent
    drive_to_bay(a, fails=9)
    a.status, a.recoveries = RobotState.STUCK, {4: 2}
    a.on_operator(OperatorCommand(command=OperatorCommand.ACK_FAULT, target='amr_2', stamp=TimeMsg(sec=5)))
    assert a.status == RobotState.TO_PICK and a.recoveries == {}
    a.drive()
    assert a.phase == 'recovering' and len(a.backups) == 1


def test_give_back_for_no_progress_blocks_the_same_task_and_every_task_until_moved(agent):
    """chain 64: re-won within 2 s each time because the stuck robot is the cheapest bidder."""
    a = agent
    drive_to_bay(a)
    a.pose = (3.44, -0.2, 1.9)
    a.give_back('no progress for 120 s', stuck=True)
    assert a.phase == 'idle' and a.task is None
    for tid in ('t0004', 't0005'):
        a.tasks[tid] = Task(task_id=tid, type=Task.GOTO, origin_id='n4', stamp=TimeMsg(sec=1000))
    assert a.bid_block(a.tasks['t0004'])[0] == 'unmoved'
    assert a.bid_block(a.tasks['t0005'])[0] == 'unmoved'
    a.pose = (3.60, -0.2, 1.9)                                         # 0.16 m: not enough
    assert a.bid_block(a.tasks['t0005'])[0] == 'unmoved'
    a.pose = (3.80, -0.2, 1.9)                                         # 0.36 m: free for other tasks ...
    assert a.bid_block(a.tasks['t0005']) is None
    assert a.bid_block(a.tasks['t0004'])[0] == 'cooldown:t0004'        # ... but not for the one it failed
    a.t += a.giveback_cooldown + 1
    assert a.bid_block(a.tasks['t0004']) is None


def test_the_odom_frame_measures_the_move_when_known(agent):
    a = agent
    drive_to_bay(a)
    a.odom_pose = lambda: (0.0, 0.0)
    a.give_back('no progress for 120 s', stuck=True)
    a.tasks['t0005'] = Task(task_id='t0005', type=Task.GOTO, origin_id='n4', stamp=TimeMsg(sec=1000))
    a.pose = (9.0, 9.0, 0.0)                                           # AMCL jumped 9 m, odometry did not
    assert a.bid_block(a.tasks['t0005'])[0] == 'unmoved'
    a.odom_pose = lambda: (0.31, 0.0)
    assert a.bid_block(a.tasks['t0005']) is None


def test_other_give_backs_are_not_gated(agent):
    """An unreachable target or an arm out of service says nothing about this robot's ability to drive."""
    a = agent
    drive_to_bay(a)
    a.give_back('n4 unreachable for 60 s')
    a.tasks['t0004'] = Task(task_id='t0004', type=Task.GOTO, origin_id='n4', stamp=TimeMsg(sec=1000))
    assert a.bid_block(a.tasks['t0004']) is None


def test_a_recovery_clears_the_gate(agent):
    a = agent
    drive_to_bay(a)
    a.give_back('no progress for 120 s', stuck=True)
    assert a.moved_gate is not None
    drive_to_bay(a)
    a.drive()
    a.recovery_done('test')
    assert a.moved_gate is None


def test_giving_the_task_back_during_a_recovery_cancels_it(agent):
    a = agent
    drive_to_bay(a)
    a.drive()
    assert a.phase == 'recovering'
    a.give_back('no progress for 120 s', stuck=True)
    assert a.phase == 'idle' and a.recoveries == {} and a.recov_handle is None
