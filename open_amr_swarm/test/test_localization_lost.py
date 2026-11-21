"""Stall reproducer B5r3 (2026-11-21): backing out of the stall pinned on a rack upright, the robot pivoted 85 deg and AMCL
walked its estimate 4 m away in 20 s (`stuck_B5r3/repro.log` amcl_err 0.16 -> 3.21 m over sim 45-65 s); the agent kept
sending paths MPPI rejected ('Resulting plan has 0 poses in it', believed (-4.70,-6.72) vs the slice at (-6.00,-8.30))
and ran its recoveries on the wrong pose. Now: an AMCL correction at the robot's position above loc_jump_m within
loc_window_s stops the robot, gives an empty-deck task back and holds STUCK until an initialpose arrives."""
import json
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

from open_amr_msgs.msg import Claim, RobotState, Task
from open_amr_swarm.agent import Leg, SwarmAgent
from open_amr_swarm.localization import LocWatch, compose


# ------------------------------------------------------------------ the watch alone
def test_compose_rotates_then_translates():
    x, y, yaw = compose((1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.1))
    assert (round(x, 6), round(y, 6), round(yaw, 6)) == (1.0, 3.0, round(math.pi / 2 + 0.1, 6))


def test_steady_map_odom_is_no_correction_however_far_the_robot_drives():
    w = LocWatch(20.0, 1.0)
    for k in range(100):
        assert w.update(k * 0.2, (5.0, -3.0, 0.4), (0.1 * k, 0.02 * k, 0.01 * k)) == pytest.approx(0.0, abs=1e-9)


def test_b5r3_drift_of_3_m_in_20_s_is_lost():
    w = LocWatch(20.0, 1.0)
    d = 0.0
    for k in range(101):                          # map->odom walks 3.0 m in 20 s while odometry barely moves
        d = w.update(k * 0.2, (0.03 * k * 0.6, 0.03 * k * 0.8, 0.0), (0.0, -0.005 * k, 0.0))
    assert d == pytest.approx(3.0, abs=0.05) and w.lost(d)


def test_slow_drift_below_the_threshold_within_the_window_is_kept():
    w = LocWatch(20.0, 1.0)
    d = 0.0
    for k in range(301):                          # 0.6 m over 60 s: 0.2 m per window
        d = max(d, w.update(k * 0.2, (0.002 * k, 0.0, 0.0), (0.0, 0.0, 0.0)))
    assert d == pytest.approx(0.2, abs=0.01) and not w.lost(d)


def test_correction_is_measured_at_the_robot_not_at_the_odom_origin():
    """A 2 deg yaw correction moves map->odom's translation by nothing but the robot 10 m out in odom by 0.35 m."""
    w = LocWatch(20.0, 1.0)
    w.update(0.0, (0.0, 0.0, 0.0), (10.0, 0.0, 0.0))
    d = w.update(1.0, (0.0, 0.0, math.radians(2.0)), (10.0, 0.0, 0.0))
    assert d == pytest.approx(10.0 * 2 * math.sin(math.radians(1.0)), abs=1e-6) and not w.lost(d)


def test_zero_threshold_switches_it_off():
    w = LocWatch(20.0, 0.0)
    w.update(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert not w.lost(w.update(1.0, (9.0, 0.0, 0.0), (0.0, 0.0, 0.0)))


# ------------------------------------------------------------------ the agent
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
    a.sent, a.states = [], []
    a.send = lambda pub, msg: a.sent.append(msg)
    a.publish_state = lambda: a.states.append(a.status)
    a.odom_pose = lambda: None
    a.synced = True
    yield a
    a.destroy_node()
    rclpy.shutdown()


def driving(a, loaded=False):
    a.route, a.last_node, a.next_idx = [2, 3, 4], 3, 2
    a.path, a.node_idx, a.stop_at = Path(), [0, 1, 2], set()
    a.pose, a.reserved = (3.4, -0.2, 0.0), [3, 4]
    a.leg = Leg(4, 4.0, RobotState.TO_PICK, final_precise=True)
    a.legs, a.phase, a.status, a.loaded = [], 'driving', RobotState.TO_PICK, loaded
    a.task = Task(task_id='t0004', type=Task.GOTO, origin_id='n4')


def drift(a, metres, seconds=20.0, t0=None):
    """Feed the watch 5 Hz samples in which AMCL moves map->odom `metres` along x over `seconds`, odometry still."""
    t0 = a.t if t0 is None else t0
    n = int(seconds * 5)
    for k in range(n + 1):
        a.t = t0 + k * 0.2
        a.check_localization(a.t, (metres * k / n, 0.0, 0.0), (3.4, -0.2, 0.0))


def test_lost_pose_stops_gives_the_empty_task_back_and_holds_stuck(agent):
    a = agent
    driving(a)
    drift(a, 3.0)
    assert a.loc_lost is not None and 'AMCL moved my pose' in a.loc_lost
    assert a.phase == 'lost' and a.status == RobotState.STUCK and a.task is None
    assert any(isinstance(m, Claim) and m.action == Claim.RELEASE and m.resource_id == 't0004' for m in a.sent)
    a.check_peers = lambda: True
    a.update_pose = lambda: True
    a.drive = lambda: pytest.fail('drove on a lost pose')
    a.auction = lambda: pytest.fail('bid on a lost pose')
    a.tick()
    assert a.states[-1] == RobotState.STUCK


def test_loaded_robot_keeps_its_task_and_resumes_the_leg_after_relocalization(agent):
    a = agent
    driving(a, loaded=True)
    drift(a, 3.0)
    assert a.phase == 'lost' and a.task is not None and a.legs and a.legs[0].target == 4 and a.leg is None
    a.on_initialpose(PoseWithCovarianceStamped())
    assert a.loc_lost is None and a.phase == 'retry' and a.status == RobotState.IDLE


def test_empty_robot_goes_idle_after_relocalization(agent):
    a = agent
    driving(a)
    drift(a, 3.0)
    a.on_initialpose(PoseWithCovarianceStamped())
    assert a.loc_lost is None and a.phase == 'idle' and a.status == RobotState.IDLE


def test_ordinary_corrections_change_nothing(agent):
    a = agent
    driving(a)
    drift(a, 0.3)                                  # c64's AMCL error peaked at 0.27 m
    assert a.loc_lost is None and a.phase == 'driving' and a.task is not None


def test_an_initialpose_jump_is_not_a_lost_pose(agent):
    """The agent's own bay relocalization (or an operator's) moves map->odom at once; the 2 s hold-off covers it."""
    a = agent
    driving(a)
    drift(a, 0.0, seconds=4.0)
    a.on_initialpose(PoseWithCovarianceStamped())
    a.t += 0.4
    a.check_localization(a.t, (0.0, 0.0, 0.0), (3.4, -0.2, 0.0))       # AMCL has not applied it yet
    a.t += 0.4
    a.check_localization(a.t, (2.5, 0.0, 0.0), (3.4, -0.2, 0.0))       # applied: 2.5 m at once
    for k in range(10):
        a.t += 0.2
        a.check_localization(a.t, (2.5, 0.0, 0.0), (3.4, -0.2, 0.0))
    assert a.loc_lost is None and a.phase == 'driving'
