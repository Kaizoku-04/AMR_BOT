"""Each failure the mock can play (fleet.py "Commands"), from a scenario file and from a command line, and what the
fleet does about it — the same rules the swarm agent applies (Swarm Design "Fail-safe core")."""
import os

import pytest
import rclpy

from open_amr_msgs.msg import Claim, RobotState, StationState

from open_amr_fleet_mock.fleet import Fleet, load_scenario
from open_amr_fleet_mock.harness import run

from conftest import GEOJSON, Tap

SCEN = os.path.join(os.path.dirname(__file__), '..', 'scenarios')


def test_silent_robot_stops_publishing_and_comes_back(graph, mg):
    f = Fleet(graph, seed=1, scenario=[{'at': 50, 'do': 'silent', 'target': 'amr_1', 'for_s': 30}])
    tap = Tap(f)
    run(f, mg(), 120)
    hb = [t for t, _, m in tap.get('/swarm/state') if m.robot_id == 'amr_1']
    assert not [t for t in hb if 50.1 < t < 80] and [t for t in hb if t > 80.5]


def test_silent_claimant_task_is_given_back_by_the_lowest_live_peer(graph, mg):
    f = Fleet(graph, seed=1)
    tap, w = Tap(f), mg()
    run(f, w, 40)
    r = next(x for x in f.robots if x.task and not x.loaded)
    tid = r.task.task_id
    f.inject('silent', r.rid, 60)
    run(f, w, 70, start_s=40)
    rel = [c for _, s, c in tap.get('/swarm/claims') if c.action == Claim.RELEASE and c.resource_id == tid]
    live = sorted(x.rid for x in f.robots if x is not r)
    assert rel and rel[0].robot_id == r.rid and rel[0].by == live[0]


def test_stuck_robot_gives_its_empty_task_back_after_120_s(graph, mg):
    f = Fleet(graph, seed=2)
    tap, w = Tap(f), mg()
    run(f, w, 40)
    r = next(x for x in f.robots if x.task and not x.loaded)
    tid = r.task.task_id
    f.inject('stuck', r.rid, 300)
    run(f, w, 140, start_s=40)
    assert any(m.status == RobotState.STUCK for t, _, m in tap.get('/swarm/state') if m.robot_id == r.rid)
    assert not [c for _, _, c in tap.get('/swarm/claims') if c.action == Claim.RELEASE and c.resource_id == tid]
    run(f, w, 170, start_s=140)
    rel = [c for _, s, c in tap.get('/swarm/claims') if c.action == Claim.RELEASE and c.resource_id == tid]
    assert rel and rel[0].robot_id == r.rid and not rel[0].by


def test_give_back_releases_and_the_wms_reoffers(graph, mg):
    f = Fleet(graph, seed=3)
    tap, w = Tap(f), mg()
    run(f, w, 40)
    r = next(x for x in f.robots if x.task and not x.loaded)
    tid, rnd = r.task.task_id, r.task.round
    assert 'give_back' in f.inject_line(f'give_back {r.rid}')
    run(f, w, 60, start_s=40)
    assert any(m.task_id == tid and m.round > rnd for _, _, m in tap.get('/swarm/tasks'))


def test_arm_fault_and_recovery_in_the_station_heartbeat(graph, mg):
    f = Fleet(graph, seed=1, scenario=[{'at': 30, 'do': 'arm_fault', 'target': 'arm_receiving', 'for_s': 60}])
    tap = Tap(f)
    run(f, mg(), 120)
    st = [(t, m.state) for t, _, m in tap.get('/swarm/stations') if m.station_id == 'arm_receiving']
    assert all(s == StationState.FAULT for t, s in st if 30.5 < t < 90)
    assert any(s != StationState.FAULT for t, s in st if t > 90.5)


def test_empty_robot_waiting_on_a_faulted_arm_gives_up_after_30_s(graph, mg):
    f = Fleet(graph, seed=1, scenario=[{'at': 20, 'do': 'arm_fault', 'target': 'arm_receiving', 'for_s': 600}])
    tap = Tap(f)
    run(f, mg(), 200)
    gave = [c for t, _, c in tap.get('/swarm/claims') if c.action == Claim.RELEASE and not c.by and t > 50]
    assert gave


def test_starved_station_reports_starved_with_nothing_available(graph, mg):
    f = Fleet(graph, seed=1, scenario=[{'at': 30, 'do': 'starve', 'target': 'arm_receiving', 'for_s': 200}])
    tap = Tap(f)
    run(f, mg(), 300)
    st = [(t, m) for t, _, m in tap.get('/swarm/stations') if m.station_id == 'arm_receiving']
    mid = [m for t, m in st if 40 < t < 225]
    assert mid and all(m.boxes_available == 0 for m in mid) and any(m.state == StationState.STARVED for m in mid)
    assert any(m.boxes_available > 0 for t, m in st if t > 235)


def test_message_drop_rate_from_a_scenario_line(graph, mg):
    f = Fleet(graph, seed=1, scenario=[{'at': 10, 'do': 'drop_rate', 'target': 'all', 'value': 0.2}])
    run(f, mg(), 60)
    assert f.bus.drop_rate == 0.2 and 0.1 < f.bus.dropped / max(f.bus.sent, 1) < 0.2


def test_bad_commands_are_refused(graph):
    f = Fleet(graph, seed=1)
    for bad in ('explode amr_1', 'silent amr_99 3', 'arm_fault arm_nowhere 3', ''):
        with pytest.raises(ValueError):
            f.inject_line(bad)


def test_the_shipped_scenario_files_load_and_run(graph, mg):
    for name in ('failures.yaml', 'c50_truth.yaml'):
        events, kw = load_scenario(os.path.join(SCEN, name))
        f = Fleet(graph, scenario=events, **kw)
        w = mg()
        run(f, w, 2700 if events else 300)
        assert w.done()


def test_ros_node_runs_publishes_clock_and_takes_inject(tmp_path):
    from open_amr_fleet_mock.node import MockFleetNode
    rclpy.init(args=['--ros-args', '-p', f'graph:={GEOJSON}', '-p', 'rtf:=0.0', '-p', 'robots:=3'])
    try:
        n = MockFleetNode()
        clocks = []
        n.pub_clock.publish = clocks.append
        n.tick()
        assert clocks and clocks[-1].clock.sec >= 5 and len(n.fleet.robots) == 3
        from rclpy.parameter import Parameter
        ok = n.set_parameters([Parameter('inject', value='stuck amr_1 30')])[0]
        bad = n.set_parameters([Parameter('inject', value='stuck amr_9 30')])[0]
        assert ok.successful and not bad.successful and 'amr_9' in bad.reason
        n.destroy_node()
    finally:
        rclpy.shutdown()
