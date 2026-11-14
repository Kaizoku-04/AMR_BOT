"""A resent transfer goal that the busy cell rejects (review 2026-11-05 finding 2): the first goal may be running with
only its acceptance lost (the cell takes one goal at a time), so the station never books the job as failed on that
rejection; it settles it from the cell's heartbeat: BUSY = still running ('deck_clear' serves the robot), done with the
transfer count up = succeeded, idle with the count unchanged = never ran (sent again), faulted = failed. The first test
is the review's reproducer, which failed on 4206a0a."""
import json

import pytest
import rclpy
from action_msgs.msg import GoalStatus

from open_amr_msgs.action import ArmTransfer
from open_amr_msgs.msg import ArmCellState
import open_amr_swarm.station_agent as station_agent
from open_amr_swarm.station import DEPALLETIZE, PREPARE
from open_amr_swarm.station_agent import StationAgent


class Clock:
    t = 100.0

    def monotonic(self):
        return self.t


class Fut:
    """A future whose callbacks the test fires by hand (nothing fires = the answer was lost)."""

    def __init__(self, value=None):
        self.cbs, self.value = [], value

    def add_done_callback(self, cb):
        self.cbs.append(cb)

    def result(self):
        return self.value

    def fire(self, value=None):
        self.value = value if value is not None else self.value
        for cb in self.cbs:
            cb(self)


class Handle:
    def __init__(self, accepted=True):
        self.accepted, self.cancelled, self.result_futs = accepted, False, []

    def cancel_goal_async(self):
        self.cancelled = True

    def get_result_async(self):
        f = Fut()
        self.result_futs.append(f)
        return f


@pytest.fixture
def station(tmp_path, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(station_agent, 'wallclock', clock)
    feats = [{'type': 'Feature', 'properties': {'id': 0, 'metadata': {'name': 'receiving_bay', 'kind': 'bay'}},
              'geometry': {'type': 'Point', 'coordinates': [0.0, 0.0]}}]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'cell:=true'])
    s = StationAgent()
    s.clock = clock
    s.sent = []
    s.client.server_is_ready = lambda: True
    s.client.send_goal_async = lambda g, feedback_callback=None: s.sent.append(Fut()) or s.sent[-1]
    yield s
    s.destroy_node()
    rclpy.shutdown()


def prepare(s):
    job = s.st.step(0.0, None, False)
    assert job is not None and job.kind == PREPARE
    s.send(job)
    return job


def cell(s, state, cycles=5, phase='', carton=False, fault=''):
    s.on_cell(ArmCellState(state=state, cycles=cycles, phase=phase, carton_on_tool=carton, fault=fault))


def orphaned_prepare(s):
    """PREPARE sent with the cell READY at 5 transfers; its acceptance lost; the resend rejected by the busy cell."""
    cell(s, ArmCellState.READY)
    job = prepare(s)
    s.clock.t += 4.0
    s.guard_lost()
    s.sent[1].fire(Handle(accepted=False))
    return job


def test_resend_rejected_by_the_busy_cell_while_the_first_goal_runs(station):
    cell(station, ArmCellState.READY)
    prepare(station)
    stock_after_pick = list(station.st.pallets)        # the station took the carton off its count when it sent PREPARE
    station.clock.t += 4.0
    station.guard_lost()                                # resent (the first acceptance never arrives)
    station.sent[1].fire(Handle(accepted=False))        # the busy cell rejects the duplicate
    assert station.st.pallets == stock_after_pick, f'carton booked back on the pallet: {station.st.pallets}'
    nxt = station.st.step(1.0, None, False)
    assert nxt is None, f'a new {nxt} job while the first goal still runs on the cell'


def test_orphaned_prepare_that_completes_is_booked_as_done(station):
    job = orphaned_prepare(station)
    station.clock.t += 1.0
    cell(station, ArmCellState.BUSY, phase='picking')
    station.settle_orphan(10.0)
    assert station.st.job is job                        # still running
    station.clock.t += 6.0
    cell(station, ArmCellState.HOLDING, cycles=6, carton=True)
    station.settle_orphan(16.0)
    assert station.st.job is None and station.st.held == (job.pallet, job.slot) and station.orphan is None
    assert len(station.sent) == 2                       # nothing sent again


def test_orphaned_job_the_cell_never_ran_is_sent_again(station):
    job = orphaned_prepare(station)
    station.clock.t += 1.0
    cell(station, ArmCellState.READY)                   # idle, transfer count unchanged: rejected for another reason
    station.settle_orphan(10.0)
    assert len(station.sent) == 3 and station.st.job is job and station.orphan is None


def test_heartbeat_from_before_the_rejection_settles_nothing(station):
    job = orphaned_prepare(station)
    cell(station, ArmCellState.READY)                   # received in the same instant as the rejection
    station.settle_orphan(10.0)
    assert len(station.sent) == 2 and station.st.job is job and station.orphan is job


def test_orphaned_depalletize_serves_the_robot_on_deck_clear(station):
    cell(station, ArmCellState.HOLDING, carton=True)
    station.st.held = (0, 23)                           # a pre-picked carton
    job = station.st.step(0.0, ('amr_1', 't9'), False)
    assert job is not None and job.kind == DEPALLETIZE
    station.send(job)
    station.clock.t += 4.0
    station.guard_lost()
    station.sent[1].fire(Handle(accepted=False))
    assert station.st.job is job
    station.clock.t += 1.0
    cell(station, ArmCellState.BUSY, phase='deck_clear')
    station.settle_orphan(5.0)
    assert station.st.served == 't9'                    # the robot may leave
    station.clock.t += 3.0
    cell(station, ArmCellState.READY, cycles=6)
    station.settle_orphan(8.0)
    assert station.st.job is None and station.orphan is None


def test_orphaned_job_ending_in_a_cell_fault_fails_with_the_carton_on_the_tool(station):
    job = orphaned_prepare(station)
    booked = []
    station.st.finished = lambda now, ok, carton, why='': booked.append((ok, carton))
    station.clock.t += 1.0
    cell(station, ArmCellState.FAULT, carton=True, fault='vacuum lost')
    station.settle_orphan(10.0)
    assert booked == [(False, 2)] and station.orphan is None    # 2 = CARTON_TOOL


def test_first_send_rejected_is_still_a_failure(station):
    cell(station, ArmCellState.READY)
    prepare(station)
    stock = list(station.st.pallets)
    station.sent[0].fire(Handle(accepted=False))        # not a resend: the cell refused the job itself
    assert station.st.job is None and station.orphan is None
    assert sum(station.st.pallets) == sum(stock) + 1    # the carton never left the pallet
