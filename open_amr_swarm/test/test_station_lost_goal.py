"""A cell station's transfer goal whose request, acceptance or result is lost (DDS under load) is resent / re-requested
(chain 39 base2: the receiving station's first prepare goal never reached the cell; the station waited the whole
shift for its answer and no carton came in)."""
import json

import pytest
import rclpy
from action_msgs.msg import GoalStatus

from open_amr_msgs.action import ArmTransfer
import open_amr_swarm.station_agent as station_agent
from open_amr_swarm.station import PREPARE
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


def test_lost_request_is_resent(station):
    prepare(station)
    assert len(station.sent) == 1
    station.clock.t += 2.0
    station.guard_lost()
    assert len(station.sent) == 1                       # not overdue yet
    station.clock.t += 2.0
    station.guard_lost()
    assert len(station.sent) == 2                       # 4 s > ack_timeout 3 s: sent again
    h = Handle()
    station.sent[1].fire(h)
    assert station.goal is h and not station.sending


def test_late_acceptance_of_an_abandoned_goal_is_cancelled(station):
    prepare(station)
    station.clock.t += 4.0
    station.guard_lost()                                # resent
    late = Handle()
    station.sent[0].fire(late)                          # the first acceptance arrives after all
    assert late.cancelled and station.goal is None and station.sending
    h = Handle()
    station.sent[1].fire(h)
    assert station.goal is h and not h.cancelled


def test_lost_result_is_requested_again_and_handled_once(station):
    job = prepare(station)
    h = Handle()
    station.sent[0].fire(h)
    assert len(h.result_futs) == 1
    station.clock.t += 31.0
    station.guard_lost()
    assert len(h.result_futs) == 2                      # re-requested after result_poll 30 s
    done = type('R', (), {'status': GoalStatus.STATUS_SUCCEEDED, 'result': ArmTransfer.Result(carton=1)})()
    station.st_finished = []
    station.st.finished = lambda *a, **k: station.st_finished.append(a)
    h.result_futs[1].fire(done)
    h.result_futs[0].fire(done)                         # the first answer, late: ignored
    assert len(station.st_finished) == 1 and station.goal is None
    assert job.kind == PREPARE


def test_no_resend_of_a_job_the_station_dropped(station):
    prepare(station)
    station.st.job = None                               # e.g. dropped on a cell fault
    station.clock.t += 4.0
    station.guard_lost()
    assert len(station.sent) == 1
