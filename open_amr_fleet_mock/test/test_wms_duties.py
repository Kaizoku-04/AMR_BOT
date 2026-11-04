"""The nine WMS duties of notes/WMS Interface.md §4, each exercised end to end: a WMS against the mock fleet on the bus.

`wms` is parametrised so the same tests run against mission_generator (today's stand-in) and, from milestone 2 on,
open_amr_wms (the replacement must keep every duty). Each test drives the situation the duty is for through the
fleet's own behaviour or a failure injection, and checks what the WMS published.
"""
import pytest

from open_amr_msgs.msg import Claim, OperatorCommand, RobotState, StationState, Task
from builtin_interfaces.msg import Time

from open_amr_fleet_mock.fleet import Fleet
from open_amr_fleet_mock.harness import run

from conftest import Tap

WMS_KINDS = ['mission_generator']


@pytest.fixture(params=WMS_KINDS)
def make_wms(request, mg, tmp_path):
    def make(**kw):
        if request.param == 'mission_generator':
            return mg(**kw)
        raise NotImplementedError(request.param)
    make.kind = request.param
    return make


def meta(w):
    return w.node.meta


def offered(tap, round_=None):
    return [m for _, s, m in tap.get('/swarm/tasks') if round_ is None or m.round == round_]


def test_1_offer_keeps_robots_plus_two_open_with_unique_ids(graph, make_wms):
    f = Fleet(graph, seed=1)
    tap, w = Tap(f), make_wms()
    run(f, w, 300)
    first = offered(tap, 0)
    assert len({t.task_id for t in first}) == len(first)
    open_ = [k for k, m in meta(w).items() if m['done'] is None and not m['stranded']]
    assert len(open_) >= 8          # mission_generator's kind-share top-up can add one beyond open_tasks


def test_2_unclaimed_tasks_are_reoffered_with_the_next_round(graph, make_wms):
    f = Fleet(graph, seed=1, scenario=[{'at': 0, 'do': 'silent', 'target': f'amr_{i}', 'for_s': 1e6} for i in range(6)])
    tap, w = Tap(f), make_wms()
    run(f, w, 15)
    t0 = offered(tap)[0].task_id
    rounds = [m.round for m in offered(tap) if m.task_id == t0]
    assert rounds[:3] == [0, 1, 2]                   # every reoffer_s (4 s): nobody hears it


def test_3_no_work_offered_for_a_faulted_arm_and_reoffers_wait(graph, make_wms):
    f = Fleet(graph, seed=1, scenario=[{'at': 100, 'do': 'arm_fault', 'target': 'arm_outbound', 'for_s': 300}])
    tap, w = Tap(f), make_wms()
    run(f, w, 400)
    during = [m for t, _, m in tap.get('/swarm/tasks') if 102 < t < 400 and m.type == Task.FETCH_FROM_BIN]
    assert during == []                              # neither new outbound tasks nor re-offers while FAULT
    run(f, w, 500, start_s=400)
    assert any(m.type == Task.FETCH_FROM_BIN for t, _, m in tap.get('/swarm/tasks') if t > 400)


def test_3b_receiving_gating_follows_the_staged_boxes(graph, make_wms, request):
    if make_wms.kind == 'mission_generator':
        # found 2026-11-05 with the mock: mission_generator's arm_room() is unlimited until a station's first
        # heartbeat arrives (mission_generator.py arm_room: `if bay not in self.stations: return 1 << 30`), and its
        # first tick runs before that: 4 inbound tasks for 2 staged boxes; the extra robots wait at a STARVED bay
        request.applymarker(pytest.mark.xfail(strict=True, reason='mission_generator offers before hearing stations'))
    f = Fleet(graph, seed=1, receiving_pallets=(2, 0), swap_s=1e6)
    tap, w = Tap(f), make_wms()
    run(f, w, 600)
    inbound = {m.task_id for m in offered(tap, 0) if m.type == Task.FETCH_FROM_STATION}
    assert len(inbound) == 2                         # two boxes staged -> two inbound tasks, ever


def test_4_bin_registry_one_task_per_bin_and_inventory_on_done(graph, make_wms):
    f = Fleet(graph, seed=4)
    tap, w = Tap(f), make_wms()
    bad = []

    def check(now):
        bins = [m['bin'] for m in meta(w).values() if m['done'] is None and m['bin']]
        if len(bins) != len(set(bins)):
            bad.append(now)
    run(f, w, 1200, on_step=check)
    assert bad == []
    inv = tap.get('/wms/inventory')
    done = [c for t, s, c in tap.get('/swarm/claims') if c.action == Claim.DONE]
    assert len(done) > 10 and len(inv) >= len(done)          # republished on every DONE
    last = set(inv[-1][2].occupied_bins)
    for c in done:
        m = meta(w)[c.resource_id]
        later = [x for x in done if meta(w)[x.resource_id]['bin'] == m['bin'] and meta(w)[x.resource_id]['done'] > m['done']]
        if not later:
            assert (m['bin'] in last) == (m['kind'] == 'inbound')


def _first(f, pred, w, start, until):
    """Run from `start` until pred() returns something; return (it, the next step's time)."""
    box = {}

    def stop(now):
        v = pred()
        if v:
            box['v'], box['t'] = v, round(now + 0.1, 6)
        return bool(v)
    run(f, w, until, start_s=start, stop=stop)
    return box.get('v'), box.get('t')


def test_5_silent_robot_blocks_withdraw_and_retarget(graph, make_wms):
    """A robot silent in an aisle: unclaimed work there is withdrawn (CANCEL by wms), nothing new is offered there,
    and a robot carrying an inbound box to a bin that is now cut off is re-targeted to another bin (same task id)."""
    f = Fleet(graph, seed=2)
    tap, w = Tap(f), make_wms()
    run(f, w, 60)
    # a loaded inbound carrier heading for its bin
    carrier, t1 = _first(f, lambda: next((r for r in f.robots if r.loaded and r.task and
                                          r.task.type == Task.FETCH_FROM_STATION and r.status == RobotState.TO_DROP),
                                         None), w, 60, 1500)
    assert carrier is not None
    dest = graph.id(carrier.task.dest_id)
    other = next(r for r in f.robots if r is not carrier and r.task is None and not r.loaded) if \
        any(r is not carrier and r.task is None for r in f.robots) else next(r for r in f.robots if r is not carrier)
    # the other robot stops (dead) on the carrier's bin node: its heartbeat says so, then it goes silent
    other.abort_task() if other.task and not other.loaded else None
    other.last_node, other.edge, other.path, other.phase = dest, None, [dest], 'idle'
    other.pose[:2] = list(graph.pos[dest])
    run(f, w, t1 + 1.0, start_s=t1)
    f.inject('silent', other.rid, 1e6)
    old_bin = carrier.task.bin_id
    run(f, w, t1 + 30, start_s=t1 + 1.0)
    re = [m for _, _, m in tap.get('/swarm/tasks') if m.task_id == carrier.task.task_id and m.bin_id != old_bin]
    assert re and graph.id(re[-1].dest_id) != dest                 # duty 5: re-targeted, same id
    blocked = w.node.blocked_nodes()
    assert dest in blocked
    later = [m for t, _, m in tap.get('/swarm/tasks') if t > t1 + 5 and m.round == 0]
    assert all(graph.id(m.dest_id or m.origin_id) not in blocked and graph.id(m.origin_id) not in blocked for m in later)
    run(f, w, t1 + 400, start_s=t1 + 30)
    assert meta(w)[carrier.task.task_id if carrier.task else re[-1].task_id]['done'] is not None or carrier.task


def test_6_stranded_box_alarm_and_late_done_accepted(graph, make_wms):
    f = Fleet(graph, seed=3)
    w = make_wms()
    run(f, w, 30)
    r, t1 = _first(f, lambda: next((x for x in f.robots if x.loaded and x.task and x.phase == 'driving'), None),
                   w, 30, 1500)
    tid = r.task.task_id
    f.inject('silent', r.rid, 40)
    run(f, w, t1 + 35, start_s=t1)
    assert meta(w)[tid]['stranded']                       # silent 20 s with the box: alarm, not re-offered
    assert any(lvl == 'error' and 'ALARM' in msg for lvl, msg in w.logged)
    run(f, w, t1 + 600, start_s=t1 + 35)
    assert meta(w)[tid]['done'] is not None               # it came back and delivered: DONE accepted


def test_7_reconcile_a_robot_that_missed_its_reauction(graph, make_wms):
    f = Fleet(graph, seed=1)
    tap, w = Tap(f), make_wms()
    run(f, w, 20)
    r, t1 = _first(f, lambda: next((x for x in f.robots if x.task and not x.loaded and x.phase == 'driving'), None),
                   w, 20, 600)
    tid = r.task.task_id
    f.inject('silent', r.rid, 45)                  # peers give its task back at 20 s, another robot wins it
    run(f, w, t1 + 44, start_s=t1)
    assert meta(w)[tid]['robot'] != r.rid             # given back on its behalf, re-offered
    run(f, w, t1 + 80, start_s=t1 + 44)            # back: its heartbeat still says tid
    assert r.task is None or r.task.task_id != tid
    holders = {s.robot_id for t, _, s in tap.get('/swarm/state') if t > t1 + 70 and s.task_id == tid}
    assert len(holders) <= 1


def test_8_operator_release_robot_frees_its_nodes(graph, make_wms):
    f = Fleet(graph, seed=1)
    w = make_wms()
    run(f, w, 30)
    f.inject('silent', 'amr_3', 1e6)
    run(f, w, 40, start_s=30)
    assert w.node.blocked_nodes()
    f.bus.publish('/swarm/operator', 'operator', OperatorCommand(command=OperatorCommand.RELEASE_ROBOT, target='amr_3',
                                                                 issued_by='test', stamp=Time(sec=41)))
    run(f, w, 43, start_s=40)
    assert not w.node.blocked_nodes()


def test_9_state_over_events_with_lossy_messages(graph, make_wms):
    """10 % of every message lost: the shift still runs, no task is done twice, lost CLAIMs are adopted from the
    heartbeats, and at the end no two robots hold the same task."""
    f = Fleet(graph, seed=6, drop_rate=0.10)
    tap, w = Tap(f), make_wms()
    run(f, w, 1800)
    done = [c.resource_id for _, _, c in tap.get('/swarm/claims') if c.action == Claim.DONE]
    assert len(done) >= 20 and len(done) == len(set(done))
    assert f.bus.dropped > 1000
    holders = {}
    for r in f.robots:
        if r.task:
            holders.setdefault(r.task.task_id, []).append(r.rid)
    assert all(len(v) == 1 for v in holders.values())
