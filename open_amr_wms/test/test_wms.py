"""open_amr_wms beyond the nine duties: mission_generator's own tests ported, persistence, restart from the store and the
heartbeats, SKU / box inventory, the same throughput as mission_generator, and the ROS node."""
import pytest
import rclpy

from open_amr_msgs.msg import Claim, StationState, Task

from open_amr_fleet_mock.fleet import Fleet
from open_amr_fleet_mock.harness import run
from open_amr_wms.store import Store

from conftest import GEOJSON, NODES, Tap


def _station(w, bay, state, n=32):
    w.node.on_station(StationState(station_id=bay, bay_node=bay, state=state, boxes_available=n))


def _open(w, kind):
    return [k for k, m in w.node.meta.items() if m['kind'] == kind and m['done'] is None]


# ---- ported from open_amr_swarm/test/test_wms_kind_share.py
def test_an_arm_back_from_a_fault_gets_its_share_at_once(wms):
    w = wms(open_tasks=4)
    w.t = 1.0
    _station(w, 'receiving_bay', StationState.IDLE)
    _station(w, 'outbound_bay', StationState.FAULT)
    w.node.tick()
    assert len(_open(w, 'inbound')) == 4 and _open(w, 'outbound') == []
    _station(w, 'outbound_bay', StationState.IDLE)
    w.node.tick()
    assert len(_open(w, 'outbound')) == 2
    w.node.tick()
    assert len(_open(w, 'outbound')) == 2 and len(_open(w, 'inbound')) == 4


def test_with_both_arms_up_the_mix_is_unchanged(wms):
    w = wms(open_tasks=4)
    w.t = 1.0
    _station(w, 'receiving_bay', StationState.IDLE)
    _station(w, 'outbound_bay', StationState.IDLE)
    w.node.tick()
    assert len(_open(w, 'inbound')) == 2 and len(_open(w, 'outbound')) == 2


# ---- ported from open_amr_swarm/test/test_station_stale_log.py
def test_a_stale_station_is_logged_once_each_way(wms):
    w = wms()
    w.t = 1.0
    _station(w, 'outbound_bay', StationState.IDLE)
    w.node.tick()
    w.t = 10.0
    w.node.tick(); w.node.tick()
    stale = [m for lvl, m in w.logged if 'no heartbeat from the station at outbound_bay' in m]
    assert len(stale) == 1 and w.node.arm_room('outbound_bay', 'outbound') == 0
    _station(w, 'outbound_bay', StationState.IDLE)
    w.node.tick()
    assert [m for lvl, m in w.logged if 'outbound_bay heard again' in m]


# ---- new
def test_no_work_before_a_station_is_heard_unless_there_are_none(wms):
    w = wms()
    w.t = 1.0
    w.node.tick()
    assert w.node.meta == {}                         # stations may still be starting
    w.t = 7.0
    w.node.tick()
    assert len(w.node.meta) == 8                     # none heard for station_wait_s: a run without station agents


def test_inventory_carries_sku_and_box_through_inbound_and_outbound(graph, wms):
    f = Fleet(graph, seed=1)
    w = wms()
    run(f, w, 1200)
    done = w.done()
    inb = [m for m in done if m['kind'] == 'inbound']
    out = [m for m in done if m['kind'] == 'outbound']
    assert inb and out
    for m in inb:
        later = [x for x in done if x['bin'] == m['bin'] and x['done'] > m['done']]
        if not later:
            assert w.node.bins[m['bin']] == dict(full=True, sku=m['sku'], box_id=m['box_id'])
    for m in out:
        assert m['sku'] and m['box_id'].startswith('BX')
    ids = [b['box_id'] for b in w.node.bins.values() if b['full']]
    assert len(ids) == len(set(ids))                 # every stored box is unique


def test_everything_is_in_the_store(graph, tmp_path, wms):
    db = str(tmp_path / 'wms.db')
    f = Fleet(graph, seed=1)
    w = wms(store=Store(db))
    run(f, w, 600)
    w.node.store.close()
    s = Store(db)
    tasks = s.tasks()
    assert set(tasks) == set(w.node.meta)
    assert {t for t, (m, st) in tasks.items() if st == 'DONE'} == {m['task_id'] for m in w.done()}
    assert s.bins() == w.node.bins
    assert s.counter('seq') == w.node.seq


def test_restart_mid_shift_resumes_from_store_and_heartbeats(graph, tmp_path, wms):
    """The WMS process dies at t = 600 s and is back 120 s later (two DONEs fall into the gap) with the same database; the fleet keeps working.
    Nothing is lost or done twice, every claimed task stays with the robot whose heartbeat shows it, and the shift
    goes on at the same rate."""
    db = str(tmp_path / 'wms.db')
    f = Fleet(graph, seed=2)
    tap = Tap(f)
    w1 = wms(store=Store(db), seed=11)
    run(f, w1, 600)
    held = {r.task.task_id: r.rid for r in f.robots if r.task}
    f.bus.unsubscribe('wms')                          # the process dies
    w1.node.store.close()

    class Dead:                                       # 120 s without a WMS: robots finish what they hold
        def attach(self, bus): pass
        def step(self, now): pass
    run(f, Dead(), 720, start_s=600.1)
    # the DONEs of those 120 s were never heard: the heartbeats can't repeat them
    w2 = wms(store=Store(db), seed=12)
    assert w2.node.restarted
    run(f, w2, 1800, start_s=720.1)
    done = [c.resource_id for _, s, c in tap.get('/swarm/claims') if c.action == Claim.DONE]
    assert len(done) == len(set(done))                # nobody delivered a task twice
    new = [m for _, s, m in tap.get('/swarm/tasks') if s == 'wms' and m.round == 0]
    ids = [m.task_id for m in new]
    assert len(ids) == len(set(ids))                  # the restarted WMS never reused a task id
    m2 = w2.node.meta
    for tid, rid in held.items():
        m = m2[tid]
        assert m['done'] is not None or m['robot'] == rid or m['stranded'] or m['releases']
    after = [m for m in w2.done() if m['done'] > 900]
    assert len(after) >= 12                           # ~76/h for 900 s: the shift goes on
    bins = [m['bin'] for m in m2.values() if m['done'] is None and m['bin'] and not m['stranded']]
    assert len(bins) == len(set(bins))
    # no stale claim: every open claimed task is shown by its robot's heartbeat (a task finished while the WMS was down
    # sent its DONE into the void; without resume_check it stayed 'claimed' for good and its bin stayed promised)
    shown = {r.task.task_id for r in f.robots if r.task}
    stale = [t for t, m in m2.items() if m['done'] is None and m['claimed'] is not None and not m['stranded']
             and t not in shown]
    assert stale == []


def test_restart_without_heartbeats_reoffers_nothing_claimed_too_early(graph, tmp_path, wms):
    db = str(tmp_path / 'wms.db')
    w1 = wms(store=Store(db))
    w1.t = 1.0
    _station(w1, 'receiving_bay', StationState.IDLE); _station(w1, 'outbound_bay', StationState.IDLE)
    w1.node.tick()
    tid = sorted(w1.node.meta)[0]
    w1.node.on_claim(Claim(resource_type=Claim.TASK, resource_id=tid, robot_id='amr_1', action=Claim.CLAIM))
    w1.node.store.close()
    w2 = wms(store=Store(db))
    w2.t = 50.0
    w2.node.tick()
    assert w2.node.meta[tid]['robot'] == 'amr_1'      # no heartbeat yet: no verdict


def test_same_throughput_as_mission_generator(graph, mg, wms):
    """open_amr_wms replaces mission_generator without costing throughput: 5 seeds x 900 s each."""
    a = b = 0
    for seed in range(1, 6):
        f = Fleet(graph, seed=seed); x = mg(seed=seed); run(f, x, 901); a += len(x.done())
        f = Fleet(graph, seed=seed); y = wms(seed=seed); run(f, y, 901); b += len(y.done())
    assert b >= 0.9 * a, (a, b)


def test_ros_node_constructs_and_ticks(tmp_path):
    from open_amr_wms.node import WmsNode
    rclpy.init(args=['--ros-args', '-p', f'graph:={GEOJSON}', '-p', f'graph_nodes:={NODES}',
                     '-p', f'db:={tmp_path / "w.db"}'])
    try:
        n = WmsNode()
        sent = []
        n.pubs['/swarm/tasks'].publish = sent.append
        t = [10.0]
        n.wms.now = lambda: t[0]
        n.wms.tick()
        t[0] = 16.0                                  # no station heard for station_wait_s: offers start
        n.wms.tick()
        assert len(sent) == 8 and all(isinstance(t, Task) for t in sent)
        n.destroy_node()
    finally:
        rclpy.shutdown()
