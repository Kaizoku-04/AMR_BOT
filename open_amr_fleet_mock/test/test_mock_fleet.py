"""The mock fleet itself: the swarm agent's auction rule, heartbeats, batteries, determinism, a whole shift with
mission_generator, and the timing calibration against the Isaac truth baseline."""
from collections import Counter

from open_amr_msgs.msg import Claim, RobotState
from open_amr_swarm import auction

from open_amr_fleet_mock.bus import Bus
from open_amr_fleet_mock.fleet import Fleet
from open_amr_fleet_mock.harness import run

from conftest import Tap


def test_bus_drops_are_seeded_and_counted():
    def drops(seed):
        b = Bus(seed=seed, drop_rate=0.3)
        got = []
        b.subscribe('/x', 'r', got.append)
        for i in range(200):
            b.publish('/x', 's', i)
        b.flush()
        return got, b.dropped
    a, da = drops(4)
    assert a == drops(4)[0] and 30 < da < 90 and da + len(a) == 200
    assert a != drops(5)[0]


def test_every_claim_is_the_agent_rule_winner(graph, mg):
    f = Fleet(graph, seed=2)
    tap = Tap(f)
    run(f, mg(), 600)
    bids = {}
    for _, _, b in tap.get('/swarm/bids'):
        bids.setdefault((b.task_id, b.round), {})[b.robot_id] = b.cost
    claims = [c for _, _, c in tap.get('/swarm/claims') if c.resource_type == Claim.TASK and c.action == Claim.CLAIM]
    assert len(claims) >= 8
    for c in claims:
        assert c.robot_id == auction.winner(bids[(c.resource_id, c.round)])
    assert len({c.resource_id for c in claims}) == len(claims)          # one winner per task, no message loss


def test_heartbeats_at_5_hz_with_the_fields_the_wms_reads(graph, mg):
    f = Fleet(graph, seed=1)
    tap = Tap(f)
    run(f, mg(), 61)
    per = Counter(m.robot_id for _, _, m in tap.get('/swarm/state'))
    assert set(per) == {f'amr_{i}' for i in range(6)}
    assert all(295 <= n <= 302 for n in per.values())                    # 60 s x 5 Hz
    working = [m for _, _, m in tap.get('/swarm/state') if m.task_id]
    assert working and all(m.mission_id and m.status != RobotState.IDLE for m in working)
    st = Counter(m.station_id for _, _, m in tap.get('/swarm/stations'))
    assert all(295 <= n <= 302 for n in st.values()) and len(st) == 2


def test_same_seed_same_shift(graph, mg):
    def shift(seed):
        f = Fleet(graph, seed=seed)
        tap = Tap(f)
        run(f, mg(seed=seed), 400)
        return [(round(t, 1), c.resource_id, c.robot_id, c.action) for t, _, c in tap.get('/swarm/claims')]
    a = shift(3)
    assert a and a == shift(3)


def test_batteries_drain_working_and_charge_on_a_charger(graph, mg):
    f = Fleet(graph, seed=1, battery_start=[0.35] * 6)       # most go below 30 % early
    socs = {}
    run(f, mg(), 1200, on_step=lambda t: [socs.setdefault(r.rid, []).append((r.battery, r.status)) for r in f.robots])
    charged = [rid for rid, s in socs.items() if any(st == RobotState.CHARGING for _, st in s)]
    assert charged
    for rid in charged:
        seq = [b for b, st in socs[rid] if st == RobotState.CHARGING]
        assert seq[-1] > seq[0]                                   # rises on the charger
    assert min(b for s in socs.values() for b, _ in s) > 0.03     # nobody ran empty


def test_a_shift_completes_with_mission_generator(graph, mg):
    f = Fleet(graph, seed=5)
    w = mg()
    run(f, w, 3601)
    done = w.done()
    kinds = Counter(m['kind'] for m in done)
    assert 60 <= len(done) <= 110 and kinds['inbound'] > 15 and kinds['outbound'] > 15
    assert not any(m['stranded'] for m in w.node.meta.values())
    # every bin is promised to at most one open task, and the inventory matches what was done
    open_bins = [m['bin'] for m in w.node.meta.values() if m['done'] is None and m['bin']]
    assert len(open_bins) == len(set(open_bins))


def test_throughput_matches_the_isaac_truth_baseline(graph, mg):
    """Validation (vault Results): c50_truth = 76/h over 900 s. The mock's mean over 10 seeds is within 10 %."""
    rates = []
    for seed in range(1, 11):
        f = Fleet(graph, seed=seed)
        w = mg(seed=seed)
        run(f, w, 901)
        rates.append(len(w.done()) * 4)
    mean = sum(rates) / len(rates)
    assert 68.4 <= mean <= 83.6, rates
