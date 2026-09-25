"""Unit tests for the battery model and charging policy (open_amr_swarm/energy.py)."""
import json
import math

import pytest

from open_amr_swarm.energy import (BatteryParams, ChargeParams, ChargerPeer, DrainEstimator, bid_penalty_s, evictions,
                                   waiting_for_charger,
                                   can_take, pick_charger, step_soc, task_need)
from open_amr_swarm.lane_graph import LaneGraph


@pytest.fixture
def bank(tmp_path):
    """One-way street s0 -> s1 -> s2 (2 m apart along x), a nose-in charger c_i 1.7 m south of each s_i."""
    pts = {i: (2.0 * i, 0.0, f's{i}', 'street') for i in range(3)}
    pts.update({10 + i: (2.0 * i, -1.7, f'c{i}', 'charger') for i in range(3)})
    edges = [(0, 1), (1, 2)] + [(i, 10 + i) for i in range(3)] + [(10 + i, i) for i in range(3)]
    feats = [{'type': 'Feature', 'properties': {'id': n, 'metadata': {'name': nm, 'kind': k}},
              'geometry': {'type': 'Point', 'coordinates': [x, y]}} for n, (x, y, nm, k) in pts.items()]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + j, 'startid': a, 'endid': b},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[list(pts[a][:2]), list(pts[b][:2])]]}}
              for j, (a, b) in enumerate(edges)]
    p = tmp_path / 'bank.geojson'
    p.write_text(json.dumps({'features': feats}))
    return LaneGraph(str(p))


CH = [10, 11, 12]


def run(soc, seconds, v=0.0, w=0.0, on_charger=False, p=BatteryParams()):
    for _ in range(int(seconds)):
        soc, _ = step_soc(soc, 1.0, v, w, on_charger, p)
    return soc


def test_spec_runtime_about_8_hours():
    # measured duty 2026-09-25: 82 % of the time driving at ~0.55 m/s, the rest standing
    p = BatteryParams()
    soc = 1.0
    hours = 0.0
    while soc > 0.0:
        soc = run(soc, 820, v=0.55, p=p)
        soc = run(soc, 180, p=p)
        hours += 1000 / 3600
    assert 7.0 < hours < 9.0


def test_charging_30_to_80_takes_about_40_min_and_tapers():
    p = BatteryParams()
    t, soc = 0, 0.30
    while soc < 0.80:
        soc = run(soc, 60, on_charger=True, p=p); t += 60
    assert 35 * 60 <= t <= 45 * 60
    fast = run(0.5, 600, on_charger=True, p=p) - 0.5
    slow = run(0.9, 600, on_charger=True, p=p) - 0.9
    assert slow < fast / 2                       # CV phase above 80 %


def test_time_scale_compresses_everything():
    assert run(1.0, 60, v=1.0, p=BatteryParams(time_scale=20.0)) == pytest.approx(run(1.0, 1200, v=1.0), abs=1e-6)


def test_discharging_current_is_negative():
    _, amps = step_soc(0.5, 1.0, 1.0, 0.0, False, BatteryParams())
    assert amps < 0


def test_travel_is_directed(bank):
    assert bank.travel(0, 12) == pytest.approx(4.0 + 1.7)
    assert math.isinf(bank.travel(2, 0))


def test_pick_nearest_free_charger(bank):
    assert pick_charger(bank, 0, CH, 'amr_0', 0.5, []) == 10
    standing = ChargerPeer('amr_1', 10, [10], 0.9)
    assert pick_charger(bank, 0, CH, 'amr_0', 0.5, [standing]) == 11
    heading = ChargerPeer('amr_2', 11, [1], 0.9)
    assert pick_charger(bank, 0, CH, 'amr_0', 0.5, [standing, heading]) == 12
    full = [ChargerPeer(f'amr_{i}', c, [c], 0.9) for i, c in enumerate(CH, 1)]
    assert pick_charger(bank, 0, CH, 'amr_0', 0.5, full) is None


def test_race_same_charger_both_sides_agree(bank):
    # amr_0 and amr_3 head for c0 in the same instant: the one below `low` keeps it, whatever its id
    a = ChargerPeer('amr_0', 10, [0], 0.70)
    b = ChargerPeer('amr_3', 10, [0], 0.25)
    assert pick_charger(bank, 0, CH, 'amr_3', 0.25, [a], current=10) == 10
    assert pick_charger(bank, 0, CH, 'amr_0', 0.70, [b], current=10) == 11
    # neither low: lower id keeps it
    b.battery = 0.6
    assert pick_charger(bank, 0, CH, 'amr_0', 0.70, [b], current=10) == 10
    assert pick_charger(bank, 0, CH, 'amr_3', 0.60, [a], current=10) == 11


def test_keeps_current_charger_even_if_another_is_nearer(bank):
    assert pick_charger(bank, 0, CH, 'amr_0', 0.5, [], current=12) == 12


def test_energy_aware_bidding(bank):
    cp = ChargeParams()
    rate = 0.15 * 20 / 3600                      # 20x demo: 3 charges per hour of work
    need = task_need(bank, 0, [2], 12.0, rate, CH, cp)
    t = ((4.0 + 1.7) / cp.plan_speed + 12.0) * cp.safety
    assert need == pytest.approx(t * rate)
    assert can_take(cp.reserve + need + 0.01, need, cp) and not can_take(cp.reserve + need - 0.01, need, cp)
    assert math.isinf(task_need(bank, 2, [0], 0.0, rate, CH, cp))     # unreachable: never bid
    assert bid_penalty_s(1.0, cp) == 0.0 and bid_penalty_s(0.0, cp) == cp.bid_weight_s


def test_drain_estimator_learns_while_working():
    d = DrainEstimator(prior_per_s=1e-5, window_s=30.0)
    soc = 1.0
    for t in range(0, 301):
        d.update(float(t), soc, working=True); soc -= 1e-4
    assert d.rate == pytest.approx(1e-4, rel=0.05)
    r = d.rate
    d.update(400.0, 0.2, working=False); d.update(500.0, 0.9, working=True)   # charging gaps don't count
    assert d.rate == r


def test_parking_spot_is_picked_like_a_charger(bank):
    # the same claim rules serve parking spots: nearest free, standing robots and heading robots excluded
    taken = [ChargerPeer('amr_1', 10, [10], 0.9), ChargerPeer('amr_2', 11, [1], 0.9)]
    assert pick_charger(bank, 0, CH, 'amr_0', 0.9, taken) == 12


def test_evictions_fullest_idle_leaves_for_a_waiting_low_robot():
    cp = ChargeParams()
    on = lambda rid, c, soc, busy=False: ChargerPeer(rid, c, [c], soc, busy)
    robots = [on('amr_0', 10, 0.95), on('amr_1', 11, 0.70), on('amr_2', 12, 0.50),      # all chargers taken
              ChargerPeer('amr_3', 20, [20], 0.25)]                                      # low, parked, waiting
    assert evictions(CH, robots, cp) == {'amr_0'}
    robots.append(ChargerPeer('amr_4', 21, [21], 0.20))                                  # a second waiting robot
    assert evictions(CH, robots, cp) == {'amr_0', 'amr_1'}                               # amr_2 < 60 %: stays
    robots.append(ChargerPeer('amr_5', 22, [22], 0.10))
    assert evictions(CH, robots, cp) == {'amr_0', 'amr_1'}                               # nobody else eligible


def test_evictions_none_when_a_charger_is_free_or_being_vacated():
    cp = ChargeParams()
    robots = [ChargerPeer('amr_0', 10, [10], 0.95), ChargerPeer('amr_1', 11, [11], 0.9),
              ChargerPeer('amr_3', 20, [20], 0.25)]
    assert evictions(CH, robots, cp) == set()                                            # charger 12 free
    robots.append(ChargerPeer('amr_2', 30, [12], 0.9))                                   # pulling out of 12
    assert evictions(CH, robots, cp) == set()
    robots.append(ChargerPeer('amr_4', 12, [2], 0.9))                                    # someone heads for 12
    assert evictions(CH, robots, cp) == {'amr_0'}


def test_evictions_tie_goes_to_higher_id_and_busy_robots_are_exempt():
    cp = ChargeParams()
    robots = [ChargerPeer('amr_0', 10, [10], 0.91), ChargerPeer('amr_1', 11, [11], 0.93),
              ChargerPeer('amr_2', 12, [12], 0.99, busy=True), ChargerPeer('amr_3', 20, [20], 0.2)]
    assert evictions(CH, robots, cp) == {'amr_1'}                                        # same 5 % bucket


def test_waiting_for_charger_only_counts_idle_low_robots_without_one():
    cp = ChargeParams()
    robots = [ChargerPeer('amr_0', 20, [20], 0.25),                 # low, parked: waiting
              ChargerPeer('amr_1', 10, [1], 0.20),                  # low, heading for a charger: not waiting
              ChargerPeer('amr_2', 21, [21], 0.25, busy=True),      # low but finishing a task
              ChargerPeer('amr_3', 22, [22], 0.52)]                 # not low
    assert [r.robot_id for r in waiting_for_charger(CH, robots, cp)] == ['amr_0']
