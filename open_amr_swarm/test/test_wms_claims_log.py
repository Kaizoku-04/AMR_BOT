"""tasks.csv keeps its columns and gets first_claimed_s and n_claims at the end; claims.csv has one row per claim /
give-back / done event (2026-11-19: a task won, given back and re-won looked like one long offered -> claimed wait)."""
import csv
import json

import pytest
import rclpy
import yaml
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import Claim, StationState
from open_amr_swarm.mission_generator import MissionGenerator


@pytest.fixture
def gen(tmp_path):
    names = [('charge_0', 'charger'), ('receiving_bay', 'bay'), ('outbound_bay', 'bay')]
    names += [(f'aisle_0_bay_{i}', 'aisle_bay') for i in range(8)]
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': n, 'kind': k}},
              'geometry': {'type': 'Point', 'coordinates': [float(i), 0.0]}} for i, (n, k) in enumerate(names)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': (i + 1) % len(names),
                                                 'metadata': {}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(len(names))]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    n = tmp_path / 'g.yaml'
    n.write_text(yaml.safe_dump({nm: {'id': i, 'kind': k} for i, (nm, k) in enumerate(names)}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'graph_nodes:=' + str(n), '-p', f'out_dir:={tmp_path}',
                     '-p', 'open_tasks:=2'])
    m = MissionGenerator()
    m.t0 = m.now()
    m.on_station(StationState(station_id='arm_outbound', bay_node='outbound_bay', state=StationState.IDLE,
                             boxes_available=32))
    m.tick()
    yield m, tmp_path
    m.destroy_node()
    rclpy.shutdown()


def claim(m, tid, robot, action, by=''):
    m.on_claim(Claim(resource_type=Claim.TASK, resource_id=tid, robot_id=robot, action=action, by=by,
                     stamp=TimeMsg(sec=1)))


def rows(path):
    return list(csv.DictReader(open(path)))


def test_won_given_back_and_rewon_keeps_both_claims(gen):
    m, out = gen
    tid = next(iter(m.meta))
    claim(m, tid, 'amr_1', Claim.CLAIM)
    claim(m, tid, 'amr_1', Claim.RELEASE)
    claim(m, tid, 'amr_2', Claim.CLAIM)
    claim(m, tid, 'amr_2', Claim.DONE)
    t = rows(out / 'tasks.csv')[0]
    assert list(t)[-2:] == ['first_claimed_s', 'n_claims'] and list(t)[:11][-1] == 'releases'
    assert t['n_claims'] == '2' and t['releases'] == '1' and t['robot'] == 'amr_2'
    assert float(t['first_claimed_s']) <= float(t['claimed_s']) <= float(t['done_s'])
    ev = [(r['event'], r['robot'], r['reason']) for r in rows(out / 'claims.csv')]
    assert ev == [('claimed', 'amr_1', 'claim 1'), ('released', 'amr_1', 'given back by the robot'),
                  ('claimed', 'amr_2', 'claim 2'), ('done', 'amr_2', '')]


def test_release_announced_by_a_peer_says_so(gen):
    m, out = gen
    tid = next(iter(m.meta))
    claim(m, tid, 'amr_1', Claim.CLAIM)
    claim(m, tid, 'amr_1', Claim.RELEASE, by='amr_3')
    assert rows(out / 'claims.csv')[-1]['reason'] == 'announced by amr_3: silent robot'
    assert rows(out / 'claims.csv')[-1]['by'] == 'amr_3'
