"""An arm that couldn't take work when tasks were created gets its share back as soon as it can (chain 45 c45_truth,
2026-10-24: the first 8 tasks were created during the outbound arm's 2 s start-up fault; each fell back to inbound,
and no outbound work was offered for ~470 s, until those inbound tasks finished)."""
import json

import pytest
import rclpy
import yaml

from open_amr_msgs.msg import StationState
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
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', f'graph_nodes:={n}', '-p', f'out_dir:={tmp_path}',
                     '-p', 'open_tasks:=4'])
    m = MissionGenerator()
    yield m
    m.destroy_node()
    rclpy.shutdown()


def _outbound(m, state):
    m.on_station(StationState(station_id='arm_outbound', bay_node='outbound_bay', state=state, boxes_available=32))


def _open(m, kind):
    return [k for k, v in m.meta.items() if v['kind'] == kind and v['done'] is None]


def test_an_arm_back_from_a_fault_gets_its_share_at_once(gen):
    m = gen
    _outbound(m, StationState.FAULT)                    # the outbound arm's start-up fault
    m.tick()
    assert len(_open(m, 'inbound')) == 4 and _open(m, 'outbound') == []
    _outbound(m, StationState.IDLE)                     # 2 s later it is in service
    m.tick()
    assert len(_open(m, 'outbound')) == 2              # its share now, not after the inbound tasks finish
    m.tick()
    assert len(_open(m, 'outbound')) == 2 and len(_open(m, 'inbound')) == 4    # topped up once, not every tick


def test_with_both_arms_up_the_mix_is_unchanged(gen):
    m = gen
    _outbound(m, StationState.IDLE)
    m.tick()
    assert len(_open(m, 'inbound')) == 2 and len(_open(m, 'outbound')) == 2
