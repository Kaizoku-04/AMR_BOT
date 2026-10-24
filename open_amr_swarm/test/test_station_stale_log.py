"""The WMS says when a station's heartbeat stops reaching it, and when it is back: while it is stale, arm_room() is 0 and
no work for that arm is offered, and nothing used to say so."""
import json

import pytest
import rclpy
import yaml

from open_amr_msgs.msg import StationState
from open_amr_swarm import mission_generator
from open_amr_swarm.mission_generator import MissionGenerator


@pytest.fixture
def gen(tmp_path, monkeypatch):
    names = [('charge_0', 'charger'), ('aisle_0_bay_0', 'aisle_bay'), ('outbound_bay', 'bay')]
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': n, 'kind': k}},
              'geometry': {'type': 'Point', 'coordinates': [float(i), 0.0]}} for i, (n, k) in enumerate(names)]
    feats += [{'type': 'Feature', 'properties': {'id': 10 + i, 'startid': i, 'endid': (i + 1) % 3, 'metadata': {}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(3)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    n = tmp_path / 'g.yaml'
    n.write_text(yaml.safe_dump({nm: {'id': i, 'kind': k} for i, (nm, k) in enumerate(names)}))
    clock = [1000.0]
    monkeypatch.setattr(mission_generator.wallclock, 'monotonic', lambda: clock[0])
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', f'graph_nodes:={n}', '-p', f'out_dir:={tmp_path}'])
    m = MissionGenerator()
    m.logged = []
    log = m.get_logger()
    monkeypatch.setattr(type(log), 'warn', lambda self, msg, **k: m.logged.append(('warn', msg)))
    monkeypatch.setattr(type(log), 'info', lambda self, msg, **k: m.logged.append(('info', msg)))
    m.clock = clock
    yield m
    m.destroy_node()
    rclpy.shutdown()


def _heartbeat(m):
    m.on_station(StationState(station_id='arm_outbound', bay_node='outbound_bay', state=StationState.IDLE,
                              boxes_available=32))


def test_a_stale_station_is_reported_once_and_its_return_too(gen):
    m = gen
    _heartbeat(m)
    m.watch_stations()
    assert m.logged == [] and m.arm_room('outbound_bay', 'outbound') == 32
    m.clock[0] += 5.0                                                    # > station_timeout_s 3.0
    m.watch_stations(); m.watch_stations()
    assert [lvl for lvl, _ in m.logged] == ['warn'] and 'outbound_bay' in m.logged[0][1]
    assert m.arm_room('outbound_bay', 'outbound') == 0                  # what stopped the outbound work
    _heartbeat(m)
    m.watch_stations()
    assert [lvl for lvl, _ in m.logged] == ['warn', 'info']
