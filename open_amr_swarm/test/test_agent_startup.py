"""Smoke test: the agent and the mission generator construct (parameters, publishers, graph loading) without a sim."""
import json
import os

import pytest
import rclpy
import yaml

from open_amr_swarm.agent import SwarmAgent
from open_amr_swarm.mission_generator import MissionGenerator


@pytest.fixture
def graph_files(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': n, 'kind': k}},
              'geometry': {'type': 'Point', 'coordinates': [float(i), 0.0]}}
             for i, (n, k) in enumerate([('charge_0', 'charger'), ('aisle_0_bay_0', 'aisle_bay'), ('receiving_bay', 'bay')])]
    feats += [{'type': 'Feature', 'properties': {'id': 10 + i, 'startid': i, 'endid': (i + 1) % 3, 'metadata': {'kind': 'street'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(3)]
    g = tmp_path / 'g.geojson'; g.write_text(json.dumps({'features': feats}))
    n = tmp_path / 'g.yaml'
    n.write_text(yaml.safe_dump({f['properties']['metadata']['name']: {'id': f['properties']['id'], 'kind': f['properties']['metadata']['kind']}
                                 for f in feats if f['geometry']['type'] == 'Point'}))
    return str(g), str(n)


def test_nodes_construct(graph_files, tmp_path):
    g, n = graph_files
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'home_node:=charge_0',
                     '-p', f'graph_nodes:={n}', '-p', f'out_dir:={tmp_path}'])
    try:
        a, m = SwarmAgent(), MissionGenerator()
        assert a.graph.edge_kind[(0, 1)] == 'street' and a.zones['charger'] == 35.0
        assert len(m.bays) == 1
        a.destroy_node(); m.destroy_node()
    finally:
        rclpy.shutdown()
