"""Shared fixtures: the real lane graph (a snapshot of OpenAMR sim/worlds/warehouse_mission_graph.*, test/data/README)
and mission_generator on the in-process bus."""
import os

import pytest
import rclpy

from open_amr_swarm.lane_graph import LaneGraph

DATA = os.path.join(os.path.dirname(__file__), 'data')
GEOJSON = os.path.join(DATA, 'warehouse_mission_graph.geojson')
NODES = os.path.join(DATA, 'warehouse_mission_graph.yaml')


@pytest.fixture(scope='session')
def graph():
    return LaneGraph(GEOJSON)


class Tap:
    """Everything published on the bus, by topic: [(t, sender, msg)]."""

    def __init__(self, fleet):
        self.fleet, self.by = fleet, {}
        fleet.bus.taps.append(self)

    def __call__(self, topic, sender, msg):
        self.by.setdefault(topic, []).append((self.fleet.now, sender, msg))

    def get(self, topic):
        return self.by.get(topic, [])


@pytest.fixture
def mg(tmp_path):
    """make(**params) -> MissionGeneratorOnBus around a fresh, unmodified mission_generator node."""
    from open_amr_swarm.mission_generator import MissionGenerator
    from open_amr_fleet_mock.harness import MissionGeneratorOnBus
    made = []

    def close():
        for w in made:
            w.node.csv.close(); w.node.soc_csv.close()
            w.node.destroy_node()
        made.clear()
        if rclpy.ok():
            rclpy.shutdown()

    def make(**params):
        close()                     # one node at a time: its parameters come from rclpy.init's arguments
        p = dict(graph=GEOJSON, graph_nodes=NODES, out_dir=str(tmp_path), open_tasks=8, report_s=100000.0)
        p.update(params)
        args = ['--ros-args', '--log-level', 'warn']
        for k, v in p.items():
            args += ['-p', f'{k}:={v}']
        rclpy.init(args=args)
        w = MissionGeneratorOnBus(MissionGenerator())
        logged = []
        log = w.node.get_logger()
        for lvl in ('info', 'warn', 'error'):
            orig = getattr(log, lvl)
            setattr(log, lvl, lambda msg, _l=lvl, _o=orig, **k: logged.append((_l, msg)))
        w.logged = logged
        made.append(w)
        return w

    yield make
    close()
