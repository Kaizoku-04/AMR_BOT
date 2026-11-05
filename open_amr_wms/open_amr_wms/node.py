"""wms: the warehouse management node (open_amr_wms.core.Wms on the real topics). Replaces mission_generator.

    ros2 run open_amr_wms wms --ros-args -p graph:=<lane graph .geojson> -p db:=~/.openamr/wms.db -p use_sim_time:=true
Same topics and QoS as mission_generator (notes/WMS Interface.md §3; profiles imported from open_amr_swarm.agent).
State persists in `db` (SQLite); after a restart it resumes from it and from the heartbeats (core.py module doc).
Timeouts run on the node clock (sim time under use_sim_time).
"""
import os
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from open_amr_msgs.msg import Claim, Inventory, OperatorCommand, RobotState, StationState, Task, Transfer
from open_amr_swarm.agent import EVENT_QOS, OPERATOR_QOS, STATE_QOS
from open_amr_swarm.lane_graph import LaneGraph

from . import importer
from .api import Api, serve
from .core import Wms
from .orders import Orders
from .store import Store

INV_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class WmsNode(Node):
    def __init__(self):
        super().__init__('wms')
        dp = self.declare_parameter
        root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
        gpath = dp('graph', os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.geojson')).value
        npath = dp('graph_nodes', gpath.replace('.geojson', '.yaml')).value
        db = os.path.expanduser(dp('db', '~/.openamr/wms.db').value)
        if db != ':memory:':
            os.makedirs(os.path.dirname(db), exist_ok=True)
        with open(npath) as f:
            nodes = yaml.safe_load(f)
        self.pubs = {'/swarm/tasks': self.create_publisher(Task, '/swarm/tasks', EVENT_QOS),
                     '/swarm/claims': self.create_publisher(Claim, '/swarm/claims', EVENT_QOS),
                     '/wms/inventory': self.create_publisher(Inventory, '/wms/inventory', INV_QOS)}
        log = self.get_logger()
        say = {'info': log.info, 'warn': log.warn, 'error': log.error}
        store = Store(db)
        self.orders = Orders(store, escalate_s=dp('escalate_s', 1800.0).value,
                             require_asn=dp('require_asn', False).value) if dp('orders', False).value else None
        self.wms = Wms(nodes, LaneGraph(gpath), publish=lambda t, m: self.pubs[t].publish(m), now=self.now,
                       log=lambda lvl, text: say[lvl](text), store=store, source=self.orders, mode=dp('mode', 'mission').value,
                       open_tasks=dp('open_tasks', 8).value, reoffer_s=dp('reoffer_s', 4.0).value,
                       peer_timeout_s=dp('peer_timeout_s', 3.0).value,
                       station_timeout_s=dp('station_timeout_s', 3.0).value, reconcile_s=dp('reconcile_s', 3.0).value,
                       station_wait_s=dp('station_wait_s', 5.0).value, seed=dp('seed', 7).value,
                       initial_fill=dp('initial_fill', 0.5).value)
        w = self.wms
        self.lock = threading.RLock()            # the REST API's threads and the executor share the core
        if self.orders is not None:
            path = dp('import_file', '').value
            if path:
                out = importer.apply(self.orders, importer.read(path), w.now(), w)
                log.info(f'imported {path}: {len(out["created"])} new, {len(out["unchanged"])} unchanged, '
                         f'errors {out["errors"]}')
        port = dp('api_port', 8080).value
        self.api = Api(w, self.orders or Orders(w.store), self.lock)
        if port:
            self.http = serve(self.api, dp('api_host', '0.0.0.0').value, port)
            log.info(f'wms: REST API and dashboard on port {port}')
        locked = lambda fn: (lambda *a: self._locked(fn, *a))
        self.create_subscription(StationState, '/swarm/stations', locked(w.on_station), STATE_QOS)
        self.create_subscription(Transfer, '/swarm/transfers', locked(w.on_transfer), EVENT_QOS)
        self.create_subscription(Claim, '/swarm/claims', locked(w.on_claim), EVENT_QOS)
        self.create_subscription(RobotState, '/swarm/state', locked(w.on_state), STATE_QOS)
        self.create_subscription(OperatorCommand, '/swarm/operator', locked(w.on_operator), OPERATOR_QOS)
        self.create_timer(0.5, locked(w.tick))
        self.create_timer(dp('report_s', 30.0).value, lambda: log.info(w.report_line()))
        w.publish_inventory()
        log.info(f'wms: db {db}, {len(w.bins)} bins ({sum(b["full"] for b in w.bins.values())} full), '
                 f'{len(w.open_ids())} open tasks{" (restarted)" if w.restarted else ""}')

    def _locked(self, fn, *a):
        with self.lock:
            return fn(*a)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = WmsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.get_logger().info(node.wms.report_line(final=True))
