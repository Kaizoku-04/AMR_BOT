"""open_amr_wms on the mock fleet's in-process bus (open_amr_fleet_mock.harness.run), for tests and stress runs."""
import yaml

from .core import Wms
from .orders import Orders
from .store import Store


class WmsOnBus:
    name = 'wms'

    def __init__(self, nodes_yaml, graph=None, store=None, orders=False, escalate_s=1800.0, **params):
        with open(nodes_yaml) as f:
            nodes = yaml.safe_load(f)
        self.t, self.bus, self.logged, self.queue = 0.0, None, [], []
        store = store or Store()
        self.orders = Orders(store, escalate_s=escalate_s) if orders else None
        self.node = Wms(nodes, graph, publish=self._pub, now=lambda: self.t,
                        log=lambda lvl, text: self.logged.append((lvl, text)), store=store, source=self.orders,
                        **params)
        self.next_tick = 0.0

    def _pub(self, topic, msg):
        if self.bus is not None:
            self.bus.publish(topic, self.name, msg)
        else:
            self.queue.append((topic, msg))

    def attach(self, bus):
        self.bus = bus
        n = self.node
        for topic, cb in (('/swarm/stations', n.on_station), ('/swarm/transfers', n.on_transfer),
                          ('/swarm/claims', n.on_claim), ('/swarm/state', n.on_state),
                          ('/swarm/operator', n.on_operator)):
            bus.subscribe(topic, self.name, cb)
        for topic, msg in self.queue:
            bus.publish(topic, self.name, msg)
        self.queue.clear()

    def step(self, now):
        self.t = now
        if now >= self.next_tick - 1e-6:
            self.next_tick = now + 0.5
            self.node.tick()

    def done(self):
        return self.node.done_tasks()
