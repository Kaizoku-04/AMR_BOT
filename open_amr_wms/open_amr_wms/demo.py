"""Dashboard demo without ROS processes: the mock fleet and the WMS in-process (open_amr_fleet_mock.harness), sample
orders and ASNs, a failure scenario, and the dashboard on http://localhost:<port>.

    python3 -m open_amr_wms.demo --rtf 20 --port 8080 [--scenario <file.yaml>] [--until 7200]
"""
import argparse
import os
import threading
import time

from open_amr_swarm.lane_graph import LaneGraph

from .api import Api, serve
from .harness import WmsOnBus

HERE = os.path.dirname(os.path.abspath(__file__))


def sample_orders(w, skus, n=40, horizon_s=7200.0, seed=3):
    import random
    rng = random.Random(seed)
    for i in range(n):
        lines = [(rng.choice(skus), rng.randint(1, 3)) for _ in range(rng.randint(1, 2))]
        w.orders.add_order(f'SO-{1000 + i}', f'customer-{rng.randint(1, 8)}', lines,
                           cutoff_s=600 + horizon_s * (i + 1) / n, priority=rng.choice([0, 0, 0, 1, 2]))
    for i in range(6):
        w.orders.add_asn(f'ASN-{500 + i}', f'supplier-{i % 3}',
                         [(rng.choice(skus), f'IN{i:02d}{k:03d}') for k in range(8)])


def main():
    from open_amr_fleet_mock.fleet import Fleet, load_scenario
    ap = argparse.ArgumentParser(description=__doc__)
    root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
    ap.add_argument('--graph', default=os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.geojson'))
    ap.add_argument('--rtf', type=float, default=20.0)
    ap.add_argument('--port', type=int, default=8080)
    ap.add_argument('--host', default='127.0.0.1', help='0.0.0.0 = the LAN (writes then need WMS_API_KEY)')
    ap.add_argument('--scenario', default='')
    ap.add_argument('--until', type=float, default=8 * 3600)
    ap.add_argument('--robots', type=int, default=6)
    a = ap.parse_args()
    g = LaneGraph(a.graph)
    events, kw = load_scenario(a.scenario) if a.scenario else ([], {})
    kw.setdefault('robots', a.robots)
    fleet = Fleet(g, scenario=events, **kw)
    w = WmsOnBus(a.graph.replace('.geojson', '.yaml'), g, orders=True, open_tasks=kw['robots'] + 2)
    skus = sorted({b['sku'] for b in w.node.bins.values() if b['full']})
    sample_orders(w, skus)
    w.attach(fleet.bus)
    lock = threading.RLock()
    srv = serve(Api(w.node, w.orders, lock), a.host, a.port)
    print(f'dashboard: http://localhost:{srv.server_address[1]}/', flush=True)
    now, dt, w0 = 1.0, 0.1, time.monotonic()
    while now < a.until:
        target = 1.0 + (time.monotonic() - w0) * a.rtf
        with lock:
            while now < target:
                fleet.step(now, dt)
                w.step(now)
                fleet.bus.flush()
                now = round(now + dt, 6)
        time.sleep(0.02)


if __name__ == '__main__':
    main()
