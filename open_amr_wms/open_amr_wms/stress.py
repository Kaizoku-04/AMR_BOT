"""Stress and throughput runs: open_amr_wms with orders against the mock fleet, in-process, for fleet sizes and
order mixes. Prints one CSV row per run (and writes it with --out).

    python3 -m open_amr_wms.stress --robots 2,4,6,8,10 --mix balanced,outbound,inbound --hours 8 --seeds 1
Mixes: balanced = orders and ASNs for about the same number of boxes; outbound = orders only (receiving still runs on
the generated pallets, its boxes unannounced); inbound = ASNs only. Orders: 1-3 lines of 1-3 boxes, cut-offs spread
over the shift, 70 % normal / 20 % expedite / 10 % urgent. Demand is sized to exceed what the fleet can do, so the
result is the fleet's capacity and how the WMS rations it (late orders), not the demand.
"""
import argparse
import random
import time

from open_amr_swarm.lane_graph import LaneGraph

from .harness import WmsOnBus

FIELDS = ['robots', 'mix', 'seed', 'hours', 'orders', 'asn_boxes', 'boxes_h', 'in_h', 'out_h', 'per_robot_h',
          'latency_mean_s', 'latency_p95_s', 'orders_closed', 'fill_rate', 'late_orders', 'yielding', 'charging',
          'wall_s', 'sim_per_wall']


def demand(w, mix, hours, robots, rng):
    skus = sorted({b['sku'] for b in w.node.bins.values() if b['full']})
    boxes = int(hours * robots * 20)                    # > a robot's ~13 boxes/h: demand exceeds capacity
    n_orders = n_asn = 0
    if mix in ('balanced', 'outbound'):
        target = boxes if mix == 'outbound' else boxes // 2
        k = 0
        while k < target:
            lines = [(rng.choice(skus), rng.randint(1, 3)) for _ in range(rng.randint(1, 3))]
            n_orders += 1
            k += sum(q for _, q in lines)
            w.orders.add_order(f'SO-{n_orders:05d}', f'c{rng.randint(1, 50)}', lines,
                               cutoff_s=900 + hours * 3600 * k / max(target, 1),
                               priority=rng.choices([0, 1, 2], [70, 20, 10])[0])
    if mix in ('balanced', 'inbound'):
        target = boxes if mix == 'inbound' else boxes // 2
        for i in range(0, target, 24):
            w.orders.add_asn(f'ASN-{i // 24:04d}', f's{rng.randint(1, 5)}',
                             [(rng.choice(skus), f'A{i + j:06d}') for j in range(24)])
            n_asn += 24
    return n_orders, n_asn


def one(graph_path, robots, mix, seed, hours):
    from open_amr_fleet_mock.fleet import Fleet
    from open_amr_fleet_mock.harness import run
    g = LaneGraph(graph_path)
    f = Fleet(g, robots=robots, seed=seed, chargers=None if robots > 2 else ('charge_0',),
              receiving_pallets=(24, 24), outbound_pallets=(0, 0))
    w = WmsOnBus(graph_path.replace('.geojson', '.yaml'), g, orders=True, seed=seed, open_tasks=robots + 2)
    n_orders, n_asn = demand(w, mix, hours, robots, random.Random(seed))
    t0 = time.monotonic()
    run(f, w, 1 + hours * 3600)
    wall = time.monotonic() - t0
    x = w.node.metrics()
    o = w.orders.summary(w.node.now())
    h = hours
    per = {}
    for m in w.done():
        per[m['robot']] = per.get(m['robot'], 0) + 1
    return dict(robots=robots, mix=mix, seed=seed, hours=hours, orders=n_orders, asn_boxes=n_asn,
                boxes_h=round((x['by_kind']['inbound'] + x['by_kind']['outbound']) / h, 1),
                in_h=round(x['by_kind']['inbound'] / h, 1), out_h=round(x['by_kind']['outbound'] / h, 1),
                per_robot_h=round((x['by_kind']['inbound'] + x['by_kind']['outbound']) / h / robots, 1),
                latency_mean_s=round(x['latency_mean']), latency_p95_s=round(x['latency_p95']),
                orders_closed=o['orders_closed'], fill_rate=round(o['fill_rate'], 3), late_orders=o['late_orders'],
                yielding=round(x['shares']['yielding'], 3), charging=round(x['shares']['charging'], 3),
                wall_s=round(wall, 1), sim_per_wall=round(hours * 3600 / wall))


def main():
    import os
    ap = argparse.ArgumentParser(description=__doc__)
    root = os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR'))
    ap.add_argument('--graph', default=os.path.join(root, 'sim', 'worlds', 'warehouse_mission_graph.geojson'))
    ap.add_argument('--robots', default='2,4,6,8,10')
    ap.add_argument('--mix', default='balanced')
    ap.add_argument('--seeds', default='1')
    ap.add_argument('--hours', type=float, default=8.0)
    ap.add_argument('--out', default='')
    a = ap.parse_args()
    rows = []
    print(','.join(FIELDS), flush=True)
    for mix in a.mix.split(','):
        for n in [int(v) for v in a.robots.split(',')]:
            for seed in [int(v) for v in a.seeds.split(',')]:
                r = one(a.graph, n, mix, seed, a.hours)
                rows.append(r)
                print(','.join(str(r[k]) for k in FIELDS), flush=True)
    if a.out:
        with open(a.out, 'w') as f:
            f.write(','.join(FIELDS) + '\n')
            for r in rows:
                f.write(','.join(str(r[k]) for k in FIELDS) + '\n')


if __name__ == '__main__':
    main()
