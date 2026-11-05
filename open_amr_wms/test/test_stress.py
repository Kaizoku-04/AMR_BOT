"""Stress runner smoke test and the index that keeps the WMS's tick independent of the shift's history (stress run
2026-11-05: scanning every task ever created on each tick was 84 % of the WMS's time in a 4 h shift)."""
from open_amr_fleet_mock.fleet import Fleet
from open_amr_fleet_mock.harness import run
from open_amr_wms import stress
from open_amr_wms.store import Store

from conftest import GEOJSON


def test_open_index_is_exactly_the_unfinished_tasks(graph, wms, tmp_path):
    f = Fleet(graph, seed=3, scenario=[{'at': 300, 'do': 'silent', 'target': 'amr_1', 'for_s': 200}])
    w = wms(orders=True, store=Store(str(tmp_path / 'w.db')))
    stock = {}
    for b in w.node.bins.values():
        if b['full']:
            stock[b['sku']] = stock.get(b['sku'], 0) + 1
    w.orders.add_order('S1', 'c', [(max(stock, key=stock.get), 4)], cutoff_s=400)    # cut-off withdraws some
    run(f, w, 1500)
    assert w.node.open == {t for t, m in w.node.meta.items() if m['done'] is None}
    assert w.orders.live == {k for k, o in w.orders.orders.items() if o['state'] not in
                             ('SHIPPED', 'SHORT_SHIPPED', 'CANCELLED')}


def test_stress_runner_one_hour_four_robots():
    r = stress.one(GEOJSON, 4, 'balanced', 1, 1.0)
    assert r['orders'] > 0 and r['asn_boxes'] > 0 and 20 < r['boxes_h'] < 80 and r['sim_per_wall'] > 50
