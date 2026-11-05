"""Orders in (milestone 3): customers, orders and ASNs -> tasks; priorities, cut-offs, partial shipments; CSV / JSON
import; the REST API as an ERP would use it."""
import json
import urllib.error
import urllib.request

import pytest

from open_amr_msgs.msg import Task

from open_amr_fleet_mock.fleet import Fleet
from open_amr_fleet_mock.harness import run
from open_amr_wms import importer
from open_amr_wms.api import Api, serve
from open_amr_wms.store import Store

from conftest import Tap


def skus(w):
    """SKU -> number of full bins holding it (the generated initial stock)."""
    out = {}
    for b in w.node.bins.values():
        if b['full']:
            out[b['sku']] = out.get(b['sku'], 0) + 1
    return out


def test_an_order_becomes_one_task_per_box_and_ships(graph, wms):
    f = Fleet(graph, seed=1)
    tap, w = Tap(f), wms(orders=True)
    stock = skus(w)
    sku = max(stock, key=stock.get)
    w.orders.add_order('SO-1', 'acme', [(sku, 3)], cutoff_s=3600)
    run(f, w, 1500)
    o = w.orders.orders['SO-1']
    tasks = [m for _, s, m in tap.get('/swarm/tasks') if s == 'wms' and m.mission_id == 'SO-1' and m.round == 0]
    assert len(tasks) == 3 and all(t.type == Task.FETCH_FROM_BIN for t in tasks)
    assert all(w.node.meta[t.task_id]['sku'] == sku for t in tasks)
    assert o['state'] == 'SHIPPED' and o['lines'][0]['picked'] == 3
    assert all(abs(t.deadline.sec - 3600) < 1 for t in tasks)           # cut-off carried for observers


def test_no_stock_no_task_and_inbound_still_runs(graph, wms):
    f = Fleet(graph, seed=1)
    w = wms(orders=True)
    w.orders.add_order('SO-X', 'acme', [('NOT-STOCKED', 2)], cutoff_s=600)
    run(f, w, 700)
    assert not [m for m in w.node.meta.values() if m['kind'] == 'outbound']
    assert [m for m in w.done() if m['kind'] == 'inbound']
    assert w.orders.orders['SO-X']['state'] == 'SHORT_SHIPPED' and w.orders.orders['SO-X']['short'] == 2


def test_urgent_orders_first_then_earlier_cutoff(graph, wms):
    w = wms(orders=True)
    stock = skus(w)
    a, b = sorted(stock, key=stock.get)[-2:]
    w.orders.add_order('late-normal', 'c1', [(a, 1)], cutoff_s=9000, priority=0)
    w.orders.add_order('early-normal', 'c1', [(a, 1)], cutoff_s=8000, priority=0)
    w.orders.add_order('urgent', 'c2', [(b, 1)], cutoff_s=9999, priority=2)
    assert [o['id'] for o in w.orders.queue(0.0)] == ['urgent', 'early-normal', 'late-normal']


def test_priority_rises_as_the_cutoff_nears(graph, wms):
    f = Fleet(graph, seed=1, scenario=[{'at': 0, 'do': 'silent', 'target': f'amr_{i}', 'for_s': 1e6} for i in range(6)])
    tap, w = Tap(f), wms(orders=True, escalate_s=60.0)
    stock = skus(w)
    w.orders.add_order('SO-2', 'acme', [(max(stock, key=stock.get), 1)], cutoff_s=100)
    run(f, w, 99)                                       # nobody takes it: it is re-offered every 4 s
    pri = [(t, m.priority) for t, s, m in tap.get('/swarm/tasks') if m.mission_id == 'SO-2']
    assert pri[0][1] == 1 and [p for t, p in pri if t > 45] and all(p == 2 for t, p in pri if t > 45)


def test_cutoff_short_ships_withdraws_and_counts_late_boxes(graph, wms):
    f = Fleet(graph, seed=1)
    tap, w = Tap(f), wms(orders=True)
    stock = skus(w)
    sku = max(stock, key=stock.get)
    w.orders.add_order('SO-3', 'acme', [(sku, min(stock[sku], 6))], cutoff_s=200)
    run(f, w, 900)
    o = w.orders.orders['SO-3']
    assert o['state'] == 'SHORT_SHIPPED' and 0 < o['short'] <= 6
    line = o['lines'][0]
    assert line['picked'] + line['late'] + o['short'] - line['late'] >= line['qty'] - line['assigned']
    cancels = [c for _, s, c in tap.get('/swarm/claims') if s == 'wms' and c.action == 3]
    assert cancels                                      # its unclaimed tasks were withdrawn at the cut-off
    assert line['assigned'] == 0


def test_asn_boxes_go_into_storage_with_their_ids(graph, wms):
    f = Fleet(graph, seed=1)
    w = wms(orders=True)
    w.orders.add_asn('ASN-1', 'supplier-1', [('SKU-Z', f'Z{i:03d}') for i in range(4)])
    run(f, w, 1200)
    stored = {b['box_id']: b['sku'] for b in w.node.bins.values() if b['full']}
    got = [f'Z{i:03d}' for i in range(4) if f'Z{i:03d}' in stored]
    assert got == ['Z000', 'Z001', 'Z002', 'Z003'] and all(stored[z] == 'SKU-Z' for z in got)
    assert w.orders.asns['ASN-1']['state'] == 'RECEIVED'
    # boxes beyond the ASN are booked as exceptions, not invented SKUs
    unannounced = [b for b in w.node.bins.values() if b['sku'] == 'UNANNOUNCED']
    assert all(b['box_id'].startswith('UNANNOUNCED-') for b in unannounced)


def test_resend_is_idempotent_changes_after_picking_are_refused(wms):
    w = wms(orders=True)
    o = w.orders
    _, new = o.add_order('SO-4', 'acme', [('SKU-A', 2)], 500)
    _, again = o.add_order('SO-4', 'acme', [('SKU-A', 2)], 500)
    assert new and not again
    o.orders['SO-4']['lines'][0]['assigned'] = 1
    with pytest.raises(ValueError, match='refused'):
        o.add_order('SO-4', 'acme', [('SKU-A', 5)], 500)
    for bad in ([('SKU-A', 0)], [], [('', 1)]):
        with pytest.raises(ValueError):
            o.add_order('SO-5', 'acme', bad, 500)
    with pytest.raises(ValueError, match='priority'):
        o.add_order('SO-6', 'acme', [('SKU-A', 1)], 500, priority=7)


def test_csv_and_json_import(tmp_path, wms):
    w = wms(orders=True)
    (tmp_path / 'o.csv').write_text('order_id,customer,sku,qty,cutoff_s,priority\nA1,acme,SKU-A,2,900,0\n'
                                    'A1,acme,SKU-B,1,900,0\nA2,beta,SKU-C,4,1200,1\n')
    (tmp_path / 'a.csv').write_text('asn_id,supplier,sku,box_id\nN1,s1,SKU-A,X1\nN1,s1,SKU-A,X2\n')
    (tmp_path / 'd.json').write_text(json.dumps({'customers': [{'id': 'gamma', 'name': 'Gamma Ltd'}], 'orders': [
        {'id': 'J1', 'customer': 'gamma', 'cutoff_in_s': 600, 'lines': [{'sku': 'SKU-D', 'qty': 1}]},
        {'id': 'J2', 'customer': 'gamma', 'lines': [{'sku': 'SKU-D', 'qty': 1}]}]}))
    r1 = importer.apply(w.orders, importer.read(str(tmp_path / 'o.csv')), 0.0)
    r2 = importer.apply(w.orders, importer.read(str(tmp_path / 'a.csv')), 0.0)
    r3 = importer.apply(w.orders, importer.read(str(tmp_path / 'd.json')), 100.0)
    assert r1['created'] == ['A1', 'A2'] and len(w.orders.orders['A1']['lines']) == 2
    assert r2['created'] == ['N1'] and len(w.orders.asns['N1']['boxes']) == 2
    assert r3['created'] == ['J1'] and w.orders.orders['J1']['cutoff_s'] == 700 and 'J2' in r3['errors'][0]
    assert w.orders.customers['gamma'] == 'Gamma Ltd'


def test_rest_api_as_an_erp_uses_it(monkeypatch, wms):
    w = wms(orders=True)
    api = Api(w.node, w.orders)
    srv = serve(api, '127.0.0.1', 0)
    url = f'http://127.0.0.1:{srv.server_address[1]}'

    def call(method, path, body=None, key=None):
        req = urllib.request.Request(url + path, method=method, data=json.dumps(body).encode() if body else None,
                                     headers={'X-Api-Key': key} if key else {})
        try:
            with urllib.request.urlopen(req) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())
    try:
        order = {'id': 'R1', 'customer': 'acme', 'cutoff_s': 900, 'lines': [{'sku': 'SKU-A', 'qty': 2}]}
        assert call('POST', '/api/v1/orders', order)[0] == 201
        assert call('POST', '/api/v1/orders', order)[0] == 200                  # ERP retry
        w.orders.orders['R1']['lines'][0]['assigned'] = 1
        assert call('POST', '/api/v1/orders', dict(order, lines=[{'sku': 'SKU-A', 'qty': 9}]))[0] == 409
        assert call('POST', '/api/v1/orders', {'id': 'R2', 'customer': 'acme', 'lines': []})[0] == 400
        code, got = call('GET', '/api/v1/orders/R1')
        assert code == 200 and got['lines'][0]['qty'] == 2 and 'effective_priority' in got
        assert call('GET', '/api/v1/orders/nope')[0] == 404
        assert call('POST', '/api/v1/asns', {'id': 'AS1', 'supplier': 's', 'boxes': [{'sku': 'K', 'box_id': 'b1'}]})[0] == 201
        assert len(call('GET', '/api/v1/inventory')[1]['bins']) == 128
        assert 'orders' in call('GET', '/api/v1/status')[1]
        assert call('GET', '/api/v1/events')[1]['events']
        assert call('DELETE', '/api/v1/orders/R1')[1]['state'] == 'CANCELLED'
        monkeypatch.setenv('WMS_API_KEY', 'k-test')
        assert call('GET', '/api/v1/status')[0] == 401 and call('GET', '/api/v1/status', key='k-test')[0] == 200
    finally:
        srv.shutdown()


def test_orders_survive_a_restart(tmp_path, wms):
    db = str(tmp_path / 'w.db')
    w1 = wms(orders=True, store=Store(db))
    w1.orders.add_order('P1', 'acme', [('SKU-A', 2)], 900)
    w1.orders.add_asn('Q1', 's', [('SKU-B', 'q1')])
    w1.node.store.close()
    w2 = wms(orders=True, store=Store(db))
    assert w2.orders.orders['P1']['lines'][0]['qty'] == 2 and 'Q1' in w2.orders.asns and 'acme' in w2.orders.customers
