"""Orders, customers and ASNs from CSV or JSON files (the batch path an ERP export takes; the REST API is the live one).

CSV, one row per line (header required):
  orders:  order_id,customer,sku,qty,cutoff_s,priority          (rows with the same order_id form one order)
  asns:    asn_id,supplier,sku,box_id                           (rows in pallet order)
JSON:    {"customers": [{"id", "name"}], "orders": [{"id", "customer", "cutoff_s" | "cutoff_in_s", "priority",
          "lines": [{"sku", "qty"}]}], "asns": [{"id", "supplier", "boxes": [{"sku", "box_id"}]}]}
Times are seconds on the WMS clock (sim time in the simulator, ADR-043); `cutoff_in_s` is relative to the import.
"""
import csv
import json

from .orders import OrderError


def _cutoff(o, now):
    if 'cutoff_s' in o and o['cutoff_s'] not in ('', None):
        return float(o['cutoff_s'])
    if 'cutoff_in_s' in o and o['cutoff_in_s'] not in ('', None):
        return now + float(o['cutoff_in_s'])
    raise OrderError(f'order {o.get("id") or o.get("order_id")}: cutoff_s or cutoff_in_s needed')


def apply(orders, doc, now, wms=None):
    """Apply a JSON-shaped document. Returns dict(created=[ids], unchanged=[ids], errors=[text]); one bad order
    doesn't stop the others (an ERP batch is not all-or-nothing)."""
    out = dict(created=[], unchanged=[], errors=[])
    for c in doc.get('customers', []):
        try:
            orders.add_customer(c['id'], c.get('name', ''))
        except (OrderError, KeyError) as e:
            out['errors'].append(f'customer: {e}')
    for o in doc.get('orders', []):
        try:
            _, new = orders.add_order(o['id'], o['customer'], o['lines'], _cutoff(o, now), o.get('priority', 0), now)
            out['created' if new else 'unchanged'].append(o['id'])
        except (OrderError, KeyError, TypeError, ValueError) as e:
            out['errors'].append(f'order {o.get("id", "?")}: {e}')
    for a in doc.get('asns', []):
        try:
            _, new = orders.add_asn(a['id'], a.get('supplier', ''), a['boxes'], now)
            out['created' if new else 'unchanged'].append(a['id'])
        except (OrderError, KeyError, TypeError, ValueError) as e:
            out['errors'].append(f'asn {a.get("id", "?")}: {e}')
    return out


def read_csv(path):
    """A CSV file -> the JSON-shaped document (orders or ASNs, told apart by the header)."""
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    if 'order_id' in rows[0]:
        by = {}
        for r in rows:
            o = by.setdefault(r['order_id'], dict(id=r['order_id'], customer=r['customer'],
                                                   cutoff_s=r.get('cutoff_s', ''), cutoff_in_s=r.get('cutoff_in_s', ''),
                                                   priority=int(r.get('priority') or 0), lines=[]))
            o['lines'].append(dict(sku=r['sku'], qty=int(r['qty'])))
        return dict(orders=list(by.values()))
    if 'asn_id' in rows[0]:
        by = {}
        for r in rows:
            by.setdefault(r['asn_id'], dict(id=r['asn_id'], supplier=r.get('supplier', ''), boxes=[]))['boxes'].append(
                dict(sku=r['sku'], box_id=r['box_id']))
        return dict(asns=list(by.values()))
    raise OrderError(f'{path}: header must start an order file (order_id,...) or an ASN file (asn_id,...)')


def read(path):
    if path.endswith('.json'):
        with open(path) as f:
            return json.load(f)
    return read_csv(path)


def main():
    """wms_import <file.csv|file.json> [--url http://<wms>:8080] — sends the file to a running WMS's REST API."""
    import argparse
    import urllib.request
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('file')
    ap.add_argument('--url', default='http://localhost:8080')
    a = ap.parse_args()
    doc = read(a.file)
    for kind in ('orders', 'asns'):
        if doc.get(kind):
            req = urllib.request.Request(f'{a.url}/api/v1/{kind}', data=json.dumps({kind: doc[kind]}).encode(),
                                         method='POST', headers={'Content-Type': 'application/json',
                                                                 'X-Api-Key': __import__('os').environ.get(
                                                                     'WMS_API_KEY', '')})
            try:
                with urllib.request.urlopen(req) as r:
                    print(kind, r.status, r.read().decode())
            except urllib.error.HTTPError as e:
                print(kind, e.code, e.read().decode())
