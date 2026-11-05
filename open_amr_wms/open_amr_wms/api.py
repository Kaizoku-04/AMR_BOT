"""The WMS's HTTP interface on the warehouse LAN: a REST API for the ERP (v1) and the operator dashboard (static files).

Standard library only (http.server, threads; ADR-046): nothing to install on the site PC, nothing loaded from the
internet. `handle(method, path, body)` is pure and unit-tested; `serve()` puts it on a port. Every call takes the
WMS lock, the same one the ROS callbacks take.

  POST   /api/v1/orders            one order or {"orders": [...]}: 201 created / 200 unchanged (an identical resend:
                                   ERP retries are safe) / 409 a change to an order already being picked / 400 invalid
  GET    /api/v1/orders[?state=]   list;  GET /api/v1/orders/<id>: one, with lines picked / assigned / late
  DELETE /api/v1/orders/<id>       cancel (its unclaimed tasks are withdrawn)
  POST   /api/v1/asns              one ASN or {"asns": [...]};  GET /api/v1/asns/<id>
  GET    /api/v1/inventory         every bin: full, SKU, box id
  GET    /api/v1/status            KPIs (fleet, orders, stations, robots)
  GET    /api/v1/events?since=<id> the event log
Auth: if the environment has WMS_API_KEY, every /api call needs header X-Api-Key with that value (no key in the repo).
"""
import json
import mimetypes
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from . import importer
from .orders import OrderError

WEB = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')


class Api:
    def __init__(self, wms, orders, lock=None, extra=None):
        self.wms, self.orders = wms, orders
        self.lock = lock or threading.RLock()
        self.extra = extra or {}          # path -> callable(method, query, body) for more endpoints (dashboard)

    def handle(self, method, path, body=None, headers=None):
        u = urlparse(path)
        parts = [p for p in u.path.split('/') if p]
        q = {k: v[-1] for k, v in parse_qs(u.query).items()}
        key = os.environ.get('WMS_API_KEY')
        if parts[:1] == ['api'] and key and (headers or {}).get('X-Api-Key') != key:
            return 401, {'error': 'missing or wrong X-Api-Key'}
        try:
            data = json.loads(body) if body else {}
        except json.JSONDecodeError as e:
            return 400, {'error': f'invalid JSON: {e}'}
        with self.lock:
            try:
                return self.route(method, parts, q, data)
            except OrderError as e:
                return (409 if 'refused' in str(e) or 'already' in str(e) or 'exists' in str(e) else 400), \
                    {'error': str(e)}

    def route(self, method, parts, q, data):
        w, o, now = self.wms, self.orders, self.wms.now()
        if parts[:2] != ['api', 'v1']:
            fn = self.extra.get('/' + '/'.join(parts))
            if fn is not None:
                return fn(method, q, data)
            return 404, {'error': 'not found'}
        res = parts[2:]
        if res == ['orders'] and method == 'POST':
            doc = data if 'orders' in data else {'orders': [data]}
            out = importer.apply(o, doc, now, w)
            conflict = [e for e in out['errors'] if 'refused' in e or 'already' in e]
            code = 409 if conflict and not out['created'] else 400 if out['errors'] and not out['created'] and \
                not out['unchanged'] else 201 if out['created'] else 200
            return code, out
        if res == ['orders'] and method == 'GET':
            rows = [x for x in o.orders.values() if 'state' not in q or x['state'] == q['state'].upper()]
            return 200, {'orders': sorted(rows, key=lambda x: x['id'])}
        if len(res) == 2 and res[0] == 'orders':
            x = o.orders.get(res[1])
            if x is None:
                return 404, {'error': f'no order {res[1]}'}
            if method == 'DELETE':
                return 200, o.cancel_order(res[1], w)
            return 200, dict(x, effective_priority=o.effective_priority(x, now))
        if res == ['asns'] and method == 'POST':
            doc = data if 'asns' in data else {'asns': [data]}
            out = importer.apply(o, doc, now, w)
            return (201 if out['created'] else 200 if out['unchanged'] else 400), out
        if len(res) == 2 and res[0] == 'asns':
            a = o.asns.get(res[1])
            return (200, a) if a else (404, {'error': f'no ASN {res[1]}'})
        if res == ['inventory']:
            return 200, {'bins': w.bins}
        if res == ['events']:
            return 200, {'events': w.store.events(int(q.get('since', 0)), int(q.get('limit', 500)))}
        if res == ['status']:
            return 200, dict(wms=w.metrics(), orders=o.summary(now), now=now)
        return 404, {'error': 'not found'}


def serve(api, host='0.0.0.0', port=8080):
    """Start the HTTP server in a daemon thread; returns the server (server.shutdown() to stop)."""

    class H(BaseHTTPRequestHandler):
        def _go(self, method):
            n = int(self.headers.get('Content-Length') or 0)
            body = self.rfile.read(n).decode() if n else None
            path = urlparse(self.path).path
            if method == 'GET' and not path.startswith('/api/') and path not in api.extra:
                return self._static(path)
            code, out = api.handle(method, self.path, body, dict(self.headers))
            raw = json.dumps(out, default=str).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def _static(self, path):
            rel = 'index.html' if path in ('', '/') else path.lstrip('/')
            full = os.path.realpath(os.path.join(WEB, rel))
            if not full.startswith(os.path.realpath(WEB) + os.sep) or not os.path.isfile(full):
                self.send_response(404); self.end_headers(); return
            with open(full, 'rb') as f:
                raw = f.read()
            self.send_response(200)
            self.send_header('Content-Type', mimetypes.guess_type(full)[0] or 'application/octet-stream')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            self._go('GET')

        def do_POST(self):
            self._go('POST')

        def do_DELETE(self):
            self._go('DELETE')

        def log_message(self, *a):
            pass

    srv = ThreadingHTTPServer((host, port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv
