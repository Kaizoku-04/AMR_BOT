"""Orders in: customers, outbound orders (SKU lines, quantities, cut-off, priority), inbound receipts (ASN per pallet),
and the mapping to tasks. Pure logic on the WMS store; decisions ADR-043..046.

Model
  customer   id, name.
  order      id, customer, lines [(sku, qty)], cutoff_s (sim/site time the truck leaves), priority (0 normal,
             1 expedite, 2 urgent; ADR-045), state OPEN -> PICKING -> SHIPPED | SHORT_SHIPPED | CANCELLED.
  asn        id, supplier, boxes [(sku, box_id)] in pallet order, state EXPECTED -> RECEIVING -> RECEIVED.
Mapping (ADR-044)
  One task per box. An outbound task picks one box of an order line's SKU from a bin that holds it; `mission_id`
  is the order id, so the robots' heartbeats show which order a robot works for. Orders are served by effective
  priority, then cut-off, then id; an order's lines in their order.
  An inbound task takes the next box of the oldest ASN still receiving; its SKU and box id go into the bin on DONE.
  Without an ASN the box is booked as 'UNANNOUNCED' (an exception in the event log) unless `require_asn`.
Deadlines (ADR-043): WMS-internal. Task.deadline carries the order's cut-off for observers; robots ignore it. The WMS
  raises the effective priority of an order's tasks when its cut-off is near (`escalate_s`, +1) and when it is late
  (+2, capped at 3), so robots consider them first (Task.priority is the only lever the robots read).
Cut-off (ADR-044): at cut-off an unfinished order ships what is done: SHORT_SHIPPED, the short quantity recorded,
  its unclaimed tasks withdrawn (CANCEL); claimed ones finish (the box reaches the outbound pallet; it is booked as
  `late` against the order).
"""
import json

SCHEMA = """
CREATE TABLE IF NOT EXISTS customers (id TEXT PRIMARY KEY, name TEXT);
CREATE TABLE IF NOT EXISTS orders (id TEXT PRIMARY KEY, data TEXT NOT NULL, state TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS asns (id TEXT PRIMARY KEY, data TEXT NOT NULL, state TEXT NOT NULL);
"""
PRIORITIES = {0: 'normal', 1: 'expedite', 2: 'urgent'}
FINAL = ('SHIPPED', 'SHORT_SHIPPED', 'CANCELLED')


class OrderError(ValueError):
    pass


class Orders:
    def __init__(self, store, escalate_s=1800.0, require_asn=False):
        self.db = store.db
        self.db.executescript(SCHEMA)
        self.store, self.escalate_s, self.require_asn = store, escalate_s, require_asn
        self.orders = {r[0]: json.loads(r[1]) for r in self.db.execute('SELECT id, data FROM orders')}
        self.asns = {r[0]: json.loads(r[1]) for r in self.db.execute('SELECT id, data FROM asns')}
        self.customers = dict(self.db.execute('SELECT id, name FROM customers').fetchall())
        self.unannounced = store.counter('unannounced')
        self.live = {k for k, o in self.orders.items() if o['state'] not in FINAL}   # orders not shipped / cancelled

    # ------------------------------------------------------------------ intake (CSV / JSON / REST all end here)
    def add_customer(self, cid, name=''):
        if not cid:
            raise OrderError('customer id missing')
        self.customers[cid] = name or self.customers.get(cid, cid)
        self.db.execute('INSERT OR REPLACE INTO customers VALUES (?,?)', (cid, self.customers[cid]))

    def add_order(self, oid, customer, lines, cutoff_s, priority=0, now=0.0):
        """lines: [(sku, qty)] or [{'sku':..., 'qty':...}]. Idempotent for an identical resend; a changed resend of an
        order already being picked is refused (an ERP retry must not double an order)."""
        lines = [(l['sku'], int(l['qty'])) if isinstance(l, dict) else (l[0], int(l[1])) for l in lines]
        if not oid or not customer or not lines:
            raise OrderError('order needs id, customer and at least one line')
        if any(q <= 0 or not s for s, q in lines):
            raise OrderError(f'order {oid}: every line needs a SKU and a quantity > 0')
        if int(priority) not in PRIORITIES:
            raise OrderError(f'order {oid}: priority must be 0 (normal), 1 (expedite) or 2 (urgent)')
        new = dict(id=oid, customer=customer, cutoff_s=float(cutoff_s), priority=int(priority), received_s=now,
                   lines=[dict(sku=s, qty=q, picked=0, assigned=0, late=0) for s, q in lines], state='OPEN',
                   shipped_s=None, short=0)
        old = self.orders.get(oid)
        if old is not None:
            same = old['customer'] == customer and [(l['sku'], l['qty']) for l in old['lines']] == lines and \
                old['cutoff_s'] == new['cutoff_s'] and old['priority'] == new['priority']
            if same:
                return old, False
            if old['state'] != 'OPEN' or any(l['assigned'] or l['picked'] for l in old['lines']):
                raise OrderError(f'order {oid} is already {old["state"].lower()}; changes are refused')
        if customer not in self.customers:
            self.add_customer(customer)
        self.orders[oid] = new
        self.save_order(oid)
        self.store.event(now, 'erp', f'order {oid} received', f'{customer}, {sum(q for _, q in lines)} boxes, '
                         f'cut-off {cutoff_s:.0f} s, {PRIORITIES[int(priority)]}', oid)
        return new, True

    def cancel_order(self, oid, wms=None, why='cancelled by the customer'):
        o = self.orders.get(oid)
        if o is None:
            raise OrderError(f'no order {oid}')
        if o['state'] in FINAL:
            raise OrderError(f'order {oid} is already {o["state"].lower()}')
        o['state'] = 'CANCELLED'
        if wms is not None:
            for tid, m in list(wms.meta.items()):
                if m.get('mission_id') == oid and m['done'] is None:
                    wms.withdraw(tid, f'order {oid} {why}')
        self.save_order(oid)
        self.store.event(wms.now() if wms else 0.0, 'erp', f'order {oid} cancelled', why, oid)
        return o

    def add_asn(self, aid, supplier, boxes, now=0.0):
        boxes = [dict(sku=b['sku'], box_id=b['box_id']) if isinstance(b, dict) else dict(sku=b[0], box_id=b[1])
                 for b in boxes]
        if not aid or not boxes or any(not b['sku'] or not b['box_id'] for b in boxes):
            raise OrderError('ASN needs an id and boxes with SKU and box id')
        if aid in self.asns:
            if [(b['sku'], b['box_id']) for b in self.asns[aid]['boxes']] == [(b['sku'], b['box_id']) for b in boxes]:
                return self.asns[aid], False
            raise OrderError(f'ASN {aid} exists with other boxes')
        known = {b['box_id'] for a in self.asns.values() for b in a['boxes']}
        dup = [b['box_id'] for b in boxes if b['box_id'] in known]
        if dup or len({b['box_id'] for b in boxes}) != len(boxes):
            raise OrderError(f'ASN {aid}: box ids already known: {dup[:3]}')
        self.asns[aid] = dict(id=aid, supplier=supplier, boxes=boxes, next=0, received=0, state='EXPECTED',
                              received_s=now)
        self.save_asn(aid)
        self.store.event(now, 'erp', f'ASN {aid} announced', f'{supplier}, {len(boxes)} boxes', aid)
        return self.asns[aid], True

    def save_order(self, oid):
        o = self.orders[oid]
        (self.live.discard if o['state'] in FINAL else self.live.add)(oid)
        self.db.execute('INSERT OR REPLACE INTO orders VALUES (?,?,?)', (oid, json.dumps(o), o['state']))

    def save_asn(self, aid):
        a = self.asns[aid]
        self.db.execute('INSERT OR REPLACE INTO asns VALUES (?,?,?)', (aid, json.dumps(a), a['state']))

    # ------------------------------------------------------------------ priorities and cut-offs
    def effective_priority(self, o, now):
        left = o['cutoff_s'] - now
        bump = 2 if left < 0 else 1 if left < self.escalate_s else 0
        return min(3, o['priority'] + bump)

    def task_priority(self, o, now):
        """Task.priority on the ADR-045 scale: 1 normal (inbound too), 2 expedite, 3 urgent, 4 late / cut-off near."""
        return min(4, 1 + self.effective_priority(o, now))

    def queue(self, now):
        live = [self.orders[k] for k in self.live]
        return sorted(live, key=lambda o: (-self.effective_priority(o, now), o['cutoff_s'], o['id']))

    # ------------------------------------------------------------------ the WMS's work source interface
    def choose(self, wms, inbound, pool, seq):
        now = wms.now()
        if inbound:
            for a in sorted(self.asns.values(), key=lambda a: (a['received_s'], a['id'])):
                if a.get('requeue'):
                    box = a['requeue'].pop(0)
                elif a['next'] < len(a['boxes']):
                    box = a['boxes'][a['next']]
                    a['next'] += 1
                else:
                    continue
                a['state'] = 'RECEIVING'
                self.save_asn(a['id'])
                return wms.rng.choice(pool), dict(mission_id=a['id'], priority=1, sku=box['sku'],
                                                  box_id=box['box_id'], asn=a['id'])
            if self.require_asn:
                return None
            self.unannounced += 1
            self.store.set_counter('unannounced', self.unannounced)
            wms.event('wms', 'box without an ASN at receiving', f'booked as UNANNOUNCED-{self.unannounced:05d}')
            return wms.rng.choice(pool), dict(mission_id='unannounced', priority=1, sku='UNANNOUNCED',
                                              box_id=f'UNANNOUNCED-{self.unannounced:05d}', asn='')
        for o in self.queue(now):
            if o['cutoff_s'] <= now:
                continue                                  # handled by tick (short ship)
            for line in o['lines']:
                if line['qty'] - line['picked'] - line['assigned'] <= 0:
                    continue
                bins = [b for b in pool if wms.bins[b]['full'] and wms.bins[b]['sku'] == line['sku']]
                if not bins:
                    continue
                b = bins[0]
                line['assigned'] += 1
                if o['state'] == 'OPEN':
                    o['state'] = 'PICKING'
                self.save_order(o['id'])
                return b, dict(mission_id=o['id'], priority=self.task_priority(o, now), sku=line['sku'],
                               box_id=wms.bins[b]['box_id'], deadline_s=o['cutoff_s'], order=o['id'])
        return None

    def tick(self, wms):
        now = wms.now()
        by_order = {}
        for tid, m in wms.live():                         # open tasks only (stress 2026-11-05: scanning every task of
            if m.get('order'):                            # the shift for every order was 84 % of the WMS's time)
                by_order.setdefault(m['order'], []).append((tid, m))
        for oid in sorted(self.live):
            o = self.orders[oid]
            if now >= o['cutoff_s']:
                self.short_ship(wms, o, now, by_order.get(oid, []))
                continue
            p = self.task_priority(o, now)
            for tid, m in by_order.get(oid, []):
                if m['claimed'] is None and m['priority'] != p:
                    m['priority'] = p                     # takes effect at the next re-offer
                    wms.save(tid)

    def short_ship(self, wms, o, now, tasks=None):
        tasks = tasks if tasks is not None else [(t, m) for t, m in wms.live() if m.get('order') == o['id']]
        for tid, m in tasks:
            if m['done'] is None and m['claimed'] is None:
                wms.withdraw(tid, f'order {o["id"]} reached its cut-off')     # withdrawn() returns the line
        short = sum(l['qty'] - l['picked'] for l in o['lines'])
        o['state'] = 'SHIPPED' if short <= 0 else 'SHORT_SHIPPED'
        o['short'], o['shipped_s'] = max(short, 0), now
        self.save_order(o['id'])
        wms.event('wms', f'order {o["id"]} {"shipped" if short <= 0 else "short-shipped"} at its cut-off',
                  f'{sum(l["picked"] for l in o["lines"])} of {sum(l["qty"] for l in o["lines"])} boxes',
                  o['id'], 'warn' if short > 0 else 'info')

    def withdrawn(self, wms, m):
        """A task of ours was withdrawn before anyone took it: its order line or ASN box is open again."""
        if m.get('order') and m['order'] in self.orders:
            o = self.orders[m['order']]
            for line in o['lines']:
                if line['sku'] == m['sku'] and line['assigned'] > 0:
                    line['assigned'] -= 1
                    break
            self.save_order(o['id'])
        elif m.get('asn') and m['asn'] in self.asns:
            a = self.asns[m['asn']]
            a.setdefault('requeue', []).append(dict(sku=m['sku'], box_id=m['box_id']))
            self.save_asn(a['id'])

    def done(self, wms, m):
        if m.get('order'):
            o = self.orders.get(m['order'])
            if o is None:
                return
            for line in o['lines']:
                if line['sku'] == m['sku'] and line['assigned'] > 0:
                    line['assigned'] -= 1
                    if o['state'] in FINAL:
                        line['late'] += 1             # arrived after the truck left
                    else:
                        line['picked'] += 1
                    break
            if o['state'] not in FINAL and all(l['picked'] >= l['qty'] for l in o['lines']):
                o['state'], o['shipped_s'] = 'SHIPPED', wms.now()
                wms.event('wms', f'order {o["id"]} complete', f'{sum(l["qty"] for l in o["lines"])} boxes',
                          o['id'])
            self.save_order(o['id'])
        elif m.get('asn'):
            a = self.asns.get(m['asn'])
            if a is not None:
                a['received'] += 1
                if a['received'] >= len(a['boxes']):
                    a['state'] = 'RECEIVED'
                    wms.event('wms', f'ASN {a["id"]} received', f'{len(a["boxes"])} boxes in storage', a['id'])
                self.save_asn(a['id'])

    # ------------------------------------------------------------------ reporting
    def summary(self, now):
        st = {}
        for o in self.orders.values():
            st[o['state']] = st.get(o['state'], 0) + 1
        final = [o for o in self.orders.values() if o['state'] in ('SHIPPED', 'SHORT_SHIPPED')]
        boxes = sum(l['qty'] for o in final for l in o['lines'])
        picked = sum(l['picked'] for o in final for l in o['lines'])
        late = sum(1 for o in final if o['state'] == 'SHORT_SHIPPED')
        return dict(states=st, fill_rate=picked / boxes if boxes else 1.0, late_orders=late, orders_closed=len(final))
