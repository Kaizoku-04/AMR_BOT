"""Operations (pure): the dashboard's live view, KPIs, alarms in plain language, the end-of-shift report, and
operator commands (published on /swarm/operator, logged with who / when / why).

Alarms are derived from state every time they are asked for (heartbeats, station states, task records), never kept
as events: an alarm clears itself when its cause is gone (state over events). Each says what, where, since when and,
where the heartbeats tell it, why ("amr_3 stuck at aisle_2_bay_4 (node 41) for 2 min, blocked by amr_5").
"""
import math

from open_amr_msgs.msg import OperatorCommand, RobotState, StationState

from .core import to_time

STATUS = {v: k for k, v in vars(RobotState).items() if k.isupper() and isinstance(v, int)}
ST_STATE = {StationState.IDLE: 'IDLE', StationState.BUSY: 'BUSY', StationState.FAULT: 'FAULT',
            StationState.STARVED: 'STARVED'}
COMMANDS = {k: v for k, v in vars(OperatorCommand).items() if k.isupper() and isinstance(v, int)}


def mins(s):
    return f'{s / 60:.0f} min' if s >= 90 else f'{s:.0f} s'


class Ops:
    def __init__(self, wms, orders=None, graph=None, stuck_alarm_s=60.0, yield_alarm_s=60.0, low_battery=0.15):
        self.w, self.o, self.g = wms, orders, graph
        self.stuck_alarm_s, self.yield_alarm_s, self.low_battery = stuck_alarm_s, yield_alarm_s, low_battery
        self.since = {}                    # (robot, status) -> when it entered that status (from heartbeats)
        self.cmd_log = []

    # ------------------------------------------------------------------ heartbeat bookkeeping (called per heartbeat)
    def on_state(self, s):
        key = s.robot_id
        cur = self.since.get(key)
        if cur is None or cur[0] != (s.status, s.last_node):
            self.since[key] = ((s.status, s.last_node), self.w.now())

    def node_name(self, nid):
        if self.g is None:
            return str(nid)
        return f'{self.g.name.get(nid, nid)} (node {nid})'

    # ------------------------------------------------------------------ alarms
    def alarms(self):
        w, now, out = self.w, self.w.now(), []
        holders = {}
        for rid, s in w.robots.items():
            for n in s.reserved_nodes:
                holders.setdefault(n, []).append(rid)
        for rid, s in sorted(w.robots.items()):
            age = now - w.robot_rx.get(rid, now)
            if age >= w.peer_timeout:
                rel = rid in w.released
                out.append(dict(level='warn' if rel else 'error', robot=rid, since=now - age,
                                text=f'{rid} not heard for {mins(age)}, last at {self.node_name(s.last_node)}'
                                     + (' — released by an operator' if rel else
                                        ' — its lanes count as blocked; release it if it was removed')
                                     + (f'; box on its deck ({s.task_id})' if s.loaded else '')))
                continue
            (st, node), t0 = self.since.get(rid, ((s.status, s.last_node), now))
            dur = now - t0
            nxt = s.route[0] if len(s.route) else None
            by = [r for r in holders.get(nxt, []) if r != rid] if nxt is not None else []
            why = f', blocked by {", ".join(by)} at {self.node_name(nxt)}' if by else ''
            if s.status == RobotState.STUCK and dur >= self.stuck_alarm_s:
                out.append(dict(level='error', robot=rid, since=t0,
                                text=f'{rid} stuck at {self.node_name(s.last_node)} for {mins(dur)}{why}'
                                     + (f' with a box on its deck ({s.task_id})' if s.loaded else '')))
            elif s.status == RobotState.YIELDING and dur >= self.yield_alarm_s:
                out.append(dict(level='warn', robot=rid, since=t0,
                                text=f'{rid} waiting at {self.node_name(s.last_node)} for {mins(dur)}{why}'))
            elif s.status == RobotState.ISOLATED:
                out.append(dict(level='error', robot=rid, since=t0,
                                text=f'{rid} hears less than half the fleet: stopped at {self.node_name(s.last_node)} '
                                     f'(network) for {mins(dur)}'))
            if s.battery < self.low_battery and s.status not in (RobotState.CHARGING,):
                out.append(dict(level='warn', robot=rid, since=now,
                                text=f'{rid} battery {100 * s.battery:.0f} % and not on a charger'))
        for bay, (st, rx) in sorted(w.stations.items()):
            age = now - rx
            if age >= w.station_timeout:
                out.append(dict(level='error', station=st.station_id, since=rx,
                                text=f'{st.station_id} silent for {mins(age)}: no work offered for {bay}'))
            elif st.state == StationState.FAULT:
                out.append(dict(level='error', station=st.station_id, since=now,
                                text=f'{st.station_id} out of service (FAULT): its robots give their tasks back after '
                                     f'30 s; a person clears the cell'))
            elif st.state == StationState.STARVED:
                what = 'no boxes staged (next pallet due)' if 'receiving' in bay else 'no free pallet slot (pickup due)'
                out.append(dict(level='info', station=st.station_id, since=now, text=f'{st.station_id}: {what}'))
        for tid, m in sorted(w.meta.items()):
            if m['stranded'] and m['done'] is None:
                out.append(dict(level='error', task=tid, since=m['claimed'] or m['offered'],
                                text=f'box {m.get("box_id") or "?"} ({m.get("sku") or "?"}, {tid}'
                                     f'{", order " + m["order"] if m.get("order") else ""}) on {m["robot"] or "?"}: '
                                     f'needs a person to recover it'))
        if self.o is not None:
            for o in self.o.queue(now):
                left = o['cutoff_s'] - now
                need = sum(l['qty'] - l['picked'] for l in o['lines'])
                if 0 <= left < max(600.0, 120.0 * need) and need > 0:      # at risk: < 10 min, or < 2 min a box
                    out.append(dict(level='warn', order=o['id'], since=now,
                                    text=f'order {o["id"]} ({o["customer"]}): cut-off in {mins(left)}, '
                                         f'{need} of {sum(l["qty"] for l in o["lines"])} boxes still to pick'))
        rank = {'error': 0, 'warn': 1, 'info': 2}
        return sorted(out, key=lambda a: (rank[a['level']], a['since']))

    # ------------------------------------------------------------------ KPIs
    def kpis(self):
        w = self.w
        x = w.metrics()
        hours = max(x['t'], 1e-6) / 3600
        per_robot = {}
        for m in w.done_tasks():
            if m['kind'] != 'goto':
                per_robot[m['robot']] = per_robot.get(m['robot'], 0) + 1
        per_robot = {r: n / hours for r, n in sorted(per_robot.items())}
        arms = {}
        for bay, d in w.dock.items():
            T = max(sum(d.values()), 1e-6)
            arms[bay] = {k: v / T for k, v in d.items()}
        robots = list(w.robots.values())
        x.update(boxes_per_h=(x['by_kind']['inbound'] + x['by_kind']['outbound']) / hours, per_robot_h=per_robot,
                 arms=arms, battery_min=min((r.battery for r in robots), default=1.0),
                 battery_mean=sum(r.battery for r in robots) / max(len(robots), 1))
        if self.o is not None:
            x['orders'] = self.o.summary(w.now())
        return x

    # ------------------------------------------------------------------ live view
    def snapshot(self):
        w, now = self.w, self.w.now()
        robots = []
        for rid, s in sorted(w.robots.items()):
            robots.append(dict(id=rid, x=s.pose.x, y=s.pose.y, theta=s.pose.theta, status=STATUS.get(s.status, '?'),
                               battery=s.battery, task=s.task_id, mission=s.mission_id, loaded=s.loaded,
                               node=s.last_node, silent=now - w.robot_rx.get(rid, now) >= w.peer_timeout,
                               route=list(s.route)))
        stations = [dict(id=st.station_id, bay=bay, state=ST_STATE.get(st.state, '?'), boxes=st.boxes_available,
                         pallets=list(st.pallets), stale=now - rx >= w.station_timeout)
                    for bay, (st, rx) in sorted(w.stations.items())]
        tasks = [dict(id=t, kind=m['kind'], origin=m['origin'], dest=m['dest'], bin=m['bin'], robot=m['robot'],
                      state=w.state_of(m), priority=m['priority'], order=m.get('order', ''))
                 for t, m in sorted(w.meta.items()) if m['done'] is None]
        return dict(now=now, robots=robots, stations=stations, tasks=tasks,
                    bins={b: r['full'] for b, r in w.bins.items()}, alarms=self.alarms(), kpis=self.kpis(),
                    events=w.store.events(max(0, self.last_event_id() - 30)))

    def last_event_id(self):
        r = self.w.store.db.execute('SELECT MAX(id) FROM events').fetchone()
        return r[0] or 0

    def layout(self):
        g = self.g
        nodes = [dict(id=n, x=p[0], y=p[1], name=g.name[n], kind=g.kind[n]) for n, p in g.pos.items()]
        edges = [[a, b, g.edge_kind.get((a, b), 'street')] for a, succ in g.succ.items() for b in succ]
        return dict(nodes=nodes, edges=edges)

    # ------------------------------------------------------------------ operator commands
    def command(self, name, target='', issued_by='', reason=''):
        """Publish an OperatorCommand (RELEASE_ROBOT, ACK_FAULT, and whatever else open_amr_msgs defines) and log it.
        issued_by and reason are required: the event log records who did what and why."""
        if name not in COMMANDS:
            raise ValueError(f'unknown command {name!r}; known: {sorted(COMMANDS)}')
        if not issued_by or not reason:
            raise ValueError('issued_by and reason are required (event log)')
        if name in ('RELEASE_ROBOT', 'ACK_FAULT') and not target:
            raise ValueError(f'{name} needs a target robot')
        msg = OperatorCommand(stamp=to_time(self.w.now()), command=COMMANDS[name], target=target, issued_by=issued_by,
                              reason=reason)
        self.w.publish('/swarm/operator', msg)
        self.w.event(issued_by, f'{name} {target or "fleet"}', reason, target, 'warn')
        if name == 'RELEASE_ROBOT':
            self.w.on_operator(msg)              # our own subscription may not see our own message
        return dict(command=name, target=target, issued_by=issued_by, reason=reason, t=self.w.now())

    # ------------------------------------------------------------------ end-of-shift report
    def report(self):
        k = self.kpis()
        w = self.w
        lines = [f'# End-of-shift report', '', f'Shift length: {k["t"] / 3600:.2f} h (WMS clock)', '',
                 '## Throughput', '',
                 f'- Boxes moved: {k["by_kind"]["inbound"] + k["by_kind"]["outbound"]} '
                 f'({k["by_kind"]["inbound"]} in, {k["by_kind"]["outbound"]} out) = **{k["boxes_per_h"]:.1f} boxes/h**',
                 f'- Task latency (offer -> done): mean {k["latency_mean"]:.0f} s, p95 {k["latency_p95"]:.0f} s',
                 f'- Per robot (boxes/h): ' + ', '.join(f'{r} {v:.1f}' for r, v in k['per_robot_h'].items()), '',
                 '## Robot time', '', '| driving | yielding | at station | charging | idle |', '|---|---|---|---|---|',
                 '| ' + ' | '.join(f'{100 * k["shares"][s]:.0f} %' for s in ('driving', 'yielding', 'station',
                                                                          'charging', 'idle')) + ' |', '',
                 '## Arm cells', '', '| bay | busy | idle | starved | fault |', '|---|---|---|---|---|']
        for bay, a in sorted(k['arms'].items()):
            lines.append(f'| {bay} | ' + ' | '.join(f'{100 * a.get(s, 0):.0f} %' for s in ('busy', 'idle', 'starved',
                                                                                          'fault')) + ' |')
        lines += ['', '## Energy', '', f'- Battery now: lowest {100 * k["battery_min"]:.0f} %, mean '
                                         f'{100 * k["battery_mean"]:.0f} %', '']
        if 'orders' in k:
            o = k['orders']
            lines += ['## Orders', '', f'- Closed: {o["orders_closed"]}; fill rate {100 * o["fill_rate"]:.1f} %; '
                                       f'short-shipped (late) {o["late_orders"]}',
                      f'- By state: ' + ', '.join(f'{s} {n}' for s, n in sorted(o['states'].items())), '']
        lines += ['## Exceptions', '', f'- Withdrawn tasks {k["withdrawn"]}, re-auctioned {k["reauctioned"]}, '
                                       f'stranded boxes {k["stranded"]}', '']
        al = self.alarms()
        lines += ['## Open alarms at the end of the shift', ''] + ([f'- [{a["level"]}] {a["text"]}' for a in al]
                                                                  or ['- none']) + ['']
        ev = [e for e in w.store.events(0, 100000) if e['who'] not in ('wms',) or 'ALARM' in e['what']]
        lines += ['## Operator and ERP actions', ''] + ([f'- t={e["t"]:.0f} s {e["who"]}: {e["what"]}'
                                                        + (f' — {e["why"]}' if e['why'] else '') for e in ev[-200:]]
                                                       or ['- none'])
        return '\n'.join(lines) + '\n'


__all__ = ['Ops', 'math']
