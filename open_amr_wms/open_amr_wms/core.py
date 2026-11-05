"""The WMS core (pure logic, no rclpy): offers tasks to the fleet, never assigns a robot (ADR-002).

Takes over every duty of notes/WMS Interface.md §4 from open_amr_swarm/mission_generator.py, rule for rule (the
mission_generator code it replaces is cited per method), and adds:
  persistence   bins (SKU, box id), tasks and the event log in SQLite (store.py), written through on every change.
  restart       a WMS that restarts resumes from the store and the heartbeats (state, not events; ADR-042):
                  - unclaimed open tasks are re-offered at once with round + 1 (rounds robots decided before stay
                    decided; the new round is a fresh auction);
                  - a claimed task: once heartbeats have been heard for `resume_grace_s`, the robot whose heartbeat
                    shows it keeps it (adopted, any round); if no heartbeat shows it and its box is not on a robot, it
                    is given back (re-offered); if its box was on a robot that no longer shows it, it is STRANDED
                    (a person checks: delivered while the WMS was down, or lost) — never guessed.
                  - promised bins are rebuilt from the open tasks.
  start-up      no work for a station's arm until that station has been heard, or `station_wait_s` passed without
                any station heartbeat at all (runs without station agents). mission_generator offered at once
                (arm_room unlimited before the first heartbeat): 4 inbound tasks for 2 staged boxes (2026-11-05).
  sim time      every timeout (peer, station, reconcile, re-offer) on one clock, `now()` — sim time under
                use_sim_time (ADR-042); mission_generator mixed sim time (offers) and wall time (timeouts).
  fresh msgs    every publish is a new Task built from the record (mission_generator republished one mutated
                object, Pitfalls 2026-11-05).
Orders (milestone 3) plug in through `source`: an object with next_inbound() / next_outbound(bin) giving
(mission_id, priority, sku, box_id); the default generates work as mission_generator does.
"""
import itertools
import math
import random

from builtin_interfaces.msg import Time
from open_amr_msgs.msg import Claim, Inventory, OperatorCommand, RobotState, StationState, Task, Transfer

from open_amr_swarm.liveness import LivenessParams, ghost_hold

from .store import Store

OPEN, CLAIMED, DONE, WITHDRAWN, STRANDED = 'OFFERED', 'CLAIMED', 'DONE', 'WITHDRAWN', 'STRANDED'
KINDS = ('inbound', 'outbound', 'goto')


def to_time(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class Generated:
    """Work source without orders: inbound boxes with a SKU from the receiving pallets, outbound of any full bin
    (mission_generator's behaviour, one mission per task)."""

    def __init__(self, rng, skus=('SKU-A', 'SKU-B', 'SKU-C', 'SKU-D')):
        self.rng, self.skus, self.box = rng, list(skus), 0

    def next_box(self):
        self.box += 1
        return self.rng.choice(self.skus), f'BX{self.box:06d}'

    def choose(self, wms, inbound, pool, seq):
        """(bin, record fields) for a new task of this kind from the candidate bins `pool`, or None for no work."""
        b = self.rng.choice(pool)
        if inbound:
            sku, box = self.next_box()
        else:
            sku, box = wms.bins[b].get('sku', ''), wms.bins[b].get('box_id', '')
        return b, dict(mission_id=f'm{seq:04d}', priority=1, sku=sku, box_id=box)

    def tick(self, wms):
        """Called every WMS tick (orders: priorities, cut-offs)."""

    def done(self, wms, m):
        """A task finished (orders: line picked, ASN box received)."""

    def withdrawn(self, wms, m):
        """A task was withdrawn unclaimed (orders: the line / ASN box is open again)."""


class Wms:
    def __init__(self, nodes, graph=None, publish=None, now=None, log=None, store=None, mode='mission', open_tasks=8,
                 reoffer_s=4.0, peer_timeout_s=3.0, station_timeout_s=3.0, reconcile_s=3.0, station_wait_s=5.0,
                 resume_grace_s=3.0, seed=7, initial_fill=0.5, source=None):
        self.mode, self.graph = mode, graph
        self.publish = publish or (lambda topic, msg: None)
        self.now = now or (lambda: 0.0)
        self.say = log or (lambda level, text: None)
        self.store = store or Store()
        self.open_target, self.reoffer_s = open_tasks, reoffer_s
        self.peer_timeout, self.station_timeout = peer_timeout_s, station_timeout_s
        self.reconcile_s, self.station_wait, self.resume_grace = reconcile_s, station_wait_s, resume_grace_s
        self.rng = random.Random(seed)
        self.stock = Generated(self.rng)          # ids for the stock found at start (an empty store)
        self.source = source or self.stock
        self.bays = sorted(n for n, v in nodes.items() if v['kind'] == 'aisle_bay')
        self.dock_bays = {v['id']: n for n, v in nodes.items() if v['kind'] == 'bay'}
        self.node_id = {n: v['id'] for n, v in nodes.items()}
        self.robots, self.robot_rx, self.released, self.mismatch = {}, {}, {}, {}
        self.stations, self.stale_stations = {}, set()
        self.t0, self.cancelled = None, 0
        self.shares, self.dock = {}, {}
        # ---- state from the store
        stored = self.store.bins()
        if stored:
            self.bins = stored
        else:
            self.bins = {}
            for bay in self.bays:
                for side in ('W', 'E'):
                    b = f'{bay}:{side}'
                    full = self.rng.random() < initial_fill
                    sku, box = self.stock.next_box() if full else ('', '')
                    self.bins[b] = dict(full=full, sku=sku, box_id=box)
                    self.store.put_bin(b, full, sku, box)
        self.seq = self.store.counter('seq')
        self.meta = {}
        self.resuming = set()
        for tid, (m, state) in self.store.tasks().items():
            m['last_offer'] = -math.inf          # re-offer unclaimed ones at once (round + 1)
            self.meta[tid] = m
            if state in (OPEN, CLAIMED) or (state == STRANDED and m['done'] is None):
                if state == CLAIMED:
                    self.resuming.add(tid)
        self.restarted = bool(self.meta)
        self.promised = {m['bin'] for m in self.meta.values() if m['done'] is None and m['bin']}
        self.first_hb = None
        if self.restarted:
            self.event('wms', 'restarted', f'{len(self.open_ids())} open tasks, {len(self.resuming)} claimed to check')

    # ------------------------------------------------------------------ records
    def state_of(self, m):
        if m['cancelled']:
            return WITHDRAWN
        if m['done'] is not None:
            return DONE
        if m['stranded']:
            return STRANDED
        return CLAIMED if m['claimed'] is not None else OPEN

    def save(self, tid):
        self.store.put_task(tid, {k: v for k, v in self.meta[tid].items() if k != 'last_offer' or math.isfinite(v)},
                            self.state_of(self.meta[tid]))

    def set_bin(self, b, full, sku='', box_id=''):
        self.bins[b] = dict(full=full, sku=sku, box_id=box_id)
        self.store.put_bin(b, full, sku, box_id, self.now())

    def event(self, who, what, why='', ref='', level='info'):
        self.store.event(self.now(), who, what, why, ref)
        self.say(level, f'{who}: {what}' + (f' — {why}' if why else ''))

    def open_ids(self):
        return [k for k, m in self.meta.items() if m['done'] is None and not m['stranded']]

    def task_msg(self, m):
        return Task(stamp=to_time(m['last_offer'] if math.isfinite(m['last_offer']) else self.now()),
                    task_id=m['task_id'], mission_id=m['mission_id'], type=m['type'], origin_id=m['origin'],
                    dest_id=m['dest'], bin_id=m['bin'] or '', priority=m['priority'], round=m['round'],
                    deadline=to_time(m.get('deadline_s', 0.0)))

    # ------------------------------------------------------------------ offering (mission_generator.new_task/pool)
    def new_task(self, inbound=None):
        self.seq += 1
        tid = f't{self.seq:04d}'
        if self.mode == 'goto_random':
            kind, binid, typ, origin, dest = 'goto', None, Task.GOTO, self.rng.choice(self.bays), ''
            info = dict(mission_id=f'm{self.seq:04d}', priority=1, sku='', box_id='')
        else:
            forced = inbound is not None
            inbound = self.seq % 2 == 1 if not forced else inbound
            pool = self.pool(inbound)
            if not pool and not forced:
                inbound = not inbound
                pool = self.pool(inbound)
            if not pool:
                self.seq -= 1
                return None
            got = self.source.choose(self, inbound, sorted(pool), self.seq)
            if got is None and not forced:
                inbound = not inbound
                pool = self.pool(inbound)
                got = self.source.choose(self, inbound, sorted(pool), self.seq) if pool else None
            if got is None:
                self.seq -= 1
                return None
            binid, info = got
            bay = binid.split(':')[0]
            if inbound:
                kind, typ, origin, dest = 'inbound', Task.FETCH_FROM_STATION, 'receiving_bay', bay
            else:
                kind, typ, origin, dest = 'outbound', Task.FETCH_FROM_BIN, bay, 'outbound_bay'
            self.promised.add(binid)
        self.store.set_counter('seq', self.seq)
        self.meta[tid] = dict(task_id=tid, kind=kind, bin=binid, type=typ, origin=origin, dest=dest, round=0,
                              offered=self.now(), claimed=None, done=None, robot='', at_arm=False, releases=0,
                              on_robot=False, stranded=False, reclaims=0, cancelled=False, last_offer=self.now(),
                              **info)
        self.save(tid)
        return tid

    def pool(self, inbound):
        bay = 'receiving_bay' if inbound else 'outbound_bay'
        blocked = self.blocked_nodes()
        if self.arm_room(bay, 'inbound' if inbound else 'outbound') <= 0 or self.node_id.get(bay) in blocked:
            return []
        return [b for b, r in self.bins.items() if r['full'] != inbound and b not in self.promised
                and self.node_id.get(b.split(':')[0]) not in blocked]

    def offer(self, tid):
        m = self.meta[tid]
        m['last_offer'] = self.now()
        self.publish('/swarm/tasks', self.task_msg(m))

    # ------------------------------------------------------------------ silent robots (duty 5)
    def silent(self, rid, t=None):
        t = self.now() if t is None else t
        return t - self.robot_rx.get(rid, t) >= self.peer_timeout

    def blocked_nodes(self):
        out = set()
        for rid, s in self.robots.items():
            if not self.silent(rid):
                continue
            if rid in self.released and self.released[rid] >= stamp_s(s.stamp):
                continue
            if self.graph is not None:
                out.update(ghost_hold(self.graph, list(s.reserved_nodes), list(s.route), s.pose.x, s.pose.y,
                                      LivenessParams().ghost_extra_m))
            else:
                out.add(s.last_node)
                out.update(s.reserved_nodes)
        return out | self.graph.cut_off(out) if out and self.graph is not None else out

    def withdraw_blocked(self, blocked):
        for tid, m in self.meta.items():
            if m['claimed'] is not None or m['done'] is not None or m['stranded']:
                continue
            hit = [n for n in (m['origin'], m['dest']) if n and self.node_id.get(n) in blocked]
            if not hit:
                continue
            m['done'], m['cancelled'] = self.now(), True
            self.cancelled += 1
            if m['bin']:
                self.promised.discard(m['bin'])
            self.save(tid)
            self.source.withdrawn(self, m)
            self.publish('/swarm/claims', Claim(stamp=to_time(self.now()), resource_type=Claim.TASK, resource_id=tid,
                                                robot_id='', action=Claim.CANCEL, round=m['round'], by='wms'))
            self.event('wms', f'withdrew {tid}', f'{hit[0]} is blocked by a silent robot', tid, 'warn')

    def retarget_blocked(self, blocked):
        for tid, m in self.meta.items():
            if m['kind'] != 'inbound' or m['done'] is not None or not m['on_robot'] or not m['robot'] or \
                    self.node_id.get(m['dest']) not in blocked or self.silent(m['robot']):
                continue
            free = [b for b, r in self.bins.items() if not r['full'] and b not in self.promised
                    and self.node_id.get(b.split(':')[0]) not in blocked]
            if not free:
                continue
            old, new = m['bin'], self.rng.choice(free)
            self.promised.discard(old); self.promised.add(new)
            m['bin'], m['dest'] = new, new.split(':')[0]
            m['last_offer'] = self.now()
            self.save(tid)
            self.publish('/swarm/tasks', self.task_msg(m))
            self.event('wms', f're-targeted {tid}', f'{m["robot"]} carries its box; {old} is cut off -> {new}', tid,
                       'warn')

    def check_stranded(self):
        silent_s = LivenessParams().reclaim_silent_s
        t = self.now()
        for tid, m in self.meta.items():
            if m['done'] is not None or m['stranded'] or not m['robot'] or not m['on_robot']:
                continue
            rx = self.robot_rx.get(m['robot'])
            if rx is not None and t - rx >= silent_s:
                self.strand(tid, f'{m["robot"]} silent for {t - rx:.0f} s with the box on its deck')

    def strand(self, tid, why):
        m = self.meta[tid]
        m['stranded'] = True
        self.save(tid)
        self.event('wms', f'ALARM {tid}: box needs a person', why + ' — recover robot and box (task stays with it)',
                   tid, 'error')

    # ------------------------------------------------------------------ stations (duty 3)
    def station(self, bay):
        m = self.stations.get(bay)
        return m[0] if m and self.now() - m[1] < self.station_timeout else None

    def watch_stations(self):
        for bay, (_, rx) in self.stations.items():
            age = self.now() - rx
            if age >= self.station_timeout and bay not in self.stale_stations:
                self.stale_stations.add(bay)
                self.event('wms', f'no heartbeat from the station at {bay} for {age:.0f} s',
                           'offering no work for it until it is heard again', bay, 'warn')
            elif age < self.station_timeout and bay in self.stale_stations:
                self.stale_stations.discard(bay)
                self.event('wms', f'station at {bay} heard again', ref=bay)

    def arm_room(self, bay, kind):
        if bay not in self.stations:
            # never heard: wait for it, unless no station has been heard at all for station_wait_s (no station agents)
            if self.stations or self.t0 is None or self.now() - self.t0 < self.station_wait:
                return 0
            return 1 << 30
        st = self.station(bay)
        if st is None or st.state == StationState.FAULT:
            return 0
        pending = sum(1 for m in self.meta.values() if m['kind'] == kind and m['done'] is None and not m['at_arm']
                      and not m['stranded'])
        return st.boxes_available - pending

    def offerable(self, m):
        for n in (m['origin'], m['dest']):
            if n in self.dock_bays.values():
                if n not in self.stations and not self.stations and self.t0 is not None and \
                        self.now() - self.t0 >= self.station_wait:
                    continue                     # no station agents in this run
                st = self.station(n)
                if st is None or st.state == StationState.FAULT:
                    return False
        return True

    # ------------------------------------------------------------------ the 0.5 s tick
    def tick(self):
        now = self.now()
        if self.t0 is None:
            if now <= 0:
                return
            self.t0 = now
        self.watch_stations()
        blocked = self.blocked_nodes()
        if blocked:
            self.withdraw_blocked(blocked)
            self.retarget_blocked(blocked)
        self.check_stranded()
        self.resume_check()
        self.source.tick(self)
        open_ = self.open_ids()
        while len(open_) < self.open_target:
            tid = self.new_task()
            if tid is None:
                break
            self.offer(tid); open_.append(tid)
        if self.mode == 'mission':
            share = (self.open_target + 1) // 2          # mission_generator tick: an arm back gets its share at once
            for inbound in (True, False):
                kind = 'inbound' if inbound else 'outbound'
                while sum(1 for k in open_ if self.meta[k]['kind'] == kind) < share:
                    tid = self.new_task(inbound)
                    if tid is None:
                        break
                    self.offer(tid); open_.append(tid)
        for tid, m in self.meta.items():
            if m['claimed'] is None and m['done'] is None and not m['stranded'] and \
                    now - m['last_offer'] > self.reoffer_s and self.offerable(m):
                m['round'] += 1
                self.save(tid)
                self.offer(tid)

    def resume_check(self):
        """After a restart: settle the claimed tasks from the heartbeats (module doc)."""
        if not self.resuming or self.first_hb is None or self.now() - self.first_hb < self.resume_grace:
            return
        shown = {s.task_id: s.robot_id for s in self.robots.values() if s.task_id and not self.silent(s.robot_id)}
        for tid in sorted(self.resuming):
            m = self.meta[tid]
            if m['done'] is not None:
                continue
            if tid in shown:
                m['robot'] = shown[tid]
                self.event('wms', f'{tid}: {shown[tid]} is on it', 'restart: learned from its heartbeat', tid)
            elif m['robot'] and self.silent(m['robot']) and m['robot'] in self.robots:
                pass                                  # its robot is silent: the silent-robot rules decide
            elif m['on_robot']:
                self.strand(tid, f'restart: its box was on {m["robot"]}, whose heartbeat no longer shows the task '
                                 f'(delivered while the WMS was down, or lost)')
            else:
                m['claimed'], m['robot'] = None, ''
                m['releases'] += 1
                m['last_offer'] = -math.inf
                self.event('wms', f'{tid} re-offered', 'restart: no heartbeat shows it', tid)
            self.save(tid)
        self.resuming.clear()

    # ------------------------------------------------------------------ inputs
    def on_claim(self, c):
        m = self.meta.get(c.resource_id)
        if c.resource_type != Claim.TASK or m is None:
            return
        tid = c.resource_id
        if c.action == Claim.CLAIM and m['claimed'] is None and m['done'] is None:
            m['claimed'], m['robot'] = self.now(), c.robot_id
            self.resuming.discard(tid)
            self.save(tid)
        elif c.action == Claim.RELEASE and m['done'] is None and m['robot'] == c.robot_id:
            by_peer = bool(c.by) and c.by != c.robot_id
            if m['on_robot']:
                self.strand(tid, f'{c.robot_id} lost with the box on its deck (released by {c.by or c.robot_id}); '
                                 f'not re-offered')
                return
            m['claimed'], m['robot'] = None, ''
            m['releases'] += 1
            m['reclaims'] += by_peer
            self.save(tid)
            self.event(c.by or c.robot_id, f'{c.robot_id} gave {tid} back',
                       f'announced by {c.by}: silent robot' if by_peer else 're-offered', tid)
        elif c.action == Claim.DONE and (m['done'] is None or m['cancelled']):
            if m['stranded']:
                self.event(c.robot_id, f'{tid}: stranded box delivered after all', ref=tid)
                m['stranded'] = False
            if m['cancelled']:
                m['cancelled'] = False
                self.cancelled -= 1
            m['done'] = self.now()
            m['robot'] = m['robot'] or c.robot_id
            if m['claimed'] is None:
                m['claimed'] = m['done']
            if m['bin']:
                if m['kind'] == 'inbound':
                    self.set_bin(m['bin'], True, m.get('sku', ''), m.get('box_id', ''))
                else:
                    self.set_bin(m['bin'], False)
                self.promised.discard(m['bin'])
                self.publish_inventory()
            self.save(tid)
            self.on_done(tid, m)

    def on_done(self, tid, m):
        self.source.done(self, m)

    def withdraw(self, tid, why, who='wms'):
        """Withdraw an unclaimed open task (Claim.CANCEL, by='wms'): its bin is free again."""
        m = self.meta[tid]
        if m['claimed'] is not None or m['done'] is not None or m['stranded']:
            return False
        m['done'], m['cancelled'] = self.now(), True
        self.cancelled += 1
        if m['bin']:
            self.promised.discard(m['bin'])
        self.save(tid)
        self.source.withdrawn(self, m)
        self.publish('/swarm/claims', Claim(stamp=to_time(self.now()), resource_type=Claim.TASK, resource_id=tid,
                                            robot_id='', action=Claim.CANCEL, round=m['round'], by='wms'))
        self.event(who, f'withdrew {tid}', why, tid, 'warn')
        return True

    def on_transfer(self, x):
        m = self.meta.get(x.task_id)
        if m is None:
            return
        if x.kind in (Transfer.PALLET_TO_ROBOT, Transfer.ROBOT_TO_PALLET):
            m['at_arm'] = True
        if x.kind in (Transfer.PALLET_TO_ROBOT, Transfer.BIN_TO_ROBOT):
            m['on_robot'] = True
        self.save(x.task_id)

    def on_station(self, s):
        self.stations[s.bay_node] = (s, self.now())
        if self.t0 is not None:
            d = self.dock.setdefault(s.bay_node, dict(idle=0.0, busy=0.0, fault=0.0, starved=0.0))
            d[{StationState.BUSY: 'busy', StationState.FAULT: 'fault', StationState.STARVED: 'starved'}.get(
                s.state, 'idle')] += 0.2

    def on_operator(self, m):
        if m.command == OperatorCommand.RELEASE_ROBOT:
            self.released[m.target] = stamp_s(m.stamp)
            self.event(m.issued_by or 'operator', f'released {m.target}', m.reason or 'its nodes are free for work',
                       m.target, 'warn')

    def on_state(self, s):
        self.robots[s.robot_id] = s
        self.robot_rx[s.robot_id] = self.now()
        if self.first_hb is None:
            self.first_hb = self.now()
        self.reconcile(s)
        sh = self.shares.setdefault(s.robot_id, dict(driving=0.0, yielding=0.0, station=0.0, charging=0.0, idle=0.0))
        key = {RobotState.YIELDING: 'yielding', RobotState.ISOLATED: 'yielding', RobotState.AT_STATION: 'station',
               RobotState.IDLE: 'idle', RobotState.CHARGING: 'charging', RobotState.TO_CHARGE: 'charging'}.get(
            s.status, 'driving')
        if s.status == RobotState.IDLE and s.speed > 0.05:
            key = 'driving'
        sh[key] += 0.2

    def reconcile(self, s):
        """Duty 7 (mission_generator.reconcile)."""
        m = self.meta.get(s.task_id)
        key = (s.robot_id, s.task_id)
        if m is not None and m['claimed'] is None and m['done'] is None and not m['cancelled'] and \
                (s.task_round == m['round'] or s.task_id in self.resuming):
            m['claimed'], m['robot'] = self.now(), s.robot_id
            self.resuming.discard(s.task_id)
            self.save(s.task_id)
            self.event('wms', f'{s.robot_id} holds {s.task_id}', 'claim event lost; learned from its heartbeat',
                       s.task_id)
        stale = m is not None and not m['stranded'] and (m['cancelled'] or m['robot'] != s.robot_id)
        if not stale:
            self.mismatch.pop(key, None)
            return
        t = self.now()
        first = self.mismatch.setdefault(key, t)
        if t - first < self.reconcile_s:
            return
        self.mismatch[key] = t
        gone = m['cancelled'] or m['done'] is not None
        self.publish('/swarm/claims', Claim(stamp=to_time(t), resource_type=Claim.TASK, resource_id=s.task_id,
                                            robot_id=s.robot_id, round=s.task_round, by='wms',
                                            action=Claim.CANCEL if gone else Claim.RELEASE))
        self.event('wms', f'{s.robot_id} is still on {s.task_id}',
                   f'it was {"withdrawn or done" if gone else "re-auctioned"} while out of contact; telling it again',
                   s.task_id, 'warn')

    def publish_inventory(self):
        self.publish('/wms/inventory', Inventory(stamp=to_time(self.now()),
                                                  occupied_bins=sorted(b for b, r in self.bins.items() if r['full'])))

    # ------------------------------------------------------------------ metrics
    def done_tasks(self):
        return [m for m in self.meta.values() if m['done'] is not None and not m['cancelled']]

    def metrics(self):
        el = max(self.now() - (self.t0 or 0.0), 1e-6)
        done = self.done_tasks()
        by = {k: sum(1 for m in done if m['kind'] == k) for k in KINDS}
        lat = sorted(m['done'] - m['offered'] for m in done)
        tot = {k: sum(sh[k] for sh in self.shares.values()) for k in ('driving', 'yielding', 'station', 'charging',
                                                                       'idle')}
        T = max(sum(tot.values()), 1e-6)
        return dict(t=el, done=len(done), by_kind=by, rate_h=len(done) / el * 3600,
                    latency_mean=sum(lat) / max(len(lat), 1), latency_p95=lat[int(0.95 * (len(lat) - 1))] if lat else 0,
                    shares={k: v / T for k, v in tot.items()}, withdrawn=self.cancelled,
                    stranded=sum(1 for m in self.meta.values() if m['stranded'] and m['done'] is None),
                    reauctioned=sum(m['reclaims'] for m in self.meta.values()))

    def report_line(self, final=False):
        x = self.metrics()
        return (f"[wms{' FINAL' if final else ''}] t={x['t']:.0f}s done={x['done']} "
                + ' '.join(f'{k}={v}' for k, v in x['by_kind'].items() if v)
                + f" rate={x['rate_h']:.0f}/h latency mean={x['latency_mean']:.0f}s p95={x['latency_p95']:.0f}s | "
                + ' '.join(f'{k} {100 * v:.0f}%' for k, v in x['shares'].items())
                + f" | withdrawn {x['withdrawn']} stranded {x['stranded']} re-auctioned {x['reauctioned']}")


__all__ = ['Wms', 'Generated', 'itertools']
