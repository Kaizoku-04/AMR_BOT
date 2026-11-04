"""One mock robot: the swarm agent's message-level behaviour without Nav2, TF or a simulator (pure logic, no rclpy).

What it shares with the real agent (open_amr_swarm), by import, not by copy:
  auction.winner / task_order / closed   who wins a closed round and which task is considered first
  energy.step_soc, task_need, can_take, bid_penalty_s, pick_charger   battery, energy-aware bids, charger choice
  liveness.outranks, announcer, reclaimable, ghost_hold   duplicate claims, re-auction of a silent peer's task
  lane_graph.LaneGraph   the real lane graph (travel lengths, cut-off nodes behind a silent robot)
What it models instead of running:
  driving      edge by edge along the shortest lane route, at the Nav2 speed limit of the edge's zone (vx_max 1.0 m/s
               x the agent's zone %: street 100, aisle 50, dock/queue/charger/parking 35), divided by an efficiency
               factor calibrated against the Isaac truth baseline (Results c50_truth, see MockTiming).
  traffic      physical: a robot doesn't enter a node another robot stands on or is driving onto (it YIELDs); two
               robots waiting on each other for `deadlock_s` pass (the agent's deadlock breaker, modelled by its cost).
               A frozen robot (silent, STUCK) blocks for real; routes avoid the nodes silent peers hold in this robot's
               own view of the fleet, as the agent's route server does.
  docking      fixed times at an arm bay (dock_s in, undock_s out: the bay-exit back-up + turn) and at a charger.
  transfers    bins: the deck's bin_dwell_s; arm bays: the station's heartbeat handshake (served_task), as the agent.
Every input arrives through the bus (lossy if configured); physical blocking is the only shared ground truth.
"""
import heapq
import math
from dataclasses import dataclass, field

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose2D
from open_amr_msgs.msg import Bid, Claim, RobotState, StationState, Task, Transfer

from open_amr_swarm import auction as rules
from open_amr_swarm.energy import (BatteryParams, ChargeParams, ChargerPeer, bid_penalty_s, can_take, pick_charger,
                                   step_soc, task_need)
from open_amr_swarm.liveness import LivenessParams, announcer, ghost_hold, outranks, reclaimable

ZONES = {'street': 100.0, 'aisle': 50.0, 'dock': 35.0, 'charging': 35.0, 'charger': 35.0, 'parking': 35.0,
         'park': 35.0}


@dataclass
class MockTiming:
    """Times and speeds. Sources: Isaac Nav2 params (OpenAMR sim/ros/open_amr_nav2_isaac.launch.py:190 vx_max 1.0),
    agent.py speed zones (zone_*_pct), agent.py bid/dwell defaults, station.py cycle_s, Results gates (marker docking
    12.2 s, bay exit 7.1-7.8 s alone). `efficiency` is the one fitted number: the fraction of the zone speed limit the
    robot averages over an edge (acceleration, turns, the RotationShim, yielding inside a lane)."""
    vmax: float = 1.0                 # m/s, controller vx_max in the Isaac runs
    efficiency: float = 0.8           # fitted: 76.4 +- 3.5/h over 10 seeds vs c50_truth 76/h (vault Results, mock row)
    turn_s: float = 2.0               # extra per node with a heading change > 45 deg (turn in place / slow arc)
    dock_s: float = 12.2              # arm bay: staging node -> docked and checked (Results: marker docking 12.2 s)
    undock_s: float = 7.5             # back 0.20 m + turn out of the bay (Results: bay exit 7.1-7.8 s alone)
    charger_dock_s: float = 8.0       # nose into a charger, contacts close
    bin_dwell_s: float = 4.0          # agent.py bin_dwell_s
    station_dwell_s: float = 8.0      # agent.py station_dwell_s (energy estimate only; the handshake decides)
    nominal_speed: float = 0.5        # agent.py nominal_speed: bid = lane length / this
    bid_window_s: float = 1.5         # agent.py bid_window_s
    station_timeout_s: float = 3.0
    station_giveup_s: float = 30.0
    idle_home_delay_s: float = 3.0
    top_up_below: float = 0.90
    deadlock_s: float = 15.0          # agent.py deadlock_after_s
    reroute_s: float = 10.0           # waiting behind a frozen robot this long: route around it


@dataclass
class Leg:
    target: int
    kind: str                  # 'station' | 'bin' | 'goto' | 'charge' | 'park'
    status: int                # RobotState drive status
    transfer: int = -1         # bins: Transfer kind


def to_time(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int((t - sec) * 1e9))


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


def shortest_path(graph, a, b, blocked=frozenset()):
    """Node list a..b along directed lanes avoiding `blocked` (a and b allowed), None if unreachable."""
    if a == b:
        return [a]
    dist, prev, pq = {a: 0.0}, {}, [(0.0, a)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == b:
            break
        if d > dist[u]:
            continue
        for v in graph.succ.get(u, ()):
            if v in blocked and v != b:
                continue
            nd = d + graph.dist(u, v)
            if nd < dist.get(v, math.inf):
                dist[v], prev[v] = nd, u
                heapq.heappush(pq, (nd, v))
    if b not in dist:
        return None
    path = [b]
    while path[-1] != a:
        path.append(prev[path[-1]])
    return path[::-1]


class MockRobot:
    def __init__(self, rid, fleet, graph, node, soc, bus, timing=None, battery=None, charge=None, liveness=None,
                 chargers=None, world=None):
        self.rid, self.graph, self.bus = rid, graph, bus
        self.fleet = [r for r in fleet if r != rid]
        self.tm = timing or MockTiming()
        self.bp = battery or BatteryParams()
        self.cp = charge or ChargeParams()
        self.lp = liveness or LivenessParams()
        self.chargers = chargers if chargers is not None else graph.of_kind('charger')
        self.parking = graph.of_kind('parking')
        self.world = world                 # the fleet: physical occupancy (who stands where)
        self.drain_per_s = 0.15 * self.bp.time_scale / 3600.0   # swarm.launch drain_prior_per_h 0.15 x scale
        self.last_node, self.battery = node, soc
        x, y = graph.pos[node]
        self.pose = [x, y, 0.0]
        self.speed = 0.0
        self.seq = 0
        self.status, self.phase = RobotState.IDLE, 'idle'
        self.task, self.leg, self.legs, self.loaded, self.docked = None, None, [], False, False
        self.path, self.edge, self.edge_left, self.edge_total = [], None, 0.0, 0.0
        self.timer = 0.0                   # docking / undocking / dwell remaining
        self.wait_s, self.wait_on = 0.0, None
        self.idle_since, self.must_charge, self.charger = 0.0, False, None
        self.stuck_since, self.station_down_since = None, None
        # views from messages
        self.tasks, self.bids, self.my_bids, self.decided = {}, {}, set(), set()
        self.claims, self.claim_round, self.done, self.reclaimed = {}, {}, set(), set()
        self.peers, self.stations, self.released = {}, {}, {}
        # failure injection
        self.link_down_until, self.stuck_until = -1.0, -1.0
        self.next_hb = 0.0
        self.log = []                       # (t, text): what the real agent would log
        for topic, cb in (('/swarm/tasks', self.on_task), ('/swarm/bids', self.on_bid), ('/swarm/claims', self.on_claim),
                          ('/swarm/state', self.on_state), ('/swarm/stations', self.on_station),
                          ('/swarm/operator', self.on_operator)):
            bus.subscribe(topic, rid, cb)
        self.now = 0.0

    # ------------------------------------------------------------------ helpers
    def say(self, text):
        self.log.append((self.now, f'{self.rid}: {text}'))

    def link_down(self):
        return self.now < self.link_down_until

    def frozen(self):
        """Silent (its quorum is gone: it stops at its node) or injected STUCK."""
        return self.link_down() or self.now < self.stuck_until

    def send(self, topic, msg):
        if not self.link_down():
            self.bus.publish(topic, self.rid, msg)

    def held(self):
        """Nodes this robot physically occupies: where it stands and the node it is driving onto."""
        return (self.last_node, self.edge[1]) if self.edge else (self.last_node,)

    def heard(self):
        return [r for r, (_, rx) in self.peers.items() if self.now - rx < self.lp.peer_timeout_s]

    def ghosts(self):
        out = {}
        for r, (s, rx) in self.peers.items():
            if self.now - rx < self.lp.peer_timeout_s:
                continue
            if r in self.released and self.released[r] >= stamp_s(s.stamp):
                continue
            out[r] = (s, ghost_hold(self.graph, list(s.reserved_nodes), list(s.route), s.pose.x, s.pose.y,
                                    self.lp.ghost_extra_m))
        return out

    def ghost_nodes(self):
        return {n for _, held in self.ghosts().values() for n in held}

    def station_state(self, bay):
        m = self.stations.get(bay)
        return m[0] if m and self.now - m[1] < self.tm.station_timeout_s else None

    def station_down(self, bay):
        st = self.station_state(bay)
        return st is None or st.state == StationState.FAULT

    # ------------------------------------------------------------------ inputs (only while the link is up)
    def on_task(self, m):
        if self.link_down():
            return
        cur = self.tasks.get(m.task_id)
        if cur is None or m.round >= cur.round:
            self.tasks[m.task_id] = m
        if self.task is not None and m.task_id == self.task.task_id and m.dest_id and \
                m.dest_id != self.task.dest_id and self.loaded:
            self.say(f'WMS re-targets {m.task_id} to {m.dest_id} (was {self.task.dest_id})')
            self.task = m
            new = self.graph.id(m.dest_id)
            for leg in ([self.leg] if self.leg else []) + self.legs:
                if leg.status == RobotState.TO_DROP and leg.kind == 'bin':
                    leg.target = new
            if self.leg is not None and self.leg.target == new and self.phase == 'driving':
                self.plan()

    def on_bid(self, m):
        if not self.link_down():
            self.bids.setdefault((m.task_id, m.round), {})[m.robot_id] = m.cost

    def on_claim(self, m):
        if self.link_down() or m.resource_type != Claim.TASK:
            return
        tid = m.resource_id
        if m.action == Claim.CLAIM:
            prev = self.claims.get(tid)
            if prev is None or outranks((False, m.round, m.robot_id), (False, self.claim_round.get(tid, 0), prev)):
                self.claims[tid], self.claim_round[tid] = m.robot_id, m.round
            if self.task and self.task.task_id == tid and m.robot_id != self.rid and not self.loaded and \
                    outranks((False, m.round, m.robot_id), (False, self.task.round, self.rid)):
                self.say(f'{m.robot_id} also claimed {tid} and outranks me; dropping it')
                self.abort_task()
        elif m.action in (Claim.DONE, Claim.CANCEL):
            self.done.add(tid)
            if m.action == Claim.CANCEL and self.task is not None and self.task.task_id == tid and not self.loaded:
                self.say(f'the WMS withdrew {tid}; dropping it')
                self.claims.pop(tid, None)
                self.abort_task()
        elif m.action == Claim.RELEASE:
            if m.by and m.by != m.robot_id:
                self.reclaimed.add((tid, m.round))
            if self.claims.get(tid) == m.robot_id:
                del self.claims[tid]
            if m.robot_id == self.rid and m.by and m.by != self.rid and self.task is not None and \
                    self.task.task_id == tid and self.task.round == m.round and not self.loaded:
                self.say(f'{m.by} gave {tid} back on my behalf (I was silent); dropping it')
                self.abort_task()

    def on_state(self, m):
        if self.link_down() or m.robot_id == self.rid:
            return
        if m.robot_id in self.released and stamp_s(m.stamp) > self.released[m.robot_id]:
            del self.released[m.robot_id]
        self.peers[m.robot_id] = (m, self.now)
        if self.task is not None and m.task_id == self.task.task_id and \
                outranks((m.loaded, m.task_round, m.robot_id), (self.loaded, self.task.round, self.rid)):
            self.say(f'{m.robot_id} is also doing {m.task_id} and outranks me; dropping it')
            self.claims[m.task_id], self.claim_round[m.task_id] = m.robot_id, m.task_round
            self.abort_task()

    def on_station(self, m):
        if not self.link_down() and m.bay_node in self.graph.by_name:
            self.stations[self.graph.by_name[m.bay_node]] = (m, self.now)

    def on_operator(self, m):
        from open_amr_msgs.msg import OperatorCommand
        if self.link_down():
            return
        if m.command == OperatorCommand.RELEASE_ROBOT and m.target != self.rid:
            self.released[m.target] = stamp_s(m.stamp)

    # ------------------------------------------------------------------ auction (agent.py auction())
    def available(self):
        return self.task is None and self.phase in ('idle', 'driving', 'charging') and not self.must_charge \
            and not self.frozen()

    def need(self, t):
        stops, dwell = [self.graph.id(t.origin_id)], 0.0
        if t.type != Task.GOTO:
            stops.append(self.graph.id(t.dest_id)); dwell = self.tm.station_dwell_s + self.tm.bin_dwell_s
        return task_need(self.graph, self.start_node(), stops, dwell, self.drain_per_s, self.chargers, self.cp)

    def start_node(self):
        return self.edge[1] if self.edge else self.last_node

    def auction(self):
        now = self.now
        for t in list(self.tasks.values()):
            key = (t.task_id, t.round)
            if key in self.decided or not rules.closed(now, stamp_s(t.stamp), self.tm.bid_window_s) or \
                    t.task_id in self.claims:
                continue
            self.decided.add(key)
            w = rules.winner(self.bids.get(key, {}))
            if w == self.rid and self.task is None and not self.frozen():
                self.claim_and_start(t)
        if not self.available():
            return
        pending = sorted((t for t in self.tasks.values() if t.task_id not in self.claims and t.task_id not in self.done),
                         key=lambda t: rules.task_order(t.priority, stamp_s(t.stamp), t.task_id))
        cut = None
        for t in pending:
            key = (t.task_id, t.round)
            if key in self.my_bids or rules.closed(now, stamp_s(t.stamp), self.tm.bid_window_s):
                continue
            if not can_take(self.battery, self.need(t), self.cp):
                continue
            ends = [self.graph.id(n) for n in (t.origin_id, t.dest_id) if n]
            if any(self.graph.kind.get(n) == 'bay' and self.station_down(n) for n in ends):
                continue
            if cut is None:
                g = self.ghost_nodes()
                cut = (g | self.graph.cut_off(g)) if g else set()
            if any(n in cut for n in ends):
                continue
            length = self.graph.travel(self.start_node(), ends[0])
            if math.isinf(length):
                continue
            cost = length / self.tm.nominal_speed + bid_penalty_s(self.battery, self.cp)
            self.my_bids.add(key)
            b = Bid(stamp=to_time(now), task_id=t.task_id, round=t.round, robot_id=self.rid, cost=float(cost))
            self.on_bid(b)
            self.send('/swarm/bids', b)
            return                          # one bid per tick, as the agent (one route request in flight)

    def claim_and_start(self, t):
        self.claims[t.task_id], self.claim_round[t.task_id] = self.rid, t.round
        self.send('/swarm/claims', Claim(stamp=to_time(self.now), resource_type=Claim.TASK, resource_id=t.task_id,
                                         robot_id=self.rid, action=Claim.CLAIM, round=t.round))
        self.charger = None
        o = self.graph.id(t.origin_id)
        if t.type == Task.GOTO:
            legs = [Leg(o, 'goto', RobotState.TO_PICK)]
        elif t.type == Task.FETCH_FROM_STATION:
            legs = [Leg(o, 'station', RobotState.TO_PICK),
                    Leg(self.graph.id(t.dest_id), 'bin', RobotState.TO_DROP, Transfer.ROBOT_TO_BIN)]
        else:
            legs = [Leg(o, 'bin', RobotState.TO_PICK, Transfer.BIN_TO_ROBOT),
                    Leg(self.graph.id(t.dest_id), 'station', RobotState.TO_DROP)]
        self.say(f'won {t.task_id} ({t.origin_id} -> {t.dest_id or "-"})')
        self.task, self.legs = t, legs
        if self.phase == 'charging':
            self.phase = 'idle'
        self.next_leg(first=True)

    def abort_task(self):
        self.task, self.legs, self.leg, self.loaded, self.stuck_since = None, [], None, False, None
        self.docked = False
        if self.phase in ('docking', 'at_station', 'dwell'):
            self.phase = 'undocking' if self.graph.kind.get(self.last_node) == 'bay' else 'idle'
            self.timer = self.tm.undock_s
        elif self.phase != 'undocking':
            self.phase = 'idle' if not self.edge else 'driving'
            self.path = self.path[:2] if self.edge else []          # finish the edge it is on, then stop
        self.status, self.idle_since = RobotState.IDLE, self.now

    def give_back(self, why):
        t = self.task
        self.say(f'{why}: giving {t.task_id} back')
        self.send('/swarm/claims', Claim(stamp=to_time(self.now), resource_type=Claim.TASK, resource_id=t.task_id,
                                         robot_id=self.rid, action=Claim.RELEASE, round=t.round))
        self.claims.pop(t.task_id, None)
        self.abort_task()

    def finish_task(self):
        t = self.task
        self.send('/swarm/claims', Claim(stamp=to_time(self.now), resource_type=Claim.TASK, resource_id=t.task_id,
                                         robot_id=self.rid, action=Claim.DONE, round=t.round))
        self.done.add(t.task_id)
        self.say(f'done {t.task_id} at {self.graph.name[self.last_node]}')
        self.task, self.leg, self.loaded = None, None, False
        self.phase, self.status, self.idle_since = 'idle', RobotState.IDLE, self.now

    # ------------------------------------------------------------------ legs and driving
    def next_leg(self, first=False):
        if not first and self.leg is not None and self.task is not None and self.task.type != Task.GOTO and \
                self.leg.status == RobotState.TO_PICK:
            self.loaded = True
        if not first and self.leg is not None and self.task is not None and not self.legs:
            self.finish_task()
            return
        if not self.legs:
            self.leg, self.phase, self.status, self.idle_since = None, 'idle', RobotState.IDLE, self.now
            return
        self.leg = self.legs.pop(0)
        self.status = self.leg.status
        self.plan()

    def plan(self):
        start = self.start_node()
        blocked = self.ghost_nodes() - {start, self.leg.target}
        path = shortest_path(self.graph, start, self.leg.target, frozenset(blocked)) or \
            shortest_path(self.graph, start, self.leg.target)
        if path is None:
            self.say(f'no route to {self.graph.name[self.leg.target]}')
            path = [start]
        self.path = ([self.last_node] + path) if self.edge else path
        self.phase = 'driving'
        self.wait_s, self.wait_on = 0.0, None

    def edge_time(self, a, b):
        kind = self.graph.edge_kind.get((a, b), 'street')
        v = self.tm.vmax * ZONES.get(kind, 100.0) / 100.0 * self.tm.efficiency
        t = self.graph.dist(a, b) / max(v, 1e-3)
        i = self.path.index(a) if a in self.path else -1
        if i > 0:
            h0 = self.graph.heading(self.path[i - 1], a)
            h1 = self.graph.heading(a, b)
            if abs(math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))) > math.radians(45):
                t += self.tm.turn_s
        return t

    def drive(self, dt):
        """Advance along self.path ([current node, ..., target])."""
        while dt > 0:
            if self.edge is not None:
                use = min(dt, self.edge_left)
                self.edge_left -= use
                dt -= use
                a, b = self.edge
                f = 1.0 - self.edge_left / max(self.edge_total, 1e-6)
                (x0, y0), (x1, y1) = self.graph.pos[a], self.graph.pos[b]
                self.pose = [x0 + f * (x1 - x0), y0 + f * (y1 - y0), math.atan2(y1 - y0, x1 - x0)]
                self.speed = self.graph.dist(a, b) / max(self.edge_total, 1e-6)
                if self.edge_left > 1e-9:
                    return
                self.last_node, self.edge = b, None
                self.path = self.path[1:]
                continue
            if len(self.path) <= 1:
                self.speed = 0.0
                self.arrive()
                return
            a, b = self.path[0], self.path[1]
            blocker = self.world.occupant(b, self.rid) if self.world is not None else None
            if blocker is not None and not self.may_pass(blocker, dt):
                self.speed = 0.0
                return
            if self.status == RobotState.YIELDING:
                self.status = self.leg.status if self.leg else RobotState.IDLE
            self.wait_s, self.wait_on = 0.0, None
            self.edge, self.edge_total = (a, b), self.edge_time(a, b)
            self.edge_left = self.edge_total

    def may_pass(self, blocker, dt):
        """Blocked by `blocker` (a MockRobot) on the next node: wait (YIELDING); pass a robot that waits on me for
        deadlock_s (the agent breaks the deadlock); route around a frozen one after reroute_s."""
        if self.wait_on != blocker.rid:
            self.wait_on, self.wait_s = blocker.rid, 0.0
        self.wait_s += dt
        if self.status not in (RobotState.STUCK,):
            self.status = RobotState.YIELDING
        if blocker.wait_on == self.rid and self.wait_s > self.tm.deadlock_s:
            return True
        if self.wait_s > self.tm.deadlock_s and blocker.status == RobotState.YIELDING and blocker.wait_s > 0:
            return True                     # a waiting chain: the agent's ageing priority lets the oldest go
        if self.wait_s > self.tm.reroute_s and self.leg is not None and int(self.wait_s / self.tm.reroute_s) != \
                int((self.wait_s - dt) / self.tm.reroute_s):
            path = shortest_path(self.graph, self.last_node, self.leg.target, frozenset(blocker.held()) - {self.leg.target})
            if path and len(path) > 1 and path[1] != self.path[1]:
                self.path = path
        return False

    def arrive(self):
        leg = self.leg
        if leg is None:
            self.phase = 'idle'
            return
        if leg.kind == 'station':
            self.phase, self.timer = 'docking', self.tm.dock_s
        elif leg.kind == 'bin':
            self.phase, self.timer, self.status = 'dwell', self.tm.bin_dwell_s, RobotState.AT_STATION
            if leg.transfer == Transfer.BIN_TO_ROBOT:
                self.loaded = True
            self.send('/swarm/transfers', Transfer(stamp=to_time(self.now), kind=leg.transfer,
                                                   task_id=self.task.task_id if self.task else '', robot_id=self.rid,
                                                   bin_id=self.task.bin_id if self.task else '',
                                                   duration_s=float(self.tm.bin_dwell_s)))
        elif leg.kind == 'charge':
            self.phase, self.timer, self.status = 'docking', self.tm.charger_dock_s, RobotState.TO_CHARGE
        elif leg.kind == 'park':
            self.leg, self.phase, self.status = None, 'idle', RobotState.IDLE
        else:
            self.next_leg()

    # ------------------------------------------------------------------ the 0.1 s step
    def step(self, now, dt):
        self.now = now
        moving = False
        if not self.frozen():
            if not self.link_down():
                self.reclaim_for_silent()
            self.auction()
            moving = self.act(dt)
        elif self.now < self.stuck_until:
            self.status = RobotState.STUCK
        if self.task is not None and not self.loaded and self.status == RobotState.STUCK and not self.link_down():
            self.stuck_since = self.stuck_since if self.stuck_since is not None else now
            if now - self.stuck_since > self.lp.reclaim_stuck_s:
                self.stuck_until = -1.0
                self.give_back(f'no progress for {now - self.stuck_since:.0f} s')
        elif self.status != RobotState.STUCK:
            self.stuck_since = None
        on_charger = self.phase == 'charging'
        self.battery, _ = step_soc(self.battery, dt, self.speed if moving else 0.0, 0.0, on_charger, self.bp)
        if self.battery < self.cp.low and not self.must_charge:
            self.must_charge = True
            self.say(f'battery {100 * self.battery:.0f} % < {100 * self.cp.low:.0f} %: charging after this task')
        if now >= self.next_hb - 1e-6:
            self.next_hb = now + 0.2
            self.heartbeat()

    def act(self, dt):
        if self.phase == 'driving':
            self.check_station()
            if self.phase == 'driving':
                self.drive(dt)
            return self.edge is not None
        if self.phase == 'docking':
            self.timer -= dt
            if self.timer <= 0:
                if self.leg is not None and self.leg.kind == 'charge':
                    self.phase, self.status = 'charging', RobotState.CHARGING
                else:
                    self.phase, self.status, self.docked = 'at_station', RobotState.AT_STATION, True
            return False
        if self.phase == 'at_station':
            st = self.station_state(self.leg.target)
            if st is not None and self.task is not None and st.task_id == self.task.task_id and \
                    st.state == StationState.BUSY and self.leg.status == RobotState.TO_PICK:
                self.loaded = True
            if st is not None and self.task is not None and st.served_task == self.task.task_id:
                self.phase, self.timer, self.docked = 'undocking', self.tm.undock_s, False
                self.status = RobotState.TO_DROP if self.leg.status == RobotState.TO_PICK else self.leg.status
                return False
            self.check_station()
            return False
        if self.phase == 'undocking':
            self.timer -= dt
            if self.timer <= 0:
                self.phase = 'driving' if self.task is not None else 'idle'
                if self.task is not None:
                    self.next_leg()
            return False
        if self.phase == 'dwell':
            self.timer -= dt
            if self.timer <= 0:
                self.next_leg()
            return False
        if self.phase == 'charging':
            target = self.cp.resume if self.must_charge else 1.0
            if self.battery >= target:
                if self.must_charge:
                    self.must_charge = False
                    self.say(f'charged to {100 * self.battery:.0f} %: back to work')
                if self.battery >= 1.0 - 1e-6:
                    self.status = RobotState.IDLE        # full, stays parked on the charger, available
            return False
        if self.phase == 'idle' and self.task is None and now_idle(self) > self.tm.idle_home_delay_s:
            self.go_home()
        return False

    def check_station(self):
        leg = self.leg
        if not (self.task is not None and leg is not None and leg.kind == 'station'
                and leg.status == RobotState.TO_PICK and not self.loaded):
            self.station_down_since = None
            return
        if not self.station_down(leg.target):
            self.station_down_since = None
            return
        if self.station_down_since is None:
            self.station_down_since = self.now
            self.say(f'{self.graph.name[leg.target]} is out of service')
        elif self.now - self.station_down_since > self.tm.station_giveup_s:
            self.station_down_since = None
            self.give_back(f'{self.graph.name[leg.target]} out of service for {self.tm.station_giveup_s:.0f} s')

    def go_home(self):
        """Idle: to a free charger (opportunity charging; must_charge or below top_up_below), else a parking spot."""
        peers = [ChargerPeer(s.robot_id, s.goal_node, list(s.reserved_nodes), s.battery, bool(s.task_id))
                 for r, (s, rx) in self.peers.items() if self.now - rx < self.lp.peer_timeout_s or r in self.ghosts()]
        if self.last_node in self.chargers and self.phase == 'idle':
            spot = None
        elif self.must_charge or self.battery < self.tm.top_up_below:
            spot = pick_charger(self.graph, self.last_node, self.chargers, self.rid, self.battery, peers, None, self.cp)
            kind = 'charge'
        else:
            spot = None
        if spot is None and self.last_node not in self.parking and self.last_node not in self.chargers:
            spot = pick_charger(self.graph, self.last_node, self.parking, self.rid, self.battery, peers, None, self.cp)
            kind = 'park'
        if spot is None:
            self.idle_since = self.now          # look again after the delay
            if self.last_node in self.chargers and self.battery < 1.0:
                self.phase, self.status = 'charging', RobotState.CHARGING
            return
        self.charger = spot if kind == 'charge' else None
        self.leg = Leg(spot, kind, RobotState.TO_CHARGE if kind == 'charge' else RobotState.IDLE)
        self.status = self.leg.status
        self.plan()

    def reclaim_for_silent(self):
        """Liveness rule 4 (agent.py reclaim_for_silent)."""
        for rid, (s, rx) in self.peers.items():
            if self.now - rx < self.lp.peer_timeout_s or not s.task_id or s.task_id in self.done:
                continue
            released = rid in self.released and self.released[rid] >= stamp_s(s.stamp)
            key = (s.task_id, s.task_round)
            if key in self.reclaimed or self.claims.get(s.task_id) not in (None, rid) or \
                    not reclaimable(self.now - rx, released, s.task_id, s.loaded, self.lp):
                continue
            if announcer(self.rid, self.heard()) != self.rid:
                continue
            self.reclaimed.add(key)
            self.claims.pop(s.task_id, None)
            self.say(f'{rid} silent for {self.now - rx:.0f} s: giving its task {s.task_id} back for re-auction')
            self.send('/swarm/claims', Claim(stamp=to_time(self.now), resource_type=Claim.TASK, resource_id=s.task_id,
                                             robot_id=rid, action=Claim.RELEASE, round=s.task_round, by=self.rid))

    def heartbeat(self):
        self.seq += 1
        route = list(self.path[1:]) if self.phase == 'driving' else []
        hb = RobotState(stamp=to_time(self.now), robot_id=self.rid, seq=self.seq, status=self.status,
                        pose=Pose2D(x=float(self.pose[0]), y=float(self.pose[1]), theta=float(self.pose[2])),
                        speed=float(self.speed), battery=float(self.battery),
                        task_id=self.task.task_id if self.task else '', task_round=self.task.round if self.task else 0,
                        loaded=self.loaded, mission_id=self.task.mission_id if self.task else '',
                        last_node=self.last_node, route=route, reserved_nodes=list(dict.fromkeys(self.held())),
                        wait_s=float(self.wait_s), docked=self.docked,
                        goal_node=self.leg.target if self.leg else self.last_node)
        hb.heard_ids = sorted(self.heard())
        hb.heard_seq = [self.peers[r][0].seq for r in hb.heard_ids]
        self.send('/swarm/state', hb)

    # ------------------------------------------------------------------ failure injection
    def go_silent(self, for_s):
        self.link_down_until = self.now + for_s
        self.say(f'TEST swarm link CUT for {for_s:.0f} s')

    def get_stuck(self, for_s):
        self.stuck_until = self.now + for_s
        self.status = RobotState.STUCK
        self.say(f'TEST STUCK for {for_s:.0f} s')

    def unstick(self):
        if self.status == RobotState.STUCK:
            self.status = self.leg.status if self.leg else RobotState.IDLE

    def hand_back(self):
        if self.task is not None and not self.loaded:
            self.give_back('TEST gives its task back')
            return True
        return False


def now_idle(r):
    return r.now - r.idle_since
