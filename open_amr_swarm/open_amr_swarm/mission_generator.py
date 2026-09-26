"""mission_generator: warehouse-management stand-in (environment side, never assigns robots).

Offers tasks on /swarm/tasks and learns the outcome from /swarm/claims; the swarm decides who does what.
  mode=mission      inbound (receiving bay -> free bin: FETCH_FROM_STATION) and outbound (occupied bin ->
                    outbound bay: FETCH_FROM_BIN) run concurrently; a bin registry makes sure a bin is only ever
                    promised to one task (Mission Design §8).
  mode=goto_random  plain GOTO tasks to random aisle bays: a traffic stress test.
Unclaimed tasks are re-offered (round + 1) until someone takes them; a task a robot gives back (RELEASE: its arm out
of service, it is stuck, or — announced by a peer — it went silent) is re-offered too, unless its box is already on
that robot's deck (a pickup transfer was seen): then it is STRANDED and needs a person; the original robot can still
finish it if it comes back. A robot that went silent is taken to still stand where it was last heard (its last node
and reserved nodes, until heard again or released on /swarm/operator): unclaimed tasks whose bin or bay is there are
withdrawn (Claim.CANCEL) and no new work is offered there. A robot that comes back from a network cut still doing a
task that was withdrawn, re-auctioned or done meanwhile (it never heard those events) is told again: its heartbeat
shows the mismatch, and after `reconcile_s` the WMS re-sends the CANCEL / RELEASE (state repairs lost events). Arm stations (station_agent heartbeats on /swarm/stations) gate the offers like a WMS
would: inbound work only for boxes actually staged at receiving, outbound only for free pallet slots, and nothing for
an arm that is out of service (FAULT) — offers resume when it's back. Inventory (occupied bins) on /wms/inventory.

Metrics (logged every `report_s` and at the end; per-task CSV and a 10 s battery timeline in `out_dir`): tasks
done and per hour by type, mean/max latency (offer -> done), robot time shares (driving / yielding / at station /
charging / idle), closest approach between two robots, longest continuous yield (deadlock watch), energy: lowest
state of charge any robot reached, charge sessions, robots that ran empty; docks: boxes in / out per hour, each arm's
time busy / idle / starved / in fault, and bay dwell (robot standing at the arm's bay, mean / max).
"""
import csv
import itertools
import math
import os
import random
import time as wallclock

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import yaml
from open_amr_msgs.msg import Claim, Inventory, OperatorCommand, RobotState, StationState, Task, Transfer

from .agent import EVENT_QOS, OPERATOR_QOS, STATE_QOS, stamp_s
from .lane_graph import LaneGraph
from .liveness import LivenessParams, ghost_hold


class MissionGenerator(Node):
    def __init__(self):
        super().__init__('mission_generator')
        dp = self.declare_parameter
        self.mode = dp('mode', 'mission').value
        nodes = yaml.safe_load(open(dp('graph_nodes', '').value))
        self.open_target = dp('open_tasks', 4).value
        self.run_s = dp('duration_s', 0.0).value
        self.report_s = dp('report_s', 30.0).value
        self.reoffer_s = dp('reoffer_s', 4.0).value
        self.out_dir = os.path.expanduser(dp('out_dir', '/tmp/swarm_metrics').value)
        self.rng = random.Random(dp('seed', 7).value)
        fill = dp('initial_fill', 0.5).value
        self.bays = sorted(n for n, v in nodes.items() if v['kind'] == 'aisle_bay')
        self.dock_bays = {v['id']: n for n, v in nodes.items() if v['kind'] == 'bay'}   # arm bays: node id -> name
        self.node_id = {n: v['id'] for n, v in nodes.items()}
        gpath = dp('graph', '').value
        self.graph = LaneGraph(gpath) if gpath else None   # to find dead ends behind a silent robot
        self.peer_timeout = dp('peer_timeout_s', 3.0).value
        self.robot_rx, self.released, self.cancelled = {}, {}, 0
        self.reconcile_s = dp('reconcile_s', 3.0).value
        self.mismatch = {}               # (robot, task) -> wall time the heartbeat first disagreed with the WMS
        self.stations = {}               # bay node name -> (StationState, receive wall time)
        self.station_timeout = dp('station_timeout_s', 3.0).value
        self.dock = {}                   # bay name -> dict(state time shares, dwell list)
        self.bay_since = {}              # robot -> (bay name, time it started standing there)
        # bin registry: each aisle bay node serves a west and an east bin
        self.bins = {f'{bay}:{side}': self.rng.random() < fill for bay in self.bays for side in ('W', 'E')}
        self.promised = set()
        self.pub_task = self.create_publisher(Task, '/swarm/tasks', EVENT_QOS)
        self.pub_claim = self.create_publisher(Claim, '/swarm/claims', EVENT_QOS)
        self.pub_inv = self.create_publisher(Inventory, '/wms/inventory', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(StationState, '/swarm/stations', self.on_station, STATE_QOS)
        self.create_subscription(Transfer, '/swarm/transfers', self.on_transfer, EVENT_QOS)
        self.create_subscription(Claim, '/swarm/claims', self.on_claim, EVENT_QOS)
        self.create_subscription(RobotState, '/swarm/state', self.on_state, STATE_QOS)
        self.create_subscription(OperatorCommand, '/swarm/operator', self.on_operator, OPERATOR_QOS)
        self.tasks, self.meta, self.seq = {}, {}, 0
        self.robots, self.shares, self.yield_run, self.max_yield = {}, {}, {}, (0.0, '')
        self.closest = (1e9, '')
        self.min_soc, self.sessions, self.empty = (1.0, ''), 0, set()
        self.t0 = None
        os.makedirs(self.out_dir, exist_ok=True)
        self.csv = open(os.path.join(self.out_dir, 'tasks.csv'), 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['task_id', 'type', 'origin', 'dest', 'bin', 'offered_s', 'claimed_s', 'done_s', 'robot', 'rounds',
                              'releases'])
        self.soc_csv = open(os.path.join(self.out_dir, 'battery.csv'), 'w', newline='')
        self.soc_writer = csv.writer(self.soc_csv)
        self.soc_writer.writerow(['t_s', 'robot', 'soc', 'status'])
        self.create_timer(10.0, self.log_battery)
        self.create_timer(0.5, self.tick)
        self.create_timer(self.report_s, self.report)
        self.publish_inventory()
        self.get_logger().info(f'mission generator: mode={self.mode}, {len(self.bays)} aisle bays, '
                               f'{sum(self.bins.values())}/{len(self.bins)} bins occupied')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ offering
    def new_task(self):
        self.seq += 1
        t = Task(task_id=f't{self.seq:04d}', mission_id=f'm{self.seq:04d}', priority=1, round=0)
        if self.mode == 'goto_random':
            t.type, t.origin_id = Task.GOTO, self.rng.choice(self.bays)
            kind, binid = 'goto', None
        else:
            inbound = self.seq % 2 == 1
            pool = self.pool(inbound)
            if not pool:
                inbound = not inbound
                pool = self.pool(inbound)
            if not pool:
                self.seq -= 1
                return None
            binid = self.rng.choice(pool)
            self.promised.add(binid)
            bay = binid.split(':')[0]
            t.bin_id = binid
            if inbound:
                t.type, t.origin_id, t.dest_id, kind = Task.FETCH_FROM_STATION, 'receiving_bay', bay, 'inbound'
            else:
                t.type, t.origin_id, t.dest_id, kind = Task.FETCH_FROM_BIN, bay, 'outbound_bay', 'outbound'
        self.meta[t.task_id] = dict(kind=kind, bin=binid, offered=self.now(), claimed=None, done=None, robot='',
                                    at_arm=False, releases=0, on_robot=False, stranded=False, reclaims=0,
                                    cancelled=False)
        return t

    def pool(self, inbound):
        """Bins a new inbound (empty bins) or outbound (full bins) task could use, [] if its arm can't take more work or
        its bay is blocked by a silent robot."""
        bay = 'receiving_bay' if inbound else 'outbound_bay'
        blocked = self.blocked_nodes()
        if self.arm_room(bay, 'inbound' if inbound else 'outbound') <= 0 or self.node_id.get(bay) in blocked:
            return []
        return [b for b, full in self.bins.items() if full != inbound and b not in self.promised
                and self.node_id.get(b.split(':')[0]) not in blocked]

    def blocked_nodes(self):
        """Nodes where a silent robot may still stand (its last node and reserved nodes, until heard or released), plus
        the dead ends behind them (LaneGraph.trapped)."""
        t, out = wallclock.monotonic(), set()
        for rid, s in self.robots.items():
            if t - self.robot_rx.get(rid, t) < self.peer_timeout:
                continue
            if rid in self.released and self.released[rid] >= stamp_s(s.stamp):
                continue
            if self.graph is not None:                   # the same frozen view the agents use (liveness rule 1)
                out.update(ghost_hold(self.graph, list(s.reserved_nodes), list(s.route), s.pose.x, s.pose.y,
                                      LivenessParams().ghost_extra_m))
            else:
                out.add(s.last_node)
                out.update(s.reserved_nodes)
        return out | self.graph.cut_off(out) if out and self.graph is not None else out

    def on_operator(self, m):
        if m.command == OperatorCommand.RELEASE_ROBOT:
            self.released[m.target] = stamp_s(m.stamp)
            self.get_logger().warn(f'operator {m.issued_by or "?"} released {m.target}: its nodes are free for work again')

    def reconcile(self, s):
        """A robot's heartbeat says it works on a task the WMS withdrew, re-auctioned or saw done (events it missed,
        e.g. during a network cut): after reconcile_s of disagreement, tell it again. A fresh claim whose event hasn't
        arrived yet agrees within that time."""
        m = self.meta.get(s.task_id)
        key = (s.robot_id, s.task_id)
        if m is not None and m['claimed'] is None and m['done'] is None and not m['cancelled'] and \
                s.task_round == self.tasks[s.task_id].round:
            # won in the current offer round, but its CLAIM event never arrived: adopt it (don't take work away)
            m['claimed'], m['robot'] = self.now(), s.robot_id
            self.get_logger().info(f'{s.robot_id} holds {s.task_id} (claim event lost; learned from its heartbeat)')
        stale = m is not None and not m['stranded'] and (m['cancelled'] or m['robot'] != s.robot_id)
        if not stale:
            self.mismatch.pop(key, None)
            return
        t = wallclock.monotonic()
        first = self.mismatch.setdefault(key, t)
        if t - first < self.reconcile_s:
            return
        self.mismatch[key] = t                       # repeat every reconcile_s while it persists
        gone = m['cancelled'] or m['done'] is not None
        self.pub_claim.publish(Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK,
                                     resource_id=s.task_id, robot_id=s.robot_id, round=s.task_round, by='wms',
                                     action=Claim.CANCEL if gone else Claim.RELEASE))
        self.get_logger().warn(f'{s.robot_id} is still on {s.task_id}, which was '
                               f'{"withdrawn or done" if gone else "re-auctioned"} while it was out of contact; telling it again')

    def retarget_blocked(self):
        """A robot carrying an inbound box whose bin is now blocked or cut off by a silent robot: give the box another
        free bin (re-published Task, same id and round; the claimant drives there instead). Outbound boxes can only go
        to the outbound arm: those robots wait parked until the way is clear."""
        blocked = self.blocked_nodes()
        if not blocked:
            return
        for tid, t in self.tasks.items():
            m = self.meta[tid]
            if m['kind'] != 'inbound' or m['done'] is not None or not m['on_robot'] or not m['robot'] or \
                    self.node_id.get(t.dest_id) not in blocked:
                continue
            if wallclock.monotonic() - self.robot_rx.get(m['robot'], 0.0) >= self.peer_timeout:
                continue                             # the carrier itself is silent: stranded, nothing to re-target
            free = [b for b, full in self.bins.items() if not full and b not in self.promised
                    and self.node_id.get(b.split(':')[0]) not in blocked]
            if not free:
                continue
            old = m['bin']
            new = self.rng.choice(free)
            self.promised.discard(old); self.promised.add(new)
            m['bin'] = new
            t.bin_id, t.dest_id = new, new.split(':')[0]
            t.stamp = self.get_clock().now().to_msg()
            self.pub_task.publish(t)
            self.get_logger().warn(f're-targeted {tid} ({m["robot"]}, box on board): {old} is cut off -> {new}')

    def check_stranded(self):
        """A claimant silent for reclaim_silent_s with the box on its deck: the agents leave its task with it (nobody
        else can deliver that box), so the WMS raises the alarm itself."""
        t = wallclock.monotonic()
        silent_s = LivenessParams().reclaim_silent_s
        for tid, m in self.meta.items():
            if m['done'] is not None or m['stranded'] or not m['robot'] or not m['on_robot']:
                continue
            rx = self.robot_rx.get(m['robot'])
            if rx is not None and t - rx >= silent_s:
                m['stranded'] = True
                self.get_logger().error(f'ALARM {tid}: {m["robot"]} silent for {t - rx:.0f} s with the box on its deck '
                                        f'— recover the robot and box by hand (task stays with it)')

    def withdraw_blocked(self):
        """Unclaimed tasks whose bin or bay a silent robot blocks can't be done: withdraw them (Claim.CANCEL), free the
        bin, and let new tasks replace them."""
        blocked = self.blocked_nodes()
        if not blocked:
            return
        for tid, t in self.tasks.items():
            m = self.meta[tid]
            if m['claimed'] is not None or m['done'] is not None or m['stranded']:
                continue
            hit = [n for n in (t.origin_id, t.dest_id) if n and self.node_id.get(n) in blocked]
            if not hit:
                continue
            m['done'], m['cancelled'] = self.now(), True
            self.cancelled += 1
            if m['bin']:
                self.promised.discard(m['bin'])
            self.pub_claim.publish(Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK, resource_id=tid,
                                         robot_id='', action=Claim.CANCEL, round=t.round, by='wms'))
            self.get_logger().warn(f'withdrew {tid}: {hit[0]} is blocked by a silent robot')

    def station(self, bay):
        m = self.stations.get(bay)
        return m[0] if m and wallclock.monotonic() - m[1] < self.station_timeout else None

    def arm_room(self, bay, kind):
        """How many more tasks of `kind` the arm at `bay` can take: its stock / free slots minus the open tasks that
        haven't been served there yet. Unlimited while no station was ever heard (runs without station agents)."""
        if bay not in self.stations:
            return 1 << 30
        st = self.station(bay)
        if st is None or st.state == StationState.FAULT:
            return 0
        pending = sum(1 for m in self.meta.values() if m['kind'] == kind and m['done'] is None and not m['at_arm'])
        return st.boxes_available - pending

    def offerable(self, t):
        """Re-offers wait while the task's arm is out of service."""
        for n in (t.origin_id, t.dest_id):
            if n in self.stations:
                st = self.station(n)
                if st is None or st.state == StationState.FAULT:
                    return False
        return True

    def offer(self, t):
        t.stamp = self.get_clock().now().to_msg()
        self.tasks[t.task_id] = t
        self.pub_task.publish(t)

    def tick(self):
        now = self.now()
        if self.t0 is None:
            if now <= 0:
                return
            self.t0 = now
        if self.run_s and now - self.t0 > self.run_s:
            self.report(final=True)
            raise SystemExit
        self.withdraw_blocked()
        self.retarget_blocked()
        self.check_stranded()
        open_ = [k for k, m in self.meta.items() if m['done'] is None and not m['stranded']]   # stranded: nobody can work it
        while len(open_) < self.open_target:
            t = self.new_task()
            if t is None:
                break
            self.offer(t); open_.append(t.task_id)
        for tid, t in self.tasks.items():
            m = self.meta[tid]
            if m['claimed'] is None and m['done'] is None and not m['stranded'] and \
                    now - stamp_s(t.stamp) > self.reoffer_s and self.offerable(t):
                t.round += 1
                self.offer(t)

    def on_claim(self, c):
        m = self.meta.get(c.resource_id)
        if c.resource_type != Claim.TASK or m is None:
            return
        if c.action == Claim.CLAIM and m['claimed'] is None:
            m['claimed'], m['robot'] = self.now(), c.robot_id
        elif c.action == Claim.RELEASE and m['done'] is None and m['robot'] == c.robot_id:
            by_peer = bool(c.by) and c.by != c.robot_id
            if m['on_robot']:
                m['stranded'] = True
                self.get_logger().error(f'ALARM {c.resource_id}: {c.robot_id} lost with the box on its deck '
                                        f'(released by {c.by or c.robot_id}); not re-offered — recover the box by hand')
                return
            m['claimed'], m['robot'] = None, ''
            m['releases'] += 1
            m['reclaims'] += by_peer
            self.get_logger().info(f'{c.robot_id} gave {c.resource_id} back'
                                   f'{f" (announced by {c.by}: silent robot)" if by_peer else ""}; re-offered')
        elif c.action == Claim.DONE and (m['done'] is None or m['cancelled']):
            if m['stranded']:
                self.get_logger().info(f'{c.resource_id}: stranded box delivered by {c.robot_id} after all')
            if m['cancelled']:                   # claimed in the same instant it was withdrawn, and done anyway
                m['cancelled'] = False
                self.cancelled -= 1
            m['done'] = self.now()
            if m['bin']:
                self.bins[m['bin']] = m['kind'] == 'inbound'
                self.promised.discard(m['bin'])
                self.publish_inventory()
            t = self.tasks[c.resource_id]
            self.writer.writerow([c.resource_id, m['kind'], t.origin_id, t.dest_id, m['bin'] or '', f"{m['offered'] - self.t0:.1f}",
                                  f"{(m['claimed'] or m['done']) - self.t0:.1f}", f"{m['done'] - self.t0:.1f}", c.robot_id, t.round,
                                  m['releases']])
            self.csv.flush()

    def publish_inventory(self):
        self.pub_inv.publish(Inventory(stamp=self.get_clock().now().to_msg(),
                                       occupied_bins=sorted(b for b, full in self.bins.items() if full)))

    def on_station(self, s):
        self.stations[s.bay_node] = (s, wallclock.monotonic())
        if self.t0 is not None:
            d = self.dock.setdefault(s.bay_node, dict(idle=0.0, busy=0.0, fault=0.0, starved=0.0, dwell=[]))
            d[{StationState.BUSY: 'busy', StationState.FAULT: 'fault', StationState.STARVED: 'starved'}.get(s.state, 'idle')] += 0.2

    def on_transfer(self, x):
        m = self.meta.get(x.task_id)
        if m is not None and x.kind in (Transfer.PALLET_TO_ROBOT, Transfer.ROBOT_TO_PALLET):
            m['at_arm'] = True               # the arm's stock / free slots now account for it
        if m is not None and x.kind in (Transfer.PALLET_TO_ROBOT, Transfer.BIN_TO_ROBOT):
            m['on_robot'] = True             # the box left its pallet / bin: the task can't simply be redone

    def log_battery(self):
        if self.t0 is None:
            return
        for rid, r in sorted(self.robots.items()):
            self.soc_writer.writerow([f'{self.now() - self.t0:.0f}', rid, f'{r.battery:.3f}', r.status])
        self.soc_csv.flush()

    # ------------------------------------------------------------------ fleet metrics
    def on_state(self, s):
        prev = self.robots.get(s.robot_id)
        self.robots[s.robot_id] = s
        self.robot_rx[s.robot_id] = wallclock.monotonic()
        self.reconcile(s)
        dt = 0.2
        sh = self.shares.setdefault(s.robot_id, dict(driving=0.0, yielding=0.0, station=0.0, charging=0.0, idle=0.0, dist=0.0))
        key = {RobotState.YIELDING: 'yielding', RobotState.ISOLATED: 'yielding', RobotState.AT_STATION: 'station',
               RobotState.IDLE: 'idle',
               RobotState.CHARGING: 'charging', RobotState.TO_CHARGE: 'charging'}.get(s.status, 'driving')
        if s.status == RobotState.IDLE and s.speed > 0.05:
            key = 'driving'
        sh[key] += dt
        if prev is not None:
            sh['dist'] += math.hypot(s.pose.x - prev.pose.x, s.pose.y - prev.pose.y)
            if s.status == RobotState.CHARGING and prev.status != RobotState.CHARGING:
                self.sessions += 1
        if self.t0 is not None and s.battery < self.min_soc[0]:
            self.min_soc = (s.battery, s.robot_id)
        if s.status == RobotState.STUCK and s.battery < 0.05:
            self.empty.add(s.robot_id)
        at_bay = self.dock_bays.get(s.last_node) if s.status == RobotState.AT_STATION else None
        cur = self.bay_since.get(s.robot_id)
        if cur and cur[0] != at_bay:
            if self.t0 is not None:
                self.dock.setdefault(cur[0], dict(idle=0.0, busy=0.0, fault=0.0, starved=0.0, dwell=[]))['dwell'].append(self.now() - cur[1])
            del self.bay_since[s.robot_id]
        if at_bay and s.robot_id not in self.bay_since:
            self.bay_since[s.robot_id] = (at_bay, self.now())
        run = self.yield_run.get(s.robot_id, 0.0) + dt if s.status == RobotState.YIELDING else 0.0
        self.yield_run[s.robot_id] = run
        if run > self.max_yield[0]:
            self.max_yield = (run, s.robot_id)
        for a, b in itertools.combinations(sorted(self.robots), 2):
            ra, rb = self.robots[a], self.robots[b]
            d = math.hypot(ra.pose.x - rb.pose.x, ra.pose.y - rb.pose.y)
            if d < self.closest[0]:
                self.closest = (d, f'{a}-{b}')

    def report(self, final=False):
        if self.t0 is None:
            return
        el = max(self.now() - self.t0, 1e-6)
        done = [m for m in self.meta.values() if m['done'] is not None and not m['cancelled']]
        by = {k: sum(1 for m in done if m['kind'] == k) for k in ('inbound', 'outbound', 'goto')}
        lat = [m['done'] - m['offered'] for m in done]
        tot = {k: sum(sh[k] for sh in self.shares.values()) for k in ('driving', 'yielding', 'station', 'charging', 'idle')}
        T = max(sum(tot.values()), 1e-6)
        line = (f"[metrics{' FINAL' if final else ''}] t={el:.0f}s done={len(done)} "
                + ' '.join(f'{k}={v}' for k, v in by.items() if v)
                + f" rate={len(done) / el * 3600:.0f}/h latency mean={sum(lat) / max(len(lat), 1):.0f}s "
                + f"max={max(lat, default=0):.0f}s | fleet time: driving {100 * tot['driving'] / T:.0f}% "
                + f"yielding {100 * tot['yielding'] / T:.0f}% station {100 * tot['station'] / T:.0f}% "
                + f"charging {100 * tot['charging'] / T:.0f}% idle {100 * tot['idle'] / T:.0f}% | closest approach {self.closest[0]:.2f} m ({self.closest[1]}) "
                + f"| longest yield {self.max_yield[0]:.0f}s ({self.max_yield[1]}) "
                + f"| distance {sum(sh['dist'] for sh in self.shares.values()):.0f} m "
                + f"| battery now {' '.join(f'{100 * r.battery:.0f}' for _, r in sorted(self.robots.items()))} % "
                + f"min {100 * self.min_soc[0]:.0f}% ({self.min_soc[1]}) charge sessions {self.sessions} "
                + f"ran empty {len(self.empty)} "
                + f"| re-auctioned {sum(m['reclaims'] for m in self.meta.values())} "
                + f"stranded {sum(m['stranded'] and m['done'] is None for m in self.meta.values())} withdrawn {self.cancelled}")
        self.get_logger().info(line)
        docks = []
        for bay in sorted(self.dock):
            d = self.dock[bay]
            T = max(d['idle'] + d['busy'] + d['fault'] + d['starved'], 1e-6)
            st = self.station(bay)
            dw = d['dwell']
            docks.append(f"{bay.replace('_bay', '')}: busy {100 * d['busy'] / T:.0f}% idle {100 * d['idle'] / T:.0f}% "
                         f"starved {100 * d['starved'] / T:.0f}% fault {100 * d['fault'] / T:.0f}% "
                         f"bay dwell mean {sum(dw) / max(len(dw), 1):.0f}s max {max(dw, default=0):.0f}s "
                         f"({len(dw)} visits){f', pallets {list(st.pallets)}' if st else ''}")
        if docks or by['inbound'] or by['outbound']:
            self.get_logger().info(f"[metrics{' FINAL' if final else ''} docks] boxes in {by['inbound']} "
                                   f"({by['inbound'] / el * 3600:.0f}/h) out {by['outbound']} ({by['outbound'] / el * 3600:.0f}/h) | "
                                   + ' | '.join(docks))


def main():
    rclpy.init()
    node = MissionGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.csv.close(); node.soc_csv.close()
