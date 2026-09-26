"""mission_generator: warehouse-management stand-in (environment side, never assigns robots).

Offers tasks on /swarm/tasks and learns the outcome from /swarm/claims; the swarm decides who does what.
  mode=mission      inbound (receiving bay -> free bin: FETCH_FROM_STATION) and outbound (occupied bin ->
                    outbound bay: FETCH_FROM_BIN) run concurrently; a bin registry makes sure a bin is only ever
                    promised to one task (Mission Design §8).
  mode=goto_random  plain GOTO tasks to random aisle bays: a traffic stress test.
Unclaimed tasks are re-offered (round + 1) until someone takes them; a task a robot gives back (RELEASE, its arm out
of service) is re-offered too. Arm stations (station_agent heartbeats on /swarm/stations) gate the offers like a WMS
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
from open_amr_msgs.msg import Claim, Inventory, RobotState, StationState, Task, Transfer

from .agent import EVENT_QOS, STATE_QOS, stamp_s


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
        self.stations = {}               # bay node name -> (StationState, receive wall time)
        self.station_timeout = dp('station_timeout_s', 3.0).value
        self.dock = {}                   # bay name -> dict(state time shares, dwell list)
        self.bay_since = {}              # robot -> (bay name, time it started standing there)
        # bin registry: each aisle bay node serves a west and an east bin
        self.bins = {f'{bay}:{side}': self.rng.random() < fill for bay in self.bays for side in ('W', 'E')}
        self.promised = set()
        self.pub_task = self.create_publisher(Task, '/swarm/tasks', EVENT_QOS)
        self.pub_inv = self.create_publisher(Inventory, '/wms/inventory', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(StationState, '/swarm/stations', self.on_station, STATE_QOS)
        self.create_subscription(Transfer, '/swarm/transfers', self.on_transfer, EVENT_QOS)
        self.create_subscription(Claim, '/swarm/claims', self.on_claim, EVENT_QOS)
        self.create_subscription(RobotState, '/swarm/state', self.on_state, STATE_QOS)
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
                                    at_arm=False, releases=0)
        return t

    def pool(self, inbound):
        """Bins a new inbound (empty bins) or outbound (full bins) task could use, [] if its arm can't take more work."""
        if self.arm_room('receiving_bay' if inbound else 'outbound_bay', 'inbound' if inbound else 'outbound') <= 0:
            return []
        return [b for b, full in self.bins.items() if full != inbound and b not in self.promised]

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
        open_ = [k for k, m in self.meta.items() if m['done'] is None]
        while len(open_) < self.open_target:
            t = self.new_task()
            if t is None:
                break
            self.offer(t); open_.append(t.task_id)
        for tid, t in self.tasks.items():
            m = self.meta[tid]
            if m['claimed'] is None and m['done'] is None and now - stamp_s(t.stamp) > self.reoffer_s and self.offerable(t):
                t.round += 1
                self.offer(t)

    def on_claim(self, c):
        m = self.meta.get(c.resource_id)
        if c.resource_type != Claim.TASK or m is None:
            return
        if c.action == Claim.CLAIM and m['claimed'] is None:
            m['claimed'], m['robot'] = self.now(), c.robot_id
        elif c.action == Claim.RELEASE and m['done'] is None and m['robot'] == c.robot_id:
            m['claimed'], m['robot'] = None, ''
            m['releases'] += 1
            self.get_logger().info(f'{c.robot_id} gave {c.resource_id} back; re-offered when its arm is available')
        elif c.action == Claim.DONE and m['done'] is None:
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
        dt = 0.2
        sh = self.shares.setdefault(s.robot_id, dict(driving=0.0, yielding=0.0, station=0.0, charging=0.0, idle=0.0, dist=0.0))
        key = {RobotState.YIELDING: 'yielding', RobotState.AT_STATION: 'station', RobotState.IDLE: 'idle',
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
        done = [m for m in self.meta.values() if m['done'] is not None]
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
                + f"ran empty {len(self.empty)}")
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
