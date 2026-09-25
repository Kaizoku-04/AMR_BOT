"""mission_generator: warehouse-management stand-in (environment side, never assigns robots).

Offers tasks on /swarm/tasks and learns the outcome from /swarm/claims; the swarm decides who does what.
  mode=mission      inbound (receiving bay -> free bin: FETCH_FROM_STATION) and outbound (occupied bin ->
                    outbound bay: FETCH_FROM_BIN) run concurrently; a bin registry makes sure a bin is only ever
                    promised to one task (Mission Design §8).
  mode=goto_random  plain GOTO tasks to random aisle bays: a traffic stress test.
Unclaimed tasks are re-offered (round + 1) until someone takes them.

Metrics (logged every `report_s` and at the end; per-task CSV in `out_dir`): tasks done and per hour by type,
mean/max latency (offer -> done), robot time shares (driving / yielding / at station / idle), closest approach
between two robots, longest continuous yield (deadlock watch).
"""
import csv
import itertools
import math
import os
import random

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import yaml
from open_amr_msgs.msg import Claim, RobotState, Task

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
        # bin registry: each aisle bay node serves a west and an east bin
        self.bins = {f'{bay}:{side}': self.rng.random() < fill for bay in self.bays for side in ('W', 'E')}
        self.promised = set()
        self.pub_task = self.create_publisher(Task, '/swarm/tasks', EVENT_QOS)
        self.create_subscription(Claim, '/swarm/claims', self.on_claim, EVENT_QOS)
        self.create_subscription(RobotState, '/swarm/state', self.on_state, STATE_QOS)
        self.tasks, self.meta, self.seq = {}, {}, 0
        self.robots, self.shares, self.yield_run, self.max_yield = {}, {}, {}, (0.0, '')
        self.closest = (1e9, '')
        self.t0 = None
        os.makedirs(self.out_dir, exist_ok=True)
        self.csv = open(os.path.join(self.out_dir, 'tasks.csv'), 'w', newline='')
        self.writer = csv.writer(self.csv)
        self.writer.writerow(['task_id', 'type', 'origin', 'dest', 'offered_s', 'claimed_s', 'done_s', 'robot', 'rounds'])
        self.create_timer(0.5, self.tick)
        self.create_timer(self.report_s, self.report)
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
            pool = [b for b, full in self.bins.items() if full != inbound and b not in self.promised]
            if not pool:
                inbound = not inbound
                pool = [b for b, full in self.bins.items() if full != inbound and b not in self.promised]
            if not pool:
                return None
            binid = self.rng.choice(pool)
            self.promised.add(binid)
            bay = binid.split(':')[0]
            if inbound:
                t.type, t.origin_id, t.dest_id, kind = Task.FETCH_FROM_STATION, 'receiving_bay', bay, 'inbound'
            else:
                t.type, t.origin_id, t.dest_id, kind = Task.FETCH_FROM_BIN, bay, 'outbound_bay', 'outbound'
        self.meta[t.task_id] = dict(kind=kind, bin=binid, offered=self.now(), claimed=None, done=None, robot='')
        return t

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
            if m['claimed'] is None and now - stamp_s(t.stamp) > self.reoffer_s:
                t.round += 1
                self.offer(t)

    def on_claim(self, c):
        m = self.meta.get(c.resource_id)
        if c.resource_type != Claim.TASK or m is None:
            return
        if c.action == Claim.CLAIM and m['claimed'] is None:
            m['claimed'], m['robot'] = self.now(), c.robot_id
        elif c.action == Claim.DONE and m['done'] is None:
            m['done'] = self.now()
            if m['bin']:
                self.bins[m['bin']] = m['kind'] == 'inbound'
                self.promised.discard(m['bin'])
            t = self.tasks[c.resource_id]
            self.writer.writerow([c.resource_id, m['kind'], t.origin_id, t.dest_id, f"{m['offered'] - self.t0:.1f}",
                                  f"{(m['claimed'] or m['done']) - self.t0:.1f}", f"{m['done'] - self.t0:.1f}", c.robot_id, t.round])
            self.csv.flush()

    # ------------------------------------------------------------------ fleet metrics
    def on_state(self, s):
        prev = self.robots.get(s.robot_id)
        self.robots[s.robot_id] = s
        dt = 0.2
        sh = self.shares.setdefault(s.robot_id, dict(driving=0.0, yielding=0.0, station=0.0, idle=0.0, dist=0.0))
        key = {RobotState.YIELDING: 'yielding', RobotState.AT_STATION: 'station', RobotState.IDLE: 'idle'}.get(s.status, 'driving')
        if s.status == RobotState.IDLE and s.speed > 0.05:
            key = 'driving'
        sh[key] += dt
        if prev is not None:
            sh['dist'] += math.hypot(s.pose.x - prev.pose.x, s.pose.y - prev.pose.y)
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
        tot = {k: sum(sh[k] for sh in self.shares.values()) for k in ('driving', 'yielding', 'station', 'idle')}
        T = max(sum(tot.values()), 1e-6)
        line = (f"[metrics{' FINAL' if final else ''}] t={el:.0f}s done={len(done)} "
                + ' '.join(f'{k}={v}' for k, v in by.items() if v)
                + f" rate={len(done) / el * 3600:.0f}/h latency mean={sum(lat) / max(len(lat), 1):.0f}s "
                + f"max={max(lat, default=0):.0f}s | fleet time: driving {100 * tot['driving'] / T:.0f}% "
                + f"yielding {100 * tot['yielding'] / T:.0f}% station {100 * tot['station'] / T:.0f}% "
                + f"idle {100 * tot['idle'] / T:.0f}% | closest approach {self.closest[0]:.2f} m ({self.closest[1]}) "
                + f"| longest yield {self.max_yield[0]:.0f}s ({self.max_yield[1]}) "
                + f"| distance {sum(sh['dist'] for sh in self.shares.values()):.0f} m")
        self.get_logger().info(line)


def main():
    rclpy.init()
    node = MissionGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.csv.close()
