"""swarm_agent: one per robot, identical code everywhere, no central coordinator.

Behaviours (design: OpenAMR notes/Swarm Design.md, notes/Mission Design.md):
  heartbeat   RobotState on /swarm/state at 5 Hz: pose, status, task, route, reserved lane-graph nodes.
  auction     Tasks appear on /swarm/tasks. Every idle agent bids its estimated time to the task origin on
              /swarm/bids; when the bid window closes (task stamp + window, sim time) every agent applies the same
              rule (lowest cost, then lowest robot id) to the same bids, so all agree on the winner without an
              auctioneer. The winner announces it on /swarm/claims.
  traffic     The agent routes on the lane graph through its own route_server (node ids, never "nearest node"),
              holds lane-graph nodes via traffic.plan_reservations and gives the controller (FollowPath) only
              the stretch of path it holds. Blocked -> it stops before the taken node (YIELDING).
  execution   GOTO / FETCH_FROM_STATION / FETCH_FROM_BIN as legs: drive, dwell (simulated transfer), next leg;
              DONE on /swarm/claims; then to a free charger unless it wins another task.
  energy      Reads the robot's BMS (`battery_state`, sensor_msgs/BatteryState). Bids only on tasks it can finish
              with reserve to spare, charges below `low` until `resume`, tops up whenever idle; chargers claimed
              through `goal_node` in the heartbeat. Rules: energy.py.
Runs in the robot's namespace (/amr_i) with /tf remapped to tf; uses sim time.
"""
import math
import time as wallclock

import rclpy
import rclpy.executors
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
import tf2_ros

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D
from nav2_msgs.action import ComputeRoute, FollowPath
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap, DynamicEdges
from nav_msgs.msg import Path
from sensor_msgs.msg import BatteryState
from open_amr_msgs.msg import Bid, Claim, RobotState, Task

from .energy import ChargeParams, ChargerPeer, DrainEstimator, bid_penalty_s, can_take, pick_charger, task_need
from .lane_graph import LaneGraph
from .traffic import Peer, TrafficParams, plan_reservations, wait_chain

STATE_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
EVENT_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST)


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class Leg:
    def __init__(self, target, dwell_s, drive_status, final_precise=True):
        self.target, self.dwell_s, self.drive_status, self.precise = target, dwell_s, drive_status, final_precise


class SwarmAgent(Node):
    def __init__(self):
        super().__init__('swarm_agent')
        dp = self.declare_parameter
        self.rid = dp('robot_id', 'amr_0').value
        self.graph = LaneGraph(dp('graph', '').value)
        self.chargers = [self.graph.id(c) for c in dp('chargers', ['']).value if c] or self.graph.of_kind('charger')
        self.speed = dp('nominal_speed', 0.5).value
        self.bid_window = dp('bid_window_s', 1.5).value
        self.station_dwell = dp('station_dwell_s', 8.0).value
        self.bin_dwell = dp('bin_dwell_s', 4.0).value
        self.peer_timeout = dp('peer_timeout_s', 3.0).value
        self.idle_home_delay = dp('idle_home_delay_s', 3.0).value
        self.idle_park_s = dp('idle_park_s', 20.0).value    # idle off a charger this long: park anyway (don't block a bay)
        # speed zones by lane type, % of the controller's max speed (1.0 m/s): full on streets, slower where
        # people/racks are close (aisles) and where precision matters (docks, queues, chargers)
        dock_pct = dp('zone_dock_pct', 35.0).value
        self.zones = {'street': dp('zone_street_pct', 100.0).value, 'aisle': dp('zone_aisle_pct', 50.0).value,
                      'dock': dock_pct, 'charging': dock_pct, 'charger': dock_pct}
        self.zone_lookahead = dp('zone_lookahead_m', 1.5).value
        self.arrive_tol = dp('arrive_tol_m', 0.3).value
        self.deadlock_s = dp('deadlock_after_s', 15.0).value
        self.blocked_reroute_s = dp('reroute_blocked_after_s', 60.0).value
        self.edges_client, self.last_reroute, self.reopen_edge = None, -1e9, None
        self.tp = TrafficParams(horizon_m=dp('horizon_m', 4.0).value, max_nodes=dp('horizon_nodes', 5).value)
        self.cp = ChargeParams(low=dp('battery_low', 0.30).value, resume=dp('battery_resume', 0.60).value,
                               reserve=dp('battery_reserve', 0.12).value, plan_speed=dp('plan_speed', 0.45).value)
        # prior for the measured drain rate: ~0.15 of a charge per working hour (x the sim's battery time scale)
        self.drain = DrainEstimator(dp('drain_prior_per_h', 0.15).value / 3600.0)

        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.pub_state = self.create_publisher(RobotState, '/swarm/state', STATE_QOS)
        self.pub_bid = self.create_publisher(Bid, '/swarm/bids', EVENT_QOS)
        self.pub_claim = self.create_publisher(Claim, '/swarm/claims', EVENT_QOS)
        self.pub_speed = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.speed_pct, self.speed_sent_at = None, 0.0
        self.create_subscription(RobotState, '/swarm/state', self.on_state, STATE_QOS)
        self.create_subscription(Task, '/swarm/tasks', self.on_task, EVENT_QOS)
        self.create_subscription(Bid, '/swarm/bids', self.on_bid, EVENT_QOS)
        self.create_subscription(Claim, '/swarm/claims', self.on_claim, EVENT_QOS)
        self.route_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.clear_local = self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, 10)

        # world knowledge
        self.peers = {}                  # robot_id -> (RobotState, receive wall time)
        self.tasks, self.claims, self.done = {}, {}, set()
        self.bids = {}                   # (task_id, round) -> {robot_id: cost}
        self.decided, self.my_bids = set(), {}
        # own state
        self.pose, self.prev_pose, self.speed_est = None, None, 0.0
        self.status, self.battery = RobotState.IDLE, 1.0
        self.bms = False                 # a BMS reports: energy rules active (without one the battery reads 1.0)
        self.charging, self.must_charge, self.charger = False, False, None
        self.no_charger_logged, self.dock_check_at, self.redocks = False, None, 0
        self.task, self.legs, self.leg = None, [], None
        self.last_node, self.route, self.path, self.node_idx = None, [], None, []
        self.next_idx, self.reserved, self.blocked = 0, [], False
        self.stop_at = set()
        self.wait_s, self.phase = 0.0, 'idle'   # idle | routing | driving | dwelling | retry
        self.retry_at = 0.0
        self.dwell_until, self.idle_since = 0.0, 0.0
        self.follow_handle, self.follow_end, self.follow_state = None, None, None
        self.route_pending, self.bid_pending = False, False
        self.leg_fail = 0
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'{self.rid}: swarm agent up, {len(self.chargers)} chargers, '
                               f'{len(self.graph.pos)} lane-graph nodes')

    # ------------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, m):
        if m.robot_id != self.rid:
            self.peers[m.robot_id] = (m, wallclock.monotonic())
            # two winners of one task (bids or claims lost, e.g. while DDS discovery is still settling): the
            # heartbeat is state, so this repairs itself even when the claim events never arrive — lower id keeps it
            if self.task is not None and m.task_id == self.task.task_id and m.robot_id < self.rid:
                self.get_logger().warn(f'{self.rid}: {m.robot_id} is also doing {m.task_id} and wins the tie; dropping it')
                self.claims[m.task_id] = m.robot_id
                self.abort_task()

    def on_task(self, m):
        cur = self.tasks.get(m.task_id)
        if cur is None or m.round >= cur.round:
            self.tasks[m.task_id] = m

    def on_bid(self, m):
        self.bids.setdefault((m.task_id, m.round), {})[m.robot_id] = m.cost

    def on_claim(self, m):
        if m.resource_type != Claim.TASK:
            return
        if m.action == Claim.CLAIM:
            prev = self.claims.get(m.resource_id)
            self.claims[m.resource_id] = m.robot_id if prev is None else min(prev, m.robot_id)
            # conflicting claim (message loss): the same rule decides; the loser drops the task
            if self.task and self.task.task_id == m.resource_id and m.robot_id != self.rid and m.robot_id < self.rid:
                self.get_logger().warn(f'{self.rid}: {m.robot_id} also claimed {m.resource_id} and wins the tie; dropping it')
                self.abort_task()
        elif m.action == Claim.DONE:
            self.done.add(m.resource_id)

    def on_battery(self, m):
        self.bms, self.battery = True, float(m.percentage)
        self.charging = m.power_supply_status in (BatteryState.POWER_SUPPLY_STATUS_CHARGING,
                                                  BatteryState.POWER_SUPPLY_STATUS_FULL)
        self.drain.update(self.now(), self.battery, working=self.task is not None and not self.charging)
        if self.battery < self.cp.low and not self.must_charge:
            self.must_charge = True
            self.get_logger().info(f'{self.rid}: battery {100 * self.battery:.0f} % < {100 * self.cp.low:.0f} %: '
                                   f'no new work, charging after the current task')
        elif self.must_charge and self.battery >= self.cp.resume:
            self.must_charge = False
            self.get_logger().info(f'{self.rid}: battery {100 * self.battery:.0f} %: back to work')

    def live_peers(self):
        t = wallclock.monotonic()
        return [Peer(s.robot_id, s.pose.x, s.pose.y, list(s.reserved_nodes), s.wait_s)
                for s, rx in self.peers.values() if t - rx < self.peer_timeout]

    # ------------------------------------------------------------------ main loop
    def tick(self):
        if not self.update_pose():
            return
        if self.last_node is None:
            self.last_node = self.graph.nearest(*self.pose[:2])
            self.reserved = [self.last_node]
            self.idle_since = self.now()
        self.auction()
        if self.phase == 'driving':
            self.drive()
        elif self.phase == 'dwelling' and self.now() >= self.dwell_until:
            self.next_leg()
        elif self.phase == 'retry' and self.now() >= self.retry_at:
            self.next_leg(first=True)
        elif self.phase == 'idle':
            self.reserved = [self.last_node]
            self.idle_energy()
        self.energy_status()
        self.publish_state()

    def update_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return False
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        x, y = t.transform.translation.x, t.transform.translation.y
        if self.pose is not None:
            self.speed_est = 0.7 * self.speed_est + 0.3 * math.hypot(x - self.pose[0], y - self.pose[1]) / 0.2
        self.pose = (x, y, yaw)
        return True

    # ------------------------------------------------------------------ auction
    def pending_tasks(self):
        return sorted((t for t in self.tasks.values() if t.task_id not in self.claims and t.task_id not in self.done),
                      key=lambda t: (-t.priority, stamp_s(t.stamp), t.task_id))

    def available(self):
        return (self.task is None and self.phase in ('idle', 'driving') and not self.bid_pending
                and not (self.bms and self.must_charge))

    def task_need(self, t):
        """State of charge this task would cost, up to reaching a charger afterwards (energy.task_need)."""
        stops, dwell = [self.graph.id(t.origin_id)], 0.0
        if t.type != Task.GOTO:
            stops.append(self.graph.id(t.dest_id)); dwell = self.station_dwell + self.bin_dwell
        return task_need(self.graph, self.start_node(), stops, dwell, self.drain.rate, self.chargers, self.cp)

    def auction(self):
        now = self.now()
        # 1) decide every closed round (all agents run this with the same bids -> same winner)
        for t in list(self.tasks.values()):
            key = (t.task_id, t.round)
            if key in self.decided or now < stamp_s(t.stamp) + self.bid_window or t.task_id in self.claims:
                continue
            self.decided.add(key)
            bids = self.bids.get(key, {})
            if not bids:
                continue
            winner = min(bids.items(), key=lambda kv: (kv[1], kv[0]))[0]
            if winner == self.rid and self.task is None:
                self.claim_and_start(t)
        # 2) bid on the most urgent open task we haven't bid on yet
        if not self.available():
            return
        for t in self.pending_tasks():
            key = (t.task_id, t.round)
            if key in self.my_bids or now >= stamp_s(t.stamp) + self.bid_window:
                continue
            if self.bms and not can_take(self.battery, self.task_need(t), self.cp):
                continue                                  # can't finish it and still reach a charger
            self.bid_pending = True
            self.estimate(self.start_node(), self.graph.id(t.origin_id), lambda cost, t=t: self.send_bid(t, cost))
            return

    def start_node(self):
        return self.route[self.next_idx] if self.phase == 'driving' and self.next_idx < len(self.route) else self.last_node

    def estimate(self, start, goal, cb):
        if start == goal:
            cb(0.0); return
        g = ComputeRoute.Goal(); g.start_id, g.goal_id, g.use_start, g.use_poses = start, goal, False, False
        if not self.route_client.wait_for_server(timeout_sec=0.0):
            self.bid_pending = False; return
        fut = self.route_client.send_goal_async(g)

        def got_handle(f):
            h = f.result()
            if not h.accepted:
                self.bid_pending = False; return
            h.get_result_async().add_done_callback(
                lambda r: cb(self.graph.route_length([n.nodeid for n in r.result().result.route.nodes]) / self.speed
                             if r.result().result.error_code == 0 else None))
        fut.add_done_callback(got_handle)

    def send_bid(self, t, cost):
        self.bid_pending = False
        if cost is None or not self.available():
            return
        if self.bms:
            cost += bid_penalty_s(self.battery, self.cp)   # work drifts to fuller robots
        b = Bid(stamp=self.get_clock().now().to_msg(), task_id=t.task_id, round=t.round, robot_id=self.rid,
                cost=float(cost))
        self.my_bids[(t.task_id, t.round)] = cost
        self.on_bid(b)
        self.pub_bid.publish(b)

    def claim_and_start(self, t):
        self.claims[t.task_id] = self.rid
        self.set_charger(None)
        self.pub_claim.publish(Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK,
                                     resource_id=t.task_id, robot_id=self.rid, action=Claim.CLAIM))
        o = self.graph.id(t.origin_id)
        if t.type == Task.GOTO:
            legs = [Leg(o, 0.0, RobotState.TO_PICK)]
        elif t.type == Task.FETCH_FROM_STATION:
            legs = [Leg(o, self.station_dwell, RobotState.TO_PICK), Leg(self.graph.id(t.dest_id), self.bin_dwell, RobotState.TO_DROP)]
        else:
            legs = [Leg(o, self.bin_dwell, RobotState.TO_PICK), Leg(self.graph.id(t.dest_id), self.station_dwell, RobotState.TO_DROP)]
        self.get_logger().info(f'{self.rid}: won {t.task_id} ({t.origin_id} -> {t.dest_id or "-"})')
        self.start_legs(legs, task=t)

    def abort_task(self):
        self.task, self.legs = None, []
        self.stop_following()
        self.phase = 'idle'; self.status = RobotState.IDLE; self.idle_since = self.now()

    # ------------------------------------------------------------------ legs
    def start_legs(self, legs, task):
        self.task, self.legs = task, list(legs)
        self.next_leg(first=True)

    def next_leg(self, first=False):
        if not first and self.leg is not None and self.task is not None and not self.legs:
            self.finish_task()
            return
        if not self.legs:
            self.phase, self.status, self.leg = 'idle', RobotState.IDLE, None
            self.idle_since = self.now()
            return
        self.leg = self.legs.pop(0)
        self.status = self.leg.drive_status
        start = self.start_node()
        if start == self.leg.target:
            self.arrived(); return
        self.phase, self.route_pending, self.leg_fail = 'routing', True, 0
        g = ComputeRoute.Goal(); g.start_id, g.goal_id, g.use_start, g.use_poses = start, self.leg.target, False, False
        self.route_client.wait_for_server(timeout_sec=2.0)
        self.route_client.send_goal_async(g).add_done_callback(self.on_route_handle)

    def on_route_handle(self, f):
        h = f.result()
        if not h.accepted:
            self.get_logger().warn(f'{self.rid}: route request rejected'); self.phase = 'idle'; return
        h.get_result_async().add_done_callback(self.on_route)

    def on_route(self, f):
        res = f.result().result
        self.route_pending = False
        if self.reopen_edge is not None and self.edges_client is not None:   # the closure was only for this re-route
            self.edges_client.call_async(DynamicEdges.Request(opened_edges=[self.reopen_edge]))
            self.reopen_edge = None
        if res.error_code != 0 or not res.route.nodes:
            self.get_logger().warn(f'{self.rid}: no route to {self.graph.name[self.leg.target]} (error {res.error_code}), retrying')
            self.legs.insert(0, self.leg); self.leg = None
            self.phase, self.retry_at = 'retry', self.now() + 2.0     # tick() re-plans the leg
            return
        self.route = [n.nodeid for n in res.route.nodes]
        self.path = res.path
        self.node_idx = self.index_nodes(self.route, res.path)
        self.stop_at = self.stop_nodes(self.route)
        self.next_idx = 1 if self.route[0] == self.last_node else 0
        self.phase, self.follow_end, self.wait_s = 'driving', None, 0.0

    def stop_nodes(self, route):
        """Route indices where the controller's path must end so the robot stops and turns in place: sharp turns
        (>= 80 deg) next to a short link (< 1.5 m) — U-turns across the 1.1 m lane spacing and turns across a lane
        into an aisle. A path that doubles back 1.1 m apart lets MPPI cut across the U and stall (2026-09-25)."""
        out = set()
        for k in range(1, len(route) - 1):
            a, b, c = route[k - 1], route[k], route[k + 1]
            turn = abs(math.remainder(self.graph.heading(b, c) - self.graph.heading(a, b), math.tau))
            if turn >= math.radians(80) and min(self.graph.dist(a, b), self.graph.dist(b, c)) < 1.5:
                out.add(k)
        return out

    def index_nodes(self, route, path):
        """Path-pose index closest to each route node, walking forward (corners are smoothed, so 'closest')."""
        pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        out, i = [], 0
        for n in route:
            nx, ny = self.graph.pos[n]
            best, bi = 1e9, i
            for j in range(i, len(pts)):
                d = math.hypot(pts[j][0] - nx, pts[j][1] - ny)
                if d < best:
                    best, bi = d, j
                elif best < 0.6 and d > best + 0.3:
                    break
            out.append(bi); i = bi
        return out

    # ------------------------------------------------------------------ driving with reservations
    def drive(self):
        x, y, _ = self.pose
        # progress: the robot's nearest path index tells which nodes are behind it
        pts = self.path.poses
        lo = self.node_idx[max(self.next_idx - 1, 0)]
        hi = min(len(pts), self.node_idx[min(self.next_idx + 1, len(self.route) - 1)] + 1)
        ridx = min(range(lo, max(hi, lo + 1)), key=lambda j: math.hypot(pts[j].pose.position.x - x, pts[j].pose.position.y - y))
        while self.next_idx < len(self.route) and (
                ridx >= self.node_idx[self.next_idx] or
                math.hypot(self.graph.pos[self.route[self.next_idx]][0] - x, self.graph.pos[self.route[self.next_idx]][1] - y) < 0.25 or
                # the controller finished a segment ending here (hold point / turn stop): it stopped within tolerance
                (self.follow_state == GoalStatus.STATUS_SUCCEEDED and self.follow_end == self.route[self.next_idx])):
            if self.next_idx == len(self.route) - 1 and not self.final_reached(x, y):
                break   # the final node counts only when the controller reports arrival (or see final_reached)
            self.last_node = self.route[self.next_idx]
            self.next_idx += 1
        if self.next_idx >= len(self.route):
            if self.follow_state == GoalStatus.STATUS_EXECUTING:
                self.stop_following()
            self.arrived(); return
        me = Peer(self.rid, x, y, self.reserved, self.wait_s)
        held = self.reserved
        self.reserved, self.blocked = plan_reservations(self.graph, self.route, self.next_idx, me, self.live_peers(), held, self.tp)
        ahead = [n for n in self.reserved if n in self.route[self.next_idx:]]
        if not ahead:
            # hold: stop before the next node
            if self.follow_end is not None:
                self.stop_following()
            self.status_wait(True)
            self.maybe_break_deadlock()
            return
        self.apply_speed_zone(x, y)
        end_node = ahead[-1]
        end_k = self.route.index(end_node, self.next_idx)
        turn_stops = [k for k in self.stop_at if self.next_idx <= k < end_k]
        if turn_stops:                                   # stop and turn in place first
            end_k = min(turn_stops); end_node = self.route[end_k]
        final = end_k == len(self.route) - 1
        self.status_wait(self.blocked and self.speed_est < 0.05)
        self.maybe_break_deadlock()
        if self.follow_end != end_node or self.follow_state in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            if self.follow_state == GoalStatus.STATUS_ABORTED:
                self.leg_fail += 1
                if self.leg_fail % 3 == 0 and self.clear_local.service_is_ready():
                    self.clear_local.call_async(ClearEntireCostmap.Request())   # stale obstacles are the usual cause
                if self.leg_fail > 8:
                    self.status = RobotState.STUCK
                    self.get_logger().warn(f'{self.rid}: controller keeps failing near {self.graph.name[self.route[self.next_idx]]}')
            self.send_path(ridx, self.node_idx[end_k], final and self.leg.precise)
            self.follow_end = end_node

    def final_reached(self, x, y):
        """Arrival at the leg's last node: the controller succeeded; or the precise final approach gave up while the
        robot stands within `arrive_tol_m` — a forward-only robot can't remove a sideways offset once it is on top of
        the goal (MPPI aborts on no progress, forever: seen 2026-09-25 at 13 and 26 cm); or at a charger the BMS
        reports the contacts closed."""
        if self.follow_state == GoalStatus.STATUS_SUCCEEDED:
            return True
        n = self.route[-1]
        d = math.hypot(self.graph.pos[n][0] - x, self.graph.pos[n][1] - y)
        if self.follow_state == GoalStatus.STATUS_ABORTED and d < self.arrive_tol:
            self.get_logger().info(f'{self.rid}: at {self.graph.name[n]} within {100 * d:.0f} cm (final approach gave up)')
            return True
        return self.bms and self.charging and n in self.chargers and d < 0.4

    def apply_speed_zone(self, x, y):
        """Speed limit = slowest zone among the current lane and the next one if it starts within lookahead."""
        k = self.next_idx
        kinds = []
        if k > 0:
            kinds.append(self.graph.edge_kind.get((self.route[k - 1], self.route[k]), 'street'))
        nx, ny = self.graph.pos[self.route[k]]
        if k + 1 < len(self.route) and math.hypot(nx - x, ny - y) < self.zone_lookahead:
            kinds.append(self.graph.edge_kind.get((self.route[k], self.route[k + 1]), 'street'))
        pct = min((self.zones.get(kd, 100.0) for kd in kinds), default=100.0)
        if pct != self.speed_pct or self.now() - self.speed_sent_at > 2.0:
            m = SpeedLimit(percentage=True, speed_limit=float(pct if pct < 100.0 else 0.0))   # 0 = no limit
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_speed.publish(m)
            self.speed_pct, self.speed_sent_at = pct, self.now()

    # ------------------------------------------------------------------ deadlock breaking
    def waits(self):
        """robot -> robot it waits for, for every yielding robot (from heartbeats + my own state)."""
        t = wallclock.monotonic()
        states = {rid: s for rid, (s, rx) in self.peers.items() if t - rx < self.peer_timeout}
        held = {rid: set(s.reserved_nodes) for rid, s in states.items()}
        held[self.rid] = set(self.reserved)
        holder = lambda n, me: min((r for r, h in held.items() if r != me and n in h), default=None)
        w = {rid: holder(s.route[0], rid) for rid, s in states.items() if s.status == RobotState.YIELDING and s.route}
        if self.phase == 'driving' and self.next_idx < len(self.route):
            w[self.rid] = holder(self.route[self.next_idx], self.rid)
        return w, states

    def maybe_break_deadlock(self):
        """A cycle of robots each waiting for the next never resolves by waiting: the highest id in the cycle
        re-routes around its blocked lane (every member computes the same cycle from the same heartbeats, so
        exactly one yields). Also re-route after a long block behind a robot that isn't moving (stuck)."""
        if self.wait_s < self.deadlock_s or self.now() - self.last_reroute < 20.0 or self.next_idx == 0:
            return
        w, states = self.waits()
        chain, cycle = wait_chain(self.rid, w)
        end = states.get(chain[-1])
        long_block = self.wait_s > self.blocked_reroute_s and end is not None and (
            end.status == RobotState.STUCK or end.wait_s > self.blocked_reroute_s)
        if (cycle and self.rid in cycle and self.rid == max(cycle)) or long_block:
            why = f'deadlock cycle {cycle}' if cycle else f'blocked {self.wait_s:.0f}s behind {chain[1:]}'
            self.reroute_around(self.route[self.next_idx - 1], self.route[self.next_idx], why)

    def reroute_around(self, a, b, why):
        eid = self.graph.edge_id.get((a, b))
        if eid is None or self.leg is None:
            return
        if self.edges_client is None:
            ns = self.get_namespace().rstrip('/')
            names = [n for n, _ in self.get_service_names_and_types() if n.startswith(ns + '/') and n.endswith('adjust_edges')]
            if not names:
                return
            self.edges_client = self.create_client(DynamicEdges, names[0])
        self.last_reroute = self.now()
        self.get_logger().warn(f'{self.rid}: {why} -> closing {self.graph.name[a]}->{self.graph.name[b]} for me and re-routing')
        self.edges_client.call_async(DynamicEdges.Request(closed_edges=[eid]))
        self.reopen_edge = eid
        self.stop_following()
        self.legs.insert(0, self.leg)
        self.last_node, self.phase = a, 'rerouting'   # start_node() must be where I stand, not the blocked node
        self.next_leg(first=True)        # route from where I stand to the same target, without that lane

    def status_wait(self, waiting):
        if waiting:
            self.wait_s += 0.2
            if self.status != RobotState.STUCK:
                self.status = RobotState.YIELDING
        else:
            self.wait_s = 0.0
            if self.leg is not None and self.status == RobotState.YIELDING:
                self.status = self.leg.drive_status

    def send_path(self, i0, i1, precise):
        p = Path(); p.header = self.path.header; p.header.stamp = self.get_clock().now().to_msg()
        p.poses = self.path.poses[max(0, i0 - 1): i1 + 1]
        if len(p.poses) < 2:
            return
        g = FollowPath.Goal(); g.path = p; g.controller_id = 'FollowPath'
        g.goal_checker_id = 'precise_goal_checker' if precise else 'general_goal_checker'
        self.follow_state = GoalStatus.STATUS_EXECUTING
        self.follow_client.send_goal_async(g).add_done_callback(self.on_follow_handle)

    def on_follow_handle(self, f):
        h = f.result()
        if not h.accepted:
            self.follow_state = GoalStatus.STATUS_ABORTED; return
        self.follow_handle = h
        h.get_result_async().add_done_callback(lambda r, h=h: self.on_follow_done(h, r))

    def on_follow_done(self, h, r):
        if h is self.follow_handle:
            self.follow_state = r.result().status

    def stop_following(self):
        if self.follow_handle is not None:
            self.follow_handle.cancel_goal_async()
        self.follow_handle, self.follow_end, self.follow_state = None, None, None

    def arrived(self):
        self.last_node = self.leg.target if self.leg else self.last_node
        self.reserved = [self.last_node]
        self.follow_handle, self.follow_end, self.follow_state = None, None, None
        if self.leg and self.leg.dwell_s > 0:
            self.phase, self.status = 'dwelling', RobotState.AT_STATION
            self.dwell_until = self.now() + self.leg.dwell_s
        else:
            self.next_leg()

    def finish_task(self):
        t = self.task
        self.pub_claim.publish(Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK,
                                     resource_id=t.task_id, robot_id=self.rid, action=Claim.DONE))
        self.done.add(t.task_id)
        self.get_logger().info(f'{self.rid}: done {t.task_id} at {self.graph.name[self.last_node]}')
        self.task, self.leg = None, None
        self.phase, self.status, self.idle_since = 'idle', RobotState.IDLE, self.now()

    # ------------------------------------------------------------------ charging
    def pick(self, start, current=None):
        t = wallclock.monotonic()
        peers = [ChargerPeer(s.robot_id, s.goal_node, list(s.reserved_nodes), s.battery)
                 for s, rx in self.peers.values() if t - rx < self.peer_timeout]
        return pick_charger(self.graph, start, self.chargers, self.rid, self.battery, peers, current, self.cp)

    def set_charger(self, c):
        """Announce charger claims/releases on /swarm/claims (for observers; the heartbeat's goal_node is what
        peers decide on)."""
        if c == self.charger:
            return
        for node, action in ((self.charger, Claim.RELEASE), (c, Claim.CLAIM)):
            if node is not None:
                self.pub_claim.publish(Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.DOCK,
                                             resource_id=self.graph.name[node], robot_id=self.rid, action=action))
        self.charger = c

    def go_charge(self, c, redock=False):
        self.set_charger(c)
        st = RobotState.TO_CHARGE if self.must_charge else RobotState.IDLE
        legs = [Leg(c, 0.0, st)]
        if redock:     # contacts didn't close: back out to the dock's access node and drive in again
            access = next(a for a, succ in self.graph.succ.items() if c in succ)
            legs.insert(0, Leg(access, 0.0, st, final_precise=False))
        self.get_logger().info(f'{self.rid}: {"re-docking at" if redock else "to"} {self.graph.name[c]} '
                               f'(battery {100 * self.battery:.0f} %{", must charge" if self.must_charge else ""})')
        self.start_legs(legs, task=None)

    def idle_energy(self):
        """Idle: park on a free charger (top up; still available for work), or go charge because the battery
        is low. On a charger: make sure the contacts actually closed."""
        if self.task is not None:
            return
        now = self.now()
        if self.last_node in self.chargers:
            self.set_charger(self.last_node)
            if not self.bms or self.charging:
                self.dock_check_at, self.redocks = None, 0
            elif self.dock_check_at is None:
                self.dock_check_at = now
            elif now - self.dock_check_at > 10.0 and self.redocks < 2:
                self.get_logger().warn(f'{self.rid}: on {self.graph.name[self.last_node]} but not charging')
                self.redocks += 1
                self.dock_check_at = None
                self.go_charge(self.last_node, redock=True)
            return
        pending = self.pending_tasks()
        idle_for = now - self.idle_since
        if not (self.must_charge or idle_for > self.idle_park_s or
                (idle_for > self.idle_home_delay and
                 (not pending or (self.bms and not any(can_take(self.battery, self.task_need(t), self.cp) for t in pending))))):
            return
        c = self.pick(self.last_node)
        if c is None:
            if not self.no_charger_logged:
                self.get_logger().warn(f'{self.rid}: no free charger'); self.no_charger_logged = True
            return
        self.no_charger_logged = False
        self.go_charge(c)

    def energy_status(self):
        if not self.bms:
            return
        if self.battery <= self.cp.critical and not self.charging and self.phase != 'empty':
            self.get_logger().error(f'{self.rid}: battery empty ({100 * self.battery:.1f} %), stopping here')
            self.stop_following(); self.phase, self.status = 'empty', RobotState.STUCK
            return
        leg = self.leg
        to_charger = self.task is None and leg is not None and leg.target in self.chargers and \
            self.phase in ('routing', 'driving')
        if to_charger:
            leg.drive_status = RobotState.TO_CHARGE if self.must_charge else RobotState.IDLE
            if self.status in (RobotState.IDLE, RobotState.TO_CHARGE):
                self.status = leg.drive_status
            if self.phase == 'driving' and not self.legs:
                c = self.pick(self.start_node(), current=leg.target)
                if c is not None and c != leg.target:     # lost a same-instant race for it
                    self.go_charge(c)
        elif self.phase == 'idle' and self.task is None:
            self.status = RobotState.CHARGING if self.charging and self.battery < 0.999 else RobotState.IDLE

    # ------------------------------------------------------------------ heartbeat
    def publish_state(self):
        m = RobotState()
        m.stamp = self.get_clock().now().to_msg()
        m.robot_id, m.status = self.rid, self.status
        m.pose = Pose2D(x=self.pose[0], y=self.pose[1], theta=self.pose[2])
        m.speed, m.battery = float(self.speed_est), float(self.battery)
        m.task_id = self.task.task_id if self.task else ''
        m.mission_id = self.task.mission_id if self.task else ''
        m.last_node = int(self.last_node)
        m.route = [int(n) for n in self.route[self.next_idx:]] if self.phase == 'driving' else []
        m.reserved_nodes = [int(n) for n in self.reserved]
        m.wait_s = float(self.wait_s)
        moving = self.leg is not None and self.phase in ('routing', 'driving', 'rerouting')
        m.goal_node = int(self.leg.target if moving else self.legs[0].target if self.legs else self.last_node)
        self.pub_state.publish(m)


def main():
    rclpy.init()
    node = SwarmAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
