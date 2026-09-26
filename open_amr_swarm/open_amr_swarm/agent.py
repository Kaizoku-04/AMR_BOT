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
  execution   GOTO / FETCH_FROM_STATION / FETCH_FROM_BIN as legs: drive, transfer, next leg; DONE on /swarm/claims;
              then to a free charger unless it wins another task. At an arm bay the robot waits for the station's
              heartbeat (/swarm/stations) to report its task served; at a bin its own conveyor deck takes bin_dwell_s.
              An empty robot gives its task back (RELEASE) when the arm stays out of service; a loaded one waits.
  energy      Reads the robot's BMS (`battery_state`, sensor_msgs/BatteryState). Bids only on tasks it can finish
              with reserve to spare, charges below `low` until `resume`, tops up whenever idle; chargers claimed
              through `goal_node` in the heartbeat. Rules: energy.py.
  liveness    A silent peer stays where it was last heard (its reserved nodes stay blocked, its charger taken, routes
              avoid it) until heard again or released by an operator (/swarm/operator). Without a quorum of the fleet
              the robot stops at its next held node (ISOLATED). A silent claimant's task is given back by the lowest
              live id; a robot stuck too long gives its own back — never with a box on the deck. Action goals whose
              acknowledgement or result is lost are resent / re-requested. Rules: liveness.py.
Runs in the robot's namespace (/amr_i) with /tf remapped to tf; uses sim time.
"""
import math
import time as wallclock

import rclpy
import rclpy.executors
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from rclpy.time import Time
import tf2_ros

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import ComputeRoute, FollowPath
from nav2_msgs.msg import SpeedLimit
from nav2_msgs.srv import ClearEntireCostmap, DynamicEdges
from nav_msgs.msg import Path
from sensor_msgs.msg import BatteryState
from open_amr_msgs.msg import Bid, Claim, OperatorCommand, RobotState, StationState, Task, Transfer

from .energy import (ChargeParams, ChargerPeer, DrainEstimator, bid_penalty_s, can_take, evictions, pick_charger,
                     task_need, waiting_for_charger)
from .lane_graph import LaneGraph
from .liveness import (LivenessParams, Pending, acked, announcer, ghost_hold, outranks, quorum, reclaimable)
from .traffic import Peer, TrafficParams, plan_reservations, wait_chain

STATE_QOS = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
EVENT_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST)
OPERATOR_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                          history=HistoryPolicy.KEEP_LAST)


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


class Leg:
    def __init__(self, target, dwell_s, drive_status, final_precise=True, station=False, transfer=None):
        self.target, self.dwell_s, self.drive_status, self.precise = target, dwell_s, drive_status, final_precise
        self.station = station           # an arm bay: the transfer ends when the station reports the task served
        self.transfer = transfer         # Transfer kind the robot's own deck performs here (bins), announced on arrival


class SwarmAgent(Node):
    def __init__(self):
        super().__init__('swarm_agent')
        dp = self.declare_parameter
        self.rid = dp('robot_id', 'amr_0').value
        self.graph = LaneGraph(dp('graph', '').value)
        # chargers in service (default: every charger in the graph) and parking spots
        self.chargers = [self.graph.id(c) for c in dp('chargers', ['']).value if c] or self.graph.of_kind('charger')
        self.parking = self.graph.of_kind('parking')
        self.top_up_below = dp('top_up_below', 0.90).value   # a parked robot moves to a free charger below this
        self.speed = dp('nominal_speed', 0.5).value
        self.bid_window = dp('bid_window_s', 1.5).value
        self.station_dwell = dp('station_dwell_s', 8.0).value
        self.bin_dwell = dp('bin_dwell_s', 4.0).value
        # arm bays: wait for the station's handshake (False: a fixed station_dwell_s, for runs without station agents)
        self.handshake = dp('station_handshake', True).value
        self.station_giveup = dp('station_giveup_s', 30.0).value   # empty robot: arm out of service this long -> give the task back
        self.station_timeout = dp('station_timeout_s', 3.0).value  # heartbeat older than this = station down
        self.peer_timeout = dp('peer_timeout_s', 3.0).value
        self.lp = LivenessParams(peer_timeout_s=self.peer_timeout, ghost_extra_m=dp('ghost_extra_m', 2.0).value,
                                 reclaim_silent_s=dp('reclaim_silent_s', 20.0).value,
                                 reclaim_stuck_s=dp('reclaim_stuck_s', 120.0).value)
        # lost acknowledgements (wall s): an action goal not accepted in ack_timeout is resent, a route not computed in
        # route_timeout is requested again, a result not received result_poll_s after the robot stopped is re-requested
        self.ack_timeout = dp('ack_timeout_s', 3.0).value
        self.route_timeout = dp('route_timeout_s', 10.0).value
        self.result_poll = dp('result_poll_s', 30.0).value   # > Nav2's 10 s progress-checker abort
        # test hook (fault injection): true = this robot's swarm link is cut, nothing in or out on /swarm/* (its own
        # Nav2 and sensors keep working, as on a real robot whose Wi-Fi drops). `ros2 param set ... link_down true`
        self.link_down = dp('link_down', False).value
        self.add_on_set_parameters_callback(self.on_params)
        # fleet roster (launch passes every robot id): don't act before hearing every peer (DDS discovery can take
        # seconds, and a robot it can't hear is invisible to reservations and charger claims), warn when one goes quiet
        self.fleet = [r for r in dp('fleet', ['']).value if r and r != self.rid]
        self.sync_timeout = dp('sync_timeout_s', 60.0).value
        self.synced, self.t_up, self.missing = not self.fleet, wallclock.monotonic(), None
        self.idle_home_delay = dp('idle_home_delay_s', 3.0).value
        self.idle_park_s = dp('idle_park_s', 20.0).value    # idle off a charger this long: park anyway (don't block a bay)
        # speed zones by lane type, % of the controller's max speed (1.0 m/s): full on streets, slower where
        # people/racks are close (aisles) and where precision matters (docks, queues, chargers)
        dock_pct = dp('zone_dock_pct', 35.0).value
        self.zones = {'street': dp('zone_street_pct', 100.0).value, 'aisle': dp('zone_aisle_pct', 50.0).value,
                      'dock': dock_pct, 'charging': dock_pct, 'charger': dock_pct, 'parking': dock_pct, 'park': dock_pct}
        self.zone_lookahead = dp('zone_lookahead_m', 1.5).value
        self.arrive_tol = dp('arrive_tol_m', 0.3).value
        self.arrive_tol_parking = dp('arrive_tol_parking_m', 0.5).value   # a parked robot needs no precision
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
        self.pub_transfer = self.create_publisher(Transfer, '/swarm/transfers', EVENT_QOS)
        self.speed_pct, self.speed_sent_at = None, 0.0
        self.create_subscription(RobotState, '/swarm/state', self.on_state, STATE_QOS)
        self.create_subscription(Task, '/swarm/tasks', self.on_task, EVENT_QOS)
        self.create_subscription(Bid, '/swarm/bids', self.on_bid, EVENT_QOS)
        self.create_subscription(Claim, '/swarm/claims', self.on_claim, EVENT_QOS)
        self.create_subscription(StationState, '/swarm/stations', self.on_station, STATE_QOS)
        self.create_subscription(OperatorCommand, '/swarm/operator', self.on_operator, OPERATOR_QOS)
        self.route_client = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')
        self.clear_local = self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.create_subscription(BatteryState, 'battery_state', self.on_battery, 10)

        # world knowledge
        self.peers = {}                  # robot_id -> (RobotState, receive wall time); kept when silent (a ghost)
        self.released = {}               # robot_id -> sim stamp of the operator's release (silence before it forgiven)
        self.has_quorum, self.isolated_since, self.ack_waits = True, None, 0
        self.seq, self.first_seq, self.confirmed = 0, {}, set()   # acknowledged reservations (liveness rule 3)
        self.ghost_closed = {}           # edge id -> ghost robot: lanes into a silent robot's nodes, closed for routing
        self.reclaimed = set()           # (task_id, round) given back on a silent claimant's behalf (announce once)
        self.claim_round = {}            # task_id -> auction round of the claim in self.claims
        self.stranded = set()            # (robot, task_id): silent with a box on the deck (logged once)
        self.in_edges = {}               # node -> [(from node, lane id)] of the lanes into it
        for (a, b), eid in self.graph.edge_id.items():
            self.in_edges.setdefault(b, []).append((a, eid))
        self.cut_off_cache, self.trapped_logged = (None, (set(), set())), False
        self.stations = {}               # bay node id -> (StationState, receive wall time)
        self.station_down_since = None
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
        self.loaded = False              # a box is on the deck (pickup started .. drop done)
        self.noroute_since, self.noroute_parked = None, False
        self.noroute_s = dp('noroute_giveup_s', 30.0).value   # target unreachable this long: give back / wait parked
        self.stuck_since = None
        self.last_node, self.route, self.path, self.node_idx = None, [], None, []
        self.next_idx, self.reserved, self.blocked = 0, [], False
        self.stop_at = set()
        self.wait_s, self.phase = 0.0, 'idle'   # idle | routing | driving | dwelling | retry
        self.retry_at = 0.0
        self.dwell_until, self.idle_since = 0.0, 0.0
        self.follow_handle, self.follow_end, self.follow_state = None, None, None
        self.route_pending, self.bid_pending = False, False
        self.route_req, self.bid_req, self.follow_req = Pending(), Pending(), Pending()
        self.result_polled_at, self.still_since = 0.0, None
        self.leg_fail = 0
        self.create_timer(0.2, self.tick)
        self.get_logger().info(f'{self.rid}: swarm agent up, {len(self.chargers)} chargers, '
                               f'{len(self.graph.pos)} lane-graph nodes')

    # ------------------------------------------------------------------ inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_params(self, params):
        for prm in params:
            if prm.name == 'link_down':
                self.link_down = bool(prm.value)
                self.get_logger().warn(f'{self.rid}: TEST swarm link {"CUT" if self.link_down else "restored"}')
        return SetParametersResult(successful=True)

    def on_state(self, m):
        if self.link_down or m.robot_id == self.rid:
            return
        prev = self.peers.get(m.robot_id)
        if prev is not None and wallclock.monotonic() - prev[1] >= self.peer_timeout:
            self.get_logger().info(f'{self.rid}: hears {m.robot_id} again after '
                                   f'{wallclock.monotonic() - prev[1]:.0f} s')
        if m.robot_id in self.released and stamp_s(m.stamp) > self.released[m.robot_id]:
            del self.released[m.robot_id]
            self.get_logger().warn(f'{self.rid}: released robot {m.robot_id} is back in the fleet')
        self.peers[m.robot_id] = (m, wallclock.monotonic())
        # two robots on one task (bids or claims lost, e.g. while DDS discovery is still settling, or a re-auction the
        # original claimant never heard): the heartbeat is state, so this repairs itself even when the claim events
        # never arrive — a box on the deck wins, then the newer auction round, then the lower id (liveness.outranks)
        if self.task is not None and m.task_id == self.task.task_id and \
                outranks((m.loaded, m.task_round, m.robot_id), (self.loaded, self.task.round, self.rid)):
            self.get_logger().warn(f'{self.rid}: {m.robot_id} is also doing {m.task_id} (round {m.task_round}) and '
                                   f'outranks me (round {self.task.round}); dropping it')
            self.claims[m.task_id], self.claim_round[m.task_id] = m.robot_id, m.task_round
            self.abort_task()

    def on_task(self, m):
        if self.link_down:
            return
        cur = self.tasks.get(m.task_id)
        if cur is None or m.round >= cur.round:
            self.tasks[m.task_id] = m
        # the WMS re-targets my box to another bin (its bin was cut off): drive there instead
        if self.task is not None and m.task_id == self.task.task_id and m.dest_id and m.dest_id != self.task.dest_id \
                and self.loaded:
            self.get_logger().warn(f'{self.rid}: WMS re-targets {m.task_id} to {m.dest_id} (was {self.task.dest_id})')
            self.task = m
            new = self.graph.id(m.dest_id)
            for leg in ([self.leg] if self.leg is not None else []) + self.legs:
                if leg.drive_status == RobotState.TO_DROP and not leg.station:
                    leg.target = new
            if self.phase in ('driving', 'retry') and self.leg is not None and self.leg.target == new:
                if self.phase == 'driving' and self.next_idx > 0:
                    self.replan('drop re-targeted')
                else:
                    self.stop_following()
                    self.legs.insert(0, self.leg); self.leg = None
                    self.phase, self.retry_at = 'retry', self.now()

    def on_bid(self, m):
        if self.link_down:
            return
        self.bids.setdefault((m.task_id, m.round), {})[m.robot_id] = m.cost

    def on_claim(self, m):
        if self.link_down or m.resource_type != Claim.TASK:
            return
        tid = m.resource_id
        if m.action == Claim.CLAIM:
            prev = self.claims.get(tid)
            if prev is None or outranks((False, m.round, m.robot_id), (False, self.claim_round.get(tid, 0), prev)):
                self.claims[tid], self.claim_round[tid] = m.robot_id, m.round
            # conflicting claim (message loss): the same rule decides; the loser drops the task
            if self.task and self.task.task_id == tid and m.robot_id != self.rid and not self.loaded and \
                    outranks((False, m.round, m.robot_id), (False, self.task.round, self.rid)):
                self.get_logger().warn(f'{self.rid}: {m.robot_id} also claimed {tid} and outranks me; dropping it')
                self.abort_task()
        elif m.action in (Claim.DONE, Claim.CANCEL):
            self.done.add(tid)
            if m.action == Claim.CANCEL and self.task is not None and self.task.task_id == tid and not self.loaded:
                self.get_logger().warn(f'{self.rid}: the WMS withdrew {tid}; dropping it')
                self.claims.pop(tid, None)
                self.abort_task()
        elif m.action == Claim.RELEASE:
            if m.by and m.by != m.robot_id:
                self.reclaimed.add((tid, m.round))
            if self.claims.get(tid) == m.robot_id:
                del self.claims[tid]              # given back: open again when the WMS re-offers it
            # my own task, given back on my behalf while my peers couldn't hear me: it's being re-auctioned
            if m.robot_id == self.rid and m.by and m.by != self.rid and self.task is not None and \
                    self.task.task_id == tid and self.task.round == m.round:
                if self.loaded:
                    self.get_logger().error(f'{self.rid}: {m.by} gave my {tid} back while the box is on my deck; '
                                            f'delivering it anyway')
                else:
                    self.get_logger().warn(f'{self.rid}: {m.by} gave {tid} back on my behalf (I was silent); dropping it')
                    self.abort_task()

    def on_operator(self, m):
        if self.link_down:
            return
        if m.command == OperatorCommand.RELEASE_ROBOT and m.target != self.rid:
            self.released[m.target] = stamp_s(m.stamp)
            self.get_logger().warn(f'{self.rid}: operator {m.issued_by or "?"} released {m.target}'
                                   f'{": " + m.reason if m.reason else ""} — its lanes and charger are free, '
                                   f'it no longer counts for quorum')

    def on_station(self, m):
        if self.link_down:
            return
        if m.bay_node in self.graph.by_name:
            self.stations[self.graph.by_name[m.bay_node]] = (m, wallclock.monotonic())

    def station_state(self, bay):
        """Latest heartbeat of the station serving `bay`, None if unheard or stale."""
        m = self.stations.get(bay)
        return m[0] if m and wallclock.monotonic() - m[1] < self.station_timeout else None

    def station_down(self, bay):
        st = self.station_state(bay)
        return st is None or st.state == StationState.FAULT

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

    def check_peers(self):
        """Roster check. False while still waiting for the first contact with every peer (max sync_timeout_s)."""
        if not self.fleet:
            return True
        t = wallclock.monotonic()
        missing = [r for r in self.fleet if r not in self.peers or t - self.peers[r][1] >= self.peer_timeout]
        if missing != self.missing:
            if missing:
                self.get_logger().warn(f'{self.rid}: can\'t hear {missing} '
                                       f'({"waiting before acting" if not self.synced else "held where last heard"})')
            elif self.missing is not None:
                self.get_logger().info(f'{self.rid}: hears all {len(self.fleet)} peers'
                                       f'{f" after {t - self.t_up:.0f} s" if not self.synced else ""}')
            self.missing = missing
        if not self.synced and (not missing or t - self.t_up > self.sync_timeout):
            self.synced = True
            self.idle_since = self.now()
        return self.synced

    def heard(self):
        t = wallclock.monotonic()
        return [r for r, (_, rx) in self.peers.items() if t - rx < self.peer_timeout]

    def ghosts(self):
        """Silent peers not released by an operator: robot_id -> (last RobotState, silent for s, nodes held)."""
        t, out = wallclock.monotonic(), {}
        for rid, (s, rx) in self.peers.items():
            if t - rx < self.peer_timeout:
                continue
            if rid in self.released and self.released[rid] >= stamp_s(s.stamp):
                continue
            out[rid] = (s, t - rx, ghost_hold(self.graph, list(s.reserved_nodes), list(s.route), s.pose.x, s.pose.y,
                                               self.lp.ghost_extra_m))
        return out

    def cut_off(self, ghosts=None):
        """(nodes silent robots hold, nodes cut off by them): on one-way lanes a stopped robot can leave a stretch with
        no way out (trapped) or no way in; nobody is sent there (liveness rule 1, LaneGraph.cut_off)."""
        ghosts = self.ghosts() if ghosts is None else ghosts
        held = frozenset(n for _, _, h in ghosts.values() for n in h)
        if self.cut_off_cache[0] != held:
            self.cut_off_cache = (held, (set(held), self.graph.cut_off(held), self.graph.trapped(held)))
        return self.cut_off_cache[1][:2]

    def trapped_nodes(self):
        self.cut_off()
        return self.cut_off_cache[1][2]

    def confirm(self):
        """Liveness rule 3: reserved nodes every peer I hear has acknowledged. Returns the confirmed set."""
        t = wallclock.monotonic()
        live = [s for s, rx in self.peers.values() if t - rx < self.peer_timeout]
        self.confirmed &= set(self.reserved)
        for n in self.reserved:
            if n not in self.confirmed and acked(self.first_seq.get(n), self.rid, live):
                self.confirmed.add(n)
        return self.confirmed

    def live_peers(self):
        """Traffic view: peers heard recently, plus silent ones frozen where they were last heard (a ghost holds its
        reservations and never ages)."""
        t = wallclock.monotonic()
        out = [Peer(s.robot_id, s.pose.x, s.pose.y, list(s.reserved_nodes), s.wait_s)
               for s, rx in self.peers.values() if t - rx < self.peer_timeout]
        out += [Peer(rid, s.pose.x, s.pose.y, held, 0.0) for rid, (s, _, held) in self.ghosts().items()]
        return out

    def check_quorum(self):
        """Liveness rule 2. Logs transitions; while isolated the robot takes no new work and stops at its next node."""
        q = quorum(self.rid, self.heard(), self.fleet, self.released) if self.fleet else True
        if q != self.has_quorum:
            now = self.now()
            if q:
                self.get_logger().warn(f'{self.rid}: hears a quorum of the fleet again after '
                                       f'{now - self.isolated_since:.0f} s isolated; resuming')
                self.isolated_since = None
            else:
                self.isolated_since = now
                self.get_logger().error(f'{self.rid}: ISOLATED — hears only {sorted(self.heard())} of {self.fleet}; '
                                        f'stopping at my next held node')
            self.has_quorum = q
        return q

    def send(self, pub, msg):
        if not self.link_down:
            pub.publish(msg)

    # ------------------------------------------------------------------ main loop
    def tick(self):
        if not self.update_pose():
            return
        if self.last_node is None:
            self.last_node = self.graph.nearest(*self.pose[:2])
            self.reserved = [self.last_node]
            self.idle_since = self.now()
        if not self.check_peers():
            self.publish_state()           # heartbeat only until every peer is heard
            return
        self.check_quorum()
        self.check_requests()
        if self.has_quorum:                # isolated: no new work, no re-auctions, no energy moves (liveness rule 2)
            self.auction()
            self.check_station()
            self.reclaim_for_silent()
            self.update_ghost_edges()
            self.check_stuck()
            self.check_noroute()
        if self.phase == 'driving':
            self.drive()
        elif self.phase == 'dwelling':
            self.dwell()
        elif self.phase == 'retry' and self.now() >= self.retry_at:
            self.next_leg(first=True)
        elif self.phase == 'idle':
            self.reserved = [self.last_node]
            if self.has_quorum:
                self.idle_energy()
        self.energy_status()
        self.publish_state()

    def check_requests(self):
        """Action requests whose answer never came (a response dropped by DDS under load: seen as 'Failed to send goal
        response (timeout)' in Nav2 at fleet scale): ask again. Late answers to abandoned requests are ignored (tokens)."""
        t = wallclock.monotonic()
        if self.phase == 'routing' and self.route_req.overdue(t, self.route_timeout):
            self.get_logger().warn(f'{self.rid}: no answer from the route server in {self.route_timeout:.0f} s; asking again')
            self.route_req.clear()
            self.legs.insert(0, self.leg); self.leg = None
            self.phase, self.retry_at = 'retry', self.now()
        if self.bid_pending and self.bid_req.overdue(t, self.ack_timeout):
            self.bid_req.clear(); self.bid_pending = False     # the bid window has passed anyway
        if self.follow_req.overdue(t, self.ack_timeout):
            self.get_logger().warn(f'{self.rid}: FollowPath goal not acknowledged in {self.ack_timeout:.0f} s; resending')
            self.follow_req.clear()
            self.follow_end, self.follow_state = None, None    # drive() sends it again
        # result lost: goal accepted, robot standing still, no result for result_poll_s -> request the result again
        if self.follow_handle is not None and self.follow_state == GoalStatus.STATUS_EXECUTING and self.speed_est < 0.02:
            self.still_since = self.still_since or t
            if t - self.still_since > self.result_poll and t - self.result_polled_at > self.result_poll:
                self.result_polled_at = t
                self.get_logger().info(f'{self.rid}: no FollowPath result after {t - self.still_since:.0f} s standing '
                                       f'still; requesting it again')
                h = self.follow_handle
                h.get_result_async().add_done_callback(lambda r, h=h: self.on_follow_done(h, r))
        else:
            self.still_since = None

    def edges(self):
        """This robot's route_server adjust_edges client (None until the service is found)."""
        if self.edges_client is None:
            ns = self.get_namespace().rstrip('/')
            names = [n for n, _ in self.get_service_names_and_types() if n.startswith(ns + '/') and n.endswith('adjust_edges')]
            if names:
                self.edges_client = self.create_client(DynamicEdges, names[0])
        return self.edges_client

    def update_ghost_edges(self):
        """Routing avoids silent robots: the lanes into the nodes a ghost holds are closed in this robot's route
        server (reopened when it's heard again or released). A route already passing one is planned again."""
        ghosts = self.ghosts()
        held, cut = self.cut_off(ghosts)
        # lanes into a ghost's nodes, and the entrances of what it cuts off (lanes inside stay open so a robot caught
        # in there can still finish what it can)
        want = {eid: 1 for n in held for _, eid in self.in_edges.get(n, ())}
        want.update({eid: 1 for n in cut for a, eid in self.in_edges.get(n, ()) if a not in cut})
        trapped = self.trapped_nodes()
        if self.last_node in trapped and not self.trapped_logged:
            self.trapped_logged = True
            self.get_logger().error(f'{self.rid}: TRAPPED at {self.graph.name[self.last_node]}: every way out passes '
                                    f'silent {sorted(ghosts)} — waiting for an operator')
        elif self.last_node not in trapped:
            self.trapped_logged = False
        if want.keys() == self.ghost_closed.keys() or self.edges() is None:
            return
        closed = sorted(set(want) - set(self.ghost_closed))
        opened = sorted(set(self.ghost_closed) - set(want))
        self.edges_client.call_async(DynamicEdges.Request(closed_edges=closed, opened_edges=opened))
        self.ghost_closed = want
        self.get_logger().info(f'{self.rid}: routing around silent {sorted(ghosts) or "nobody"} '
                               f'({len(closed)} lanes closed, {len(opened)} reopened, {len(cut)} nodes cut off)')
        blocked = held | cut
        if closed and self.phase == 'driving' and self.leg is not None and self.next_idx > 0 and \
                blocked & set(self.route[self.next_idx:]) and self.route[self.next_idx] not in self.reserved:
            self.replan(f'my route passes silent {sorted(ghosts)}')

    def replan(self, why):
        """Plan the current leg again from the node just passed (the route server has changed lanes)."""
        self.get_logger().warn(f'{self.rid}: {why}; re-planning to {self.graph.name[self.leg.target]}')
        self.stop_following()
        self.legs.insert(0, self.leg)
        self.last_node, self.phase = self.route[self.next_idx - 1], 'rerouting'
        self.next_leg(first=True)

    def reclaim_for_silent(self):
        """Liveness rule 4: the task of a claimant silent for reclaim_silent_s (or released by an operator) is given
        back on its behalf by the lowest-id robot I can hear (myself included) — once per task round."""
        t = wallclock.monotonic()
        for rid, (s, rx) in self.peers.items():
            if t - rx < self.peer_timeout or not s.task_id or s.task_id in self.done:
                continue
            released = rid in self.released and self.released[rid] >= stamp_s(s.stamp)
            if s.loaded and (rid, s.task_id) not in self.stranded:
                self.stranded.add((rid, s.task_id))
                self.get_logger().error(f'{self.rid}: {rid} went silent with a box on its deck ({s.task_id}); '
                                        f'its task stays with it — a person has to recover the box')
            key = (s.task_id, s.task_round)
            if key in self.reclaimed or self.claims.get(s.task_id) not in (None, rid) or \
                    not reclaimable(t - rx, released, s.task_id, s.loaded, self.lp):
                continue
            if announcer(self.rid, self.heard()) != self.rid:
                continue
            self.reclaimed.add(key)
            self.claims.pop(s.task_id, None)
            self.get_logger().warn(f'{self.rid}: {rid} silent for {t - rx:.0f} s{" (released)" if released else ""}: '
                                   f'giving its task {s.task_id} back for re-auction')
            self.send(self.pub_claim, Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK,
                                            resource_id=s.task_id, robot_id=rid, action=Claim.RELEASE,
                                            round=s.task_round, by=self.rid))

    def check_noroute(self):
        """The leg's target has been unreachable for noroute_s (a silent robot cut it off): don't wait in the lane.
        Empty: give the task back (or drop the trip to a charger / parking spot). Loaded: wait on a parking spot and
        try again from there (the WMS may re-target the box to another bin meanwhile)."""
        if self.phase != 'retry' or self.noroute_since is None or self.now() - self.noroute_since < self.noroute_s:
            return
        target = self.legs[0].target if self.legs else None
        self.noroute_since = None
        if self.task is None:
            self.get_logger().warn(f'{self.rid}: {self.graph.name.get(target, target)} unreachable; choosing another spot')
            self.legs, self.leg = [], None
            self.phase, self.status, self.idle_since = 'idle', RobotState.IDLE, self.now() - self.idle_park_s
        elif not self.loaded:
            self.give_back(f'{self.graph.name.get(target, target)} unreachable for {self.noroute_s:.0f} s')
        elif not self.noroute_parked and self.last_node not in self.parking:
            spot = self.pick(self.last_node, self.parking)
            if spot is not None:
                self.noroute_parked = True
                self.get_logger().warn(f'{self.rid}: loaded, {self.graph.name.get(target, target)} unreachable; waiting at '
                                       f'{self.graph.name[spot]} out of the lanes')
                self.legs.insert(0, Leg(spot, 0.0, self.legs[0].drive_status if self.legs else RobotState.TO_DROP,
                                        final_precise=False))
                self.next_leg(first=True)

    def check_stuck(self):
        """Liveness rule 4: an empty robot that can't make progress for reclaim_stuck_s (controller failing, battery
        empty, blocked, or no route) gives its task back so another robot can do it."""
        stuck = self.status == RobotState.STUCK or self.wait_s > 0.0 or self.phase == 'retry'
        if self.task is None or self.loaded or not stuck:
            self.stuck_since = None
            return
        now = self.now()
        self.stuck_since = self.stuck_since or now
        if now - self.stuck_since > self.lp.reclaim_stuck_s:
            self.give_back(f'no progress for {now - self.stuck_since:.0f} s')

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
            if self.handshake and any(self.graph.kind.get(self.graph.id(n)) == 'bay' and self.station_down(self.graph.id(n))
                                      for n in (t.origin_id, t.dest_id) if n):
                continue                                  # its arm is out of service
            if any(self.graph.id(n) in b for n in (t.origin_id, t.dest_id) if n for b in self.cut_off()):
                continue                                  # a silent robot blocks it (the WMS withdraws it too)
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
        tok = self.bid_req.send(wallclock.monotonic())
        fut = self.route_client.send_goal_async(g)

        def got_handle(f):
            h = f.result()
            if not self.bid_req.current(tok):
                return                                   # abandoned (answer came too late)
            if not h.accepted:
                self.bid_req.clear(); self.bid_pending = False; return

            def got_result(r):
                if self.bid_req.ack(tok):
                    res = r.result().result
                    cb(self.graph.route_length([n.nodeid for n in res.route.nodes]) / self.speed
                       if res.error_code == 0 else None)
            h.get_result_async().add_done_callback(got_result)
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
        self.send(self.pub_bid, b)

    def claim_and_start(self, t):
        self.claims[t.task_id], self.claim_round[t.task_id] = self.rid, t.round
        self.set_charger(None)
        self.send(self.pub_claim, Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK,
                                     resource_id=t.task_id, robot_id=self.rid, action=Claim.CLAIM, round=t.round))
        o = self.graph.id(t.origin_id)
        if t.type == Task.GOTO:
            legs = [Leg(o, 0.0, RobotState.TO_PICK)]
        elif t.type == Task.FETCH_FROM_STATION:
            legs = [Leg(o, self.station_dwell, RobotState.TO_PICK, station=True),
                    Leg(self.graph.id(t.dest_id), self.bin_dwell, RobotState.TO_DROP, transfer=Transfer.ROBOT_TO_BIN)]
        else:
            legs = [Leg(o, self.bin_dwell, RobotState.TO_PICK, transfer=Transfer.BIN_TO_ROBOT),
                    Leg(self.graph.id(t.dest_id), self.station_dwell, RobotState.TO_DROP, station=True)]
        self.get_logger().info(f'{self.rid}: won {t.task_id} ({t.origin_id} -> {t.dest_id or "-"})')
        self.start_legs(legs, task=t)

    def abort_task(self):
        self.task, self.legs, self.loaded, self.stuck_since = None, [], False, None
        self.noroute_since, self.noroute_parked = None, False
        self.stop_following()
        self.phase = 'idle'; self.status = RobotState.IDLE; self.idle_since = self.now()

    # ------------------------------------------------------------------ legs
    def start_legs(self, legs, task):
        self.task, self.legs = task, list(legs)
        self.next_leg(first=True)

    def next_leg(self, first=False):
        if not first and self.leg is not None and self.task is not None and self.task.type != Task.GOTO and \
                self.leg.drive_status == RobotState.TO_PICK:
            self.loaded = True                 # the pickup leg is done: the box is on the deck
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
        tok = self.route_req.send(wallclock.monotonic())
        self.route_client.send_goal_async(g).add_done_callback(lambda f, tok=tok: self.on_route_handle(f, tok))

    def on_route_handle(self, f, tok):
        h = f.result()
        if not self.route_req.current(tok):
            return                                   # abandoned request (answered too late)
        if not h.accepted:
            self.get_logger().warn(f'{self.rid}: route request rejected, retrying')
            self.route_req.clear()
            self.legs.insert(0, self.leg); self.leg = None
            self.phase, self.retry_at = 'retry', self.now() + 2.0
            return
        h.get_result_async().add_done_callback(lambda r, tok=tok: self.on_route(r, tok))

    def on_route(self, f, tok):
        if not self.route_req.ack(tok):
            return
        res = f.result().result
        self.route_pending = False
        if self.reopen_edge is not None and self.edges_client is not None:   # the closure was only for this re-route
            if self.reopen_edge not in self.ghost_closed:
                self.edges_client.call_async(DynamicEdges.Request(opened_edges=[self.reopen_edge]))
            self.reopen_edge = None
        if res.error_code != 0 or not res.route.nodes:
            self.get_logger().warn(f'{self.rid}: no route to {self.graph.name[self.leg.target]} (error {res.error_code}), '
                                   f'retrying', throttle_duration_sec=30.0)
            self.noroute_since = self.noroute_since or self.now()
            self.legs.insert(0, self.leg); self.leg = None
            self.phase, self.retry_at = 'retry', self.now() + 2.0     # tick() re-plans the leg
            return
        self.noroute_since = None
        self.route = [n.nodeid for n in res.route.nodes]
        self.path = res.path
        if self.route[0] != self.last_node and self.pose is not None:
            self.prepend_lead_in(self.route[0])
        self.node_idx = self.index_nodes(self.route, self.path)
        self.stop_at = self.stop_nodes(self.route)
        self.next_idx = 1 if self.route[0] == self.last_node else 0
        self.phase, self.follow_end, self.wait_s = 'driving', None, 0.0

    def prepend_lead_in(self, n):
        """The route starts at the node ahead (re-planned while driving): prepend a straight stretch from where the
        robot is to that node, so the controller's path starts at the robot. Without it the first node counted as
        reached at once, and a route that doubled back past the robot on the other lane pulled MPPI onto that stretch
        (facing the wrong way: 'Failed to make progress' forever, 2026-09-25)."""
        x0, y0, _ = self.pose
        x1, y1 = self.graph.pos[n]
        L = math.hypot(x1 - x0, y1 - y0)
        if L < 0.1:
            return
        yaw = math.atan2(y1 - y0, x1 - x0)
        tmpl = self.path.poses[0] if self.path.poses else None
        lead = []
        for i in range(int(L / 0.05)):
            p = PoseStamped()
            p.header = tmpl.header if tmpl is not None else self.path.header
            p.pose.position.x, p.pose.position.y = x0 + (x1 - x0) * i * 0.05 / L, y0 + (y1 - y0) * i * 0.05 / L
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            lead.append(p)
        self.path.poses = lead + list(self.path.poses)

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
            # turning into a nose-in spur (charger / parking bay): stop on the lane, turn, drive straight in. Taking
            # the smoothed corner put the robot into the 1.7 m spur at an angle; forward-only, it can't remove that
            # offset in 1.7 m: 26 cm off a charger, 41 cm off a parking bay, final approach retried forever (2026-09-25)
            # same for any final approach that starts with a sharp turn less than 2.5 m before the goal: the first bay
            # of an aisle is 1.5 m past the turn off the street (0.32 m off at aisle_0_bay_0, 2026-09-25)
            elif turn >= math.radians(60) and k == len(route) - 2 and (
                    self.graph.kind.get(c) in ('charger', 'parking') or self.graph.dist(b, c) < 2.5):
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
        lo = self.node_idx[self.next_idx - 1] if self.next_idx > 0 else 0   # node 0 may lie ahead (lead-in)
        hi = min(len(pts), self.node_idx[min(self.next_idx + 1, len(self.route) - 1)] + 1)
        ridx = min(range(lo, max(hi, lo + 1)), key=lambda j: math.hypot(pts[j].pose.position.x - x, pts[j].pose.position.y - y))
        while self.next_idx < len(self.route) and self.node_reached(self.next_idx, ridx, x, y):
            if self.next_idx == len(self.route) - 1 and not self.final_reached(x, y):
                break   # the final node counts only when the controller reports arrival (or see final_reached)
            self.last_node = self.route[self.next_idx]
            self.next_idx += 1
            self.leg_fail = 0          # controller failures count per node
        if self.next_idx >= len(self.route):
            if self.follow_state == GoalStatus.STATUS_EXECUTING:
                self.stop_following()
            self.arrived(); return
        me = Peer(self.rid, x, y, self.reserved, self.wait_s)
        held = self.reserved
        self.reserved, self.blocked = plan_reservations(self.graph, self.route, self.next_idx, me, self.live_peers(), held,
                                                        self.tp, extend=self.has_quorum)
        if not self.has_quorum:
            # isolated: stop at the next node I hold (peers, which can't hear me, keep seeing my last reservation)
            nxt = self.route[self.next_idx]
            if nxt in self.reserved:
                self.reserved = self.reserved[:self.reserved.index(nxt) + 1]
        # drive only onto nodes my peers have acknowledged (the rest are published, waiting for their acks)
        ok = self.confirm()
        ahead = []
        for n in self.reserved:
            if n not in self.route[self.next_idx:]:
                continue
            if n not in ok:
                self.ack_waits += 1
                break
            ahead.append(n)
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
            # precise stop also where the robot turns in place right before a precise final approach: stopping
            # 0.25 m past the turn node (loose checker, arriving at street speed) carried a 0.3 m sideways offset
            # straight into the first bay of an aisle (2026-09-25)
            pre_final = end_k == len(self.route) - 2 and end_k in self.stop_at
            self.send_path(ridx, self.node_idx[end_k], (final or pre_final) and self.leg.precise)
            self.follow_end = end_node

    def node_reached(self, k, ridx, x, y):
        n = self.route[k]
        if self.follow_state == GoalStatus.STATUS_SUCCEEDED and self.follow_end == n:
            return True     # the controller finished a segment ending here (hold point / turn stop)
        d = math.hypot(self.graph.pos[n][0] - x, self.graph.pos[n][1] - y)
        if k in self.stop_at and k < len(self.route) - 1:
            # a turn stop counts only once the robot has actually stopped there — passing within 0.25 m while still
            # braking used to drop the stop, so the robot turned on the move (0.3 m off into aisle_0_bay_0)
            # ... or, after the controller gave up on it 3 times, anyway: a turn stop only exists so the robot turns in
            # place, never worth getting stuck on (0.36 m off park_5_access, facing away, retried forever, 2026-09-25)
            return self.follow_end == n and self.follow_state == GoalStatus.STATUS_ABORTED and (
                d < self.arrive_tol or self.leg_fail >= 3)
        return ridx >= self.node_idx[k] or d < 0.25

    def final_reached(self, x, y):
        """Arrival at the leg's last node: the controller succeeded; or the precise final approach gave up while the
        robot stands within `arrive_tol_m` — a forward-only robot can't remove a sideways offset once it is on top of
        the goal (MPPI aborts on no progress, forever: seen 2026-09-25 at 13 and 26 cm); or at a charger the BMS
        reports the contacts closed."""
        if self.follow_state == GoalStatus.STATUS_SUCCEEDED:
            return True
        n = self.route[-1]
        d = math.hypot(self.graph.pos[n][0] - x, self.graph.pos[n][1] - y)
        tol = self.arrive_tol_parking if n in self.parking else self.arrive_tol
        if self.follow_state == GoalStatus.STATUS_ABORTED and d < tol:
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
        """robot -> robot it waits for, for every yielding robot (from heartbeats + my own state); silent robots
        (ghosts) hold their frozen nodes and wait for nobody."""
        t = wallclock.monotonic()
        states = {rid: s for rid, (s, rx) in self.peers.items() if t - rx < self.peer_timeout}
        held = {rid: set(s.reserved_nodes) for rid, s in states.items()}
        ghosts = self.ghosts()
        held.update({rid: set(h) for rid, (_, _, h) in ghosts.items()})
        held[self.rid] = set(self.reserved)
        holder = lambda n, me: min((r for r, h in held.items() if r != me and n in h), default=None)
        w = {rid: holder(s.route[0], rid) for rid, s in states.items() if s.status == RobotState.YIELDING and s.route}
        if self.phase == 'driving' and self.next_idx < len(self.route):
            w[self.rid] = holder(self.route[self.next_idx], self.rid)
        return w, states, ghosts

    def maybe_break_deadlock(self):
        """A cycle of robots each waiting for the next never resolves by waiting: the highest id in the cycle
        re-routes around its blocked lane (every member computes the same cycle from the same heartbeats, so
        exactly one yields). Also re-route after a long block behind a robot that isn't moving (stuck), and after
        deadlock_after_s behind a silent one."""
        if not self.has_quorum or self.wait_s < self.deadlock_s or self.now() - self.last_reroute < 20.0 or self.next_idx == 0:
            return
        w, states, ghosts = self.waits()
        chain, cycle = wait_chain(self.rid, w)
        end = states.get(chain[-1])
        long_block = chain[-1] in ghosts or (self.wait_s > self.blocked_reroute_s and end is not None and (
            end.status == RobotState.STUCK or end.wait_s > self.blocked_reroute_s))
        if (cycle and self.rid in cycle and self.rid == max(cycle)) or long_block:
            why = f'deadlock cycle {cycle}' if cycle else f'blocked {self.wait_s:.0f}s behind {chain[1:]}'
            self.reroute_around(self.route[self.next_idx - 1], self.route[self.next_idx], why)

    def reroute_around(self, a, b, why):
        eid = self.graph.edge_id.get((a, b))
        if eid is None or self.leg is None or self.edges() is None:
            return
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
        self.follow_handle, self.still_since = None, None
        tok = self.follow_req.send(wallclock.monotonic())
        self.follow_client.send_goal_async(g).add_done_callback(lambda f, tok=tok: self.on_follow_handle(f, tok))

    def on_follow_handle(self, f, tok):
        h = f.result()
        if not self.follow_req.ack(tok):
            if h.accepted and self.follow_handle is not h:
                h.cancel_goal_async()        # abandoned (stopped, or resent after a lost ack): must not drive
            return
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
        self.follow_req.clear()              # a goal still waiting for its acceptance is cancelled when it arrives
        self.follow_handle, self.follow_end, self.follow_state = None, None, None

    def arrived(self):
        self.last_node = self.leg.target if self.leg else self.last_node
        self.reserved = [self.last_node]
        self.follow_handle, self.follow_end, self.follow_state = None, None, None
        if self.leg and self.leg.dwell_s > 0:
            self.phase, self.status = 'dwelling', RobotState.AT_STATION
            self.dwell_until = self.now() + self.leg.dwell_s
            if self.leg.transfer is not None and self.task is not None:
                if self.leg.transfer == Transfer.BIN_TO_ROBOT:
                    self.loaded = True           # the deck starts pulling the box in
                self.send(self.pub_transfer, Transfer(stamp=self.get_clock().now().to_msg(), kind=self.leg.transfer,
                                                   task_id=self.task.task_id, robot_id=self.rid, pallet=-1, slot=-1,
                                                   bin_id=self.task.bin_id, duration_s=float(self.leg.dwell_s)))
        else:
            self.next_leg()

    def dwell(self):
        """At a bin: the deck's fixed transfer time. At an arm bay: until the station reports this task served."""
        if not (self.leg.station and self.handshake):
            if self.now() >= self.dwell_until:
                self.next_leg()
            return
        st = self.station_state(self.leg.target)
        if st is not None and self.task is not None and st.task_id == self.task.task_id and \
                st.state == StationState.BUSY and self.leg.drive_status == RobotState.TO_PICK:
            self.loaded = True                   # the arm is putting the box on the deck
        if st is not None and self.task is not None and st.served_task == self.task.task_id:
            self.next_leg()

    def check_station(self):
        """An empty robot whose arm stays out of service (fault, or no heartbeat) gives its task back instead of
        blocking the bay or its queue; the WMS re-offers it once the arm is back. A loaded robot keeps waiting: it
        can't put the box anywhere else."""
        leg = self.leg
        if not (self.handshake and self.task is not None and leg is not None and leg.station
                and leg.drive_status == RobotState.TO_PICK and not self.loaded):
            self.station_down_since = None
            return
        if not self.station_down(leg.target):
            self.station_down_since = None
            return
        now = self.now()
        if self.station_down_since is None:
            self.station_down_since = now
            self.get_logger().warn(f'{self.rid}: {self.graph.name[leg.target]} is out of service')
        elif now - self.station_down_since > self.station_giveup:
            self.station_down_since = None
            self.give_back(f'{self.graph.name[leg.target]} out of service for {self.station_giveup:.0f} s')

    def give_back(self, why):
        """Hand my (empty-deck) task back to the WMS for re-auction and go idle."""
        t = self.task
        self.get_logger().warn(f'{self.rid}: {why}: giving {t.task_id} back')
        self.send(self.pub_claim, Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK,
                                        resource_id=t.task_id, robot_id=self.rid, action=Claim.RELEASE, round=t.round))
        self.claims.pop(t.task_id, None)
        self.leg, self.legs = None, []
        self.abort_task()

    def finish_task(self):
        t = self.task
        self.send(self.pub_claim, Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.TASK,
                                     resource_id=t.task_id, robot_id=self.rid, action=Claim.DONE, round=t.round))
        self.done.add(t.task_id)
        self.get_logger().info(f'{self.rid}: done {t.task_id} at {self.graph.name[self.last_node]}')
        self.task, self.leg, self.loaded, self.noroute_parked = None, None, False, False
        self.phase, self.status, self.idle_since = 'idle', RobotState.IDLE, self.now()

    # ------------------------------------------------------------------ charging & parking
    def robots_view(self, with_me=False):
        """Charger view: live robots as they report; a silent one keeps the charger it stands on (its held nodes)
        but heads for none, needs none and can't be asked to give one up (liveness rule 1)."""
        t = wallclock.monotonic()
        out = [ChargerPeer(m.robot_id, m.goal_node, list(m.reserved_nodes), m.battery, bool(m.task_id))
               for m, rx in self.peers.values() if t - rx < self.peer_timeout]
        out += [ChargerPeer(rid, m.last_node, held, 1.0, True) for rid, (m, _, held) in self.ghosts().items()]
        if with_me:
            out.append(ChargerPeer(self.rid, self.goal_node(), list(self.reserved), self.battery, self.task is not None))
        return out

    def pick(self, start, spots, current=None):
        held, cut = self.cut_off()
        if held:
            spots = [c for c in spots if c not in held and c not in cut]   # a silent robot blocks it or its way
        if set(spots) <= set(self.chargers) and self.bms and not self.must_charge and \
                waiting_for_charger(self.chargers, self.robots_view(), self.cp):
            return None                       # a robot low on battery waits for a charger: it goes first
        return pick_charger(self.graph, start, spots, self.rid, self.battery, self.robots_view(), current, self.cp)

    def set_charger(self, c):
        """Announce charger claims/releases on /swarm/claims (for observers; the heartbeat's goal_node is what
        peers decide on)."""
        if c == self.charger:
            return
        for node, action in ((self.charger, Claim.RELEASE), (c, Claim.CLAIM)):
            if node is not None:
                self.send(self.pub_claim, Claim(stamp=self.get_clock().now().to_msg(), resource_type=Claim.DOCK,
                                             resource_id=self.graph.name[node], robot_id=self.rid, action=action))
        self.charger = c

    def go_spot(self, c, why='', redock=False):
        """Drive to a charger or a parking spot (no task)."""
        is_charger = c in self.chargers
        self.set_charger(c if is_charger else None)
        st = RobotState.TO_CHARGE if self.must_charge and is_charger else RobotState.IDLE
        legs = [Leg(c, 0.0, st, final_precise=is_charger)]
        if redock:     # contacts didn't close: back out to the dock's access node and drive in again
            access = next(a for a, succ in self.graph.succ.items() if c in succ)
            legs.insert(0, Leg(access, 0.0, st, final_precise=False))
        verb = 're-docking at' if redock else 'to' if is_charger else 'parking at'
        self.get_logger().info(f'{self.rid}: {verb} {self.graph.name[c]} (battery {100 * self.battery:.0f} %'
                               f'{", must charge" if self.must_charge else ""}{", " + why if why else ""})')
        self.start_legs(legs, task=None)

    def go_park(self, start, why):
        spot = self.pick(start, self.parking)
        if spot is None:
            if not self.no_charger_logged:
                self.get_logger().warn(f'{self.rid}: no free charger or parking spot ({why})')
                self.no_charger_logged = True
            return False
        self.no_charger_logged = False
        self.go_spot(spot, why)
        return True

    def must_give_way(self):
        """Idle on (or heading for) a charger, charged >= resume, and a robot below `low` has none: the fullest
        such robots leave for parking (energy.evictions — every robot computes the same set)."""
        if not self.bms or self.must_charge or self.battery < self.cp.resume or not self.parking:
            return False
        return self.rid in evictions(self.chargers, self.robots_view(with_me=True), self.cp)

    def idle_energy(self):
        """Idle: top up on a free charger (still available for work), go charge because the battery is low, or wait
        on a parking spot when no charger is free. On a charger: make sure the contacts actually closed; give way
        to a waiting low robot once charged enough."""
        if self.task is not None:
            return
        now = self.now()
        if self.last_node in self.chargers:
            self.set_charger(self.last_node)
            if self.must_give_way():
                self.go_park(self.last_node, 'giving the charger to a robot low on battery')
                return
            if not self.bms or self.charging:
                self.dock_check_at, self.redocks = None, 0
            elif self.dock_check_at is None:
                self.dock_check_at = now
            elif now - self.dock_check_at > 10.0 and self.redocks < 2:
                self.get_logger().warn(f'{self.rid}: on {self.graph.name[self.last_node]} but not charging')
                self.redocks += 1
                self.dock_check_at = None
                self.go_spot(self.last_node, redock=True)
            return
        self.set_charger(None)
        if self.last_node in self.parking:
            # parked: move to a charger as soon as one is free, if charging would help
            if self.bms and (self.must_charge or self.battery < self.top_up_below):
                c = self.pick(self.last_node, self.chargers)
                if c is not None:
                    self.go_spot(c, 'a charger came free')
            return
        pending = self.pending_tasks()
        idle_for = now - self.idle_since
        if not (self.must_charge or idle_for > self.idle_park_s or
                (idle_for > self.idle_home_delay and
                 (not pending or (self.bms and not any(can_take(self.battery, self.task_need(t), self.cp) for t in pending))))):
            return
        c = self.pick(self.last_node, self.chargers)
        if c is not None:
            self.no_charger_logged = False
            self.go_spot(c)
        else:
            self.go_park(self.last_node, 'no free charger')

    def energy_status(self):
        if not self.bms:
            return
        if self.battery <= self.cp.critical and not self.charging and self.phase != 'empty':
            self.get_logger().error(f'{self.rid}: battery empty ({100 * self.battery:.1f} %), stopping here')
            self.stop_following(); self.phase, self.status = 'empty', RobotState.STUCK
            return
        if not self.has_quorum:
            return                                 # isolated: no charger decisions on a stale view
        leg = self.leg
        to_spot = self.task is None and leg is not None and self.phase in ('routing', 'driving') and not self.legs
        if to_spot and leg.target in self.chargers:
            leg.drive_status = RobotState.TO_CHARGE if self.must_charge else RobotState.IDLE
            if self.status in (RobotState.IDLE, RobotState.TO_CHARGE):
                self.status = leg.drive_status
            if self.phase == 'driving':
                if self.must_give_way():
                    self.go_park(self.start_node(), 'giving the charger to a robot low on battery')
                    return
                c = self.pick(self.start_node(), self.chargers, current=leg.target)
                if c is None:                             # lost a same-instant race and nothing else is free
                    self.go_park(self.start_node(), 'lost the charger')
                elif c != leg.target:
                    self.go_spot(c, 'lost the race for the other one')
        elif to_spot and leg.target in self.parking and self.phase == 'driving':
            spot = self.pick(self.start_node(), self.parking, current=leg.target)
            if spot is not None and spot != leg.target:
                self.go_spot(spot, 'lost the race for the other one')
        elif self.phase == 'idle' and self.task is None:
            self.status = RobotState.CHARGING if self.charging and self.battery < 0.999 else RobotState.IDLE

    # ------------------------------------------------------------------ heartbeat
    def goal_node(self):
        moving = self.leg is not None and self.phase in ('routing', 'driving', 'rerouting')
        return self.leg.target if moving else self.legs[0].target if self.legs else self.last_node

    def publish_state(self):
        m = RobotState()
        m.stamp = self.get_clock().now().to_msg()
        m.robot_id = self.rid
        m.status = RobotState.ISOLATED if not self.has_quorum else self.status
        self.seq += 1
        m.seq = self.seq
        for n in self.reserved:
            self.first_seq.setdefault(n, self.seq)       # first heartbeat that lists the node
        for n in [n for n in self.first_seq if n not in self.reserved]:
            del self.first_seq[n]                        # released: a new hold of it needs new acks
        heard = sorted((rid, s.seq) for rid, (s, _) in self.peers.items())
        m.heard_ids, m.heard_seq = [r for r, _ in heard], [int(q) for _, q in heard]
        m.pose = Pose2D(x=self.pose[0], y=self.pose[1], theta=self.pose[2])
        m.speed, m.battery = float(self.speed_est), float(self.battery)
        m.task_id = self.task.task_id if self.task else ''
        m.mission_id = self.task.mission_id if self.task else ''
        m.task_round = int(self.task.round) if self.task else 0
        m.loaded = bool(self.loaded)
        m.last_node = int(self.last_node)
        m.route = [int(n) for n in self.route[self.next_idx:]] if self.phase == 'driving' else []
        m.reserved_nodes = [int(n) for n in self.reserved]
        m.wait_s = float(self.wait_s)
        m.goal_node = int(self.goal_node())
        self.send(self.pub_state, m)


def main():
    rclpy.init()
    node = SwarmAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
