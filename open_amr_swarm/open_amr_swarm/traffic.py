"""Traffic control on the lane graph: node reservations (pure logic, no ROS — unit-tested).

Every robot publishes the lane-graph nodes it holds (`RobotState.reserved_nodes`) in its heartbeat. A robot may
only drive onto nodes it holds, so two robots never share a node: intersections, merges, following distance,
FIFO dock queues and exclusive bays/chargers all fall out of this one rule. Reservations are state, not events
(each heartbeat carries the full list), so a lost message can't leave a stale lock.

Rules, identical on every robot so both sides of a conflict reach the same decision from the same heartbeats:
  1. First come: a node can be added only if no live peer holds it or physically stands on it.
  2. Race (both added the same node in the same cycle): a robot already committed to the node (closer than
     `commit_dist`, i.e. inside its stopping distance) keeps it; otherwise the higher priority keeps it —
     longer waiting first (ageing, in `age_bucket_s` buckets so small clock skew can't flip it), then lower id.
  3. Deadlocks (a cycle of robots each waiting for the next) are broken by the highest robot id in the cycle,
     which re-routes around its blocked lane (see wait_chain and the agent).
  4. Look-ahead: hold nodes along the route until `horizon_m` of path is reserved (or `max_nodes`), so the
     controller cruises instead of stopping at every node, but never a node beyond horizon + `overreach_m` unless
     it is the very next one; release a node once `release_dist` past it.
"""
import math
from dataclasses import dataclass, field


@dataclass
class Peer:
    robot_id: str
    x: float
    y: float
    reserved: list = field(default_factory=list)
    wait_s: float = 0.0


@dataclass
class TrafficParams:
    horizon_m: float = 4.0
    max_nodes: int = 5
    commit_dist: float = 1.0      # inside this a robot can't stop before the node any more
    release_dist: float = 1.0     # drop a passed node once this far beyond it
    overreach_m: float = 2.0      # never hold a node further than horizon + this, except the very next one
    occupy_dist: float = 0.9      # a robot standing this close to a node blocks it even without a reservation
    age_bucket_s: float = 10.0


def priority(robot_id, wait_s, p: TrafficParams):
    """Sort key: smaller = higher priority (longer wait bucket first, then lower id)."""
    return (-int(wait_s // p.age_bucket_s), robot_id)


def wait_chain(me_id, waits):
    """Follow 'who am I waiting for' from me. waits: robot_id -> robot_id it waits on (or None).
    Returns (chain, cycle): the robots visited in order, and the cycle's members if the chain loops back."""
    chain, cur = [me_id], waits.get(me_id)
    while cur is not None and cur not in chain:
        chain.append(cur); cur = waits.get(cur)
    return chain, (chain[chain.index(cur):] if cur is not None else [])


def plan_reservations(graph, route, next_idx, me, peers, held, p: TrafficParams = TrafficParams()):
    """Decide which nodes this robot holds this cycle.

    graph: LaneGraph; route: node ids of the current leg; next_idx: index in `route` of the first node not yet
    reached; me: Peer for this robot (x, y, wait_s, robot_id); peers: live Peers; held: nodes held last cycle.
    Returns (reserved node list in route order, blocked) where blocked means the route continues past the last
    reserved node but the next node is taken.
    """
    x, y = me.x, me.y
    d_to = lambda n: math.hypot(graph.pos[n][0] - x, graph.pos[n][1] - y)
    reserved = []
    # the node just passed stays held until we're clear of it (our tail is still on it)
    if next_idx > 0 and d_to(route[next_idx - 1]) < p.release_dist:
        reserved.append(route[next_idx - 1])
    my_key = priority(me.robot_id, me.wait_s, p)
    dist_ahead, prev = 0.0, None
    for k in range(next_idx, len(route)):
        n = route[k]
        dist_ahead += d_to(n) if prev is None else graph.dist(prev, n)
        prev = n
        if dist_ahead > p.horizon_m + p.overreach_m and any(m not in route[:next_idx] for m in reserved):
            break      # a stuck robot 6.6 m short of a node held it and blocked the parking exit (2026-09-25)
        holders = [q for q in peers if n in q.reserved]
        # physical presence only counts for peers that hold no reservations (not cooperating / not an agent):
        # an agent's body is always covered by its own held nodes, and at compact junctions (lanes 1.1 m apart)
        # "stands within 0.9 m of a node" made two agents block each other's next node -> deadlock (2026-09-25)
        standing = [q for q in peers if not q.reserved and
                    math.hypot(graph.pos[n][0] - q.x, graph.pos[n][1] - q.y) < p.occupy_dist]
        if standing:
            return reserved, True
        if holders:
            if n not in held:
                return reserved, True                      # rule 1: first come
            i_commit = d_to(n) < p.commit_dist
            they_commit = any(math.hypot(graph.pos[n][0] - q.x, graph.pos[n][1] - q.y) < p.commit_dist
                              for q in holders)
            if not i_commit and (they_commit or any(priority(q.robot_id, q.wait_s, p) < my_key for q in holders)):
                return reserved, True                      # rule 2: race lost
        reserved.append(n)
        ahead = [m for m in reserved if m not in (route[:next_idx])]
        if (dist_ahead >= p.horizon_m and len(ahead) >= 1) or len(ahead) >= p.max_nodes:
            break
    return reserved, False
