"""Liveness: what a robot assumes about a peer it can no longer hear, and about itself when it can't hear the fleet.
Pure logic, no ROS — unit-tested. Design: OpenAMR notes/Swarm Design.md "Fail-safe core".

A robot can't tell a dead peer from one that is alive but cut off, so every rule here is safe for both:
  1. Silent peer = still there. A peer whose heartbeat stopped keeps, for traffic, the nodes it last reserved, the
     route nodes within `ghost_extra_m` past them (a margin; rule 3 is what makes the reserved list complete) and the
     node nearest its last pose. It keeps the charger it stands on, but never counts as
     heading for one, asking for one, or able to give one up. It stays so until it is heard again or an operator
     releases it (OperatorCommand.RELEASE_ROBOT, which also takes it out of the quorum count).
  2. Quorum. A robot moves on, bids and re-auctions only while it hears a majority of the fleet (itself included).
     Without quorum it stops at the next node it holds (ISOLATED) and waits. After a network split only the majority
     side keeps moving, and it sees the minority frozen at its last reservations — which is where the minority
     stops. A fleet split in half stops on both sides. A single cut-off robot is the minority of one.
  3. Acknowledged reservations. A robot drives onto a newly reserved node only once every peer it hears has
     acknowledged a heartbeat of its that lists the node (`RobotState.seq`, `heard_ids`/`heard_seq`). So whatever it
     may drive on, its peers have received — the ghost view of it is complete even if its link drops the next
     instant, and a one-way cut (it hears them, they don't hear it) just stops it. Costs ~0.2-0.4 s before a node
     just taken can be used. (Replaces a 'fresh view' rule: a scenario test showed a robot grab 3 m of lane in the tick
     after its link was cut, from the last view it had, and drive it while its peers never saw it — 2026-09-26.)
  4. Re-auction. The task of a claimant silent for `reclaim_silent_s` (or released by an operator) is given back
     (Claim.RELEASE, `by` = announcer) by the lowest-id robot that has quorum — unless its last heartbeat says a box
     is on its deck (`loaded`): then only a person can recover it. A robot stuck (STUCK status, or blocked) for
     `reclaim_stuck_s` gives its own task back, again only while empty.
"""
import math
from dataclasses import dataclass


@dataclass
class LivenessParams:
    peer_timeout_s: float = 3.0     # heartbeat older than this (wall s): the peer is silent
    ghost_extra_m: float = 2.0      # a silent peer also holds its route this far past its last reserved node
    reclaim_silent_s: float = 20.0  # a silent claimant's task is re-auctioned after this
    reclaim_stuck_s: float = 120.0  # a robot stuck this long gives its task back


def quorum(me, heard, fleet, released=()):
    """True if `me` plus the peers it hears are a strict majority of the fleet. heard: peer ids heard within the
    peer timeout; fleet: every robot id (me may be included); released: ids an operator took out of service (they
    count again when heard)."""
    members = {r for r in fleet if r != me and (r not in released or r in heard)}
    return 1 + len(members & set(heard)) > (1 + len(members)) / 2


def ghost_hold(graph, reserved, route, x, y, extra_m):
    """Nodes a silent peer is taken to hold: its reserved nodes, its route's nodes up to `extra_m` past the last
    reserved one (at least one), and the node nearest its last pose. route: its remaining route from the heartbeat."""
    held = list(dict.fromkeys(reserved))
    last = max((k for k, n in enumerate(route) if n in held), default=-1)
    dist, prev = 0.0, route[last] if last >= 0 else None
    for n in route[last + 1:]:
        dist += graph.dist(prev, n) if prev is not None else math.hypot(graph.pos[n][0] - x, graph.pos[n][1] - y)
        if held and dist > extra_m and n != route[last + 1]:
            break
        held.append(n)
        prev = n
    here = graph.nearest(x, y)
    if here not in held:
        held.append(here)
    return held


def acked(first_seq, me, peer_states):
    """Rule 3: every peer heard (their latest heartbeats) reports having received my heartbeat number `first_seq`
    (or a later one). first_seq None = never published yet."""
    if first_seq is None:
        return False
    for s in peer_states:
        heard = dict(zip(s.heard_ids, s.heard_seq))
        if heard.get(me, -1) < first_seq:
            return False
    return True


def announcer(me, live):
    """The robot that announces re-auctions: the lowest id among me and the peers I hear (all robots with quorum
    compute the same one from the same heartbeats)."""
    return min([me, *live])


def reclaimable(silent_for, released, task_id, loaded, p: LivenessParams):
    """A silent peer's task may be given back on its behalf."""
    return bool(task_id) and not loaded and (released or silent_for >= p.reclaim_silent_s)


def outranks(a, b):
    """Two robots on one task (lost claims, a re-auction the original claimant didn't hear): which keeps it.
    a, b: (loaded, round, robot_id). A box on the deck wins, then the newer auction round, then the lower id."""
    return (not a[0], -a[1], a[2]) < (not b[0], -b[1], b[2])


class Pending:
    """One request to an action server whose acknowledgement or result may be lost (DDS under load). Tokens tell a
    late answer to an abandoned request from the current one."""

    def __init__(self):
        self.token, self.sent_at, self.tries = 0, None, 0

    def send(self, now, retry=False):
        self.token += 1
        self.sent_at = now
        self.tries = self.tries + 1 if retry else 1
        return self.token

    def ack(self, token):
        """True if `token` is the outstanding request (it's now answered); False for a stale one."""
        if token != self.token or self.sent_at is None:
            return False
        self.sent_at = None
        return True

    def current(self, token):
        return token == self.token

    def overdue(self, now, timeout):
        return self.sent_at is not None and now - self.sent_at > timeout

    def clear(self):
        self.token += 1
        self.sent_at = None
