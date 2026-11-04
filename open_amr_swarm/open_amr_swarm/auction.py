"""Auction rules every swarm_agent applies identically (pure logic, no ROS — unit-tested).

Moved out of agent.py (2026-11-05) so that anything else that has to play a robot at the message level (the mock fleet,
open_amr_fleet_mock) uses the very same rule instead of a copy; agent.py calls these functions and behaves exactly as
before (the bodies are the expressions agent.py had inline at `auction()` and `pending_tasks()`).
  winner       lowest cost, then lowest robot id: every agent evaluates the same bids of a closed round with it, so all
               agree on the winner without an auctioneer (Swarm Design "Auction", ADR-007).
  task_order   the order in which a robot considers open tasks: higher priority first, then older offer, then task id.
  closed       a round's bid window has passed (task stamp + window, sim time).
"""


def winner(bids):
    """bids: {robot_id: cost} of one (task_id, round). The robot every agent picks; None without bids."""
    if not bids:
        return None
    return min(bids.items(), key=lambda kv: (kv[1], kv[0]))[0]


def task_order(priority, stamp_s, task_id):
    """Sort key of an open task: higher priority, then older offer (stamp, s), then id."""
    return (-priority, stamp_s, task_id)


def closed(now_s, stamp_s, window_s):
    """True once the bid window of a round offered at `stamp_s` is over."""
    return now_s >= stamp_s + window_s
