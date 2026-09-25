"""Energy: the battery model (sim side) and the charging policy (agent side). Pure logic, no ROS — unit-tested.

Battery (OpenAMR spec: 24 V, 48-56 Ah, "8 hours life"): 24 V x 50 Ah = 1.2 kWh. Power draw = hotel load (compute,
lidar, electronics) + drive load when the wheels turn, rising with speed and turning rate. At the fleet's measured
duty (82 % driving at ~0.45 m/s average, 2026-09-25 run) that is ~150 W, i.e. ~8 h per charge, matching the spec.
Charger: 900 W (0.75 C), constant power up to 80 % then tapering (CV phase) — 30 -> 80 % in ~40 min.
`time_scale` compresses battery time for demos (20 = an 8 h shift in 24 min); every rate scales, nothing else.

Charging policy (every agent runs the same rules; no charging coordinator):
  - Opportunity charging: an idle robot with no open work parks on a free charger and tops up. It stays
    available: it leaves as soon as it wins a task.
  - Below `low` a robot takes no new work; after its current task it goes to the nearest free charger and
    stays until `resume`. Charging is never started while carrying a box or at a station (Mission Design §8).
  - Energy-aware bidding: a robot bids on a task only if, after the task and the drive to the nearest charger
    (estimated from the lane graph and its own measured drain rate, with a safety factor), at least `reserve`
    is left. So nobody accepts a job it cannot finish. Bids carry a small penalty for low charge so work drifts
    to fuller robots.
  - Chargers are claimed through the heartbeat (`goal_node`): a charger is free if no live peer stands on it
    (holds it) or heads for it. Two robots picking the same one in the same instant: the one below `low`
    keeps it, else the lower id — every robot computes the same answer.
"""
import math
from dataclasses import dataclass


@dataclass
class BatteryParams:
    capacity_wh: float = 1200.0      # 24 V x 50 Ah
    nominal_v: float = 24.0
    hotel_w: float = 50.0            # compute, lidar, IMU, electronics: always on
    drive_w: float = 40.0            # drives enabled and moving
    drive_w_per_mps: float = 150.0   # traction, rising with speed
    turn_w_per_rads: float = 40.0    # in-place turns scrub the casters
    charge_w: float = 900.0          # into the battery
    taper_from: float = 0.80         # CV phase: charge power falls linearly to `taper_floor` at 100 %
    taper_floor: float = 0.1
    time_scale: float = 1.0


def draw_w(v, w, p: BatteryParams):
    """Electrical load at linear speed v (m/s) and turn rate w (rad/s)."""
    moving = abs(v) > 0.02 or abs(w) > 0.05
    return p.hotel_w + (p.drive_w + p.drive_w_per_mps * abs(v) + p.turn_w_per_rads * abs(w) if moving else 0.0)


def charge_power_w(soc, p: BatteryParams):
    if soc <= p.taper_from:
        return p.charge_w
    return p.charge_w * max(p.taper_floor, 1.0 - (1.0 - p.taper_floor) * (soc - p.taper_from) / (1.0 - p.taper_from))


def step_soc(soc, dt, v, w, on_charger, p: BatteryParams):
    """Advance the state of charge by dt seconds. Returns (soc, battery current in A; negative = discharging)."""
    net_w = -draw_w(v, w, p)
    if on_charger and soc < 1.0:
        net_w += charge_power_w(soc, p)
    soc = min(1.0, max(0.0, soc + net_w * dt * p.time_scale / 3600.0 / p.capacity_wh))
    return soc, net_w / p.nominal_v


@dataclass
class ChargeParams:
    low: float = 0.30          # below: take no new work, charge after the current task
    resume: float = 0.60       # a robot sent to charge by `low` works again from here
    reserve: float = 0.12      # a task must leave this much after reaching a charger
    critical: float = 0.03     # battery empty: the robot stops where it is
    safety: float = 1.3        # margin on the energy estimate
    plan_speed: float = 0.45   # m/s average incl. yielding and speed zones (measured 2026-09-25)
    bid_weight_s: float = 10.0  # bid penalty at 0 % charge, linear


def task_need(graph, start, stops, dwell_s, drain_per_s, chargers, p: ChargeParams):
    """Estimated state-of-charge fraction to drive start -> stops... -> nearest charger, plus the dwells."""
    legs = [start] + list(stops)
    length = sum(graph.travel(a, b) for a, b in zip(legs, legs[1:]))
    length += min((graph.travel(legs[-1], c) for c in chargers), default=0.0)
    if math.isinf(length):
        return math.inf
    return (length / p.plan_speed + dwell_s) * drain_per_s * p.safety


def can_take(soc, need, p: ChargeParams):
    return soc - need >= p.reserve


def bid_penalty_s(soc, p: ChargeParams):
    return p.bid_weight_s * (1.0 - min(max(soc, 0.0), 1.0))


@dataclass
class ChargerPeer:
    robot_id: str
    goal_node: int
    reserved: list
    battery: float


def pick_charger(graph, start, chargers, me_id, me_battery, peers, current=None, p: ChargeParams = ChargeParams()):
    """Nearest free charger from `start` (lane-graph length), or None if all are taken. `current`: the charger I
    already head for — kept unless a peer that outranks me heads for it too (race within one heartbeat)."""
    rank = lambda rid, soc: (0 if soc < p.low else 1, rid)
    mine = rank(me_id, me_battery)
    free = []
    for c in chargers:
        if any(c in q.reserved for q in peers):
            continue                                   # someone stands on it (or is just pulling out)
        rivals = [q for q in peers if q.goal_node == c]
        if rivals and (c != current or any(rank(q.robot_id, q.battery) < mine for q in rivals)):
            continue
        free.append(c)
    if current in free:
        return current
    reach = [(graph.travel(start, c), c) for c in free]
    reach = [rc for rc in reach if not math.isinf(rc[0])]
    return min(reach)[1] if reach else None


class DrainEstimator:
    """Measured discharge rate while working (fraction per second), EMA over windows of `window_s`."""

    def __init__(self, prior_per_s, window_s=30.0):
        self.rate, self.window = prior_per_s, window_s
        self.t0 = self.soc0 = None

    def update(self, t, soc, working):
        if not working:
            self.t0 = None
            return
        if self.t0 is None:
            self.t0, self.soc0 = t, soc
            return
        if t - self.t0 >= self.window:
            r = (self.soc0 - soc) / (t - self.t0)
            if r > 0:
                self.rate = 0.6 * self.rate + 0.4 * r
            self.t0, self.soc0 = t, soc
