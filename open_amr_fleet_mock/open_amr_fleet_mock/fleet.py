"""The mock fleet: N MockRobots + the two MockStations on one bus, the physical world they share, and failure
injection (scenario file or a command string). Pure logic, no rclpy: node.py puts it on real DDS topics, harness.py
runs it in-process against a WMS for tests and stress runs.

Defaults reproduce the Isaac fleet runs' configuration (open_amr_swarm/launch/swarm.launch.py and the c50_truth shift):
6 robots, receiving pallets 8 and 24 boxes, outbound 16 and 0, pallet swap 180 s, arm cycle 8 s, battery start spread
90 % .. 40 %, 2 chargers in service (charge_0, charge_1) at battery time scale 20.

Commands (scenario `do:` values, and the node's `inject` parameter "<do> <target> [for_s|rate]"):
  silent <robot> <s>      the robot's swarm link drops (nothing in or out); it stops where it is (no quorum alone)
  stuck <robot> <s>       the robot reports STUCK and doesn't move (after 120 s an empty one gives its task back)
  give_back <robot>       the robot hands its (empty-deck) task back: Claim RELEASE
  arm_fault <station> <s> the arm is out of service (StationState FAULT)
  starve <station> <s>    receiving: no boxes staged; outbound: no free pallet slot, for that long
  drop_rate all <p>       every message is lost with probability p per recipient from now on (0 = none)
"""
import yaml

from open_amr_swarm.energy import BatteryParams, ChargeParams
from open_amr_swarm.liveness import LivenessParams

from .bus import Bus
from .robot import MockRobot, MockTiming
from .stations import MockStation


class Fleet:
    def __init__(self, graph, robots=6, seed=1, chargers=('charge_0', 'charge_1'), battery_scale=20.0,
                 battery_start=None, receiving_pallets=(8, 24), outbound_pallets=(16, 0), swap_s=180.0, cycle_s=8.0,
                 drop_rate=0.0, timing=None, bus=None, scenario=None):
        self.graph = graph
        self.bus = bus or Bus(seed=seed, drop_rate=drop_rate)
        self.tm = timing or MockTiming()
        ids = [f'amr_{i}' for i in range(robots)]
        start = list(battery_start) if battery_start else [round(0.9 - 0.5 * i / max(robots - 1, 1), 2)
                                                             for i in range(robots)]
        charger_ids = [graph.id(c) for c in chargers] if chargers else graph.of_kind('charger')
        homes = charger_ids + [c for c in graph.of_kind('charger') if c not in charger_ids] + graph.of_kind('parking')
        homes += [n for n in graph.of_kind('street') if n not in homes]
        bp = BatteryParams(time_scale=battery_scale)
        self.robots = [MockRobot(r, ids, graph, homes[i], start[i % len(start)], self.bus, self.tm, bp, ChargeParams(),
                                 LivenessParams(), charger_ids, self) for i, r in enumerate(ids)]
        self.by_id = {r.rid: r for r in self.robots}
        self.stations = [MockStation('arm_receiving', 'receiving', 'receiving_bay', graph, self.bus,
                                     list(receiving_pallets), cycle_s, swap_s),
                         MockStation('arm_outbound', 'outbound', 'outbound_bay', graph, self.bus,
                                     list(outbound_pallets), cycle_s, swap_s)]
        self.by_station = {s.sid: s for s in self.stations}
        self.events = sorted(scenario or [], key=lambda e: float(e['at']))
        self.now = 0.0
        self.log = []

    # ------------------------------------------------------------------ the shared physical world
    def occupant(self, node, me):
        for r in self.robots:
            if r.rid != me and node in r.held():
                return r
        return None

    # ------------------------------------------------------------------ failures
    def inject(self, do, target='', value=0.0):
        """Apply one command (see module doc). Returns a one-line description, raises ValueError on a bad one."""
        value = float(value or 0.0)
        if do in ('silent', 'stuck', 'give_back'):
            r = self.by_id.get(target)
            if r is None:
                raise ValueError(f'no robot {target!r}')
            r.now = self.now
            if do == 'silent':
                r.go_silent(value)
            elif do == 'stuck':
                r.get_stuck(value)
            elif not r.hand_back():
                return f'{target} has no task it can give back'
        elif do in ('arm_fault', 'starve'):
            s = self.by_station.get(target)
            if s is None:
                raise ValueError(f'no station {target!r}')
            s.now = self.now
            (s.fault if do == 'arm_fault' else s.starve)(value)
        elif do == 'drop_rate':
            self.bus.drop_rate = value
        else:
            raise ValueError(f'unknown command {do!r}')
        text = f'inject {do} {target} {value:g}'.strip()
        self.log.append((self.now, text))
        return text

    def inject_line(self, line):
        parts = line.split()
        if not parts:
            raise ValueError('empty command')
        return self.inject(parts[0], parts[1] if len(parts) > 1 else '', parts[2] if len(parts) > 2 else 0.0)

    # ------------------------------------------------------------------ time
    def step(self, now, dt):
        self.now = now
        while self.events and float(self.events[0]['at']) <= now:
            e = self.events.pop(0)
            self.inject(e['do'], e.get('target', ''), e.get('for_s', e.get('value', 0.0)))
        for s in self.stations:
            s.step(now)
        for r in self.robots:
            r.step(now, dt)
            # a robot frozen by an injected STUCK ends it on time
            if r.stuck_until > 0 and now >= r.stuck_until:
                r.stuck_until = -1.0
                r.unstick()

    def drain_logs(self):
        out = self.log
        self.log = []
        for x in self.robots + self.stations:
            out += x.log
            x.log = []
        return sorted(out, key=lambda e: e[0])


def load_scenario(path):
    """A scenario file: YAML {events: [{at: s, do: <command>, target: id, for_s: s | value: x}, ...], ...}; the other
    keys are Fleet keyword arguments (robots, seed, drop_rate, chargers, ...)."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    events = data.pop('events', [])
    return events, data
