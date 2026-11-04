"""Run a WMS against the mock fleet in-process, on sim time, as fast as the CPU allows (tests, stress runs).

A WMS endpoint is anything with `attach(bus)` (subscribe its callbacks) and `step(now)` (its timers). Two exist:
  MissionGeneratorOnBus   the current WMS stand-in, open_amr_swarm/mission_generator.py, unmodified: the rclpy node is
                          constructed (rclpy must be initialised) but never spun; its publishers are pointed at the bus,
                          its clock is set to the sim time (ROS time override, as /clock would), and the wall clock it
                          uses for heartbeat timeouts is replaced by the sim time (in Isaac 1 sim s ~ 1-4 wall s, here
                          ~0.01 wall s: wall-clock timeouts would never fire).
  open_amr_wms's WmsOnBus (open_amr_wms/harness.py) for the new WMS.
run() advances the fleet in `dt` steps, calls the WMS's timers, then delivers the step's messages.
"""
import time as _wall

from open_amr_msgs.msg import Claim, Inventory, OperatorCommand, RobotState, StationState, Task, Transfer


class _Pub:
    def __init__(self, bus, topic, name):
        self.bus, self.topic, self.name = bus, topic, name

    def publish(self, msg):
        self.bus.publish(self.topic, self.name, msg)


class _SimWall:
    """Stands in for the `time` module inside mission_generator: monotonic() = sim seconds."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t


class MissionGeneratorOnBus:
    name = 'mission_generator'

    def __init__(self, node):
        from rclpy.time import Time
        from open_amr_swarm import mission_generator as mg
        self.node, self._Time = node, Time
        self.wall = _SimWall()
        mg.wallclock = self.wall          # module-level `time as wallclock`
        clk = node.get_clock()
        clk._set_ros_time_is_active(True)
        clk.set_ros_time_override(Time(nanoseconds=1))
        self.next = {}
        self.logs = []

    def attach(self, bus):
        n = self.node
        n.pub_task = _Pub(bus, '/swarm/tasks', self.name)
        n.pub_claim = _Pub(bus, '/swarm/claims', self.name)
        n.pub_inv = _Pub(bus, '/wms/inventory', self.name)
        for topic, cb in (('/swarm/stations', n.on_station), ('/swarm/transfers', n.on_transfer),
                          ('/swarm/claims', n.on_claim), ('/swarm/state', n.on_state),
                          ('/swarm/operator', n.on_operator)):
            bus.subscribe(topic, self.name, cb)

    def set_time(self, now):
        self.wall.t = now
        self.node.get_clock().set_ros_time_override(self._Time(nanoseconds=int(now * 1e9)))

    def step(self, now):
        self.set_time(now)
        n = self.node
        for key, period, fn in (('tick', 0.5, n.tick), ('report', n.report_s, n.report), ('bat', 10.0, n.log_battery)):
            if now >= self.next.get(key, 0.0):
                self.next[key] = now + period
                fn()

    def done(self):
        return [m for m in self.node.meta.values() if m['done'] is not None and not m['cancelled']]


def run(fleet, wms, until_s, dt=0.1, start_s=1.0, on_step=None, stop=None):
    """Advance fleet + WMS from start_s to until_s sim seconds. on_step(now) after each step; stop(now) -> True ends
    early. Returns (sim seconds run, wall seconds taken)."""
    if not getattr(wms, '_attached', False):
        wms.attach(fleet.bus)
        wms._attached = True
    w0 = _wall.monotonic()
    now = start_s
    while now < until_s:
        fleet.step(now, dt)
        wms.step(now)
        fleet.bus.flush()
        if on_step is not None:
            on_step(now)
        if stop is not None and stop(now):
            break
        now = round(now + dt, 6)
    return now - start_s, _wall.monotonic() - w0


__all__ = ['MissionGeneratorOnBus', 'run', 'Claim', 'Inventory', 'OperatorCommand', 'RobotState', 'StationState',
           'Task', 'Transfer']
