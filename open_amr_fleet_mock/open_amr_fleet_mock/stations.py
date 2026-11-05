"""Mock arm station: open_amr_swarm.station.Station (the model station_agent runs when cells:=false) on the bus.

Identical handshake to station_agent.py: the robot waiting at the bay is learned from heartbeats (AT_STATION on the
bay node with a task, heard within peer_timeout), a transfer is announced on /swarm/transfers when it starts, and
StationState goes out at 5 Hz. Failure injection: fault(for_s) (FAULT, the arm out of service) and starve(for_s)
(receiving: the staged pallets are empty and the next truck is late; outbound: no free slot until the pallets are
picked up), both from a scenario file or the node's `inject` parameter.
"""
from open_amr_msgs.msg import RobotState, StationState, Transfer

from open_amr_swarm.station import Station, StationParams

from .robot import to_time


class MockStation:
    def __init__(self, sid, role, bay_name, graph, bus, pallets, cycle_s=8.0, swap_s=180.0, capacity=24,
                 peer_timeout_s=3.0):
        self.sid, self.bay_name, self.bay, self.bus = sid, bay_name, graph.id(bay_name), bus
        self.st = Station(StationParams(role=role, capacity=capacity, cycle_s=cycle_s, swap_s=swap_s), pallets)
        self.robots = {}
        self.peer_timeout = peer_timeout_s
        self.fault_until = -1.0
        self.now, self.next_hb = 0.0, 0.0
        self.log = []
        bus.subscribe('/swarm/state', sid, self.on_state)

    def on_state(self, m):
        self.robots[m.robot_id] = (m, self.now)

    def at_bay(self):
        here = sorted((m.robot_id, m.task_id) for m, rx in self.robots.values()
                      if self.now - rx < self.peer_timeout and m.last_node == self.bay
                      and m.status == RobotState.AT_STATION and m.task_id)
        return here[0] if here else None

    def fault(self, for_s):
        self.fault_until = self.now + for_s
        self.log.append((self.now, f'{self.sid}: TEST fault for {for_s:.0f} s'))

    def starve(self, for_s):
        """Every pallet spent (receiving empty / outbound full), each back `for_s` from now."""
        s = self.st
        for i in range(len(s.pallets)):
            s.pallets[i] = 0 if s.receiving else s.p.capacity
            s.swap_at[i] = self.now + for_s
        self.log.append((self.now, f'{self.sid}: TEST starved for {for_s:.0f} s'))

    def step(self, now):
        self.now = now
        if now < self.next_hb - 1e-6:
            return
        self.next_hb = now + 0.2
        started = self.st.step(now, self.at_bay(), now < self.fault_until)
        self.log += [(t, f'{self.sid}: {text}') for t, text in self.st.log]
        self.st.log.clear()
        if started:
            recv = self.st.receiving
            self.bus.publish('/swarm/transfers', self.sid, Transfer(
                stamp=to_time(now), kind=Transfer.PALLET_TO_ROBOT if recv else Transfer.ROBOT_TO_PALLET,
                task_id=started.task, robot_id=started.robot, station_id=self.sid, pallet=started.pallet,
                slot=started.slot, duration_s=float(self.st.p.cycle_s)))
        s = self.st
        self.bus.publish('/swarm/stations', self.sid, StationState(
            stamp=to_time(now), station_id=self.sid, bay_node=self.bay_name, state=s.state, claimed_by=s.robot,
            task_id=s.task, served_task=s.served, boxes_available=s.available(), pallets=[int(n) for n in s.pallets],
            capacity=s.p.capacity, transfers=s.transfers, cycle_s=float(s.p.cycle_s)))
