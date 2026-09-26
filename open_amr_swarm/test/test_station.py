"""Arm station model: handshake, pallet order and swaps, starvation, faults (station.py)."""
from open_amr_swarm.station import BUSY, FAULT, IDLE, STARVED, Station, StationParams


def run(st, t0, t1, at_bay, fault=False, dt=0.2):
    """Step from t0 to t1; returns the transfers started."""
    out, t = [], t0
    while t <= t1 + 1e-9:
        s = st.step(t, at_bay, fault)
        if s:
            out.append((t, s))
        t += dt
    return out


def test_receiving_serves_once_per_task():
    st = Station(StationParams('receiving', capacity=4, cycle_s=8.0), [3, 4])
    started = run(st, 0.0, 1.0, ('amr_1', 't1'))
    assert len(started) == 1
    s = started[0][1]
    assert (s.robot, s.task, s.pallet, s.slot) == ('amr_1', 't1', 0, 2)   # emptiest pallet first, its top box
    assert st.state == BUSY and st.pallets == [2, 4] and st.served == ''
    assert run(st, 1.2, 9.0, ('amr_1', 't1')) == []                       # completes, never restarts for t1
    assert st.served == 't1' and st.state == IDLE and st.transfers == 1
    s2 = run(st, 9.2, 9.4, ('amr_2', 't2'))[0][1]                        # next robot
    assert (s2.pallet, s2.slot) == (0, 1)


def test_receiving_swaps_empty_pallet_and_starves():
    st = Station(StationParams('receiving', capacity=2, cycle_s=1.0, swap_s=10.0), [1, 0])
    st.step(0.0, None, False)
    assert st.available() == 1                                             # pallet 1 being replaced
    run(st, 0.2, 1.4, ('amr_1', 't1'))
    assert st.served == 't1' and st.pallets == [0, 0]
    run(st, 1.6, 2.0, ('amr_2', 't2'))
    assert st.state == STARVED and st.robot == ''                          # robot waits at the bay
    started = run(st, 2.2, 10.4, ('amr_2', 't2'))                          # pallet 1 restocked at t=10
    assert len(started) == 1 and started[0][1].pallet == 1 and started[0][0] >= 10.0


def test_outbound_fills_fullest_then_ships():
    st = Station(StationParams('outbound', capacity=2, cycle_s=1.0, swap_s=5.0), [0, 1])
    s = run(st, 0.0, 0.2, ('amr_1', 't1'))[0][1]
    assert (s.pallet, s.slot) == (1, 1)                                    # fullest non-full pallet
    run(st, 0.4, 1.2, ('amr_1', 't1'))
    assert st.pallets == [0, 2] and st.available() == 2                   # full pallet awaits pickup
    run(st, 1.4, 6.4, None)
    assert st.pallets == [0, 0] and st.available() == 4                   # shipped, empty pallet staged


def test_fault_interrupts_and_resumes():
    st = Station(StationParams('receiving', capacity=4, cycle_s=4.0), [4, 4])
    run(st, 0.0, 1.0, ('amr_1', 't1'))
    assert st.pallets == [3, 4]
    run(st, 1.2, 5.0, ('amr_1', 't1'), fault=True)
    assert st.state == FAULT and st.pallets == [4, 4] and st.served == ''  # box back on the pallet
    started = run(st, 5.2, 10.0, ('amr_1', 't1'))
    assert len(started) == 1 and st.served == 't1'


def test_robot_leaving_aborts_transfer():
    st = Station(StationParams('receiving', capacity=4, cycle_s=4.0), [4, 4])
    run(st, 0.0, 1.0, ('amr_1', 't1'))
    run(st, 1.2, 1.4, None)
    assert st.state == IDLE and st.pallets == [4, 4] and st.served == ''
