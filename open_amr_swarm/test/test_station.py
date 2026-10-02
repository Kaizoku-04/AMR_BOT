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


# ------------------------------------------------------------------ CellStation (a real cell controller ends the jobs)
from open_amr_swarm.station import (C_DECK, C_LOST, C_PALLET, C_TOOL, DEPALLETIZE, PALLETIZE, PREPARE,  # noqa: E402
                                    CellStation)


def test_cell_receiving_prepicks_then_hands_over():
    st = CellStation(StationParams('receiving', capacity=4), [3, 4])
    j = st.step(0.0, None, False)
    assert (j.kind, j.pallet, j.slot) == (PREPARE, 0, 2) and st.pallets == [2, 4]   # carton leaves when picked
    assert st.step(0.2, ('amr_1', 't1'), False) is None                            # one job at a time
    st.finished(1.0, True, C_TOOL)
    assert st.held == (0, 2) and st.available() == 7
    j = st.step(1.2, ('amr_1', 't1'), False)
    assert (j.kind, j.pallet, j.slot, j.robot, j.task) == (DEPALLETIZE, 0, 2, 'amr_1', 't1') and st.state == BUSY
    assert st.served == ''
    st.deck_clear(2.0)
    assert st.served == 't1' and st.transfers == 1                                  # robot may leave
    st.finished(4.0, True, C_DECK)
    assert st.held is None and st.state == IDLE
    j = st.step(4.2, ('amr_1', 't1'), False)                                         # still there: not served twice
    assert j.kind == PREPARE and (j.pallet, j.slot) == (0, 1)


def test_cell_robot_leaving_before_dock_keeps_the_carton_held():
    st = CellStation(StationParams('receiving', capacity=4), [3, 4])
    st.step(0.0, None, False)
    st.finished(1.0, True, C_TOOL)
    st.step(1.2, ('amr_1', 't1'), False)
    st.cancelled(3.0)
    assert st.held == (0, 2) and st.state == IDLE and st.pallets == [2, 4]
    assert st.step(3.2, None, False) is None                                         # nothing new to pre-pick


def test_cell_outbound_counts_the_carton_when_stacked():
    st = CellStation(StationParams('outbound', capacity=2, swap_s=10.0), [1, 0])
    j = st.step(0.0, ('amr_2', 't2'), False)
    assert (j.kind, j.pallet, j.slot) == (PALLETIZE, 0, 1)                           # fullest first, next free slot
    st.deck_clear(1.0)
    assert st.served == 't2' and st.pallets == [1, 0]
    st.finished(3.0, True, C_PALLET)
    assert st.pallets == [2, 0] and st.swap_at[0] == 13.0                            # full: pickup due


def test_cell_faults_put_the_carton_where_the_cell_says():
    st = CellStation(StationParams('outbound', capacity=4), [0, 0])
    st.step(0.0, ('amr_2', 't2'), False)
    st.finished(1.0, False, C_DECK, 'interlock')                                     # never left the robot's deck
    assert st.state == FAULT and st.pallets == [0, 0] and st.served == ''
    assert st.step(1.2, ('amr_2', 't2'), True) is None
    j = st.step(5.0, ('amr_2', 't2'), False)                                         # reset: the robot is served again
    assert j.kind == PALLETIZE and j.task == 't2'
    st.deck_clear(6.0)
    st.finished(7.0, False, C_LOST, 'vacuum lost')                                   # dropped after the robot left
    assert st.lost == 1 and st.pallets == [0, 0] and st.served == 't2'
    rc = CellStation(StationParams('receiving', capacity=4), [4, 4])
    rc.step(0.0, ('amr_1', 't1'), False)                                             # no pre-pick: picks for the robot
    assert rc.pallets == [3, 4]
    rc.finished(1.0, False, C_PALLET, 'carton out of place')                        # never picked: back on the pallet
    assert rc.pallets == [4, 4]
    rc.step(2.0, ('amr_1', 't1'), False)
    rc.finished(3.0, False, C_DECK, 'interlock')                                     # on the deck, arm not clear yet
    assert rc.served == ''
    rc.step(3.2, ('amr_1', 't1'), True)
    j = rc.step(9.0, ('amr_1', 't1'), False)                                         # reset: served without a new pick,
    assert rc.served == 't1' and j.kind == PREPARE and rc.pallets == [2, 4]          # then the next pre-pick
