"""The auction rules moved out of agent.py into auction.py (2026-11-05) so the mock fleet uses the same ones: lowest cost
then lowest id wins, tasks are considered by priority, then offer time, then id, and a round closes at stamp + window."""
from open_amr_swarm import auction


def test_lowest_cost_wins():
    assert auction.winner({'amr_2': 10.0, 'amr_0': 12.5, 'amr_1': 11.0}) == 'amr_2'


def test_equal_cost_lower_id_wins():
    assert auction.winner({'amr_3': 5.0, 'amr_1': 5.0}) == 'amr_1'


def test_no_bids_no_winner():
    assert auction.winner({}) is None


def test_task_order_priority_then_age_then_id():
    tasks = [(1, 20.0, 't3'), (2, 30.0, 't9'), (1, 10.0, 't5'), (1, 10.0, 't4')]
    order = sorted(tasks, key=lambda t: auction.task_order(*t))
    assert [t[2] for t in order] == ['t9', 't4', 't5', 't3']


def test_round_closes_at_stamp_plus_window():
    assert not auction.closed(101.4, 100.0, 1.5)
    assert auction.closed(101.5, 100.0, 1.5)
