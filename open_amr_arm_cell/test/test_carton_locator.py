"""Unit tests for the carton locator's matching (open_amr_arm_cell/carton_locator.py)."""
import pytest

from open_amr_arm_cell.carton_locator import LocateFault, match


def test_nearest_detection_gives_the_offset():
    nominal = (1.0, 2.0, 0.5, 30.0)
    dets = [(1.36, 2.0, 0.5, 30.0), (1.02, 1.99, 0.5, 31.5), (1.0, 2.0, 0.25, 30.0)]   # neighbour, ours, one below
    dx, dy, dyaw = match(dets, nominal, 0.03, 5.0)
    assert dx == pytest.approx(0.02) and dy == pytest.approx(-0.01) and dyaw == pytest.approx(1.5)


def test_square_carton_yaw_is_modulo_90():
    assert match([(1.0, 2.0, 0.5, 30.0 + 90.0 - 2.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)[2] == pytest.approx(-2.0)


def test_out_of_place_and_missing_cartons_fault():
    with pytest.raises(LocateFault, match='out of place'):
        match([(1.06, 2.0, 0.5, 30.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)
    with pytest.raises(LocateFault, match='out of place'):
        match([(1.0, 2.0, 0.5, 38.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)
    with pytest.raises(LocateFault, match='no carton'):
        match([(1.0, 2.0, 0.25, 30.0)], (1.0, 2.0, 0.5, 30.0), 0.03, 5.0)     # only the layer below
