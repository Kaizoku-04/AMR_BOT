"""Unit tests for the palletizing cell geometry (open_amr_arm_cell/cell_geometry.py)."""
import math

import pytest

from open_amr_arm_cell.cell_geometry import Cell, CellParams


def test_pattern_fits_the_pallet_and_boxes_do_not_overlap():
    c = Cell((0.0, 0.0, 0.0))
    p = c.p
    assert p.capacity == 24 and p.per_layer == 6
    for i in (0, 1):
        (cx, cy), yaw = c.pallet_cell(i)
        tops = [c.slot_cell(i, k)[0] for k in range(p.capacity)]
        u = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        v = (-u[1], u[0])
        for x, y, z in tops:           # box footprint inside the pallet footprint
            du = (x - cx) * u[0] + (y - cy) * u[1]
            dv = (x - cx) * v[0] + (y - cy) * v[1]
            assert abs(du) + p.box[0] / 2 <= p.pallet_size[0] / 2 + 1e-9
            assert abs(dv) + p.box[1] / 2 <= p.pallet_size[1] / 2 + 1e-9
        for a in range(p.capacity):    # boxes of a layer at least one pitch apart
            for b in range(a + 1, p.capacity):
                (xa, ya, za), (xb, yb, zb) = tops[a], tops[b]
                if abs(za - zb) < 1e-9:
                    assert math.hypot(xa - xb, ya - yb) >= p.box[1] + p.gap - 1e-9


def test_layers_stack_from_the_pallet_top():
    c = Cell((0.0, 0.0, 0.0))
    assert c.slot_cell(0, 0)[0][2] == pytest.approx(0.144 + 0.25)
    assert c.slot_cell(0, 23)[0][2] == pytest.approx(0.144 + 4 * 0.25)
    with pytest.raises(ValueError):
        c.slot_cell(0, 24)


def test_pallets_mirror_and_clear_pedestal_and_amr():
    c = Cell((0.0, 0.0, 0.0))
    (x0, y0), _ = c.pallet_cell(0)
    (x1, y1), _ = c.pallet_cell(1)
    assert x0 == pytest.approx(x1) and y0 == pytest.approx(-y1) and y0 > 0
    # nearest pallet corner stays clear of the pedestal and of the docked AMR (0.64 wide, nose at deck - 0.403)
    for i in (0, 1):
        (cx, cy), yaw = c.pallet_cell(i)
        u = (math.cos(math.radians(yaw)), math.sin(math.radians(yaw)))
        v = (-u[1], u[0])
        corners = [(cx + su * 0.6 * u[0] + sv * 0.4 * v[0], cy + su * 0.6 * u[1] + sv * 0.4 * v[1])
                   for su in (-1, 1) for sv in (-1, 1)]
        assert min(math.hypot(x, y) for x, y in corners) > c.p.pedestal_size / 2 * math.sqrt(2)
        amr_x0 = c.p.deck_distance - 0.403
        assert all(abs(y) > 0.32 + 0.2 or x < amr_x0 for x, y in corners)


def test_world_frame_and_bay_pose():
    c = Cell((-14.85, 7.0, 0.0))                      # receiving: bay east of the arm
    x, y, yaw = c.bay_pose()
    assert (x, y) == pytest.approx((-14.0, 7.0)) and abs(yaw) == pytest.approx(180.0)
    o = Cell((14.85, 7.0, 180.0))                    # outbound: mirror
    x, y, yaw = o.bay_pose()
    assert (x, y) == pytest.approx((14.0, 7.0)) and abs(yaw) == pytest.approx(0.0, abs=1e-9)
    (wx, wy, wz), _ = c.slot_world(0, 0)
    (cx, cy, cz), _ = c.slot_cell(0, 0)
    assert (wx - -14.85, wy - 7.0, wz) == pytest.approx((cx, cy, cz))


def test_params_from_layout():
    p = CellParams.from_layout({'arm_cell': {'pedestal_height': 1.2, 'pattern': [3, 2, 3]},
                                'box': {'size': [0.35, 0.35, 0.3]}})
    assert p.pedestal_height == 1.2 and p.capacity == 18 and p.box == (0.35, 0.35, 0.3)
