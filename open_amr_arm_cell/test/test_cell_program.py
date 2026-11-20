"""Unit tests for the cell program's deck location (ADR-048): DEPALLETIZE places where the cell's camera sees the deck."""
import pytest

from open_amr_arm_cell import cell_program as prog
from open_amr_arm_cell.carton_locator import LocateFault
from open_amr_arm_cell.cell_geometry import Cell

T = dict(slot_above=[0.0] * 6, deck_hold=[0.0] * 6, grip_yaw=0.0, deck_yaw=180.0, via=False)
DECK_STEPS = ('down to deck', 'place on deck', 'clear deck')


def _by_name(steps):
    return {s.name: s for s in steps}


def test_an_offset_deck_shifts_all_three_deck_steps():
    cell = Cell((0.0, 0.0, 0.0))
    nom, off = _by_name(prog.depalletize(cell, 0, 3, T)), _by_name(prog.depalletize(cell, 0, 3, T,
                                                                                   deck_offset=(0.011, -0.004, 1.4)))
    for name in DECK_STEPS:
        (x0, y0, z0, w0), (x1, y1, z1, w1) = nom[name].pose, off[name].pose
        assert (x1 - x0, y1 - y0, z1 - z0, w1 - w0) == pytest.approx((0.011, -0.004, 0.0, 1.4))
    assert off['down to deck'].locate == ('deck_frame',)
    assert nom['down to deck'].locate == ('deck_frame',)           # located in every depalletize, offset or not


def test_the_pick_offset_is_kept_when_the_deck_offset_is_added():
    cell = Cell((0.0, 0.0, 0.0))
    pick = (0.007, 0.002, -0.8)
    a = _by_name(prog.program(cell, True, 0, 3, T, offset=pick))
    b = _by_name(prog.program(cell, True, 0, 3, T, offset=pick, deck_offset=(0.011, 0.0, 1.4)))
    for name in ('down to slot', 'lift'):
        assert b[name].pose == a[name].pose
    assert a['down to slot'].pose != _by_name(prog.program(cell, True, 0, 3, T))['down to slot'].pose


def test_palletize_is_unchanged_by_a_deck_offset():
    cell = Cell((0.0, 0.0, 0.0))
    assert prog.program(cell, False, 0, 3, T, deck_offset=(0.02, 0.0, 0.0)) == prog.program(cell, False, 0, 3, T)


class _Loc:
    def __init__(self, r):
        self.r = r

    def locate(self, target, nominal, timeout=1.0):
        assert target == ('deck_frame',)
        if isinstance(self.r, Exception):
            raise self.r
        self.nominal = nominal
        return self.r


def _motion(loc):
    from open_amr_arm_cell.cell_motion import CellMotion
    cell = Cell((0.0, 0.0, 0.0))
    m = object.__new__(CellMotion)
    m.cell, m.p, m.locator, m.logs = cell, cell.p, loc, []
    m.log = m.logs.append
    m.deck_frame = None
    m.set_scene = lambda *a, **k: None
    m.pick_offset = (0.007, 0.002, -0.8)
    m.cycle = (True, 0, 3, T)
    m.steps = prog.program(cell, True, 0, 3, T, offset=m.pick_offset)
    m.pc = next(i for i, s in enumerate(m.steps) if s.name == 'down to deck')
    return m


def test_locate_deck_rebuilds_the_program_keeping_the_pick_offset_and_resumes_at_the_same_step():
    loc = _Loc((0.011, -0.004, 1.4))
    m = _motion(loc)
    before = _by_name(m.steps)
    m._locate(m.steps[m.pc])
    after = _by_name(m.steps)
    assert m.steps[m.pc].name == 'down to deck'
    assert after['lift'].pose == before['lift'].pose and after['down to slot'].pose == before['down to slot'].pose
    assert after['place on deck'].pose[0] - before['place on deck'].pose[0] == pytest.approx(0.011)
    assert m.deck_frame == (0.011, -0.004, 1.4)
    assert m.logs == ['deck located +11 / -4 mm, +1.4 deg: place corrected']


def test_locate_deck_beyond_limits_or_without_detection_faults_instead_of_placing():
    from open_amr_arm_cell.cell_motion import CellFault
    for err in (LocateFault('deck out of place by 40 mm / +0.0 deg (correctable: 30 mm / 5 deg)'),
                LocateFault('no deck detections for 1.0 s')):
        m = _motion(_Loc(err))
        with pytest.raises(CellFault, match='at the deck'):
            m._locate(m.steps[m.pc])
        assert m.deck_frame is None
