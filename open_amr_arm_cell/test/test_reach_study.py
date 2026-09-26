"""Integration tests of the reach study (MoveIt 2 + Pilz, the planner the cell runs). Skipped without moveit_py.
Each runs the study in its own process (one MoveItPy per process; it can't be torn down cleanly in-process)."""
import os
import subprocess
import sys

import pytest

pytest.importorskip('moveit.planning')

LAYOUT = os.path.join(os.environ.get('OPENAMR_ROOT', os.path.expanduser('~/Robotics/OpenAMR')), 'sim', 'configs',
                      'warehouse_layout.yaml')
pytestmark = pytest.mark.skipif(not os.path.exists(LAYOUT), reason='OpenAMR layout not found')


def study(*args, tmp):
    return subprocess.run([sys.executable, '-m', 'open_amr_arm_cell.reach_study', '--layout', LAYOUT,
                           '--out', str(tmp / 'p.yaml'), *args], capture_output=True, text=True, timeout=600)


def test_collision_checking_and_determinism(tmp_path):
    """A move only the carried box collides on fails; taught points are deterministic."""
    r = study('--check', tmp=tmp_path)
    assert r.returncode == 0 and 'self-check: OK' in r.stdout, r.stdout[-2000:]


def test_whole_pattern_is_reachable(tmp_path):
    """Gate of Phase 4b: every slot of both pallets, both directions, collision-checked."""
    r = study(tmp=tmp_path)
    assert r.returncode == 0, r.stdout[-2000:]
    assert 'reach study: 96/96' in r.stdout
