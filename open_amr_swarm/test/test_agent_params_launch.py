"""swarm.launch.py agent_params: 'name:=value,...' -> typed swarm_agent parameters (A/B shifts without a rebuild)."""
import importlib.util
import os

import pytest

spec = importlib.util.spec_from_file_location(
    'swarm_launch', os.path.join(os.path.dirname(__file__), '..', 'launch', 'swarm.launch.py'))
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


def test_types():
    assert mod.agent_params('bid_strict_order:=true, strict_wait_s:=6.5,recover_max:=3,mode:=x') == \
        {'bid_strict_order': True, 'strict_wait_s': 6.5, 'recover_max': 3, 'mode': 'x'}


def test_empty():
    assert mod.agent_params('') == {} and mod.agent_params(' , ') == {}


def test_malformed_is_refused():
    with pytest.raises(ValueError):
        mod.agent_params('bid_strict_order=true')
