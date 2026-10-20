"""controller_starter against a fake controller manager whose first load_controller reply is lost (chain 39 base2).

The negative control runs controller_manager's own spawner against the same fake: it retries the load, gets "already
loaded" and gives up, as the outbound cell's spawner did. HP test domain 91, LOCALHOST.
"""
import os
import subprocess
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '91'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

import pytest  # noqa: E402
import rclpy  # noqa: E402
from controller_manager_msgs.msg import ControllerState  # noqa: E402
from controller_manager_msgs.srv import (ConfigureController, ListControllers, LoadController,  # noqa: E402
                                         SwitchController)
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from open_amr_arm_cell.controller_starter import Starter  # noqa: E402

NAMES = ['joint_state_broadcaster', 'joint_trajectory_controller']


class FakeControllerManager(Node):
    """Loads, configures, activates like the real one; the first load reply arrives `lose_first` s late (lost)."""

    def __init__(self, ns, lose_first):
        super().__init__('controller_manager', namespace=ns)
        self.state, self.lose_first, self.loads = {}, lose_first, 0
        g = ReentrantCallbackGroup()
        self.create_service(ListControllers, 'controller_manager/list_controllers', self.on_list, callback_group=g)
        self.create_service(LoadController, 'controller_manager/load_controller', self.on_load, callback_group=g)
        self.create_service(ConfigureController, 'controller_manager/configure_controller', self.on_configure,
                            callback_group=g)
        self.create_service(SwitchController, 'controller_manager/switch_controller', self.on_switch, callback_group=g)

    def on_list(self, _, res):
        res.controller = [ControllerState(name=n, state=s, type='fake') for n, s in self.state.items()]
        return res

    def on_load(self, req, res):
        self.loads += 1
        res.ok = req.name not in self.state           # the real one: "A controller named ... was already loaded"
        if res.ok:
            self.state[req.name] = 'unconfigured'
        if self.loads == 1 and self.lose_first:
            time.sleep(self.lose_first)               # the reply comes after the caller stopped waiting: lost
        return res

    def on_configure(self, req, res):
        res.ok = self.state.get(req.name) in ('unconfigured', 'inactive')
        if res.ok:
            self.state[req.name] = 'inactive'
        return res

    def on_switch(self, req, res):
        res.ok = all(self.state.get(n) == 'inactive' for n in req.activate_controllers)
        if res.ok:
            for n in req.activate_controllers:
                self.state[n] = 'active'
        return res


@pytest.fixture
def fake_cm(request):
    rclpy.init()
    cm = FakeControllerManager('/arm_test', request.param)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(cm)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    yield cm
    ex.shutdown()
    cm.destroy_node()
    rclpy.try_shutdown()


@pytest.mark.parametrize('fake_cm', [0.0, 7.0], indirect=True)
def test_all_active_even_when_the_load_reply_is_lost(fake_cm):
    node = Starter(namespace='/arm_test', parameter_overrides=[
        Parameter('controllers', value=NAMES), Parameter('timeout', value=40.0), Parameter('call_timeout', value=2.0)])
    assert node.run() == 0
    assert all(fake_cm.state[n] == 'active' for n in NAMES)
    node.destroy_node()


@pytest.mark.parametrize('fake_cm', [12.0], indirect=True)
def test_negative_control_stock_spawner_gives_up(fake_cm):
    r = subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', *NAMES, '--controller-manager',
                        '/arm_test/controller_manager', '--controller-manager-timeout', '30'],
                       env=dict(os.environ, ROS_HOME='/tmp/ros_home_arm_test'), capture_output=True, text=True,
                       timeout=90)
    assert r.returncode != 0
    assert 'Failed loading controller' in r.stdout + r.stderr
    assert fake_cm.state['joint_state_broadcaster'] == 'unconfigured'      # loaded, never activated
