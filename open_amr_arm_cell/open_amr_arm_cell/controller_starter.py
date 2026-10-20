#!/usr/bin/env python3
"""Bring a controller manager's controllers to active, state-based (in place of controller_manager's spawner).

The spawner retries a call whose reply did not arrive, but load_controller is not idempotent. Chain 39 base2: the
outbound cell's load reply was dropped under start-up load ("failed to send response to .../load_controller (timeout):
client will not receive response"), the retry got "A controller named 'joint_state_broadcaster' was already loaded"
and the spawner exited for good: no controllers, the cell never homed, 0 boxes all shift. This node reads the state
before every step (list_controllers) and only asks for the next transition (load -> configure -> activate), so a lost
reply costs one more round and nothing else.

    ros2 run open_amr_arm_cell controller_starter --ros-args -r __ns:=/arm_outbound \
        -p controllers:="['joint_state_broadcaster', 'joint_trajectory_controller']" [-p timeout:=120.0]
Exits 0 once every controller is active, 1 at the timeout (wall clock).
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ConfigureController, ListControllers, LoadController, SwitchController
from rclpy.node import Node


class Starter(Node):
    def __init__(self, **kwargs):
        super().__init__('controller_starter', **kwargs)
        self.controllers = [c for c in self.declare_parameter('controllers', ['']).value if c]
        self.timeout = self.declare_parameter('timeout', 120.0).value             # s, whole start-up
        self.call_timeout = self.declare_parameter('call_timeout', 5.0).value     # s, one service call
        cm = self.declare_parameter('controller_manager', 'controller_manager').value
        self.list = self.create_client(ListControllers, f'{cm}/list_controllers')
        self.load = self.create_client(LoadController, f'{cm}/load_controller')
        self.configure = self.create_client(ConfigureController, f'{cm}/configure_controller')
        self.switch = self.create_client(SwitchController, f'{cm}/switch_controller')

    def call(self, client, request):
        """The response, or None if the service isn't up or no reply came within call_timeout."""
        if not client.wait_for_service(timeout_sec=self.call_timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout)
        if not future.done():
            client.remove_pending_request(future)
            return None
        return future.result()

    def states(self):
        res = self.call(self.list, ListControllers.Request())
        return None if res is None else {c.name: c.state for c in res.controller}

    def step(self, states):
        """Ask for the next transition of every controller not yet active (the next round re-reads the result)."""
        for name in self.controllers:
            state = states.get(name)
            if state is None:
                res = self.call(self.load, LoadController.Request(name=name))
                self.get_logger().info(f'load {name}: {"ok" if res and res.ok else "no reply" if res is None else "refused"}')
            elif state == 'unconfigured':
                res = self.call(self.configure, ConfigureController.Request(name=name))
                self.get_logger().info(f'configure {name}: {"ok" if res and res.ok else "no reply" if res is None else "refused"}')
        inactive = [n for n in self.controllers if states.get(n) == 'inactive']
        if inactive:
            res = self.call(self.switch, SwitchController.Request(
                activate_controllers=inactive, strictness=SwitchController.Request.STRICT, activate_asap=True,
                timeout=Duration(sec=int(self.call_timeout))))
            self.get_logger().info(f'activate {inactive}: {"ok" if res and res.ok else "no reply" if res is None else "refused"}')

    def run(self):
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            states = self.states()
            if states is not None:
                if all(states.get(n) == 'active' for n in self.controllers):
                    self.get_logger().info(f'all active: {self.controllers}')
                    return 0
                self.step(states)
            time.sleep(0.2)
        self.get_logger().error(f'not all active after {self.timeout:.0f} s: {self.controllers}')
        return 1


def main():
    rclpy.init()
    node = Starter()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
