"""Reproducer (review 2026-11-05): a task dropped while its robot is docking (WMS CANCEL, a peer's outranking claim)
leaves the DockRobot goal running: abort_task() only cancels FollowPath (agent.py stop_following)."""
import json

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg

from open_amr_msgs.msg import Claim, Task
from open_amr_swarm.agent import SwarmAgent


class Handle:
    cancelled = False

    def cancel_goal_async(self):
        self.cancelled = True


@pytest.fixture
def agent(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'bay' if i == 7 else 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(8)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(7)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_0', '-p', 'dock_bays:=true'])
    a = SwarmAgent()
    a.sent = []
    a.send = lambda pub, msg: a.sent.append(msg)
    a.pose, a.last_node, a.reserved, a.synced = (6.5, 0.0, 0.0), 6, [6, 7], True
    yield a
    a.destroy_node()
    rclpy.shutdown()


def test_wms_cancel_while_docking_cancels_the_dock_goal(agent):
    agent.task = Task(task_id='t1', round=0, type=Task.FETCH_FROM_STATION, origin_id='n7', dest_id='n2')
    agent.loaded = False
    h = Handle()
    agent.dock_handle, agent.dock_state = h, 'docking'
    agent.on_claim(Claim(resource_type=Claim.TASK, resource_id='t1', robot_id='', action=Claim.CANCEL, by='wms',
                         stamp=TimeMsg(sec=5)))
    assert agent.task is None                 # the task is dropped ...
    assert h.cancelled, 'DockRobot goal still running after the task was dropped'   # ... the dock goal is not
