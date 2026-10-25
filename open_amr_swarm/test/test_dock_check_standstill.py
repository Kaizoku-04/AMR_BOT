"""The dock is judged from marker detections averaged at standstill, not from the docking server's dock pose.

Fleet runs c53_real / c50_real (2026-10-25, real inputs): a robot reported docked at 6 mm against the server's dock
pose (its filtered estimate from the approach, which the robot stopped within 5 mm of), while by the sim's truth it
stood 16 / 20 mm short; the cell's dock sensor (15 mm) then stopped the arm and the cell was out for the shift. At
standstill the detections agree with the truth to 0.5 mm (chain 58), so their mean is the reference."""
import json
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped

from open_amr_swarm.agent import SwarmAgent

BAY = (-14.0, 7.0, math.pi)                 # dock pose: base_footprint docked, facing the marker (-x)
TO_DOCK = 0.6


@pytest.fixture
def agent(tmp_path):
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(6)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(5)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_0', '-p', 'fleet:=["amr_0", "amr_1"]',
                     '-p', 'dock_bays:=true'])
    a = SwarmAgent()
    a.route, a.docking, a.leg, a.next_idx = [0, 1, 2, 3, 4], True, None, 4
    a.undock_sent = []
    a.undock_client.send_goal_async = lambda g: a.undock_sent.append(g) or \
        type('F', (), {'add_done_callback': lambda s, cb: None})()
    a.localized = []
    a.localize_at_bay = lambda name: a.localized.append(name)
    a.pose = (4.0, 0.0, 0.0)
    a.dock_state, a.settle_since = 'settling', a.now() - 2.0
    yield a
    a.destroy_node()
    rclpy.shutdown()


def robot_at(a, short_m):
    """The robot's own odom -> base_footprint: docked, but short_m short of the bay (towards the staging node)."""
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = 'odom', 'base_footprint'
    t.transform.translation.x = BAY[0] + short_m                # the bay faces -x: short = larger x
    t.transform.translation.y = BAY[1]
    t.transform.rotation.z, t.transform.rotation.w = math.sin(BAY[2] / 2), math.cos(BAY[2] / 2)
    a.tf.lookup_transform = lambda target, source, time: t


def server_dock_pose(a, robot_short_m, server_reads_m):
    """The docking server's dock pose, placed so the robot reads server_reads_m from it."""
    m = PoseStamped()
    m.header.frame_id = 'odom'
    m.pose.position.x, m.pose.position.y = BAY[0] + robot_short_m - server_reads_m, BAY[1]
    m.pose.orientation.z, m.pose.orientation.w = math.sin(BAY[2] / 2), math.cos(BAY[2] / 2)
    a.dock_pose_msg = m


def detections(a, n=25):
    """The marker apex where it really is: TO_DOCK beyond the bay pose along its heading (small scan noise)."""
    for k in range(n):
        m = PoseStamped()
        m.header.frame_id = 'odom'
        m.pose.position.x = BAY[0] + TO_DOCK * math.cos(BAY[2]) + 0.0006 * ((k % 5) - 2)
        m.pose.position.y = BAY[1] + TO_DOCK * math.sin(BAY[2])
        m.pose.orientation.z, m.pose.orientation.w = math.sin(BAY[2] / 2), math.cos(BAY[2] / 2)
        a.on_detection(m)


def test_c53_dock_16_mm_short_is_caught_at_standstill(agent):
    robot_at(agent, 0.016)
    server_dock_pose(agent, 0.016, 0.003)             # the server's pose says 3 mm (c53: 6 mm)
    detections(agent)
    err = agent.dock_error()
    assert abs(math.hypot(err[0], err[1]) - 0.016) < 0.001
    agent.check_dock('n4')
    assert len(agent.undock_sent) == 1 and agent.dock_state == 'undocking' and agent.dock_fails == 1
    assert agent.localized == []


def test_without_detections_the_server_pose_is_used(agent):
    """Negative control: the same dock with no detections falls back to the server's pose and is accepted (the old
    behaviour, which let c53's dock through)."""
    robot_at(agent, 0.016)
    server_dock_pose(agent, 0.016, 0.003)
    agent.check_dock('n4')
    assert agent.undock_sent == [] and agent.localized == ['n4']


def test_a_good_dock_is_accepted(agent):
    robot_at(agent, 0.004)
    server_dock_pose(agent, 0.004, 0.002)
    detections(agent)
    agent.check_dock('n4')
    assert agent.undock_sent == [] and agent.localized == ['n4']


def test_detections_from_before_the_robot_settled_are_not_used(agent):
    robot_at(agent, 0.016)
    server_dock_pose(agent, 0.016, 0.003)
    detections(agent)
    agent.settle_since = agent.now() + 1.0            # all detections predate the standstill
    assert agent.standstill_dock() is None
