"""A turn stop stops on the lane at the node, with the incoming heading, and the next stretch starts there heading out
(the robot turns in place). On route_server's smoothed path the pose closest to the node is the arc's middle: the
stop came 0.17 m inside the corner at half the turn, and c64 amr_2 started its last 1.4 m into aisle 0 45 deg off the
aisle and drove into the west rack (2026-11-20). Geometry from the mission world: south_w_1 (-2.6, -9.75) ->
south_w_0 (-6.0, -9.75) -> aisle_0_bay_0 (-6.0, -8.25), smoothing radius 0.4 m."""
import json
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from open_amr_swarm.agent import SwarmAgent, square_corner

A, N, C = (-2.6, -9.75), (-6.0, -9.75), (-6.0, -8.25)
R = 0.4


def smoothed(step=0.05):
    """route_server's path: west along the street, a 0.4 m arc (centre (-5.6, -9.35)) turning right, north to the bay."""
    pts = []
    x = A[0]
    while x > N[0] + R + 1e-9:
        pts.append((x, A[1], math.pi)); x -= step
    cx, cy = N[0] + R, N[1] + R
    for i in range(1, 13):                       # angle -90 deg -> -180 deg about the centre, clockwise
        th = -math.pi / 2 - i * (math.pi / 2) / 12
        pts.append((cx + R * math.cos(th), cy + R * math.sin(th), th - math.pi / 2))
    y = N[1] + R + step
    while y <= C[1] + 1e-9:
        pts.append((N[0], y, math.pi / 2)); y += step
    return pts


def closest(pts, q):
    return min(range(len(pts)), key=lambda i: math.hypot(pts[i][0] - q[0], pts[i][1] - q[1]))


def ang(a, b):
    return abs(math.remainder(a - b, math.tau))


def test_smoothed_path_puts_the_turn_stop_inside_the_corner_at_half_the_turn():
    """Negative control: what the agent stopped on before the fix."""
    pts = smoothed()
    i = closest(pts, N)
    assert math.hypot(pts[i][0] - N[0], pts[i][1] - N[1]) > 0.15
    assert ang(pts[i][2], math.radians(135)) < math.radians(10)


def test_square_corner_puts_the_corner_back():
    pts = smoothed()
    i_s, i_e, seg, j = square_corner([(p[0], p[1]) for p in pts], closest(pts, N), A, N, C)
    assert seg[j][:2] == N and ang(seg[j][2], math.pi) < 1e-9          # stop on the node, facing west (incoming)
    assert all(ang(s[2], math.pi) < 1e-9 and abs(s[1] - N[1]) < 1e-9 for s in seg[:j])        # on the street axis
    assert all(ang(s[2], math.pi / 2) < 1e-9 and abs(s[0] - N[0]) < 1e-9 for s in seg[j + 1:])  # on the aisle axis
    assert len(seg) > j + 1
    # joins the untouched path without a gap: the arc lies inside the window, the straight parts outside it
    assert math.hypot(seg[0][0] - pts[i_s - 1][0], seg[0][1] - pts[i_s - 1][1]) < 0.06
    assert math.hypot(seg[-1][0] - pts[i_e + 1][0], seg[-1][1] - pts[i_e + 1][1]) < 0.06
    for p in pts[i_s:i_e + 1]:                   # every replaced pose was near the corner (the arc or its approach)
        assert math.hypot(p[0] - N[0], p[1] - N[1]) < 1.0


@pytest.fixture
def agent(tmp_path):
    names = [('south_w_1', 'street', A), ('south_w_0', 'street', N), ('aisle_0_bay_0', 'aisle_bay', C)]
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': nm, 'kind': kd}},
              'geometry': {'type': 'Point', 'coordinates': list(xy)}} for i, (nm, kd, xy) in enumerate(names)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'street'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[list(names[i][2]), list(names[i + 1][2])]]}}
              for i in range(2)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', 'robot_id:=amr_0', '-p', 'fleet:=["amr_0", "amr_1"]'])
    a = SwarmAgent()
    yield a
    a.destroy_node()
    rclpy.shutdown()


def to_path(pts):
    p = Path(); p.header.frame_id = 'map'
    for x, y, yaw in pts:
        ps = PoseStamped(); ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        p.poses.append(ps)
    return p


def yaw(ps):
    return 2 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w)


def test_agent_stops_on_the_node_facing_in_and_starts_the_last_stretch_facing_out(agent):
    agent.route, agent.path = [0, 1, 2], to_path(smoothed())
    agent.node_idx = agent.index_nodes(agent.route, agent.path)
    agent.stop_at = agent.stop_nodes(agent.route)
    assert agent.stop_at == {1}                  # 90 deg turn 1.5 m before the bay: a turn stop
    agent.square_stop_corners()
    k = agent.node_idx[1]
    stop = agent.path.poses[k]
    assert (stop.pose.position.x, stop.pose.position.y) == N and ang(yaw(stop), math.pi) < 1e-6
    nxt = agent.path.poses[k + 1]
    assert ang(yaw(nxt), math.pi / 2) < 1e-6 and abs(nxt.pose.position.x - N[0]) < 1e-9
    # the other nodes keep their poses: the street start and the bay
    assert agent.node_idx[0] == 0
    end = agent.path.poses[agent.node_idx[2]]
    assert math.hypot(end.pose.position.x - C[0], end.pose.position.y - C[1]) < 0.06
    # the path stays densely sampled (no gap the controller would cut across)
    pts = [(p.pose.position.x, p.pose.position.y) for p in agent.path.poses]
    assert max(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])) < 0.06
