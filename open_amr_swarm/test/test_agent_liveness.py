"""Agent-level tests of the fail-safe core: heartbeats are fed straight into a SwarmAgent (no sim, no spinning) and
what it would publish is captured."""
import json
import time as wallclock

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Pose2D

from open_amr_msgs.msg import Claim, OperatorCommand, RobotState, Task
from open_amr_swarm.agent import SwarmAgent


@pytest.fixture
def agent(tmp_path, request):
    rid, fleet = getattr(request, 'param', ('amr_0', 4))
    feats = [{'type': 'Feature', 'properties': {'id': i, 'metadata': {'name': f'n{i}', 'kind': 'aisle_bay'}},
              'geometry': {'type': 'Point', 'coordinates': [1.0 * i, 0.0]}} for i in range(8)]
    feats += [{'type': 'Feature', 'properties': {'id': 100 + i, 'startid': i, 'endid': i + 1, 'metadata': {'kind': 'aisle'}},
               'geometry': {'type': 'MultiLineString', 'coordinates': [[[i, 0], [i + 1, 0]]]}} for i in range(7)]
    g = tmp_path / 'g.geojson'
    g.write_text(json.dumps({'features': feats}))
    rclpy.init(args=['--ros-args', '-p', f'graph:={g}', '-p', f'robot_id:={rid}',
                     '-p', f'fleet:={[f"amr_{i}" for i in range(fleet)]}'.replace("'", '"')])
    a = SwarmAgent()
    a.sent = []
    a.send = lambda pub, msg: a.sent.append(msg)
    a.pose, a.last_node, a.reserved, a.synced = (0.0, 0.0, 0.0), 0, [0], True
    yield a
    a.destroy_node()
    rclpy.shutdown()


def beat(a, rid, silent_s=0.0, stamp=100.0, **kw):
    """Feed a heartbeat of `rid` as if received `silent_s` wall seconds ago."""
    m = RobotState(robot_id=rid, stamp=TimeMsg(sec=int(stamp)), pose=Pose2D(x=kw.pop('x', 0.0), y=0.0), **kw)
    a.on_state(m)
    a.peers[rid] = (m, wallclock.monotonic() - silent_s)
    return m


def claims(a, action):
    return [m for m in a.sent if isinstance(m, Claim) and m.action == action]


def test_silent_peer_keeps_its_nodes_and_charger(agent):
    beat(agent, 'amr_1')
    beat(agent, 'amr_2', silent_s=10.0, x=3.5, reserved_nodes=[3, 4], route=[4, 5, 6], last_node=3)
    peers = {p.robot_id: p for p in agent.live_peers()}
    assert set(peers) == {'amr_1', 'amr_2'}              # the silent one is still there ...
    assert peers['amr_2'].reserved == [3, 4, 5, 6]       # ... holding its nodes + the stretch it may have added
    view = {r.robot_id: r for r in agent.robots_view()}
    assert view['amr_2'].busy and view['amr_2'].battery == 1.0 and view['amr_2'].goal_node == 3
    assert set(agent.ghosts()) == {'amr_2'}


def test_operator_release_frees_a_silent_robot(agent):
    beat(agent, 'amr_2', silent_s=10.0, stamp=100.0, reserved_nodes=[3])
    agent.on_operator(OperatorCommand(command=OperatorCommand.RELEASE_ROBOT, target='amr_2', stamp=TimeMsg(sec=150)))
    assert agent.ghosts() == {} and [p.robot_id for p in agent.live_peers()] == []
    beat(agent, 'amr_2', stamp=200.0)                    # heard again after the release: back in the fleet
    assert 'amr_2' not in agent.released


def test_isolated_robot_reports_it_and_takes_no_work(agent):
    beat(agent, 'amr_1')                                 # hears 1 of 3 peers: 2 of 4 is no majority
    beat(agent, 'amr_2', silent_s=10.0)
    beat(agent, 'amr_3', silent_s=10.0)
    assert not agent.check_quorum()
    agent.publish_state()
    assert agent.sent[-1].status == RobotState.ISOLATED
    beat(agent, 'amr_2')
    assert agent.check_quorum()                          # 3 of 4
    agent.publish_state()
    assert agent.sent[-1].status == RobotState.IDLE


def test_lowest_live_robot_gives_a_silent_claimants_task_back_once(agent):
    beat(agent, 'amr_1')
    beat(agent, 'amr_3')
    beat(agent, 'amr_2', silent_s=30.0, task_id='t0007', task_round=1)
    agent.reclaim_for_silent()
    agent.reclaim_for_silent()
    rel = claims(agent, Claim.RELEASE)
    assert len(rel) == 1
    assert (rel[0].resource_id, rel[0].robot_id, rel[0].by, rel[0].round) == ('t0007', 'amr_2', 'amr_0', 1)


def test_no_give_back_with_a_box_on_the_deck_or_too_early(agent):
    beat(agent, 'amr_1')
    beat(agent, 'amr_2', silent_s=30.0, task_id='t0007', loaded=True)
    beat(agent, 'amr_3', silent_s=5.0, task_id='t0008')
    agent.reclaim_for_silent()
    assert claims(agent, Claim.RELEASE) == []


@pytest.mark.parametrize('agent', [('amr_2', 4)], indirect=True)
def test_only_the_lowest_live_id_announces(agent):
    beat(agent, 'amr_1')
    beat(agent, 'amr_0', silent_s=30.0, task_id='t0007')  # silent: amr_1 is the lowest live one, not me
    beat(agent, 'amr_3')
    agent.reclaim_for_silent()
    assert claims(agent, Claim.RELEASE) == []


def test_task_given_back_on_my_behalf_is_dropped_unless_loaded(agent):
    agent.task = Task(task_id='t0007', round=1, type=Task.GOTO)
    agent.on_claim(Claim(resource_type=Claim.TASK, resource_id='t0007', robot_id='amr_0', action=Claim.RELEASE,
                         round=1, by='amr_1'))
    assert agent.task is None
    agent.task, agent.loaded = Task(task_id='t0008', round=0, type=Task.FETCH_FROM_BIN), True
    agent.on_claim(Claim(resource_type=Claim.TASK, resource_id='t0008', robot_id='amr_0', action=Claim.RELEASE,
                         round=0, by='amr_1'))
    assert agent.task is not None                        # the box is on my deck: I deliver it


@pytest.mark.parametrize('agent', [('amr_1', 6)], indirect=True)
def test_newer_auction_round_wins_over_lower_id(agent):
    agent.task = Task(task_id='t0007', round=0, type=Task.GOTO)
    beat(agent, 'amr_5', task_id='t0007', task_round=1)  # re-auctioned while I couldn't be heard
    assert agent.task is None and agent.claims['t0007'] == 'amr_5'


def test_link_down_drops_everything(agent):
    agent.link_down = True
    agent.on_state(RobotState(robot_id='amr_1'))
    agent.on_operator(OperatorCommand(command=OperatorCommand.RELEASE_ROBOT, target='amr_1'))
    assert agent.peers == {} and agent.released == {}


def test_wms_withdraws_work_under_a_silent_robot(tmp_path):
    import yaml
    from open_amr_swarm.mission_generator import MissionGenerator
    nodes = {'receiving_bay': {'id': 0, 'kind': 'bay'}, 'outbound_bay': {'id': 1, 'kind': 'bay'},
             'aisle_0_bay_0': {'id': 2, 'kind': 'aisle_bay'}, 'aisle_0_bay_1': {'id': 3, 'kind': 'aisle_bay'}}
    n = tmp_path / 'g.yaml'
    n.write_text(yaml.safe_dump(nodes))
    rclpy.init(args=['--ros-args', '-p', f'graph_nodes:={n}', '-p', f'out_dir:={tmp_path}', '-p', 'initial_fill:=1.0'])
    try:
        wms = MissionGenerator()
        sent = []
        wms.pub_claim.publish = sent.append
        wms.t0 = 0.0
        t = wms.new_task()                                   # outbound from a full bin
        wms.offer(t)
        bay = t.origin_id
        other = 'aisle_0_bay_1' if bay == 'aisle_0_bay_0' else 'aisle_0_bay_0'
        wms.on_state(RobotState(robot_id='amr_3', last_node=nodes[bay]['id'], reserved_nodes=[nodes[bay]['id']]))
        wms.robot_rx['amr_3'] -= 10.0                        # silent
        wms.withdraw_blocked()
        assert [c.action for c in sent] == [Claim.CANCEL] and wms.meta[t.task_id]['cancelled']
        assert all(b.startswith(other) for b in wms.pool(inbound=False))   # nothing new offered under it
        wms.on_operator(OperatorCommand(command=OperatorCommand.RELEASE_ROBOT, target='amr_3', stamp=TimeMsg(sec=5)))
        assert any(b.startswith(bay) for b in wms.pool(inbound=False))     # released: its bins are workable again
        wms.destroy_node()
    finally:
        rclpy.shutdown()


def test_wms_repairs_a_robot_back_from_a_network_cut(tmp_path):
    import yaml
    from open_amr_swarm.mission_generator import MissionGenerator
    nodes = {'receiving_bay': {'id': 0, 'kind': 'bay'}, 'outbound_bay': {'id': 1, 'kind': 'bay'},
             'aisle_0_bay_0': {'id': 2, 'kind': 'aisle_bay'}}
    n = tmp_path / 'g.yaml'
    n.write_text(yaml.safe_dump(nodes))
    rclpy.init(args=['--ros-args', '-p', f'graph_nodes:={n}', '-p', f'out_dir:={tmp_path}', '-p', 'initial_fill:=1.0',
                     '-p', 'reconcile_s:=0.0'])
    try:
        wms = MissionGenerator()
        sent = []
        wms.pub_claim.publish = sent.append
        wms.t0 = 0.0
        t = wms.new_task()
        wms.offer(t)
        wms.on_state(RobotState(robot_id='amr_1', task_id=t.task_id))      # its CLAIM got lost: adopted
        assert sent == [] and wms.meta[t.task_id]['robot'] == 'amr_1'
        wms.on_claim(Claim(resource_type=Claim.TASK, resource_id=t.task_id, robot_id='amr_1', action=Claim.RELEASE,
                           by='amr_0'))                                    # given back on its behalf (it was silent)
        wms.tasks[t.task_id].stamp = TimeMsg(sec=0)
        wms.tick()                                                         # re-offered: round 1
        assert wms.tasks[t.task_id].round == 1
        wms.on_state(RobotState(robot_id='amr_1', task_id=t.task_id))      # ... but it's back and still on it
        assert [(c.action, c.robot_id) for c in sent] == [(Claim.RELEASE, 'amr_1')]
        wms.meta[t.task_id]['cancelled'] = True
        wms.on_state(RobotState(robot_id='amr_1', task_id=t.task_id))
        assert sent[-1].action == Claim.CANCEL
        wms.destroy_node()
    finally:
        rclpy.shutdown()


def test_drives_only_onto_acknowledged_nodes(agent):
    """Liveness rule 3: a node just reserved is used only after every peer I hear has acknowledged it."""
    beat(agent, 'amr_1', seq=1, heard_ids=['amr_0'], heard_seq=[0])
    beat(agent, 'amr_2', seq=1, heard_ids=['amr_0'], heard_seq=[0])
    beat(agent, 'amr_3', seq=1, heard_ids=['amr_0'], heard_seq=[0])
    agent.reserved = [0, 1, 2]
    agent.publish_state()                            # heartbeat 1 lists nodes 0..2
    assert agent.confirm() == set()                  # nobody has heard it yet
    beat(agent, 'amr_1', seq=2, heard_ids=['amr_0'], heard_seq=[1])
    beat(agent, 'amr_2', seq=2, heard_ids=['amr_0'], heard_seq=[1])
    assert agent.confirm() == set()                  # amr_3 still hasn't
    beat(agent, 'amr_3', seq=2, heard_ids=['amr_0'], heard_seq=[1])
    assert agent.confirm() == {0, 1, 2}
    agent.reserved = [1, 2, 3]
    agent.publish_state()                            # heartbeat 2 adds node 3
    assert agent.confirm() == {1, 2}                 # 3 waits for acks of heartbeat 2
    sent_hb = [m for m in agent.sent if isinstance(m, RobotState)][-1]
    assert sent_hb.seq == 2 and sent_hb.heard_ids == ['amr_1', 'amr_2', 'amr_3'] and list(sent_hb.heard_seq) == [2, 2, 2]


def test_wms_alarms_on_a_silent_robot_with_a_box(tmp_path):
    import yaml
    from open_amr_msgs.msg import Transfer
    from open_amr_swarm.mission_generator import MissionGenerator
    nodes = {'receiving_bay': {'id': 0, 'kind': 'bay'}, 'outbound_bay': {'id': 1, 'kind': 'bay'},
             'aisle_0_bay_0': {'id': 2, 'kind': 'aisle_bay'}}
    n = tmp_path / 'g.yaml'
    n.write_text(yaml.safe_dump(nodes))
    rclpy.init(args=['--ros-args', '-p', f'graph_nodes:={n}', '-p', f'out_dir:={tmp_path}', '-p', 'initial_fill:=1.0'])
    try:
        wms = MissionGenerator()
        wms.pub_claim.publish = lambda m: None
        wms.t0 = 0.0
        t = wms.new_task()
        wms.offer(t)
        wms.on_claim(Claim(resource_type=Claim.TASK, resource_id=t.task_id, robot_id='amr_1', action=Claim.CLAIM))
        wms.on_transfer(Transfer(kind=Transfer.BIN_TO_ROBOT, task_id=t.task_id, robot_id='amr_1'))
        wms.on_state(RobotState(robot_id='amr_1', task_id=t.task_id, loaded=True))
        wms.check_stranded()
        assert not wms.meta[t.task_id]['stranded']
        wms.robot_rx['amr_1'] -= 30.0                        # silent 30 s, box on board
        wms.check_stranded()
        assert wms.meta[t.task_id]['stranded']
        wms.destroy_node()
    finally:
        rclpy.shutdown()


def test_unreachable_target_empty_robot_gives_task_back(agent):
    from open_amr_swarm.agent import Leg
    agent.task = Task(task_id='t0007', round=0, type=Task.FETCH_FROM_BIN, origin_id='n5', dest_id='n6')
    agent.legs, agent.phase = [Leg(5, 0.0, RobotState.TO_PICK)], 'retry'
    agent.noroute_since = agent.now() - 31.0
    agent.check_noroute()
    assert agent.task is None
    assert [(c.action, c.resource_id) for c in claims(agent, Claim.RELEASE)] == [(Claim.RELEASE, 't0007')]


def test_wms_retarget_moves_the_drop(agent):
    from open_amr_swarm.agent import Leg
    agent.task = Task(task_id='t0007', round=0, type=Task.FETCH_FROM_STATION, origin_id='n1', dest_id='n6')
    agent.loaded, agent.leg, agent.legs, agent.phase = True, None, [Leg(6, 4.0, RobotState.TO_DROP)], 'retry'
    agent.on_task(Task(task_id='t0007', round=0, type=Task.FETCH_FROM_STATION, origin_id='n1', dest_id='n3'))
    assert agent.task.dest_id == 'n3' and agent.legs[0].target == 3


def test_wms_retargets_a_loaded_inbound_box(tmp_path):
    import yaml
    from open_amr_msgs.msg import Transfer
    from open_amr_swarm.mission_generator import MissionGenerator
    nodes = {'receiving_bay': {'id': 0, 'kind': 'bay'}, 'outbound_bay': {'id': 1, 'kind': 'bay'},
             'aisle_0_bay_0': {'id': 2, 'kind': 'aisle_bay'}, 'aisle_0_bay_1': {'id': 3, 'kind': 'aisle_bay'}}
    n = tmp_path / 'g.yaml'
    n.write_text(yaml.safe_dump(nodes))
    rclpy.init(args=['--ros-args', '-p', f'graph_nodes:={n}', '-p', f'out_dir:={tmp_path}', '-p', 'initial_fill:=0.0'])
    try:
        wms = MissionGenerator()
        wms.pub_claim.publish = lambda m: None
        sent = []
        wms.pub_task.publish = sent.append
        wms.t0 = 0.0
        t = wms.new_task()                                   # inbound: receiving -> an empty bin
        assert wms.meta[t.task_id]['kind'] == 'inbound'
        wms.offer(t)
        wms.on_claim(Claim(resource_type=Claim.TASK, resource_id=t.task_id, robot_id='amr_1', action=Claim.CLAIM))
        wms.on_transfer(Transfer(kind=Transfer.PALLET_TO_ROBOT, task_id=t.task_id, robot_id='amr_1'))
        bay = t.dest_id
        wms.on_state(RobotState(robot_id='amr_1', task_id=t.task_id, loaded=True))
        wms.on_state(RobotState(robot_id='amr_5', last_node=nodes[bay]['id'], reserved_nodes=[nodes[bay]['id']]))
        wms.robot_rx['amr_5'] -= 10.0                        # a silent robot stands at that bin
        wms.retarget_blocked()
        assert sent[-1].task_id == t.task_id and sent[-1].dest_id != bay
        assert wms.meta[t.task_id]['bin'].split(':')[0] == sent[-1].dest_id
        n_sent = len(sent)
        wms.robot_rx['amr_1'] -= 10.0                        # the carrier goes silent: nothing to re-target
        wms.on_state(RobotState(robot_id='amr_5', last_node=nodes[sent[-1].dest_id]['id'],
                                reserved_nodes=[nodes[sent[-1].dest_id]['id']]))
        wms.robot_rx['amr_5'] -= 10.0
        wms.robot_rx['amr_1'] -= 10.0
        wms.retarget_blocked()
        assert len(sent) == n_sent
        wms.destroy_node()
    finally:
        rclpy.shutdown()
