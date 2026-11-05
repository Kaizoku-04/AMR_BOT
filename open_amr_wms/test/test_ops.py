"""Operations: alarms in plain language with their cause, KPIs, the shift report, operator commands with who and why,
and the dashboard's endpoints (served without any external file)."""
import os

import pytest

from open_amr_msgs.msg import Claim, OperatorCommand

from open_amr_fleet_mock.fleet import Fleet
from open_amr_fleet_mock.harness import run
from open_amr_wms.api import WEB, Api
from open_amr_wms.ops import Ops

from conftest import Tap


def setup(graph, wms, scenario, until, **kw):
    f = Fleet(graph, seed=4, scenario=scenario)
    tap, w = Tap(f), wms(orders=True, **kw)
    ops = Ops(w.node, w.orders, graph)
    w.node.state_observers.append(ops.on_state)
    run(f, w, until)
    return f, tap, w, ops


def test_alarms_name_the_robot_the_place_the_time_and_the_cause(graph, wms):
    f, tap, w, ops = setup(graph, wms, [{'at': 100, 'do': 'stuck', 'target': 'amr_3', 'for_s': 1000},
                                        {'at': 120, 'do': 'silent', 'target': 'amr_5', 'for_s': 1000},
                                        {'at': 150, 'do': 'arm_fault', 'target': 'arm_outbound', 'for_s': 1000}], 200)
    text = [a['text'] for a in ops.alarms()]
    assert any(t.startswith('amr_3 stuck at ') and '(node ' in t and ' min' in t for t in text), text
    assert any(t.startswith('amr_5 not heard for') and 'release it' in t for t in text)
    assert any('arm_outbound out of service' in t for t in text)
    blocked = [t for t in text if 'blocked by' in t]
    assert all(' at ' in t for t in blocked)          # 'waiting at X for N, blocked by amr_k at Y'
    run(f, w, 1300, start_s=200.1)                     # the causes are gone: the alarms clear themselves
    text = [a['text'] for a in ops.alarms()]
    assert not any('amr_3 stuck' in t or 'amr_5 not heard' in t or 'out of service' in t for t in text)


def test_kpis_and_the_shift_report(graph, wms):
    f, tap, w, ops = setup(graph, wms, [], 1800)
    k = ops.kpis()
    assert k['boxes_per_h'] > 40 and sum(k['per_robot_h'].values()) == pytest.approx(k['boxes_per_h'], rel=1e-6)
    assert set(k['arms']) == {'receiving_bay', 'outbound_bay'} and 0.99 < sum(k['shares'].values()) < 1.01
    md = ops.report()
    for h in ('# End-of-shift report', '## Throughput', '## Robot time', '## Arm cells', '## Energy', '## Orders',
              '## Exceptions', '## Open alarms'):
        assert h in md
    assert f'{k["boxes_per_h"]:.1f} boxes/h' in md


def test_operator_commands_are_published_and_logged_with_who_and_why(graph, wms):
    f, tap, w, ops = setup(graph, wms, [{'at': 50, 'do': 'silent', 'target': 'amr_2', 'for_s': 1e6}], 80)
    assert w.node.blocked_nodes()
    with pytest.raises(ValueError, match='required'):
        ops.command('RELEASE_ROBOT', 'amr_2', '', '')
    with pytest.raises(ValueError, match='unknown'):
        ops.command('SELF_DESTRUCT', 'amr_2', 'laith', 'test')
    ops.command('RELEASE_ROBOT', 'amr_2', 'laith', 'robot taken to the workshop')
    f.bus.flush()
    sent = [m for _, s, m in tap.get('/swarm/operator') if s == 'wms']
    assert sent and sent[0].command == OperatorCommand.RELEASE_ROBOT and sent[0].issued_by == 'laith'
    assert not w.node.blocked_nodes()
    ev = [e for e in w.node.store.events() if e['who'] == 'laith']
    assert ev and ev[0]['why'] == 'robot taken to the workshop' and ev[0]['what'] == 'RELEASE_ROBOT amr_2'


def test_dashboard_endpoints_and_offline_assets(graph, wms):
    f, tap, w, ops = setup(graph, wms, [], 120)
    api = Api(w.node, w.orders, ops=ops)
    code, live = api.handle('GET', '/api/v1/live')
    assert code == 200 and len(live['robots']) == 6 and len(live['bins']) == 128 and 'kpis' in live
    code, lay = api.handle('GET', '/api/v1/layout')
    assert code == 200 and len(lay['nodes']) == len(graph.pos)
    code, md = api.handle('GET', '/api/v1/report?format=md')
    assert code == 200 and md.startswith('# End-of-shift report')
    assert api.handle('POST', '/api/v1/operator', '{"command": "ACK_FAULT"}')[0] == 400
    for name in ('index.html', 'app.js', 'style.css'):
        text = open(os.path.join(WEB, name)).read()
        assert 'http://' not in text.replace('http://www.w3.org/2000/svg', '') and 'https://' not in text
