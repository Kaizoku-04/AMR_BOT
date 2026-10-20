"""The cell's set_io / set_payload calls: a lost reply (DDS under load) is sent again; only a refusal or no reply to
any attempt faults the cell (which a person must clear)."""
import pytest

from open_amr_arm_cell.cell_motion import CellFault, VacuumIO


class Fut:
    def __init__(self, ok=None):
        self.ok = ok

    def done(self):
        return self.ok is not None

    def result(self):
        return type('R', (), {'success': self.ok})()


class Client:
    def __init__(self, *answers):
        self.answers, self.calls, self.dropped = list(answers), 0, 0

    def call_async(self, req):
        self.calls += 1
        return Fut(self.answers.pop(0))

    def remove_pending_request(self, fut):
        self.dropped += 1


def call(cli):
    VacuumIO._call(VacuumIO.__new__(VacuumIO), cli, None, 'set_io vacuum on', timeout=0.05)


def test_lost_reply_is_sent_again():
    cli = Client(None, True)                 # the first reply never comes
    call(cli)
    assert cli.calls == 2 and cli.dropped == 1


def test_refusal_faults_at_once():
    cli = Client(False, True)
    with pytest.raises(CellFault, match='refused'):
        call(cli)
    assert cli.calls == 1


def test_no_reply_to_any_attempt_faults():
    cli = Client(None, None, None, True)
    with pytest.raises(CellFault, match='no reply to 3 attempts'):
        call(cli)
    assert cli.calls == 3
