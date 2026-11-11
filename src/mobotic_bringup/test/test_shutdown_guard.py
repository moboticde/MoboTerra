"""Shutdown regression: verify requested AND actual state, including partial failures."""
from types import SimpleNamespace
import pytest
from mobotic_bringup.shutdown_guard import DisableBus, WIRE, disable_and_verify, drive_bindings


class VirtualBus:
    def __init__(self, missing=(), stuck=()):
        self.calls = []
        self.enabled = {i: 1 for i in range(1, 9)}
        self.missing, self.stuck = missing, stuck

    def exchange(self, node, index, value=None):
        self.calls.append((node, index, value))
        if node in self.missing:
            raise TimeoutError('no reply')
        if value is not None:
            assert value == 0  # cleanup must never reset/enable
            if node not in self.stuck:
                self.enabled[node] = 0
        elif index == 0x3004:
            return self.enabled[node]
        elif index == 0x3002:
            return self.enabled[node]
        elif index == 0x6040:
            return self.enabled[node] * 15
        elif index == 0x6041:
            return self.enabled[node] * 4
        else:
            raise AssertionError(index)


def test_disables_every_drive_before_verification():
    bus = VirtualBus()
    result = disable_and_verify(bus, [(i, 'micontrol') for i in range(1, 9)])
    assert all(r['verified'] and r['software_enable'] == 0 for r in result.values())
    assert bus.calls[:8] == [(i, 0x3004, 0) for i in range(1, 9)]


@pytest.mark.parametrize('fault', ['missing', 'stuck'])
def test_partial_failure_does_not_skip_other_drives(fault):
    bus = VirtualBus(**{fault: (1,)})
    result = disable_and_verify(bus, [(i, 'micontrol') for i in range(1, 9)])
    assert not result[1]['verified']
    assert all(result[i]['verified'] for i in range(2, 9))


def test_zero_requested_is_not_confirmation_of_actual_disabled():
    class StillEnabled(VirtualBus):
        def exchange(self, node, index, value=None):
            return 0 if index == 0x3004 else 1
    assert not disable_and_verify(StillEnabled(), [(1, 'micontrol')])[1]['verified']


def test_elmo_shutdown_uses_controlword_and_actual_operation_enabled_bit():
    bus = VirtualBus()
    result = disable_and_verify(bus, [(2, 'elmo')])
    assert result[2]['verified']
    assert bus.calls == [(2, 0x6040, 0), (2, 0x6040, None), (2, 0x6041, None)]


def test_profile_drives_are_derived_instead_of_hardcoded():
    cfg = SimpleNamespace(platform={'actuators': {'steering': {'controller': 'micontrol'},
        'traction': {'controller': 'elmo'}}, 'modules': [
            {'steering': {'can_node_id': '0x07'}, 'traction': {'can_node_id': 8}}]})
    assert drive_bindings(cfg) == [(7, 'micontrol'), (8, 'elmo')]
    cfg.platform['modules'][0]['traction']['can_node_id'] = 7
    with pytest.raises(ValueError, match='duplicate'):
        drive_bindings(cfg)


def test_wire_protocol_ignores_mismatched_reply_and_accepts_matching_ack():
    class Socket:
        def setblocking(self, flag):
            self.draining = not flag
        def settimeout(self, timeout):
            self.draining = False
        def send(self, packet):
            self.sent = WIRE.unpack(packet)
            self.replies = [WIRE.pack(0x581, 8, bytes.fromhex('6003300000000000')),
                            WIRE.pack(0x581, 8, bytes.fromhex('6004300000000000'))]
        def recv(self, size):
            if self.draining:
                raise BlockingIOError()
            return self.replies.pop(0)
    bus = DisableBus.__new__(DisableBus)
    bus.socket, bus.last_send = Socket(), 0
    bus.exchange(1, 0x3004, 0)
    assert bus.socket.sent == (0x601, 8, bytes.fromhex('2304300000000000'))


def test_sigint_keeps_child_alive_until_authority_withdrawal_then_verifies(tmp_path):
    import os
    from pathlib import Path
    import signal
    import subprocess
    import sys
    import time
    profile = Path(__file__).parents[2] / 'mobotic_config/config/platform-four-micontrol.yaml'
    ready = tmp_path / 'ready'
    child_code = '''import signal,time,pathlib,sys
signal.signal(signal.SIGINT,lambda *_:sys.exit(0))
pathlib.Path(sys.argv[1]).write_text('ready')
while True: time.sleep(.1)
'''
    guard_code = '''import sys
from mobotic_bringup import shutdown_guard as g
class Bus:
    def __init__(self,*a):pass
    def close(self):pass
    def exchange(self,node,index,value=None):return 0
g.DisableBus=Bus
sys.exit(g.main(sys.argv[1:]))
'''
    command = [sys.executable, '-c', guard_code, '--platform-config', str(profile),
               '--interface', 'unused', '--', sys.executable, '-c', child_code, str(ready)]
    guard = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             start_new_session=True)
    try:
        deadline = time.monotonic() + 8
        while not ready.exists() and guard.poll() is None and time.monotonic() < deadline:
            time.sleep(.02)
        assert ready.exists()
        start = time.monotonic()
        guard.send_signal(signal.SIGINT)
        output, _ = guard.communicate(timeout=15)
        assert guard.returncode == 0, output
        assert time.monotonic() - start >= .55
        assert 'all configured drives verified disabled' in output
    finally:
        if guard.poll() is None:
            os.killpg(guard.pid, signal.SIGTERM)
            guard.wait(timeout=5)
