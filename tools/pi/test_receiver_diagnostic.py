"""Hardware-free regressions for the receiver diagnostic's CAN boundaries."""
from collections import deque
import json
from pathlib import Path
import socket
import struct
import time
from types import SimpleNamespace

import pytest

import receiver_diagnostic as d

TOPOLOGY = {'front_left': (1, 2), 'front_right': (3, 4),
            'rear_left': (7, 8), 'rear_right': (5, 6)}


@pytest.mark.parametrize('nodes,expected', [
    ({1, 2}, ['front_left']), ({3, 4}, ['front_right']),
    ({5, 6}, ['rear_right']), ({7, 8}, ['rear_left']),
    ({1, 2, 7, 8}, ['front_left', 'rear_left']),
    ({1, 2, 3, 4, 7, 8}, ['front_left', 'front_right', 'rear_left']),
    (set(range(1, 9)), list(TOPOLOGY))])
def test_one_to_four_modules(nodes, expected):
    assert d.select_modules(TOPOLOGY, nodes) == expected


@pytest.mark.parametrize('nodes', [{1, 2, 7}, {5}, {6}, set()])
def test_partial_or_insufficient_modules_refuse(nodes):
    with pytest.raises(d.DiagnosticError):
        d.select_modules(TOPOLOGY, nodes)


def test_explicit_selection_cannot_hide_another_responding_module():
    with pytest.raises(d.DiagnosticError, match='CAN discovery found'):
        d.select_modules(TOPOLOGY, {1, 2, 3, 4, 7, 8}, ['front_left', 'rear_left'])


def test_whitelist_rejects_all_drive_writes_and_broadcast_commands():
    for node in range(1, 9):
        for command in (0x23, 0x2F, 0x2B):
            for index in (0x3000, 0x3003, 0x3004, 0x3300, 0x3790, 0x1010):
                payload = bytes((command, index & 255, index >> 8, 0, 1, 0, 0, 0))
                assert not d.permitted(0x600 + node, payload, set(range(1, 9)), 10, 11)
    for payload in (b'\x01\x00', b'\x82\x00', b'\x82\x01', b'\x81\x0a'):
        assert not d.permitted(0, payload, set(range(1, 9)), 10, 11)
    assert d.permitted(0, b'\x82\x0a', set(range(1, 9)), 10, 11)
    assert d.permitted(0, b'\x01\x0a', set(range(1, 9)), 10, 11)


class FakeBus:
    def __init__(self):
        self.pending = deque()
        self.sent = []
        self.timeout = .001
        self.status = 0x10200
        self.enable = 0
        self.abort = False
        self.emcy_on_read = False
        self.omit = False
        self.responding = {1, 2, 7, 8}

    def settimeout(self, value):
        self.timeout = value

    def send(self, packet):
        cob, dlc, data = d.WIRE.unpack(packet)
        self.sent.append((cob, data[:dlc]))
        if not 0x600 < cob < 0x680 or self.omit:
            return len(packet)
        node = cob - 0x600
        if node not in self.responding | {10}:
            return len(packet)
        index = int.from_bytes(data[1:3], 'little')
        value = self.status if index == 0x3002 else self.enable if index == 0x3004 else 0
        if self.emcy_on_read:
            self.pending.append(d.WIRE.pack(0x8A, 8, bytes.fromhex('5081110000000000')))
        reply = bytes((0x80 if self.abort else 0x43,)) + data[1:4] + value.to_bytes(4, 'little')
        self.pending.append(d.WIRE.pack(0x580 + node, 8, reply))
        return len(packet)

    def recvmsg(self, *args):
        if self.pending:
            return self.pending.popleft(), [], 0, None
        time.sleep(min(self.timeout, .0002))
        raise socket.timeout()


@pytest.fixture
def session(tmp_path):
    profile = {'can': {'master_node_id': '0x0B', 'drives': {'interface': 'can0', 'bitrate': 500000}},
               'modules': [dict(name=name, steering={'can_node_id': nodes[0]},
                                traction={'can_node_id': nodes[1]}) for name, nodes in TOPOLOGY.items()]}
    mapping = SimpleNamespace(node_id=10)
    s = d.Session(FakeBus(), tmp_path, profile, mapping)
    s.stage('test')
    yield s
    s.close()


def test_upload_requires_fresh_matching_complete_reply(session):
    session.latest[0x581] = (time.monotonic(), bytes.fromhex('4302300001000000'))
    assert session.read(1, 0x3002) == 0x10200
    session.bus.omit = True
    assert session.read(1, 0x3002, timeout=.003, allow_missing=True) is None


@pytest.mark.parametrize('field,value,index', [('status', 1, 0x3002), ('enable', 1, 0x3004)])
def test_enabled_drive_aborts_without_drive_writes(session, field, value, index):
    setattr(session.bus, field, value)
    with pytest.raises(d.DiagnosticError, match='enable'):
        session.read(1, index)
    assert all(payload[0] == 0x40 for cob, payload in session.bus.sent)


def test_sdo_abort_is_not_treated_as_absent(session):
    session.bus.abort = True
    with pytest.raises(d.DiagnosticError, match='SDO abort'):
        session.read(1, 0x3004, allow_missing=True)


def test_emcy_records_preceding_request_and_stops_traffic(session):
    session.bus.emcy_on_read = True
    with pytest.raises(d.ReceiverFault):
        session.read(1, 0x3002, fail_on_fault=True)
    assert session.first_fault['code'] == '0x8150'
    assert session.first_fault['preceding_tx_1s'][-1]['data'] == '4002300000000000'
    assert len(session.bus.sent) == 1


def test_other_transmitter_aborts(session):
    session.bus.pending.append(d.WIRE.pack(0x601, 8, bytes.fromhex('2304300001000000')))
    with pytest.raises(d.DiagnosticError, match='Other CAN transmitter'):
        session.pump(.01)
    assert not session.bus.sent


def test_normal_pre_enable_phases_only_send_allowed_reads_and_heartbeat(session):
    session.nodes = [1, 2, 7, 8]
    driver = dict(can_heartbeat_period=.01, status_feedback_timeout=.35, diagnostic_sdo_period=1)
    for name in d.PHASES:
        start = len(session.bus.sent)
        session.run_phase(name, .035, driver)
        phase_frames = session.bus.sent[start:]
        assert all(d.permitted(cob, data, set(session.nodes), 10, 11) for cob, data in phase_frames)
        if name == 'baseline':
            assert len(phase_frames) == 1  # register check after timed quiet phase
    assert session.first_fault is None


def test_neutral_guard_rejects_stale_pdos_before_reset():
    with pytest.raises(d.DiagnosticError, match='Fresh digital'):
        d.neutral({}, SimpleNamespace(node_id=10), time.monotonic())


def test_fault_preceding_requests_are_persisted_in_event_file(session):
    session.bus.emcy_on_read = True
    with pytest.raises(d.ReceiverFault):
        session.read(1, 0x3002, fail_on_fault=True)
    events = [json.loads(line) for line in (session.output / 'events.jsonl').read_text().splitlines()]
    fault = next(event for event in events if event['kind'] == 'receiver_emcy')
    assert fault['preceding_tx_count_1s'] == 1
    assert fault['preceding_tx_1s'][0]['id'] == '00000601'
    session.first_fault = None  # beginning verified measurement must not lose setup evidence
    assert session.first_fault_by_phase['test']['code'] == '0x8150'


@pytest.mark.parametrize('register,reset_count', [(0, 0), (0x11, 1)])
def test_clear_only_resets_communication_fault_after_disabled_checks(session, monkeypatch, register, reset_count):
    session.nodes = [1, 2, 5, 6]
    reads = []
    neutral_checks = []
    receiver_values = iter([register, 0, 0, 0, 0, 0])
    def read(node, index, **kwargs):
        reads.append((node, index))
        if node == 10:
            return next(receiver_values)
        return 0
    monkeypatch.setattr(session, 'read', read)
    monkeypatch.setattr(session, 'pump', lambda *args: None)
    monkeypatch.setattr(d, 'neutral', lambda *args: neutral_checks.append(len(session.bus.sent)))
    session.clear()
    assert reads[:8] == [(node, index) for node in session.nodes for index in (0x3004, 0x3002)]
    resets = [data for cob, data in session.bus.sent if cob == 0 and data[0] == 0x82]
    assert len(resets) == reset_count
    assert session.bus.sent == ([(0, b'\x82\x0a'), (0, b'\x01\x0a')] if register else [])
    assert neutral_checks[0] == 0


def test_another_receiver_error_never_resets(session, monkeypatch):
    session.nodes = []
    monkeypatch.setattr(session, 'read', lambda *args, **kwargs: 0x80)
    with pytest.raises(d.DiagnosticError, match='reset allowed only for 0x11'):
        session.clear()
    assert not session.bus.sent


def test_no_new_requests_after_first_measured_fault(session):
    session.nodes = [1, 2, 7, 8]
    session.bus.emcy_on_read = True
    driver = dict(can_heartbeat_period=.1, status_feedback_timeout=.35, diagnostic_sdo_period=1)
    with pytest.raises(d.ReceiverFault):
        session.run_phase('combined', .1, driver)
    assert session.first_fault['phase'] == 'combined'
    assert sum(cob >= 0x600 and cob < 0x680 for cob, _ in session.bus.sent) == 1


def test_kernel_timestamp_and_socket_loss_are_recorded(session):
    stamp_ns = time.time_ns() - 1000000
    sec, nsec = divmod(stamp_ns, 1000000000)
    packet = d.WIRE.pack(0x8A, 8, bytes.fromhex('1081110000000000'))
    original = session.bus.recvmsg
    responses = deque([(packet, [(socket.SOL_SOCKET, 35, struct.pack('@ll', sec, nsec)),
                                (socket.SOL_SOCKET, 40, struct.pack('=I', 3))], 0, None)])
    session.bus.recvmsg = lambda *args: responses.popleft() if responses else original(*args)
    session.pump(.002)
    assert session.first_fault['kernel_time_ns'] == stamp_ns
    assert session.drops == 3
    rows = [json.loads(line) for line in (session.output / 'frames.jsonl').read_text().splitlines()]
    assert rows[-1]['kernel_time_ns'] == stamp_ns
    assert rows[-1]['socket_rx_drops'] == 3


def test_one_explicit_module_is_accepted():
    assert d.select_modules(TOPOLOGY, {5, 6}, ['rear_right']) == ['rear_right']
