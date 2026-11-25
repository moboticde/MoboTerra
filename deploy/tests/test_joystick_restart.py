"""Replay raw joystick and drive CAN traffic without ROS or a joystick."""

import importlib.util
from pathlib import Path
import socket

import pytest


SOURCE = Path(__file__).resolve().parents[1] / 'host/joystick_restart.py'
spec = importlib.util.spec_from_file_location('joystick_restart', SOURCE)
host = importlib.util.module_from_spec(spec)
spec.loader.exec_module(host)

MS = 1_000_000
DRIVES = (1, 2, 3, 4, 5, 6, 7, 8)
B1, B2 = 4, 8  # Active PAIL uses SP3/SP4.


class Bus:
    def __init__(self):
        self.now = 10_000 * MS
        self.requests = []
        self.restarts = []
        operator = host.deployment.configuration().platform['operator']
        mapping = host.RemdeviceMapping(operator['mapping'])
        receiver = host.RemdeviceReceiver(mapping.node_id, errors_advisory=True)
        self.restart = host.JoystickRestart(mapping, receiver, DRIVES, 350 * MS,
                                           self.requests.append, lambda: self.restarts.append(self.now))

    def frame(self, can_id, payload, stamp=None):
        self.restart.receive(can_id, payload, self.now if stamp is None else stamp, self.now)

    def pair(self, buttons=0, *, link=0x28, direction=0x10, magnitude=0, dt=50):
        self.now += dt * MS
        self.frame(0x18a, bytes([direction, 0, buttons, 0, 0, 0, 0, link]))
        self.frame(0x28a, bytes([0, 0, magnitude, 0, 0, 0, 0, 0]))

    def hold(self, *, prime=True, samples=103):
        if prime:
            self.pair()
            self.pair(B1)
        for _ in range(samples):
            self.pair(B1 | B2)

    def status(self, node, enabled=False, *, stamp=None):
        self.frame(0x580 + node, host.STATE_REPLY + bytes([int(enabled), 0, 0, 0]), stamp)

    def poll(self):
        self.restart.poll(self.now)


def test_uses_fresh_passive_disabled_reports_when_the_driver_still_polls():
    bus = Bus()
    bus.hold()
    for node in DRIVES:
        bus.status(node)
    bus.pair(B2)
    bus.poll()
    assert not bus.requests and not bus.restarts
    bus.pair()
    bus.poll()
    assert not bus.requests and not bus.restarts
    bus.now += MS
    for node in DRIVES[:-1]:
        bus.status(node)
    bus.poll()
    assert not bus.restarts
    bus.status(DRIVES[-1])
    bus.poll()
    bus.pair()
    bus.poll()
    assert len(bus.restarts) == 1
    assert not bus.requests


def test_reads_native_state_and_restarts_when_the_ros_stack_sends_nothing():
    bus = Bus()
    bus.hold()
    bus.pair()
    for node in DRIVES:
        bus.poll()
        assert bus.requests[-1] == node
        assert not bus.restarts
        bus.now += 4 * MS
        bus.status(node)
    bus.poll()
    assert bus.requests == list(DRIVES)
    assert len(bus.restarts) == 1


def test_valid_gesture_reaches_the_host_socket_once(tmp_path, monkeypatch):
    monkeypatch.setattr(host.deployment, 'RESTART_SOCKET_DIR', tmp_path)
    bus = Bus()
    bus.restart.restart = host.connect_restart_socket
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(tmp_path / 'restart.sock'))
        server.listen()
        server.setblocking(False)
        bus.hold()
        bus.pair()
        for node in DRIVES:
            bus.poll()
            bus.now += 4 * MS
            bus.status(node)
        bus.poll()
        connection, _ = server.accept()
        connection.close()
        bus.pair()
        bus.poll()
        with pytest.raises(BlockingIOError):
            server.accept()


def test_silence_from_a_drive_fails_closed():
    bus = Bus()
    bus.hold()
    bus.pair()
    bus.poll()
    bus.pair()
    bus.pair()
    bus.poll()
    assert bus.requests == [1]
    assert not bus.restarts
    assert bus.restart.release_stamp is None


@pytest.mark.parametrize('reason', ['competing_request', 'competing_reply', 'drive_enabled', 'enable_request',
                                   'no_link', 'non_neutral'])
def test_cancels_pending_verification_when_conditions_change(reason):
    bus = Bus()
    bus.hold()
    bus.pair()
    bus.poll()
    bus.now += 4 * MS
    if reason == 'competing_request':
        bus.frame(0x608, host.READ_STATE)
    elif reason == 'competing_reply':
        bus.frame(0x588, b'\x43' + host.ENABLE_OBJECT + b'\0' * 4)
    elif reason == 'drive_enabled':
        bus.status(1, enabled=True)
    elif reason == 'enable_request':
        bus.frame(0x608, b'\x23' + host.ENABLE_OBJECT + b'\x01\x00\x00\x00')
    elif reason == 'no_link':
        bus.pair(link=0)
    else:
        bus.pair(direction=0x14, magnitude=255)
    bus.poll()
    assert bus.requests == [1]
    assert not bus.restarts
    assert bus.restart.release_stamp is None


def test_probes_ignore_old_status():
    bus = Bus()
    bus.hold()
    bus.pair()
    bus.poll()
    bus.status(1, stamp=bus.now - MS)
    bus.now += 4 * MS
    bus.poll()
    assert bus.requests == [1]
    assert not bus.restarts
    bus.status(1)
    bus.poll()
    assert bus.requests == [1, 2]


@pytest.mark.parametrize('interruption', ['unprimed', 'short_hold', 'link_loss', 'pdo_gap', 'outside_manual'])
def test_held_buttons_and_incomplete_or_interrupted_gestures_never_probe(interruption):
    bus = Bus()
    if interruption == 'unprimed':
        bus.hold(prime=False)
    elif interruption == 'short_hold':
        bus.hold(samples=100)
    else:
        bus.hold(samples=60)
        if interruption == 'link_loss':
            bus.pair(B1 | B2, link=0)
        elif interruption == 'pdo_gap':
            bus.pair(B1 | B2, dt=300)
        else:
            bus.pair(B1 | B2, direction=0)
        bus.hold(prime=False)
    bus.pair()
    bus.poll()
    assert not bus.requests and not bus.restarts
