#!/usr/bin/env python3
"""Listen for a PAIL restart directly on SocketCAN, outside Docker and ROS."""

import logging
from pathlib import Path
import socket
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import current_stack as deployment

sys.path.insert(0, str(deployment.ROOT / 'src/mobotic_manual_control'))
from mobotic_config.operator_inputs import RemdeviceMapping
from mobotic_manual_control.remdevice_protocol import RemdeviceReceiver
from mobotic_manual_control.restart_gesture import RestartGesture, controls_neutral


LOG = logging.getLogger('moboterra-joystick-restart')
CAN_FRAME = struct.Struct('=IB3x8s')
TIMESPEC = struct.Struct('@ll')
SO_TIMESTAMPNS = getattr(socket, 'SO_TIMESTAMPNS', 35)
CAN_RAW_ERR_FILTER = getattr(socket, 'CAN_RAW_ERR_FILTER', 2)
CAN_ERR_FLAG = 0x20000000
CAN_ERR_BUS_STATE = 0x40 | 0x100  # BUS-OFF and controller restarted.
STANDARD_DATA_MASK = 0xC00007FF
READ_STATE = b'\x40\x02\x30\x00\x00\x00\x00\x00'
STATE_REPLY = b'\x43\x02\x30\x00'
ENABLE_OBJECT = b'\x04\x30\x00'
PROBE_GAP_NS = 2_000_000
PROBE_TIMEOUT_NS = 100_000_000
VERIFY_TIMEOUT_NS = 2_000_000_000


class JoystickRestart:
    def __init__(self, mapping, receiver, drive_ids, status_timeout_ns, read_state, restart):
        self.mapping = mapping
        self.receiver = receiver
        self.drive_ids = tuple(drive_ids)
        self.status_timeout_ns = status_timeout_ns
        self.read_state = read_state
        self.restart = restart
        self.gesture = RestartGesture(receiver.timeout_ns)
        self.drive_states = {}
        self.last_drive_activity = 0
        self.previous_b2 = None
        self.release_stamp = None
        self.verify_deadline = None
        self.probe = None
        self.last_probe = 0
        self.last_now = 0

    def _cancel(self, reason):
        if self.release_stamp is not None:
            LOG.warning('B1+B2 restart cancelled: %s', reason)
        self.release_stamp = self.verify_deadline = self.probe = None
        self.previous_b2 = None
        self.gesture.clear()

    def _check_clock(self, now):
        if now < self.last_now:
            self._cancel('host time moved backwards')
            self.drive_states.clear()
            self.last_drive_activity = self.last_probe = 0
        self.last_now = now

    def _external_drive_activity(self, now):
        self.last_drive_activity = now
        if self.release_stamp is not None and self.last_probe >= self.release_stamp:
            self._cancel('other drive CAN traffic resumed during verification')

    def _fresh(self, stamp, now):
        return 0 <= now - stamp <= self.status_timeout_ns

    def _permitted(self, sample, now):
        digital = self.receiver.digital(now)
        return (sample is not None and digital is not None and digital['LINK']
                and digital['EMGY'] and not digital['STOP']
                and not self.mapping.button('mode', digital)
                and controls_neutral(self.mapping, sample))

    def receive(self, can_id, payload, stamp, now):
        self._check_clock(now)
        if can_id & CAN_ERR_FLAG:
            self._cancel('CAN bus state changed')
            self.drive_states.clear()
            self.receiver.invalidate('CAN bus state changed', now)
            return

        if can_id - 0x600 in self.drive_ids:
            self._external_drive_activity(now)
            if (len(payload) == 8 and payload[0] & 0xe0 == 0x20
                    and payload[1:4] == ENABLE_OBJECT and any(payload[4:])):
                self.drive_states[can_id - 0x600] = (now, True)
                self._cancel('a drive enable was requested')
            return

        node = can_id - 0x580
        if node in self.drive_ids:
            valid_state = len(payload) == 8 and payload[:4] == STATE_REPLY and self._fresh(stamp, now)
            if self.probe is not None:
                probe_node, sent = self.probe
                if stamp < sent:
                    return
                if node != probe_node or not valid_state or stamp - sent >= PROBE_TIMEOUT_NS:
                    self._external_drive_activity(now)
                    return
                self.probe = None
            else:
                # The paced sender can suppress echoing its requests on the
                # host. Replies also establish that the bus is being polled.
                self._external_drive_activity(now)
            previous = self.drive_states.get(node)
            if valid_state and (previous is None or stamp > previous[0]):
                enabled = bool(payload[4] & 1)
                self.drive_states[node] = (stamp, enabled)
                if enabled:
                    self._cancel(f'drive CAN node {node} is enabled')
            return

        accepted = self.receiver.update(can_id, len(payload), payload, stamp, now)
        digital = self.receiver.digital(now)
        if (digital is None or not digital['LINK'] or not digital['EMGY'] or digital['STOP']
                or self.mapping.button('mode', digital)):
            self._cancel('joystick input unavailable or outside Manual')
            return
        if not accepted:
            return
        sample = self.receiver.snapshot(now, consume=True)
        if sample is None:
            return
        if not self._permitted(sample, now):
            self._cancel('joystick controls moved out of neutral')
            return
        b1 = self.mapping.button('boost', sample.digital)
        b2 = self.mapping.button('reset', sample.digital)
        previous_b2, self.previous_b2 = self.previous_b2, b2
        if self.release_stamp is not None:
            if b1 or b2:
                self._cancel('buttons were pressed again during verification')
            return
        permitted = not any(enabled and self._fresh(when, now)
                            for when, enabled in self.drive_states.values())
        _, requested = self.gesture.observe(sample.stamp_ns, b1=b1, b2=b2,
                                           previous_b2=previous_b2, permitted=permitted)
        if requested:
            self.release_stamp = sample.stamp_ns
            self.verify_deadline = now + VERIFY_TIMEOUT_NS
            LOG.info('B1+B2 released after full hold. Verifying native disabled-drive status.')

    def poll(self, now):
        self._check_clock(now)
        if self.release_stamp is None:
            return
        if not self._permitted(self.receiver.snapshot(now), now):
            self._cancel('fresh neutral joystick input was lost')
            return
        digital = self.receiver.digital(now)
        if self.mapping.button('boost', digital) or self.mapping.button('reset', digital):
            self._cancel('buttons were pressed again during verification')
            return
        if now >= self.verify_deadline:
            self._cancel('fresh disabled status did not arrive from every drive')
            return
        missing = [node for node in self.drive_ids if not self._verified_disabled(node, now)]
        if not missing:
            self.release_stamp = self.verify_deadline = self.probe = None
            self.gesture.clear()
            self.previous_b2 = None
            try:
                self.restart()
            except OSError as error:
                LOG.error('B1+B2 could not reach the host restart socket: %s', error)
            else:
                LOG.info('B1+B2 requested moboterra-ctl restart after all drives reported disabled')
            return
        if self.probe is not None:
            node, sent = self.probe
            if now - sent >= PROBE_TIMEOUT_NS:
                self._cancel(f'no native drive-state reply from CAN node {node}')
            return
        if (now - self.last_drive_activity >= self.status_timeout_ns
                and now - self.last_probe >= PROBE_GAP_NS):
            node = missing[0]
            self.read_state(node)
            self.probe = (node, now)
            self.last_probe = now

    def _verified_disabled(self, node, now):
        state = self.drive_states.get(node)
        return (state is not None and not state[1] and state[0] >= self.release_stamp
                and self._fresh(state[0], now))


def connect_restart_socket():
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as request:
        request.settimeout(0.05)
        request.connect(str(Path(deployment.RESTART_SOCKET_DIR) / 'restart.sock'))


def profile_version(path):
    stat = path.stat()
    return stat.st_ino, stat.st_mtime_ns, stat.st_size


def listen(interface, mapping, drive_ids, joystick_timeout_ns, status_timeout_ns, operator, profile_matches):
    receiver = RemdeviceReceiver(mapping.node_id, timeout_ns=joystick_timeout_ns,
                                 errors_advisory=operator.get('receiver_errors_advisory', False),
                                 can_overrun_advisory=operator.get('receiver_can_overrun_advisory', False))
    ids = [receiver.digital_id, receiver.analog_id, receiver.heartbeat_id, receiver.emergency_id]
    ids.extend(base + node for base in (0x580, 0x600) for node in drive_ids)
    filters = b''.join(struct.pack('=II', can_id, STANDARD_DATA_MASK) for can_id in ids)

    def restart_current_profile():
        if not profile_matches():
            raise OSError('active platform changed during restart verification')
        connect_restart_socket()

    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        bus.setsockopt(socket.SOL_CAN_RAW, CAN_RAW_ERR_FILTER, CAN_ERR_BUS_STATE)
        bus.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        bus.settimeout(0.02)
        bus.bind((interface,))
        restart = JoystickRestart(mapping, receiver, drive_ids, status_timeout_ns,
                                  lambda node: bus.send(CAN_FRAME.pack(0x600 + node, 8, READ_STATE)),
                                  restart_current_profile)
        LOG.info('Listening for B1+B2 on %s outside ROS', interface)
        while profile_matches():
            try:
                packet, controls, _, _ = bus.recvmsg(CAN_FRAME.size, socket.CMSG_SPACE(TIMESPEC.size))
            except TimeoutError:
                restart.poll(time.time_ns())
                continue
            now = time.time_ns()
            for level, kind, value in controls:
                if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(value) == TIMESPEC.size:
                    seconds, nanos = TIMESPEC.unpack(value)
                    if len(packet) == CAN_FRAME.size:
                        can_id, length, payload = CAN_FRAME.unpack(packet)
                        if length <= len(payload):
                            restart.receive(can_id, payload[:length], seconds * 1_000_000_000 + nanos, now)
                    break
            restart.poll(now)


def main():
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(message)s')
    profile = deployment.ROOT / 'runtime/platform.yaml'
    while True:
        try:
            version = profile_version(profile)
            config = deployment.configuration(profile)
            operator = config.platform.get('operator', {})
            if operator.get('source') != 'remdevice' or not operator.get('mapping', {}).get('verified'):
                LOG.info('Host joystick restart requires a verified REMdevice mapping')
                return
            mapping = RemdeviceMapping(operator['mapping'])
            ids = tuple(int(value, 0) for value in config.parameters('driver')['drives.can_node_ids'])
            interface = config.platform['can']['drives']['interface']
            joystick_timeout_ns = int(config.parameters('manual')['joystick_timeout'] * 1e9)
            status_timeout_ns = int(config.parameters('supervisor')['wheel_status_timeout'] * 1e9)
            listen(interface, mapping, ids, joystick_timeout_ns, status_timeout_ns, operator,
                   lambda: profile_version(profile) == version)
        except OSError as error:
            LOG.warning('Waiting for an active MoboTerra profile and CAN interface: %s', error)
            time.sleep(2)


if __name__ == '__main__':
    main()
