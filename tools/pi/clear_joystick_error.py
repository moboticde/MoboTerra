"""Clear node-10 communication fault only, with native ROS stack stopped."""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import socket
import struct
import subprocess
import time

from mobotic_config.operator_inputs import RemdeviceMapping
from mobotic_config.configuration import Configuration
from mobotic_manual_control.remdevice_protocol import decode_digital, signed_axis


PROJECT_ROOT = Path(__file__).resolve().parents[2]
LOCK_PATH = PROJECT_ROOT / 'runtime' / 'native-stack.lock'

def acquire_stack_lock(inherited_fd=None, lock_path=LOCK_PATH):
    # The launcher already owns this flock. A duplicate of its inherited open
    # file description preserves exclusion without unlocking the parent.
    try:
        if inherited_fd is None:
            lock = open(lock_path, 'a')
        else:
            expected = os.stat(lock_path)
            actual = os.fstat(inherited_fd)
            if (actual.st_dev, actual.st_ino) != (expected.st_dev, expected.st_ino):
                raise SystemExit('Inherited descriptor is not the native stack lock.')
            lock = os.fdopen(os.dup(inherited_fd), 'a')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            raise SystemExit('Stop the native ROS stack before clearing the joystick receiver.')
        return lock
    except OSError as error:
        raise SystemExit(f'Cannot acquire native stack lock: {error}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stack-lock-fd', type=int,
                        help='Inherited native-stack lock owned by the startup launcher')
    parser.add_argument('--allow-receiver-transmit-once', action='store_true',
                        help='Explicit one-time receiver clear despite receive-only mode; keep the saved setting unchanged')
    args = parser.parse_args(argv)
    cfg = Configuration()
    receive_only_override = (not cfg.parameters('manual')['receiver_transmit_enabled']
                             and args.allow_receiver_transmit_once)
    if not cfg.parameters('manual')['receiver_transmit_enabled'] and not receive_only_override:
        raise SystemExit('PAIL receive-only: receiver clearing disabled; no CAN messages sent.')
    if receive_only_override:
        print('One-time receiver clear authorized; saved receive-only setting remains disabled for TX.', flush=True)
    lock = acquire_stack_lock(args.stack_lock_fd)
    if subprocess.run(['systemctl', 'is-active', '--quiet', 'moboterra-supervisor.service']).returncode == 0:
        raise SystemExit('Stop the managed robot stack before clearing the receiver.')
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            args = process.joinpath('cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if {Path(x.decode(errors='replace')).name for x in args[:3] if x} & {
                'mobotic_driver', 'remdevice_control', 'socket_can_sender_node_exe', 'paced_socket_can_sender'}:
            raise SystemExit(f'Stop ROS CAN writers first (PID {process.name}).')
    profile = cfg.platform
    mapping = RemdeviceMapping(profile['operator']['mapping'])
    if not mapping.verified or mapping.node_id != 10:
        raise SystemExit('Expected the verified node-10 receiver mapping.')
    drive_nodes = []
    for module in profile['modules']:
        for role in ('steering', 'traction'):
            drive = module[role]
            if drive['controller'].lower() != 'micontrol':
                raise SystemExit('This disabled-state check requires miControl drives.')
            value = drive['can_node_id']
            drive_nodes.append(int(value, 0) if isinstance(value, str) else value)
    if set(drive_nodes) != set(range(1, 9)) or len(drive_nodes) != 8:
        raise SystemExit('Expected exactly the commissioned eight drive nodes 1–8.')
    root = Path.home() / 'can_traces'
    root.mkdir(exist_ok=True)
    result_file = root / datetime.datetime.now().strftime('joystick-clear-%Y%m%d-%H%M%S-%f.json')
    result = {'events': [], 'drive_states': {}, 'drive_software_enable': {},
              'error_after_reads': [], 'reset_sent': False, 'cleared': False,
              'receive_only_override_once': receive_only_override}
    latest = {}
    wire = struct.Struct('=IB3x8s')
    bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    bus.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    bus.bind((profile['can']['drives']['interface'],))
    bus.settimeout(.02)

    def send(cob, payload):
        upload = (cob == 0x60a and payload == bytes.fromhex('4001100000000000')) or (
            0x601 <= cob <= 0x608 and payload in (
                bytes.fromhex('4002300000000000'), bytes.fromhex('4004300000000000')))
        assert upload or (cob == 0 and payload in (bytes.fromhex('820A'), bytes.fromhex('010A')))
        bus.send(wire.pack(cob, len(payload), payload.ljust(8, b'\0')))
        result['events'].append({'time_ns': time.time_ns(), 'direction': 'tx',
            'id': f'{cob:03X}', 'data': payload.hex().upper()})

    def receive(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                packet = bus.recv(16)
            except socket.timeout:
                continue
            cob, dlc, payload = wire.unpack(packet)
            latest[cob] = (time.monotonic(), payload[:dlc])
            if cob in (0x8a, 0x58a, 0x70a) or 0x581 <= cob <= 0x588:
                result['events'].append({'time_ns': time.time_ns(), 'direction': 'rx',
                    'id': f'{cob:03X}', 'data': payload[:dlc].hex().upper()})

    def read(node, index):
        latest.pop(0x580 + node, None)
        request = bytes((0x40, index & 255, index >> 8, 0, 0, 0, 0, 0))
        sent = time.monotonic()
        send(0x600 + node, request)
        receive(.2)
        reply = latest.get(0x580 + node)
        if reply is None or reply[0] < sent or len(reply[1]) != 8 or reply[1][1:4] != request[1:4]:
            raise RuntimeError(f'No matching fresh reply from node {node} for 0x{index:04X}')
        if reply[1][0] not in (0x43, 0x4f):
            raise RuntimeError(f'Unsupported or aborted read: {reply[1].hex()}')
        if index == 0x3002 and reply[1][0] != 0x43:
            raise RuntimeError(f'Actual drive status must be a complete 32-bit reply: {reply[1].hex()}')
        n = 4 if reply[1][0] == 0x43 else 1
        return int.from_bytes(reply[1][4:4+n], 'little')

    try:
        for node_id in drive_nodes:
            requested = read(node_id, 0x3004)
            result['drive_software_enable'][str(node_id)] = requested
            if requested != 0:
                raise RuntimeError(f'Drive {node_id} still has software enable requested; reset not sent.')
            state = read(node_id, 0x3002)
            result['drive_states'][str(node_id)] = state
            if state & 1:
                raise RuntimeError(f'Drive {node_id} actual power stage is enabled; reset not sent.')
        receive(.25)
        before = read(10, 0x1001)
        result['error_before'] = before
        print(f'Receiver current error register: 0x{before:02X}', flush=True)
        if before not in (0, 0x11):
            raise RuntimeError(f'Receiver has another fault (0x{before:02X}); no reset or EEPROM write sent.')
        if before:
            now = time.monotonic()
            digital = latest.get(0x18a)
            analog = latest.get(0x28a)
            if any(item is None or not 0 <= now-item[0] <= .25 or len(item[1]) != 8 for item in (digital, analog)):
                raise RuntimeError('Fresh joystick PDOs required; reset not sent.')
            signals = decode_digital(list(digital[1]))
            if mapping.button('reset', signals):
                raise RuntimeError('Release B2 before clearing the receiver.')
            if not all(abs(signed_axis(analog[1][spec['channel']-1],
                    signals[f"U{spec['channel']}"], signals[f"D{spec['channel']}"],
                    zero=spec['zero'], full_scale=spec['full_scale'], invert=spec['invert'])) <= .08
                    for spec in mapping.axes.values()):
                raise RuntimeError('Keep all joystick axes neutral; reset not sent.')
            send(0, bytes.fromhex('820A'))
            result['reset_sent'] = True
            receive(1)
            send(0, bytes.fromhex('010A'))
            receive(.5)
        for _ in range(5):
            value = read(10, 0x1001)
            result['error_after_reads'].append(value)
            print(f'Verification: 0x{value:02X}', flush=True)
            receive(.3)
        result['cleared'] = all(value == 0 for value in result['error_after_reads'])
        if not result['cleared']:
            raise RuntimeError('Receiver still reports an error; no further reset or settings write attempted.')
        print('Joystick receiver fault cleared and verified. Drives remain disabled.', flush=True)
    except Exception as error:
        result['error'] = str(error)
        print(f'Not cleared: {error}', flush=True)
    finally:
        bus.close()
        result_file.write_text(json.dumps(result, indent=2))
        print(f'Record: {result_file}', flush=True)
    return 0 if result['cleared'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
