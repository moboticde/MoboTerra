"""Own the native launch lifecycle and verify controller disable after exit.

A separate process group keeps terminal SIGINT from racing the CAN sender shut.
Direct CAN cleanup runs only after this launch's writers have exited. This is a
bounded software stop, not a replacement for independent hardware safety.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

from mobotic_config.configuration import Configuration

WIRE = struct.Struct('=IB3x8s')
GAP = .002


class DisableBus:
    def __init__(self, interface):
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.bind((interface,))
        self.socket.settimeout(.02)
        self.last_send = 0.

    def close(self):
        self.socket.close()

    def exchange(self, node, index, value=None):
        # Drain queued/old responses before issuing a new transaction.
        self.socket.setblocking(False)
        try:
            for _ in range(4096):
                self.socket.recv(16)
        except BlockingIOError:
            pass
        finally:
            self.socket.settimeout(.02)
        if value is None:
            command, data = 0x40, b'\0' * 4
        else:
            size = 2 if index == 0x6040 else 4
            command = 0x2b if size == 2 else 0x23
            data = value.to_bytes(size, 'little').ljust(4, b'\0')
        request = bytes((command, index & 255, index >> 8, 0)) + data
        time.sleep(max(0., GAP - (time.monotonic() - self.last_send)))
        self.socket.send(WIRE.pack(0x600 + node, 8, request))
        self.last_send = time.monotonic()
        deadline = self.last_send + .15
        while time.monotonic() < deadline:
            try:
                packet = self.socket.recv(16)
            except socket.timeout:
                continue
            cob, length, reply = WIRE.unpack(packet)
            if cob != 0x580 + node or length != 8 or reply[1:4] != request[1:4]:
                continue
            if reply[0] == 0x80:
                raise RuntimeError(f'node {node} SDO abort 0x{int.from_bytes(reply[4:], "little"):08x}')
            if value is not None and reply[0] == 0x60:
                return None
            sizes = {0x43: 4, 0x47: 3, 0x4b: 2, 0x4f: 1}
            if value is None and reply[0] in sizes:
                return int.from_bytes(reply[4:4+sizes[reply[0]]], 'little')
        raise TimeoutError(f'node {node} no fresh reply for 0x{index:04x}')


def drive_bindings(cfg):
    result = []
    for module in cfg.platform['modules']:
        for role in ('steering', 'traction'):
            drive = {**cfg.platform['actuators'][role], **module[role]}
            node = drive['can_node_id']
            node = int(node, 0) if isinstance(node, str) else int(node)
            controller = drive['controller'].lower()
            if controller not in ('micontrol', 'elmo') or not 1 <= node <= 127:
                raise ValueError('unsupported shutdown controller/node')
            result.append((node, controller))
    if len({node for node, _ in result}) != len(result):
        raise ValueError('duplicate shutdown node')
    return result


def disable_and_verify(bus, drives):
    """Attempt every drive even if another is absent. Never enable or reset."""
    results = {}
    # Revoke all enables before spending time reading back any controller.
    for node, controller in drives:
        try:
            bus.exchange(node, 0x3004 if controller == 'micontrol' else 0x6040, 0)
        except Exception as error:
            results[node] = {'verified': False, 'error': str(error)}
    for node, controller in drives:
        for attempt in range(2):
            try:
                if attempt:
                    time.sleep(.05)
                    bus.exchange(node, 0x3004 if controller == 'micontrol' else 0x6040, 0)
                if controller == 'micontrol':
                    requested = bus.exchange(node, 0x3004)
                    actual = bus.exchange(node, 0x3002)
                    verified = requested == 0 and not actual & 1
                    result = {'software_enable': requested, 'device_status': actual, 'verified': verified}
                else:
                    requested = bus.exchange(node, 0x6040)
                    actual = bus.exchange(node, 0x6041)
                    result = {'controlword': requested, 'status_word': actual,
                              'verified': requested == 0 and not bool(actual & 4)}
                results[node] = result
                if result['verified']:
                    break
            except Exception as error:
                results[node] = {'verified': False, 'error': str(error)}
    return results


def group_members(group):
    for proc in Path('/proc').iterdir():
        if proc.name.isdigit():
            try:
                pid = int(proc.name)
                if os.getpgid(pid) == group:
                    yield pid, proc.joinpath('cmdline').read_bytes().split(b'\0')
            except (ProcessLookupError, PermissionError, FileNotFoundError):
                pass


def signal_group(group, sig):
    try:
        os.killpg(group, sig)
    except ProcessLookupError:
        pass


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--platform-config', required=True)
    parser.add_argument('--interface', required=True)
    args, command = parser.parse_known_args(argv)
    if command[:1] == ['--']:
        command.pop(0)
    if not command:
        parser.error('launch command required after --')
    # Match the last effective ROS launch override, including caller overrides.
    launch_values = dict(token.split(':=', 1) for token in command if ':=' in token)
    if launch_values.get('stack_type', 'hardware') != 'hardware':
        parser.error('this hardware shutdown guard requires stack_type:=hardware')
    cfg = Configuration(launch_values.get('platform_config', args.platform_config))
    interface = launch_values.get('can_interface', args.interface)
    if interface == 'auto':
        interface = cfg.platform['can']['drives']['interface']
    drives = drive_bindings(cfg)
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopping.set())
    child = subprocess.Popen(command, start_new_session=True)
    result = 1
    try:
        while child.poll() is None and not stopping.wait(.1):
            pass
        print('Shutdown: withdrawing authority; keeping driver/CAN alive for disable.', flush=True)
        for pid, cmd in group_members(child.pid):
            if any(a in (b'__node:=manual_drive_control', b'__node:=mobotic_supervisor') for a in cmd):
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        # The unchanged driver lease is 300 ms in this installed profile.
        time.sleep(max(.6, cfg.parameters('driver')['supervisor_timeout'] + .3))
        signal_group(child.pid, signal.SIGINT)
        deadline = time.monotonic() + 10
        while list(group_members(child.pid)) and time.monotonic() < deadline:
            child.poll()  # Reap launch when it exits, avoiding a zombie in the group.
            time.sleep(.05)
        if list(group_members(child.pid)):
            signal_group(child.pid, signal.SIGTERM)
            deadline = time.monotonic() + 3
            while list(group_members(child.pid)) and time.monotonic() < deadline:
                child.poll()
                time.sleep(.05)
        if list(group_members(child.pid)):
            raise RuntimeError('launch writers did not exit; disable could not be verified exclusively')
        child.wait()
        print('Shutdown: disabling and verifying every drive over CAN.', flush=True)
        bus = DisableBus(interface)
        try:
            results = disable_and_verify(bus, drives)
        finally:
            bus.close()
        print('Shutdown drive verification: ' + json.dumps(results, sort_keys=True), flush=True)
        if all(r['verified'] for r in results.values()):
            print('Shutdown complete: all configured drives verified disabled.', flush=True)
            result = 0 if stopping.is_set() or child.returncode == 0 else child.returncode
        else:
            print('ERROR: shutdown disable NOT verified for every drive; check hardware safety/power/CAN.', file=sys.stderr)
    except Exception as error:
        print('ERROR: shutdown disable NOT verified: ' + str(error), file=sys.stderr, flush=True)
        signal_group(child.pid, signal.SIGTERM)
    return result


if __name__ == "__main__":
    sys.exit(main())
