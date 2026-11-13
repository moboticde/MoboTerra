"""Isolate PAIL receiver faults with 1–4 disabled miControl wheel modules.

Only drive SDO uploads, receiver 0x1001 uploads, master heartbeat, and targeted
receiver Reset Communication/Start can pass the transmit whitelist.
"""
import argparse
from collections import deque
import datetime
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import time

from mobotic_config.configuration import Configuration, default_platform_path
from mobotic_config.operator_inputs import RemdeviceMapping
from mobotic_manual_control.remdevice_protocol import decode_digital, signed_axis
from clear_joystick_error import acquire_stack_lock, LOCK_PATH
from busmaster_trace import BusmasterTrace

WIRE = struct.Struct('=IB3x8s')
PHASES = ('baseline', 'heartbeat', 'receiver-poll', 'drives', 'combined')
FAULT_NAMES = {0x8110: 'CAN receive overrun', 0x8150: 'transmit COB-ID collision',
               0x6000: 'EEPROM error'}


class DiagnosticError(RuntimeError):
    pass


class ReceiverFault(DiagnosticError):
    pass


def integer(value):
    return int(value, 0) if isinstance(value, str) else int(value)


def module_nodes(profile):
    return {m['name']: tuple(integer(m[r]['can_node_id']) for r in ('steering', 'traction'))
            for m in profile['modules']}


def select_modules(topology, responding, requested=None):
    """Never silently accept half a module or omit a responding drive."""
    partial = [name for name, nodes in topology.items()
               if set(nodes) & responding and not set(nodes) <= responding]
    if partial:
        raise DiagnosticError('Partial wheel module response: ' + ', '.join(partial))
    found = [name for name, nodes in topology.items() if set(nodes) <= responding]
    if not 1 <= len(found) <= 4:
        raise DiagnosticError(f'Expected 1–4 complete responding modules; found {found}')
    if requested is not None and set(found) != set(requested):
        raise DiagnosticError(f'Expected modules {requested}; CAN discovery found {found}')
    return found


def permitted(cob, payload, drive_nodes, receiver, master):
    if cob == 0:
        return payload in (bytes((0x82, receiver)), bytes((1, receiver)))
    if cob == 0x700 + master:
        return payload == b'\x05'
    if len(payload) != 8 or payload[0] != 0x40 or payload[3:] != bytes(5):
        return False
    index = int.from_bytes(payload[1:3], 'little')
    return ((cob - 0x600 in drive_nodes and index in (0x3001, 0x3002, 0x3003, 0x3004))
            or (cob == 0x600 + receiver and index == 0x1001))


def disabled(index, value, node):
    if index == 0x3004 and value != 0:
        raise DiagnosticError(f'Node {node} software enable is {value}; stopping without writes.')
    if index == 0x3002 and value & 1:
        raise DiagnosticError(f'Node {node} power stage enabled; stopping without writes.')


def neutral(latest, mapping, now):
    digital = latest.get(0x180 + mapping.node_id)
    analog = latest.get(0x280 + mapping.node_id)
    if any(item is None or not 0 <= now - item[0] <= .25 or len(item[1]) != 8
           for item in (digital, analog)):
        raise DiagnosticError('Fresh digital and analog joystick PDOs required for receiver reset.')
    signals = decode_digital(list(digital[1]))
    if mapping.button('reset', signals):
        raise DiagnosticError('Release B2 before receiver reset.')
    for spec in mapping.axes.values():
        channel = spec['channel']
        axis = signed_axis(analog[1][channel - 1], signals[f'U{channel}'], signals[f'D{channel}'],
                           zero=spec['zero'], full_scale=spec['full_scale'], invert=spec['invert'])
        if abs(axis) > .08:
            raise DiagnosticError('Keep all joystick axes neutral before receiver reset.')


def snapshot(interface):
    return json.loads(subprocess.check_output(
        ['ip', '-details', '-statistics', '-json', 'link', 'show', interface]))[0]


def stopped_stack():
    if subprocess.run(['systemctl', 'is-active', '--quiet', 'moboterra-supervisor.service']).returncode == 0:
        raise DiagnosticError('Stop moboterra-supervisor.service first.')
    writers = {'mobotic_driver', 'remdevice_control', 'socket_can_sender_node_exe',
               'paced_socket_can_sender', 'clear_joystick_error.py'}
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            args = path.joinpath('cmdline').read_bytes().split(b'\0')
        except (OSError, ProcessLookupError):
            continue
        if {Path(a.decode(errors='replace')).name for a in args[:3] if a} & writers:
            raise DiagnosticError(f'Stop existing CAN writer PID {path.name} first.')


class Session:
    def __init__(self, bus, output, profile, mapping):
        self.bus, self.output = bus, output
        self.mapping = mapping
        self.receiver = mapping.node_id
        self.master = integer(profile['can']['master_node_id'])
        self.all_nodes = {n for pair in module_nodes(profile).values() for n in pair}
        self.nodes = sorted(self.all_nodes)
        self.latest = {}
        self.phase = 'startup'
        self.first_fault = None
        self.first_fault_by_phase = {}
        self.phase_stats = {}
        self.recent_tx = deque(maxlen=1000)
        self.last_send = 0
        self.drops = 0
        self.started = time.monotonic()
        self.frames = (output / 'frames.jsonl').open('x', buffering=1)
        self.events = (output / 'events.jsonl').open('x', buffering=1)
        self.trace = BusmasterTrace(output / 'busmaster.log', time.time_ns(),
            profile['can']['drives']['interface'], profile['can']['drives']['bitrate'],
            tx_origin='Tx means diagnostic socket acceptance, not hardware wire time; monotonic elapsed time is in frames.jsonl')

    def event(self, kind, **fields):
        event = dict(time_ns=time.time_ns(), elapsed=time.monotonic() - self.started,
                     phase=self.phase, kind=kind, **fields)
        self.events.write(json.dumps(event) + '\n')
        self.trace.note(event['time_ns'], json.dumps(event))
        return event

    def stage(self, name):
        self.phase = name
        self.phase_stats.setdefault(name, {'tx': 0, 'rx': 0, 'emcy': [], 'registers': [],
                                          'started_elapsed': time.monotonic() - self.started,
                                          'socket_drops_at_start': self.drops})
        self.event('phase_start')
        print(f'[{time.monotonic() - self.started:7.2f}s] {name}', flush=True)

    def record(self, direction, cob, data, **extra):
        record = dict(time_ns=time.time_ns(), elapsed=time.monotonic() - self.started,
                      phase=self.phase, direction=direction, id=f'{cob:08X}', data=data.hex().upper(), **extra)
        self.frames.write(json.dumps(record) + '\n')
        self.phase_stats[self.phase][direction] += 1
        self.trace.frame(record['time_ns'], 'Tx' if direction == 'tx' else 'Rx',
                         cob if cob & 0x20000000 else cob & (0x1FFFFFFF if cob & 0x80000000 else 0x7FF),
                         len(data), data, extended=bool(cob & 0x80000000),
                         rtr=bool(cob & 0x40000000), error=bool(cob & 0x20000000))
        return record

    def fault(self, kind, **details):
        if self.phase not in self.first_fault_by_phase:
            elapsed = time.monotonic() - self.started
            details['preceding_tx_1s'] = [r for r in self.recent_tx if elapsed - r['elapsed'] <= 1]
            details['preceding_tx_count_1s'] = len(details['preceding_tx_1s'])
        event = self.event(kind, **details)
        self.first_fault_by_phase.setdefault(self.phase, event)
        if self.first_fault is None:
            self.first_fault = event
            print(f'FIRST FAULT: {json.dumps({k:v for k,v in event.items() if k != "preceding_tx_1s"})}', flush=True)

    def pump(self, seconds, fail_on_fault=False):
        end = time.monotonic() + max(0, seconds)
        while time.monotonic() < end:
            self.bus.settimeout(min(.02, max(.0001, end - time.monotonic())))
            try:
                packet, ancillary, flags, _ = self.bus.recvmsg(WIRE.size, 128)
            except socket.timeout:
                continue
            if len(packet) != WIRE.size:
                raise DiagnosticError('Unexpected CAN frame size.')
            cob, dlc, payload = WIRE.unpack(packet)
            if dlc > 8:
                raise DiagnosticError('Unexpected CAN DLC.')
            data = payload[:dlc]
            kernel_ns = None
            for level, option, value in ancillary:
                if level == socket.SOL_SOCKET and option == 40:
                    self.drops = struct.unpack('=I', value[:4])[0]
                elif level == socket.SOL_SOCKET and option == 35:
                    sec, nsec = struct.unpack('@ll', value[:struct.calcsize('@ll')])
                    kernel_ns = sec * 1_000_000_000 + nsec
            self.record('rx', cob, data, local_echo=bool(flags & socket.MSG_DONTROUTE),
                        kernel_time_ns=kernel_ns, socket_rx_drops=self.drops)
            if cob & 0x20000000:
                self.fault('linux_can_error', can_id=f'{cob:08X}', data=data.hex())
            elif not cob & 0xC0000000:
                self.latest[cob] = (time.monotonic(), data)
                if cob == 0x80 + self.receiver and len(data) == 8:
                    code = int.from_bytes(data[:2], 'little')
                    self.phase_stats[self.phase]['emcy'].append(f'0x{code:04X}')
                    if code:
                        self.fault('receiver_emcy', code=f'0x{code:04X}', register=data[2],
                                   description=FAULT_NAMES.get(code, 'receiver fault'),
                                   kernel_time_ns=kernel_ns)
                if cob in {0x600 + n for n in self.all_nodes | {self.receiver}} | {0, 0x700 + self.master}:
                    raise DiagnosticError(f'Other CAN transmitter detected: {cob:03X}#{data.hex()}; isolate the test.')
            if fail_on_fault and self.first_fault is not None:
                raise ReceiverFault('New CAN/receiver fault; stopping test traffic.')

    def send(self, cob, data, fail_on_fault=False):
        if not permitted(cob, data, self.all_nodes, self.receiver, self.master):
            raise DiagnosticError(f'Transmit whitelist rejected {cob:03X}#{data.hex()}')
        self.pump(max(0, .002 - (time.monotonic() - self.last_send)), fail_on_fault)
        self.bus.send(WIRE.pack(cob, len(data), data.ljust(8, b'\0')))
        self.last_send = time.monotonic()
        self.recent_tx.append(self.record('tx', cob, data))

    def read(self, node, index, timeout=.1, allow_missing=False, fail_on_fault=False):
        # Drain queued frames before sending; a cached reply is never used.
        self.pump(.001, fail_on_fault)
        cob = 0x580 + node
        self.latest.pop(cob, None)
        request = bytes((0x40, index & 255, index >> 8, 0, 0, 0, 0, 0))
        self.send(0x600 + node, request, fail_on_fault)
        sent = self.last_send
        while time.monotonic() - sent < timeout:
            self.pump(min(.002, timeout - (time.monotonic() - sent)), fail_on_fault)
            item = self.latest.get(cob)
            if item is None or item[0] < sent or len(item[1]) != 8 or item[1][1:4] != request[1:4]:
                continue
            data = item[1]
            if data[0] == 0x80:
                raise DiagnosticError(f'SDO abort node {node} object {index:04X}: {data.hex()}')
            sizes = {0x43: 4, 0x47: 3, 0x4B: 2, 0x4F: 1}
            if data[0] not in sizes or (node in self.all_nodes and data[0] != 0x43):
                raise DiagnosticError(f'Invalid SDO upload node {node} object {index:04X}: {data.hex()}')
            value = int.from_bytes(data[4:4 + sizes[data[0]]], 'little')
            if node in self.all_nodes:
                disabled(index, value, node)
            if node == self.receiver:
                self.phase_stats[self.phase]['registers'].append(value)
                self.event('receiver_register', value=value)
                if value and fail_on_fault:
                    self.fault('receiver_register_nonzero', register=value)
                    raise ReceiverFault(f'Receiver error register is 0x{value:02X}')
            return value
        self.event('sdo_timeout', node=node, index=f'0x{index:04X}')
        if not allow_missing:
            raise DiagnosticError(f'No fresh SDO reply: node {node}, object 0x{index:04X}')
        return None

    def discover(self, topology, requested):
        observations = []
        for sweep in range(2):
            self.stage(f'discovery-{sweep + 1}')
            present = set()
            for node in sorted(self.all_nodes):
                enable = self.read(node, 0x3004, allow_missing=True)
                if enable is None:
                    continue
                self.read(node, 0x3002)
                present.add(node)
            observations.append(present)
        if observations[0] != observations[1]:
            raise DiagnosticError('Controller presence changed between discovery sweeps.')
        found = select_modules(topology, observations[0], requested)
        self.nodes = sorted(observations[0])
        self.event('discovered_modules', modules=found, nodes=self.nodes)
        print('Responding complete modules: ' + ', '.join(found), flush=True)
        return found

    def clear(self):
        self.stage('receiver-clear')
        # Recheck after discovery, immediately before any targeted reset.
        for node in self.nodes:
            self.read(node, 0x3004)
            self.read(node, 0x3002)
        before = self.read(self.receiver, 0x1001)
        self.event('clear_begin', error_before=before)
        if before not in (0, 0x11):
            raise DiagnosticError(f'Receiver register 0x{before:02X}: reset allowed only for 0x11.')
        if before:
            self.pump(.05)
            neutral(self.latest, self.mapping, time.monotonic())
            self.send(0, bytes((0x82, self.receiver)))
            self.event('receiver_reset_sent')
            self.pump(1)
            self.send(0, bytes((1, self.receiver)))
            self.pump(.5)
        self.first_fault = None
        for _ in range(5):
            if self.read(self.receiver, 0x1001, fail_on_fault=True) != 0:
                raise DiagnosticError('Receiver error did not stay zero after clear; no further reset attempted.')
            self.pump(.3, True)
        self.pump(.05, True)
        neutral(self.latest, self.mapping, time.monotonic())
        self.event('clear_verified', reset_sent=bool(before))
        print('Receiver register zero on five reads; drives disabled. Starting measured phases.', flush=True)

    def run_phase(self, name, duration, driver, rate=None):
        self.stage(name)
        # Each task has its own deadline. Late tasks skip missed slots.
        tasks = []
        now = time.monotonic()
        heartbeat = name in ('heartbeat', 'combined') or rate is not None
        receiver_poll = name in ('receiver-poll', 'combined') or rate is not None
        if heartbeat:
            tasks.append([now, float(driver['can_heartbeat_period']), 'heartbeat', None, None])
        if receiver_poll:
            tasks.append([now, 1.0, 'read', self.receiver, 0x1001])
        if name in ('drives', 'combined'):
            for index, period in ((0x3002, min(.1, float(driver['status_feedback_timeout']) / 3)),
                                  (0x3003, 1.0), (0x3001, float(driver['diagnostic_sdo_period']))):
                for offset, node in enumerate(self.nodes):
                    tasks.append([now + offset * .002, period, 'read', node, index])
        load_reads = [(n, i) for i in (0x3002, 0x3003, 0x3001) for n in self.nodes]
        load_cursor = 0
        if rate is not None:
            tasks.append([now, 1 / rate, 'load', None, None])
        finish = now + duration
        before = self.phase_stats[name]['tx']
        while time.monotonic() < finish:
            if not tasks:
                self.pump(min(.02, finish - time.monotonic()), True)
                continue
            task = min(tasks, key=lambda t: t[0])
            if task[0] > time.monotonic():
                self.pump(min(.02, task[0] - time.monotonic(), finish - time.monotonic()), True)
                continue
            if task[2] == 'heartbeat':
                self.send(0x700 + self.master, b'\x05', True)
            else:
                node, index = task[3:5]
                if task[2] == 'load':
                    node, index = load_reads[load_cursor % len(load_reads)]
                    load_cursor += 1
                self.read(node, index, fail_on_fault=True)
            task[0] = max(task[0] + task[1], self.last_send + task[1])
        elapsed = time.monotonic() - now
        self.phase_stats[name].update(duration=elapsed,
            actual_tx_per_second=(self.phase_stats[name]['tx'] - before) / elapsed,
            requested_drive_sdo_per_second=rate, socket_drops_at_end=self.drops)
        self.event('phase_end', **self.phase_stats[name])
        self.stage(name + ':end-register-check')
        self.read(self.receiver, 0x1001, fail_on_fault=True)

    def close(self):
        self.trace.close()
        self.frames.close()
        self.events.close()


def parser_for_cli():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--modules', nargs='+', help='Optional expected corner names; otherwise discover over CAN')
    parser.add_argument('--phases', nargs='+', choices=PHASES, default=list(PHASES))
    parser.add_argument('--seconds', type=float, default=45, help='Duration of each measured phase (default 45)')
    parser.add_argument('--rates', nargs='+', type=float, default=[],
                        help='Optional extra aggregate drive SDO/s stages, e.g. 50 125 225; diagnostic load only')
    parser.add_argument('--output-dir', type=Path, help='New directory (default ~/can_traces/receiver-test-TIMESTAMP)')
    parser.add_argument('--discover-only', action='store_true', help='Read drive presence and disabled state only; no receiver reset or test phases')
    parser.add_argument('--check', action='store_true', help='Print configuration and test plan without CAN access')
    return parser


def main(argv=None):
    parser = parser_for_cli()
    args = parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 3600:
        parser.error('seconds must be in (0, 3600]')
    if any(not math.isfinite(r) or not 0 < r <= 300 for r in args.rates):
        parser.error('rates must be in (0, 300]')
    if len(set(args.phases)) != len(args.phases):
        parser.error('phases must be unique')
    cfg = Configuration()
    profile = cfg.platform
    mapping = RemdeviceMapping(profile['operator']['mapping'])
    topology = module_nodes(profile)
    if args.modules and (len(set(args.modules)) != len(args.modules) or
                         not set(args.modules) <= topology.keys() or not 1 <= len(args.modules) <= 4):
        parser.error('--modules requires 1–4 unique configured corner names: ' + ', '.join(topology))
    if not mapping.verified or mapping.node_id != 10:
        parser.error('Expected commissioned verified node-10 receiver mapping')
    if any(m[r]['controller'].lower() != 'micontrol' for m in profile['modules'] for r in ('steering', 'traction')):
        parser.error('Only miControl drives supported')
    plan = {'profile': str(default_platform_path()), 'interface': profile['can']['drives'],
            'topology': topology, 'expected_modules': args.modules or 'CAN discovery',
            'phases': args.phases, 'seconds_per_phase': args.seconds, 'extra_drive_sdo_rates': args.rates,
            'driver_policy': cfg.parameters('driver'),
            'note': 'Read-only drive diagnostics, targeted receiver clear, no drive initialization or enable. '
                    'Discovery proves responding controllers, not physical absence of silent hardware.'}
    if args.check:
        print(json.dumps(plan, indent=2))
        return 0
    root = args.output_dir or Path.home() / 'can_traces' / datetime.datetime.now().strftime('receiver-test-%Y%m%d-%H%M%S-%f')
    lock = bus = session = None
    root_created = False
    result = {'plan': plan, 'outcome': 'incomplete'}
    try:
        LOCK_PATH.parent.mkdir(parents=True, exist_ok=True)
        lock = acquire_stack_lock()
        stopped_stack()
        interface = profile['can']['drives']['interface']
        info = snapshot(interface)
        can = info['linkinfo']['info_data']
        if ('UP' not in info['flags'] or can.get('bittiming', {}).get('bitrate') != profile['can']['drives']['bitrate']
                or can.get('state') == 'BUS-OFF' or 'LISTEN-ONLY' in can.get('ctrlmode', [])):
            raise DiagnosticError('CAN must be UP at configured bitrate, transmitting and not BUS-OFF.')
        root.mkdir(parents=True, exist_ok=False)
        root_created = True
        result['interface_before'] = info
        (root / 'plan.json').write_text(json.dumps(plan, indent=2))
        print(f'Record: {root}', flush=True)
        bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        bus.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        bus.setsockopt(socket.SOL_SOCKET, 40, 1)  # SO_RXQ_OVFL
        bus.setsockopt(socket.SOL_SOCKET, 35, 1)  # SO_TIMESTAMPNS
        result['socket_receive_buffer_bytes'] = bus.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        bus.setsockopt(socket.SOL_CAN_RAW, getattr(socket, 'CAN_RAW_ERR_FILTER', 2), struct.pack('=I', 0x1FFFFFFF))
        bus.bind((interface,))
        session = Session(bus, root, profile, mapping)
        result['modules'] = session.discover(topology, args.modules)
        if args.discover_only:
            result['outcome'] = 'discovery_only'
        else:
            session.clear()
            for phase in args.phases:
                session.run_phase(phase, args.seconds, cfg.parameters('driver'))
            for number, rate in enumerate(args.rates):
                session.run_phase(f'load-{number + 1}-{rate:g}-sdo-per-sec', args.seconds,
                                  cfg.parameters('driver'), rate)
            result['outcome'] = 'completed_no_measured_fault'
        code = 0
    except ReceiverFault as error:
        result.update(outcome='fault_observed', error=str(error))
        code = 2
    except (DiagnosticError, OSError, ValueError, subprocess.SubprocessError) as error:
        result.update(outcome='aborted', error=str(error))
        code = 1
    except KeyboardInterrupt:
        result.update(outcome='interrupted')
        code = 130
    finally:
        if session:
            try:
                # Receive-only aftermath, with no further resets or requests.
                session.stage('aftermath')
                session.pump(2)
            except (DiagnosticError, OSError) as error:
                result['aftermath_error'] = str(error)
            result.update(first_fault=session.first_fault, first_fault_by_phase=session.first_fault_by_phase,
                          phases=session.phase_stats,
                          socket_rx_drops=session.drops, capture_has_socket_loss=bool(session.drops))
            try:
                result['interface_after'] = snapshot(profile['can']['drives']['interface'])
            except (OSError, subprocess.SubprocessError) as error:
                result['interface_after_error'] = str(error)
            session.close()
        if root_created:
            (root / 'summary.json').write_text(json.dumps(result, indent=2))
        if bus:
            bus.close()
        if lock:
            lock.close()
    print(json.dumps({k: v for k, v in result.items() if k in ('outcome', 'error', 'modules')}, indent=2), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
