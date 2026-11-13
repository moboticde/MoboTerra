"""Subscribe-only raw CAN and ROS transmit recorder. Never sends CAN frames."""
import argparse
import collections
import datetime
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time

from busmaster_trace import BusmasterTrace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--namespace', default='/moboterra')
    parser.add_argument('--receiver-node', type=int, default=10)
    parser.add_argument('--output-dir', type=Path, help='New directory; defaults to ~/can_traces/<timestamp>')
    parser.add_argument('--duration', type=float, help='Seconds; otherwise record until Ctrl+C')
    parser.add_argument('--no-ros', action='store_true', help='Record raw CAN reception only')
    args = parser.parse_args()
    if not 1 <= args.receiver_node <= 127 or (args.duration is not None and args.duration <= 0):
        parser.error('receiver node must be 1..127 and duration must be positive')
    if not args.no_ros:
        import rclpy
        from can_msgs.msg import Frame
        from mobotic_interfaces.msg import OperatorState, WheelModuleStatusArray, VehicleModeState
        from sensor_msgs.msg import Joy, JointState
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.convert import message_to_ordereddict

    root = args.output_dir or Path.home() / 'can_traces' / datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    root = root.expanduser().absolute()
    root.mkdir(parents=True, exist_ok=False)
    raw = (root / 'can_rx.log').open('w', buffering=1)
    hardware_rx = (root / 'can_rx_hardware.jsonl').open('w', buffering=1)
    tx = (root / 'ros_can_tx.log').open('w', buffering=1)
    tx_json = (root / 'ros_can_tx.jsonl').open('w', buffering=1)
    socket_tx_json = (root / 'socket_can_tx.jsonl').open('w', buffering=1)
    states = (root / 'ros_state.jsonl').open('w', buffering=1)
    event_file = (root / 'joystick_events.log').open('w', buffering=1)
    stopped = threading.Event()
    event_lock = threading.Lock()
    counts = collections.Counter()
    faults = collections.Counter()
    errors = []
    drops = 0
    node = None
    thread = None
    bus = None
    started = time.time_ns()

    def network_snapshot():
        try:
            return json.loads(subprocess.check_output([
                'ip', '-details', '-statistics', '-json', 'link', 'show', args.interface]))[0]
        except (OSError, subprocess.CalledProcessError, ValueError) as error:
            return {'error': str(error)}

    metadata = {'started_ns': started, 'interface': args.interface,
        'namespace': args.namespace, 'receiver_node': args.receiver_node,
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'ros_enabled': not args.no_ros, 'interface_before': network_snapshot(),
        'capture_note': 'can_rx.log records frames delivered to the raw socket. '
            'The ROS sender disables loopback, so its outgoing frames are absent there. '
            'ros_can_tx.log and ros_can_tx.jsonl record outgoing ROS publications '
            'before the sender, not independently verified wire transmissions. '
            'socket_can_tx.jsonl records paced-sender socket acceptance, not a hardware wire timestamp. '
            'All use Unix wall-clock timestamps for correlation.'}
    bitrate = metadata['interface_before'].get('linkinfo', {}).get('info_data', {}).get('bittiming', {}).get('bitrate', 500000)
    combined = BusmasterTrace(root / 'busmaster.log', started, args.interface, bitrate)
    socket_combined = BusmasterTrace(root / 'busmaster_socket.log', started, args.interface, bitrate,
        tx_origin='Tx observes paced-sender socket acceptance; no independent wire timestamp. Without a paced sender this file has Rx only')
    metadata['combined_trace'] = 'busmaster.log: Rx and Tx plus event notes, sorted on stop; BUSMASTER CAN ASCII layout'
    metadata['socket_timeline_trace'] = 'busmaster_socket.log: raw Rx plus paced-sender socket-accepted Tx, sorted on stop; source Tx publications are excluded'
    metadata['hardware_rx_trace'] = ('can_rx_hardware.jsonl stores software Unix time and raw hardware '
        'receive time separately. Hardware time may use a different clock domain; compare intervals '
        'within that domain, never subtract it directly from ROS or software timestamps. '
        'Hardware receive timestamps do not establish host transmit times.')
    (root / 'metadata.json').write_text(json.dumps(metadata, indent=2))

    def event(message, stamp_ns=None, terminal=False):
        stamp_ns = stamp_ns if stamp_ns is not None else time.time_ns()
        line = f'({stamp_ns / 1e9:.9f}) {message}'
        with event_lock:
            event_file.write(line + '\n')
            combined.note(stamp_ns, message)
            socket_combined.note(stamp_ns, message)
            if terminal:
                print(line, flush=True)

    def collect_can():
        nonlocal drops
        wire = struct.Struct('=IB3x8s')
        try:
            while not stopped.is_set():
                try:
                    packet, ancillary, flags, _ = bus.recvmsg(72, 256)
                except socket.timeout:
                    continue
                if len(packet) != wire.size:
                    event(f'Unexpected CAN packet length {len(packet)}')
                    continue
                cob, dlc, data = wire.unpack(packet)
                received_ns = time.time_ns()
                stamp_ns = received_ns
                hardware_ns = None
                for level, kind, payload in ancillary:
                    if level == socket.SOL_SOCKET and kind == 35:
                        sec, nsec = struct.unpack('@ll', payload[:16])
                        stamp_ns = sec * 1_000_000_000 + nsec
                    elif level == socket.SOL_SOCKET and kind == 37 and len(payload) >= 48:
                        sec, nsec, _, _, hw_sec, hw_nsec = struct.unpack('@6l', payload[:48])
                        if sec or nsec:
                            stamp_ns = sec * 1_000_000_000 + nsec
                        if hw_sec or hw_nsec:
                            hardware_ns = hw_sec * 1_000_000_000 + hw_nsec
                    elif level == socket.SOL_SOCKET and kind == 40:
                        drops = struct.unpack('=I', payload[:4])[0]
                is_error = bool(cob & 0x20000000)
                extended = bool(cob & 0x80000000)
                identifier = cob & (0x1fffffff if extended else 0x7ff)
                label = f'{cob:08X}' if is_error else (f'{identifier:08X}' if extended else f'{identifier:03X}')
                payload = f'R{dlc}' if cob & 0x40000000 else data[:dlc].hex().upper()
                raw.write(f'({stamp_ns / 1e9:.6f}) {args.interface} {label}#{payload}\n')
                hardware_rx.write(json.dumps({'software_ns': stamp_ns, 'received_ns': received_ns,
                    'hardware_ns': hardware_ns, 'can_id_flags': cob, 'dlc': dlc,
                    'data': data[:dlc].hex().upper(), 'local_echo': bool(flags & socket.MSG_DONTROUTE)}) + '\n')
                counts['hardware_rx_timestamps' if hardware_ns is not None else 'missing_hardware_rx_timestamps'] += 1
                combined.frame(stamp_ns, 'Tx' if flags & socket.MSG_DONTROUTE else 'Rx',
                    cob if is_error else identifier, dlc, data[:dlc], extended=extended,
                    rtr=bool(cob & 0x40000000), error=is_error)
                socket_combined.frame(stamp_ns, 'Tx' if flags & socket.MSG_DONTROUTE else 'Rx',
                    cob if is_error else identifier, dlc, data[:dlc], extended=extended,
                    rtr=bool(cob & 0x40000000), error=is_error)
                counts['can_frames'] += 1
                if is_error:
                    counts['linux_can_error_frames'] += 1
                    event(f'LINUX_CAN_ERROR {label}#{payload}', stamp_ns, terminal=True)
                elif not extended and not cob & 0x40000000 and identifier == 0x80 + args.receiver_node and dlc == 8:
                    code = int.from_bytes(data[:2], 'little')
                    faults[f'0x{code:04X}'] += 1
                    description = {0: 'reset/no error', 0x8110: 'CAN receive overrun',
                        0x8150: 'transmit COB-ID collision', 0x6000: 'EEPROM error'}.get(code, 'receiver fault')
                    event(f'JOYSTICK_EMCY code=0x{code:04X} register=0x{data[2]:02X} '
                        f'{description} frame={label}#{payload}', stamp_ns,
                        terminal=faults[f'0x{code:04X}'] <= 3 or faults[f'0x{code:04X}'] % 100 == 0)
                elif not extended and identifier == 0x580 + args.receiver_node and dlc == 8 and data[:4] == bytes.fromhex('4F011000'):
                    event(f'JOYSTICK_ERROR_REGISTER 0x{data[4]:02X}', stamp_ns)
        except BaseException as error:
            errors.append(f'CAN collector: {error!r}')
            event(errors[-1], terminal=True)
            stopped.set()

    previous_operator = None

    def record_tx(message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        received = time.time_ns()
        label = f'{message.id:08X}' if message.is_extended or message.is_error else f'{message.id:03X}'
        payload = f'R{message.dlc}' if message.is_rtr else bytes(message.data[:message.dlc]).hex().upper()
        tx.write(f'({(stamp or received) / 1e9:.6f}) ros_can_tx {label}#{payload}\n')
        tx_json.write(json.dumps({'source_ns': stamp, 'received_ns': received,
            'id': message.id, 'dlc': message.dlc, 'data': payload,
            'extended': message.is_extended, 'error': message.is_error,
            'rtr': message.is_rtr}) + '\n')
        combined.frame(stamp or received, 'Tx', message.id, message.dlc,
            bytes(message.data[:message.dlc]), extended=message.is_extended,
            rtr=message.is_rtr, error=message.is_error)
        counts['ros_tx_publications'] += 1

    def record_state(topic, message):
        nonlocal previous_operator
        states.write(json.dumps({'received_ns': time.time_ns(), 'topic': topic,
            'message': message_to_ordereddict(message)}) + '\n')
        counts['ros_state_messages'] += 1
        if topic == 'operator/state':
            key = (message.connected, message.estop_state_known, message.estop_pressed, message.status_message)
            if key != previous_operator:
                event(f'OPERATOR connected={message.connected} STOP={message.estop_pressed} '
                    f'STOP_known={message.estop_state_known}: {message.status_message}')
                previous_operator = key

    def record_socket_tx(message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        socket_tx_json.write(json.dumps({'socket_accepted_ns': stamp, 'received_ns': time.time_ns(),
            'id': message.id, 'dlc': message.dlc,
            'data': bytes(message.data[:message.dlc]).hex().upper(),
            'extended': message.is_extended, 'error': message.is_error, 'rtr': message.is_rtr}) + '\n')
        socket_combined.frame(stamp, 'Tx', message.id, message.dlc,
            bytes(message.data[:message.dlc]), extended=message.is_extended,
            rtr=message.is_rtr, error=message.is_error)
        counts['socket_tx_accepted'] += 1

    def stop_signal(signum, frame):
        stopped.set()

    signal.signal(signal.SIGINT, stop_signal)
    signal.signal(signal.SIGTERM, stop_signal)
    print(f'Recording to: {root}', flush=True)
    print(f'Combined Rx/Tx trace: {root / "busmaster.log"} (sorted on stop)', flush=True)
    print(f'Paced-sender Rx/Tx timeline: {root / "busmaster_socket.log"}', flush=True)
    print('Start ROS in another terminal. Ctrl+C stops this recorder only.', flush=True)
    try:
        bus = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        bus.setsockopt(socket.SOL_CAN_RAW, 2, struct.pack('=I', 0x1fffffff))
        bus.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        bus.setsockopt(socket.SOL_SOCKET, 35, 1)
        try:
            # Receive-only: RX_HARDWARE | RX_SOFTWARE | SOFTWARE | RAW_HARDWARE.
            # This socket never requests TX timestamps or transmits CAN frames.
            bus.setsockopt(socket.SOL_SOCKET, 37, (1 << 2) | (1 << 3) | (1 << 4) | (1 << 6))
            metadata['hardware_rx_timestamp_requested'] = True
        except OSError as error:
            metadata['hardware_rx_timestamp_requested'] = False
            metadata['hardware_rx_timestamp_error'] = str(error)
        (root / 'metadata.json').write_text(json.dumps(metadata, indent=2))
        bus.setsockopt(socket.SOL_SOCKET, 40, 1)
        bus.bind((args.interface,))
        bus.settimeout(.2)
        thread = threading.Thread(target=collect_can, name='passive-can-capture')
        thread.start()
        if not args.no_ros:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node('moboterra_trace_recorder_' + str(os.getpid()))
            namespace = '/' + args.namespace.strip('/')
            subscriptions = [node.create_subscription(Frame, namespace + '/can_tx', record_tx,
                QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE))]
            subscriptions.append(node.create_subscription(Frame, namespace + '/can_tx_sent', record_socket_tx,
                QoSProfile(depth=10000, reliability=ReliabilityPolicy.BEST_EFFORT)))
            for topic, kind in [('operator/state', OperatorState),
                                ('wheel_modules/status', WheelModuleStatusArray),
                                ('operator/scanner_selector', Joy), ('can_tx_status', String),
                                ('vehicle/mode', VehicleModeState), ('joint_states', JointState),
                                ('vehicle/measured_velocity', TwistStamped),
                                ('operator/raw', String)]:
                subscriptions.append(node.create_subscription(kind, namespace + '/' + topic,
                    lambda msg, name=topic: record_state(name, msg),
                    QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)))
        event('RECORDER_STARTED')
        deadline = time.monotonic() + args.duration if args.duration else None
        while not stopped.is_set() and (deadline is None or time.monotonic() < deadline):
            if node is not None:
                rclpy.spin_once(node, timeout_sec=.1)
            else:
                stopped.wait(.1)
    except Exception as error:
        errors.append(repr(error))
        event(f'RECORDER_ERROR {error}', terminal=True)
    finally:
        stopped.set()
        if thread is not None:
            thread.join(timeout=3)
        if bus is not None:
            bus.close()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        summary = {'started_ns': started, 'finished_ns': time.time_ns(),
            'counts': dict(counts), 'receiver_emcy_codes': dict(faults),
            'kernel_capture_drops': drops, 'errors': errors, 'interface_after': network_snapshot()}
        (root / 'summary.json').write_text(json.dumps(summary, indent=2))
        event('RECORDER_STOPPED ' + json.dumps(summary['counts']), terminal=True)
        try:
            combined.close(summary['finished_ns'])
            socket_combined.close(summary['finished_ns'])
        except Exception as error:
            errors.append(f'Combined trace finalization: {error!r}')
            summary['errors'] = list(errors)
            (root / 'summary.json').write_text(json.dumps(summary, indent=2))
            print(errors[-1], flush=True)
        for handle in (raw, hardware_rx, tx, tx_json, socket_tx_json, states, event_file):
            handle.close()
        print(f'Saved: {root}\nReceiver EMCYs: {dict(faults)}; capture drops: {drops}', flush=True)
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
