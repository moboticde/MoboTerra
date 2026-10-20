"""Receive-only SocketCAN capture and offline PAIL signal-change inspection."""
import argparse
import json
from pathlib import Path
import socket
import struct
import time

from .remdevice_protocol import decode_auxiliary, decode_digital


# Standard data frames only. CAN_ERR_FLAG must not be included here: Linux
# registers masks containing that bit on its error-frame receive list.
STANDARD_DATA_MASK = 0x80000000 | 0x40000000 | 0x7FF  # EFF flag, RTR flag, SFF ID.


def decode_frame(cob_id, data, node_id):
    if cob_id == 0x180 + node_id and len(data) == 8:
        return decode_digital(data)
    if cob_id == 0x280 + node_id and len(data) == 8:
        return {f'A{i + 1}': value for i, value in enumerate(data)}
    if cob_id == 0x380 + node_id and len(data) == 8:
        return decode_auxiliary(data)
    if cob_id == 0x700 + node_id and len(data) == 1:
        return {'NMT': data[0]}
    if cob_id == 0x80 + node_id and len(data) == 8:
        return {'EMCY': int.from_bytes(bytes(data[:2]), 'little'), 'error_register': data[2]}
    return None


def changes(previous, decoded):
    result = {name: value for name, value in decoded.items() if previous.get(name) != value}
    previous.update(decoded)
    return result


def inspect_trace(path, node_id):
    previous = {}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        record = json.loads(line)
        if 'label' in record:
            print('LABEL:', record['label'])
        if 'id' not in record:
            continue
        decoded = decode_frame(record['id'], record['data'], node_id)
        if decoded is None:
            continue
        delta = changes(previous, decoded)
        if delta:
            print(record['stamp_ns'], f"{record['id']:03X}", json.dumps(delta, sort_keys=True))


def capture(interface, path, node_id, label, seconds):
    if not hasattr(socket, 'AF_CAN'):
        raise RuntimeError('Live capture requires Linux SocketCAN; offline --read works on Windows.')
    # Linux classic CAN frame: can_id, length, padding, eight data bytes.
    wire = struct.Struct('=IB3x8s')
    ids = [base + node_id for base in (0x180, 0x280, 0x380, 0x700, 0x80)]
    filters = b''.join(struct.pack('=II', cob, STANDARD_DATA_MASK) for cob in ids)
    previous = {}
    count = 0
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
        bus.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        bus.settimeout(0.2)
        bus.bind((interface,))
        with Path(path).open('a', encoding='utf-8', buffering=1) as output:
            output.write(json.dumps({'stamp_ns': time.time_ns(), 'label': label,
                                     'interface': interface, 'node_id': node_id}) + '\n')
            print(f'Receive-only capture for {seconds:g}s. Label: {label}. Move only this control, then return neutral.')
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                try:
                    packet = bus.recv(wire.size)
                except socket.timeout:
                    continue
                stamp = time.time_ns()
                if len(packet) != wire.size:
                    continue
                cob, dlc, payload = wire.unpack(packet)
                if dlc > 8:
                    continue
                data = list(payload[:dlc])
                output.write(json.dumps({'stamp_ns': stamp, 'id': cob, 'data': data}) + '\n')
                count += 1
                decoded = decode_frame(cob, data, node_id)
                if decoded is not None:
                    delta = changes(previous, decoded)
                    if delta:
                        print(f'{cob:03X}', json.dumps(delta, sort_keys=True))
    print(f'Captured {count} frames; appended to {path}.')
    if not count:
        raise RuntimeError(f'No receiver frames on {interface} for node {node_id}. '
                           'Check the configured bitrate, receiver power and NMT state.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--node-id', type=int, default=9)
    parser.add_argument('--read', metavar='JSONL', help='Inspect a saved capture; no CAN access')
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--output', default='pail.jsonl', help='Capture is appended, never overwritten')
    parser.add_argument('--label', default='unlabelled')
    parser.add_argument('--seconds', type=float, default=8.0)
    args = parser.parse_args()
    if not 1 <= args.node_id <= 127 or not 0 < args.seconds <= 3600:
        parser.error('node-id must be 1..127 and seconds must be in (0, 3600]')
    try:
        if args.read:
            inspect_trace(args.read, args.node_id)
        else:
            capture(args.interface, args.output, args.node_id, args.label, args.seconds)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, str(error) + '\n')
    except KeyboardInterrupt:
        parser.exit(130, 'Capture stopped. Recorded frames remain in the output file.\n')


if __name__ == '__main__':
    main()
