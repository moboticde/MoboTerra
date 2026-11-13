"""Combine an existing recorder capture into one timestamp-sorted BUSMASTER log."""
import argparse
from decimal import Decimal
import json
from pathlib import Path
import re

from busmaster_trace import BusmasterTrace


def timestamp_ns(text):
    return int(Decimal(text) * 1_000_000_000)


def export(root, output=None):
    metadata = json.loads((root / 'metadata.json').read_text())
    summary = json.loads((root / 'summary.json').read_text())
    path = output or root / 'busmaster.log'
    bitrate = metadata['interface_before'].get('linkinfo', {}).get('info_data', {}).get('bittiming', {}).get('bitrate', 500000)
    writer = BusmasterTrace(path, metadata['started_ns'], metadata['interface'], bitrate)
    try:
        with (root / 'can_rx.log').open() as received:
            for line in received:
                match = re.match(r'\(([0-9.]+)\)\s+\S+\s+([0-9A-F]+)#(R[0-8]|[0-9A-F]*)$', line.strip())
                if not match:
                    raise ValueError('Unrecognized raw CAN line: ' + line[:100])
                stamp, label, payload = match.groups()
                cob = int(label, 16)
                error = bool(cob & 0x20000000)
                extended = len(label) == 8 and not error
                rtr = payload.startswith('R')
                dlc = int(payload[1:]) if rtr else len(payload) // 2
                writer.frame(timestamp_ns(stamp), 'Rx', cob, dlc,
                    b'' if rtr else bytes.fromhex(payload), extended=extended, rtr=rtr, error=error)
        with (root / 'ros_can_tx.jsonl').open() as transmitted:
            for line in transmitted:
                item = json.loads(line)
                writer.frame(item['source_ns'] or item['received_ns'], 'Tx', item['id'], item['dlc'],
                    b'' if item['rtr'] else bytes.fromhex(item['data']),
                    extended=item['extended'], rtr=item['rtr'], error=item['error'])
        with (root / 'joystick_events.log').open() as events:
            for line in events:
                match = re.match(r'\(([0-9.]+)\) (.*)', line)
                if match:
                    writer.note(timestamp_ns(match[1]), match[2])
        writer.close(summary['finished_ns'])
    except BaseException:
        if not writer.closed:
            writer.close(summary['finished_ns'])
        raise
    print(f'{path}: {writer.frames} frames, {writer.notes} event notes; timestamp-sorted')
    return path


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path)
    args = parser.parse_args()
    for capture in args.captures:
        export(capture)
