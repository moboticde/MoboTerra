#!/usr/bin/env python3
"""Guide a labelled, receive-only PAIL capture on the real joystick CAN bus."""
import argparse
from collections import Counter
from contextlib import redirect_stdout
from datetime import datetime
import json
from pathlib import Path
import re
import sys

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / 'src/mobotic_manual_control'))
from mobotic_manual_control.remdevice_trace import capture, decode_frame


STEPS = (
    ('neutral', 'Leave all sticks centred, release B1/B2, and set RP1 to MANUAL.'),
    ('speed-forward', 'Move the CROSS joystick fully FORWARD and hold.'),
    ('speed-reverse', 'Move the CROSS joystick fully BACKWARD and hold.'),
    ('crab-left', 'Move the RIGHT STICK fully LEFT and hold.'),
    ('crab-right', 'Move the RIGHT STICK fully RIGHT and hold.'),
    ('steer-left', 'Move the LEFT PADDLE fully LEFT and hold.'),
    ('steer-right', 'Move the LEFT PADDLE fully RIGHT and hold.'),
    ('mode-manual', 'Keep sticks centred and set RP1 to MANUAL.'),
    ('mode-auto', 'Keep sticks centred and set RP1 to AUTO.'),
    ('rp2-safety-enabled', 'Keep sticks centred and set RP2 to SAFETY ENABLED.'),
    ('rp2-safety-disabled', 'Keep sticks centred and set RP2 to SAFETY DISABLED.'),
    ('boost-released', 'Return RP1 to MANUAL. Leave B1 released and sticks centred.'),
    ('boost-pressed', 'Keep sticks centred and hold B1 (boost).'),
    ('reset-released', 'Release B1/B2 and keep sticks centred.'),
    ('reset-pressed', 'Keep sticks centred and hold B2 (reset).'),
    ('neutral-end', 'Release B1/B2, centre all sticks, and leave RP1 in MANUAL.'),
)


def summarize(path, node_id):
    segments = {}
    segment = None
    for line in path.read_text().splitlines():
        record = json.loads(line)
        if 'label' in record:
            segment = segments.setdefault(record['label'], {
                'frames': Counter(), 'analog_ranges': {},
                'digital_true_counts': Counter(), 'digital_samples': 0,
            })
        if segment is None or 'id' not in record:
            continue
        segment['frames'][f"{record['id']:03X}"] += 1
        decoded = decode_frame(record['id'], record['data'], node_id)
        if decoded is None:
            continue
        if record['id'] == 0x180 + node_id:
            segment['digital_samples'] += 1
            for signal, value in decoded.items():
                segment['digital_true_counts'][signal] += int(value)
        elif record['id'] == 0x280 + node_id:
            for signal, value in decoded.items():
                bounds = segment['analog_ranges'].setdefault(signal, [value, value])
                bounds[0], bounds[1] = min(bounds[0], value), max(bounds[1], value)
    return {'node_id': node_id, 'segments': segments}


def run_session(interface, node_id, seconds, output):
    output.parent.mkdir(parents=True, exist_ok=True)
    output.touch(exist_ok=False)
    log_path = output.with_suffix('.log')
    summary_path = output.with_suffix('.summary.json')
    print(f'Real PAIL receiver: node {node_id} on {interface}; expected bitrate 250000.')
    print('This recorder receives CAN only. It does not launch ROS or send motor/NMT commands.')
    print('Power the receiver/transmitter and establish the radio link as usual.')
    print('Before each step, centre the sticks and release the other buttons.')
    print(f'After Enter, perform the displayed action and hold it for {seconds:g} seconds.')
    print(f'Raw recordings: {output}\n', flush=True)
    try:
        with log_path.open('x') as log:
            for index, (label, instruction) in enumerate(STEPS, 1):
                print(f'[{index}/{len(STEPS)}] {instruction}', flush=True)
                input('Press Enter to begin this recording: ')
                print(f'RECORDING: {instruction}', flush=True)
                with redirect_stdout(log):
                    capture(interface, output, node_id, label, seconds)
                log.flush()
                summary = summarize(output, node_id)
                segment = summary['segments'][label]
                summary_path.write_text(json.dumps(summary, indent=2) + '\n')
                print(f"COMPLETE: {sum(segment['frames'].values())} frames saved. Return controls to neutral.\n",
                      flush=True)
                if not all(segment['frames'].get(f'{base + node_id:03X}', 0)
                           for base in (0x180, 0x280)):
                    raise RuntimeError('Both digital TPDO1 and analog TPDO2 are required; '
                                       'stop here and check receiver/PDO reception.')
    finally:
        summary_path.write_text(json.dumps(summarize(output, node_id), indent=2) + '\n')
        print(f'Raw data saved: {output}', flush=True)
        print(f'Summary saved: {summary_path}', flush=True)
    print('Capture complete. Leave controls neutral. The virtual mapping has not been changed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True,
                        help='Current interface wired to the joystick; independent of saved battery/platform adapter assignments')
    parser.add_argument('--node-id', type=int, default=10)
    parser.add_argument('--seconds', type=float, default=6)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,15}', args.interface):
        parser.error('invalid CAN interface name')
    if not 1 <= args.node_id <= 127 or not 1 <= args.seconds <= 60:
        parser.error('node-id must be 1..127 and seconds must be 1..60')
    output = args.output or REPOSITORY / 'can_telemetry_records' / (
        'pail-mapping-' + datetime.now().strftime('%Y%m%d-%H%M%S') + '.jsonl')
    try:
        flags = int(Path(f'/sys/class/net/{args.interface}/flags').read_text().strip(), 16)
        if not flags & 1:
            raise RuntimeError(f'{args.interface} is DOWN. Bring the isolated joystick bus up at 250000 bit/s first.')
        run_session(args.interface, args.node_id, args.seconds, output.resolve())
    except (OSError, ValueError, RuntimeError) as error:
        print(f'ERROR: {error}', file=sys.stderr)
        if 'No receiver frames' in str(error) or 'TPDO' in str(error):
            print(f'If node {args.node_id} is Pre-Operational, explicitly start that receiver '
                  f'on the isolated joystick bus: cansend {args.interface} 000#01{args.node_id:02X}', file=sys.stderr)
        return 1
    except (KeyboardInterrupt, EOFError):
        print('\nRecording stopped. Existing raw data is preserved.', file=sys.stderr)
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
