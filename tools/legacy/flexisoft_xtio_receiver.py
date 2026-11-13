#!/usr/bin/env python3
"""Receive Flexi Soft TCP data sets and display the supplied XTIO mapping.

SICK 8018172, TCP/IP tables 44/49: big-endian command + four WORD lengths.
This diagnostic sends no application messages and publishes no robot readiness.
"""
import argparse
import json
import socket
import struct
import time
from pathlib import Path

HEADER = struct.Struct('!5H')


class DataSetStream:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, chunk):
        self.buffer.extend(chunk)
        result = []
        while len(self.buffer) >= HEADER.size:
            command, *lengths = HEADER.unpack_from(self.buffer)
            if command not in (0x001F, 0x002E):
                raise ValueError(f'Unexpected data message command 0x{command:04X}; header={self.buffer[:10].hex(" ")}')
            if any(length > 1024 for length in lengths):
                raise ValueError(f'Invalid data set lengths: {lengths}')
            size = HEADER.size + sum(lengths)
            if len(self.buffer) < size:
                break
            offset = HEADER.size
            datasets = {}
            for number, length in enumerate(lengths, 1):
                datasets[number] = bytes(self.buffer[offset:offset + length])
                offset += length
            del self.buffer[:size]
            result.append((command, datasets))
        return result


def xtio(data):
    if len(data) < 17:
        raise ValueError('Data set 1 is shorter than expected (requires at least 17 bytes)')
    return {
        'dataset1_bytes': len(data),
        'logic_results': list(data[:4]),
        'xtio_input_byte': f'0x{data[4]:02X}',
        'xtio_input_bits_b0_b7': [(data[4] >> bit) & 1 for bit in range(8)],
        'xtio_output_byte': f'0x{data[16]:02X}',
        'xtio_output_bits_b0_b3': [(data[16] >> bit) & 1 for bit in range(4)],
        'direct_gateway_bytes_28_31': list(data[28:32]) if len(data) >= 32 else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind', default='10.60.20.250')
    parser.add_argument('--port', type=int, default=12345)
    parser.add_argument('--gateway', default='10.60.20.251')
    parser.add_argument('--seconds', type=float, default=60, help='0 keeps listening until interrupted')
    parser.add_argument('--log', type=Path, default=Path('/home/mobotic/flexisoft_xtio_capture.jsonl'))
    args = parser.parse_args()
    if args.seconds < 0 or not 1 <= args.port <= 65535:
        parser.error('invalid duration or port')
    deadline = time.monotonic() + args.seconds if args.seconds else float('inf')
    counts = {'connections': 0, 'bytes': 0, 'messages': 0, 'dataset1_messages': 0}
    def emit(event, **fields):
        record = {'time': time.time(), 'event': event, **fields}
        line = json.dumps(record)
        print(line, flush=True)
        with args.log.open('a') as log:
            log.write(line + '\n')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.bind, args.port))
        server.listen(2)
        server.settimeout(.5)
        emit('listening', bind=args.bind, port=args.port, expected_gateway=args.gateway)
        while time.monotonic() < deadline:
            try:
                connection, peer = server.accept()
            except socket.timeout:
                continue
            with connection:
                if peer[0] != args.gateway:
                    emit('unexpected_peer', peer=peer)
                    continue
                counts['connections'] += 1
                emit('connected', peer=peer)
                connection.settimeout(.5)
                decoder = DataSetStream()
                previous = None
                last_print = 0
                while time.monotonic() < deadline:
                    try:
                        chunk = connection.recv(8192)
                    except socket.timeout:
                        continue
                    except OSError as error:
                        emit('connection_error', error=str(error))
                        break
                    if not chunk:
                        emit('disconnected', incomplete_bytes=len(decoder.buffer))
                        break
                    counts['bytes'] += len(chunk)
                    try:
                        messages = decoder.feed(chunk)
                        for command, datasets in messages:
                            counts['messages'] += 1
                            if not datasets[1]:
                                continue
                            state = xtio(datasets[1])
                            counts['dataset1_messages'] += 1
                            if state != previous or time.monotonic() - last_print >= 5:
                                emit('xtio', command=f'0x{command:04X}', **state, counts=counts.copy())
                                previous = state
                                last_print = time.monotonic()
                    except ValueError as error:
                        emit('decode_error', error=str(error), raw_chunk=chunk[:128].hex(' '))
                        break
        emit('summary', **counts)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
