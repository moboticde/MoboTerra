"""Exercise receive-only capture and offline signal inspection without hardware."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import socket
import struct
import tempfile
import unittest
from unittest.mock import patch

from mobotic_manual_control.remdevice_trace import capture, decode_frame, inspect_trace


class TraceTest(unittest.TestCase):
    def test_decodes_receiver_nine_and_ignores_other_nodes(self):
        self.assertIsNone(decode_frame(0x18a, [0] * 8, 9))
        self.assertEqual(decode_frame(0x289, list(range(8)), 9)['A8'], 7)
        self.assertEqual(decode_frame(0x709, [127], 9), {'NMT': 127})
        self.assertEqual(decode_frame(0x089, [0x30, 0x81, 1, 0, 0, 0, 0, 0], 9)['EMCY'], 0x8130)

    def test_capture_appends_raw_frames_and_never_transmits(self):
        class ReceiveOnlyBus:
            def __enter__(self): return self
            def __exit__(self, *args): pass
            def setsockopt(self, *args): pass
            def settimeout(self, *args): pass
            def bind(self, *args): pass
            def send(self, *args): raise AssertionError('capture must never transmit')
            def recv(self, count):
                if getattr(self, 'received', False): raise socket.timeout()
                self.received = True
                return struct.pack('=IB3x8s', 0x189, 8, bytes([0] * 7 + [0x28]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pail.jsonl'
            path.write_text('{"label":"earlier segment"}\n', encoding='utf-8')
            with patch.multiple(socket, AF_CAN=29, SOCK_RAW=3, CAN_RAW=1, SOL_CAN_RAW=101,
                                CAN_RAW_FILTER=1, create=True), patch.object(socket, 'socket', return_value=ReceiveOnlyBus()), redirect_stdout(io.StringIO()):
                capture('can0', path, 9, 'neutral', 0.002)
            records = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(records[0]['label'], 'earlier segment')
            self.assertEqual(records[1]['label'], 'neutral')
            self.assertEqual(records[2]['id'], 0x189)
            output = io.StringIO()
            with redirect_stdout(output): inspect_trace(path, 9)
            self.assertIn('LABEL: neutral', output.getvalue())
            self.assertIn('"EMGY": true', output.getvalue())
