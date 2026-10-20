"""The guided recorder must preserve evidence and stop on missing control PDOs."""
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).parents[1] / 'checks/map_pail.py'
spec = importlib.util.spec_from_file_location('pail_mapping', SOURCE)
mapper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapper)


def fake_capture(interface, output, node_id, label, seconds):
    records = [
        {'label': label, 'interface': interface, 'node_id': node_id},
        {'id': 0x180 + node_id, 'data': [1, 0, 1, 0, 0, 0, 0, 0x28]},
        {'id': 0x280 + node_id, 'data': [255, 0, 0, 0, 0, 0, 0, 0]},
    ]
    with output.open('a') as stream:
        for record in records:
            stream.write(json.dumps(record) + '\n')


class PailMappingTest(unittest.TestCase):
    def test_complete_session_preserves_labels_and_node_ten_signals(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'mapping.jsonl'
            with patch.object(mapper, 'capture', side_effect=fake_capture) as capture, \
                    patch('builtins.input', return_value=''), redirect_stdout(io.StringIO()):
                mapper.run_session('can0', 10, 6, output)
            self.assertEqual(capture.call_count, len(mapper.STEPS))
            self.assertEqual([json.loads(line)['label'] for line in output.read_text().splitlines()
                              if 'label' in json.loads(line)], [label for label, _ in mapper.STEPS])
            summary = json.loads(output.with_suffix('.summary.json').read_text())
            self.assertEqual(summary['node_id'], 10)
            neutral = summary['segments']['neutral']
            self.assertEqual(neutral['frames'], {'18A': 1, '28A': 1})
            self.assertEqual(neutral['analog_ranges']['A1'], [255, 255])
            self.assertEqual(neutral['digital_true_counts']['U1'], 1)
            self.assertEqual(neutral['digital_true_counts']['D1'], 0)

    def test_heartbeat_without_control_pdos_stops_and_keeps_evidence(self):
        def heartbeat_only(interface, output, node_id, label, seconds):
            output.write_text(json.dumps({'label': label}) + '\n' +
                              json.dumps({'id': 0x700 + node_id, 'data': [127]}) + '\n')
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'mapping.jsonl'
            with patch.object(mapper, 'capture', side_effect=heartbeat_only) as capture, \
                    patch('builtins.input', return_value=''), redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError, 'TPDO1 and analog TPDO2'):
                    mapper.run_session('can0', 10, 6, output)
            self.assertEqual(capture.call_count, 1)
            summary = json.loads(output.with_suffix('.summary.json').read_text())
            self.assertEqual(summary['segments']['neutral']['frames'], {'70A': 1})

    def test_interruption_preserves_completed_steps(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'mapping.jsonl'
            with patch.object(mapper, 'capture', side_effect=fake_capture), \
                    patch('builtins.input', side_effect=['', KeyboardInterrupt]), \
                    redirect_stdout(io.StringIO()):
                with self.assertRaises(KeyboardInterrupt):
                    mapper.run_session('can0', 10, 6, output)
            summary = json.loads(output.with_suffix('.summary.json').read_text())
            self.assertEqual(list(summary['segments']), ['neutral'])


if __name__ == '__main__':
    unittest.main()
