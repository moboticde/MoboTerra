import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('host_health', Path(__file__).parents[1] /
                                             'mobotic_bringup/host_health.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class HostHealthTest(unittest.TestCase):
    def test_missing_corrupt_stale_and_future_state_inhibit_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'state.json'
            self.assertEqual(module.read_host_health(p, 100)[:2], (False, True))
            for value in ('bad JSON', 'null', '{}',
                          json.dumps({'state': 'RUNNING', 'updated_at': 94}),
                          json.dumps({'state': 'RUNNING', 'updated_at': 101}),
                          json.dumps({'state': 'RUNNING', 'updated_at': True})):
                p.write_text(value)
                self.assertEqual(module.read_host_health(p, 100)[:2], (False, True))

    def test_error_and_recovery_are_distinct_from_container_health(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'state.json'
            p.write_text(json.dumps({'state': 'ERROR', 'detail': 'Battery adapter missing', 'updated_at': 100}))
            valid, fault, reason = module.read_host_health(p, 100)
            self.assertTrue(valid and fault)
            self.assertIn('Battery adapter missing', reason)
            p.write_text(json.dumps({'state': 'RUNNING', 'updated_at': 100}))
            self.assertEqual(module.read_host_health(p, 101), (True, False, ''))
