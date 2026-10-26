"""Readiness distinguishes validated configuration from live hardware evidence."""
import copy
import sys
from pathlib import Path
from types import SimpleNamespace as Msg
import unittest

PACKAGES = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGES / 'mobotic_config'))
sys.path.insert(0, str(PACKAGES / 'mobotic_bringup'))
from mobotic_config.configuration import Configuration
from mobotic_bringup.readiness import configuration_report, live_report


class ReadinessTest(unittest.TestCase):
    def setUp(self):
        self.cfg = Configuration(PACKAGES / 'mobotic_config/config/platform-front-right-rear-left.yaml')
        header = Msg(stamp=Msg(sec=10, nanosec=0))
        self.samples = {
            'vehicle/mode': (Msg(header=header, status_message='waiting for a motion command', motion_permitted=False,
                error_active=False, reset_in_progress=False, safety_stop_active=False, readiness_known=True,
                drives_enabled=True, ready_to_drive=True, reason_codes=[], selected_mode=0), 10_000_000_000),
            'wheel_modules/status': (Msg(header=header, modules=[Msg(name=name, startup_failed=False, startup_complete=True,
                initialized=True, enabled=True, position_valid=True, feedback_fresh=True, error_code=0, status_message='')
                for name in ('front_right', 'rear_left')]), 10_000_000_000),
            'battery/system_state': (Msg(header=header, communication_ok=True, system_ready=True,
                high_voltage_connected=True, faulted_batteries=[], minimum_percentage=.8, status_message=''), 10_000_000_000),
            'operator/state': (Msg(header=header, connected=True, estop_state_known=True, estop_pressed=False,
                status_message='neutral'), 10_000_000_000),
        }

    def test_offline_check_cannot_claim_live_readiness(self):
        report = configuration_report(self.cfg)
        self.assertTrue(report['configuration_ready'])
        self.assertFalse(report['live_checked'])
        self.assertIsNone(report['software_ready'])
        self.assertFalse(report['hardware_verified'])
        self.assertEqual(report['receiver_node'], 10)

    def test_unverified_four_module_operator_remains_a_configuration_blocker(self):
        self.assertFalse(configuration_report(Configuration())['configuration_ready'])

    def test_ready_neutral_operator_needs_no_motion_command(self):
        report = live_report(self.cfg, self.samples, 10_000_000_000)
        self.assertTrue(report['software_ready'], report['blockers'])
        self.assertFalse(report['motion_permitted'])
        self.assertFalse(report['hardware_verified'])

    def test_missing_stale_and_future_messages_cannot_pass(self):
        for key in self.samples:
            for mutation in ('missing', 'receipt', 'acquisition', 'future'):
                samples = copy.deepcopy(self.samples)
                if mutation == 'missing': del samples[key]
                elif mutation == 'receipt': samples[key] = (samples[key][0], 9_000_000_000)
                else: samples[key][0].header.stamp.sec = 9 if mutation == 'acquisition' else 11
                report = live_report(self.cfg, samples, 10_000_000_000)
                self.assertFalse(report['software_ready'], (key, mutation))

    def test_startup_fault_feedback_battery_and_operator_blockers(self):
        cases = [('wheel_modules/status', 'startup_failed', True),
                 ('wheel_modules/status', 'startup_complete', False),
                 ('wheel_modules/status', 'feedback_fresh', False),
                 ('wheel_modules/status', 'position_valid', False),
                 ('battery/system_state', 'high_voltage_connected', False),
                 ('battery/system_state', 'minimum_percentage', .1),
                 ('battery/system_state', 'faulted_batteries', ['battery1']),
                 ('operator/state', 'connected', False),
                 ('operator/state', 'estop_state_known', False),
                 ('operator/state', 'estop_pressed', True)]
        for key, field, value in cases:
            samples = copy.deepcopy(self.samples)
            record = samples[key][0]
            if key == 'wheel_modules/status': record = record.modules[0]
            setattr(record, field, value)
            report = live_report(self.cfg, samples, 10_000_000_000)
            self.assertFalse(report['software_ready'], (key, field))

    def test_duplicate_or_missing_modules_are_rejected(self):
        self.samples['wheel_modules/status'][0].modules[1].name = 'front_right'
        self.assertFalse(live_report(self.cfg, self.samples, 10_000_000_000)['software_ready'])


if __name__ == '__main__':
    unittest.main()
