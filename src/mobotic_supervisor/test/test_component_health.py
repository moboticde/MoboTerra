"""Report liveness and measured-input freshness use their own deadlines."""
from types import SimpleNamespace as NS
import unittest

import test_command_recovery as recovery
import test_enable_gate as h


class ComponentHealthTest(unittest.TestCase):
    def setUp(self):
        self.fixture = recovery.CommandRecoveryTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.required_health_sources = ['kinematics', 'odometry']
        self.node.health_timeout = .3
        self.node.health_input_timeout = .5
        self.node.component_health = {}
        self.node.component_health_report_stamps = {}
        self.node.started_at = h.Time(1)
        self.node.startup_grace = 0
        self.node.drive_startup_requested = True
        self.node.drive_startup_completed_at = h.Time(1)

    def report(self, source=9.65, reported=None, valid=True, fault=False):
        if reported is None:
            reported = self.fixture.platform.now.nanoseconds / 1e9
        message = NS(header=NS(stamp=h.Time(reported).to_msg()), components=[
            NS(header=NS(stamp=h.Time(source).to_msg()), component_id=name,
               monitoring_valid=valid, fault_active=fault)
            for name in self.node.required_health_sources])
        self.node._health_callback(message)
        return message

    def test_new_report_accepts_input_within_producer_deadline_without_restamping_it(self):
        report = self.report()
        self.assertTrue(self.node._motion_hardware_gate(self.fixture.platform.now)[0])
        errors, _ = self.node._system_conditions(self.fixture.platform.now)
        self.assertFalse(any(code.startswith('COMPONENT_HEALTH_LOST:') for code in errors))
        for name in self.node.required_health_sources:
            self.assertEqual(self.node.component_health[name][0].header.stamp, h.Time(9.65).to_msg())
            self.assertIs(self.node.component_health_report_stamps[name], report.header.stamp)

    def test_fresh_reports_cannot_keep_expired_measurements_alive(self):
        self.report(source=10)
        self.fixture.advance(.501)
        self.report(source=10)
        for name in self.node.required_health_sources:
            self.assertFalse(self.node._component_health_fresh(name, self.fixture.platform.now))
        self.assertFalse(self.node._motion_hardware_gate(self.fixture.platform.now)[0])

    def test_report_watchdog_stays_300ms_even_when_measurement_is_still_valid(self):
        self.report(source=10)
        self.fixture.advance(.301)
        self.assertTrue(self.node._stamp_valid(h.Time(10).to_msg(), .5))
        self.assertFalse(self.node._component_health_fresh('kinematics', self.fixture.platform.now))

    def test_old_or_replayed_reports_cannot_renew_receipt_or_replace_input(self):
        self.report(source=9.9, reported=10)
        original = self.node.component_health['kinematics']
        self.fixture.advance(.2)
        self.report(source=10.2, reported=10)
        self.assertIs(self.node.component_health['kinematics'], original)
        self.report(source=10.2, reported=9.8)
        self.assertIs(self.node.component_health['kinematics'], original)
        self.fixture.advance(.101)
        self.assertFalse(self.node._component_health_fresh('kinematics', self.fixture.platform.now))

    def test_invalid_monitoring_and_active_faults_still_inhibit_motion_immediately(self):
        for valid, fault in ((False, False), (True, True)):
            with self.subTest(valid=valid, fault=fault):
                self.setUp()
                self.report(valid=valid, fault=fault)
                self.assertFalse(self.node._motion_hardware_gate(self.fixture.platform.now)[0])
                self.assertIn('COMPONENT_HEALTH_LOST:kinematics', self.node._system_conditions(self.fixture.platform.now)[0])

    def test_host_input_retains_its_existing_300ms_deadline(self):
        self.node.required_health_sources = ['host']
        self.report(source=9.65)
        self.assertFalse(self.node._component_health_fresh('host', self.fixture.platform.now))

    def test_duplicate_component_ids_clear_reports_and_input_cache(self):
        message = self.report()
        self.fixture.advance(.05)
        message.header.stamp = self.fixture.platform.now.to_msg()
        message.components[1].component_id = 'kinematics'
        self.node._health_callback(message)
        self.assertEqual(self.node.component_health, {})
        self.assertEqual(self.node.component_health_report_stamps, {})

    def test_partial_reports_cannot_renew_a_missing_component(self):
        self.report(source=10)
        self.fixture.advance(.2)
        message = NS(header=NS(stamp=self.fixture.platform.now.to_msg()), components=[
            NS(header=NS(stamp=self.fixture.platform.now.to_msg()), component_id='kinematics',
               monitoring_valid=True, fault_active=False)])
        self.node._health_callback(message)
        self.fixture.advance(.101)
        self.assertTrue(self.node._component_health_fresh('kinematics', self.fixture.platform.now))
        self.assertFalse(self.node._component_health_fresh('odometry', self.fixture.platform.now))
