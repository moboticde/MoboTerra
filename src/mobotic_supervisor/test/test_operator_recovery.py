"""Faults wait for a human correction and a new B2 transaction, without restart."""
from types import SimpleNamespace
import unittest

import test_enable_gate as harness
import test_reset_recovery as recovery


class OperatorRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.fixture = recovery.ResetRecoveryTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.fault_codes.clear()
        self.node.operator_drives_disabled = True
        self.logs = []
        self.node.get_logger = lambda: SimpleNamespace(warning=self.logs.append, info=lambda _: None)

    @property
    def now(self):
        return self.fixture.platform.now

    def b2(self):
        return self.node._enable_drives_service(None, SimpleNamespace())

    def complete_b2_recovery(self):
        response = self.b2()
        self.assertTrue(response.success, response.message)
        self.fixture.inputs(11.1, fault=0, ack=True)
        self.node._control()
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)
        for stamp in (11.2, 11.45):
            self.fixture.inputs(stamp, enabled=True, motion=True, fault=0, ack=True)
            self.node._control()
        self.assertIsNone(self.node.reset_started_at)
        self.assertFalse(self.node.fault_codes)
        self.assertFalse(self.node.operator_drives_disabled)
        self.assertIsNone(self.node.manual_command)
        self.assertFalse(self.fixture.platform.states[-1][2])
        self.assertFalse(self.fixture.fixture.publish_state(permitted=False).motion_permitted)

    def test_startup_waits_for_b2_without_disabled_drive_motion_telemetry(self):
        for module in self.node.wheel_status.modules:
            module.startup_complete = module.startup_failed = module.startup_pending = False
            module.initialized = False
        for _ in range(20):
            self.node._control()
        self.assertTrue(all(not msg.enable for msg in self.node.module_command_publisher.messages))
        self.assertIn('press B2', self.node._operator_wait_status(self.now))
        response = self.b2()
        self.assertTrue(response.success, response.message)
        self.node._control()
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
        self.assertGreater(self.node.module_command_publisher.messages[-1].reset_id, 0)
        self.fixture.inputs(11.1, fault=0, ack=True)
        self.node._control()
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)

    def test_charger_disconnect_then_b2_recovers_without_program_restart(self):
        self.node.battery_state.system_ready = False
        self.node.battery_state.status_message = 'charging or charger mains connected; drive readiness inhibited'
        self.node._control()
        message = self.node._operator_wait_status(self.now)
        self.assertIn('Disconnect the charger', message)
        self.assertIn('press B2', message)
        self.assertFalse(self.b2().success)
        self.node.battery_state.system_ready = True
        self.node.battery_state.status_message = 'battery system ready'
        for _ in range(10):
            self.node._control()
            self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
        self.complete_b2_recovery()

    def test_stale_battery_telemetry_then_b2_recovers_without_program_restart(self):
        self.node.battery_received = harness.Time(9)
        self.node._control()
        self.assertIn('BMS_MONITOR_LOST', self.node.fault_codes)
        self.assertIn('Check battery power and CAN', self.node._operator_wait_status(self.now))
        rejected = self.b2()
        self.assertFalse(rejected.success)
        self.assertIn('Action:', rejected.message)
        self.node.battery_received = self.now
        self.node._control()
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
        self.complete_b2_recovery()

    def test_joystick_restart_then_b2_recovers_without_program_restart(self):
        self.node.manual_state.connected = False
        self.node.manual_state.status_message = 'joystick receiver fault EMCY=0x8150'
        self.node._control()
        message = self.node._operator_wait_status(self.now)
        self.assertIn('0x8150', message)
        self.assertIn('restart the joystick and receiver', message)
        self.assertFalse(self.b2().success)
        self.node.manual_state.connected = True
        self.node.manual_state.status_message = 'neutral'
        self.node._control()
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
        self.complete_b2_recovery()

    def test_startup_failure_then_b2_retries_without_program_restart(self):
        for module in self.node.wheel_status.modules:
            module.startup_complete = False
            module.startup_failed = True
            module.initialized = False
        self.node._control()
        self.assertIn('DRIVE_STARTUP_FAILED:front', self.node.fault_codes)
        self.assertNotIn('restart driver', self.node._operator_wait_status(self.now))
        self.complete_b2_recovery()

    def test_deflection_or_missing_native_status_cannot_start_initial_recovery(self):
        for kind in ('deflected', 'unknown', 'stale', 'moving'):
            with self.subTest(kind=kind):
                self.setUp()
                if kind == 'deflected':
                    self.node.manual_state.controls_neutral = False
                elif kind == 'unknown':
                    self.node.wheel_status.modules[0].steering.device_state_known = False
                elif kind == 'stale':
                    self.node.wheel_status.modules[0].steering.status_stamp = harness.Time(9).to_msg()
                else:
                    self.fixture.inputs(11, motion=True)
                    self.node.joint_state.velocity[0] = 1.0
                self.assertFalse(self.b2().success)
                self.assertTrue(self.node.operator_drives_disabled)

    def test_wait_message_is_published_and_rate_limited_while_program_keeps_running(self):
        self.node.battery_received = harness.Time(9)
        for _ in range(100):
            self.node._control()
        waits = [message for message in self.logs if message.startswith('WAITING:')]
        self.assertEqual(len(waits), 1)
        state = self.fixture.fixture.publish_state()
        self.assertIn('B2_ENABLE_REQUIRED', state.reason_codes)
        self.assertIn('Action:', state.status_message)
        self.assertFalse(state.motion_permitted)
        self.assertEqual(len(self.node.module_command_publisher.messages), 100)
        self.node.battery_received = self.now
        self.node._control()
        self.assertIn('press B2', self.logs[-1])

    def test_manual_receiver_fault_never_reports_ready_or_accepts_enable(self):
        self.node.require_operator_interlock = False
        self.node.manual_state.connected = False
        self.node.manual_state.controls_neutral = False
        self.node.manual_state.status_message = 'joystick receiver fault EMCY=0x8150 (CAN-ID collision)'
        self.node._control()
        message = self.node._operator_wait_status(self.now)
        self.assertIn('EMCY=0x8150', message)
        self.assertIn('CAN node/COB-ID', message)
        self.assertNotIn('ready for drive recovery', message)
        response = self.b2()
        self.assertFalse(response.success)
        self.assertIn('EMCY=0x8150', response.message)
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)

    def test_stale_manual_operator_never_reports_ready_or_accepts_enable(self):
        self.node.require_operator_interlock = False
        self.node.manual_state_received = harness.Time(9)
        self.node._control()
        self.assertIn('joystick telemetry stale', self.node._operator_wait_status(self.now))
        self.assertFalse(self.b2().success)

    def test_recovery_advice_identifies_each_main_hardware_action(self):
        for reason, action in (
            ('front_left scanner field data unavailable', 'scanner power and Ethernet'),
            ('protective stop active', 'Clear the scanner protective fields'),
            ('emergency stop active in safety I/O state', 'Release the emergency stops'),
            ('safety communication unavailable', 'FlexiSoft power and network'),
            ('driver status stale', 'drive power and CAN'),
            ('battery high voltage is not connected', 'high-voltage supply'),
            ('fresh neutral operator controls required', 'all joystick axes to neutral'),
        ):
            with self.subTest(reason=reason):
                self.assertIn(action, self.node._recovery_action(reason))

    def test_repeated_b2_during_initialization_keeps_one_recovery_transaction(self):
        self.assertTrue(self.b2().success)
        reset_id = self.node.reset_id
        started = self.node.reset_started_at
        for _ in range(10):
            response = self.b2()
            self.assertTrue(response.success, response.message)
            self.assertIn('already in progress', response.message)
            self.assertEqual(self.node.reset_id, reset_id)
            self.assertIs(self.node.reset_started_at, started)
            self.assertFalse(self.node.operator_drives_disabled)
        self.fixture.inputs(11.1, fault=0, ack=True)
        self.node._control()
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)

    def test_repeated_b2_when_enabled_does_not_disable_or_reinitialize(self):
        self.complete_b2_recovery()
        reset_id = self.node.reset_id
        for _ in range(10):
            response = self.b2()
            self.assertTrue(response.success, response.message)
            self.assertIn('already enabled', response.message)
            self.assertEqual(self.node.reset_id, reset_id)
            self.assertIsNone(self.node.reset_started_at)
            self.assertFalse(self.node.operator_drives_disabled)

    def test_repeated_b2_while_charging_never_admits_drive_initialization(self):
        self.node.battery_state.system_ready = False
        self.node.battery_state.status_message = 'charging or charger mains connected'
        for _ in range(10):
            self.assertFalse(self.b2().success)
            self.node._control()
        self.assertEqual(self.node.reset_id, 0)
        self.assertIsNone(self.node.reset_started_at)
        self.assertTrue(all(not msg.enable for msg in self.node.module_command_publisher.messages))

    def test_timeout_and_controller_fault_stay_visible_with_retry_instructions(self):
        for reason in ('RESET_TIMEOUT', 'MODE_TRANSITION_TIMEOUT', 'controller'):
            with self.subTest(reason=reason):
                self.setUp()
                for module in self.node.wheel_status.modules:
                    module.error_code = 0
                if reason == 'controller':
                    self.node.wheel_status.modules[0].error_code = 0x2310
                else:
                    self.node.fault_codes.add(reason)
                message = self.node._operator_wait_status(self.now)
                self.assertIn('0x2310' if reason == 'controller' else 'timed out', message)
                self.assertIn('press B2 to retry', message)
                self.assertIn('program can remain running', message)


if __name__ == '__main__':
    unittest.main()


class HomingOperatorSelectionTest(unittest.TestCase):
    def setUp(self):
        self.fixture = OperatorRecoveryTest(); self.fixture.setUp()
        self.node = self.fixture.node
        self.now = self.fixture.now
        self.node.manual_state.source_id = 'remdevice'
        self.node.manual_state.estop_pressed = False
        self.node.manual_state.connected = False
        self.node.homing_state = SimpleNamespace(source_id='remdevice_homing',
            header=SimpleNamespace(stamp=self.now.to_msg()), connected=True,
            controls_neutral=True, estop_state_known=True, estop_pressed=False)
        self.node.homing_state_received = self.now

    def test_initial_enable_requires_normal_operator(self):
        self.node.reset_started_at = None
        self.assertIs(self.node._recovery_operator(self.now)[0], self.node.manual_state)

    def test_active_homing_uses_physical_interlocks(self):
        self.node.reset_started_at = self.now
        self.assertIs(self.node._recovery_operator(self.now)[0], self.node.homing_state)

    def test_stop_or_stale_homing_channel_cannot_be_overridden(self):
        self.node.reset_started_at = self.now
        self.node.manual_state.estop_pressed = True
        self.assertIs(self.node._recovery_operator(self.now)[0], self.node.manual_state)
        self.node.manual_state.estop_pressed = False
        self.node.homing_state_received = harness.Time(0)
        self.assertIs(self.node._recovery_operator(self.now)[0], self.node.manual_state)

    def test_diagnostics_do_not_create_operator_loss_during_homing(self):
        self.node.reset_started_at = self.now
        ready, reason = self.node._reset_external_gate(self.now, allow_drive_faults=True)
        self.assertTrue(ready, reason)
        errors, _ = self.node._system_conditions(self.now)
        self.assertNotIn('OPERATOR_LOST', errors)
        # Without an active homing transaction, normal diagnostic faults apply.
        self.node.reset_started_at = None
        self.assertFalse(self.node._reset_external_gate(self.now, allow_drive_faults=True)[0])

    def test_homing_physical_stop_still_rejects_recovery(self):
        self.node.reset_started_at = self.now
        self.node.homing_state.estop_pressed = True
        ready, reason = self.node._reset_external_gate(self.now, allow_drive_faults=True)
        self.assertFalse(ready)
        self.assertIn('E-stop', reason)
