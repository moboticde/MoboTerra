"""Supervised drive bootstrap must not depend on enabled-drive telemetry."""
from types import SimpleNamespace
import unittest
import test_enable_gate as harness


class DriveStartupTest(unittest.TestCase):
    def setUp(self):
        harness.EnableGateTest.setUp(self)
        node = self.node
        node.safety_source = 'hardwired_sto'
        node.started_at = harness.Time(1)
        node.startup_grace = 0.0
        node.selected_mode = harness.MODES.AUTO_VELOCITY
        node.required_health_sources = ['kinematics']
        node.health_timeout = 0.3
        node.component_health = {}
        for module in node.wheel_status.modules:
            module.startup_pending = False
            module.startup_complete = False
            module.startup_failed = False
            module.initialized = module.feedback_fresh = module.position_valid = False

    def test_late_readiness_starts_drives_without_motion_or_kinematics_feedback(self):
        node = self.node
        node.required_health_sources = ['kinematics', 'odometry']
        self.assertEqual(node._system_conditions(self.now)[0], [])
        self.assertTrue(node._enable_gate(self.now)[0])
        self.assertFalse(node._base_gate(self.now)[0])
        node._control()
        self.assertTrue(node.module_command_publisher.messages[-1].enable)
        self.assertEqual(node.fault_codes, set())

    def test_waiting_for_initial_inputs_does_not_latch_loss(self):
        node = self.node
        node.required_health_sources = ['kinematics', 'odometry']
        node.battery_state = node.battery_received = None
        node.require_operator_interlock = True
        node.selected_mode = harness.MODES.MANUAL
        node._control()
        self.assertEqual(node.fault_codes, set())
        self.assertFalse(node.module_command_publisher.messages[-1].enable)
        # A loss after an actual enable request must still latch and stop.
        node.drive_startup_requested = True
        node._control()
        self.assertIn('BMS_MONITOR_LOST', node.fault_codes)
        self.assertIn('OPERATOR_LOST', node.fault_codes)

    def test_initial_host_discovery_wait_does_not_latch_before_drive_startup(self):
        node = self.node
        node.required_health_sources = ['host']
        node.component_health = {}
        node._control()
        self.assertFalse(node.module_command_publisher.messages[-1].enable)
        self.assertEqual(node.fault_codes, set())
        record = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()),
                                 monitoring_valid=True, fault_active=False)
        node.component_health = {'host': (record, self.now)}
        node._control()
        self.assertTrue(node.module_command_publisher.messages[-1].enable)
        record.fault_active = True
        node._control()
        self.assertIn('COMPONENT_HEALTH_LOST:host', node.fault_codes)
        self.assertFalse(node.module_command_publisher.messages[-1].enable)

    def test_physical_operator_can_authorize_zero_initialization_while_command_rearms(self):
        node = self.node
        node.require_operator_interlock = True
        node.selected_mode = harness.MODES.MANUAL
        node.manual_state_received = self.now
        node.manual_state = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()),
            connected=True, command_active=False, controls_neutral=False,
            estop_state_known=True, estop_pressed=False)
        node._control()
        self.assertTrue(node.module_command_publisher.messages[-1].enable)
        self.assertFalse(node._source_gate(self.now)[0])
        self.assertEqual(node.last_output, [0.0, 0.0, 0.0])
        node.manual_state.connected = False
        node._control()
        self.assertFalse(node.module_command_publisher.messages[-1].enable)

    def test_initial_battery_fault_can_resolve_before_first_startup(self):
        node = self.node
        node.battery_state.faulted_batteries = ['battery1']
        node._control()
        self.assertFalse(node.module_command_publisher.messages[-1].enable)
        self.assertEqual(node.fault_codes, set())
        node.battery_state.faulted_batteries = []
        node._control()
        self.assertTrue(node.module_command_publisher.messages[-1].enable)
        self.assertTrue(node.drive_startup_requested)

    def test_initial_fault_can_be_cleared_once_by_driver_startup(self):
        self.node.wheel_status.modules[0].error_code = 0x2310
        self.assertTrue(self.node._enable_gate(self.now)[0])
        self.assertEqual(self.node._system_conditions(self.now)[0], [])

    def test_external_interlocks_still_cancel_startup(self):
        node = self.node
        for module in node.wheel_status.modules:
            module.startup_pending = True
        node.battery_state.high_voltage_connected = False
        self.assertFalse(node._enable_gate(self.now)[0])
        node.battery_state.high_voltage_connected = True
        node.require_operator_interlock = True
        self.assertFalse(node._enable_gate(self.now)[0])
        node.require_operator_interlock = False
        node.wheel_status.modules[0].sto_state_known = True
        node.wheel_status.modules[0].sto_active = True
        self.assertFalse(node._enable_gate(self.now)[0])
        node.wheel_status.modules[0].sto_active = False
        node.safety_source = 'flexisoft'
        node.safety_state.motion_permitted = False
        self.assertFalse(node._enable_gate(self.now)[0])

    def test_failed_startup_latches_and_cannot_request_enable(self):
        self.node.wheel_status.modules[0].startup_failed = True
        self.assertFalse(self.node._enable_gate(self.now)[0])
        self.node._control()
        self.assertIn('DRIVE_STARTUP_FAILED:front', self.node.fault_codes)
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)

    def test_completed_startup_restores_all_motion_and_fault_checks(self):
        node = self.node
        for module in node.wheel_status.modules:
            module.startup_complete = True
            module.initialized = module.feedback_fresh = module.position_valid = module.enabled = True
        self.assertTrue(node._base_gate(self.now)[0])
        node.wheel_status.modules[0].feedback_fresh = False
        self.assertFalse(node._base_gate(self.now)[0])
        self.assertIn('DRIVE_FEEDBACK_LOST:front', node._system_conditions(self.now)[0])
        node.wheel_status.modules[0].position_valid = False
        self.assertIn('POSITION_CONTINUITY_UNKNOWN:front', node._system_conditions(self.now)[0])
        node.wheel_status.modules[0].error_code = 1
        self.assertFalse(node._enable_gate(self.now)[0])
        self.assertIn('DRIVE_FAULT:front', node._system_conditions(self.now)[0])

    def test_kinematics_gets_only_bounded_first_feedback_grace(self):
        node = self.node
        for module in node.wheel_status.modules:
            module.startup_complete = True
            module.initialized = module.feedback_fresh = module.position_valid = True
        self.assertNotIn('COMPONENT_HEALTH_LOST:kinematics', node._system_conditions(self.now)[0])
        errors = node._system_conditions(harness.Time(10.7))[0]
        self.assertIn('COMPONENT_HEALTH_LOST:kinematics', errors)

    def prepare_reset_without_motion_feedback(self):
        node = self.node
        node.reset_started_at = harness.Time(9.9)
        node.reset_timeout = 5.0
        node.reset_id = 1
        node.fault_codes = {'DRIVE_FAULT:front'}
        for module in node.wheel_status.modules:
            module.startup_complete = module.initialized = True
            module.reset_id = 1
            module.reset_acknowledged = True
        return node

    def test_acknowledged_reset_can_reenable_to_reacquire_motion_feedback(self):
        node = self.prepare_reset_without_motion_feedback()
        node._control()
        operation = node.module_command_publisher.messages[-1]
        self.assertTrue(operation.enable)
        self.assertEqual(operation.reset_id, 1)
        self.assertIsNotNone(node.reset_started_at)
        self.assertTrue(node.fault_codes)  # Recovery is not yet complete.
        self.assertFalse(node.motion_inputs_ready)

    def test_reset_reenable_needs_every_ack_and_external_readiness(self):
        node = self.prepare_reset_without_motion_feedback()
        node.wheel_status.modules[0].reset_acknowledged = False
        node._control()
        self.assertFalse(node.module_command_publisher.messages[-1].enable)
        node.wheel_status.modules[0].reset_acknowledged = True
        node.battery_state.high_voltage_connected = False
        node._control()
        self.assertFalse(node.module_command_publisher.messages[-1].enable)
        node.battery_state.high_voltage_connected = True
        node.battery_state.faulted_batteries = ['battery1']
        node._control()
        self.assertFalse(node.module_command_publisher.messages[-1].enable)

    def test_reset_reacquires_odometry_after_disabled_drive_feedback_loss(self):
        node = self.prepare_reset_without_motion_feedback()
        node.required_health_sources = ['kinematics', 'odometry']
        node.drive_startup_completed_at = harness.Time(9)
        node._control()
        self.assertTrue(node.module_command_publisher.messages[-1].enable)
        self.assertIn('COMPONENT_HEALTH_LOST:odometry', node.fault_codes)


if __name__ == '__main__':
    unittest.main()
