"""Reset a disabled robot without inventing standstill from missing telemetry."""
import copy
from types import SimpleNamespace
import unittest

import test_enable_gate as harness
import test_interface_update as interfaces


class ResetRecoveryTest(unittest.TestCase):
    def setUp(self):
        self.fixture = interfaces.InterfaceUpdateTest()
        self.fixture.setUp()
        self.platform = self.fixture.fixture
        self.node = self.fixture.node
        del self.node._source_gate
        node = self.node
        node.safety_source = 'hardwired_sto'
        node.require_operator_interlock = True
        node.required_health_sources = ['kinematics', 'odometry']
        node.health_timeout = .3
        node.drive_startup_requested = True
        node.drive_startup_completed_at = harness.Time(1)
        node.started_at = harness.Time(1)
        node.startup_grace = 0
        node.fault_codes = {'DRIVE_FAULT:front'}
        node.manual_state = SimpleNamespace(header=SimpleNamespace(stamp=0),
            connected=True, controls_neutral=True, estop_state_known=True,
            estop_pressed=False, command_active=False, status_message='neutral rearm')
        self.inputs(11)

    def inputs(self, seconds, *, enabled=False, motion=False, fault=0x2310, ack=False):
        self.platform.now = harness.Time(seconds)
        now = self.platform.now
        if motion:
            self.platform.feedback()
        node = self.node
        node.manual_state.header.stamp = now.to_msg()
        for name, (state, _) in node.scanner_states.items():
            state.header.stamp = now.to_msg()
            node.scanner_states[name] = (state, now)
        node.manual_state_received = now
        node.battery_state.header.stamp = now.to_msg()
        node.battery_received = now
        node.wheel_status_received = now
        modules = []
        for name, steering, traction in zip(node.expected_module_names,
                node.expected_steering_joint_names, node.expected_traction_joint_names):
            def drive(joint):
                return SimpleNamespace(name=joint, status_known=True, device_state_known=True,
                    enabled=enabled, status_stamp=now.to_msg())
            modules.append(SimpleNamespace(name=name, steering=drive(steering), traction=drive(traction),
                startup_complete=True, startup_pending=False, startup_failed=False, initialized=True,
                sto_state_known=False, sto_active=False,
                enabled=enabled, feedback_fresh=motion, position_valid=motion, error_code=fault,
                reset_id=node.reset_id if ack else 0, reset_acknowledged=ack))
        node.wheel_status = SimpleNamespace(header=SimpleNamespace(stamp=now.to_msg()), modules=modules)
        node.component_health = {name: (SimpleNamespace(header=SimpleNamespace(stamp=now.to_msg()),
            monitoring_valid=motion, fault_active=False), now) for name in node.required_health_sources}

    def request(self):
        return self.node._reset_errors_service(None, SimpleNamespace())

    def acknowledge(self):
        self.assertTrue(self.request().success)
        self.inputs(11.1, fault=0, ack=True)
        self.node._control()
        operation = self.node.module_command_publisher.messages[-1]
        self.assertTrue(operation.enable)
        self.assertEqual(operation.reset_id, 1)
        self.assertTrue(self.node.fault_codes)
        self.assertFalse(self.node.motion_inputs_ready)

    def test_disabled_fault_then_expired_feedback_recovers_without_allowing_motion(self):
        self.assertFalse(self.node._message_fresh(self.node.joint_state,
            self.node.joint_state_received, .3, self.platform.now))
        response = self.request()
        self.assertTrue(response.success)
        self.node._control()
        operation = self.node.module_command_publisher.messages[-1]
        self.assertFalse(operation.enable)
        self.assertEqual(operation.reset_id, 1)
        self.assertFalse(self.platform.states[-1][2])
        self.inputs(11.1, fault=0, ack=True)
        self.node._control()
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)
        self.assertTrue(self.node.fault_codes)
        self.assertTrue(self.fixture.publish_state().reset_in_progress)
        self.assertFalse(self.fixture.publish_state().motion_permitted)
        self.inputs(11.2, enabled=True, motion=True, fault=0, ack=True)
        self.node._control()
        self.assertIsNotNone(self.node.reset_started_at)
        self.assertFalse(self.node.motion_inputs_ready)
        self.inputs(11.45, enabled=True, motion=True, fault=0, ack=True)
        self.node._control()
        self.assertIsNone(self.node.reset_started_at)
        self.assertFalse(self.node.fault_codes)
        self.assertIsNone(self.node.manual_command)
        self.assertEqual(self.node.module_command_publisher.messages[-1].reset_id, 0)
        self.assertFalse(self.platform.states[-1][2])  # Still no rearmed manual command.
        for final in self.node.final_command_publisher.messages:
            self.assertTrue(all(m.traction_velocity == 0 and m.steering_profile_velocity == 0 for m in final.modules))

    def test_each_native_disabled_state_must_be_known_fresh_and_match_topology(self):
        changes = (
            lambda n: setattr(n.wheel_status.modules[0].steering, 'status_known', False),
            lambda n: setattr(n.wheel_status.modules[0].traction, 'device_state_known', False),
            lambda n: setattr(n.wheel_status.modules[0].steering, 'enabled', True),
            lambda n: setattr(n.wheel_status.modules[0].traction, 'status_stamp', harness.Time(10).to_msg()),
            lambda n: setattr(n.wheel_status.modules[0].traction, 'status_stamp', harness.Time(11.01).to_msg()),
            lambda n: setattr(n.wheel_status.modules[0].steering, 'name', 'wrong_joint'),
            lambda n: setattr(n.wheel_status.modules[0], 'startup_complete', False),
            lambda n: setattr(n.wheel_status.modules[0], 'initialized', False),
            lambda n: setattr(n.wheel_status.modules[0], 'startup_failed', True),
            lambda n: n.wheel_status.modules.pop(),
            lambda n: setattr(n.wheel_status.modules[0], 'name', n.wheel_status.modules[1].name),
        )
        for change in changes:
            with self.subTest(change=change):
                self.setUp()
                change(self.node)
                self.assertFalse(self.request().success)
                self.assertEqual(self.node.reset_id, 0)

    def test_fresh_movement_in_one_channel_cannot_hide_behind_missing_other_channel(self):
        for body in (False, True):
            with self.subTest(body=body):
                self.setUp()
                if body:
                    message = harness.TwistStamped()
                    message.header.stamp = self.platform.now.to_msg()
                    message.twist.linear.x = .03
                    self.node.measured_velocity = message
                    self.node.measured_velocity_received = self.platform.now
                else:
                    self.node.joint_state.header.stamp = self.platform.now.to_msg()
                    self.node.joint_state.velocity[0] = .11
                    self.node.joint_state_received = self.platform.now
                self.assertIn('movement', self.request().message)
                self.assertIsNone(self.node.reset_started_at)

    def test_external_fault_stop_disconnect_or_deflected_controls_block_acceptance(self):
        changes = (
            lambda n: setattr(n.battery_state, 'high_voltage_connected', False),
            lambda n: setattr(n.battery_state, 'faulted_batteries', ['battery1']),
            lambda n: setattr(n.battery_state, 'minimum_percentage', .1),
            lambda n: setattr(n.manual_state, 'connected', False),
            lambda n: setattr(n.manual_state, 'estop_pressed', True),
            lambda n: setattr(n.manual_state, 'estop_state_known', False),
            lambda n: setattr(n.manual_state, 'controls_neutral', False),
            lambda n: setattr(n.wheel_status.modules[0], 'sto_active', True),
        )
        for change in changes:
            with self.subTest(change=change):
                self.setUp()
                self.node.wheel_status.modules[0].sto_state_known = True
                change(self.node)
                self.assertFalse(self.request().success)

    def test_empty_duplicate_or_old_acknowledgements_never_enable_recovery(self):
        for bad_ack in ('empty', 'duplicate', 'old', 'missing'):
            with self.subTest(bad_ack=bad_ack):
                self.setUp()
                self.assertTrue(self.request().success)
                self.inputs(11.1, fault=0, ack=True)
                if bad_ack == 'empty':
                    self.node.wheel_status.modules = []
                elif bad_ack == 'duplicate':
                    self.node.wheel_status.modules[0].name = self.node.wheel_status.modules[1].name
                elif bad_ack == 'old':
                    self.node.wheel_status.header.stamp = harness.Time(11).to_msg()
                else:
                    self.node.wheel_status.modules[0].reset_id = 0
                self.node._control()
                self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
                self.assertIsNotNone(self.node.reset_started_at)

    def test_frozen_zero_samples_cannot_complete_post_reset_standstill(self):
        self.acknowledge()
        self.inputs(11.2, enabled=True, motion=True, fault=0, ack=True)
        self.node._control()
        self.inputs(11.45, enabled=True, motion=False, fault=0, ack=True)
        # Keep health valid to isolate the acquisition-time standstill proof.
        for health, received in self.node.component_health.values():
            health.monitoring_valid = True
        for module in self.node.wheel_status.modules:
            module.feedback_fresh = module.position_valid = True
        self.node._control()
        self.assertIsNotNone(self.node.reset_started_at)
        self.assertTrue(self.node.fault_codes)

    def test_interlock_loss_during_zero_enable_keeps_latch_and_requests_disable(self):
        for field, value in (('controls_neutral', False), ('estop_pressed', True), ('connected', False)):
            with self.subTest(field=field):
                self.setUp()
                self.acknowledge()
                setattr(self.node.manual_state, field, value)
                self.node._control()
                self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
                self.assertTrue(self.node.fault_codes)
                self.assertIsNotNone(self.node.reset_started_at)

    def test_missing_motion_data_times_out_and_cannot_leave_enable_active(self):
        self.acknowledge()
        self.inputs(16, enabled=True, fault=0, ack=True)
        self.node._control()
        self.assertIsNone(self.node.reset_started_at)
        self.assertIn('RESET_TIMEOUT', self.node.fault_codes)
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
        self.assertEqual(self.node.module_command_publisher.messages[-1].reset_id, 0)

    def test_movement_between_control_ticks_restarts_the_standstill_hold(self):
        for body in (False, True):
            with self.subTest(body=body):
                self.setUp()
                self.acknowledge()
                self.inputs(11.2, enabled=True, motion=True, fault=0, ack=True)
                self.node._control()
                self.inputs(11.4, enabled=True, motion=True, fault=0, ack=True)
                original = copy.deepcopy(self.node.measured_velocity if body else self.node.joint_state)
                moving = copy.deepcopy(original)
                callback = self.node._velocity_feedback_callback if body else self.node._joint_state_callback
                if body:
                    moving.twist.linear.x = .03
                else:
                    moving.velocity[0] = .11
                callback(moving)
                callback(original)
                self.node._control()
                self.inputs(11.45, enabled=True, motion=True, fault=0, ack=True)
                self.node._control()
                self.assertIsNotNone(self.node.reset_started_at)
                self.inputs(11.65, enabled=True, motion=True, fault=0, ack=True)
                self.node._control()
                self.assertIsNone(self.node.reset_started_at)

    def test_feedback_gap_restarts_the_standstill_hold(self):
        self.acknowledge()
        self.inputs(11.2, enabled=True, motion=True, fault=0, ack=True)
        self.node._control()
        self.inputs(11.6, enabled=True, motion=True, fault=0, ack=True)
        self.node._control()
        self.assertIsNotNone(self.node.reset_started_at)
        self.inputs(11.85, enabled=True, motion=True, fault=0, ack=True)
        self.node._control()
        self.assertIsNone(self.node.reset_started_at)

    def test_recovery_preserves_each_autonomous_mode_without_replaying_commands(self):
        for mode in (harness.MODES.AUTONOMOUS, harness.MODES.KINEMATICOFF):
            with self.subTest(mode=mode):
                self.setUp()
                self.node.selected_mode = mode
                self.acknowledge()
                for time in (11.2, 11.45):
                    self.inputs(time, enabled=True, motion=True, fault=0, ack=True)
                    self.node._control()
                self.assertIsNone(self.node.reset_started_at)
                self.assertFalse(self.node.fault_codes)
                self.assertEqual(self.node.selected_mode, mode)
                self.assertIsNone(self.node.autonomy_command)
                self.assertIsNone(self.node.autonomy_joint_command)
                self.assertFalse(self.platform.states[-1][2])

    def test_standstill_proof_finishing_at_deadline_does_not_clear_timeout(self):
        self.acknowledge()
        for time in (15.75, 16):
            self.inputs(time, enabled=True, motion=True, fault=0, ack=True)
            self.node._control()
        self.assertIn('RESET_TIMEOUT', self.node.fault_codes)
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)


if __name__ == '__main__':
    unittest.main()
