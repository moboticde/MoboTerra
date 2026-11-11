"""Regression tests for automatic modes, single final authority and explicit reset."""
from types import SimpleNamespace
from unittest.mock import patch
import unittest
import test_enable_gate as harness
import test_mode_transition as transitions


class State:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)


for _name, _value in vars(harness.MODES).items():
    setattr(State, _name, _value)


class InterfaceUpdateTest(unittest.TestCase):
    def setUp(self):
        self.fixture = transitions.ModeTransitionTest()
        self.fixture.setUp()
        self.node = self.fixture.node
        self.node.supervisor_instance = 'test-supervisor'
        self.node.authority_generation = 7
        self.node.velocity_ledger = {}
        self.node.last_final = {}
        self.node.final_command_publisher = harness.Publisher()
        del self.node._emit_final
        self.node.mode_state_publisher = harness.Publisher()
        self.node.requested_mode = self.node.selected_mode
        self.node._source_gate = lambda now, **kwargs: (True, 'ready')
        self.node.reset_timeout = 5.0

    def publish_state(self, permitted=True):
        with patch.dict(harness.namespace, VehicleModeState=State):
            harness.Supervisor._publish_mode_state(self.node, self.fixture.now, permitted, 'test')
        return self.node.mode_state_publisher.messages[-1]

    def joints(self):
        msg = harness.JointState()
        msg.header.stamp = self.fixture.now.to_msg()
        msg.name = ['front_steering', 'rear_steering', 'front_traction', 'rear_traction']
        msg.position = [0.2, -0.1, 99.0, -99.0]
        msg.velocity = [1.0, 1.0, 2.0, -3.0]
        return msg

    def test_low_soc_warning_preserves_startup_and_motion_and_clears_on_recovery(self):
        for soc in (0.0, .1, .199, .2, .8):
            with self.subTest(soc=soc):
                self.node.battery_state.minimum_percentage = soc
                state = self.publish_state()
                self.assertTrue(state.ready_to_drive)
                self.assertTrue(state.motion_permitted)
                self.assertFalse(state.error_active)
                self.assertFalse(self.node.fault_codes)
                self.assertEqual('LOW_BATTERY_SOC' in state.reason_codes, soc < .2)
                self.assertEqual('advisory only' in state.status_message, soc < .2)
        self.node.battery_received = harness.Time(9)
        self.assertFalse(self.node._enable_gate(self.fixture.now)[0])
        self.assertFalse(self.publish_state().ready_to_drive)
        self.assertNotIn('LOW_BATTERY_SOC', self.publish_state().reason_codes)

    def test_safety_and_error_are_automatic_and_overlap_is_visible(self):
        self.node.safety_state.protective_stop_active = True
        self.node.safety_io_state.scanner_state_known = True
        state = self.publish_state()
        self.assertEqual(state.current_mode, State.SAFETY)
        self.assertEqual(state.selected_mode, State.MANUAL)
        self.assertTrue(state.safety_stop_active)
        self.assertFalse(state.motion_permitted)
        self.node.fault_codes.add('DRIVE_FAULT:front')
        state = self.publish_state()
        self.assertEqual(state.current_mode, State.ERROR)
        self.assertTrue(state.error_active and state.safety_stop_active)
        for mode in (State.ERROR, State.SAFETY, State.UNKNOWN):
            self.assertFalse(self.node._request_mode(mode)[0])

    def test_unconfirmed_safety_timeout_is_not_a_confirmed_stop(self):
        self.node.safety_state.communication_ok = False
        self.node.safety_state.emergency_stop_active = True
        self.node.safety_io_state.emergency_stop_state_known = False
        self.node.started_at = harness.Time(0)
        state = self.publish_state()
        self.assertEqual(state.current_mode, State.ERROR)
        self.assertFalse(state.safety_stop_active)

    def test_unknown_initialization_and_empty_status_are_not_ready(self):
        self.node.wheel_status.modules = []
        state = self.publish_state()
        self.assertEqual(state.current_mode, State.UNKNOWN)
        self.assertFalse(state.ready_to_drive or state.readiness_known or state.motion_permitted)

    def test_ready_to_drive_includes_enable_external_readiness_and_health(self):
        self.assertTrue(self.publish_state().ready_to_drive)
        self.node.wheel_status.modules[0].enabled = False
        self.assertFalse(self.publish_state().ready_to_drive)
        self.node.wheel_status.modules[0].enabled = True
        self.node.battery_state.high_voltage_connected = False
        self.assertFalse(self.publish_state().ready_to_drive)
        self.node.battery_state.high_voltage_connected = True
        self.node.required_health_sources = ['odometry']
        self.node.component_health = {}
        self.node.health_timeout = .3
        self.assertFalse(self.publish_state().ready_to_drive)

    def test_final_array_preserves_source_stamp_and_accepts_candidate_once(self):
        msg = self.joints()
        source_stamp = harness.Time(9.9).to_msg()
        self.node.velocity_ledger[msg.header.stamp] = (7, harness.WheelModuleCommandArray.MOTION, source_stamp)
        self.node._kinematics_candidate_callback(msg)
        output = self.node.final_command_publisher.messages[-1]
        self.assertEqual(output.source_stamp, source_stamp)
        self.assertEqual(output.supervisor_instance, 'test-supervisor')
        self.assertEqual(output.authority_generation, 7)
        self.assertEqual([(m.name, m.steering_position, m.traction_velocity) for m in output.modules],
                         [('front', 0.2, 2.0), ('rear', -0.1, -3.0)])
        self.node._kinematics_candidate_callback(msg)
        self.assertEqual(len(self.node.final_command_publisher.messages), 1)

    def test_fresh_neutral_cancels_uncapped_request_without_supervisor_ramp(self):
        self.node.manual_state = SimpleNamespace(controls_neutral=True, connected=True,
            header=SimpleNamespace(stamp=self.fixture.now.to_msg()))
        self.node.manual_state.header.stamp = self.fixture.now.to_msg()
        self.node.manual_state_received = self.fixture.now
        self.node.last_output = [.3, .2, .1]
        self.node.last_control_time = harness.Time(9.99)
        self.node.max_acceleration_linear = self.node.max_acceleration_angular = .5
        source = harness.TwistStamped()
        source.header.stamp = harness.Time(9.95).to_msg()
        source.twist.linear.x = 2.
        self.node._publish_velocity(source, self.fixture.now)
        out = self.node.cmd_vel_publisher.messages[-1].twist
        self.assertEqual([out.linear.x, out.linear.y, out.angular.z], [0., 0., 0.])
        self.assertEqual(self.node.last_output, [0., 0., 0.])

    def test_delayed_candidate_cannot_restart_traction_or_old_steering_after_neutral(self):
        self.node.manual_state = SimpleNamespace(controls_neutral=True, connected=True,
            header=SimpleNamespace(stamp=self.fixture.now.to_msg()))
        self.node.manual_state.header.stamp = self.fixture.now.to_msg()
        self.node.manual_state_received = self.fixture.now
        self.node.joint_state = self.joints()
        self.node.joint_state_received = self.fixture.now
        msg = self.joints()
        msg.position[:2] = [7., -9.]
        self.node.last_final = {'front': .3, 'rear': -.3}
        self.node.velocity_ledger[msg.header.stamp] = (
            7, harness.WheelModuleCommandArray.MOTION, harness.Time(9.95).to_msg())
        self.node._kinematics_candidate_callback(msg)
        output = self.node.final_command_publisher.messages[-1]
        self.assertEqual(output.purpose, harness.WheelModuleCommandArray.HOLD)
        self.assertEqual([m.steering_position for m in output.modules], [.2, -.1])
        self.assertEqual([m.steering_profile_velocity for m in output.modules], [0., 0.])
        self.assertEqual([m.traction_velocity for m in output.modules], [0., 0.])

    def test_neutral_proof_does_not_override_newer_input_or_other_modes(self):
        self.node.manual_state = SimpleNamespace(controls_neutral=True, connected=True,
            header=SimpleNamespace(stamp=self.fixture.now.to_msg()))
        self.node.manual_state.header.stamp = harness.Time(9.9).to_msg()
        self.node.manual_state_received = self.fixture.now
        self.assertFalse(self.node._manual_stop_requested(harness.Time(9.95).to_msg(), self.fixture.now))
        for mode in (harness.MODES.AUTONOMOUS, harness.MODES.KINEMATICOFF):
            self.node.selected_mode = mode
            self.assertFalse(self.node._manual_stop_requested(harness.Time(9.8).to_msg(), self.fixture.now))
        self.node.selected_mode = harness.MODES.MANUAL
        self.node.manual_state.header.stamp = harness.Time(9.).to_msg()
        self.assertFalse(self.node._manual_stop_requested(harness.Time(9.8).to_msg(), self.fixture.now))

    def test_explicit_manual_zero_cancels_ramp_and_late_candidates_without_all_sticks_neutral(self):
        self.node.manual_state = SimpleNamespace(controls_neutral=False, connected=True,
            header=SimpleNamespace(stamp=self.fixture.now.to_msg()))
        source = harness.TwistStamped()
        source.header.stamp = self.fixture.now.to_msg()
        self.node.manual_command = source
        self.node.manual_command_received = self.fixture.now
        self.node.last_output = [.3, .2, .1]
        self.node.last_control_time = harness.Time(9.99)
        self.node._publish_velocity(source, self.fixture.now)
        out = self.node.cmd_vel_publisher.messages[-1].twist
        self.assertEqual([out.linear.x, out.linear.y, out.angular.z], [0., 0., 0.])
        self.node.joint_state = self.joints()
        self.node.joint_state_received = self.fixture.now
        msg = self.joints()
        self.node.velocity_ledger[msg.header.stamp] = (
            7, harness.WheelModuleCommandArray.MOTION, harness.Time(9.95).to_msg())
        self.node._kinematics_candidate_callback(msg)
        output = self.node.final_command_publisher.messages[-1]
        self.assertEqual(output.purpose, harness.WheelModuleCommandArray.HOLD)
        self.assertEqual([m.traction_velocity for m in output.modules], [0., 0.])
        # A later real command can start moving again; an older zero cannot veto it.
        self.assertFalse(self.node._manual_stop_requested(harness.Time(10.001).to_msg(), harness.Time(10.01)))

    def configure_neutral_home(self):
        from mobotic_kinematics.manual_steering import ManualSteering
        self.node.manual_steering_policy = ManualSteering({
            'wheels_x': [.845, -.845], 'wheels_y': [.495, -.495],
            'steering_joint_names': self.node.expected_steering_joint_names,
            'traction_joint_names': self.node.expected_traction_joint_names,
            'wheel_mount_angles': [0., 3.14152], 'max_steering_velocity': .3,
            'steering_angle_limits_enabled': False, 'steering_return_home_on_neutral': True,
            'steering_neutral_home_velocity': 1.2})
        self.node.manual_state = SimpleNamespace(controls_neutral=True, connected=True, command_active=True,
            header=SimpleNamespace(stamp=self.fixture.now.to_msg()))
        self.node.manual_state_received = self.fixture.now
        self.node.manual_command = harness.TwistStamped()
        self.node.manual_command.header.stamp = self.fixture.now.to_msg()
        self.node.manual_command_received = self.fixture.now
        self.node.joint_state = self.joints()
        self.node.joint_state.velocity = [0.] * 4
        self.node.joint_state_received = self.fixture.now

    def neutral_candidate(self, purpose=None):
        msg = self.joints()
        self.node.velocity_ledger[msg.header.stamp] = (7,
            harness.WheelModuleCommandArray.MOTION if purpose is None else purpose,
            self.node.manual_command.header.stamp, True, True, (0.,0.,0.))
        self.node._kinematics_candidate_callback(msg)
        return self.node.final_command_publisher.messages[-1]

    def test_neutral_returns_home_only_after_new_traction_standstill_feedback(self):
        self.configure_neutral_home()
        first = self.neutral_candidate()
        self.assertEqual([m.traction_velocity for m in first.modules], [0., 0.])
        self.assertEqual([m.steering_profile_velocity for m in first.modules], [0., 0.])
        self.fixture.now = harness.Time(10.1)
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: self.fixture.now)
        self.node.joint_state.header.stamp = self.fixture.now.to_msg()
        self.node.joint_state_received = self.fixture.now
        homing = self.neutral_candidate()
        self.assertEqual(homing.purpose, harness.WheelModuleCommandArray.MOTION)
        self.assertEqual([m.steering_position for m in homing.modules], [0., 0.])
        self.assertEqual([m.steering_profile_velocity for m in homing.modules], [1.2, 1.2])
        self.assertEqual([m.traction_velocity for m in homing.modules], [0., 0.])

    def test_neutral_never_homes_for_inhibited_stale_or_non_neutral_sources(self):
        for cause in ('hold', 'source_gate', 'stale', 'stick_held'):
            with self.subTest(cause=cause):
                self.setUp()
                self.configure_neutral_home()
                if cause == 'source_gate':
                    self.node._source_gate = lambda now, **kwargs: (False, 'safety stop')
                elif cause == 'stale':
                    self.node.manual_state.header.stamp = harness.Time(9.).to_msg()
                elif cause == 'stick_held':
                    self.node.manual_state.controls_neutral = False
                if cause == 'source_gate':
                    msg = self.joints()
                    self.node.velocity_ledger[msg.header.stamp] = (7, harness.WheelModuleCommandArray.MOTION,
                        self.node.manual_command.header.stamp, True, True, (0.,0.,0.))
                    self.node._kinematics_candidate_callback(msg)
                    self.assertFalse(self.node.final_command_publisher.messages)
                else:
                    result = self.neutral_candidate(harness.WheelModuleCommandArray.HOLD if cause == 'hold' else None)
                    self.assertEqual(result.purpose, harness.WheelModuleCommandArray.HOLD)
                    self.assertEqual([m.steering_profile_velocity for m in result.modules], [0., 0.])
                self.assertIsNone(self.node.manual_steering_policy.neutral_home_targets)

    def test_speed_limit_reason_identifies_each_warning_and_clears(self):
        self.node.scanner_states['front_left'][0].warning_field_clear=False
        state=self.publish_state()
        self.assertIn('SCANNER_WARNING:front_left',state.reason_codes)
        self.assertIn('speed limited to 0.3 m/s',state.status_message)
        self.assertFalse(state.error_active)
        self.node.scanner_states['front_left'][0].warning_field_clear=True
        self.node.safety_state.safety_override_active=True
        state=self.publish_state()
        self.assertIn('SAFETY_DISABLED_FLEXISOFT',state.reason_codes)
        self.assertNotIn('SCANNER_WARNING:front_left',state.reason_codes)
        self.node.safety_state.safety_override_active=False
        state=self.publish_state()
        self.assertFalse(self.node._warning_active())
        self.assertNotIn('speed limited',state.status_message)
        self.assertFalse(any('WARNING' in code or 'DISABLED' in code for code in state.reason_codes))

    def test_wrong_generation_direct_mode_and_current_effort_drop_candidates(self):
        for mode, generation, effort in [(State.MANUAL, 6, []), (State.KINEMATICOFF, 7, []),
                                          (State.MANUAL, 7, [1.0] * 4)]:
            self.node.selected_mode = mode
            msg = self.joints(); msg.effort = effort
            self.node.velocity_ledger[msg.header.stamp] = (generation, harness.WheelModuleCommandArray.MOTION, msg.header.stamp)
            self.node._kinematics_candidate_callback(msg)
        self.assertEqual(self.node.final_command_publisher.messages, [])

    def test_stop_holds_actual_steering_and_never_increases_or_reverses(self):
        self.node.last_final = {'front': 1.0, 'rear': 2.0}
        self.node._emit_final(self.joints(), self.fixture.now.to_msg(), self.fixture.now,
                              harness.WheelModuleCommandArray.CONTROLLED_STOP)
        output = self.node.final_command_publisher.messages[-1]
        self.assertEqual([m.traction_velocity for m in output.modules], [1.0, 0.0])
        self.assertEqual([m.steering_position for m in output.modules], [0.7, -0.4])
        self.assertEqual([m.steering_profile_velocity for m in output.modules], [0.0, 0.0])

    def test_future_external_command_and_future_reset_proof_rejected(self):
        self.node.motion_inputs_ready = True
        self.assertFalse(self.node._command_stamp_valid(harness.Time(10.01).to_msg(), 0.25))
        self.node.joint_state.header.stamp = harness.Time(10.01).to_msg()
        self.node._observe_reset(self.fixture.now, [])
        self.assertIsNone(self.node.standstill_since)

    def test_reset_requires_new_id_ack_and_clears_cached_motion(self):
        self.node.fault_codes = {'DRIVE_FAULT:front'}
        self.node.standstill_since = harness.Time(9.7)
        response = self.node._reset_errors_service(None, SimpleNamespace())
        self.assertTrue(response.success)
        self.assertEqual(self.node.reset_id, 1)
        self.assertIn('not yet verified', response.message)
        self.assertFalse(self.node._reset_errors_service(None, SimpleNamespace()).success)
        self.fixture.now = harness.Time(10.1)
        self.fixture.feedback()
        for module in self.node.wheel_status.modules:
            module.reset_id = 0; module.reset_acknowledged = True
        self.node._observe_reset(self.fixture.now, [])
        self.assertTrue(self.node.fault_codes)
        for module in self.node.wheel_status.modules: module.reset_id = 1
        self.node._observe_reset(self.fixture.now, [])
        self.assertTrue(self.node.fault_codes)  # A single zero sample is insufficient.
        self.fixture.now = harness.Time(10.35)
        self.fixture.feedback()
        for module in self.node.wheel_status.modules:
            module.reset_id, module.reset_acknowledged = 1, True
        self.node._observe_reset(self.fixture.now, [])
        self.assertEqual(self.node.fault_codes, set())
        self.assertIsNone(self.node.reset_started_at)
        self.assertIsNone(self.node.autonomy_command)

    def test_reset_timeout_remains_latched(self):
        self.node.reset_started_at = harness.Time(4.0)
        for module in self.node.wheel_status.modules:
            module.reset_id = 0; module.reset_acknowledged = False
        self.node._observe_reset(self.fixture.now, [])
        self.assertIn('RESET_TIMEOUT', self.node.fault_codes)

    def test_successful_b2_transaction_publishes_enable_again(self):
        self.node.fault_codes = {'DRIVE_FAULT:front'}
        self.node.standstill_since = harness.Time(9.7)
        self.assertTrue(self.node._reset_errors_service(None, SimpleNamespace()).success)
        for module in self.node.wheel_status.modules:
            module.reset_id, module.reset_acknowledged = 0, False
        self.node._control()
        self.assertFalse(self.node.module_command_publisher.messages[-1].enable)
        self.assertEqual(self.node.module_command_publisher.messages[-1].reset_id, 1)
        self.fixture.now = harness.Time(10.1)
        self.fixture.feedback()
        for module in self.node.wheel_status.modules:
            module.reset_id, module.reset_acknowledged = 1, True
        self.node._control()
        self.assertTrue(self.node.module_command_publisher.messages[-1].enable)
        self.assertEqual(self.node.module_command_publisher.messages[-1].reset_id, 1)
        self.assertTrue(self.node.fault_codes)
        self.fixture.now = harness.Time(10.35)
        self.fixture.feedback()
        for module in self.node.wheel_status.modules:
            module.reset_id, module.reset_acknowledged = 1, True
        self.node._control()
        self.assertEqual(self.node.module_command_publisher.messages[-1].reset_id, 0)
        self.assertFalse(self.node.fault_codes)
        self.assertIsNone(self.node.manual_command)

    def test_reset_rejects_expired_external_readiness(self):
        self.node.fault_codes = {'DRIVE_FAULT:front'}
        self.node.standstill_since = harness.Time(9.7)
        self.fixture.now = harness.Time(10.4)
        response = self.node._reset_errors_service(None, SimpleNamespace())
        self.assertFalse(response.success)
        self.assertIn('scanner field data unavailable', response.message)
        self.assertIsNone(self.node.reset_started_at)
        self.assertEqual(self.node.reset_id, 0)

    def test_host_error_blocks_enable_during_startup_and_remains_latched_after_recovery(self):
        now = self.fixture.now
        self.node.required_health_sources = ['host']
        self.node.health_timeout = 0.3
        self.node.started_at = now  # Host fault must not be hidden by startup grace.
        record = SimpleNamespace(header=SimpleNamespace(stamp=now.to_msg()),
                                 monitoring_valid=True, fault_active=True)
        self.node.component_health = {'host': (record, now)}
        self.assertFalse(self.node._enable_gate(now)[0])
        with patch.dict(harness.namespace, VehicleModeState=State):
            self.node._control()
        self.assertIn('COMPONENT_HEALTH_LOST:host', self.node.fault_codes)
        self.assertEqual(self.publish_state().current_mode, State.ERROR)
        record.fault_active = False
        with patch.dict(harness.namespace, VehicleModeState=State):
            self.node._control()
        self.assertEqual(self.publish_state().current_mode, State.ERROR)
        self.assertFalse(self.node._enable_gate(now)[0])
