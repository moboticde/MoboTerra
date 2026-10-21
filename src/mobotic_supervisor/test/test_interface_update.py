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
        self.node._source_gate = lambda now: (True, 'ready')
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
        self.assertEqual(self.node.module_command_publisher.messages[-1].reset_id, 0)
        self.assertFalse(self.node.fault_codes)
        self.assertIsNone(self.node.manual_command)

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
