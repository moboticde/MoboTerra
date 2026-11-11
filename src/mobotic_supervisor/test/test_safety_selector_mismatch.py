"""Fresh RP2 Safety Enabled plus PLC override must stop every motion source."""
import copy
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_enable_gate as h
import test_interface_update as interfaces
import test_operator_scanner_override as overrides


class SafetySelectorMismatchTest(unittest.TestCase):
    def configure(self, count=4, mode=h.MODES.MANUAL):
        fixture = overrides.FourModuleOperatorScannerOverrideTest()
        fixture.setUp()
        self.n, self.f = fixture.n, fixture.f
        n = self.n
        names = ['front_right', 'rear_left'] if count == 2 else [
            'front_left', 'front_right', 'rear_left', 'rear_right']
        template = n.wheel_status.modules[0]
        n.wheel_status.modules = [copy.deepcopy(template) for _ in names]
        for name, module in zip(names, n.wheel_status.modules): module.name = name
        n.expected_module_names = names
        n.expected_steering_joint_names = [name + '_steering' for name in names]
        n.expected_traction_joint_names = [name + '_traction' for name in names]
        n.direct_steering_encoder_resolutions = [4096.] * count
        n.direct_traction_control_mode = 'velocity'
        n.joint_state.name = n.expected_steering_joint_names + n.expected_traction_joint_names
        n.joint_state.position = n.joint_state.velocity = [0.] * (2 * count)
        n.selected_mode = n.requested_mode = mode
        n.operator_selector.buttons[10] = 0
        n.safety_state.protective_stop_active = False
        n.safety_state.motion_permitted = True
        n.safety_state.safety_override_active = False
        n.safety_io_state.front_left_protective_field_clear = True
        n.safety_io_state.front_left_ossd_active = True
        n.scanner_states = {name: (SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg()),
            scanner_id=name, communication_ok=True, protective_state_known=True,
            warning_state_known=True, protective_field_clear=True, warning_field_clear=True), self.f.now)
            for name in ('front_left', 'rear_right')}
        del n._source_gate
        del n._publish_mode_state
        n._refresh_motion_readiness(self.f.now)
        self.assertTrue(n.motion_inputs_ready)
        return n

    def setUp(self):
        guard = patch.dict(h.namespace, VehicleModeState=interfaces.State)
        guard.start()
        self.addCleanup(guard.stop)
        self.configure()

    def plc(self, override):
        self.f.now = h.Time(self.f.now.nanoseconds / 1e9 + .01)
        msg = copy.deepcopy(self.n.safety_state)
        msg.header.stamp = self.f.now.to_msg()
        msg.safety_override_active = override
        self.n._safety_callback(msg)

    def velocity(self, stamp=None):
        msg = h.TwistStamped()
        msg.header.stamp = self.f.now.to_msg() if stamp is None else stamp
        msg.header.frame_id = self.n.output_frame_id
        msg.twist.linear.x = .2
        return msg

    def assert_stopped(self):
        n = self.n
        self.assertFalse(n._safety_gate(self.f.now)[0])
        self.assertIsNone(n.manual_command)
        self.assertIsNone(n.autonomy_command)
        self.assertIsNone(n.autonomy_joint_command)
        body = n.cmd_vel_publisher.messages[-1].twist
        self.assertEqual((body.linear.x, body.linear.y, body.angular.z), (0, 0, 0))
        final = n.final_command_publisher.messages[-1]
        self.assertEqual(len(final.modules), len(n.expected_module_names))
        self.assertTrue(all(m.traction_velocity == 0 and m.steering_profile_velocity == 0 for m in final.modules))
        state = n.mode_state_publisher.messages[-1]
        self.assertEqual(state.current_mode, h.MODES.SAFETY)
        self.assertTrue(state.safety_stop_active)
        self.assertFalse(state.error_active or state.motion_permitted)
        self.assertIn('SAFETY_SELECTOR_MISMATCH', state.reason_codes)
        self.assertIn('disagrees', state.status_message)
        self.assertFalse(n.fault_codes)

    def test_plc_callback_stops_every_mode_in_both_flexisoft_layouts(self):
        for count in (2, 4):
            for mode in (h.MODES.MANUAL, h.MODES.AUTONOMOUS, h.MODES.KINEMATICOFF):
                with self.subTest(count=count, mode=mode):
                    n = self.configure(count, mode)
                    n.manual_command = n.autonomy_command = self.velocity()
                    n.autonomy_joint_command = h.JointState()
                    n.last_output = [.2, .1, .3]
                    n.last_final = dict.fromkeys(n.expected_module_names, 1.)
                    self.plc(True)
                    self.assert_stopped()
                    n._control()
                    self.assertFalse(n.module_command_publisher.messages[-1].enable)
                    generation = n.authority_generation
                    self.plc(True)
                    self.assertEqual(n.authority_generation, generation)

    def test_selector_arriving_after_plc_also_stops_immediately(self):
        n = self.n
        n.operator_selector.buttons[10] = 1
        n.safety_state.safety_override_active = True
        n._refresh_motion_readiness(self.f.now)
        n.manual_command = self.velocity()
        self.f.now = h.Time(10.01)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=self.f.now.to_msg(), frame_id='base_link'), buttons=[0] * 11)
        n._operator_selector_callback(msg)
        self.assert_stopped()

    def test_operator_arriving_last_also_reports_and_stops_mismatch(self):
        n = self.n
        msg = copy.deepcopy(n.manual_state)
        n.manual_state = None
        n.safety_state.safety_override_active = True
        self.f.now = h.Time(10.01)
        msg.header.stamp = self.f.now.to_msg()
        n._manual_state_callback(msg)
        self.assert_stopped()

    def test_startup_b2_and_automatic_selection_cannot_override_disagreement(self):
        self.plc(True)
        self.n.started_at = self.f.now  # Startup grace cannot grant motion.
        self.assertFalse(self.n._enable_gate(self.f.now)[0])
        self.assertTrue(self.n._toggle_drives_service(None, SimpleNamespace()).success)  # Disable stays available.
        for module in self.n.wheel_status.modules:
            module.enabled = False  # Observe the completed disable before retrying enable.
        self.assertFalse(self.n._toggle_drives_service(None, SimpleNamespace()).success)  # Enable is blocked.
        for mode in (h.MODES.AUTONOMOUS, h.MODES.KINEMATICOFF):
            self.assertFalse(self.n._request_mode(mode)[0])

    def test_disagreement_stops_an_in_progress_mode_transition(self):
        self.assertTrue(self.n._request_mode(h.MODES.AUTONOMOUS)[0])
        self.n.last_output = [.2, 0, .1]
        self.plc(True)
        self.assert_stopped()
        self.assertTrue(self.n.transition_in_progress)
        self.assertEqual(self.n.selected_mode, h.MODES.MANUAL)
        self.n._control()
        self.assertFalse(self.n.module_command_publisher.messages[-1].enable)

    def test_matching_signals_recover_without_replaying_old_motion(self):
        for mode in (h.MODES.MANUAL, h.MODES.AUTONOMOUS):
            with self.subTest(mode=mode):
                n = self.configure(mode=mode)
                callback = n._manual_command_callback if mode == h.MODES.MANUAL else n._autonomy_command_callback
                old = self.velocity()
                callback(old)
                self.assertTrue(n._source_gate(self.f.now)[0])
                self.plc(True)
                callback(self.velocity())
                self.assertIsNone(n.manual_command if mode == h.MODES.MANUAL else n.autonomy_command)
                self.plc(False)
                self.assertFalse(n._source_gate(self.f.now)[0])
                callback(old)
                self.assertFalse(n._source_gate(self.f.now)[0])
                callback(self.velocity())
                self.assertFalse(n._source_gate(self.f.now)[0])
                self.assertTrue(n.operator_drives_disabled)
                self.assertFalse(n.fault_codes)
                self.assertFalse(n._system_conditions(self.f.now)[1])

    def test_unknown_inputs_do_not_create_a_false_disagreement(self):
        for missing in ('selector', 'operator', 'plc', 'stale_selector', 'stale_operator'):
            with self.subTest(missing=missing):
                n = self.configure()
                n.safety_state.safety_override_active = True
                if missing == 'selector': n.operator_selector = None
                elif missing == 'operator': n.manual_state = None
                elif missing == 'plc': n.safety_state = None
                elif missing == 'stale_selector': n.operator_selector.header.stamp = h.Time(9).to_msg()
                else: n.manual_state.header.stamp = h.Time(9).to_msg()
                self.assertFalse(n._safety_selector_mismatch(self.f.now))
                self.assertFalse(n._safety_gate(self.f.now)[0])


if __name__ == '__main__':
    unittest.main()
