"""KINEMATICOFF current targets, mode acknowledgement and stop boundaries."""
import copy
import math
import unittest

import test_enable_gate as h
import test_interface_update as interface


class CurrentDirectTest(unittest.TestCase):
    def setUp(self):
        self.fixture = interface.InterfaceUpdateTest()
        self.fixture.setUp()
        self.node = n = self.fixture.node
        del n._source_gate
        n.direct_traction_control_mode = 'current'
        n.kinematicoff_current_supported = True
        n.max_direct_traction_current = 4.0
        n.selected_mode = h.MODES.KINEMATICOFF
        n.command_accept_after = h.Time(9)
        for module in n.wheel_status.modules:
            module.control_mode = 3
        self.now = self.fixture.fixture.now

    def command(self):
        msg = self.fixture.joints()
        msg.velocity[2:] = [0.0, 0.0]
        msg.effort = [0.0, 0.0, 1.25, -2.0]
        return msg

    def test_current_command_requires_correct_arrays_and_current_limits(self):
        msg = self.command()
        self.assertTrue(self.node._joint_command_valid(msg))
        for mutate in (
                lambda m: m.effort.clear(),
                lambda m: m.effort.pop(),
                lambda m: m.effort.__setitem__(0, .1),
                lambda m: m.effort.__setitem__(2, 4.01),
                lambda m: m.effort.__setitem__(3, -4.01),
                lambda m: m.effort.__setitem__(2, math.nan),
                lambda m: m.velocity.__setitem__(2, .1)):
            candidate = copy.deepcopy(msg)
            mutate(candidate)
            self.assertFalse(self.node._joint_command_valid(candidate))

    def test_final_command_has_amperes_and_explicitly_zero_velocity(self):
        msg = self.command()
        self.node._publish_direct(msg, self.now)
        final = self.node.final_command_publisher.messages[-1]
        self.assertEqual(final.source_stamp, msg.header.stamp)
        self.assertEqual([m.traction_current for m in final.modules], [1.25, -2.0])
        self.assertTrue(all(m.traction_current_control for m in final.modules))
        self.assertTrue(all(m.traction_velocity == 0 for m in final.modules))

    def test_stop_clears_every_current_and_holds_measured_steering(self):
        self.node._publish_direct(self.command(), self.now)
        self.node._publish_direct_stop(self.now)
        final = self.node.final_command_publisher.messages[-1]
        self.assertTrue(all(m.traction_current_control for m in final.modules))
        self.assertTrue(all(m.traction_current == 0 and m.traction_velocity == 0 for m in final.modules))
        self.assertTrue(all(m.steering_profile_velocity == 0 for m in final.modules))

    def test_missing_current_mode_acknowledgement_inhibits_motion(self):
        self.node.wheel_status.modules[0].control_mode = 2
        self.assertFalse(self.node._motion_hardware_gate(self.now)[0])

    def test_warning_inhibits_current_instead_of_scaling_amperes_as_velocity(self):
        self.node.scanner_states['front_left'][0].warning_field_clear = False
        self.assertFalse(self.node._motion_hardware_gate(self.now)[0])
        before = len(self.node.final_command_publisher.messages)
        self.node._publish_direct(self.command(), self.now)
        self.assertEqual(len(self.node.final_command_publisher.messages), before)
        self.node._publish_direct_stop(self.now)
        self.assertTrue(all(m.traction_current == 0 for m in self.node.final_command_publisher.messages[-1].modules))

    def test_unverified_controller_family_rejects_current_mode_selection(self):
        self.node.kinematicoff_current_supported = False
        self.assertFalse(self.node._request_mode(h.MODES.KINEMATICOFF)[0])

    def test_transition_waits_for_standstill_then_controller_mode_acknowledgement(self):
        n = self.node
        n.selected_mode = h.MODES.MANUAL
        n.wheel_status.modules[0].control_mode = 2
        n.wheel_status.modules[1].control_mode = 2
        n._request_mode(h.MODES.KINEMATICOFF)
        self.assertFalse(n.traction_mode_change_ready)
        n._control()
        self.assertFalse(n.module_command_publisher.messages[-1].reconfigure_traction_mode)
        self.fixture.fixture.now = h.Time(self.now.nanoseconds / 1e9 + .25)
        self.fixture.fixture.feedback()
        # The common fixture refreshes state; install the real mode evidence.
        for module in n.wheel_status.modules:
            module.control_mode = 2
        n._control()
        self.assertTrue(n.traction_mode_change_ready)
        self.assertTrue(n.transition_in_progress)
        self.assertEqual(n.selected_mode, h.MODES.MANUAL)
        n._control()
        op = n.module_command_publisher.messages[-1]
        self.assertTrue(op.reconfigure_traction_mode and op.traction_current_control)
        for module in n.wheel_status.modules:
            module.control_mode = 3
        n._control()
        self.assertFalse(n.transition_in_progress)
        self.assertEqual(n.selected_mode, h.MODES.KINEMATICOFF)
        self.assertIsNone(n.autonomy_joint_command)


if __name__ == '__main__':
    unittest.main()
