"""Four powered modules: full status, standstill, direct commands and stop output."""

from types import SimpleNamespace
import unittest

import test_enable_gate as harness
import test_mode_transition as transition


MODULES = ('front_left', 'front_right', 'rear_left', 'rear_right')


class FourModuleSupervisorTest(unittest.TestCase):
    module_names = MODULES

    def setUp(self):
        transition.ModeTransitionTest.setUp(self)
        self.node.expected_module_names = list(self.module_names)
        self.node.expected_steering_joint_names = [name + '_steering' for name in self.module_names]
        self.node.direct_steering_encoder_resolutions = [4096.0] * len(self.module_names)
        self.node.expected_traction_joint_names = [name + '_traction' for name in self.module_names]
        self.node.direct_traction_control_mode = 'velocity'
        self.node.transition_started_at = self.now
        self.feedback()

    def feedback(self, *, vx=0.0, vy=0.0, wz=0.0, joint_velocity=0.0):
        node = self.node
        node.safety_received = node.safety_io_received = node.battery_received = self.now
        for state in (node.safety_state, node.safety_io_state, node.battery_state):
            state.header.stamp = self.now.to_msg()
        node.wheel_status_received = self.now
        node.wheel_status = SimpleNamespace(header=SimpleNamespace(stamp=self.now.to_msg()), modules=[
            SimpleNamespace(name=name, enabled=True, error_code=0, feedback_fresh=True)
            for name in node.expected_module_names
        ])
        velocity = harness.TwistStamped()
        velocity.header.stamp = self.now.to_msg()
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.angular.z = vx, vy, wz
        node._velocity_feedback_callback(velocity)
        joints = harness.JointState()
        joints.header.stamp = self.now.to_msg()
        joints.name = node.expected_steering_joint_names + node.expected_traction_joint_names
        joints.position = [0.1 * index for index in range(len(joints.name))]
        joints.velocity = [joint_velocity] * len(joints.name)
        node._joint_state_callback(joints)

    def test_every_module_required_for_motion(self):
        self.assertTrue(self.node._base_gate(self.now)[0])
        for name in self.module_names:
            with self.subTest(missing=name):
                self.feedback()
                self.node.wheel_status.modules = [module for module in self.node.wheel_status.modules
                                                  if module.name != name]
                self.assertFalse(self.node._base_gate(self.now)[0])

    def test_fault_disabled_or_stale_feedback_on_any_module_blocks_motion(self):
        for index, name in enumerate(self.module_names):
            for field, value in (('feedback_fresh', False), ('enabled', False), ('error_code', 1)):
                with self.subTest(module=name, field=field):
                    self.feedback()
                    setattr(self.node.wheel_status.modules[index], field, value)
                    self.assertFalse(self.node._base_gate(self.now)[0])

    def test_standstill_requires_all_configured_stationary_joints(self):
        self.assertTrue(self.node._standstill_gate(self.now)[0])
        for index in range(2 * len(self.module_names)):
            with self.subTest(moving_index=index):
                self.feedback()
                self.node.joint_state.velocity[index] = 0.2
                self.node._joint_state_callback(self.node.joint_state)
                self.assertFalse(self.node._standstill_gate(self.now)[0])
        self.feedback()
        self.node.joint_state.name.pop()
        self.node.joint_state.position.pop()
        self.node.joint_state.velocity.pop()
        self.node._joint_state_callback(self.node.joint_state)
        self.assertFalse(self.node._standstill_gate(self.now)[0])

    def test_direct_command_and_stop_cover_all_configured_joints(self):
        node = self.node
        node.motion_inputs_ready = True
        command = harness.JointState()
        command.header.stamp = self.now.to_msg()
        command.name = node.expected_steering_joint_names + node.expected_traction_joint_names
        count = len(command.name)
        command.position = [0.0] * count
        command.velocity = [0.0] * count
        self.assertTrue(node._joint_command_valid(command))
        command.name.pop()
        command.position.pop()
        command.velocity.pop()
        self.assertFalse(node._joint_command_valid(command))
        node._publish_direct_stop(self.now)
        stop = node.direct_command_publisher.messages[-1]
        self.assertEqual(stop.name, node.expected_steering_joint_names + node.expected_traction_joint_names)
        self.assertEqual(stop.velocity, [0.0] * count)
        self.assertEqual(stop.position[:len(self.module_names)], node.joint_state.position[:len(self.module_names)])


class TwoModuleSupervisorTest(FourModuleSupervisorTest):
    module_names = ('front_right', 'rear_left')


if __name__ == '__main__':
    unittest.main()
