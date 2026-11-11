"""Original-model adapter tests: geometry conversion, limits and launch wiring."""
import math
import hashlib
import json
import os
import tempfile
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobotic_kinematics.romb.coordinates import RombContract
from mobotic_kinematics.launching import romb_kinematics_nodes, kinematics_nodes
from mobotic_kinematics.romb.runtime import runtime_command


class RombContractTest(unittest.TestCase):
    def setUp(self):
        self.modules = ['front_left', 'front_right', 'rear_left', 'rear_right']
        self.steering = [m + '_steering' for m in self.modules]
        self.traction = [m + '_traction' for m in self.modules]
        self.names = self.steering + self.traction
        self.mounts = [0.0, 3.14152, 0.0, 3.14152]
        self.contract = RombContract(self.modules, self.steering, self.traction,
                                         self.mounts, .3, .3)

    def test_feedback_translates_mounting_angles_without_changing_mps(self):
        positions = [.1] * 4 + [5.0] * 4
        output = self.contract.feedback_positions(self.names, positions, [0.0] * 8)
        self.assertEqual(output[:4], [.1 + m for m in self.mounts])
        self.assertEqual(output[4:], positions[4:])
        self.assertEqual(positions[:4], [.1] * 4)

    def test_both_crab_directions_have_mirrored_controller_targets(self):
        for direction in (-1, 1):
            self.contract.reset()
            angles, speeds, profiles = self.contract.commands(self.names,
                [direction * math.pi / 2] * 4 + [0.0] * 4,
                [0.0] * 4 + [.1] * 4, self.names, [0.0] * 8)
            for i, (angle, speed) in enumerate(zip(angles, speeds)):
                self.assertAlmostEqual(speed * math.cos(angle + self.mounts[i]), 0.0)
                self.assertAlmostEqual(speed * math.sin(angle + self.mounts[i]), direction * .1)
            self.assertEqual(angles[0], angles[2])
            self.assertEqual(angles[1], angles[3])
            self.assertLess(abs(angles[0] + angles[1]), 1e-4)
            self.assertEqual(profiles, [.3] * 4)

    def test_ackermann_wheel_vectors_and_speed_ratios_are_preserved(self):
        xs, ys = [.845, .845, -.845, -.845], [.495, -.495, .495, -.495]
        vectors = [(.5 - .2 * y, .2 * x) for x, y in zip(xs, ys)]
        directions = [math.atan2(vy, vx) for vx, vy in vectors]
        requested = [math.hypot(vx, vy) for vx, vy in vectors]
        angles, speeds, _ = self.contract.commands(self.names, directions + [0.0] * 4,
            [0.0] * 4 + requested, self.names, [0.0] * 8)
        ratio = .3 / max(requested)
        for i, (angle, speed) in enumerate(zip(angles, speeds)):
            self.assertAlmostEqual(speed * math.cos(angle + self.mounts[i]), vectors[i][0] * ratio)
            self.assertAlmostEqual(speed * math.sin(angle + self.mounts[i]), vectors[i][1] * ratio)

    def test_stop_holds_previous_target_and_clears_profile_and_contact_speed(self):
        angles, _, _ = self.contract.commands(self.names, [.5] * 4 + [0.0] * 4,
            [0.0] * 4 + [.1] * 4, self.names, [0.0] * 8)
        held, speeds, profiles = self.contract.commands(self.names, [0.0] * 8,
            [0.0] * 8, self.names, [0.0] * 8)
        self.assertEqual(held, angles)
        self.assertEqual(speeds + profiles, [0.0] * 8)

    def test_existing_controller_turn_is_not_rewound_by_wrapped_romb_angle(self):
        measured = [2 * math.pi + .02] * 4 + [0.0] * 4
        angles, _, _ = self.contract.commands(self.names, [0.0] * 8,
            [0.0] * 4 + [.1] * 4, self.names, measured)
        self.assertTrue(all(abs(a - b) < .021 for a, b in zip(angles, measured[:4])))

    def test_malformed_samples_are_rejected_before_adapting_any_target(self):
        for positions, speeds, names in [([0.0] * 7, [0.0] * 8, self.names),
            ([float('nan')] + [0.0] * 7, [0.0] * 8, self.names),
            ([0.0] * 8, [float('inf')] + [0.0] * 7, self.names),
            ([0.0] * 8, [0.0] * 8, [self.names[0]] * 8)]:
            with self.assertRaises(ValueError):
                self.contract.feedback_positions(names, positions, speeds)
            self.assertIsNone(self.contract.targets)

    def test_launch_uses_original_executable_and_configured_bringup_geometry(self):
        config = dict(module_names=self.modules, steering_joint_names=self.steering,
            traction_joint_names=self.traction, wheel_mount_angles=self.mounts,
            wheels_x=[.845, .845, -.845, -.845], wheels_y=[.495, -.495, .495, -.495],
            max_steering_velocity=.3, max_traction_velocity=.3, minimum_linear_speed=.001,
            cmd_vel_timeout=.5, joint_feedback_timeout=.3, output_frame_id='base_link')
        with patch('mobotic_kinematics.launching.runtime_command'):
            model, adapter = romb_kinematics_nodes(lambda **kw: kw, 'test_robot', config)
        self.assertEqual(model['package'], 'mobotic_kinematics')
        self.assertEqual(adapter['package'], 'mobotic_kinematics')
        self.assertEqual(model['executable'], 'romb_kinematics')
        self.assertEqual(adapter['executable'], 'romb_kinematics_adapter')
        self.assertEqual(model['parameters'][0]['wheel_x'], config['wheels_x'])
        self.assertTrue(model['parameters'][0]['is_omnidirectional'])
        self.assertEqual(adapter['parameters'][0]['wheel_mount_angles'], self.mounts)
        self.assertIn(('~/inverse_out', 'romb/kinematics/raw_setpoints'), model['remappings'])

    def test_romb_is_default_and_other_backends_are_rejected(self):
        with patch('mobotic_kinematics.launching.romb_kinematics_nodes', return_value=['romb']) as launch:
            self.assertEqual(kinematics_nodes(dict, 'robot', {}), ['romb'])
            launch.assert_called_once_with(dict, 'robot', {})
        for backend in ('native', 'unknown'):
            with self.assertRaisesRegex(ValueError, 'ROMB'):
                kinematics_nodes(dict, 'robot', {}, backend)


class RombRuntimeTest(unittest.TestCase):
    def prepare(self, directory):
        root = Path(directory)
        files = ['qemu-x86_64-static', 'rootfs/usr/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2',
                 'rootfs/opt/ros/jazzy/lib/agv_kinematics/agv_kinematics', 'udp-only.xml']
        for name in files:
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(name.encode())
        record = {'kinematics_binary_sha256': hashlib.sha256(files[2].encode()).hexdigest(),
                  'qemu_binary_sha256': hashlib.sha256(files[0].encode()).hexdigest()}
        (root / 'manifest.json').write_text(json.dumps(record))
        return root

    def test_runtime_preserves_domain_and_scopes_transport_changes_to_guest(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.prepare(directory)
            with patch.dict(os.environ, {'ROS_DOMAIN_ID': '42', 'ROS_LOCALHOST_ONLY': '1'}):
                command, env = runtime_command(['--ros-args'], root)
                self.assertEqual(os.environ['ROS_LOCALHOST_ONLY'], '1')
            self.assertEqual(env['ROS_DOMAIN_ID'], '42')
            self.assertNotIn('ROS_LOCALHOST_ONLY', env)
            self.assertEqual(env['ROS_AUTOMATIC_DISCOVERY_RANGE'], 'SYSTEM_DEFAULT')
            self.assertEqual(command[-1], '--ros-args')
            self.assertIn(str(root / 'rootfs/opt/ros/jazzy/lib/agv_kinematics/agv_kinematics'), command)

    def test_tampered_romb_binary_is_rejected_before_execution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.prepare(directory)
            (root / 'rootfs/opt/ros/jazzy/lib/agv_kinematics/agv_kinematics').write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError, 'checksum mismatch'):
                runtime_command([], root)

    def test_missing_runtime_is_not_silently_replaced_by_native_kinematics(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(RuntimeError, 'incomplete'):
                runtime_command([], directory)


if __name__ == '__main__':
    unittest.main()
