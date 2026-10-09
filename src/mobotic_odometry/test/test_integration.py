"""Exercise the production integration engine without ROS or hardware."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest


SOURCE = Path(__file__).parents[1] / 'mobotic_odometry' / 'integration.py'
spec = importlib.util.spec_from_file_location('mobotic_odometry_integration_test', SOURCE)
integration = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = integration
spec.loader.exec_module(integration)


def observe(engine, seconds, twist=(1.0, 0.0, 0.0), now=None, receipt=None):
    return engine.observe(round(seconds * 1e9), round((seconds if now is None else now) * 1e9),
                          seconds if receipt is None else receipt, twist)


class IntegrationTest(unittest.TestCase):
    def test_startup_primes_without_inventing_past_motion(self):
        engine = integration.PlanarOdometry()
        self.assertFalse(engine.fresh(10_000_000_000, 10.0))
        first = observe(engine, 10.0)
        self.assertEqual(first.pose, (0.0, 0.0, 0.0))
        self.assertEqual(first.stamp_ns, 10_000_000_000)
        self.assertEqual(first.twist, (1.0, 0.0, 0.0))

    def test_forward_reverse_and_crab_motion(self):
        for twist in ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.4, -0.6, 0.0)):
            with self.subTest(twist=twist):
                engine = integration.PlanarOdometry()
                observe(engine, 10.0, twist)
                for i in range(1, 51):
                    result = observe(engine, 10.0 + i * 0.02, twist)
                self.assertAlmostEqual(result.pose[0], twist[0], places=10)
                self.assertAlmostEqual(result.pose[1], twist[1], places=10)
                self.assertEqual(result.pose[2], 0.0)

    def test_translation_uses_world_heading_not_body_coordinates(self):
        engine = integration.PlanarOdometry(initial_pose=(2.0, 3.0, math.pi / 2))
        observe(engine, 10.0)
        result = observe(engine, 10.1)
        self.assertAlmostEqual(result.pose[0], 2.0)
        self.assertAlmostEqual(result.pose[1], 3.1)

    def test_exact_combined_curve_and_negative_rotation(self):
        for wz in (1.0, -1.0):
            engine = integration.PlanarOdometry()
            twist = (1.0, 0.3, wz)
            observe(engine, 10.0, twist)
            for i in range(1, 51):
                result = observe(engine, 10.0 + i * 0.02, twist)
            a, b = math.sin(wz) / wz, (1 - math.cos(wz)) / wz
            self.assertAlmostEqual(result.pose[0], a - b * 0.3, places=10)
            self.assertAlmostEqual(result.pose[1], b + a * 0.3, places=10)
            self.assertAlmostEqual(result.pose[2], wz, places=10)

    def test_pure_rotation_wraps_heading_and_keeps_translation_zero(self):
        engine = integration.PlanarOdometry()
        twist = (0.0, 0.0, 2.0)
        observe(engine, 10.0, twist)
        for i in range(1, 201):
            result = observe(engine, 10 + i * 0.02, twist)
        self.assertEqual(result.pose[:2], (0.0, 0.0))
        self.assertAlmostEqual(result.pose[2], math.remainder(8.0, 2 * math.pi), places=10)

    def test_two_sample_velocity_average_handles_acceleration(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10.0, (0.0, 0.0, 0.0))
        result = observe(engine, 10.1, (2.0, 0.0, 0.0))
        self.assertAlmostEqual(result.pose[0], 0.1)
        self.assertEqual(result.twist, (2.0, 0.0, 0.0))

    def test_zero_and_near_zero_yaw_are_numerically_stable(self):
        for wz in (0.0, 1e-12, -1e-12, 1e-5, 0.001):
            dx, dy, _, jacobian = integration.body_step(1.0, 0.0, wz, 0.1)
            self.assertAlmostEqual(dx, 0.1, places=8)
            self.assertAlmostEqual(dy, wz * 0.005, places=10)
            self.assertTrue(all(math.isfinite(value) for row in jacobian for value in row))

    def test_step_velocity_jacobian_matches_finite_differences(self):
        for twist in ((0.7, -0.4, 0.0), (0.7, -0.4, 1e-6), (0.7, -0.4, -1.2)):
            *_, jacobian = integration.body_step(*twist, 0.2)
            for axis in range(3):
                plus, minus = list(twist), list(twist)
                plus[axis] += 1e-6
                minus[axis] -= 1e-6
                upper = integration.body_step(*plus, 0.2)[:3]
                lower = integration.body_step(*minus, 0.2)[:3]
                for row in range(3):
                    self.assertAlmostEqual(jacobian[row][axis], (upper[row] - lower[row]) / 2e-6, places=7)

    def test_covariance_is_symmetric_positive_and_ros_axes_are_correct(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10.0, (1.0, 0.3, 0.8))
        for i in range(1, 51):
            result = observe(engine, 10 + i * 0.02, (1.0, 0.3, 0.8))
        p = engine.covariance
        for i in range(3):
            self.assertGreater(p[i][i], 0.01)
            for j in range(3):
                self.assertAlmostEqual(p[i][j], p[j][i], places=12)
        for direction in ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, -2, 3), (-3, 5, -7)):
            self.assertGreater(sum(direction[i] * p[i][j] * direction[j] for i in range(3) for j in range(3)), 0)
        for axis in (2, 3, 4):
            self.assertEqual(result.pose_covariance[axis * 6 + axis], 1e6)
            self.assertEqual(result.twist_covariance[axis * 6 + axis], 1e6)
        self.assertAlmostEqual(result.pose_covariance[5], p[0][2])
        self.assertAlmostEqual(result.pose_covariance[30], p[2][0])
        self.assertEqual(result.twist_covariance[35], 0.01)

    def test_duplicate_and_reordered_samples_do_not_move_or_renew(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10.0)
        first = observe(engine, 10.1)
        self.assertIsNone(observe(engine, 10.1, (3, 0, 0), now=10.2, receipt=10.2))
        self.assertIsNone(observe(engine, 10.0, (3, 0, 0), now=10.2, receipt=10.2))
        self.assertEqual(tuple(engine.pose), first.pose)
        self.assertEqual(engine.last_received, 10.1)
        self.assertFalse(engine.fresh(10_500_000_000, 10.5))

    def test_bad_timestamps_are_rejected(self):
        for stamp, now in ((0, 10), (9.6, 10), (10.2, 10), (-1, 10)):
            engine = integration.PlanarOdometry()
            self.assertIsNone(observe(engine, stamp, now=now))
            self.assertFalse(engine.valid)
        engine = integration.PlanarOdometry()
        self.assertIsNotNone(observe(engine, 10.05, now=10.0, receipt=10.0))

    def test_invalid_newer_velocity_breaks_chain_and_recovers_without_gap_motion(self):
        for twist in ((math.nan, 0, 0), (0, math.inf, 0), (0, 0, -math.inf), (6, 0, 0), (0, 0, 6)):
            engine = integration.PlanarOdometry()
            observe(engine, 10.0)
            self.assertIsNone(observe(engine, 10.1, twist))
            result = observe(engine, 10.2)
            self.assertEqual(result.pose, (0, 0, 0))
            self.assertTrue(engine.pose_degraded)
            next_result = observe(engine, 10.3)
            self.assertAlmostEqual(next_result.pose[0], 0.1)

    def test_long_acquisition_gap_rebases_pose_and_inflates_uncertainty(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10.0)
        before = observe(engine, 10.1)
        after = observe(engine, 12.0)
        self.assertEqual(before.pose, after.pose)
        self.assertGreater(after.pose_covariance[0], before.pose_covariance[0])
        self.assertTrue(engine.pose_degraded)
        recovered = observe(engine, 12.1)
        self.assertAlmostEqual(recovered.pose[0], 0.2)

    def test_long_steady_gap_does_not_integrate_even_if_source_gap_is_short(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10, receipt=100)
        result = observe(engine, 10.1, receipt=101)
        self.assertEqual(result.pose, (0, 0, 0))
        self.assertTrue(engine.pose_degraded)

    def test_paused_and_backward_ros_clocks_do_not_reuse_motion(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10, receipt=100)
        self.assertFalse(engine.fresh(10_000_000_000, 100.31))
        self.assertIsNone(observe(engine, 10, receipt=101))
        self.assertEqual(engine.pose, [0, 0, 0])
        engine = integration.PlanarOdometry()
        observe(engine, 10)
        self.assertIsNone(observe(engine, 10.1, now=9, receipt=10.1))
        self.assertIn('backwards', engine.reason)
        self.assertIsNone(observe(engine, 9.1, now=9.1, receipt=10.2))
        result = observe(engine, 10.2, receipt=10.3)
        self.assertEqual(result.pose, (0, 0, 0))

    def test_delayed_acquisition_has_only_remaining_freshness_budget(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10, now=10.2, receipt=100)
        self.assertTrue(engine.fresh(10_290_000_000, 100.09))
        self.assertFalse(engine.fresh(10_310_000_000, 100.11))

    def test_reset_primes_again_and_does_not_admit_old_samples(self):
        engine = integration.PlanarOdometry()
        observe(engine, 10)
        observe(engine, 10.1)
        engine.reset()
        self.assertIsNone(observe(engine, 10.1, now=10.2, receipt=10.2))
        self.assertEqual(engine.pose, [0, 0, 0])
        result = observe(engine, 10.2)
        self.assertEqual(result.pose, (0, 0, 0))
        self.assertFalse(engine.pose_degraded)

    def test_parameters_reject_invalid_configuration(self):
        for parameters in ({'feedback_timeout': 0}, {'max_integration_interval': 0.4},
                           {'initial_pose': [0, 0]}, {'twist_variances': [0, 1, 1]},
                           {'process_variance_rates': [1, math.nan, 1]},
                           {'gap_variance_rates': [-1, 1, 1]}, {'max_linear_speed': math.inf}):
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                integration.PlanarOdometry(**parameters)


if __name__ == '__main__':
    unittest.main()
