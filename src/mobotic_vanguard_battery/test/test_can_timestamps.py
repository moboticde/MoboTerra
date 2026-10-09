"""Delayed/replayed CAN delivery must not renew battery telemetry freshness."""

from types import SimpleNamespace
import math
import unittest

import test_monitor as harness


class CanTimestampTest(unittest.TestCase):
    def setUp(self):
        self.platform = harness.BatteryMonitorTest()
        self.platform.setUp()
        self.node = self.platform.node
        self.battery = self.platform.battery

    def clear_samples(self):
        self.battery.status_stamp = None
        self.battery.soc_stamp = None
        self.battery.electrical_stamp = None
        self.battery.acquisition_stamps.clear()

    def frame(self, field, seconds):
        pgn = getattr(self.battery, 'voltage_pgn' if field == 'electrical' else field + '_pgn')
        data = {
            'status': [0, 0, 0, 0, 0, 0x21, 0, 0],
            'soc': [0, 125, 0, 0, 0, 0, 0, 0],
            'electrical': [0, 0, 0, 0, 192, 3, 0, 125],
        }[field]
        return SimpleNamespace(
            id=(pgn << 8) | self.battery.source_address,
            header=SimpleNamespace(stamp=harness.Time(seconds).to_msg()),
            is_extended=True, is_error=False, is_rtr=False, dlc=8, data=data,
        )

    def send_all(self, seconds):
        for field in ('status', 'soc', 'electrical'):
            self.node._can_callback(self.frame(field, seconds))

    def test_acquisition_time_is_retained_for_every_telegram(self):
        self.clear_samples()
        self.send_all(9.8)
        for field in ('status', 'soc', 'electrical'):
            self.assertEqual(getattr(self.battery, field + '_stamp').nanoseconds, harness.Time(9.8).nanoseconds)
        self.node._publish()
        self.assertTrue(self.platform.system[-1].system_ready)

    def test_delayed_delivery_expires_at_acquisition_plus_timeout(self):
        self.clear_samples()
        self.send_all(9.51)  # 490 ms old with a 500 ms timeout.
        self.node._publish()
        self.assertTrue(self.platform.system[-1].system_ready)
        self.platform.now = harness.Time(10.02)
        self.node._publish()
        self.assertFalse(self.platform.system[-1].communication_ok)
        self.assertFalse(self.platform.system[-1].system_ready)
        self.assertEqual(self.platform.system[-1].stale_batteries, ['system'])
        self.assertTrue(math.isnan(self.platform.primary[-1].percentage))
        self.assertTrue(math.isnan(self.platform.primary[-1].voltage))
        self.assertFalse(self.platform.primary[-1].present)

    def test_stale_or_far_future_frames_cannot_restore_readiness(self):
        for stamp in (1.0, 9.499, 10.101):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.clear_samples()
                self.send_all(stamp)
                self.node._publish()
                self.assertFalse(self.platform.system[-1].system_ready)
                self.assertIsNone(self.battery.status_stamp)
                self.assertIsNone(self.battery.soc_stamp)
                self.assertIsNone(self.battery.electrical_stamp)

    def test_invalid_timestamp_does_not_overwrite_fresh_values(self):
        for stamp in (1.0, 10.101):
            with self.subTest(stamp=stamp):
                frame = self.frame('soc', stamp)
                frame.data[:2] = [255, 255]
                self.node._can_callback(frame)
                self.assertEqual(self.battery.percentage, 0.5)
                self.assertEqual(self.battery.soc_stamp.nanoseconds, harness.Time(10.0).nanoseconds)

    def test_zero_timestamp_uses_reception_time(self):
        self.clear_samples()
        self.send_all(0.0)
        self.node._publish()
        self.assertTrue(self.platform.system[-1].system_ready)
        for field in ('status', 'soc', 'electrical'):
            self.assertEqual(getattr(self.battery, field + '_stamp').nanoseconds, self.platform.now.nanoseconds)
        self.platform.now = harness.Time(10.51)
        self.node._publish()
        self.assertFalse(self.platform.system[-1].system_ready)

    def test_small_future_skew_is_clamped_to_receipt_time(self):
        self.clear_samples()
        self.send_all(10.1)
        self.node._publish()
        self.assertTrue(self.platform.system[-1].system_ready)
        for field in ('status', 'soc', 'electrical'):
            self.assertEqual(getattr(self.battery, field + '_stamp').nanoseconds, self.platform.now.nanoseconds)
        self.platform.now = harness.Time(10.51)
        self.node._publish()
        self.assertFalse(self.platform.system[-1].system_ready)

    def test_out_of_order_samples_cannot_overwrite_newer_fields(self):
        for field in ('status', 'soc', 'electrical'):
            with self.subTest(field=field):
                self.setUp()
                older = self.frame(field, 9.9)
                # Make the older payload contradict the current sample.
                older.data = [255] * 8
                self.node._can_callback(older)
                self.assertEqual(getattr(self.battery, field + '_stamp').nanoseconds, harness.Time(10.0).nanoseconds)
                self.assertEqual(self.battery.high_voltage_status, 1)
                self.assertEqual(self.battery.operational_status, 2)
                self.assertEqual(self.battery.percentage, 0.5)
                self.assertEqual(self.battery.voltage, 48.0)
                self.assertEqual(self.battery.current, 0.0)

    def test_replaying_same_acquisition_does_not_refresh_sample(self):
        for acquisition, replay, expiry, expected_sample in (
            (9.9, 10.2, 10.41, 9.9),
            (10.1, 10.05, 10.51, 10.0),
        ):
            with self.subTest(acquisition=acquisition):
                self.setUp()
                self.clear_samples()
                self.send_all(acquisition)
                self.platform.now = harness.Time(replay)
                self.send_all(acquisition)
                self.platform.now = harness.Time(expiry)
                self.node._publish()
                self.assertFalse(self.platform.system[-1].system_ready)
                self.assertEqual(self.battery.soc_stamp.nanoseconds, harness.Time(expected_sample).nanoseconds)

    def test_one_expired_field_blocks_all_participant_readiness(self):
        for field in ('status', 'soc', 'electrical'):
            with self.subTest(field=field):
                self.setUp()
                self.clear_samples()
                self.send_all(10.0)
                self.platform.now = harness.Time(10.4)
                for other in ('status', 'soc', 'electrical'):
                    if other != field:
                        self.node._can_callback(self.frame(other, 10.4))
                self.platform.now = harness.Time(10.51)
                self.node._can_callback(self.frame(field, 10.0))
                self.node._publish()
                self.assertFalse(self.platform.system[-1].communication_ok)
                self.assertFalse(self.platform.system[-1].system_ready)

    def test_exact_timeout_boundary(self):
        self.clear_samples()
        self.send_all(9.5)
        self.node._publish()
        self.assertTrue(self.platform.system[-1].system_ready)
        self.platform.now = harness.Time(10.000000001)
        self.node._publish()
        self.assertFalse(self.platform.system[-1].system_ready)

    def test_older_healthy_sample_cannot_clear_fault_but_new_samples_recover(self):
        bad = self.frame('soc', 10.0)
        bad.data[:2] = [255, 255]
        self.node._can_callback(bad)
        self.node._can_callback(self.frame('soc', 9.9))
        self.node._publish()
        self.assertTrue(math.isnan(self.battery.percentage))
        self.assertFalse(self.platform.system[-1].system_ready)
        self.assertEqual(self.platform.system[-1].faulted_batteries, ['system'])
        self.platform.now = harness.Time(10.6)
        self.send_all(10.6)
        self.node._publish()
        self.assertTrue(self.platform.system[-1].system_ready)
        self.assertAlmostEqual(self.platform.system[-1].minimum_percentage, 0.50016)

    def test_clock_reversal_cannot_replace_samples_with_earlier_receipts(self):
        for stamp in (0.0, 9.9):
            with self.subTest(stamp=stamp):
                self.setUp()
                self.platform.now = harness.Time(9.9)
                self.send_all(stamp)
                self.node._publish()
                self.assertFalse(self.platform.system[-1].system_ready)
                self.assertEqual(self.battery.soc_stamp.nanoseconds, harness.Time(10.0).nanoseconds)
                self.platform.now = harness.Time(10.1)
                self.send_all(10.1)
                self.node._publish()
                self.assertTrue(self.platform.system[-1].system_ready)


if __name__ == '__main__':
    unittest.main()
