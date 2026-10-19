"""Wire-format and freshness tests; no ROS installation or hardware required."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mobotic_manual_control.remdevice_protocol import (
    RemdeviceReceiver, decode_auxiliary, decode_digital, signed_axis,
)


class WireFormatTest(unittest.TestCase):
    def test_digital_manual_c9(self):
        values = decode_digital([0x81, 0x42, 0x01, 0x80, 0x02, 0x40, 0xff, 0xad])
        asserted = {name for name, value in values.items() if value}
        self.assertEqual(asserted, {'U1', 'U8', 'D2', 'D7', 'SP1', 'SP16', 'SA2', 'SA15',
                                    'STRT', 'TMST', 'EMGY', 'LINK', 'STOP'})

    def test_each_status_bit_is_distinct(self):
        for bit, name in enumerate(('STRT', 'BATT', 'TMST', 'EMGY', 'FBOK', 'LINK', 'SFTY', 'STOP')):
            values = decode_digital([0] * 7 + [1 << bit])
            self.assertEqual({key for key, value in values.items() if value}, {name})

    def test_auxiliary_manual_c14(self):
        values = decode_auxiliary([1, 2, 3, 4, 255, 255, 1, 0x81])
        self.assertEqual([values[f'A{i}'] for i in range(9, 13)], [1, 2, 3, 4])
        self.assertTrue(all(values[name] for name in ('SA17', 'SA25', 'SA32', 'U9', 'D12')))
        self.assertFalse(values['SA24'])
        self.assertFalse(values['U12'])

    def test_unsigned_magnitude_and_separate_direction(self):
        self.assertEqual(signed_axis(255, True, False), 1)
        self.assertEqual(signed_axis(255, False, True), -1)
        self.assertEqual(signed_axis(128, False, False), 0)
        self.assertAlmostEqual(signed_axis(128, True, False), 128 / 255)
        self.assertEqual(signed_axis(255, False, True, invert=True), 1)

    def test_receiver_offset_and_scaling(self):
        self.assertEqual(signed_axis(10, True, False, zero=10, full_scale=210), 0)
        self.assertEqual(signed_axis(110, True, False, zero=10, full_scale=210), 0.5)

    def test_invalid_direction_and_calibration(self):
        for args, kwargs in [((100, True, True), {}), ((256, True, False), {}),
                             ((100, True, False), {'zero': 200}),
                             ((255, True, False), {'full_scale': 200}),
                             ((100, True, False), {'full_scale': 0}),
                             ((True, True, False), {})]:
            with self.subTest(args=args, kwargs=kwargs), self.assertRaises(ValueError):
                signed_axis(*args, **kwargs)


class FreshnessTest(unittest.TestCase):
    def setUp(self):
        self.rx = RemdeviceReceiver(10)
        self.now = 10_000_000_000

    def send(self, cob=None, data=None, *, dt=0, stamp=None, **flags):
        self.now += dt
        data = [0] * 8 if data is None else data
        return self.rx.update(self.rx.digital_id if cob is None else cob, len(data), data,
                              self.now if stamp is None else stamp, self.now, **flags)

    def pair(self):
        self.send(data=[1, 0, 0, 0, 0, 0, 0, 0x28])
        self.send(self.rx.analog_id, [255] * 8, dt=1_000_000)
        return self.rx.snapshot(self.now, consume=True)

    def test_requires_both_pdos(self):
        self.send()
        self.assertIsNone(self.rx.snapshot(self.now))
        self.send(self.rx.analog_id)
        sample = self.rx.snapshot(self.now)
        self.assertIsNotNone(sample)
        self.assertEqual(sample.stamp_ns, self.now)

    def test_pair_stamp_is_oldest_acquisition(self):
        before = self.now
        sample = self.pair()
        self.assertEqual(sample.stamp_ns, before)
        self.assertEqual(sample.analog, (255,) * 8)

    def test_single_repeating_pdo_cannot_renew_command(self):
        self.pair()
        self.send(dt=50_000_000)
        self.assertIsNone(self.rx.snapshot(self.now, consume=True))
        self.send(self.rx.analog_id, dt=1_000_000)
        self.assertIsNotNone(self.rx.snapshot(self.now, consume=True))
        self.assertIsNone(self.rx.snapshot(self.now, consume=True))

    def test_whole_bus_loss_expires_on_timer(self):
        self.pair()
        self.assertIsNone(self.rx.snapshot(self.now + 251_000_000))
        self.assertEqual(self.rx.reason, 'PDO timeout')

    def test_missing_one_pdo_requires_new_pair_after_timeout(self):
        self.pair()
        self.send(dt=251_000_000)
        self.assertIsNone(self.rx.snapshot(self.now))
        self.send(self.rx.analog_id, dt=1_000_000)
        self.assertIsNotNone(self.rx.snapshot(self.now))

    def test_excessive_pair_skew(self):
        self.send()
        self.send(self.rx.analog_id, dt=80_000_000)
        self.assertIsNone(self.rx.snapshot(self.now))

    def test_radio_loss_stop_available_without_new_analog(self):
        self.pair()
        self.send(data=[0] * 7 + [0x80], dt=1_000_000)
        digital = self.rx.digital(self.now)
        self.assertTrue(digital['STOP'])
        self.assertFalse(digital['EMGY'])
        self.assertFalse(digital['LINK'])
        self.assertIsNone(self.rx.snapshot(self.now, consume=True))

    def test_stale_future_and_zero_stamps_invalidate(self):
        for offset in (-251_000_000, 1, -self.now):
            self.pair()
            self.assertFalse(self.send(stamp=self.now + offset))
            self.assertIsNone(self.rx.snapshot(self.now))
            self.now += 1_000_000

    def test_replayed_or_out_of_order_pdo_ignored(self):
        self.pair()
        old = self.rx.samples[self.rx.digital_id]
        self.assertFalse(self.send(stamp=old[0]))
        self.assertFalse(self.send(stamp=old[0] - 1))
        self.assertEqual(self.rx.samples[self.rx.digital_id], old)

    def test_bootup_blocks_queued_pre_reset_pdo(self):
        self.pair()
        old = self.now
        self.send(self.rx.heartbeat_id, [0], dt=10_000_000)
        self.assertFalse(self.send(stamp=old))
        self.send(self.rx.analog_id, dt=1_000_000)
        self.assertIsNone(self.rx.snapshot(self.now))
        self.send(dt=1_000_000)
        self.assertIsNotNone(self.rx.snapshot(self.now))

    def test_nonoperational_and_operational_heartbeat(self):
        for status in (0, 4, 127, 132, 255):
            self.pair()
            self.send(self.rx.heartbeat_id, [status], dt=1_000_000)
            self.assertIsNone(self.rx.snapshot(self.now))
            self.now += 1_000_000
        self.pair()
        self.send(self.rx.heartbeat_id, [5], dt=1_000_000)
        self.assertIsNotNone(self.rx.snapshot(self.now))

    def test_emcy_latches_until_explicit_error_reset(self):
        self.pair()
        self.send(self.rx.emergency_id, [0x20, 0x23, 1, 0, 0, 0, 0, 0], dt=1_000_000)
        self.assertEqual(self.rx.emergency_code, 0x2320)
        self.assertIsNone(self.pair())
        self.send(self.rx.emergency_id, [0] * 8, dt=1_000_000)
        self.now += 1_000_000
        self.assertIsNotNone(self.pair())

    def test_zero_emcy_with_nonzero_error_register_does_not_clear_fault(self):
        self.send(self.rx.emergency_id, [0, 0, 1, 0, 0, 0, 0, 0])
        self.now += 1_000_000
        self.assertIsNone(self.pair())

    def test_clock_reversal_clears_pair_and_accepts_new_epoch(self):
        self.pair()
        self.now -= 1_000_000_000
        self.assertIsNone(self.rx.snapshot(self.now))
        self.assertIsNotNone(self.pair())

    def test_wrong_dlc_and_invalid_payload_clear_pair(self):
        for data in ([0] * 7, [0] * 9, [-1] * 8, [256] * 8):
            self.pair()
            self.assertFalse(self.send(data=data, dt=1_000_000))
            self.assertIsNone(self.rx.snapshot(self.now))
            self.now += 1_000_000

    def test_unrelated_extended_and_rtr_do_not_change_state(self):
        self.pair()
        self.assertFalse(self.send(0x185))
        self.assertFalse(self.send(is_extended=True))
        self.assertFalse(self.send(is_rtr=True))
        self.assertIsNotNone(self.rx.snapshot(self.now))

    def test_bus_error_invalidates(self):
        self.pair()
        self.assertFalse(self.send(0x20000040, is_error=True))
        self.assertIsNone(self.rx.snapshot(self.now))

    def test_invalid_receiver_configuration(self):
        for node in (0, 128, True, '10'):
            with self.assertRaises(ValueError):
                RemdeviceReceiver(node)
        with self.assertRaises(ValueError):
            RemdeviceReceiver(10, timeout_ns=1, max_skew_ns=2)


if __name__ == '__main__':
    unittest.main()
