"""The commissioned telemetry-only policy still rejects loss, invalid data and charging."""
import math
from types import SimpleNamespace
import unittest
from test_monitor import BatteryMonitorTest, Time


class NonChargingReadinessTest(unittest.TestCase):
    def setUp(self):
        f = BatteryMonitorTest(); f.setUp()
        self.f, self.n, self.b = f, f.node, f.battery
        self.n.require_operational_status = False
        self.n.require_not_charging = True

    def ready(self):
        self.n._publish()
        return self.f.system[-1].system_ready

    def test_cycling_nibble_does_not_generate_false_faults(self):
        for code in range(16):
            self.b.operational_status = code
            self.assertTrue(self.ready())
    def test_charger_mains_and_charge_request_inhibit_but_regeneration_does_not(self):
        self.b.current = .15
        self.assertTrue(self.ready())
        self.b.current = 0
        self.n.charger_stamp = self.f.now
        self.n.charger_inhibits = True
        self.assertFalse(self.ready())
        self.n.charger_stamp = Time(8)
        self.assertTrue(self.ready())
        self.n.charge_request_stamp = self.f.now
        self.n.charge_request_inhibits = True
        self.assertFalse(self.ready())

    def test_invalid_telemetry_hv_loss_and_timeout_still_block(self):
        for field, bad in (('percentage', math.nan), ('voltage', math.nan), ('current', math.nan),
                           ('high_voltage_status', 0), ('soc_stamp', None), ('electrical_stamp', None)):
            original = getattr(self.b, field)
            setattr(self.b, field, bad)
            self.assertFalse(self.ready())
            setattr(self.b, field, original)

    def test_charger_wire_message_is_receive_only_and_expires(self):
        frame = SimpleNamespace(id=0x0CFFBC49, is_extended=True, is_rtr=False, is_error=False, dlc=8,
            header=SimpleNamespace(stamp=self.f.now.to_msg()), data=[0]*4+[3,1,0,0])
        self.n._can_callback(frame)
        self.assertFalse(self.ready())
        self.f.now = Time(10.1)
        frame.header.stamp = self.f.now.to_msg(); frame.data[4:6] = [11,0]
        self.n._can_callback(frame)
        self.assertTrue(self.ready())
        frame.data[5] = 255
        self.f.now = Time(10.2); frame.header.stamp = self.f.now.to_msg()
        self.n._can_callback(frame)
        self.assertFalse(self.ready())


if __name__ == '__main__':
    unittest.main()
