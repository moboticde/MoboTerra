"""REMdevice move CANopen wire format (Service Manual v2.0, C-9..C-14).

This module has no ROS or CAN transmit dependency. It decodes signal names,
not the customized PAIL controls. Physical roles need a measured mapping.
"""
from dataclasses import dataclass


STATUS_BITS = ('STRT', 'BATT', 'TMST', 'EMGY', 'FBOK', 'LINK', 'SFTY', 'STOP')


def _payload(data, size):
    if len(data) < size or any(type(x) is not int or not 0 <= x <= 255 for x in data[:size]):
        raise ValueError('invalid CAN payload')
    return bytes(data[:size])


def decode_digital(data):
    """TPDO1: U1..8, D1..8, SP1..16, SA1..16, reserved, status."""
    data = _payload(data, 8)
    result = {}
    for prefix, offset, count in (('U', 0, 8), ('D', 1, 8), ('SP', 2, 16), ('SA', 4, 16)):
        for bit in range(count):
            result[f'{prefix}{bit + 1}'] = bool(data[offset + bit // 8] & (1 << (bit % 8)))
    result.update({name: bool(data[7] & (1 << bit)) for bit, name in enumerate(STATUS_BITS)})
    return result


def decode_auxiliary(data):
    """TPDO3: A9..12, two reserved bytes, SA17..32 (C-14)."""
    data = _payload(data, 8)
    result = {f'A{i + 9}': data[i] for i in range(4)}
    result.update({f'SA{i + 17}': bool(data[6 + i // 8] & (1 << (i % 8))) for i in range(16)})
    for i in range(4):
        result[f'U{i + 9}'] = bool(data[7] & (1 << i))
        result[f'D{i + 9}'] = bool(data[7] & (1 << (i + 4)))
    return result


def signed_axis(magnitude, up, down, *, zero=0, full_scale=255, invert=False):
    """Combine unsigned An with its Un/Dn qualifiers; never assume center=128.

    zero/full_scale describe the *received* magnitude after receiver scaling.
    Simultaneous directions are invalid. No direction means neutral, even if
    the receiver applies a nonzero analog offset at rest.
    """
    if (type(magnitude) is not int or not 0 <= magnitude <= 255
            or type(zero) is not int or type(full_scale) is not int
            or not 0 <= zero < full_scale <= 255
            or any(type(value) is not bool for value in (up, down, invert))):
        raise ValueError('invalid axis value or calibration')
    if up and down:
        raise ValueError('contradictory U/D direction qualifiers')
    if not up and not down:
        return 0.0
    if not zero <= magnitude <= full_scale:
        raise ValueError('axis magnitude exceeds commissioned range')
    value = (magnitude - zero) / (full_scale - zero)
    return value * (-1 if down else 1) * (-1 if invert else 1)


@dataclass(frozen=True)
class PdoSample:
    stamp_ns: int
    digital: dict
    analog: tuple


class RemdeviceReceiver:
    """Validate and collect factory-layout TPDO1/2 without transmitting.

    Each returned command sample requires BOTH PDOs to advance, with bounded
    skew and age. A timer or a single repeating PDO cannot renew old input.
    CAN headers must carry acquisition time in the same clock as now_ns.
    """

    def __init__(self, node_id, *, timeout_ns=250_000_000, max_skew_ns=75_000_000):
        if type(node_id) is not int or not 1 <= node_id <= 127:
            raise ValueError('node_id must be 1..127')
        if (type(timeout_ns) is not int or type(max_skew_ns) is not int
                or not 0 < max_skew_ns <= timeout_ns):
            raise ValueError('require 0 < max_skew_ns <= timeout_ns')
        self.node_id = node_id
        self.timeout_ns = timeout_ns
        self.max_skew_ns = max_skew_ns
        self.digital_id = 0x180 + node_id
        self.analog_id = 0x280 + node_id
        self.heartbeat_id = 0x700 + node_id
        self.emergency_id = 0x80 + node_id
        self.samples = {}
        self.last_stamps = {}
        self.consumed = {}
        self.last_now_ns = None
        self.epoch_ns = 0
        self.emergency_code = 0
        self.emergency_register = 0
        self.reason = 'waiting for both PDOs'

    def invalidate(self, reason, now_ns):
        self.samples.clear()
        self.consumed.clear()
        self.epoch_ns = now_ns
        self.reason = reason

    def _tick(self, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            raise ValueError('positive integer clock required')
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.last_stamps.clear()
            self.invalidate('clock moved backwards', now_ns)
        self.last_now_ns = now_ns
        if any(not 0 <= now_ns - stamp <= self.timeout_ns for stamp, _ in self.samples.values()):
            self.invalidate('PDO timeout', now_ns)

    def update(self, cob_id, dlc, data, stamp_ns, now_ns, *, is_extended=False,
               is_rtr=False, is_error=False):
        """Return True for a newly accepted PDO; unrelated traffic is ignored.

        Call snapshot() on the publishing timer too, so total bus loss expires
        state even when no receive callback runs. Malformed relevant traffic
        invalidates the pair. Boot-up/non-operational NMT or EMCY also clears it.
        """
        self._tick(now_ns)
        if is_error:
            self.invalidate('CAN error frame', now_ns)
            return False
        relevant = {self.digital_id, self.analog_id, self.heartbeat_id, self.emergency_id}
        if is_extended or is_rtr or cob_id not in relevant:
            return False
        expected = 1 if cob_id == self.heartbeat_id else 8
        if (type(dlc) is not int or dlc != expected or type(stamp_ns) is not int
                or stamp_ns <= 0 or not 0 <= now_ns - stamp_ns <= self.timeout_ns):
            self.invalidate('invalid DLC or acquisition timestamp', now_ns)
            return False
        try:
            payload = _payload(data, expected)
        except (TypeError, ValueError):
            self.invalidate('invalid CAN payload', now_ns)
            return False
        if stamp_ns < self.epoch_ns or stamp_ns <= self.last_stamps.get(cob_id, -1):
            return False
        self.last_stamps[cob_id] = stamp_ns
        if cob_id == self.heartbeat_id:
            # Bit 7 may be the node-guarding toggle when another master polls.
            if payload[0] & 0x7f != 5:
                self.invalidate('receiver boot-up or non-operational', now_ns)
            return False
        if cob_id == self.emergency_id:
            self.emergency_code = int.from_bytes(payload[:2], 'little')
            self.emergency_register = payload[2]
            self.invalidate(f'EMCY 0x{self.emergency_code:04x}', now_ns)
            return False
        if self.emergency_code or self.emergency_register:
            self.invalidate('receiver EMCY is uncleared', now_ns)
            return False
        self.samples[cob_id] = (stamp_ns, payload)
        self.reason = 'waiting for both PDOs'
        return True

    def digital(self, now_ns):
        """Fresh status, including STOP/radio loss, independent of analog PDO."""
        self._tick(now_ns)
        sample = self.samples.get(self.digital_id)
        return decode_digital(sample[1]) if sample is not None else None

    def snapshot(self, now_ns, *, consume=False):
        self._tick(now_ns)
        if not {self.digital_id, self.analog_id} <= self.samples.keys():
            return None
        digital_stamp, digital = self.samples[self.digital_id]
        analog_stamp, analog = self.samples[self.analog_id]
        if abs(digital_stamp - analog_stamp) > self.max_skew_ns:
            self.reason = 'PDOs too far apart'
            return None
        if consume and any(self.samples[cob][0] <= self.consumed.get(cob, -1)
                           for cob in (self.digital_id, self.analog_id)):
            return None
        if consume:
            self.consumed = {cob: self.samples[cob][0] for cob in (self.digital_id, self.analog_id)}
        self.reason = 'fresh PDO pair'
        return PdoSample(min(digital_stamp, analog_stamp), decode_digital(digital), tuple(analog))
