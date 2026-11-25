"""Recognize a fresh, deliberate B1+B2 hold and release without ROS."""

from .remdevice_protocol import signed_axis


RESTART_HOLD_NS = 5_000_000_000


def controls_neutral(mapping, sample):
    try:
        for axis in mapping.axes.values():
            channel = axis['channel']
            value = signed_axis(sample.analog[channel - 1], sample.digital[f'U{channel}'],
                                sample.digital[f'D{channel}'], zero=axis['zero'],
                                full_scale=axis['full_scale'], invert=axis['invert'])
            if abs(value) > 0.08:
                return False
    except ValueError:
        return False
    return True


class RestartGesture:
    def __init__(self, timeout_ns):
        self.timeout_ns = timeout_ns
        self.clear()

    def clear(self):
        self.stamp_ns = None
        self.armed = False
        self.retry_with_b1 = False
        self.hold_started_ns = None
        self.waiting_release = False

    def observe(self, stamp_ns, *, b1, b2, previous_b2, permitted):
        """Return whether B2 is reserved and whether to request a restart."""
        if self.stamp_ns is not None and not 0 < stamp_ns - self.stamp_ns <= self.timeout_ns:
            self.clear()
        self.stamp_ns = stamp_ns

        if not b1 and not b2:
            restart = self.waiting_release and permitted
            self.armed = True
            self.retry_with_b1 = False
            self.hold_started_ns = None
            self.waiting_release = False
            return False, restart

        if self.waiting_release or (b1 and b2):
            can_start = self.armed or self.retry_with_b1
            self.armed = self.retry_with_b1 = False
            if not permitted:
                self.hold_started_ns = None
                self.waiting_release = False
            elif can_start and previous_b2 is False:
                self.hold_started_ns = stamp_ns
            elif b1 and b2 and self.hold_started_ns is not None:
                self.waiting_release = stamp_ns - self.hold_started_ns >= RESTART_HOLD_NS
            return True, False

        if b1 and self.hold_started_ns is not None:
            self.retry_with_b1 = True
        elif not b1:
            self.retry_with_b1 = False
        self.hold_started_ns = None
        if b2:
            reserved = not self.armed
            self.armed = False
            return reserved, False
        return False, False
