"""Read-only Vanguard/J1939 decoding with fractional state of charge."""
import math


def decode_soc(raw):
    # Both installed packs report 0xFC40 (100.93% at the Vanguard scale)
    # after a full charge. Keep the ROS fraction within its 0..1 range.
    if not 0 <= raw <= 0xFC40:
        return math.nan
    return min(raw * 0.001563 / 100.0, 1.0)


def decode_electrical(raw_voltage, raw_current):
    # J1939 16-bit error/not-available ranges are above 0xFAFF.
    voltage = raw_voltage * 0.05 if 0 <= raw_voltage <= 0xFAFF else math.nan
    current = raw_current * 0.05 - 1600.0 if 0 <= raw_current <= 0xFAFF else math.nan
    return voltage, current


def values_valid(percentage, voltage, current):
    return (all(math.isfinite(value) for value in (percentage, voltage, current))
            and 0.0 <= percentage <= 1.0 and voltage >= 0.0)
