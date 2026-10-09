"""Read-only Vanguard/J1939 decoding; BatteryState SOC is a fraction, not percent."""
import math


def decode_soc(raw):
    # Reference scale is 0.001563 percent/count (~1/640 percent).
    # Accept full-scale rounding, never turn reserved/invalid data into a full battery.
    return min(raw * 0.001563 / 100.0, 1.0) if 0 <= raw <= 64000 else math.nan


def decode_electrical(raw_voltage, raw_current):
    # J1939 16-bit error/not-available ranges are above 0xFAFF.
    voltage = raw_voltage * 0.05 if 0 <= raw_voltage <= 0xFAFF else math.nan
    current = raw_current * 0.05 - 1600.0 if 0 <= raw_current <= 0xFAFF else math.nan
    return voltage, current


def values_valid(percentage, voltage, current):
    return (all(math.isfinite(value) for value in (percentage, voltage, current))
            and 0.0 <= percentage <= 1.0 and voltage >= 0.0)
