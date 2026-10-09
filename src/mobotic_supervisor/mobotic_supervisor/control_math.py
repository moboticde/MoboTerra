import math


def steering_position_representable(angle, resolution):
    """Match CAN signed-32-bit ticks and C++ rounding away from zero at halves."""
    if not math.isfinite(angle) or not math.isfinite(resolution) or resolution <= 0.0:
        return False
    ticks = angle * resolution / (2.0 * math.pi)
    # Strict half-tick boundaries avoid Python's different round-to-even rule.
    return math.isfinite(ticks) and -(2**31) - 0.5 < ticks < (2**31 - 1) + 0.5


def clamp(value, minimum, maximum):
    return min(max(value, minimum), maximum)


def clamp_planar(x_value, y_value, maximum_magnitude):
    magnitude = math.hypot(x_value, y_value)
    if magnitude <= maximum_magnitude or magnitude == 0.0:
        return x_value, y_value
    scale = maximum_magnitude / magnitude
    return x_value * scale, y_value * scale


def slew(value, previous, maximum_delta):
    return clamp(value, previous - maximum_delta, previous + maximum_delta)
