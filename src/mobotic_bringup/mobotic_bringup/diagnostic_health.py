"""Pure health rules shared by the ROS monitor and host-side tests."""
import math

OK, WARN, ERROR, STALE = 0, 1, 2, 3


def freshness(received, now, timeout, stamp_age=None):
    if received is None or not 0.0 <= now - received <= timeout:
        return STALE, 'No recent message'
    if stamp_age is not None and (not math.isfinite(stamp_age) or not -0.1 <= stamp_age <= timeout):
        return STALE, 'Message timestamp stale/invalid'
    return OK, 'Receiving'


def battery_health(msg, minimum=0.2):
    if not msg.communication_ok:
        return ERROR, 'Battery telemetry missing/stale'
    if (not msg.system_ready or not msg.high_voltage_connected or
            not math.isfinite(msg.minimum_percentage) or not 0 <= msg.minimum_percentage <= 1):
        return ERROR, 'Battery not ready/invalid'
    if msg.minimum_percentage < minimum:
        return ERROR, 'SOC below enable threshold'
    return OK, 'Battery ready'


def safety_health(msg):
    if not msg.communication_ok:
        return ERROR, 'FlexiSoft telemetry missing/stale'
    if msg.emergency_stop_active or msg.sto_active or not msg.system_ready:
        return ERROR, msg.status_message or 'Safety not ready'
    if not msg.motion_permitted or msg.warning_field_active or msg.safety_override_active:
        return WARN, msg.status_message or 'Safety restriction/override'
    return OK, msg.status_message or 'Safety ready'
