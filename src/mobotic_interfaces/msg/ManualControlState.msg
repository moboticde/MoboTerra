# State of the manual joystick command source.

std_msgs/Header header

bool connected
# Physical button state only; false when no deadman is configured.
bool deadman_pressed
# Valid connected source with its optional deadman requirement satisfied.
# Supervisor gates this field, not the telemetry-only deadman_pressed field.
bool command_active
bool turbo_active
string status_message
