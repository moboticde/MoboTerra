# High-level safety result consumed by the vehicle supervisor.
# This message reports the hardware safety system; it does not replace it.

std_msgs/Header header

bool communication_ok
bool system_ready
bool motion_permitted
bool protective_stop_active
bool warning_field_active
bool emergency_stop_active
bool sto_active
bool safety_override_active
string status_message
