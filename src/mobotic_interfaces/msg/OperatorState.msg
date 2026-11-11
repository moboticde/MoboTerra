# Actual operator-provider sample; timer republication must preserve this stamp.
std_msgs/Header header
string source_id
bool connected
bool command_active
# Optional physical hold-to-drive button; both fields are false when unconfigured.
# Reserved compatibility fields; manual input always reports false.
# The supervisor uses command_active.
bool deadman_pressed
bool deadman_state_known
bool estop_pressed
bool estop_state_known
bool controls_neutral
bool turbo_active
string status_message
