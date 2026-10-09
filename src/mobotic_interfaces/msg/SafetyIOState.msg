# Individual safety inputs for monitoring and diagnostics.
# OSSD active means the corresponding safety outputs are energized.

std_msgs/Header header

bool emergency_stop_active
bool safety_enable_active
bool sto_active
# True only for a separately mapped, fresh hardware signal. Defaults below
# may be conservative ROS interlocks, not confirmation of physical STO.
bool sto_state_known
bool safety_enable_state_known
uint8 flexisoft_status_raw

bool front_left_protective_field_clear
bool front_left_warning_field_clear
bool front_left_ossd_active

bool rear_right_protective_field_clear
bool rear_right_warning_field_clear
bool rear_right_ossd_active
