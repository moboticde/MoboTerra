# Authoritative operating mode published by the vehicle supervisor.

std_msgs/Header header

uint8 MANUAL=0
uint8 AUTONOMOUS=1
uint8 KINEMATICOFF=2
uint8 AUTO_VELOCITY=1
uint8 AUTO_DIRECT=2
uint8 SAFETY=3
uint8 ERROR=4
uint8 UNKNOWN=255

uint8 current_mode
uint8 selected_mode
uint8 requested_mode
bool transition_in_progress
bool motion_permitted
bool safety_stop_active
bool error_active
bool reset_in_progress
bool drives_enabled
bool ready_to_drive
bool readiness_known
string supervisor_instance
uint64 authority_generation
uint64 reset_id
string reset_result
string[] reason_codes
string status_message
