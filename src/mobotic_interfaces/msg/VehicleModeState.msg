# Authoritative operating mode published by the vehicle supervisor.

std_msgs/Header header

uint8 MANUAL=0
uint8 AUTO_VELOCITY=1
uint8 AUTO_DIRECT=2
uint8 UNKNOWN=255

uint8 current_mode
uint8 requested_mode
bool transition_in_progress
bool motion_permitted
string status_message
