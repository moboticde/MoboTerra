# Semantic operating-mode request from the operator interface.
# This topic accepts MANUAL/AUTO_VELOCITY only. AUTO_DIRECT requires SetVehicleMode.

std_msgs/Header header

uint8 MANUAL=0
uint8 AUTO_VELOCITY=1
uint8 AUTO_DIRECT=2
uint8 UNKNOWN=255

uint8 requested_mode
