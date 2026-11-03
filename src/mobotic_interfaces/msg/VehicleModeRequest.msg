# Semantic operating-mode request from the operator interface.
# This topic accepts MANUAL/AUTONOMOUS only. KINEMATICOFF requires SetVehicleMode.

std_msgs/Header header

uint8 MANUAL=0
uint8 AUTONOMOUS=1
uint8 KINEMATICOFF=2
uint8 UNKNOWN=255

uint8 requested_mode
