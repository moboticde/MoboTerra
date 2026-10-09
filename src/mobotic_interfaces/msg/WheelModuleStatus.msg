# Aggregated status for one steering and traction wheel module.

uint8 CONTROL_MODE_UNKNOWN=0
uint8 CONTROL_MODE_POSITION=1
uint8 CONTROL_MODE_VELOCITY=2
uint8 CONTROL_MODE_CURRENT=3

string name
bool enabled
# Both drives have fresh status, motion and fault telemetry. Independent of
# the array publication time. False prohibits motion, including cached enabled.
bool feedback_fresh
string status_message
bool brake_state_known
bool brake_released
uint8 control_mode
float64 motor_current
uint16 status_word
int32 error_code
# error_code is the last known fault; consult feedback_fresh before treating
# zero as confirmation of a healthy module.
