# One named module in the final supervised motion set. Enable/reset is separate.
string name
float64 steering_position # radians in the controller steering coordinate
float64 steering_profile_velocity # radians/s
float64 traction_velocity # m/s at the wheel contact point
# KINEMATICOFF uses current in amperes; velocity must then be zero.
bool traction_current_control
float64 traction_current
