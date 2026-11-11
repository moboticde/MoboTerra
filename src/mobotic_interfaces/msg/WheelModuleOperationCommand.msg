# Supervisor-owned lease and idempotent, explicitly requested reset transaction.
std_msgs/Header header
string supervisor_instance
uint64 authority_generation
bool enable
uint64 reset_id
# Current for KINEMATICOFF; velocity for MANUAL/AUTONOMOUS.
bool traction_current_control
# Issued only after the supervisor proves standstill during a mode transition.
bool reconfigure_traction_mode
