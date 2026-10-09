# Aggregate readiness of all configured battery participants.

std_msgs/Header header

bool communication_ok
bool system_ready
bool high_voltage_connected
float32 minimum_percentage
string[] stale_batteries
string[] faulted_batteries
string status_message
