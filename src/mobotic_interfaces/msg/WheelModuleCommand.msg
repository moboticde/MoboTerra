# Management command applied simultaneously to every configured wheel module.
# Enabling the drives also releases their brakes; disabling engages the brakes.
# This message never carries steering or traction motion setpoints.

std_msgs/Header header

bool enable

# One-shot request to clear errors on every steering and traction controller.
bool clear_errors
