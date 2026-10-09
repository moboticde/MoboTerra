"""Launch a profile-selected virtual wheel actuator."""
from mobotic_config.launching import virtual_launch


def generate_launch_description():
    return virtual_launch()
