"""Launch kinematics using the shared, validated platform profile."""
from mobotic_config.launching import standalone_launch


def generate_launch_description():
    return standalone_launch('kinematics')
