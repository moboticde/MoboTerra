"""Launch the standalone scanner field adapter."""
from mobotic_config.launching import standalone_launch


def generate_launch_description():
    return standalone_launch('safety')
