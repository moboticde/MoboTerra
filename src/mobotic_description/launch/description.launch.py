"""Launch only the state publisher using the shared platform geometry."""
from mobotic_config.launching import description_launch


def generate_launch_description():
    return description_launch()
