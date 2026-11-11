"""Launch original ROMB kinematics with the shared platform profile."""


def generate_launch_description():
    from mobotic_kinematics.launching import generate_launch_description as launch_kinematics
    return launch_kinematics()
