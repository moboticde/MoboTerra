# Source this file in any terminal that needs the MoboTerra ROS environment.
# Resolve the real path so compatibility symlinks work too.
MOBOTERRA_PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
export MOBOTERRA_PROJECT_ROOT
for moboterra_setup in /opt/ros/jazzy/setup.bash \
    "$MOBOTERRA_PROJECT_ROOT/install/local_setup.bash" \
    "$MOBOTERRA_PROJECT_ROOT/install-controller-types/local_setup.bash"; do
    if [[ ! -r "$moboterra_setup" ]]; then
        echo "Missing ROS setup: $moboterra_setup" >&2
        return 1
    fi
    source "$moboterra_setup" || return
done
source "$MOBOTERRA_PROJECT_ROOT/config/raspberry-pi/ros.env" || return
unset moboterra_setup
