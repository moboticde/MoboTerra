# Source this file in any terminal that needs the MoboTerra ROS environment.
# Resolve the real path so compatibility symlinks work too.
MOBOTERRA_PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
export MOBOTERRA_PROJECT_ROOT
moboterra_setups=(/opt/ros/jazzy/setup.bash)
if [[ -n "${MOBOTERRA_ROS_OVERLAY:-}" ]]; then
    # Container builds include the controller driver in one merged overlay.
    moboterra_setups+=("$MOBOTERRA_ROS_OVERLAY/local_setup.bash")
else
    moboterra_setups+=("$MOBOTERRA_PROJECT_ROOT/install/local_setup.bash"
        "$MOBOTERRA_PROJECT_ROOT/install-controller-types/local_setup.bash")
fi
for moboterra_setup in "${moboterra_setups[@]}"; do
    if [[ ! -r "$moboterra_setup" ]]; then
        echo "Missing ROS setup: $moboterra_setup" >&2
        return 1
    fi
    source "$moboterra_setup" || return
done
source "$MOBOTERRA_PROJECT_ROOT/config/raspberry-pi/ros.env" || return
unset moboterra_setup moboterra_setups
