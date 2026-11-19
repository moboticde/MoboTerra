#!/usr/bin/env bash
# Standard deployment: host bootstrap plus complete ROS/ROMB/RViz container.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/.." && pwd)"
usage() {
    cat <<'USAGE'
Usage: bash install.sh [options]
       bash deploy/install.sh [options]

Installs missing host prerequisites automatically. Builds the supplied complete
Docker image containing ROS 2, the original ROMB solver and RViz. Installs
moboterra-ctl and enables the current stack at boot, leaving it stopped now.
No native ROS installation or separately prepared ROMB runtime is needed.

  --start              Enable boot startup and start the ROS stack now.
  --no-start           Install with boot startup disabled; leave the stack stopped.
  --software-only      Build/check image and install CLI without a robot service.
  --image IMAGE        Use a prepared complete image instead of a source build.
  --image-archive FILE Load a Docker image archive locally. Requires --image.
  --ros-domain-id ID   Set the project ROS domain (0..232).
  --owner USER         Optional runtime user; selected automatically by default.
  -h, --help           Show this help.
USAGE
}
arguments=("$@")
selected_modes=0
software_only=0
image=
image_archive=
while (( $# )); do
    case "$1" in
        --owner|--image|--image-archive|--ros-domain-id)
            [[ $# -ge 2 && -n "$2" && "$2" != --* ]] || { echo "Missing value for $1" >&2; exit 2; }
            if [[ "$1" == --ros-domain-id ]]; then
                [[ "$2" =~ ^[0-9]{1,3}$ ]] && (( 10#$2 <= 232 )) || { echo 'ROS domain must be 0..232.' >&2; exit 2; }
            fi
            case "$1" in
                --image) image="$2" ;;
                --image-archive) image_archive="$2" ;;
            esac
            shift 2 ;;
        --start|--no-start) selected_modes=$((selected_modes + 1)); shift ;;
        --software-only) selected_modes=$((selected_modes + 1)); software_only=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unsupported option: $1" >&2; usage >&2; exit 2 ;;
    esac
done
(( selected_modes <= 1 )) || { echo 'Choose only one of --start, --no-start and --software-only.' >&2; exit 2; }
if [[ -n "$image_archive" ]]; then
    [[ -n "$image" ]] || { echo '--image-archive requires --image.' >&2; exit 2; }
    [[ -f "$image_archive" ]] || { echo "Image archive does not exist: $image_archive" >&2; exit 2; }
fi
if [[ "$EUID" -ne 0 ]]; then
    exec sudo /bin/bash "$script_dir/install.sh" "${arguments[@]}"
fi
[[ "$(uname -s)" == Linux ]] || { echo 'Run on the Ubuntu robot computer.' >&2; exit 1; }
MOBOTERRA_SOFTWARE_ONLY="$software_only" /bin/bash "$script_dir/bootstrap_host.sh"
exec python3 "$script_dir/current_stack.py" install "${arguments[@]}"
