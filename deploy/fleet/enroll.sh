#!/usr/bin/env bash
# Install the fleet updater on this host and stage its approved release.
set -euo pipefail

repository=https://github.com/moboticde/MoboTerra.git
image_repository=ghcr.io/moboticde/moboterra
bootstrap_commit=8a68bb6fd416114c75ef5d2e7e89646e4870e28b
service=moboterra-current-stack.service
config=/etc/moboterra-fleet.json

fail() {
    printf 'moboterra enrollment: %s\n' "$*" >&2
    exit 1
}

robot=${1:-}
mode=${2:-}
case "$robot" in
    MoboTerra_1|MoboTerra_2|MoboTerra_3) ;;
    *) fail 'Usage: bash enroll.sh MoboTerra_1 [--install-docker]' ;;
esac
[[ $# -le 2 && ( -z "$mode" || "$mode" == --install-docker ) ]] ||
    fail 'The optional argument is --install-docker.'
(( EUID != 0 )) || fail 'Run as the account that owns the robot checkout. You will be prompted for sudo.'
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] ||
    fail 'Ubuntu 24.04 is required by the MoboTerra installer.'
case "$(dpkg --print-architecture)" in
    amd64|arm64) ;;
    *) fail 'This host needs a 64-bit AMD64 or ARM64 Ubuntu install.' ;;
esac

checkout_owner=$(id -un)
sudo -v
if sudo test -f "$config"; then
    configured_robot=$(sudo python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["robot"])' "$config")
    [[ "$configured_robot" == "$robot" ]] ||
        fail "This PC is enrolled as $configured_robot. Verify the physical robot and its enrollment before changing it."
fi

load_state=$(systemctl show "$service" -p LoadState --value)
if [[ "$load_state" == loaded ]]; then
    control=$(readlink -f /usr/local/bin/moboterra-ctl)
    checkout=$(dirname "$control")/../..
    cd "$checkout"
    [[ "$(pwd -P)/deploy/host/moboterra_ctl.py" == "$control" &&
       -d .git && -f runtime/docker-stack.env && "$(stat -c %U .)" == "$checkout_owner" ]] ||
        fail 'The installed Docker stack needs a Git checkout owned by this account.'
    if [[ "$mode" == --install-docker ]]; then
        printf '%s\n' 'Docker deployment already exists. Enrolling that checkout.'
    fi
elif [[ "$load_state" == not-found ]]; then
    [[ "$mode" == --install-docker ]] ||
        fail 'Docker deployment is missing. Stop native ROS in maintenance and rerun with --install-docker.'
    if systemctl is-active --quiet moboterra-supervisor.service ||
        pgrep -f '[r]os2.*launch.*(moboterra|mobotic_bringup)' >/dev/null; then
        fail 'Stop the existing MoboTerra ROS launch before installing Docker.'
    fi
    checkout="$HOME/MoboTerra-docker"
    if [[ ! -e "$checkout" ]]; then
        GIT_TERMINAL_PROMPT=0 git clone --branch main "$repository" "$checkout"
        git -C "$checkout" switch --detach "$bootstrap_commit"
    fi
    cd "$checkout"
    [[ -d .git && "$(stat -c %U .)" == "$checkout_owner" &&
       "$(git rev-parse HEAD)" == "$bootstrap_commit" && -z "$(git status --porcelain)" ]] ||
        fail "Review the existing checkout at $checkout before retrying."
    image=$(curl -fsSL --max-time 25 \
        "https://github.com/moboticde/MoboTerra/releases/download/docker-$bootstrap_commit/release.json" |
        python3 -c 'import json, re, sys
record = json.load(sys.stdin)
image = record["image"]
if record["commit"] != sys.argv[1] or not re.fullmatch(r"ghcr\.io/moboticde/moboterra@sha256:[0-9a-f]{64}", image):
    raise SystemExit("Invalid bootstrap image release record")
print(image)' "$bootstrap_commit")
    bash ./install.sh --no-start --image "$image"
else
    fail "Check the current Docker service before enrolling. Its load state is $load_state."
fi

checkout=$(pwd -P)
GIT_TERMINAL_PROMPT=0 git fetch --no-tags origin refs/heads/main:refs/remotes/origin/main
git merge-base --is-ancestor "$bootstrap_commit" origin/main ||
    fail 'The installed checkout origin must contain the MoboTerra fleet release.'
files=$(mktemp -d)
trap 'rm -rf -- "$files"' EXIT
git archive "$bootstrap_commit" deploy/fleet | tar -x -C "$files"

sudo install -m 0755 "$files/deploy/fleet/moboterra_fleet.py" /usr/local/sbin/moboterra-fleet
sudo install -m 0644 "$files"/deploy/fleet/moboterra-fleet-stage.{service,timer} /etc/systemd/system/
python3 -c 'import json, sys; print(json.dumps(dict(zip(
    ("checkout", "owner", "robot", "image_repository"), sys.argv[1:])), indent=2))' \
    "$checkout" "$checkout_owner" "$robot" "$image_repository" |
    sudo install -m 0600 /dev/stdin "$config"
sudo systemctl daemon-reload
sudo systemctl enable --now moboterra-fleet-stage.timer
sudo systemctl start moboterra-fleet-stage.service
sudo /usr/local/sbin/moboterra-fleet status
