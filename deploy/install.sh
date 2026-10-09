#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

platform_serial=""
battery_serial=""
ros_domain_id=""
no_start=0
software_only=0
reconfigure_can=0

usage() {
    cat <<'USAGE'
Usage: sudo bash ./deploy/install.sh [options]

Options:
  --platform-serial SERIAL  Configure the platform CAN adapter non-interactively.
  --battery-serial SERIAL   Configure the battery CAN adapter non-interactively.
  --ros-domain-id ID        ROS domain ID (0..232; default 42 on first install).
  --reconfigure-can         Ignore saved serials and run adapter identification again.
  --no-start                Install and build, but do not enable or start the service.
  --software-only           Install runtime, GUI and CAD tools without CAN adapters or a robot service.
  -h, --help                Show this help.
USAGE
}

while (( $# > 0 )); do
    case "$1" in
        --platform-serial) platform_serial="${2:?missing serial}"; shift 2 ;;
        --battery-serial) battery_serial="${2:?missing serial}"; shift 2 ;;
        --ros-domain-id) ros_domain_id="${2:?missing domain ID}"; shift 2 ;;
        --reconfigure-can) reconfigure_can=1; shift ;;
        --no-start) no_start=1; shift ;;
        --software-only) software_only=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

die() {
    echo "ERROR: $*" >&2
    exit 1
}

if [[ "${EUID}" -ne 0 ]]; then
    die "Run this installer with sudo"
fi
[[ -r /etc/os-release ]] || die "Cannot identify the operating system"
source /etc/os-release
[[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]] || die "Ubuntu 24.04 is required"
[[ "$(dpkg --print-architecture)" == "arm64" ]] || die "This deployment supports ARM64 only"

existing_value() {
    local key="$1" config=/etc/moboterra/moboterra.env
    [[ -r "${config}" ]] || config=/etc/moboterra/software.env
    [[ -r "${config}" ]] || return 0
    sed -n "s/^${key}=//p" "${config}" | tail -n 1
}

if (( reconfigure_can == 0 )); then
    [[ -n "${platform_serial}" ]] || platform_serial="$(existing_value PLATFORM_CAN_SERIAL)"
    [[ -n "${battery_serial}" ]] || battery_serial="$(existing_value BATTERY_CAN_SERIAL)"
fi
[[ -n "${ros_domain_id}" ]] || ros_domain_id="$(existing_value ROS_DOMAIN_ID)"
[[ -n "${ros_domain_id}" ]] || ros_domain_id=42
required_nodes="$(existing_value MOBOTERRA_REQUIRED_NODES)"
required_nodes="${required_nodes:-/moboterra/mobotic_supervisor,/moboterra/platform_diagnostics}"
can_stable_seconds="$(existing_value CAN_STABLE_SECONDS)"
can_stable_seconds="${can_stable_seconds:-3}"
can_poll_seconds="$(existing_value CAN_POLL_SECONDS)"
can_poll_seconds="${can_poll_seconds:-2}"
ros_start_timeout="$(existing_value ROS_START_TIMEOUT)"
ros_start_timeout="${ros_start_timeout:-120}"
[[ "${ros_domain_id}" =~ ^[0-9]{1,3}$ ]] || die "ROS domain ID must be numeric (0..232)"
ros_domain_id=$((10#${ros_domain_id}))
(( ros_domain_id >= 0 && ros_domain_id <= 232 )) || die "ROS domain ID must be between 0 and 232"

ensure_prerequisites() {
    local docker_missing=0 host_packages=()
    command -v docker >/dev/null 2>&1 || docker_missing=1
    if (( docker_missing == 0 )) && ! docker compose version >/dev/null 2>&1; then
        if dpkg-query -W -f='${Status}' docker-ce 2>/dev/null | grep -q 'install ok installed'; then
            host_packages+=(docker-compose-plugin)
        elif dpkg-query -W -f='${Status}' docker.io 2>/dev/null | grep -q 'install ok installed'; then
            host_packages+=(docker-compose-v2)
        else
            die "Unknown Docker installation without Compose; install its matching Compose plugin."
        fi
    fi
    command -v ip >/dev/null 2>&1 || host_packages+=(iproute2)
    command -v udevadm >/dev/null 2>&1 || host_packages+=(udev)
    command -v python3 >/dev/null 2>&1 || host_packages+=(python3)
    command -v candump >/dev/null 2>&1 || host_packages+=(can-utils)
    [[ -r /etc/ssl/certs/ca-certificates.crt ]] || host_packages+=(ca-certificates)
    if (( docker_missing == 1 )); then
        host_packages+=(docker.io docker-compose-v2)
    fi
    if (( ${#host_packages[@]} > 0 )); then
        apt-get -o Acquire::Retries=3 update
        DEBIAN_FRONTEND=noninteractive apt-get -o Acquire::Retries=3 install -y "${host_packages[@]}"
    fi
    systemctl enable --now docker.service
    docker compose version >/dev/null
}

ensure_kernel_support() {
    modprobe can
    modprobe can_raw
    if ! modprobe gs_usb; then
        echo "WARNING: gs_usb is unavailable; verify the driver required by your USB-to-CAN firmware." >&2
    fi
}

connected_serials() {
    local interface type serial
    for interface in /sys/class/net/*; do
        [[ -r "${interface}/type" ]] || continue
        read -r type <"${interface}/type"
        [[ "${type}" == "280" ]] || continue
        serial="$(udevadm info --query=property --path="${interface}" | sed -n 's/^ID_SERIAL_SHORT=//p' | head -n 1)"
        [[ -n "${serial}" ]] && printf '%s\n' "${serial}"
    done
}

identify_one() {
    local role="$1" serials=()
    [[ -t 0 ]] || die "${role} serial is required for non-interactive installation"
    echo >&2
    echo "Disconnect other USB-to-CAN adapters, connect only the ${role} adapter, then press Enter." >&2
    read -r
    udevadm settle
    mapfile -t serials < <(connected_serials | sort -u)
    (( ${#serials[@]} == 1 )) || die "Expected one serial-numbered CAN adapter, found ${#serials[@]}"
    printf '%s' "${serials[0]}"
}

ensure_prerequisites
if (( software_only == 0 )); then
    ensure_kernel_support

    [[ -n "${platform_serial}" ]] || platform_serial="$(identify_one platform)"
    if [[ -z "${battery_serial}" ]]; then
        if [[ -t 0 ]]; then
            echo
            echo "Disconnect the platform adapter before continuing."
            read -r -p "Press Enter when it is disconnected. "
        fi
        battery_serial="$(identify_one battery)"
    fi

    [[ "${platform_serial}" =~ ^[[:alnum:]_.:+-]+$ ]] || die "Invalid platform serial"
    [[ "${battery_serial}" =~ ^[[:alnum:]_.:+-]+$ ]] || die "Invalid battery serial"
    [[ "${platform_serial}" != "${battery_serial}" ]] || die "The two adapter serials must be different"

    input_gid="$(getent group input | cut -d: -f3 || true)"
    if [[ -z "${input_gid}" && -e /dev/input ]]; then
        input_gid="$(stat -c %g /dev/input)"
    fi
    [[ "${input_gid}" =~ ^[0-9]+$ ]] || die "Could not determine the host input-device group ID"
fi

commit="$(git -C "${repo_root}" rev-parse --short=12 HEAD 2>/dev/null || printf 'source')"
build_date="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
# ZIP downloads have no commit ID; each installation still needs a unique release.
version="${commit}-$(date -u +%Y%m%d%H%M%S)-$$"
runtime_image="moboterra:${version}-jazzy-arm64"
gui_image="moboterra-gui:${version}-jazzy-arm64"
tools_image="moboterra-tools:${version}-jazzy-arm64"

echo "Pulling and resolving the ROS 2 Jazzy base image..."
for attempt in 1 2 3; do
    if docker pull ros:jazzy-ros-base-noble; then break; fi
    (( attempt < 3 )) || die "ROS base image download failed after three attempts"
    sleep 2
done
ros_image="$(docker image inspect --format '{{index .RepoDigests 0}}' ros:jazzy-ros-base-noble)"
[[ -n "${ros_image}" && "${ros_image}" == *@sha256:* ]] || die "Could not resolve an immutable ROS image digest"

common_build_args=(
    --file "${repo_root}/deploy/Dockerfile"
    --build-arg "ROS_IMAGE=${ros_image}"
    --build-arg TARGETARCH=arm64
    --build-arg "VCS_REF=${commit}"
    --build-arg "BUILD_DATE=${build_date}"
)

echo "Building and testing ${runtime_image}..."
docker build "${common_build_args[@]}" --target runtime --tag "${runtime_image}" "${repo_root}"
echo "Building ${gui_image}..."
docker build "${common_build_args[@]}" --target gui --tag "${gui_image}" "${repo_root}"
echo "Building offline CAD tools ${tools_image}..."
docker build "${common_build_args[@]}" --target tools --tag "${tools_image}" "${repo_root}"
echo "Checking installed container startup and virtual feedback..."
docker run --rm --init --network none --env "ROS_DOMAIN_ID=${ros_domain_id}" \
    "${runtime_image}" python3 /opt/moboterra-checks/virtual_smoke.py
docker run --rm --network none --env QT_QPA_PLATFORM=offscreen "${gui_image}" rviz2 --help >/dev/null
docker run --rm --network none --env QT_QPA_PLATFORM=offscreen "${gui_image}" rqt_graph --help >/dev/null
docker run --rm --network none "${tools_image}" python -c 'import OCP, vtk, xacro, yaml'
echo "Checking headless CAD preview rendering..."
docker run --rm --network none "${tools_image}"

install -d -m 0755 /opt/moboterra/releases /etc/moboterra

release_dir="/opt/moboterra/releases/${version}"
[[ ! -e "${release_dir}" ]] || die "Release already exists: ${release_dir}"
staging="/opt/moboterra/releases/.staging-${version}-$$"
install -d -m 0755 "${staging}"
install -m 0644 "${script_dir}/compose.yaml" "${staging}/compose.yaml"
install -m 0755 "${script_dir}/host/moboterra_supervisor.py" "${staging}/moboterra_supervisor.py"
install -m 0755 "${script_dir}/host/moboterra_ctl.py" "${staging}/moboterra-ctl"
install -m 0755 "${script_dir}/host/moboterra-gui" "${staging}/moboterra-gui"
install -m 0755 "${script_dir}/host/moboterra-tools" "${staging}/moboterra-tools"
python3 - "${staging}/release.json" "${version}" "${runtime_image}" "${gui_image}" "${tools_image}" <<'PY'
import json
from pathlib import Path
import sys
Path(sys.argv[1]).write_text(json.dumps({
    "version": sys.argv[2],
    "runtime_image": sys.argv[3],
    "gui_image": sys.argv[4],
    "tools_image": sys.argv[5],
}, indent=2) + "\n", encoding="utf-8")
PY
mv -- "${staging}" "${release_dir}"

# Desktop and offline tooling also work before the robot service is configured.
software_tmp="$(mktemp /etc/moboterra/software.env.XXXXXX)"
cat >"${software_tmp}" <<EOF
ROS_DOMAIN_ID=${ros_domain_id}
MOBOTERRA_GUI_IMAGE=${gui_image}
MOBOTERRA_TOOLS_IMAGE=${tools_image}
EOF
chmod 0644 "${software_tmp}"
mv -- "${software_tmp}" /etc/moboterra/software.env
software_link="/opt/moboterra/.software.new.$$"
ln -s -- "${release_dir}" "${software_link}"
mv -Tf -- "${software_link}" /opt/moboterra/software
install -d -m 0755 /usr/local/bin
ln -sfn /opt/moboterra/software/moboterra-gui /usr/local/bin/moboterra-gui
ln -sfn /opt/moboterra/software/moboterra-tools /usr/local/bin/moboterra-tools
if [[ ! -e /usr/local/bin/moboterra-ctl && ! -L /usr/local/bin/moboterra-ctl ]]; then
    ln -s /opt/moboterra/software/moboterra-ctl /usr/local/bin/moboterra-ctl
fi
install -d -m 0755 /usr/local/share/applications
for desktop in "${script_dir}"/desktop/*.desktop; do
    install -m 0644 "${desktop}" /usr/local/share/applications/
done

if (( software_only == 1 )); then
    echo "Software ${version} installed and verified; no hardware service was configured or started."
    echo "Launch moboterra-gui rviz2, moboterra-gui rqt_graph, or moboterra-tools."
    exit 0
fi
install -d -m 0750 -o 10001 -g 10001 /var/lib/moboterra/ros-logs

config_backup=""
if [[ -f /etc/moboterra/moboterra.env ]]; then
    config_backup="$(mktemp)"
    cp --preserve=mode,ownership /etc/moboterra/moboterra.env "${config_backup}"
fi
config_tmp="$(mktemp /etc/moboterra/moboterra.env.XXXXXX)"
cat >"${config_tmp}" <<EOF
PLATFORM_CAN_SERIAL=${platform_serial}
BATTERY_CAN_SERIAL=${battery_serial}
CAN_BITRATE=500000
ROS_DOMAIN_ID=${ros_domain_id}
INPUT_GID=${input_gid}
MOBOTERRA_IMAGE=${runtime_image}
MOBOTERRA_GUI_IMAGE=${gui_image}
MOBOTERRA_TOOLS_IMAGE=${tools_image}
MOBOTERRA_RELEASE=${version}
MOBOTERRA_REQUIRED_NODES=${required_nodes}
CAN_STABLE_SECONDS=${can_stable_seconds}
CAN_POLL_SECONDS=${can_poll_seconds}
ROS_START_TIMEOUT=${ros_start_timeout}
EOF
chmod 0644 "${config_tmp}"
chown root:root "${config_tmp}"
mv -- "${config_tmp}" /etc/moboterra/moboterra.env

previous_target=""
if [[ -L /opt/moboterra/current ]]; then
    previous_target="$(readlink -f /opt/moboterra/current)"
fi
new_link="/opt/moboterra/.current.new.$$"
ln -s -- "${release_dir}" "${new_link}"
mv -Tf -- "${new_link}" /opt/moboterra/current
if [[ -n "${previous_target}" && "${previous_target}" == /opt/moboterra/releases/* ]]; then
    previous_link="/opt/moboterra/.previous.new.$$"
    ln -s -- "${previous_target}" "${previous_link}"
    mv -Tf -- "${previous_link}" /opt/moboterra/previous
fi

install -m 0644 "${script_dir}/systemd/moboterra-supervisor.service" /etc/systemd/system/moboterra-supervisor.service
install -m 0644 "${script_dir}/systemd/moboterra-tmpfiles.conf" /etc/tmpfiles.d/moboterra.conf
ln -sfn /opt/moboterra/current/moboterra-ctl /usr/local/bin/moboterra-ctl
systemd-tmpfiles --create /etc/tmpfiles.d/moboterra.conf
systemd-analyze verify /etc/systemd/system/moboterra-supervisor.service
systemctl daemon-reload

if (( no_start == 1 )); then
    systemctl disable --now moboterra-supervisor.service >/dev/null 2>&1 || true
    [[ -z "${config_backup}" ]] || rm -f "${config_backup}"
    echo "Installed ${version}; service start was skipped."
    exit 0
fi

if [[ -t 0 ]]; then
    echo
    read -r -p "Connect both configured CAN adapters, then press Enter to start MoboTerra. "
fi

rollback_install() {
    echo "Deployment did not become healthy; restoring the previous release." >&2
    systemctl stop moboterra-supervisor.service || true
    if [[ -n "${previous_target}" && -d "${previous_target}" ]]; then
        repair_link="/opt/moboterra/.current.rollback.$$"
        ln -s -- "${previous_target}" "${repair_link}"
        mv -Tf -- "${repair_link}" /opt/moboterra/current
        [[ -z "${config_backup}" ]] || cp --preserve=mode,ownership "${config_backup}" /etc/moboterra/moboterra.env
        systemctl start moboterra-supervisor.service || true
    else
        rm -f /opt/moboterra/current
        systemctl disable moboterra-supervisor.service || true
    fi
    [[ -z "${config_backup}" ]] || rm -f "${config_backup}"
    exit 1
}

systemctl enable --now moboterra-supervisor.service
deadline=$((SECONDS + 180))
while (( SECONDS < deadline )); do
    if [[ -r /run/moboterra-supervisor/state.json ]] \
        && grep -q '"state": "RUNNING"' /run/moboterra-supervisor/state.json; then
        [[ -z "${config_backup}" ]] || rm -f "${config_backup}"
        echo "MoboTerra ${version} is running."
        moboterra-ctl status || true
        exit 0
    fi
    if ! systemctl is-active --quiet moboterra-supervisor.service; then
        rollback_install
    fi
    sleep 2
done
rollback_install

