#!/usr/bin/env bash
# Host-only prerequisites. All ROS, ROMB and RViz dependencies stay in Docker.
set -euo pipefail
die() { echo "ERROR: $*" >&2; exit 1; }
[[ "$EUID" -eq 0 ]] || die 'Run through bash install.sh so sudo can install host prerequisites.'
source /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] || die 'Ubuntu 24.04 is required.'
host_arch="$(dpkg --print-architecture)"
case "$host_arch" in arm64|amd64) ;; *) die 'Supported architectures: arm64 and amd64.' ;; esac
installed() { dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -q 'install ok installed'; }
apt_install() {
    DEBIAN_FRONTEND=noninteractive apt-get -o Acquire::Retries=3 install -y "$@"
}

host_packages=()
for executable in python3 ip candump udevadm modprobe curl usbreset; do
    case "$executable" in
        python3) package=python3 ;; ip) package=iproute2 ;; candump) package=can-utils ;;
        udevadm) package=udev ;; modprobe) package=kmod ;; curl) package=curl ;;
        usbreset) package=usbutils ;;
    esac
    command -v "$executable" >/dev/null 2>&1 || host_packages+=("$package")
done
if ! python3 -c 'import yaml' >/dev/null 2>&1; then host_packages+=(python3-yaml); fi
[[ -r /etc/ssl/certs/ca-certificates.crt ]] || host_packages+=(ca-certificates)
if (( ${#host_packages[@]} )); then
    apt-get -o Acquire::Retries=3 update
    apt_install "${host_packages[@]}"
fi

# Preserve an existing distro Docker/containerd installation. Fresh hosts use
# Docker's official signed Ubuntu repository; never run a downloaded shell script.
if installed docker.io || { ! command -v docker >/dev/null 2>&1 && { installed containerd || installed runc; }; }; then
    docker_packages=()
    command -v docker >/dev/null 2>&1 || docker_packages+=(docker.io)
    docker compose version >/dev/null 2>&1 || docker_packages+=(docker-compose-v2)
    docker buildx version >/dev/null 2>&1 || docker_packages+=(docker-buildx)
    if (( ${#docker_packages[@]} )); then
        apt-get -o Acquire::Retries=3 update
        apt_install "${docker_packages[@]}"
    fi
elif installed docker-ce || ! command -v docker >/dev/null 2>&1; then
    docker_packages=()
    command -v docker >/dev/null 2>&1 || docker_packages+=(docker-ce docker-ce-cli containerd.io)
    docker compose version >/dev/null 2>&1 || docker_packages+=(docker-compose-plugin)
    docker buildx version >/dev/null 2>&1 || docker_packages+=(docker-buildx-plugin)
    if (( ${#docker_packages[@]} )); then
        if ! grep -Rqs 'download.docker.com/linux/ubuntu' /etc/apt/sources.list.d /etc/apt/sources.list; then
            apt-get -o Acquire::Retries=3 update
            apt_install ca-certificates curl
            install -d -m 0755 /etc/apt/keyrings
            curl --fail --show-error --silent --location --retry 3 \
                https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
            chmod 0644 /etc/apt/keyrings/docker.asc
            cat > /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${UBUNTU_CODENAME:-$VERSION_CODENAME}
Components: stable
Architectures: $host_arch
Signed-By: /etc/apt/keyrings/docker.asc
EOF
        fi
        apt-get -o Acquire::Retries=3 update
        apt_install "${docker_packages[@]}"
    fi
else
    docker compose version >/dev/null 2>&1 && docker buildx version >/dev/null 2>&1 ||
        die 'Unsupported Docker installation. Service personnel must identify its package source before migration.'
fi

systemctl enable --now docker.service
docker compose version >/dev/null
docker buildx version >/dev/null
docker info >/dev/null
# Standard SocketCAN support; the actual adapter driver remains hardware-specific.
if [[ "${MOBOTERRA_SOFTWARE_ONLY:-0}" != 1 ]]; then
    modprobe can
    modprobe can_raw
fi
