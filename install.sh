#!/usr/bin/env bash
# First-time entry point: install host prerequisites and build the Docker images.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if (( $# == 0 )); then
    set -- --software-only
fi
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    exec /bin/bash "${repo_root}/deploy/install.sh" "$@"
fi
if [[ "${EUID}" -ne 0 ]]; then
    exec sudo /bin/bash "${repo_root}/deploy/install.sh" "$@"
fi
exec /bin/bash "${repo_root}/deploy/install.sh" "$@"
