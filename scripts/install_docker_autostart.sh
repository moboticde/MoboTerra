#!/usr/bin/env bash
# Compatibility entry point; use the complete standard installer.
set -euo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/.." && pwd)"
if [[ "$(uname -s)" != Linux ]]; then
  echo 'Run this installer on the Ubuntu robot PC with Docker Engine (SocketCAN requires Linux).' >&2
  exit 1
fi
exec /bin/bash "$PROJECT_ROOT/install.sh" "$@"
