#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source /opt/moboterra/setup.bash
set -u

exec "$@"

