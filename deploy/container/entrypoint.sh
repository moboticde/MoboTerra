#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source /opt/moboterra/setup.bash

exec "$@"

