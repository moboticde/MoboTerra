#!/usr/bin/env bash
set -euo pipefail
project_dir=/home/mobotic/Downloads/MoboTerra-main
candidate=/home/mobotic/Downloads/MoboTerra-main/config/raspberry-pi/platform-front-right-rear-left.yaml
stamp=$(date +%Y%m%d_%H%M%S)
export PYTHONPATH="${project_dir}/src/mobotic_config"
python3 -m mobotic_config.configuration --platform-config "${candidate}"
cp -a /etc/moboterra/platform.yaml "/etc/moboterra/platform.yaml.before-project-start-${stamp}"
cp -a /etc/moboterra/moboterra.env "/etc/moboterra/moboterra.env.before-project-start-${stamp}"
install -m 0644 "${candidate}" /etc/moboterra/platform.yaml
cd "${project_dir}"
bash ./deploy/install.sh --no-start --platform-serial 0040001D5847570D20343432 --battery-serial 001B00305346571320373231 --ros-domain-id 42
