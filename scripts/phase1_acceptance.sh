#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
exec python3 scripts/phase1_acceptance.py "$@"
