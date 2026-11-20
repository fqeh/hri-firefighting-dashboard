#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
mkdir -p results
exec python3 scripts/smoke_test.py
