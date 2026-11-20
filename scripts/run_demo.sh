#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
scenario="${1:-forest}"
if [[ $# -gt 0 ]]; then shift; fi
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
launch_args=()
for arg in "$@"; do
  case "$arg" in forest_case:=|oil_case:=|scenario_config:=|warning_config:=|response_config:=|q_policy:=) continue;; esac
  launch_args+=("$arg")
done
exec ros2 launch hri_bringup demo.launch.py "scenario:=$scenario" "${launch_args[@]}"
