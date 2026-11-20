#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s ros2_ws/src/hri_sim/test -v
python3 -m unittest discover -s ros2_ws/src/hri_dashboard/test -v
PYTHONPATH="$PWD/ros2_ws/src/hri_bringup:$PWD/ros2_ws/src/hri_reasoning${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest discover -s ros2_ws/src/hri_bringup/test -v
python3 -m compileall -q ros2_ws/src evaluation
if [[ -d dashboard/node_modules ]]; then
  npm --prefix dashboard run build
fi
