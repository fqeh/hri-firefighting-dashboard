#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
if [[ ! -f .runtime/bridge_ws/install/hri_bridge_revision.txt ]] || \
   [[ "$(cat .runtime/bridge_ws/install/hri_bridge_revision.txt)" != 95e6fdea72fb57ef55a4136292dfa62370ab3fb9 ]]; then
  bash scripts/build_bridge.sh
fi
source .runtime/bridge_ws/install/local_setup.bash
cd ros2_ws
set -u
colcon build --symlink-install --executor sequential --event-handlers console_direct+
