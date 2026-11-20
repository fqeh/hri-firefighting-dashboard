#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [[ -f /work/.runtime/bridge_ws/install/local_setup.bash ]]; then
  source /work/.runtime/bridge_ws/install/local_setup.bash
fi
if [[ -f /work/ros2_ws/install/setup.bash ]]; then
  source /work/ros2_ws/install/setup.bash
fi
export GZ_SIM_RESOURCE_PATH="/work/ros2_ws/src/hri_sim/models:${GZ_SIM_RESOURCE_PATH:-}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/work/results/ros_logs}"
exec "$@"
