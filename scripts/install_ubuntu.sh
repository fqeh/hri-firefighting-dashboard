#!/usr/bin/env bash
# Native installation only. On macOS use Docker; do not execute this script.
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 ]]; then
  echo 'This installer requires Ubuntu 24.04. Use the Docker workflow on other hosts.' >&2
  exit 2
fi
# This HTTPS mirror preserves upstream ROS signing; override for another signed
# ROS 2 Ubuntu package mirror when needed. The key remains the upstream ROS key.
ROS_APT_MIRROR="${ROS_APT_MIRROR:-https://ftp.osuosl.org/pub/ros/packages.ros.org/ros2/ubuntu}"
ROS_APT_MIRROR="${ROS_APT_MIRROR%/}"
if [[ "$ROS_APT_MIRROR" != https://* || "$ROS_APT_MIRROR" =~ [[:space:]] ]]; then
  echo 'ROS_APT_MIRROR must be an HTTPS URL without whitespace.' >&2
  exit 2
fi
sudo apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update
sudo apt-get install -y curl gnupg ca-certificates software-properties-common locales
sudo add-apt-repository -y universe
sudo locale-gen en_US.UTF-8
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
architecture="$(dpkg --print-architecture)"
printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] %s noble main\n' "$architecture" "$ROS_APT_MIRROR" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get -o Acquire::Retries=3 -o APT::Update::Error-Mode=any update
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup ros-jazzy-rviz2 ros-jazzy-ament-cmake-pytest \
  python3-colcon-common-extensions python3-pytest python3-yaml \
  libgl1-mesa-dri mesa-utils xvfb xauth
echo 'Installed. Run: bash scripts/build.sh'
