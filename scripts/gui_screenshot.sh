#!/usr/bin/env bash
# Capture the actual X11 desktop, including Gazebo's native rendered window.
set -euo pipefail
export DISPLAY="${DISPLAY:-:99}"
output="${1:-/work/results/gui/desktop.png}"
mkdir -p "$(dirname "$output")"
xdpyinfo -display "$DISPLAY" >/dev/null
if ! xdotool search --onlyvisible --name 'Gazebo' >/dev/null; then
  echo 'No visible Gazebo window exists; refusing to label an empty desktop as a GUI capture.' >&2
  exit 1
fi
scrot --overwrite "$output"
echo "$output"
