#!/usr/bin/env bash
# Native host: durable session controls + React UI. ROS gateway lives in Docker.
set -Eeuo pipefail
cd "$(dirname "$0")/.."
if [[ ! -d dashboard/node_modules ]]; then
  echo 'Install frontend dependencies first: npm --prefix dashboard ci' >&2
  exit 2
fi
pids=()
cleanup() { for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
python3 scripts/dashboard_session.py &
pids+=("$!")
npm --prefix dashboard run dev -- --host 127.0.0.1 &
pids+=("$!")
echo 'Dashboard: http://127.0.0.1:5173'
echo 'Choose a scenario, load it, then Resume simulation. Native Gazebo: http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale'
# macOS /bin/bash does not have wait -n.
while kill -0 "${pids[0]}" 2>/dev/null && kill -0 "${pids[1]}" 2>/dev/null; do sleep 1; done
exit 1
