#!/usr/bin/env bash
# Expose a genuine Gazebo GUI desktop through local noVNC. The physics server
# stays on the tested headless ROS launch path in the same Gazebo partition.
set -Eeuo pipefail
cd "$(dirname "$0")/.."
scenario="${1:-forest}"
if [[ $# -gt 0 ]]; then shift; fi
case "$scenario" in minimal|forest|industrial|oil_facility) ;; *) echo 'Expected minimal, forest, industrial or oil_facility.' >&2; exit 2;; esac
export DISPLAY="${DISPLAY:-:99}"
export QT_QPA_PLATFORM=xcb
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
desktop_size="${HRI_GUI_SIZE:-1280x800}"
if [[ ! "$desktop_size" =~ ^[0-9]+x[0-9]+$ ]]; then echo 'HRI_GUI_SIZE must be WIDTHxHEIGHT.' >&2; exit 2; fi
output="${HRI_GUI_OUTPUT:-/work/results/gui}"
mkdir -p "$output"
for command in Xvfb xdpyinfo openbox x11vnc websockify gz scrot; do
  command -v "$command" >/dev/null || { echo "Missing $command; build the gui image target." >&2; exit 2; }
done
if [[ ! -f ros2_ws/install/setup.bash ]]; then echo 'Build the ROS workspace first: bash scripts/build.sh' >&2; exit 2; fi
pids=()
cleanup() {
  trap - EXIT INT TERM
  # Close Gazebo while its X server still exists. Stopping Xvfb first caused
  # misleading XIO fatal errors during otherwise healthy scenario restarts.
  for pid in "${gui_pid:-}" "${server_pid:-}"; do
    [[ -n "$pid" ]] || continue
    kill -INT "$pid" 2>/dev/null || true
    for ((attempt=0; attempt<80; attempt++)); do
      if ! kill -0 "$pid" 2>/dev/null; then break; fi
      sleep .1
    done
    kill -TERM "$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for ((attempt=0; attempt<30; attempt++)); do
    alive=false
    for pid in "${pids[@]}"; do if kill -0 "$pid" 2>/dev/null; then alive=true; fi; done
    "$alive" || break
    sleep .1
  done
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

Xvfb "$DISPLAY" -screen 0 "${desktop_size}x24" -nolisten tcp +extension GLX +render -noreset >"$output/xvfb.log" 2>&1 &
pids+=("$!")
for ((attempt=0; attempt<100; attempt++)); do
  if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then break; fi
  sleep .1
done
xdpyinfo -display "$DISPLAY" >/dev/null
openbox >"$output/openbox.log" 2>&1 &
pids+=("$!")
x11vnc -display "$DISPLAY" -localhost -rfbport 5900 -forever -shared -nopw -noxdamage >"$output/x11vnc.log" 2>&1 &
pids+=("$!")
websockify --web=/usr/share/novnc 0.0.0.0:6080 127.0.0.1:5900 >"$output/novnc.log" 2>&1 &
pids+=("$!")
glxinfo -B >"$output/opengl.txt" 2>&1
echo "Starting $scenario with Gazebo's native GUI; logs: $output"
bash scripts/run_demo.sh "$scenario" gui:=false dashboard:=false "log_path:=$output/events.jsonl" "$@" >"$output/simulator.log" 2>&1 &
server_pid=$!
pids+=("$server_pid")
ready=false
for ((attempt=0; attempt<40; attempt++)); do
  if ! kill -0 "$server_pid" 2>/dev/null; then echo 'ROS/Gazebo launch exited; inspect simulator.log.' >&2; exit 1; fi
  if timeout 3s gz service -l 2>/dev/null | grep -Fq "/world/$scenario/control"; then ready=true; break; fi
  sleep .25
done
if [[ "$ready" != true ]]; then echo 'Gazebo world service did not become available.' >&2; exit 1; fi
gz sim -g --gui-config /work/docker/gazebo-gui.config -v 3 >"$output/gazebo-gui.log" 2>&1 &
gui_pid=$!
pids+=("$gui_pid")
echo 'Gazebo desktop: http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale'
wait -n "${pids[@]}"
