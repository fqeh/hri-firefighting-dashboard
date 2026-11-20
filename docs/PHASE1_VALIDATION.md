# Phase 1 acceptance — passed 2026-11-16

The real Gazebo desktop opens through the Mac's local browser, renders the robot in a forest world, and runs the waypoint mission. Fresh-world tests separately verify physical movement, telemetry, TF, emergency stop, command timeout and obstacle contact. Existing HRI functionality and original experiment logs were retained.

## Tested platform

| Component | Verified value |
| --- | --- |
| Host | macOS 15.3.2 (24D81), Apple M3 ARM64, Metal capable |
| Docker | 29.7.2; 8 virtual CPUs; approximately 4 GB RAM |
| Linux runtime | Ubuntu 24.04.5 ARM64 |
| Robotics | ROS 2 Jazzy; Gazebo Harmonic 8.15.0; ros_gz 1.0.24 |
| Dependencies | rclpy 7.1.12; tf2_ros 0.36.23; colcon-core 0.21.3; Nav2 1.3.13 |
| Graphics | Mesa 25.2.8 llvmpipe, software OpenGL 4.5, Ogre2 |
| Browser test | Playwright 1.65.1; Chromium 159.0.8154.6 |

Jazzy/Harmonic is a supported pairing in the [official Gazebo matrix](https://gazebosim.org/docs/harmonic/ros_installation/). Native ROS/Gazebo is not installed on this Mac. The GPU's Metal capability does not imply Docker hardware acceleration. The native Ubuntu installer was syntax checked but not executed on the host. The Docker base image is digest pinned; apt packages may receive patch updates. [Environment output](../results/phase1-environment.log) and the [final runtime/source manifest](../results/phase1-runtime-manifest.json) record actual versions and hashes.

## Passed checks

- **GUI:** actual Gazebo minimal and forest scenes visibly render the robot, ground/terrain, obstacles and lighting. [Browser screenshot](../results/phase1-browser-verified/gazebo-browser.png), [minimal desktop](../results/gui/minimal-desktop.png), [GUI evidence](../results/gui/validation.json). noVNC carries the native desktop; it does not implement replacement physics.
- **Telemetry:** advancing `/clock`, wheel odometry, independent Gazebo pose, lidar, IMU and 320×240 RGB images. Final run received 565 physical poses, 831 odometry messages, 283 scans, 1,414 IMU samples and 142 images. Counts are not claimed as measured topic rates.
- **TF/timing:** `odom → base_footprint → base_link → laser_frame / camera_optical_frame`; checked sensor offsets, frame names and simulation timestamps. The safety watchdog deliberately uses a steady wall clock to operate during simulation pauses.
- **Motion:** a 0.35 m/s request produced **1.0493 m** physical displacement; straight-line wheel odometry agreed. Front lidar changed from **4.3203 to 3.2709 m**.
- **Collision:** robot model x stopped at **4.100001 m** against the obstacle face x=4.5 m, with robot front offset 0.4 m. Fresh positive drive commands continued for two simulated seconds with **0 m** further motion. Collisions remain enabled.
- **Stop:** stopping distance **0.08094 m**, subsequent drift **0 m** under continued raw commands. Four repeated release checks reject buffered requests. Command timeout also passed. [Foundation result](../results/phase1-acceptance/foundation.json).
- **Mission:** both `(1,0)` and `(1,1)` completed. Independent physical closest errors **0.0669 m / 0.2651 m**, within the stated **0.35 m** tolerance. Wheel odometry path length **1.7948 m**. Turning error exists, so odometry alone cannot pass this test. [Mission result](../results/phase1-acceptance/mission.json), [physical trajectory CSV](../results/phase1-acceptance/mission_physical.csv), [odometry CSV](../results/phase1-acceptance/mission_odom.csv).
- **Browser:** actual stop/release clicks confirmed real ROS service results and latch telemetry, including while simulation time was paused. Resume advanced the clock from **253.340 to 253.381 s**. Camera PNG and desktop canvas passed; zero browser page exceptions. [Browser result](../results/phase1-browser-verified/browser-check.json), [dashboard screenshot](../results/phase1-browser-verified/dashboard-stop.png).
- **Build/tests:** four ROS packages built; **65 reported checks passed, zero errors/failures/skips** (64 Python cases plus CTest suite wrapper). Frontend production build passed. [Build](../results/phase1-build.log), [tests](../results/phase1-colcon-tests-final.log), [frontend](../results/phase1-dashboard-build.log).
- **Logs:** final headless launches and shutdowns have zero critical error matches. The final live GUI/server audit also found zero. Nonfatal SDF extension, Qt layout, OBJ material and antialiasing fallback warnings remain recorded. [Acceptance report](../results/phase1-acceptance/acceptance.json), [GUI audit](../results/gui/final/log-audit.json).

## Exact commands executed

Working directory: `/Users/ali/Documents/HRI/Project`. Build/test stdout and stderr were saved in the linked logs. The acceptance JSON also records each exact ROS launch/probe command and its isolated ROS domain and Gazebo partition.

```bash
sw_vers
uname -m
system_profiler SPDisplaysDataType
docker info --format 'Server={{.ServerVersion}} OS={{.OperatingSystem}} Arch={{.Architecture}} CPUs={{.NCPU}} Memory={{.MemTotal}}'
docker compose run --rm sim bash -c 'cat /etc/os-release; gz sim --versions; dpkg-query -W ros-jazzy-ros-gz ros-jazzy-rclpy ros-jazzy-navigation2 ros-jazzy-tf2-ros python3-colcon-core; xvfb-run -a glxinfo -B'
python3 ros2_ws/src/hri_sim/scripts/generate_assets.py
docker compose run --rm sim bash scripts/build.sh
docker compose -f compose.yaml -f compose.gui.yaml --profile gui build gui
docker compose run --rm sim bash scripts/phase1_acceptance.sh --output results/phase1-acceptance
docker compose run --rm sim bash -c 'cd ros2_ws && colcon test --executor sequential --event-handlers console_direct+ && colcon test-result --verbose'
npm --prefix dashboard run build
HRI_GUI_OUTPUT=/work/results/gui/final docker compose -f compose.yaml -f compose.gui.yaml --profile gui up -d --force-recreate gui
npm --prefix dashboard run dev -- --host 127.0.0.1
npm install --prefix /tmp/hri-browser-check --no-audit --no-fund playwright
PLAYWRIGHT_BROWSERS_PATH=/tmp/hri-browser-check/browsers /tmp/hri-browser-check/node_modules/.bin/playwright install chromium
NODE_PATH=/tmp/hri-browser-check/node_modules PLAYWRIGHT_BROWSERS_PATH=/tmp/hri-browser-check/browsers node scripts/verify_browser.cjs results/phase1-browser-verified
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec gui /hri-entrypoint.sh python3 scripts/record_environment.py --output results/phase1-runtime-manifest.json
```

For reproduction, pin the temporary browser dependency to `playwright@1.65.1` and use a new acceptance output directory. See the [README](../README.md) for startup, reset and stop-service commands.

## Failures found and fixed

1. Initial physical assertions passed but the test harness delivered SIGINT twice by signaling ROS launch and its children. It now signals launch first and lets launch manage children. [Original failure](../results/phase1-attempt1/acceptance.json).
2. Stronger checks intermittently found queued velocity requests arriving after stop release. Release now replaces the volatile DDS reader and requires new messages; its queue depth is one. Four repeated live checks pass. [Original failure](../results/phase1-final/foundation.json).
3. A final zero-command publish raced rclpy's asynchronous context shutdown. The new motion nodes now retain context until explicit cleanup completes. [Original error](../results/phase1-verified/mission_simulator.log); final shutdowns pass the log audit.
4. The first browser test used an obsolete noVNC canvas selector; it was corrected for the installed version. [Original result](../results/phase1-browser/browser-check.json).
5. Sandbox restrictions initially blocked Docker, local server binding and downloads; commands were rerun with authorized access. A direct Docker exec manifest command lacked ROS environment sourcing; the documented `/hri-entrypoint.sh` command succeeds. Failed evidence remains archived.

## Blockers and limits

**No remaining blocker for Phase 1 acceptance on the tested Mac/Docker environment.** The GUI and dashboard were left running. The software stop cannot guarantee braking if its process or the simulator fails; it is not a hardware stop. Physical stop testing covers forward motion, not exhaustive angular/fault testing. Native Ubuntu/RViz and industrial GUI interaction were not newly verified. Software rendering consumes CPU and is not a real-time guarantee. This milestone does not claim HRI effectiveness, autonomous UAV flight or live advanced communication-policy integration.
