# Gazebo's native GUI in a local browser

This optional image runs the actual `gz sim -g` application on Xvfb, with Openbox,
x11vnc and noVNC transporting the X11 desktop to the Mac browser. Gazebo still
renders and controls the 3D world; there is no browser-side substitute simulator.
The original `sim` service and default Dockerfile target remain headless.

From the project root, after the normal ROS workspace build:

```bash
docker compose -f compose.yaml -f compose.gui.yaml --profile gui build gui
docker compose stop sim
docker compose -f compose.yaml -f compose.gui.yaml --profile gui up gui
```

Open **http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale** for Gazebo
and **http://127.0.0.1:8080** for the ROS dashboard. The default forest launch
starts the waypoint mission (`mission:=true`), so the robot moves after startup.
Set `HRI_GUI_MISSION=false` to leave it stationary or `HRI_GUI_DASHBOARD=false`
to omit the dashboard node. The noVNC Connect button can reconnect to the same Gazebo window. Gazebo's normal mouse
orbit, pan, zoom and entity-tree controls operate inside this desktop.

For the minimal scene, choose the scenario before starting the service:

```bash
HRI_GUI_SCENARIO=minimal docker compose -f compose.yaml -f compose.gui.yaml --profile gui up gui
```

Stop this GUI service without removing other services or containers:

```bash
docker compose -f compose.yaml -f compose.gui.yaml --profile gui stop gui
```

The default GUI service is isolated on ROS domain 46 and Gazebo partition
`hri_gui_validation`. It exposes only host loopback ports 6080 and 8080.
Do not run the `sim` service concurrently; both services bind dashboard port
8080 by default. Override `HRI_GUI_ROS_DOMAIN`, `HRI_GUI_PARTITION`,
`HRI_GUI_PORT`, `HRI_GUI_DASHBOARD_PORT` or `HRI_GUI_SIZE` (default `1280x800`)
when an integration run needs different settings. `HRI_GUI_OUTPUT` can select
a separate evidence directory inside `/work`. Do not run a second server in
the same partition. The native Ubuntu `gui:=true` workflow is unchanged and needs none of the VNC tools.

Capture the actual desktop and verify the browser transport:

```bash
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec gui \
  bash scripts/gui_screenshot.sh /work/results/gui/desktop.png
python3 scripts/gui_connection_check.py --output results/gui/connection.json
```

The screenshot script requires a visible Gazebo window; inspect the PNG to
confirm a rendered scene and robot. A successful HTTP or VNC handshake alone
does not establish that OpenGL rendered the world. The connection check proves
both the noVNC page and the WebSocket-to-VNC protocol path. Runtime graphics and
process logs are in `results/gui`, including the `glxinfo -B` renderer report.

The GUI uses Mesa software OpenGL on the existing ARM64 Linux VM. It consumes
more CPU/memory than the headless workflow, so stop it before batch experiments.
Gazebo's Ogre2 renderer needs OpenGL above 3.3; see the [official rendering
troubleshooting guide](https://gazebosim.org/docs/harmonic/troubleshooting/).
[noVNC](https://github.com/novnc/noVNC) and
[websockify](https://github.com/novnc/websockify) supply the browser transport;
their own package licenses remain in the installed Ubuntu packages.

## Watching movement from the beginning

The GUI now starts paused by default. Press the triangular Play button at the bottom left to start both missions. The teal robot follows a short two-waypoint route; the orange human proxy continues toward safety. The raised camera shows both starting positions. Both stop on completion. To replay, use `docker compose -f compose.yaml -f compose.gui.yaml --profile gui restart gui` and reconnect. The circular Gazebo Reset button does not restart ROS mission state. Use `HRI_GUI_START_PAUSED=false` for automatic start.
