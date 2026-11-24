# Fieldwatch local supervisory dashboard

This React/TypeScript console displays messages received from the live ROS 2 system. There is no demo-data mode. An offline gateway or missing sensor leaves the corresponding panel unavailable; values become stale after five wall-clock seconds. The camera shows RGB pixels rendered by Gazebo. The route graph is explicitly a 2D diagnostic view, not a replacement simulator. Use the native Gazebo client for an interactive 3D world view.

## Run

Requires Node.js 22.12+ and a built ROS workspace. The frontend runs on the host; the backend runs alongside ROS, including inside the project container. Expose container port 8080 to the host loopback interface.

For the complete dashboard, with the simulator and ROS gateway already running, start both the session supervisor and frontend from the project root:

```sh
bash scripts/run_dashboard.sh
```

This enables scenario switching, experiment controls, and result links in addition to live telemetry. The session supervisor is separate from the ROS gateway and does not provide telemetry. If launching processes manually, run `python3 scripts/dashboard_session.py` from the project root as well as Vite; without it, session-dependent controls remain unavailable.

For telemetry-only manual startup:

```sh
# In a sourced ROS environment (or enable the backend in the project launch):
ros2 run hri_dashboard dashboard_server --ros-args -p use_sim_time:=true

# In a second terminal at the project root:
cd dashboard
npm ci
npm run dev
# Open http://127.0.0.1:5173
```

The Vite development server proxies `/api` to `127.0.0.1:8080`. This is a local research console: its gateway is not an Internet service. Bind Docker's published port to `127.0.0.1`. The ROS backend defaults to `0.0.0.0` inside its container; a native host launch can set `-p host:=127.0.0.1`.

```sh
npm run build
# Gateway tests from project root, no ROS install required:
python3 -m unittest discover -s ros2_ws/src/hri_dashboard/test -v
```

The presentation UI (single screen, browser-rendered 3D world, layers, operator escalation) is documented in [docs/DEMO_DASHBOARD.md](../docs/DEMO_DASHBOARD.md).

## Telemetry contract

| Endpoint | Content |
| --- | --- |
| `GET /api/health` | Gateway identity; does not assert simulator health |
| `GET /api/state` | `connected`, `updated_at`, topic data with wall-clock age/staleness, bounded timeline |
| `GET /api/events` | Server-sent events carrying the same state at 2 Hz |
| `GET /api/camera.png` | Latest genuine ROS image encoded as PNG; HTTP 503 if missing or stale |
| `POST /api/control` | Validated JSON command queued for `/hri/control`; HTTP 202 does not claim execution |
| `GET /api/session/scene` | (session supervisor) Static world geometry of the active scenario for the web 3D view: SDF primitives with composed poses, entity models and spawn poses, logical-camera parameters, hazard templates. No incident truth. |

If `/api/events` opens but delivers nothing (as through a Cloudflare quick tunnel), the UI polls `/api/state` every 0.5 s instead. `send_operator` (no fields) records that the human operator dispatched a rescue after `/hri/escalation` reported no escape route.

Commands are `pause`, `resume`, `inject` (optional `hazard_id`), `waypoint` (required finite numeric `x`, `y` between -100 and 100 metres), `cancel_waypoint`, and `acknowledge_warning` (required nonempty `hazard_id`, at most 128 characters). Waypoints require the Nav2 launch option. A resume command remains possible after a previously observed simulation pauses its clock. Scenario events and `/hri/navigation` provide execution results. A reproducible reset requires restarting the complete simulation with the desired seed.

The prominent robot stop control sends `{"command":"emergency_stop","enabled":true}`; explicit release sends the same command with `enabled:false`. Only a JSON boolean is accepted. These requests take a priority queue to the real `/robot/emergency_stop` `std_srvs/SetBool` service, including while the simulator is paused or its telemetry is stale. The gateway's steady-time timer does not depend on simulation-clock advancement.

HTTP 202 means queued. The HTTP snapshot reports the actual asynchronous service outcome under `topics["/dashboard/emergency_stop_result"].data` (`requested_enabled`, `state`, `success`, `message`), independently of the safety guard's reliable, transient-local `/robot/emergency_stop_active` Bool heartbeat. The UI shows a confirmed latch state only after a fresh heartbeat; absent, disconnected, or stale state is unknown. Service responses never create or alter latch telemetry. Service errors, unavailability, and five-second response timeouts remain explicit. The UI allows a stop attempt even when no simulator telemetry is available, and enables release only for a currently confirmed active latch. Physical stopping is validated by the root project's Gazebo acceptance test.

Subscriptions: `/clock`, `/robot/odom`, `/human/odom`, `/uav/odom`, `/robot/scan`, `/robot/camera/image`, and the public `/hri/{observations,human_intent,belief,warning,routes,status,events,navigation,operator_belief,operator_warning,operator_routes,uav_status}` JSON topics. The gateway does not subscribe to `/evaluation/truth` or `/evaluation/human_belief`.

Operator advisories form a separate loop from the simulated responder's belief model. The operator panel shows the latest inferred belief and observed-hazard route advice. Acknowledgment is an observable operator report; the UI does not claim access to the operator's true belief or silently change destinations. A recommended waypoint can fill the coordinate form; the operator cancels the active goal and explicitly sends the revised waypoint. Cancellation is confirmed only by the navigation node's actual action result.

The gateway retains operator posteriors in `/hri/operator_belief` topic data under `beliefs[hazard_id]`, with per-hazard update age and staleness. The warning panel selects its posterior and reported acknowledgment by the displayed warning's hazard ID, so a later gas update cannot replace a fire warning's belief. The original latest-message fields remain available for compatibility.

Warning panels identify historical messages and their simulation timestamps. The hazard map retains previous detections by ID with per-hazard source, last-seen simulation stamp, and wall-clock age; a hazard disappearing from a camera frustum is never displayed as cleared. The UAV mission panel uses `/hri/uav_status` from the mission monitor: `state`, `mission`, `constraint`, `position`, `pose_stamp`, `detection_stamp`, `pose_fresh`, and `sensor_fresh`. It displays the reported mission and constraint, actual source timestamps, and whether the monitor is observing, awaiting its sensor, or reporting stale telemetry. A stale monitor message is marked stale independently; no mission state is invented from position alone.

The image encoder supports `rgb8`, `bgr8`, `rgba8`, `bgra8`, and `mono8`, respects ROS row stride, and explicitly reports unsupported encodings. Lidar infinite returns are encoded as absent measurements, not invented distances.

## Dependencies and validation

Runtime frontend dependencies are MIT-licensed React, React DOM and three.js; development tools are TypeScript (Apache-2.0) and Vite (MIT). Versions are pinned in `package.json` and the exact dependency tree in `package-lock.json`. See [React documentation](https://react.dev/) and [Vite's installation requirements](https://vite.dev/guide/). The backend uses Python's standard library and ROS 2 packages supplied by the project environment. No paid APIs are required.

Unit tests cover real HTTP command validation, absent and stale telemetry, camera expiration, channel conversion, and padded image rows. Those tests are independent of integrated Gazebo/ROS validation; see the root validation record for actual runtime evidence.

Validation on 2026-11-16: the React/TypeScript production build passed; selected `colcon` testing collected 13 gateway tests, all passed with zero errors or skips. Coverage includes separate fire/gas posteriors, per-hazard freshness, acknowledgment isolation, strict stop/release boolean validation, service rejection/error/timeout handling, and independence of service results from confirmed latch telemetry. Results are recorded in `ros2_ws/build/hri_dashboard/pytest.xml`. Browser visual QA was not performed. Integrated simulator verification is recorded separately by the root project.
