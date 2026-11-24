# Single-screen demo dashboard

The presentation build replaces the tabbed console (Live / Details / Experiment) with one GIS-style screen. The 3D world is drawn by the browser from Gazebo's own world file and live ROS telemetry. Gazebo still owns all physics and sensing; the browser only renders what the simulator and the reasoning nodes publish.

```
┌ top bar: scenario · policy · ▶ Resume / ❚❚ Pause · Reset run · sim clock · connection ┐
├ rail ┬ sidebar (one section) ┬──────────── world view ────────────┬ right panel ┬ rail ┤
│ Scen │ Scenario / Layers /   │ 3D web view (three.js)  |  Gazebo  │ Explain or  │ Expl │
│ Lay  │ Robot / Experiment    │ window (noVNC) · camera controls · │ Log         │ Log  │
│ Rob  │                       │ status bar (cursor, pitch, sim     │             │      │
│ Exp  │                       │ time, real-time factor)            │             │      │
└──────┴───────────────────────┴────────────────────────────────────┴─────────────┴──────┘
```

Clicking the open rail tab collapses its panel, so the map can take almost the whole screen.

## Running it

```sh
bash scripts/run_dashboard.sh          # session supervisor (8081) + Vite dev server (5173)
# open http://127.0.0.1:5173
```

The simulator container must be running (`docker compose -f compose.yaml -f compose.gui.yaml --profile gui up -d gui`, or load a scenario from the dashboard).

Optional host settings in the git-ignored `.env` at the repository root:

| Variable | Default | Effect |
| --- | --- | --- |
| `HRI_GUI_SIZE` | `1280x800` | Virtual desktop of the Gazebo window. `960x600` lowers its software-rendering cost. |
| `HRI_CPU_SHARES` | `1024` | Docker CPU weight of the `gui` container (`compose.gui.yaml`). A higher value gives the simulation priority over other loads on a shared host; it changes nothing when the host is idle. |

## Sections

**Scenario.** Configuration only: nothing reaches the simulator until **Load scenario & policy**.
- *Demo presets.* **Crew knows** (forest case E) and **Crew unaware** (case F) fill in the form: B4, seed 7, crew sight radius 1 m, no map glances, no decision lapses.
- *Scenario, case and warning policy.* Policies are labelled like the cases, for example `B4 · Belief-aware (ours)`.
- *More settings.* Decision engine, seed, manual test incidents and the crew parameters.
- *Load state.* A box says whether the form matches the configuration that is running (`● Not loaded yet` or `✓ This configuration is the one running`).
- *Test incident.* A separate "during the run" group. Each incident exists once per run and several can coexist. Options show whether an incident is scheduled ("also appears on its own") or already in the simulation (with its time), and the latter are disabled.

**Layers.** Cards with visibility and opacity, remembered per browser:

| Layer | Source | Content |
| --- | --- | --- |
| Gazebo world | simulation | Terrain, trails, buildings and agents from the SDF world and live poses |
| Trees | simulation | Forest canopy and undergrowth (forest only); hide it to see the trails |
| What the UAV / robot sees | observation | Sensor coverage, observed hazards, observed smoke, trail states (clear, blocked, unknown) |
| What the crew / operator believes | inference | Bayesian ToM estimate drawn as a floating map above the trails; dashed line: the route being taken; `≠`/`=` marker on the trail in focus |
| Recommended route | inference | Risk-weighted route from the planner |
| Fire model (truth) | simulation truth | Fire envelope from the research fire model; off by default, never an inference input |
| Labels | — | Names of agents, hazards and trail nodes |

**Robot.** Status of the UAV (forest) or ground robot (oil). In the oil facility it also holds operator route approval, the emergency stop, the manual navigation goal and the equipment registry.

**Experiment.** Fixed-duration trials with logs and optional evaluation-truth counters. A trial uses the configuration in the Scenario form.

**Explain** (right). The panels run top to bottom along the system's pipeline:
1. The UAV or robot camera.
2. **Lookout status:** an explicit state (Monitoring, Hazard seen, False belief detected, Warning sent, No escape route) and the four steps Observe (sensors) → Infer (W2: ToM + HMM) → Decide (W3: MDP) → Warn (W1·D2: trust), each with its live value.
3. The pending warning, if any, with Acknowledge and Dismiss.
4. *What the crew believes:* the HMM posterior over four hypotheses for the trail in focus.
5. *Should the lookout speak?:* P(false belief) against the break-even threshold, the expected loss of each action, and what B1–B4 would do at the same moment.

**Log** (right). Warning history and the live event log.

## World view

**3D web view** (default). three.js renders the scene with the viewer's GPU, so it stays smooth even though the server renders Gazebo in software.
- *Controls:* drag to orbit, right-drag to pan, scroll to zoom. The buttons are reset view, top-down, follow the crew (or robot) and full screen.
- *Agents:* the crew, UAV and robot are drawn at their odometry with smoothing. Before the first odometry (a paused run at t = 0) they are drawn at their spawn pose from the world file, labelled `· start`.
- *Hazards:* only observed hazards are drawn. A hazard nobody has seen does not appear, which keeps the view honest about what the robot knows.

**Gazebo window.** Gazebo's own GUI over noVNC (`/gazebo`, proxied by Vite). It is rendered on the server CPU.
- While the web view is shown, the dashboard pauses this window (`SIGSTOP` on `gz sim -g`) once per run. Physics, sensors and ROS keep running, and that CPU goes back to the simulation.
- Switching to the Gazebo tab resumes it. These implicit toggles never surface errors.
- The explicit Pause/Resume button in the Gazebo tab reports its outcome.

## Operator escalation (forest)

When every escape route to the safety zone is blocked, warning the crew is no longer enough.
- The rescue planner publishes `/hri/escalation` with `active: true` and the blocked edges.
- The dashboard shows **🚨 No escape route · Send operator** over the map and at the top of Explain, and Lookout status becomes *No escape route*.
- **Send operator** posts `{"command":"send_operator"}`. The dispatch time is logged (`operator_dispatched`); the rescue itself is not simulated.

See [FOREST_SCENARIO.md](FOREST_SCENARIO.md#presentation-build-four-escape-routes-demo-incidents-and-escalation).

## Sharing with remote teammates

```sh
setsid nohup bash scripts/run_public.sh > results/public-host.log 2>&1 &
grep -m1 "PUBLIC DASHBOARD" results/public-host.log      # https://<random>.trycloudflare.com
bash scripts/run_public.sh rebuild                        # after code changes; keeps the same URL
```

How it works:
- `run_public.sh` builds the UI and serves it with `vite preview` on `127.0.0.1:4173`, with the same `/api` and `/gazebo` proxies as the dev server.
- A Cloudflare quick tunnel (`cloudflared`, found on `PATH` or in `~/bin`) publishes it.
- Quick tunnels do not deliver server-sent events. When `/api/events` opens but delivers nothing, the UI falls back to polling `/api/state` every 0.5 s. Locally it keeps using SSE.

**The public URL has no authentication.** Anyone holding it can load scenarios, inject incidents and press the robot emergency stop, and remote viewers share the same simulation. The URL changes whenever the tunnel restarts.

## Implementation notes

| Piece | Location |
| --- | --- |
| World geometry export | `scripts/scene_export.py`, served as `GET /api/session/scene`. It turns the active SDF world (built with the same `prepare_world` as the launch) into boxes, cylinders, spheres, ellipsoids and OBJ meshes with composed poses, and adds entity models in their own frame, spawn poses, logical-camera parameters and hazard templates. Runtime-spawned hazards are not in the world file, so the export carries no incident truth. |
| Renderer | `dashboard/src/world3d.ts` (Z-up, matching Gazebo): layer groups, signature-based redraws, entity smoothing, the belief "second storey", sensor frustums and wedges. |
| Telemetry → view | `liveView()` in `dashboard/src/scene3d.tsx`, a pure and unit-tested projection of the ROS topics. |
| Status pipeline, belief panel, escalation | `dashboard/src/insight.tsx` |
| Supervisor additions | `scripts/dashboard_session.py`: scene endpoint, `view` command (Gazebo window pause), queuing of quick toggles behind each other instead of rejecting them, and `manual_only` in the public incident catalogue (no positions or schedule times). |
| Figures for slides | Open `/?capture`: `window.fieldwatchCapture.{focus, entity, layer}` place the camera and toggle layers from a script. It does nothing without the query parameter. |

Tests: `npm --prefix dashboard test` (37 tests), `python3 -m unittest tests.test_scene_export tests.test_dashboard_session`, and the ROS package tests listed in the root README.
