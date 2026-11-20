# Phase 6 dashboard acceptance

**Status: passed, 2026-11-18.** The dashboard is connected to real Gazebo/ROS telemetry; it has no integrated synthetic-data mode. The full acceptance evidence is in [the recorded forest/oil-facility run](../results/phase6-visible/result.json) and [the control, experiment and reconnect run](../results/phase6-controls-passed/result.json).

## Verified

- The React/TypeScript suite passes: `npm test` reports 14 tests across 4 files. `npm run build` passes TypeScript and the production Vite build.
- The ROS dashboard gateway suite passes: 17 Python tests.
- The visible integrated run passed the forest movement, physical fallen-tree injection, route-blocking, alternative-route, live posterior, warning, acknowledgment, scenario-switch/new-epoch, industrial Nav2 movement, operator approval, emergency stop, and laptop-layout checks. It saved actual ROS state snapshots and browser/Gazebo captures.
- The expanded control run passed warning-policy changes, a real UAV waypoint command, simulation-time experiment completion, result/log links, evaluation-truth counters, private-belief log protection, telemetry disconnect/reconnect, full simulation reset, and observed industrial fire/gas route replanning. It recorded no browser page errors.
- Live browser visual QA found no horizontal overflow at 375px or 1280px; the map, robot status and camera panels were visible at the laptop viewport. The running page received the actual ROS gateway stream and its current session controls.
- Battery is explicitly marked **Not modeled**. Camera panels identify the real simulated RGB feed and do not claim thermal sensing or image-derived hotspot overlays. Truth, observation, inference and configuration sources are distinguished in the UI.

## Demonstration captures

These are screenshots from the actual integrated Gazebo/ROS run, not mock data:

- [Forest, crew/UAV moving](../results/phase6-visible/forest-moving-dashboard.png)
- [Forest, fallen-tree warning and route alternative](../results/phase6-visible/forest-warning-dashboard.png)
- [Forest, warning acknowledged](../results/phase6-visible/forest-acknowledged-dashboard.png)
- [Industrial, observed debris warning](../results/phase6-visible/oil-warning-dashboard.png)
- [Industrial, approved alternative route](../results/phase6-visible/oil-approved-detour-dashboard.png)
- [Industrial, paused run](../results/phase6-visible/oil-paused-dashboard.png)
- [Laptop layout](../results/phase6-visible/laptop-dashboard.png)
- [Experiment monitor and evaluation controls](../results/phase6-controls-passed/experiment-complete.png)
- [Industrial fire/gas observations and replanned routes](../results/phase6-controls-passed/fire-gas-dashboard.png)
- [Recorded browser walkthrough](../results/phase6-visible/page@837bacd0b0c7e5c4fe7d046a757e3c69.webm)

The complete recording run also saves native Gazebo captures and per-step ROS snapshots beside the screenshots in `results/phase6-visible/`.

## Reproduction

Start a live simulator and its ROS dashboard gateway, then start the host session supervisor and frontend:

```sh
docker compose up sim
# In another terminal at the project root:
bash scripts/run_dashboard.sh
```

Open `http://127.0.0.1:5173`. The host launcher starts both the scenario/experiment session supervisor and Vite; the ROS gateway remains the telemetry source. For the native 3D viewer on this Mac, use the GUI compose profile documented in the root [README](../README.md).

The automated browser integration runner and visible demo runner are `scripts/verify_dashboard_controls.cjs` and `scripts/record_dashboard_demo.cjs`. They require Playwright in the acceptance environment; Playwright is not a runtime dependency of the dashboard.

## Remaining qualifications

The dashboard is a research supervisory console, not certified emergency-response equipment. Fire/gas behavior and semantic sensing are simulation approximations; the UAV motion is scripted rather than validated flight; battery and thermal sensing are not modeled. Evaluation counters are available only when the selected run actually records evaluation truth, and private human-belief labels remain excluded from normal telemetry and public logs.
