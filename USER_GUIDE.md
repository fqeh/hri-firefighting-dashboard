# User guide

> The current presentation UI is a single-screen dashboard (no Live/Details/Experiment tabs); see [docs/DEMO_DASHBOARD.md](docs/DEMO_DASHBOARD.md). The walkthrough below describes the earlier tabbed console; its controls still exist in the Scenario, Robot, Experiment and Log sections.

## Start a demonstration

1. Start Docker Desktop.
2. From the project root, run `docker compose up sim`.
3. In a second terminal, run `bash scripts/run_dashboard.sh`.
4. Open `http://127.0.0.1:5173`. The map, camera, status and warning panels should show real ROS/Gazebo telemetry; missing or stale data is explicitly labeled.
5. To view the actual 3D world, use the GUI profile described in the [README](README.md), at `http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale`.

## Dashboard controls

- Choose **Forest wildfire** or **Oil facility**, select a case, policy, seed, and configured behavior parameters, then select **Load scenario & policy**. This creates a fresh paused run.
- **Resume/Pause** controls the simulation clock. **Reset run** starts a fresh process group; it is the deterministic reset boundary.
- **Inject test incident** creates the selected physical Gazebo hazard when manual incidents are enabled. The dashboard does not invent a detection; the robot/UAV must observe it.
- In the forest, enter a world-coordinate UAV waypoint to replace its scripted observation target.
- In the oil facility, request a destination recommendation, inspect the observed risk and alternative, then approve the current route revision. A stale revision or route marked unsafe is not accepted. When a newly observed hazard changes the approved route, Nav2 may stop and require a fresh recommendation and approval; wait for live route telemetry before approving again. The planner also supports a robot located between surveyed graph nodes, provided its connector follows a currently verified road segment. Coordinate goals must match surveyed road nodes.
- In the oil facility's manual-incident mode, select **Tank TK-01 damage aftermath** to add the configured visual damage marker and a separate debris obstacle on `north_road`. The robot must observe the debris before its route map changes. Once observed, the planner marks `north_road` blocked; Nav2 stops until you inspect and approve the newly recommended route revision.
- Acknowledge and dismiss apply to the displayed warning. Dismissal does not clear a hazard.
- **Emergency stop** applies to the ground robot only. The latch display changes only on a fresh ROS heartbeat; an HTTP acceptance is not proof of stopping.
- The experiment monitor starts a seeded, fixed simulation-time run, records actual ROS events, and links results. Ground-truth counters are opt-in evaluation data and may be unavailable when the trial does not produce the required labels.

## Industrial equipment registry

In the oil-facility scenario, the **Industrial equipment registry** panel lists 13 configured objects linked to generated Gazebo model names and positions: storage tanks, pipelines, process columns, and valves. The panel is configuration metadata, not a sensor feed. Inventory, temperature, pressure, integrity, leak, and safety-system values remain explicitly not modeled or not assessed unless future milestones add validated simulation sources.

Use **Edit contents metadata** to change the contents label, material class, or metadata source. The save endpoint accepts only these three descriptive fields for a known equipment ID; it cannot edit model identity, geometry, position, or telemetry. Saving updates `ros2_ws/src/hri_industrial/config/equipment_registry.json`, and the running industrial manager republishes it even while simulation time is paused. Editing is disabled during an active experiment.

Battery is not modeled. Camera views are actual simulated RGB; no thermal sensor or pixel-based hotspot classifier is implemented. The UAV motion is scripted rather than validated flight. Every displayed route map is diagnostic and does not replace Gazebo's 3D world or Nav2 costmaps.

The tank aftermath is a repeatable, operator-triggered demonstration event: the tank damage is a visual overlay and the debris is a separately spawned route blocker. It does not simulate an impact, tank degradation, fire, smoke, material release, dispersion, or equipment process telemetry. See the [Phase 8 Milestone 2 verification record](docs/PHASE8_MILESTONE2.md).

For screenshots and the tested walkthrough, see [Phase 6 validation](docs/PHASE6_VALIDATION.md). For the final Phase 7 evidence, see [Phase 7 status](RESULTS.md).
