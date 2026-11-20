# Phase 8 · Milestone 1: industrial equipment registry

## Status

Implemented and automated acceptance tests pass. The registry is configuration metadata only; this milestone does not add process telemetry, combustion, dispersion, thermal sensing, or damage physics. A live GUI session was not restarted as part of this milestone.

## What is included

- `ros2_ws/src/hri_industrial/config/equipment_registry.json` registers 13 storage tanks, pipelines, process columns, and valves using generated Gazebo model IDs, SDF geometry references, and world positions.
- Registry loading rejects duplicate IDs/models, missing Gazebo models, nonmatching positions, invalid geometry references, and malformed metadata.
- The industrial manager publishes `/hri/equipment_registry` immediately at startup and again on a wall-clock timer. Dashboard provenance identifies it as `configuration`, with a ROS simulation timestamp and normal stale-data handling.
- The oil-facility dashboard displays registry entries and explicitly labels unsupported measurements as not modeled or not assessed.
- The local session endpoint permits editing only contents label, material class, and metadata source for a known equipment ID. It uses atomic file replacement, rejects malformed or oversized fields, and blocks writes during active experiments.

## Verification

Commands run:

- `npm --prefix dashboard test` — 17 dashboard tests passed.
- `npm --prefix dashboard run build` — TypeScript check and Vite production build passed.
- `docker compose run --rm --no-deps sim bash -lc 'cd /work && python3 -m unittest discover -s tests -p test_dashboard_session.py -v && cd ros2_ws && colcon build --packages-up-to hri_industrial hri_dashboard --symlink-install --event-handlers console_direct+ && colcon test --packages-select hri_industrial hri_dashboard --event-handlers console_direct+ && colcon test-result --verbose'`

The session API suite passed 7 tests, including a real loopback HTTP request, active-experiment rejection, metadata persistence, and immutable telemetry checks. The two selected ROS packages passed 27 tests (17 dashboard gateway/protocol and 10 industrial); `colcon test-result` reported 133 tests, zero errors, and zero failures across the workspace's stored results.

## Boundaries and next gate

The registry can identify an object and store operator-provided descriptive metadata, but it does not establish a material identity, sensor reading, equipment condition, release, fire temperature, or secondary-hazard probability. Those remain unknown/not modeled. Do not treat the panel as an operational industrial monitoring system.

The automated acceptance gate passed. Before claiming live end-to-end integration, restart the oil-facility scenario intentionally and confirm the registry topic reaches the dashboard while paused and after a metadata edit; that live check was deliberately not run here to avoid disrupting an existing simulation.
