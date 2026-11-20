# Phase 8 Milestone 2 — Tank aftermath and route update

## Status

**Implemented and live-verified.** The oil-facility operator can trigger a predefined TK-01 damage aftermath without replacing existing scenarios. The simulation records the request and entity creation, then the robot's logical-camera observation drives the public hazard map and route planner. No missile, combustion, dispersion, or tank-process physics are claimed.

## Behavior

- The aftermath is manual-only and idempotent. It adds a non-colliding visual damage marker to the tank and separately spawns a colliding debris object at `[6, 6, 0.6]` on `north_road`.
- The operator request and successful creation are recorded on `/hri/events` and in the episode event log. A failed entity creation is also reported and remains retryable.
- The robot's semantic logical-camera observation, not the scenario manager's private truth, marks the debris as a blocked route hazard.
- On observation, the route planner excludes `north_road`. The active navigation goal is canceled; a supervisor must inspect and approve the current safe route revision before the robot resumes.
- Existing forest and industrial cases are retained. The generated `normal` run configuration adds the manual incident without editing the original case file.

## Live verification

Run: oil facility `normal`, seed 7, policy B1, manual incidents enabled. The evidence is from the actual ROS 2/Gazebo episode, not a fixture:

| Simulation time (s) | Observed event |
|---:|---|
| 11.081 | A request using the catalog event name was rejected (`No configured incident with this ID`); no incident was created. |
| 92.242 | Correct operator request recorded for `tank_damage_aftermath`. |
| 92.564 | Tank visual and debris entity creation recorded; affected edge was `north_road`. |
| 122.642 | Robot logical-camera perception recorded `hazard_debris_tk01_aftermath` on `north_road`. |
| 122.803 | Route revision 12 marked `north_road` blocked and recommended `corridor_b → junction → north_junction`. |
| 122.999 | Live navigation telemetry showed the prior goal canceled while awaiting operator approval. |
| 127.402 | Fresh route revision 13 still excluded `north_road` and retained the same safe detour. |
| 132.117 | After the operator approved the fresh recommendation, navigation was active on the detour, with the robot at approximately `(10.21, 0.58)` and its current waypoint at `(9, 0)`. |
| 133.201 | Fresh route revision 14 continued to recommend `junction → north_junction`, using `corridor_b` and `corridor_c`, not `north_road`. |

The simulation was paused after this check. The complete event sequence, including route revisions and the corrected incident request, is in [the episode event log](../results/oil_facility_runs/1795043846897023000_normal/events.jsonl). The saved generated scenario configuration is [scenario.json](../results/oil_facility_runs/1795043846897023000_normal/scenario.json), and the simulator output is [simulator.log](../results/oil_facility_runs/1795043846897023000_normal/simulator.log).

## Automated verification

The updated `hri_reasoning` package built successfully in the ROS Jazzy container. Its focused suite passed **85 tests, 0 errors, 0 failures**:

```sh
docker exec project-gui-1 bash -lc \
  'cd /work/ros2_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon test --packages-select hri_reasoning --pytest-args -q && colcon test-result --test-result-base build/hri_reasoning --verbose'
```

The regression tests cover generic hazard IDs with configured suffixes, including `hazard_debris_tk01_aftermath`, while preserving existing tree, fire, gas, and debris naming patterns.

## Reproduce in the dashboard

1. Start a fresh oil-facility `normal` run with **Manual incidents** enabled and seed 7; resume the simulation.
2. Select `north_service` as the destination and approve the fresh route recommendation.
3. Trigger **Tank TK-01 damage aftermath**. The incident's control payload uses hazard ID `hazard_debris_tk01_aftermath` (not catalog event ID `tank_damage_aftermath`).
4. Wait for the robot observation. Confirm the dashboard marks `north_road` blocked and shows the alternative route.
5. Approve the latest route revision and confirm navigation resumes on the alternative; do not approve a stale revision.

## Limits

The damage marker is visual-only and does not alter the tank's physical geometry or process state. The debris is a configured collision obstacle. Logical-camera semantic identity and pose are idealized; this is not validated image recognition. The event does not model missile guidance or impact dynamics, fire or smoke physics, material release, gas dispersion, or measured tank temperature, pressure, inventory, or integrity.
