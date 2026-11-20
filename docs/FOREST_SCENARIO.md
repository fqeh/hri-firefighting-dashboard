# Phase 2 forest scenario

The environment remains native Gazebo Harmonic with ROS 2 Jazzy. Terrain, robot/crew motion, collisions, RGB cameras, logical cameras and pose messages are provided by Gazebo. The route graph and fire model are planning/research abstractions in world coordinates, not replacement physics or a grid-world simulator.

## Run and replay

Build the existing workspace and GUI image as described in the root README. Then:

```bash
python3 scripts/forestctl.py start --case D --seed 7
```

Open http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale and press Play. The GUI starts paused so the incidents cannot occur before the viewer arrives. For immediate execution, add `--play`. The dashboard at http://127.0.0.1:5173 requires `npm --prefix dashboard run dev`; its forest panel displays edge states, route recommendations and the actual UAV camera.

```bash
python3 scripts/forestctl.py pause
python3 scripts/forestctl.py resume
python3 scripts/forestctl.py reset
python3 scripts/forestctl.py replay
python3 scripts/forestctl.py status
python3 scripts/forestctl.py stop
```

Reset/replay restart the complete ROS/Gazebo service, restoring controller state, knowledge, RNG and physics. Each run uses a new timestamped directory under `results/forest_runs`; the selected case and seed are retained in `results/forest-session.json`. Gazebo's circular reset button alone does not reset ROS controllers.

Headless equivalent:

```bash
docker compose run --rm sim bash scripts/run_demo.sh forest forest_case:=D seed:=7 dashboard:=false log_path:=/work/results/forest-D.jsonl
```

## Scenario definitions

Explicit JSON files are under `ros2_ws/src/hri_sim/config/forest_scenarios/`:

| Case | Condition |
| --- | --- |
| A | No fire and no incidents |
| B | Fire begins at 4 simulated seconds near the original route |
| C | A colliding fallen tree blocks `trail_a` at 8 seconds |
| D | Both fire and tree |
| E | Same tree as F; crew initially knows `trail_a` is blocked |
| F | Same tree as E; crew initially believes routes clear |

Files contain initial robot, crew and UAV poses; world-coordinate route nodes; safety node; fire origins, fuel patches and wind; incident schedules; seed; and UAV observation waypoints. E/F represent different prior knowledge of the same configured incident, not a claim that the tree was already physically present before its scheduled appearance. Original `forest.json`, industrial scenarios and recorded baseline results remain separate.

The world instancer applies initial poses, adds understory away from trails and a visible UAV patrol path, and reuses the forest's terrain, trees, route geometry and safety-zone assets. Route/safety geometry is fixed to the existing authored graph: changing graph node coordinates requires updating the corresponding world geometry as well. The scripts do not claim arbitrary edited paths are obstacle-free.

## UAV control boundary

The UAV is a visible, colliding Gazebo model with a downward RGB/logical camera. The scripted controller requests small pose changes through Gazebo's `set_pose` service and reads actual returned pose telemetry. This is **scripted kinematics, not validated flight dynamics**. It has no aerodynamic, motor, battery or collision-avoidance controller. Observation goals are restricted to a configured forest envelope and altitudes 6–12 m; this is not a general obstacle-avoidance guarantee.

Thin cyan lines overhead show the original planned patrol. The translucent blue circle is inscribed in the square downward camera footprint, assuming flat ground and a downward-facing camera. Its radius updates in 0.25 m increments when altitude changes; terrain occlusion is not modeled in this overlay. The actual traveled trajectory is published as `/uav/trajectory` (`nav_msgs/Path`).

- Pose: `/uav/odom` (`nav_msgs/Odometry`), from Gazebo.
- Camera: `/uav/camera/image`; HTTP `/api/uav-camera.png`.
- Mission: `/hri/uav_status`, including actual pose freshness and current target.
- Command: `/uav/observation_goal` (`geometry_msgs/PoseStamped`, frame `world`). A valid command replaces the patrol with a single observation goal and holds it.

Example while the GUI service is running:

```bash
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec gui /hri-entrypoint.sh ros2 topic pub --once /uav/observation_goal geometry_msgs/msg/PoseStamped '{header: {frame_id: world}, pose: {position: {x: 6.0, y: 0.0, z: 8.0}, orientation: {w: 1.0}}}'
```

For a future PX4 adapter, launch with `uav_controller:=external` to disable the scripted node. The adapter must consume the world-frame observation-goal interface and publish mission status. Replace the static scripted UAV SDF with a dynamic PX4-compatible vehicle, preserving or adapting the sensor/pose topic interfaces; disabling the scripted node alone does not create a flight-dynamics vehicle. PX4 integration, NED/ENU conversion and real flight dynamics are not implemented or validated by this phase.

## Fire model assumptions

Each ignition has 12 radial fronts at fixed world directions. At fixed **one-second simulation-time steps**, each front advances by:

`base_rate × local_fuel_load × max(0.1, 1 + wind_gain × wind_speed × cos(front_direction − wind_direction)) × seeded_variation`

Variation is sampled once per front from [0.95, 1.05]. Fuel is piecewise constant in circular world-coordinate patches; later patches override earlier ones. Zero fuel prevents further advance at the front. Radius is capped by `max_radius`. The origin/time, wind vector, fuel map, exposure cost and random seed are configurable. Evaluating the same seed at the same time reproduces the same front state independently of ROS callback frequency.

This is an intentionally small research model. It has no combustion chemistry, heat transfer, fuel depletion, spotting, slope/wind coupling, suppression, plume transport or empirical wildfire calibration. Burning regions persist; a radius cap is a simulation bound, not a predicted firebreak. There is no claim of physically validated wildfire prediction or firefighter safety.

`/forest/fire_state` explicitly contains **model truth**, separate from the normal HRI belief stream. Flame/smoke meshes are non-solid visual cues; new hotspots are inserted when the maximum front radius crosses an integer metre. They are a coarse projection and do not define the mathematical exposure boundary. Smoke is translucent geometry, not fluid simulation or visual-perception degradation. Exposure uses interpolated radial boundaries, with costs integrated along each route in approximately 0.2 m midpoint intervals.

## Routes, observations and incidents

A fallen log is dynamically created across the 2.2 m-wide `trail_a`, with its cylinder collision active. This is a pre-fallen log placement, not a simulated toppling fracture. Successful creation, configured event time and affected edge are recorded in `/evaluation/truth`; normal HRI inference learns of it through Gazebo logical-camera detections. The incident schedule is fixed by scenario seed, including optional seeded timing jitter.

The forest planner consumes UAV logical observations and sensor-observed hazards. A route is surveyed only after all 21 sampled centreline positions have been covered by the downward camera footprint. Logical cameras supply ideal identities without occlusion or detection errors; this is an explicitly favorable perception assumption. Surveyed states persist, so an unseen later change remains possible.

States are separate: **blocked** edges are excluded; **hazardous** edges carry integrated fire exposure cost; **unknown** edges incur an uncertainty penalty; **clear** means sampled coverage under the stated assumptions. Fire estimates enter planning only after the fire is observed, and then use the exact research fire model; this is an idealized model-assisted observation, not inferred thermal sensing.

Recommendations minimize distance plus exposure-weighted cost and unknown penalties. The planner connects the crew to the nearest graph node; connectors beyond 0.5 m are explicitly unverified. Consequently it reports “No verified route currently available” whenever the candidate contains unknown/hazardous edges or an unverified connector. It can still provide a conditional alternative; if disconnected, it returns no path. It does not certify the continuous swept robot footprint or real thermal safety.

Cyan **ground strips** show the current recommended graph route; red ground strips mark blocked edges. The crew retains its separate behavior/knowledge controller and responds to local observations, probabilistic warnings and configured prior knowledge. Its body is a wheeled humanoid proxy, not an articulated gait model. `/human/odom` and `/hri/human_intent` log actual position and selected route. The UAV/route overlay does not teleport the crew or robot.

## Assets and tests

Existing authored Apache-2.0 terrain, vegetation, wheeled robot, humanoid proxy, UAV and hazard assets are reused. Added shrubs and path overlays are authored SDF primitives under the same license. No third-party asset download or additional license obligation is introduced.

```bash
docker compose run --rm sim bash scripts/build.sh
docker compose run --rm sim python3 scripts/test_forest.py --output results/forest-new-acceptance
docker compose run --rm sim bash -c 'cd ros2_ws && colcon test --executor sequential && colcon test-result --verbose'
```

The physical test launches A once and D twice, verifies actual crew/UAV movement and camera data, commands a new UAV observation goal, checks time-varying fire, incident creation, blocked-edge exclusion and an alternative route. A test-only robot placement at `(5,-2)` positions it for a sustained forward drive against the spawned log; independent Gazebo pose checks that the log physically stops it. Replay compares configured event IDs/order/times; asynchronous physical execution times are recorded separately and are not promised bit-identical.
