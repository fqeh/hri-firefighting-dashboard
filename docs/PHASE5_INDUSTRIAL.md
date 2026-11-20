# Industrial emergency scenario

This fictional desert energy facility is an original, generic layout. It is not a replica of a Saudi Arabian installation. All facility primitives and the reused differential-drive robot/hazard assets have the repository's Apache-2.0 license. Gazebo owns collisions, wheel dynamics and sensor data. The overhead road diagram in the dashboard is a diagnostic of the same world, not another simulator.

## Run

On the tested macOS/Docker setup, build once:

```sh
docker compose run --rm --no-deps sim bash scripts/build.sh
cd dashboard
npm ci
npm run dev -- --host 127.0.0.1
```

In another terminal at the repository root:

```sh
python3 scripts/scenarioctl.py start --scenario oil_facility --case blockage
```

Open [native Gazebo](http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale) and [dashboard](http://127.0.0.1:5173). Press Play in Gazebo or run `python3 scripts/scenarioctl.py resume`. Select **East assembly point**, prefer **Corridor B**, and request a recommendation. Review and explicitly approve it. Debris appears after eight simulation seconds. The robot's observations update the map and cancel any affected approved route. Requesting Corridor B again supplies observable operator-action evidence. Approve the alternative route to authorize further movement. Camera, LiDAR, posterior, expected losses and Nav2 state are live ROS data.

The same selector switches back to the forest:

```sh
python3 scripts/scenarioctl.py start --scenario forest --case D
python3 scripts/scenarioctl.py pause
python3 scripts/scenarioctl.py replay
python3 scripts/scenarioctl.py stop
```

Reset/replay starts fresh Gazebo and ROS processes with the saved configuration and seed. `--play` runs immediately; the default starts paused. `--simulated-operator` enables a seeded experimental operator. Interactive approvals remain required unless `operator.auto_approve` is explicitly enabled in a custom trial file.

Native Ubuntu/ROS or a headless container can use:

```sh
bash scripts/run_demo.sh oil_facility oil_case:=blockage
bash scripts/run_demo.sh oil_facility oil_case:=normal gui:=true
bash scripts/run_demo.sh forest forest_case:=D
```

`nav2:=auto` enables Nav2 for oil facilities only. `mission:=true` cannot compete with Nav2. Preserve the stop latch via `/robot/emergency_stop`; clearing it does not approve a new route.

## Cases and control contracts

Cases: `normal`, `blockage`, `fire`, `gas` (blockage plus gas on the north alternative), `combined`, `known`, `unaware`, and `no_safe`. Files in `hri_industrial/config/scenarios` specify spawn, incident times/positions/radii, random seed and private operator initialization. Override with `scenario_config:=/absolute/path.json`. `known` and `unaware` differ only in private initial operator knowledge and case label.

Publish JSON `std_msgs/String` on `/hri/control`, or POST the JSON object to `/api/control`:

```json
{"command":"select_destination","destination":"assembly","via":"corridor_b"}
```

This is a request, not permission to drive. Approve the current `/hri/robot_route.revision` with `{"command":"approve_route","revision":2}`. A stale revision is rejected. `cancel_waypoint`, `acknowledge_warning` with `hazard_id`, `pause`, and `resume` retain their existing contracts. Coordinate waypoint requests must be within 0.75 m of a surveyed node and also require approval. Routes marked `navigation_allowed:false` cannot be approved.

## Information boundaries and reused algorithms

The existing `RouteToM`, four-state HMM, Boltzmann likelihood, `DecisionPolicy`, MDP/value iteration, frozen Q-learning comparison, baseline B1–B4, risk-cost function and `HumanResponse` are imported unchanged. SHA-256 regression checks compare the research core against the start of Phase 5. Only adapters, scene inputs and public explanations differ. `warning_engine:=expected_loss|value_iteration|q_learning` and `policy:=B1|B2|B3|B4|oracle` retain their meanings and configurable costs. Q-learning limitations from Phase 4 still apply.

`operator_input` publishes explicit commands on `/hri/operator_actions` and public intent on `/hri/operator_intent` (also the common `/hri/human_intent` contract). Autonomous Nav2 movement is never counted as an operator decision. The operator HMM receives only these actions, public sensor observations, delivered warnings and acknowledgements. Robot camera availability does not imply that the remote operator attended to it. `/hri/belief` contains the same four-state posterior, uncertainty and mismatch fields, with `subject:operator` and `last_operator_evidence`. It contains no private belief truth.

Only the trial operator, evaluation logger and explicitly selected oracle access `/evaluation/human_belief`. The generator uses its own stale map and the existing Boltzmann model; warnings use the existing seeded compliance model. Acknowledgement is observable message receipt, not proof of hidden compliance or actual human trust. Inference acquisition probabilities are configurable assumptions.

## Geometry, navigation and hazards

The public graph is grounded in native SDF roads, gates and corridor walls. Tanks, buildings, utilities, process columns, personnel and debris have active collision geometry. Road keepout masks prevent Nav2 from shortcutting across restricted process areas. Both costmaps also consume real `/robot/scan` obstacle returns. Hazard masks are updated from actual Gazebo semantic-sensor observations, never from incident schedules.

Nav2 uses wheel odometry initialized at the surveyed origin, `world → odom → base_footprint → base_link` and sensor frames. This is odometry navigation, not a validated AMCL/SLAM system; accumulated drift is a limitation. The model includes actual differential-drive control, LiDAR, RGB camera, IMU, odometry and independent evaluation pose. Only evaluation uses Gazebo truth pose.

The initial public road survey assumes configured edges were commissioned as clear at t=0. Later incidents remain unknown until observed. Set `initial_survey_edges:[]` in a custom scenario to test refusal without verified routes. Risk routing excludes physical blockages, unknown segments and any known hazard exposure by default; `max_route_exposure` and existing route-cost parameters are explicit configuration. Connectors must lie inside allowed corridors and outside observed hazard envelopes. If no sufficiently safe route remains, the robot cancels motion and requests assistance.

Fire and gas are persistent, configurable exclusion envelopes activated at scheduled simulation times. Flame/smoke and gas-colored markers are visual explanations of the mathematical zones. Gas detection is an idealized semantic sensor surrogate with a visible marker; it is not RGB gas recognition. There is no validated gas-dispersion, combustion or explosion solver. Debris represents generic emergency aftermath and physically spans the selected road. No weapon impacts are modeled.

Keepout parameters follow the [Nav2 Jazzy KeepoutFilter documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_filters/keepout_filter/). Lethal override is disabled; collisions stay active. Nav2 configuration is generated from the installed Jazzy defaults by `hri_bringup/navigation.py`, with industrial keepout/inflation filters added to both costmaps.

## Verification

```sh
docker compose run --rm --no-deps sim python3 -m pytest -q ros2_ws/src/hri_industrial/test
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-acceptance
```

Each run writes the exact launch command, checks, trajectory, laser samples, public action/decision history and critical log lines to its `result.json`; privileged evaluation data stays in `events.jsonl`. A test failure is not counted as acceptance. Native GUI visibility is checked separately. See [the validation record](PHASE5_VALIDATION.md) for executed commands and current results.

The first build fetches a SHA-256-checked upstream Jazzy archive at commit `95e6fdea72fb57ef55a4136292dfa62370ab3fb9` and builds `ros_gz_bridge` in `.runtime/bridge_ws`. This includes the [upstream Jazzy shutdown/lifetime backport](https://github.com/gazebosim/ros_gz/pull/991). System apt packages are preserved. A single compiler job is used because the tested Docker VM has about 4 GB RAM. The archive cache supports subsequent offline builds. The project workspace records this overlay in its setup chain; confirm selection with `ros2 pkg prefix ros_gz_bridge`.

Additional checks:

```sh
docker compose run --rm --no-deps sim python3 scripts/verify_industrial_collision.py --output results/my-industrial-contact
docker compose run --rm --no-deps sim python3 evaluation/run_industrial_experiments.py --output results/my-industrial-policies
```

The contact fixture deliberately sends a sustained command into debris with Nav2 disabled, verifying that physical collisions stop the robot. Normal operation uses Nav2 and the approval gate. The policy comparison runs known/unaware pairs with the same static robot and physical hazard; only the oracle reads private beliefs. Event order and configured absolute simulation times reproduce; asynchronous sensor receipt and floating-point physics are not claimed bitwise deterministic.
