# Belief-Aware Warnings for Human–Robot Firefighting Teams

A university HRI research prototype using **real Gazebo Harmonic physics and sensors**, ROS 2 Jazzy, Bayesian belief inference, and a local React/TypeScript supervisor. The research question is whether targeted warnings about a teammate's outdated route knowledge can improve simulated outcomes with fewer interruptions.

Gazebo owns every physical entity and all motion. Python implements behavior, inference, communication decisions and scoring—not a replacement environment. The diagnostic waypoint map is not the simulator.

**Verification and remaining limits:** see [results/VALIDATION.md](results/VALIDATION.md). Source implementation, independent tests, simulator integration and research evidence are reported separately there.

**Phase 1 foundation acceptance passed:** [tested commands, evidence, failures fixed and remaining limits](docs/PHASE1_VALIDATION.md).

**Phase 6 dashboard acceptance passed:** [live ROS/Gazebo demonstration, responsive visual QA, control integration and screenshots](docs/PHASE6_VALIDATION.md).

**Phase 7 evaluation and handoff:** [validation status](docs/PHASE7_VALIDATION.md), [final measured results and candid research status](RESULTS.md), [research report](docs/FINAL_RESEARCH_REPORT.md), [reproduction checklist](docs/REPRODUCIBILITY_CHECKLIST.md), and [demonstration script](docs/DEMO_SCRIPT.md). The prototype and evaluation pipeline pass engineering acceptance; the powered research study remains incomplete.

## Phase 5 industrial emergency

The fictional oil facility uses the same Bayesian HMM and warning policies with operator-command evidence, a physical differential-drive robot, Nav2, LiDAR and camera. Destination selection requests a recommendation; explicit approval authorizes motion. Debris, fire and gas observations cause risk-based replanning, and unavailable routes are refused.

```sh
docker compose run --rm --no-deps sim bash scripts/build.sh
python3 scripts/scenarioctl.py start --scenario oil_facility --case blockage
python3 scripts/scenarioctl.py resume
# Switch the same native Gazebo launcher back to the forest
python3 scripts/scenarioctl.py start --scenario forest --case D
```

Open [Gazebo](http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale) and [the dashboard](http://127.0.0.1:5173). See [industrial controls, scenario files, information boundaries and assumptions](docs/PHASE5_INDUSTRIAL.md) and the [passing acceptance record, commands and recordings](docs/PHASE5_VALIDATION.md). The first build also compiles a pinned upstream Jazzy `ros_gz_bridge` shutdown fix in an isolated local workspace; it does not change the research algorithms or system packages.

## Phase 4 warning decisions and rescue routes

The Bayesian posterior now drives configurable expected-loss, value-iteration or frozen Q-learning warning policies. Public risk routing excludes observed physical blockages and penalizes fire exposure. Warnings include evidence and an alternative route; simulated acknowledgement, compliance and trust are logged separately. See [policy equations, MDP, interfaces and assumptions](docs/WARNING_POLICY.md) and [execution evidence](docs/PHASE4_VALIDATION.md).

```bash
# Combined fire + falling-tree forest; starts paused for viewing
python3 scripts/forestctl.py start --case D --policy B4 --engine expected_loss --observe true --human-radius 1 --map-rate 0 --lapse 0 --trust 0.9 --seed 7
python3 scripts/forestctl.py resume
# Full-process deterministic reset (paused by default)
python3 scripts/forestctl.py replay
```

Open [the native Gazebo viewer](http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale) and [the dashboard](http://127.0.0.1:5173). The crew moves physically, the UAV observes, and the dashboard shows the posterior, warning loss, acknowledgement and risk-weighted alternative. In this forest demonstration the ground robot is a stationary sensor platform; enable Nav2 and approve a route to move it independently.

```bash
# Same physical incident, five policies, known/unaware crew knowledge
# Stop the GUI first on small Docker VMs. Each output directory must be new.
python3 scripts/forestctl.py stop
docker compose run --rm --no-deps sim python3 evaluation/run_warning_experiments.py --duration 80 --output results/my-warning-comparison
# Offline communication MDP training/evaluation; never blocks a demo
docker compose run --rm --no-deps sim python3 evaluation/train_warning_policies.py --output results/my-warning-mdp
```

Select `warning_engine:=value_iteration`, or `warning_engine:=q_learning q_policy:=/work/results/my-warning-mdp/q_policy.json` on `scripts/run_demo.sh` / ROS launch. Missing or incompatible learned tables fall back to expected loss. Modeled costs, trust, fire exposure and UAV motion are research approximations.

## Phase 3 Bayesian Theory of Mind

The default inference now publishes four belief hypotheses per trail and uses confirmed Gazebo crew movement. The new inference process receives a sanitized public map and public observations; private crew knowledge is reserved for behavior and evaluation.

```bash
python3 scripts/forestctl.py start --belief unaware --policy B1 --engine legacy --seed 7 --play
python3 scripts/forestctl.py start --belief known --policy B1 --engine legacy --seed 7 --play
```

Run one command at a time to compare the same physical tree incident. Open [Gazebo](http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale) and the [dashboard](http://127.0.0.1:5173). Seed 7 includes initial waiting; the crew chooses its branch around 11 simulated seconds. See [algorithms, state separation, controls, experiments and assumptions](docs/BAYESIAN_TOM.md) and [tested commands, results and corrected failures](docs/PHASE3_VALIDATION.md). The earlier two-state baseline is preserved with `tom:=false`; its experiment runner selects that mode explicitly.

## Phase 2 forest scenarios

Run `python3 scripts/forestctl.py start --case D --seed 7`, open the Gazebo viewer, then press Play. Cases A–F cover normal evacuation, growing fire, fallen trees, combined incidents and known/unknown crew knowledge. The UAV patrols with a live camera; cyan ground strips show route recommendations and red marks blocked trails. Use `forestctl.py replay` to restart the complete scenario with a new log directory.

See [forest controls, UAV interface, fire assumptions and tests](docs/FOREST_SCENARIO.md). The original baseline launches remain available without `forest_case`.

[Phase 2 validation and recorded rerouting demonstration](docs/PHASE2_VALIDATION.md).

## Quick start on this Mac

Tested host: **macOS 15.3.2, Apple M3 ARM64**. The simulator runs in **Ubuntu 24.04.5 ARM64, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, ros_gz 1.0.24**, with colcon 0.21.3. Docker has eight virtual CPUs and approximately 4 GB RAM; the verified renderer is **Mesa 25.2.8 llvmpipe / OpenGL 4.5** (software rendering). Start Docker Desktop first.

To open the **real Gazebo 3D GUI on this Mac**, build the optional desktop image and ROS workspace:

```bash
cd /Users/ali/Documents/HRI/Project
docker compose -f compose.yaml -f compose.gui.yaml --profile gui build gui
docker compose -f compose.yaml -f compose.gui.yaml --profile gui run --rm gui bash scripts/build.sh
docker compose -f compose.yaml -f compose.gui.yaml --profile gui up gui
```

Open **http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale**, then press **Play (▶) at the bottom left**. The GUI starts paused so the short missions cannot finish before you arrive. The teal robot follows two waypoints; the orange human proxy follows the forest route. Both stop when their missions complete. The robot's measured odometry is written to `results/waypoint_trajectory.csv`.

This is Gazebo's native 3D GUI through noVNC, with a raised camera overlooking the starting area. To replay **both** missions, restart the full service and reconnect the browser:

```bash
docker compose -f compose.yaml -f compose.gui.yaml --profile gui restart gui
```

Gazebo's circular Reset button resets simulator entities but does not restart the ROS mission controllers; use the command above for a complete replay. Set `HRI_GUI_START_PAUSED=false` for automatic start or `HRI_GUI_SCENARIO=minimal` for the small collision-test world.

The GUI launch also exposes the ROS dashboard backend on **127.0.0.1:8080**. In another host terminal:

```bash
cd /Users/ali/Documents/HRI/Project/dashboard
npm ci
npm run dev
```

Open **http://127.0.0.1:5173** for sensor imagery, telemetry and the **Stop robot / Release stop** controls. Stop state is confirmed from the ROS guard, separately from HTTP request acceptance. Do not run the headless `sim` service and GUI service together on the same published backend port.

To stop or release the robot from another terminal without the dashboard:

```bash
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec gui /hri-entrypoint.sh ros2 service call /robot/emergency_stop std_srvs/srv/SetBool '{data: true}'
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec gui /hri-entrypoint.sh ros2 service call /robot/emergency_stop std_srvs/srv/SetBool '{data: false}'
```

This is a latched **simulation software stop**. The guard sends zero velocity while stopped and when commands become stale, using a steady wall clock even if simulation time is paused. Releasing it permits fresh commands, including commands from a still-active mission. It is not an independent hardware safety system.

## Headless execution and acceptance tests

From this repository:

```bash
docker compose build sim
docker compose run --rm sim bash scripts/build.sh
docker compose run --rm sim bash scripts/smoke_test.sh
docker compose run --rm sim bash scripts/phase1_acceptance.sh --output results/phase1-new-run
```

The smoke test checks actual simulation time, independent Gazebo pose, odometry, lidar, IMU, rendered pixels and commanded physical motion. The extended Phase 1 acceptance adds TF, timestamps, emergency-stop latching, command timeout, real obstacle contact, a two-waypoint mission, trajectory CSVs and a critical-log scan. It refuses to overwrite an existing acceptance report; use a fresh output directory. GUI visibility has separate screenshot/connection evidence.

For a headless moving robot demonstration:

```bash
docker compose run --rm --service-ports sim bash scripts/run_demo.sh minimal mission:=true
```

Robot commands follow `/robot/cmd_vel → safety_controller → /robot/cmd_vel_safe → ros_gz_bridge → Gazebo`. The ROS TF chain is `odom → base_footprint → base_link → laser_frame / camera_optical_frame`. Wheel odometry has turning error relative to independent Gazebo pose; acceptance checks both and uses an explicitly reported 0.35 m physical waypoint tolerance.

Start the forest experiment and live backend:

```bash
docker compose up sim
```

In another host terminal:

```bash
cd dashboard
npm ci
npm run dev
```

Open **http://127.0.0.1:5173**. It shows actual camera imagery, pose, hazard observations, intended/recommended routes, inferred belief and warning events. Empty or stale panels indicate missing data. The camera is rendered by Gazebo; the 2D map is labeled as a diagnostic view. The frontend proxies to the backend at port 8080.

For the industrial scenario with the real Nav2 stack:

```bash
docker compose down
docker compose run --rm --service-ports sim bash scripts/run_demo.sh industrial nav2:=true seed:=7 policy:=B4
```

Use the dashboard to approve the current risk-weighted route, or use its waypoint form to replace it with an `odom`-frame Nav2 goal. Route approvals carry a revision and `world`-frame waypoints; newly observed hazards on approved edges require renewed approval. Command acceptance is distinct from navigation success. The robot uses real lidar obstacle costmaps and cannot certify unobserved space as safe.

With Nav2 enabled, a separate **operator adviser** compares the actual plan and requested waypoint against robot-observed hazards. Its panel shows inferred awareness and a conditional graph alternative. Acknowledgment is an explicit operator report; canceling a waypoint waits for Nav2's cancellation result. Selecting an alternative fills the form, and the operator explicitly submits the revised goal. The automated baseline experiments below evaluate the simulated responder, not this interactive operator workflow.

Stop with Ctrl-C. A deterministic **reset** means stopping the entire launch and starting it again with the same seed. This resets physics, RNG, human knowledge, posterior state, policy cooldowns and logs together.

## Native Ubuntu with interactive 3D

Use Ubuntu 24.04; Jazzy and Harmonic are a matched pair. Review and run the installer, then build:

```bash
bash scripts/install_ubuntu.sh
bash scripts/build.sh
bash scripts/run_demo.sh forest gui:=true log_path:="$PWD/results/native_demo.jsonl"
```

For RViz diagnostics in another sourced terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
rviz2 -d ros2_ws/src/hri_sim/config/diagnostics.rviz --ros-args -p use_sim_time:=true
```

The default `sim` container is headless. The optional `gui` service above provides the actual Linux Gazebo desktop on this Mac through noVNC. Native Ubuntu can run the same GUI directly; RViz interaction on a native Ubuntu desktop has not been tested here.

## Experiments and tests

```bash
# Algorithms, assets, gateway and frontend; no Gazebo installation needed
bash scripts/test_local.sh

# Build/test ROS packages in the simulator runtime
docker compose run --rm sim bash -c 'cd ros2_ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose'

# Repeated physical simulation trials, two scenarios, B1–B4 and oracle
docker compose run --rm sim python3 evaluation/run_experiments.py --seeds 7 17 27 --duration 90

# Compare actual logs; run this inside the container or on the host
python3 evaluation/compare.py results/trials/*.jsonl --output results/comparison.json

# Assumption sensitivity: same inference beta, varied simulated behavior
docker compose run --rm sim python3 evaluation/run_experiments.py --scenarios forest --human-betas 0.2 1.2 3.0 --output results/robustness

# Communication-model-only comparison; NOT a Gazebo safety result
python3 evaluation/policy_comparison.py --output results/communication_policies.json
```

Check `--help` for configurable policies, seeds, observation radius, trust and timeouts. Trial validity requires physical odometry and sensor evidence, successful scheduled incident creation and observed detection; invalid or incomplete trials are excluded from comparisons. Missing outcomes are null, not fabricated zeros. See [evaluation/README.md](evaluation/README.md) for metric definitions and statistical limitations.

## Project guide

- [ARCHITECTURE.md](ARCHITECTURE.md): modules, information boundaries and data flow.
- [INSTALLATION.md](INSTALLATION.md): tested Docker/ROS setup and validation commands.
- [USER_GUIDE.md](USER_GUIDE.md): forest/industrial dashboard walkthrough.
- [EXPERIMENTS.md](EXPERIMENTS.md): policies, conditions, metrics and experiment commands.
- [RESULTS.md](RESULTS.md): Phase 7 outcomes, exports and uncompleted research.
- [LIMITATIONS.md](LIMITATIONS.md): remaining scientific and modeling limitations.
- [CONTRIBUTING.md](CONTRIBUTING.md): change, testing and evidence expectations.
- [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md): software and asset license summary.
- [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md): independently testable gates.
- [Simulation assets](ros2_ws/src/hri_sim/README.md): worlds, sensor/robot approximations and regeneration.
- [HRI reasoning](ros2_ws/src/hri_reasoning/README.md): hypotheses, warning costs, messages and baselines.
- [Dashboard](dashboard/README.md): telemetry API and local development.
- [Dependencies](docs/DEPENDENCIES.md): platform compatibility, sources and licenses.
- [Research report](docs/RESEARCH_REPORT.md): hypotheses, interpretation and limitations.
- [Measured outcomes](results/report/RESULTS.md): actual baseline results, including the null B4 benefit in this matrix.
- [Limitations](docs/LIMITATIONS.md): remaining work against the full specification.

## Installation troubleshooting

The Dockerfile uses an HTTPS Oregon State ROS mirror with the official ROS signing key. Package signatures and checksums remain enforced. Override `ROS_APT_MIRROR` only with a compatible ROS 2 Ubuntu mirror; changing Gazebo or ROS distributions is not a fix for transport errors.

Docker Desktop's credential helper stalled while pulling the public base image on this host. If that recurs, use an isolated anonymous configuration (this does not edit your Docker account settings):

```bash
mkdir -p /tmp/hri-docker-anonymous
printf '{"auths":{},"credsStore":""}\n' > /tmp/hri-docker-anonymous/config.json
docker --config /tmp/hri-docker-anonymous --host "unix://$HOME/.docker/run/docker.sock" build -t belief-aware-hri:jazzy -f docker/Dockerfile .
```

If rendering fails, inspect `results/smoke_simulator.log`. On native Linux try a working hardware OpenGL driver; the provided container installs Mesa software rendering and Xvfb. Do not treat a non-rendering run as camera/lidar validation.

Original project code and generated SDF/terrain assets are Apache-2.0 licensed. Upstream ROS/Gazebo/Nav2 and frontend dependencies retain their own licenses. This models generic fictional emergency aftermath, not attacks or real industrial sites.
