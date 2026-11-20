# hri_reasoning

## Current Phase 4 path

`human_behavior` → public intent/odometry → `bayesian_tom` → `warning_decision` implements the live belief-aware loop. `rescue_routing` supplies risk-weighted alternatives; `oracle_warning` is the separately privileged comparator. Engines are `expected_loss`, `value_iteration`, and frozen `q_learning`, with fallback to expected loss. Configuration files live in `config/`. See [full policy and interface documentation](../../../docs/WARNING_POLICY.md) and [verified results](../../../docs/PHASE4_VALIDATION.md). The following two-state description is retained for the explicitly selected legacy baseline.

This package separates communication/behavior algorithms from Gazebo physics.
`core.py` imports only Python's standard library. `runtime.py` provides four
separate ROS processes, all launched with `use_sim_time=true`.

| Process | Inputs | Outputs / responsibility |
|---|---|---|
| scenario_manager | Config, `/hri/control`, ROS clock | Creates actual SDF entities through Gazebo's `create` service; publishes `/evaluation/truth` only after success. |
| human_behavior | `/human/odom`, `/human/detections`, `/hri/warning`, observed shared map | Publishes physical `/human/cmd_vel`, observable `/hri/human_intent`, private evaluator `/evaluation/human_belief`. |
| belief_warning | Robot/UAV `LogicalCameraImage`, declared human route choices | Bayesian posterior, conditional alternative route, warnings. Normal modes have no truth/belief subscription. |
| event_logger | Public events, evaluator-only topics, independent raw odometry/camera streams | JSONL provenance and outcome record; never drives policies. |

Use `scenario`, `config`, `seed`, `policy` (`B1`, `B2`, `B3`, `B4`, `oracle`) on
all nodes. `human_behavior` additionally accepts `human_beta`,
`observation_radius`, `trust`, `passive_map_rate` (default .05 per simulated second,
zero disables voluntary map glances); `belief_warning` accepts `assumed_beta`;
`event_logger` accepts `log_path`.

The semantic catalog identifies possible incident models and their affected
graph edges. A catalog entry is **not** an observation or active incident.
Inference creates a hazard record only when its model is in a real Gazebo
logical-camera message, reconstructs position from camera/model poses, and
discards event scheduling metadata. It never treats absence from the frustum as
an observed-clear route. Alternative routes remain `conditional` whenever they
contain unobserved edges.

The HMM hidden states for each detected change are `unaware` and `aware`.
Uncertainty is represented by the full posterior and binary entropy. Each
one-second prediction step applies `P(aware | unaware)=.02` and
`P(unaware | aware)=.005`. Distinct declared waypoint decisions update likelihoods
under a Boltzmann route-cost model with .05 lapse probability. Repeated copies
of one intention message are not counted as independent actions. Announced
warnings induce a modeled awareness transition; actual compliance remains
hidden from normal inference and is tested through subsequent choices.

Human route preferences start from a separately stored stale-clear map. Local
sensor observations, voluntary Poisson-timed glances at the shared robot-observation
map, and probabilistic warning compliance update that map. All baselines use
the same attention process, with independent RNG streams for attention, route
choices and compliance. The controller
issues Twist commands toward chosen graph coordinates. Its position and yaw
always come from Gazebo odometry. It never advances pose in Python.
If its last odometry source timestamp becomes more than .5 s old, it sends a
zero-velocity command and waits; cached poses cannot sustain apparent progress.

The logical-camera ROS interface is supplied by maintained Apache-2.0
[`ros_gz`](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_interfaces/msg).
Physical models and plugin dependencies are documented in `hri_sim`.

Local algorithm validation:

```bash
python3 -m unittest discover -s tests -p 'test_reasoning*.py' -v
```

This verifies reasoning and metric math; it does **not** verify Gazebo startup,
ROS bridges or physical sensor telemetry. Those require the integrated smoke
checks and trials in the root project.
