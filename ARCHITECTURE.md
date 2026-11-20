# Belief-Aware Warnings for Human–Robot Firefighting Teams

This is a simulation-based HRI research prototype. Gazebo Harmonic owns physical state, collision, motion, and sensors. Python never advances an alternative environment. A waypoint graph annotates trails in the actual 3D worlds; it is not a grid world.

## Platform decision

The inspected host is macOS 15.3.2 on Apple Silicon, with Python 3.14 and no ROS or Gazebo installation. The selected runtime is Ubuntu 24.04 ARM64 in Docker Desktop, ROS 2 Jazzy, Gazebo Harmonic, ros_gz, and Nav2. Ubuntu-native operation uses the same packages and workspace. Software rendering supports headless checks; native Ubuntu with a GPU is preferable for interactive Gazebo and RViz.

The pairing is documented by [Gazebo](https://gazebosim.org/docs/latest/ros_installation/) and [ros_gz](https://github.com/gazebosim/ros_gz/tree/jazzy). Installation success and simulator execution are distinct validation gates.

## Information boundaries

1. **Physical truth:** Gazebo entities and actual successful incident injection. The scenario manager publishes `/evaluation/truth` for scoring only.
2. **Robot observation:** Gazebo logical-camera detections, lidar, IMU and RGB camera, bridged through ros_gz. Hazard inference consumes observations, never the incident schedule or hidden belief labels.
3. **Human belief:** The behavior node maintains a separate stale route map, updates it from its own restricted sensors or accepted warnings, and selects actions with a Boltzmann model. Its hidden state is published only on `/evaluation/human_belief`.
4. **Inferred belief:** Bayesian hypotheses and an HMM transition model use observable actions and robot evidence. Uncertainty remains explicit. The oracle is separately enabled and labeled; only it may subscribe to simulated hidden belief.

Gazebo's logical camera is an idealized object-identity/frustum sensor, not a trained fire detector. Its limitations, including occlusion behavior, must be considered when interpreting results. Human motion is a physical responder proxy and UAV hover is a constrained lookout, not validated locomotion or flight dynamics.

## Packages and data flow

| Location | Responsibility |
| --- | --- |
| `ros2_ws/src/hri_sim` | SDF worlds, colliding models, sensor and bridge configuration, graph annotations |
| `ros2_ws/src/hri_bringup` | Launch, simulation orchestration, TF, Nav2 integration, actual telemetry checks |
| `ros2_ws/src/hri_reasoning` | Incidents, simulated behavior, Bayesian inference, warning policies, event logging |
| `ros2_ws/src/hri_dashboard` | ROS subscriber and local HTTP/SSE/camera server; no fabricated telemetry |
| `dashboard` | React/TypeScript supervisory interface |
| `evaluation` | Fresh-process seeded runs, structured-log scoring and comparison |
| `tests` | Independent algorithm and configuration checks |

Gazebo → ros_gz → ROS observations → belief posterior → cost-sensitive warning → simulated human update → physical velocity command → Gazebo. The evaluator receives independent truth and observed decisions. The browser receives only supervisory telemetry through a local backend.

The optional industrial operator path is separate: dashboard waypoint → Nav2 action server → actual `/plan` and robot motion. An operator adviser combines that plan with `/hri/observations` and the endpoint request to publish `/hri/operator_belief`, `/hri/operator_warning` and `/hri/operator_routes`. Explicit acknowledgment is observable evidence of reported awareness. The adviser never reads simulated private operator beliefs and never substitutes its graph route for Nav2 control. A mission-monitor node reports UAV pose/sensor freshness and its assigned constrained-lookout mission on `/hri/uav_status`.

Physical robot telemetry uses standard ROS messages (`Odometry`, `LaserScan`, `Imu`, `Image`, `LogicalCameraImage`). Versioned HRI records use JSON in `std_msgs/String`, documented in the reasoning package. This avoids custom-message build coupling during the first vertical slice; schemas and validation still apply.

## Execution and reset

All scenario scheduling and HRI timing use simulation time from `/clock`. A fresh simulator and fresh ROS nodes are launched for each seed/policy combination. Restart resets physics, beliefs, RNG, inference, warning cooldowns and logs together; resetting only Gazebo time is not a valid research reset. Failures and incomplete runs must remain failures, with unobserved metrics represented as null rather than zero.

## Research limits

Synthetic behavior does not establish benefits for real firefighters. Simulated fire/gas zones encode risk, not combustion, fluid dynamics, or toxicology. Proposed benefits require repeated runs, calibrated assumptions, and ultimately ethical human-participant evaluation. A module is only runtime-verified when an actual execution artifact supports it; static assets and unit tests alone do not establish simulator integration.
