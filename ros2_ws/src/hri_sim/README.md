# Native Gazebo simulation assets

`minimal.sdf` is the first telemetry/actuation milestone. It contains one physics-driven differential-drive robot, a box at `(5, 0)` and a side wall. `forest.sdf` and `industrial.sdf` add the colliding responder proxy, the elevated UAV lookout, and actual 3D scenario objects. No Python environment simulation exists: the Python generator only writes SDF and mesh files; Gazebo alone calculates motion, collisions and sensors.

All included resources resolve offline when `GZ_SIM_RESOURCE_PATH` includes the installed `hri_sim/models` directory. Assets are authored for this project and licensed Apache-2.0. There are no downloaded model dependencies.

## Geometry and topic contracts

The robot starts at `(0,0,0.02)` with a wheeled chassis using Gazebo's maintained DiffDrive system. `/robot/cmd_vel` is a `geometry_msgs/Twist` command. Gazebo produces wheel odometry `/robot/odom`, the `/robot/tf` transform, lidar `/robot/scan`, IMU `/robot/imu`, camera `/robot/camera/image`, and logical detections `/robot/detections`. `config/bridge.yaml` contains the Gazebo→ROS mappings. A separate OdometryPublisher emits actual simulator position on `/evaluation/robot_pose`; this is evaluator-only data and must not be used by belief inference.

DiffDrive publishes its planar body frame as `base_link`. The lidar sensor is at `(0.18,0,0.49)` relative to the model origin, and the camera is at `(0.41,0,0.41)`. The named physical chassis link is centered 0.24 m above that origin. The launch package supplies the corresponding static TFs.

The human responder starts at `(0,-2,0.02)`. A helmet and torso identify the responder visually, but locomotion uses a physical wheeled proxy, not human gait biomechanics. `/human/odom` is actual Gazebo world pose from OdometryPublisher; the DiffDrive wheel-integrated odometry is deliberately separate on `/human/wheel_odom`. Human observation radius is limited by a logical camera with a 4 m far plane, with a configurable additional radius cap in the behavior node. Camera orientation and field of view also restrict perception.

The UAV is a colliding static quadrotor-shaped model at `(6,0,8)`. It represents a stabilized hover observation station: aerodynamics, battery and flight control are **not implemented**. Its downward logical camera and RGB camera generate live Gazebo observations. `/uav/odom` comes from Gazebo OdometryPublisher. Mission-state semantics belong to the scenario supervisor.

## Worlds and incident models

The forest contains 21 colliding trees, an authored triangulated 3D hill mesh with matching collision mesh, a flat traversable trail network, and a marked safety zone. The industrial site is fictional and contains storage tanks, buildings, pipes, roads, a gate and evacuation zone. Both JSON route graphs use meters in the Gazebo world frame; every edge has a matching trail/road visual in the world.

`hazard_tree` is an already-fallen, solid log. Injecting it dynamically blocks `trail_a`; the falling motion itself is not modeled. `hazard_debris` is a solid obstruction. `hazard_fire` and `hazard_gas` are non-solid observable incident volumes: a person can enter them, and the evaluator measures exposure rather than an artificial collision stopping them. Fire growth is discrete insertion of additional Gazebo hazard entities from the scenario schedule, not a combustion, fluid, toxicology or heat-transfer model. Smoke is a translucent 3D visual approximation.

Gazebo logical cameras report objects inside their frustum and expose model identities and poses. They are an idealized semantic observation sensor, **not** RGB fire recognition, line-of-sight ray tracing, or an occlusion-aware perception pipeline. The belief node must consume only these sensor detections and observable actions, never the hidden human belief or scenario manager's incident truth. Ground-truth pose topics must be named and used accordingly in research claims.

## Reproduction and validation

Regenerate assets deterministically with:

```bash
python3 ros2_ws/src/hri_sim/scripts/generate_assets.py
python3 -m unittest discover -s ros2_ws/src/hri_sim/test -v
```

These tests validate XML, scene/graph alignment, physics/sensor presence, collision semantics and local resource paths. They do **not** prove that Gazebo loaded a sensor or that the robot moved; see the repository's runtime smoke test and recorded validation status for that evidence.

## Maintained upstream dependencies

- [Gazebo Sim, `gz-sim8`](https://github.com/gazebosim/gz-sim/tree/gz-sim8), Apache-2.0: Physics, DiffDrive, Imu, Sensors, LogicalCamera, UserCommands and OdometryPublisher. The [upstream differential-drive example](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive.sdf) documents the controller interface; this project supplies its own smaller chassis geometry.
- [Gazebo logical-camera example](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/logical_camera_sensor.sdf), Apache-2.0, documents the sensor configuration.
- [ros_gz Jazzy bridge](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge), Apache-2.0, lists the supported `LogicalCameraImage`, odometry, image, IMU and lidar conversions.
- [OdometryPublisher API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html) documents simulator-derived 3D pose publishing.

On a Linux desktop with the ROS workspace sourced, open diagnostics beside the simulation:

```bash
rviz2 -d "$(ros2 pkg prefix --share hri_sim)/config/diagnostics.rviz" --ros-args -p use_sim_time:=true
```

This diagnostic view uses actual `/robot/scan`, `/robot/odom`, `/robot/camera/image` and TF. RViz is a diagnostic viewer; Gazebo remains the 3D environment and physics simulator.

The real `gz sdf -k` parser has also validated all 3 worlds and 8 model files in the Jazzy/Harmonic container; evidence is stored in `results/sdf-validation.log`. Reproduce from the container with:

```bash
bash ros2_ws/src/hri_sim/scripts/validate_sdf.sh /work/ros2_ws/src/hri_sim
```

Standalone `gz sdf` uses `SDF_PATH` for include resolution; `gz sim` uses `GZ_SIM_RESOURCE_PATH`. The parser reports known extension warnings for `gz_frame_id`, which is retained for Gazebo sensor frame naming. All files pass validity checks. Actual sensor delivery and actuation require the separate runtime smoke test.

Terrain importer regression: the generated OBJ includes one explicit unit normal per expanded triangle vertex. This is required by the installed Gazebo DART/ODE mesh conversion; omitting normals caused a real importer crash during validation. The corrected forest completed 3,000 physics iterations in Gazebo Sim 8.15.0 with headless rendering (exit 0), recorded in `results/terrain-runtime.log`. The local suite now checks mesh normals as well as scene structure.
