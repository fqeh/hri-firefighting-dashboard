# Implementation and validation gates

Each gate has an independent acceptance check. The live status belongs in `results/VALIDATION.md`; this plan does not claim any gate passed.

1. **Inspect and choose runtime:** record host, existing files, compatible upstream versions/licenses, and Docker availability.
2. **First vertical slice:** build Ubuntu/Jazzy/Harmonic environment; launch a colliding differential-drive robot in minimal SDF; bridge clock, odometry, IMU, lidar and image; assert increasing timestamps, finite measurements, and motion caused by ROS velocity commands. Save measured evidence.
3. **Forest environment:** actual colliding terrain/trees/trails, physical human proxy and UAV sensor lookout. Inject a blocking tree through Gazebo and observe its identity in sensor output.
4. **HRI loop:** separately model human belief, infer from observable actions, issue a targeted warning and record compliance/rerouting. Unit-test information boundaries, posterior updates, uncertainty and asymmetric costs. Validate integrated events from a real run.
5. **Supervisory dashboard:** compile React/TypeScript; check HTTP live/stale states and camera decoding; verify actual ROS telemetry and commands end to end.
6. **Industrial environment and navigation:** tanks, pipelines, buildings and incident zones in Gazebo; Nav2 receives robot TF/scan/odom, operator waypoint goals and replans around physical obstacles. Verify an actual goal outcome before claiming navigation completion.
7. **Research comparisons:** B1–B4 and oracle with repeated paired seeds; independent truth labels, time-to-safety, warnings and exposure measures; explicitly label missing/censored runs and assumption sensitivity. No invented result tables.
8. **Research handoff:** exact build/launch/reset/experiment commands; evidence and blockers; model assumptions, reproducibility metadata, limitations and future human study design.

Advanced MDP and Q-learning policies can be tested independently, but are not substitutes for passing the physical integration gate. If a dependency or rendering backend blocks execution, retain exact failure output and continue independently testable components.
