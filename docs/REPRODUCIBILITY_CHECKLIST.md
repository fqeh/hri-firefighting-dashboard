# Reproducibility checklist

- [ ] Record host OS/CPU, Docker version, image ID, ROS/Gazebo/Nav2 package manifests and source hashes.
- [ ] Use the project-pinned Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic environment.
- [ ] Save scenario JSON, warning-policy and human-response config, seed, policy, actual/assumed β, trust, observation radius, passive map rate, requested duration and config hash.
- [ ] Launch a fresh process group for every trial; never reset only Gazebo physics/time.
- [ ] Retain complete, failed, timed-out, crashed and censored runs in separate records; do not overwrite prior logs.
- [ ] Verify independent Gazebo/ROS physical odometry, clock and required sensor streams.
- [ ] Confirm every due physical incident was created; log detection separately because frustum-limited non-detection can be valid.
- [ ] Score evaluator-only truth offline; do not expose private belief labels to normal policies or public logs.
- [ ] Keep oracle results visibly labeled as privileged reference results.
- [ ] Report null for undefined precision, recall, lead time and censored completion; do not convert null to zero.
- [ ] Keep analytical MDP/cost calculations and unit-test calculations separate from Gazebo measurements.
- [ ] Use paired seeds, report the actual n, and avoid significance claims with the current small matrix.
- [ ] Rebuild and run local tests, ROS package tests, and scenario acceptance checks.
- [ ] Preserve CSV/JSON/event-log exports, screenshots, recordings, commands and any acceptance failure provenance.

Reproduction commands and the verified runtime are in [INSTALLATION.md](../INSTALLATION.md), [EXPERIMENTS.md](../EXPERIMENTS.md), and [RESULTS.md](../RESULTS.md).
