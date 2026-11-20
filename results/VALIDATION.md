# Validation record

**Phase 4 acceptance:** [current report](../docs/PHASE4_VALIDATION.md), [machine-readable acceptance](phase4-report/acceptance.json), and [visible forest recording](phase4-visible-verified/result.json). Historical results below remain separately labeled.

Updated 2026-11-16. These are recorded executions, not expected outputs. Failed development attempts are retained alongside the successful runs.

**Subsequent Phase 1 acceptance passed:** [full record](../docs/PHASE1_VALIDATION.md). Actual Gazebo GUI through noVNC, browser controls, emergency stop, command timeout, TF, physical collisions and waypoint movement now pass. Latest regression: 65 reported checks, zero failures. Historical results below retain their original provenance.

## Verified physical vertical slice

- Ubuntu 24.04 ARM64 container, ROS 2 Jazzy, Gazebo Harmonic **8.15.0**, ros_gz **1.0.24**, Nav2 **1.3.13**. Installed packages and source hashes: [runtime_manifest.json](runtime_manifest.json).
- Four ROS workspace packages built. The initial complete package test run passed **39 tests, zero failures**: [colcon-tests.log](colcon-tests.log). The final consolidated run reports **51 tests, zero errors/failures/skips** (50 individual Python cases plus the CTest suite wrapper): [final-colcon-tests.log](final-colcon-tests.log). The updated source/runtime manifest is [runtime_manifest_final.json](runtime_manifest_final.json). Original experiment manifests are preserved.
- All three worlds and eight models pass Gazebo SDF validation: [sdf-validation.log](sdf-validation.log). The terrain mesh was corrected after a real DART importer crash; the corrected forest completed 3,000 physics steps: [terrain-runtime.log](terrain-runtime.log).
- The ROS velocity-command smoke test passed with independent Gazebo pose, wheel odometry, advancing clock, lidar, IMU and 320×240 RGB camera messages. Measured motion was approximately **1.043 m**, with the front lidar distance decreasing by approximately the same amount. See [telemetry.json](telemetry.json) and [smoke-test-output.log](smoke-test-output.log).

## Verified HRI, dashboard and Nav2 paths

- A physical forest pilot created and detected the fallen tree, issued a belief-aware warning, rerouted the simulated responder and reached safety at about 56.8 simulated seconds: [pilot comparison](pilot-comparison.json).
- The real ROS-backed dashboard returned rendered camera bytes, paused/resumed the Gazebo clock, and sent an actual Nav2 waypoint. The robot reached `(10, -2)` while detouring around debris at `(7, -2)`. The measured trajectory passed above the debris at y≈0.36–0.49 m, rather than through its volume: [dashboard-integration.json](dashboard-integration.json), [camera image](dashboard-integration.png), [industrial-runtime.log](industrial-runtime.log).
- React/TypeScript production builds and gateway tests pass. The gateway displays missing/stale data explicitly and contains no integrated mock-data mode. Browser and Gazebo GUI checks were subsequently completed in Phase 1; native Ubuntu/RViz interaction remains untested.
- The operator-adviser integration **passed** using actual ROS observations, Nav2, Gazebo and HTTP controls: [operator-integration.json](operator-integration.json), [runtime log](operator-runtime.log). A waypoint through observed fire triggered a warning; canceling returned Nav2 action status 5; acknowledgment produced an operator event and awareness posterior 1.0 for that hazard; the revised `(2, 0)` goal succeeded with final position approximately `(1.778, 0.005)`. UAV monitoring reported live pose and observation freshness. The pre-acknowledgment posterior was not captured in this check, so it does not establish a measured before/after posterior change. The [first failed attempt](operator-first-failure.json) is preserved; subsequent fixes increased Nav2 action acknowledgment timeout and retained independent per-hazard dashboard beliefs.

## Recorded experiments

**20/20 baseline trials completed:** two worlds × B1/B2/B3/B4/oracle × seeds 7 and 17, each for 70 simulated seconds. A stronger post-run audit found **60/60 scheduled physical incidents created and detected**. Original logs and manifests remain in [trials](trials).

The outcome is a null effectiveness result for these settings: every policy reached safety with zero geometric exposure. B4 and B3 both issued one warning per trial. Relative to B1, B4 increased mean time to safety by about 11.00 s in forest and 11.35 s in industrial. This does not demonstrate improved safety or fewer interruptions. See [measured report](report/RESULTS.md), [comparison JSON](comparison.json), [CSV](comparison.csv), and [plot](report/comparison.png).

Two additional physical sensitivity trials completed with actual rationality β=0.2 and β=3.0 while inference assumed β=1.2, forest/B4/seed 7. Both reached safety with zero exposure and one warning. Times to safety were 56.8 s and 44.2 s; belief accuracy was approximately 0.455 in each. See [robustness-comparison.json](robustness-comparison.json). One seed per altered condition is an instrumentation check, not a robustness claim.

The baseline and sensitivity trials evaluate the physical **responder** loop; they run with `nav2:=false`. They are not a controlled study of the newer waypoint-operator adviser. The [MDP/Q-learning output](communication_policies.json) is an abstract communication-model comparison, not an integrated Gazebo policy result.

## Remaining scope

A larger, prespecified study, calibrated human/perception assumptions, live MDP/Q-learning controllers, autonomous UAV flight, and human-participant validation remain beyond the verified prototype. See [limitations](../docs/LIMITATIONS.md).

## Reproduction

```bash
docker compose build sim
docker compose run --rm sim bash scripts/build.sh
docker compose run --rm sim bash scripts/smoke_test.sh
docker compose run --rm sim bash -c 'cd ros2_ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose'
docker compose run --rm sim python3 evaluation/run_experiments.py --seeds 7 17 --duration 70
python3 evaluation/compare.py results/trials/*.jsonl --output results/comparison.json
```

The archived matrix used separate `results/trials/forest` and `results/trials/industrial` directories and isolated ROS domains for parallel execution. Preserve those original logs; do not replace them with regenerated data. New interactive features and telemetry were added after the matrix, so its original manifests remain the provenance for those runs. The final source offers the same responder conditions with additional independent supervisory features.
