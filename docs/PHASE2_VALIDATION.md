# Phase 2 validation — 2026-11-16

**Implemented and exercised in real Gazebo.** A visible tree-blockage demonstration was recorded from the native Gazebo GUI through noVNC. The current scene is paused just after rerouting, ready for inspection or continuation.

## Evidence against the requested tests

| Test | Recorded evidence |
| --- | --- |
| Forest loads with correct models | Native forest GUI renders terrain, vegetation, robot, crew, UAV, trails and safety zone; real sensors publish. Existing asset-contract tests also pass. |
| UAV position observable through ROS | All three physical runs received actual Gazebo UAV odometry and camera images; maximum displacement from start was 7.211 m. |
| Crew moves through world | Maximum physical displacement: 11.716 m in A; 11.774 m and 11.672 m in the two D runs. |
| Fire changes over simulation time | D records changing radial fire state; acceptance requires over 1 m growth. Unit checks separately verify fuel dependence, wind asymmetry and deterministic stepping. |
| Fallen tree appears and blocks trail | D creates the log at 8.0 s. A test robot driven into it stops at x≈6.161 m with 61 fresh contact samples. Collisions stay enabled. |
| Affected route unavailable | `trail_a` becomes `blocked` and is excluded from recommendations. |
| Alternative when available | GUI demonstration changes from `start → fork → trail_a → safety` to `fork → trail_b → merge → safety`. Remaining uncertainty is explicitly reported. |
| Same-seed replay | Both D runs record fire at 4.0 s and tree at 8.0 s, with identical configured event IDs/order/times. Physical trajectories differ slightly; bit-identical physics is not claimed. |

The physical acceptance suite ran A once and D twice. B/C/E/F scenario definitions and the E/F prior-knowledge difference are tested in the model suite; C additionally ran in the recorded GUI demonstration. No separate physical B/E/F effectiveness comparison is claimed.

- [Physical acceptance report](../results/phase2-attempt2/acceptance.json), including exact ROS launch commands, observed event times and per-run results.
- [Visible demonstration result](../results/phase2-visible-demo/result.json): original recommendation at 2 s; blocked route and alternative at 10 s; no browser page errors.
- [Before](../results/phase2-visible-demo/before.png), [after](../results/phase2-visible-demo/after.png), and [recorded Gazebo video](../results/phase2-visible-demo/forest-rerouting.webm).
- [Actual UAV camera image](../results/phase2-uav-camera.png), [paused-dashboard check](../results/phase2-visible-demo/dashboard-check.json).
- [Regression log](../results/phase2-colcon-tests.log): **70 reported checks passed, zero errors/failures/skips** (69 Python cases plus the CTest suite wrapper). [Frontend production build](../results/phase2-dashboard-build-final.log) passed.
- [Final runtime manifest](../results/phase2-runtime-manifest-final.json), preserving package versions and source hashes.

All three final physical test launches and shutdowns have zero critical error matches. Known nonfatal SDF/Gazebo rendering warnings remain in the simulator logs. The first physical test attempt failed because the probe assigned an integer zero to a ROS floating-point velocity field; this was corrected to `0.0`. The original failed attempt is retained under `results/phase2-attempt1`. Host Python lacked pytest; model/regression tests ran in the existing ROS container instead.

## Commands used

From the repository root, with build/test output redirected to the linked logs:

```bash
python3 scripts/create_forest_scenarios.py
docker compose run --rm sim bash scripts/build.sh
docker compose run --rm sim python3 scripts/test_forest.py --output results/phase2-attempt2
docker compose run --rm sim bash -c 'cd ros2_ws && colcon test --executor sequential --event-handlers console_direct+ && colcon test-result --verbose'
npm --prefix dashboard run build
python3 scripts/forestctl.py start --case C --seed 7
NODE_PATH=/tmp/hri-browser-check/node_modules PLAYWRIGHT_BROWSERS_PATH=/tmp/hri-browser-check/browsers node scripts/record_forest_demo.cjs results/phase2-visible-demo
NODE_PATH=/tmp/hri-browser-check/node_modules PLAYWRIGHT_BROWSERS_PATH=/tmp/hri-browser-check/browsers node scripts/check_forest_dashboard.cjs
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec gui /hri-entrypoint.sh python3 scripts/record_environment.py --output results/phase2-runtime-manifest-final.json
```

The browser recording uses the temporary Playwright 1.65.1 tooling installed for Phase 1 and requires the local dashboard dev server. Acceptance output directories must be new to preserve provenance.

## Qualifications

The UAV uses explicitly labeled pose-service kinematics, not validated flight dynamics or PX4 SITL. Fire is an uncalibrated fuel/wind radial-front research model, not wildfire prediction. Fire/smoke geometry is a coarse visual projection. The fallen tree is dynamically placed already fallen, not a toppling-fracture simulation. The humanoid crew is a wheeled proxy. Logical observation identities are idealized and do not model occlusion.

The route alternative in the recorded demonstration is **conditional**, because some segments and the connector from the crew require verification. The system explicitly reports that no fully verified route is available; an alternative recommendation is not a safety certification. The graph planner includes exposure cost; that cost is not automatically injected into the separate Nav2 controller. See [full assumptions and control interfaces](FOREST_SCENARIO.md).
