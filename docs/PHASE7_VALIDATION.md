# Phase 7 validation and acceptance status

## Status

The **implementation, test pipeline, exports, documentation, and fresh combined live dashboard acceptance pass**. The **research evaluation is not statistically complete**: the archived baseline contains 20 valid Gazebo runs (n=2 per policy/environment), the β-mismatch conditions contain one run each, and not all requested ablations/robustness conditions have been executed. Do not claim RQ2 efficacy, powered cross-environment consistency, or real-world safety.

The required forest and industrial physical integration paths have previous successful executed acceptance artifacts: forest completion in [Phase 4 validation](PHASE4_VALIDATION.md), industrial obstacle/Nav2/route approval and goal completion in [Phase 5 validation](PHASE5_VALIDATION.md), and the combined live dashboard sequence in [Phase 6 validation](PHASE6_VALIDATION.md). Those records are dated; they are not represented as a new Phase 7 powered experiment.

## Commands executed for Phase 7

| Command | Result |
|---|---|
| `bash scripts/test_local.sh` | Passed |
| Targeted ROS regression: `docker compose run --rm --no-deps sim bash -c 'cd ros2_ws && colcon test --packages-select hri_reasoning hri_industrial --event-handlers console_direct+ && colcon test-result --verbose'` | Passed: 79 reasoning tests and 8 industrial tests; aggregated workspace result: 131 tests, 0 errors, 0 failures, 0 skipped |
| `cd dashboard && npm test` | Passed: 14 tests across 4 files |
| `cd dashboard && npm run build` | Passed: TypeScript and Vite production build |
| `python3 evaluation/phase7_exports.py` | Passed: 20 episode rows, 133,829 raw event records, 20 run configuration records, 8 figures |

The current host is macOS 15.3.2 Apple Silicon. Simulator execution used the existing Ubuntu 24.04 ARM64 / ROS 2 Jazzy / Gazebo Harmonic / Nav2 Docker environment. Native Ubuntu/RViz and real emergency-response hardware were unavailable and are not claimed.

## Measured evaluation

The source matrix is [comparison.json](../results/comparison.json), with per-run [comparison.csv](../results/comparison.csv) and original JSONL logs under `results/trials/{forest,industrial}`. It crosses two environments, five policies (including the privileged oracle reference), and seeds 7 and 17 at 70 simulated seconds. All 20 runs completed with zero measured geometric exposure. B4 averaged one alert, equal to B3 and one more than B1. Means and limitations are in the root [RESULTS.md](../RESULTS.md) and [measured comparison report](../results/report/RESULTS.md).

The two B4 rationality-mismatch runs are in [robustness-comparison.json](../results/robustness-comparison.json) and have one seed each. The baseline lead-time metric has zero eligible entries and is undefined. A single separate accepted mechanism run recorded a warning 10.5 seconds after mismatch onset; see [visible metrics](../results/phase4-report/visible-metrics.json). This is not sensor-to-exposure lead time.

`python3 evaluation/phase7_exports.py` exports:

- `results/phase7-final/episode-metrics.csv`
- `results/phase7-final/per-event.csv`
- `results/phase7-final/experiment-configurations.json`
- `results/phase7-final/summary.json`
- `results/phase7-final/paired-differences.csv` with seed-matched B4-minus-baseline metrics
- Eight figures `01`–`08` in SVG. Figure 7 is analytic cost-threshold sensitivity from the configured equation, not executed simulation. Figure 5 reports unavailable empirical detection-to-exposure samples.

The paired CSV retains individual seed-level deltas. Descriptive paired means, sample SDs and observed ranges for selected contrasts are reported in [RESULTS.md](../RESULTS.md); with n=2 per cell, no inferential intervals or significance claims are supported.

## End-to-end evidence

- [Forest accepted complete run](../results/phase4-visible-verified/result.json) and its initial/blocked/warning/acknowledged/complete telemetry snapshots.
- [Industrial multi-case acceptance](../results/phase5-report/acceptance.json) includes normal, blockage, fire, gas, no-safe-route, known/unaware and collision tests.
- [Phase 6 real dashboard demonstration](../results/phase6-visible/result.json) and [control/reconnect/experiment acceptance](../results/phase6-controls-passed/result.json).
- [Post-fix Phase 7 combined dashboard/Gazebo acceptance](../results/phase7-final/visible-e2e-warning-route-fixed/result.json) passed all 14 checks. The captured [forest warning](../results/phase7-final/visible-e2e-warning-route-fixed/forest-warning-dashboard.png), [industrial warning](../results/phase7-final/visible-e2e-warning-route-fixed/oil-warning-dashboard.png), [approved detour](../results/phase7-final/visible-e2e-warning-route-fixed/oil-approved-detour-gazebo.png), and [assembly arrival](../results/phase7-final/visible-e2e-warning-route-fixed/oil-arrival-gazebo.png) are from the live ROS/Gazebo session. The ground robot's final odometry was `(14.822, 6.144)` for the `(15, 6)` target and Nav2 reported `succeeded`. The warning path and explanation both recommend `junction → north_junction → north_service → assembly`, excluding blocked `corridor_b`. The run also verified warning acknowledgment, emergency stop, pause, and a 1280-pixel laptop layout; telemetry validation reported no errors. Videos and telemetry snapshots are in the same output directory.

These are real Gazebo/ROS results, separate from unit tests and analytic policy calculations. See the linked validation documents for exact scenario commands, successful measurements and retained failures.

## Attempts that failed and were corrected

- The generic VS Code test-discovery tool did not discover Vitest test paths; the repository's own `npm test` command ran all 14 tests successfully.
- Initial dashboard-control acceptance attempts exposed assumptions in the test harness about a preexisting evaluation-toggle value, browser offline simulation, and asynchronous map observation. The harness was corrected and the final control run passed all 10 checks with zero page errors.
- The Phase 7 export generator initially referenced the wrong stored visible-metrics path and wrapper shape. Both were corrected; the final export completed with the counts above.
- Initial fresh dashboard replays exposed test-sequence races: an early route approval arrived before Nav2 activation, and changing scenario selection attempted to select a case before its options rendered. The recorder now waits for Nav2 readiness before choosing/approving a route and waits for the selected case option.
- An early industrial incident trigger left the robot in repeated Nav2 recovery at the approach junction. The acceptance sequence now waits for the robot to reach the junction before injecting the manual incident; the resulting approved detour completed in the fresh run above. The initial failed recordings are retained separately in `results/phase7-final/visible-e2e/`, `visible-e2e-retest/`, and `visible-e2e-final/`.
- The first completed fresh replay exposed a stale-route warning race: the warning text and recommended route could disagree after a hazard update. The runtime now waits briefly for route recomputation and fails safe without recommending a route if the updated route is unavailable; the post-fix live replay above confirms the warning path and explanation agree.
- A later live combined-case replay exposed a separate planner crash at the circular hazard-angle wraparound: floating-point rounding could yield a radius-array index equal to its length. The angle interpolation now wraps the index safely. The targeted reasoning and industrial suites passed, and a fresh combined run with debris, fire, and gas kept publishing a fresh verified southern route; after approval, Nav2 was observed navigating with zero recoveries and the robot moved from its spawn.
- A subsequent replay exposed a route-graph location edge case rather than a process crash: after the robot traveled midway along the southern road, nearest-node routing snapped its pose to the geometrically closer but disconnected `corridor_b` node. Since that corridor was blocked, the planner correctly refused motion but incorrectly missed the drivable southern continuation. Industrial planning now evaluates connectors along verified road segments and plans from their endpoints. Regression coverage passes, and in live combined case `results/oil_facility_runs/1795017105531961000_combined`, all three hazards were observed and a fresh verified route from mid-south-road to Assembly (`south_service → assembly`, using `south_road` and `south_exit`) was produced with no telemetry validation errors. It remains awaiting explicit operator approval.

No unresolved test failure is being relabeled as a successful research run. Historical failures remain in their original artifacts.

## Reproduction and scientific work remaining

See [INSTALLATION.md](../INSTALLATION.md) for verified environment setup and [EXPERIMENTS.md](../EXPERIMENTS.md) for actual trial commands. A powered study still requires at least 30 paired seeds per condition when resources allow, prespecified parameter ranges, broader robustness/ablation coverage, paired uncertainty analysis and calibrated assumptions. The current hardware budget is not enough to complete the requested 300-run baseline matrix in this validation session. Until then, Phase 7 is complete as an honest engineering/research handoff, but **not complete as a statistically conclusive evaluation**.
