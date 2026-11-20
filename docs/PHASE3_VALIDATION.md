# Phase 3 validation — 16 November 2026

The Bayesian ToM implementation is integrated with the physical Gazebo forest. Verification covers the algorithm, private/public input boundary, actual movement, six experimental conditions, warning acquisition, and the native GUI/dashboard comparison. It does not establish calibrated human behavior or a general safety benefit.

## Evidence

- [Final evidence bundle](../results/phase3-complete/report.json), including explicit paths to every selected trial and the corrected warning retest. Earlier failed runs are retained, not overwritten.
- [Workspace test log](../results/phase3-colcon-tests.log): **93 checks, zero errors/failures/skips** (92 Python tests plus the CTest wrapper). This includes 23 new Bayesian, boundary, obstruction and evaluation checks.
- [Runtime and source manifest](../results/phase3-runtime-manifest-final.json): Ubuntu 24.04.5 ARM64, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, ros_gz 1.0.24, Python 3.12.3. Host: macOS 15.3.2 / Apple M3; software OpenGL in Docker.
- [Actual ROS subscription/parameter audit](../results/phase3-acceptance/1_unaware/boundary-audit.json): only `/clock`, `/human/odom`, `/hri/human_intent`, `/hri/observations`, and `/hri/warning`; public map has no incident schedule or private knowledge.
- [Matched world/branch assertions](../results/phase3-acceptance/summary.json): identical world configuration except the private initial crew knowledge; different physically confirmed branch choices. That original suite's overall status remains **failed** because its one-shot warning message was lost. The final evidence bundle explicitly substitutes [the passing warning retest](../results/phase3-warning-confirmed/6_warning/result.json).
- [Final GUI recording assertions](../results/phase3-demo-final/result.json), [known crew in Gazebo](../results/phase3-demo-final/known/branch-world.png), [unaware crew in Gazebo](../results/phase3-demo-final/unaware/branch-world.png), and [four-state dashboard](../results/phase3-demo-final/unaware/dashboard.png). Each condition's directory contains actual browser WebM recordings and the captured public ROS state.

The native camera now looks down more steeply so foreground tree crowns do not hide the crew at the fork. The crew is the orange/yellow helmeted proxy; the teal ground robot remains a stationary observer in this experiment. The UAV follows the existing scripted observation patrol.

## Matched result

After physically confirmed branch movement, both simulations have the same blocked trail and the robot has the same prior and observation model:

| Private condition | Observed choice | P(open/safe belief) | P(blocked belief) | P(open/hazardous belief) | P(unknown/stale) |
|---|---|---:|---:|---:|---:|
| Unaware crew | `fork → trail_a` | 83.2% | 0.19% | 0.19% | 16.4% |
| Informed crew | `fork → trail_b` | about 2.4% | about 39.7% | about 39.7% | about 18.2% |

These are posterior values, not hidden truth supplied to the inference. The robot cannot confidently distinguish “blocked” from “hazardous” from detouring alone. The dangerous-mismatch probability is about **83.4%** for the unaware crew's selected blocked route and **0%** for the informed crew's detour. The informed result's unresolved reason for avoidance remains visible in the posterior.

In the corrected warning trial, the public warning arrived at 14.162 simulated seconds. Immediately before it, the safe-belief posterior was .8283; with configured assumed compliance 1, the posterior then transitioned to blocked with probability 1. The private evaluation record independently confirms compliance at 14.166 s and the later route change. The filter did not subscribe to that evaluation record.

## Descriptive metrics

All scores concern the changed trail after physical incident creation. They are correlated simulation time samples, not independent people or trials.

| Condition | Classification accuracy | Mean P(true hypothesis) | Multiclass Brier | ECE | Mismatch precision | Recall |
|---|---:|---:|---:|---:|---:|---:|
| Unaware | .991 | .706 | .165 | .277 | 1.000 | .357 |
| Already informed | .453 | .259 | .838 | .573 | undefined | undefined |
| Noisy actions | .358 | .213 | .928 | .416 | undefined | .000 |
| Missing robot observation | .676 | .424 | .696 | .455 | undefined | .000 |
| Beta misspecification | .358 | .205 | .982 | .448 | undefined | .000 |
| Warning acquisition | 1.000 | .754 | .141 | .246 | 1.000 | .280 |

The noise and rationality cases deliberately expose misleading detours: a crew can choose the longer route without knowing the blockage. The limited-observation case leaves hazard mismatch unassessed and records false negatives instead of declaring safety. The informed case is uncertain before the first discriminating action and retains blocked/hazardous ambiguity afterward. These are material model limitations, not results to discard.

In the unaware condition, threshold detection occurs 0.40 s **after** the declared hazardous branch decision, because the filter requires movement confirmation. It occurs 2.15 s before entry into the defined 2 m hazard-proximity zone. The warning case has a −0.49 s decision lead and +2.149 s proximity lead. This is not a validated exposure/dose measure.

## Commands executed

From `/Users/ali/Documents/HRI/Project`:

```bash
docker compose run --rm --no-deps sim bash scripts/build.sh
docker compose run --rm --no-deps sim python3 -m pytest -q ros2_ws/src/hri_reasoning/test
docker compose run --rm --no-deps sim bash -c \
  'cd ros2_ws && colcon test --executor sequential --event-handlers console_direct+ && colcon test-result --verbose' \
  > results/phase3-colcon-tests.log 2>&1
```

From `dashboard`: `npm run build` passed TypeScript and the Vite production build.

```bash
docker compose run --rm --no-deps sim python3 evaluation/run_tom_experiments.py --output results/phase3-attempt1
docker compose run --rm --no-deps sim python3 evaluation/run_tom_experiments.py --output results/phase3-validation
docker compose run --rm --no-deps sim python3 evaluation/run_tom_experiments.py --output results/phase3-final
docker compose run --rm --no-deps sim python3 evaluation/run_tom_experiments.py --output results/phase3-acceptance
docker compose run --rm --no-deps sim python3 evaluation/run_tom_experiments.py \
  --conditions 6_warning --output results/phase3-warning-confirmed
```

Every condition's `command.json` records its exact expanded `ros2 launch` command, ROS domain and Gazebo partition. `warning_command.json` records the corrected one-shot publisher, which waits for all three consumers, uses reliable QoS, and stays alive for two seconds after publishing. Every successful selected run has advancing physical odometry, nonzero movement, normalized four-state posteriors, movement evidence, a physical incident, a passing boundary audit, clean shutdown, and no critical simulator/ROS errors.

The actual browser recordings were run with:

```bash
NODE_PATH=/tmp/hri-browser-check/node_modules \
PLAYWRIGHT_BROWSERS_PATH=/tmp/hri-browser-check/browsers \
node scripts/record_tom_demo.cjs results/phase3-demo-final

python3 evaluation/report_tom.py --suite results/phase3-acceptance \
  --warning-retest results/phase3-warning-confirmed --gui results/phase3-demo-final \
  --output results/phase3-complete

docker compose run --rm --no-deps sim python3 scripts/record_environment.py \
  --output results/phase3-runtime-manifest-final.json
```

Playwright/Chromium are only needed for automated recording; they are not simulation dependencies. Earlier GUI captures are retained in `phase3-visible-demo`, `phase3-visible-verified`, and `phase3-demonstration`.

## Failures found and addressed

1. A bridge teardown assertion under active sensor traffic: the experiment runner pauses physics and drains callbacks before shutdown.
2. Python subscription errors during ROS context destruction: reasoning, forest and dashboard nodes retain the context until callbacks unwind and nodes are destroyed.
3. A graph-audit discovery race: the audit waits for the node to appear in the ROS graph before querying subscriptions.
4. The original “partial” observer could still see the hazard: the corrected condition places the tree behind the robot's real camera and withholds the UAV detection stream from inference.
5. A one-shot warning exited before all consumers received it: corrected discovery/QoS/lifetime settings passed the dedicated retest, including the private update and public posterior transition.
6. Foreground trees obscured the crew in the wide shot: the camera angle was raised. GUI teardown now closes Gazebo before its X server. Recorder startup checks wait for the native window; an initial check sampled toolbar text and was corrected to sample its background.

Remaining warnings include Gazebo's accepted `gz_frame_id` extension, software-renderer antialiasing fallback, QML layout messages, and OBJ material fallback. They did not prevent the verified scene, cameras, physics or telemetry. No software blocker remains for this Phase 3 demonstration. Remaining research limitations are partial observability, approximate route factorization, non-identifiable beliefs, uncalibrated response parameters, ideal semantic detections and the absence of a larger prespecified study. See [model assumptions and formulas](BAYESIAN_TOM.md).
