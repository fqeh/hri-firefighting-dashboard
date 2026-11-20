# Phase 4 validation — 17 November 2026

Implementation includes posterior-based warning decisions, an expected-loss fallback, finite MDP/value iteration, frozen tabular Q-learning, risk-weighted rescue routing, explanations, simulated acknowledgement/compliance/trust, and Nav2 route approval/replacement. The mathematics and information boundaries are documented in [WARNING_POLICY.md](WARNING_POLICY.md).

## Tested runtime

macOS 15.3.2 / Apple M3 host; Docker Linux ARM64; Ubuntu 24.04.5; ROS 2 Jazzy; Gazebo Harmonic 8.15.0; ros_gz 1.0.24; Nav2 BT Navigator 1.3.13; Python 3.12.3 inside Docker. Gazebo uses Mesa 25.2.8 llvmpipe software rendering. The native Gazebo GUI is served through local noVNC; the React dashboard consumes actual ROS data through port 8080. Runtime/source hashes are saved in `results/phase4-report/runtime_manifest_final.json`.

## Verified matched comparison

[Machine-readable report](../results/phase4-report/comparison.json) preserves both the initial failures and explicitly selected retests. All ten selected trials pass movement, posterior/decision/route publication, physical incident creation, blocked-route rejection, 80 simulated seconds, clean logs and clean exit checks. Semantic robot hazard observations are identical across the ten runs. The tree event occurs at 1 s; both robot and UAV remain stationary and OBSERVE is disabled for this comparison. All other parameters are identical except the crew's initial knowledge. Receipt timestamps are asynchronous.

| Policy | Warnings: informed / unaware | False alarms: informed / unaware | Missed dangerous episodes: informed / unaware | Evacuation time: informed / unaware |
| --- | ---: | ---: | ---: | ---: |
| B1 passive | 0 / 0 | 0 / 0 | 0 / 1 | 48.25 / 59.55 s |
| B2 proximity | 1 / 1 | 1 / 0 | 0 / 0 | 48.25 / 48.25 s |
| B3 hazard change | 1 / 1 | 1 / 0 | 0 / 0 | 48.35 / 48.35 s |
| B4 belief aware | 0 / 1 | 0 / 0 | 0 / 0 | 48.35 / 49.00 s |
| Oracle | 0 / 1 | 0 / 0 | 0 / 0 | 48.45 / 48.25 s |

For B4, the first confirmed branch movement was fork → trail_b in the informed case and fork → trail_a in the unaware case. The posterior assigned **2.42% versus 83.23%** to “believes Trail A open and safe.” Beliefs that a route is blocked or hazardous remain partly ambiguous when either explains avoidance; the system does not claim certainty that the informed crew knows the exact incident. The unaware B4 warning occurred at 11.5 s, 10.5 s after the dangerous episode began. The informed crew received no unnecessary warning.

All crews completed evacuation; B1-unaware spent 7.8 s inside the blockage envelope before local discovery and replanning. Every other selected run had zero measured exposure and no run met the defined entrapment criterion. Among warnings actually sent, simulated compliance and immediate route-change rates were 100% for this seed. Precision/recall are 1/1 for B4-unaware; precision is undefined without alerts and recall is undefined without a dangerous episode. Replanning computation averaged approximately 0.10–0.18 ms per plan. Individual reports retain maxima, first-detection-to-route latency, acknowledgement counts, route revisions and censored outcomes.

This is a seed-7 mechanism demonstration, not a statistical comparison or a human-subject result. Exposure is a research envelope metric, not predicted injury. B3 warns earlier than B4 in the unaware condition; B4 trades that delay for avoiding the informed-condition interruption.

## Algorithms, runtime engines and information access

- Workspace tests: **117 reported checks, zero errors/failures/skips**, including the CTest suite wrapper; `results/phase4-tests-complete.log`.
- Frontend TypeScript and Vite production build pass.
- [MDP report](../results/phase4-mdp/report.json): 271 states; value iteration converges in 23 iterations, residual `6.48e-9`.
- Q-learning: 18,000 seeded training episodes; repeated training reproduces the same table/policy. Separate seed 1007, 1,000 frozen evaluation episodes and zero exploration. Agreement with value iteration is 93.70%; discounted modeled losses are 11.164 (VI) and 11.293 (Q). **The Q table has not converged to the VI tolerance**: Bellman residual 176.11. It is an experimental comparison, not a validated optimal controller.
- `results/phase4-value-iteration-live` and `results/phase4-q-learning-live`: each engine passed a 25 s real Gazebo integration run and issued one warning. These are deliberately short engine checks; evacuation completion is censored there. The ten-policy matrix uses expected loss.
- [Normal boundary audit](../results/phase4-normal-boundary.json): inference, warning and rescue routing processes receive only allowlisted public topics/maps.
- [Oracle boundary audit](../results/phase4-oracle-boundary.json): only the oracle warning process adds private human-belief input; ToM and route inputs remain public.

Tests cover expected losses, posterior-sensitive decisions, emergency override, repeated-warning suppression, acknowledgement without compliance, cost-sensitive observation, invalid configuration, malformed/incompatible Q-table fallback, stochastic MDP transitions, terminal rewards, VI convergence, seeded Q training/frozen evaluation, blocked/fire/unknown route handling, no available route, evaluation ordering and growing-fire exposure. Earlier foundation/HMM/sensor/model/gateway tests remain included.

## Actual ground-robot navigation

[Nav2 integration evidence](../results/phase4-navigation-verified/result.json) includes independent Gazebo world pose and a trajectory. A stale route revision was rejected. An approved route moved the robot; observed physical debris then caused cancellation and required approval of the alternative. The approved alternative was replaced by the operator goal `(2, 0)` in odometry coordinates. Nav2 canceled the previous goal, accepted the replacement and reported success. The final independent world pose was `(1.707, -0.044)` m, within the explicit 0.45 m physical tolerance. Initial approved-route displacement exceeded 0.35 m. The final run exited cleanly with no critical log lines.

This verifies the shared route interface, real Nav2 control and operator approval/replacement in the industrial world. It does not establish a complete future oil-field rescue mission, global localization accuracy or operational safety.

## Complete visible forest demonstration

Final accepted recording and live telemetry: `results/phase4-visible-verified/`. The recorder starts case D (fire at 4 s, physical fallen tree at 8 s), enables real UAV observation requests and follows the crew until it reaches the safety zone. It checks the initial route, actual blockage, alternative, confirmed movement evidence, warning, acknowledgement, completion, browser errors and simulator critical logs including pending service failures after pause.

The accepted replay contains:

| Simulation time | Observed policy/route change | Trail A incompatible-belief probability |
| --- | --- | ---: |
| 0.90 s | SILENT; initial route via Trail A; no observed hazard | Prior only; no detected mismatch |
| 8.54 s | OBSERVE; fallen tree observed; alternative via Trail B | 69.1%, still uncertain |
| 11.55 s | EXPLICIT_WARNING after confirmed fork → Trail A movement | 83.4% |
| 11.96 s | Acknowledgement received; subsequent updates suppress repeats | 16.7% after modeled warning acquisition |
| 49.40 s | Crew controller reports completed at the safety zone | 0.71% |

Exactly one warning was sent, with one acknowledgement and a compliant route change; physical exposure was zero. The conservative per-incident metric gives recall 0.5 in this **combined fire/tree** case: the tree warning also diverts the crew from the fire, but does not count as a separate fire warning. The single-tree matched trials above have their own separately reported recall. Final videos, screenshots and snapshots are all retained; browser and simulator critical-error checks pass.

The initial recorded sequence in `results/phase4-visible-final/` reached safety at 49.44 s, with SILENT/no observed hazard at 0.884 s; OBSERVE/uncertain belief with Trail A blocked at 8.55 s; EXPLICIT_WARNING/probably unaware at 11.649 s; and an acknowledged warning followed by suppression at 12.1 s. Its log review found a pending footprint update timeout after pausing, so that attempt is explicitly marked failed despite successful evacuation and screenshots. The final accepted replay uses the corrected footprint controller; its exact times and captures are in its `result.json`, with every sampled posterior/decision/route in `history.json` and videos of both Gazebo and the dashboard.

The teal ground robot remains a sensor platform in this forest warning demo. The crew drives physically and the UAV moves through scripted Gazebo pose control. Ground-robot motion was separately verified with real Nav2 above. Collisions remain active. The shortest graph route is replaced by the lower-exposure alternative; unobserved portions are marked conditional, not certified safe.

## Commands executed

Run from `/Users/ali/Documents/HRI/Project`; use new output directories for reproducible runners. The per-run `command.json` files contain exact ROS arguments, configuration copies, ROS domains and Gazebo partitions.

```bash
# Build and tests
docker compose run --rm --no-deps sim bash scripts/build.sh
docker compose run --rm --no-deps sim bash -lc 'source /opt/ros/jazzy/setup.bash && source /work/ros2_ws/install/setup.bash && cd ros2_ws && colcon test --executor sequential --event-handlers console_direct+ && colcon test-result --verbose'
npm --prefix dashboard run build

# Communication model and physical experiments
docker compose run --rm --no-deps sim python3 evaluation/train_warning_policies.py --output results/phase4-mdp
docker compose run --rm --no-deps sim python3 evaluation/run_warning_experiments.py --policies B4 --knowledge unaware --duration 25 --output results/phase4-pilot2
docker compose run --rm --no-deps sim python3 evaluation/run_warning_experiments.py --duration 80 --output results/phase4-comparison
docker compose run --rm --no-deps sim python3 evaluation/run_warning_experiments.py --policies B1 B4 --knowledge known --duration 80 --output results/phase4-known-retest
docker compose run --rm --no-deps sim python3 evaluation/run_warning_experiments.py --policies B4 --knowledge unaware --engine value_iteration --duration 25 --output results/phase4-value-iteration-live
docker compose run --rm --no-deps sim python3 evaluation/run_warning_experiments.py --policies B4 --knowledge unaware --engine q_learning --q-policy /work/results/phase4-mdp/q_policy.json --duration 25 --output results/phase4-q-learning-live
PYTHONPATH=ros2_ws/src/hri_reasoning python3 evaluation/summarize_warning_results.py --runs results/phase4-comparison results/phase4-known-retest --output results/phase4-report/comparison.json

# Physical Nav2 approval, cancellation, reapproval and replacement
docker compose run --rm --no-deps sim python3 scripts/verify_rescue_navigation.py --output results/phase4-navigation-verified

# Native GUI plus dashboard recording (uses the existing temporary Playwright installation)
docker compose stop sim
NODE_PATH=/tmp/hri-browser-check/node_modules PLAYWRIGHT_BROWSERS_PATH=/tmp/hri-browser-check/browsers node scripts/record_warning_demo.cjs results/phase4-visible-verified

# Live software information-boundary audits
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec -T gui /hri-entrypoint.sh python3 scripts/audit_warning_boundary.py --output results/phase4-normal-boundary.json
docker exec -e ROS_DOMAIN_ID=51 project-sim-run-8fa28791b369 /hri-entrypoint.sh python3 scripts/audit_warning_boundary.py --oracle --output results/phase4-oracle-boundary.json
# The oracle command ran during that trial; its ephemeral container no longer exists.
```

Final build output is in `results/phase4-build-verified.log`; final test output is in `results/phase4-tests-complete.log`. The source manifest was recorded with:

```bash
docker compose -f compose.yaml -f compose.gui.yaml --profile gui exec -T gui /hri-entrypoint.sh python3 scripts/record_environment.py --output results/phase4-report/runtime_manifest_final.json
```

The recorder itself runs `python3 scripts/forestctl.py start --case D --policy B4 --engine expected_loss --observe true --human-radius 1 --map-rate 0 --lapse 0 --trust 0.9 --seed 7`, then `resume` and `pause`. The one-metre human radius and no passive map glances make the limited-awareness mechanism observable. They do not alter robot observation access or collision geometry.

To watch again without recording:

```bash
python3 scripts/forestctl.py replay
# Open the viewers, then press Gazebo Play or:
python3 scripts/forestctl.py resume
```

Open [native Gazebo](http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale) and [the ROS dashboard](http://127.0.0.1:5173). If needed, start the dashboard using `cd dashboard && npm run dev`. At completion the robot/crew may be stationary because the mission has ended; replay restarts controllers and simulation together.

## Failures retained and fixes

- `phase4-pilot`: an empty `q_policy:=` ROS launch argument was rejected. Optional empty launch arguments are now omitted. `phase4-pilot2` passed.
- Initial B1-known: Python interruption during a native ROS subscription take surfaced as a pybind conversion exception on shutdown. Reasoning nodes now request loop exit from signal handlers, finish the callback and then release the ROS context. The B1-known retest passed.
- Initial B4-known: the installed `parameter_bridge` aborted during shutdown with a glibc mutex assertion. The exact retest passed without a bridge change. **This intermittent upstream shutdown risk is not proven eliminated**; its original log is retained. It occurred after the full physical trial, not during movement or inference. Upstream has [bridge shutdown/lifetime fixes](https://github.com/gazebosim/ros_gz/pull/977), but this work does not silently replace the supported installed ROS/Gazebo packages with a different distribution.
- `phase4-navigation`: the probe submitted before the Nav2 lifecycle was active. It now waits for activation. `phase4-navigation-ready` exposed the separate cancellation-result deadline and a probe ordering race. The installed Nav2 default was 20 ms; `default_cancel_timeout` is now 2,000 ms, consistent with the [documented cancellation parameter](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_bt_navigator/#default_cancel_timeout). `phase4-navigation-final` retained the early obstacle-cancellation probe failure. The corrected `phase4-navigation-verified` passed, including actual cancellation and replacement.
- First GUI attempt: this project's headless service occupied port 8080; it was stopped before switching to GUI. The first recorder waited for decision telemetry while physics was paused; it now resumes before waiting. Both attempts' outputs are retained.
- `phase4-visible-final`: redundant stationary UAV footprint service calls could still be pending at pause. The controller now updates footprint position only after actual motion, and the recorder checks for delayed critical errors before accepting a run.
- The combined-incident metric was corrected to start blockage-replanning latency at the first observed blockage, rather than an earlier fire on the same edge. A regression test covers this.
- The summary helper initially mishandled a null movement-evidence field; it now explicitly handles the initial no-evidence state. This did not alter the experiments.

## Remaining research limits

The demonstrated expected-loss path works without RL training. The Q baseline remains unconverged. One seed is insufficient for comparative efficacy claims. Fire propagation/exposure, human rationality, trust and compliance are configurable research models; UAV motion is scripted kinematics. Logical-camera coverage omits foliage occlusion, and routes through unknown space remain conditional. Acknowledgement does not establish compliance. Nav2 uses local odometry and observed lidar obstacles, not a certified map or drift-free localization. The intermittent installed-bridge shutdown assertion above remains a reliability caveat; final selected acceptance logs are clean.
