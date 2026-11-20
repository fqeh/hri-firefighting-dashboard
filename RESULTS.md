# Phase 7 final results and status

**Engineering and demonstration acceptance: passed. Statistical research evaluation: incomplete.** All figures and measurements below are derived from executed Gazebo/ROS runs or are specifically labeled analytical. The available sample does not support a claim that belief-aware warnings improve safety or reduce interruptions.

## Executed baseline matrix

The preserved matrix contains **20 complete trials**: forest and industrial × B1/B2/B3/B4/oracle × seeds 7 and 17, each with 70 simulated seconds. All 20 reached the configured goal with zero measured hazard exposure. Each policy/environment cell has n=2; these are engineering checks, not a powered comparison.

| Scenario | Policy | n | Completion | Mean time to safety (s) | Exposure (s) | Alerts/trial | False alarms/trial | Belief accuracy |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Forest | B1 | 2 | 1.00 | 49.62 | 0.00 | 0.00 | 0.00 | 0.66 |
| Forest | B2 | 2 | 1.00 | 49.78 | 0.00 | 1.00 | 0.00 | 0.66 |
| Forest | B3 | 2 | 1.00 | 60.83 | 0.00 | 1.00 | 0.00 | 0.68 |
| Forest | B4 | 2 | 1.00 | 60.62 | 0.00 | 1.00 | 0.00 | 0.68 |
| Forest | Oracle reference | 2 | 1.00 | 60.68 | 0.00 | 1.00 | 0.00 | 0.68 |
| Industrial | B1 | 2 | 1.00 | 49.48 | 0.00 | 0.00 | 0.00 | 0.66 |
| Industrial | B2 | 2 | 1.00 | 49.73 | 0.00 | 1.00 | 0.00 | 0.66 |
| Industrial | B3 | 2 | 1.00 | 60.53 | 0.00 | 1.00 | 0.00 | 0.69 |
| Industrial | B4 | 2 | 1.00 | 60.83 | 0.00 | 1.00 | 0.00 | 0.68 |
| Industrial | Oracle reference | 2 | 1.00 | 60.73 | 0.00 | 1.00 | 0.00 | 0.68 |

These are group means from [the measured report](results/report/RESULTS.md) and [comparison JSON](results/comparison.json). B4 produced one alert per run, same as B3; versus B1 it added one alert and approximately 11.00 s in forest and 11.35 s in industrial. This matrix provides no evidence of improved safety or fewer interruptions. Oracle uses private simulated belief and is a reference only.

The per-seed paired deltas are in [paired-differences.csv](results/phase7-final/paired-differences.csv); values below are B4 minus the comparison policy, reported as mean ± sample SD with the observed range (n=2):

| Scenario | Contrast | Interruption difference | Time-to-safety difference (s) | Exposure difference (s) |
|---|---|---:|---:|---:|
| Forest | B4 − B1 | +1.00 ± 0.00 [1, 1] | +11.00 ± 2.26 [9.40, 12.60] | 0.00 ± 0.00 [0, 0] |
| Forest | B4 − B3 | 0.00 ± 0.00 [0, 0] | −0.20 ± 0.14 [−0.30, −0.10] | 0.00 ± 0.00 [0, 0] |
| Industrial | B4 − B1 | +1.00 ± 0.00 [1, 1] | +11.35 ± 1.91 [10.00, 12.70] | 0.00 ± 0.00 [0, 0] |
| Industrial | B4 − B3 | 0.00 ± 0.00 [0, 0] | +0.30 ± 0.14 [0.20, 0.40] | 0.00 ± 0.00 [0, 0] |

These are descriptive paired summaries only. No inferential confidence intervals or significance tests are reported at n=2; the CSV contains the individual seed-level differences for every baseline contrast.

## RQ findings supported by current evidence

- **RQ1:** The standard matrix records posterior accuracy and Brier scores, but no exposure samples: detection-to-exposure lead time has **n=0 and is undefined**, not zero. The separate robustness runs show B4 classification accuracy ≈0.455 at actual β=.2 and ≈0.455 at β=3 with inference assuming β=1.2; one seed each. A separate accepted single-seed mechanism run issued a warning 10.5 simulated seconds after a dangerous mismatch began. It is not a detection-to-exposure estimate.
- **RQ2:** The 20-run matrix does not show a safety or interruption advantage for B4. A separate Phase 4 matched seed-7 informed/unaware comparison showed the intended mechanism: B4 warned once in the unaware case and stayed silent in the informed case, while B3 warned in both. This is a mechanistic demonstration, not statistical evidence.
- **RQ3:** Both simulation environments and their live interfaces were exercised. The baseline outcomes are similar in this tiny matrix, but two seeds per cell do not establish consistency or transfer.

## Other measured evidence

- An accepted Phase 4 combined forest demonstration recorded one warning, precision 1.0, episode recall 0.5 across two dangerous episodes, zero exposure, evacuation completion, and one simulated acknowledgment/compliant route change. This is a single demonstration, not a comparative estimate.
- The Phase 5 industrial acceptance passed blockage, fire, gas, no-safe-route, known/unaware, normal mission, and collision checks. Nav2 replanned around observed obstacles, required operator route approval, and refused motion when no sufficiently safe route was known.
- The Phase 6 dashboard acceptance passed the live forest/oil workflow, scenario switching, experiment and control checks, and laptop visual layout. See [Phase 6 evidence](docs/PHASE6_VALIDATION.md).
- The post-fix Phase 7 live forest-to-oil-facility dashboard/Gazebo demonstration passed all 14 recorded checks in [result.json](results/phase7-final/visible-e2e-warning-route-fixed/result.json). It captured crew/UAV motion, a real forest incident and acknowledged warning, scenario/epoch switch, an industrial debris warning, a safe approved Nav2 detour, emergency stop, pause, and arrival at `(14.822, 6.144)` for target `(15, 6)`. The warning's recommended path and explanation both identify `junction → north_junction → north_service → assembly`, excluding blocked `corridor_b`; see [warning telemetry](results/phase7-final/visible-e2e-warning-route-fixed/oil-warning.json), [industrial arrival telemetry](results/phase7-final/visible-e2e-warning-route-fixed/oil-arrival.json), [forest warning screenshot](results/phase7-final/visible-e2e-warning-route-fixed/forest-warning-dashboard.png), [industrial warning screenshot](results/phase7-final/visible-e2e-warning-route-fixed/oil-warning-dashboard.png), and [detour/arrival Gazebo screenshots](results/phase7-final/visible-e2e-warning-route-fixed/oil-approved-detour-gazebo.png). These are demonstration results, not extra samples in the policy study.
- Value iteration and frozen Q-learning execute in the abstract communication MDP and have short live integration checks. The recorded Q-table has not converged to the value-iteration Bellman tolerance; model output is not Gazebo outcome evidence.

## Exports and figures

The preserved original trial logs are in `results/trials/`. Per-run summaries are in [comparison.csv](results/comparison.csv) and [comparison.json](results/comparison.json); sensitivity results are in [robustness-comparison.csv](results/robustness-comparison.csv) and [robustness-comparison.json](results/robustness-comparison.json). Phase 7 reproducible exports are in `results/phase7-final/`: per-episode CSV, per-event CSV, run/configuration JSON, machine-readable summary, per-seed [paired-differences.csv](results/phase7-final/paired-differences.csv), and eight SVG figures. Paired means, sample SDs and observed ranges above are descriptive only; no inferential intervals are warranted at n=2. The analytic cost-threshold plot is explicitly not a simulation result.

## Statistical scope and remaining work

No significance test is reported. n=2 per main condition and n=1 per mismatched-β condition are insufficient for reliable uncertainty estimates or efficacy claims. There are no independent human participants. Exposure is a geometric simulation proxy, not dose or injury. Trust and compliance are scripted parameters/outcomes.

To pursue the original research claims, preregister and execute at least 30 paired seeds per scenario/policy/condition; define parameter ranges before viewing results; cross awareness, rationality, observation radius, sensing coverage, trust/compliance and hazard dynamics; add the proposed cost and model ablations; report clustered/paired confidence intervals and censored failures; and validate behavior/perception assumptions before any human study. See [LIMITATIONS.md](LIMITATIONS.md) and [the final research report](docs/FINAL_RESEARCH_REPORT.md).

## Exact validation commands

```sh
bash scripts/test_local.sh
docker compose run --rm --no-deps sim bash -c 'cd ros2_ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose'
cd dashboard && npm test && npm run build
python3 evaluation/phase7_exports.py
```

Current run: local script passed; ROS workspace reported 129 tests, zero errors/failures/skips; frontend reported 14 tests passed and production build passed. Previously recorded full physical forest and industrial acceptance artifacts are linked in [VALIDATION.md](results/VALIDATION.md), [Phase 5](docs/PHASE5_VALIDATION.md), and [Phase 6](docs/PHASE6_VALIDATION.md). A larger powered Phase 7 matrix has not been executed.
