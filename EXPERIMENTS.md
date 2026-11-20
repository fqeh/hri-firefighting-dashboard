# Experiments

## Research questions

- **RQ1:** How accurately and how early can Bayesian Theory of Mind infer outdated route beliefs from public hazard observations and confirmed route actions?
- **RQ2:** Do belief-aware warnings reduce missed dangerous mismatches or unnecessary interruptions versus passive, proximity and every-change baselines?
- **RQ3:** Does behavior transfer consistently across the forest and fictional industrial scene?

The testable direction is that B4 should avoid warning an already-informed or route-avoiding teammate, while warning an unaware teammate when the expected cost of silence is larger. The oracle is a privileged simulated-belief reference, **not a realizable robot policy**.

## Policies and equal information

| Policy | Operational rule |
|---|---|
| B1 | Passive interface and modeled voluntary map glances; no proactive warning |
| B2 | Proximity warning for a relevant observed hazard |
| B3 | Warn on each relevant observed hazard change |
| B4 | Expected-loss decision using posterior outdated-belief probability and configured asymmetric costs |
| Oracle | B4-style reference with evaluator-only simulated belief; never a deployable policy |

All non-oracle policies use the same observed hazard stream, behavior model, route graph and incident schedule. Value iteration and frozen Q-learning can be selected in the industrial live integration, and were also assessed in the abstract communication MDP. They are separate decision variants, not independent human-subject conditions; the Q-table's Bellman residual shows it is not converged to the VI tolerance. Offline communication-MDP results are analytic/model results, not Gazebo safety measurements.

## Conditions and reproducibility

The archived baseline uses seeds 7 and 17, both environments, five policies, and 70 s requested duration: 20 complete Gazebo trials (n=2 per policy/environment). Each run stores actual simulation metadata and a configuration hash. Repeat runs with fresh processes; do not reset Gazebo alone. Preserve incomplete runs and distinguish launch failure, timeout, invalid telemetry and task failure.

The measured robustness data has only two additional physical B4 trials: actual human β=0.2 or 3.0 while inference assumes 1.2, one seed each. Trust, radius, observation coverage, transition parameters, compliance, hazard dynamics and cost values have not been comprehensively crossed. The source configuration specifies single default cost values, not prespecified robustness intervals; the Phase 7 cost figure is an **analytical sensitivity calculation on an explicitly analyst-selected grid**, not an executed sweep.

At least 30 seeds per condition were not run. The existing 20-trial matrix took a full simulator setup/launch per trial; 30 seeds × five policies × two environments would require 300 complete fresh Gazebo trials, beyond the feasible single-session validation budget. These archived samples are engineering evidence only and are not powered for efficacy.

## Metrics

Per-run measurements are defined in [evaluation/README](evaluation/README.md): posterior accuracy/Brier score, mismatch precision/recall, detection-to-exposure lead time, true/false/missed warning counts, interruption count, simulated exposure/entrapment, time-to-safety, completion, modeled trust/compliance, and route planning latency. Undefined denominators and unobserved outcomes are `null`; censored runs are not assigned fabricated completion times.

The evaluated standard matrix includes no physically exposed hazard, so its detection-to-exposure lead time has zero eligible samples and is undefined. A separate single-seed accepted mechanism run recorded a warning 10.5 simulated seconds after a dangerous mismatch episode began. That is warning-response delay, not sensor-to-exposure lead time.

## Run and export

The historical baseline is reproduced using:

```sh
docker compose run --rm sim python3 evaluation/run_experiments.py --scenarios forest industrial --policies B1 B2 B3 B4 oracle --seeds 7 17 --duration 70
python3 evaluation/compare.py results/trials/forest/*.jsonl results/trials/industrial/*.jsonl --output results/comparison.json
python3 evaluation/audit_incidents.py results/trials/forest/*.jsonl results/trials/industrial/*.jsonl --output results/incident-audit.json
python3 evaluation/report.py results/comparison.json --incident-audit results/incident-audit.json --output results/report
python3 evaluation/phase7_exports.py
```

The Phase 7 export consumes existing logs only; it does not launch or create trials. For matched informed/unaware policy checks, robustness commands, metric details, and the optional model-training separation, see [evaluation/README](evaluation/README.md), [warning policy](docs/WARNING_POLICY.md), and [Phase 4 validation](docs/PHASE4_VALIDATION.md).

The export also writes `results/phase7-final/paired-differences.csv`, with per-seed B4-minus-baseline deltas for interruption count, time to safety, exposure, and mission completion. Reported paired means, sample standard deviations, and observed ranges are descriptive; with only two seeds per cell, no inferential interval or significance claim is justified. The [results report](RESULTS.md) gives selected summaries and the CSV retains every paired observation.
