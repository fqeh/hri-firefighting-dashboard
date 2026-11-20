# Reproducible evaluation

## Phase 4 runners

Use `run_warning_experiments.py` for the current five-policy known/unaware matrix, `train_warning_policies.py` for the finite MDP and frozen Q baseline, and `summarize_warning_results.py` to combine explicit retests with retained failure provenance. These use the four-state ToM and actual Gazebo crew movement. See [current definitions and assumptions](../docs/WARNING_POLICY.md) and [measured Phase 4 results](../docs/PHASE4_VALIDATION.md). `warning_metrics.py` uses private labels only for evaluation, pre-response knowledge for alert precision, dynamic fire truth for exposure, and the first **blockage** observation for blockage replanning latency.

The scripts and cost/metric definitions below describe the preserved earlier `tom:=false` baseline, not the current Phase 4 matrix.

These scripts consume or launch **real Gazebo + ROS 2 trials**. They do not
implement a Python environment. Unit-test fixtures verify arithmetic only and
must never be reported as simulation outcomes.

After building the Ubuntu container and ROS workspace, run from `/work`:

```bash
source /opt/ros/jazzy/setup.bash
source /work/ros2_ws/install/setup.bash
python3 evaluation/run_experiments.py --scenarios forest industrial \
  --policies B1 B2 B3 B4 oracle --seeds 7 17 27 --duration 90
python3 evaluation/compare.py results/trials/*.jsonl --output results/comparison.json
```

A full design uses at least 20 paired seeds per condition. The three seeds above
are a small pipeline check, not adequate power for a research conclusion. Policy
and seed are crossed within scenario. Each trial starts a fresh launch with its
own Gazebo transport partition and ROS domain. This resets physical entities,
hazard schedules, RNG and beliefs together. A Gazebo reset service alone does not
reset all ROS-side state and is intentionally not offered as an experimental
reset. Stop interactive simulations before batch runs to avoid GPU contention.

The runner waits for timestamps from independently logged **Gazebo odometry** to
reach the fixed simulation duration; it does not mistake wall time or a node
republishing a cached position for simulated outcomes. It fails if launch exits,
physical human pose telemetry stops, a hazard injection fails, sensor streams are
missing, or the wall-time budget expires. Once the first incident is scheduled,
that incident must have both a successful Gazebo creation event and a
sensor-derived detection. Every subsequent incident due in the scenario manager's
simulation-time schedule must also have a successful Gazebo creation record.
Detection of later incidents is reported but is not required: a limited viewing
frustum can legitimately miss them. It writes `run_end.valid=true` only after these checks
and the duration criterion. Existing logs are preserved
unless `--overwrite` is requested. The output manifest records parameters,
Gazebo/Python versions and hashes of reasoning, worlds, models, configuration,
launch files, scripts and container build sources. Exact floating-point
trajectories can still differ by processor, render/physics versions and message
scheduling; seeded model decisions and configured incident schedules are
reproducible, not a claim of bit-identical distributed simulation.

Audit existing frozen trial logs with the current stricter incident validator:

```bash
python3 evaluation/audit_incidents.py results/trials/forest/*.jsonl \
  results/trials/industrial/*.jsonl --output results/incident-audit.json
python3 evaluation/report.py results/comparison.json \
  --incident-audit results/incident-audit.json --output results/report
```

The audit adds an analysis artifact with original-log hashes; it never rewrites
run-end labels or replaces the source manifests recorded when trials ran.

For robustness to incorrect behavioral assumptions:

```bash
python3 evaluation/run_experiments.py --scenarios forest --policies B4 \
  --seeds 7 17 27 --human-betas 0.2 1.2 3.0 --assumed-beta 1.2 \
  --output results/robustness
```

Vary `--observation-radius`, `--trust`, `--assumed-beta`, and `--passive-map-rate`
separately. Keep
matched seeds across policies, record all failures, and choose parameters before
examining outcomes. Changing simulated compliance is a sensitivity analysis,
not evidence about actual firefighter trust.

## Baselines and models

| Name | Rule |
|---|---|
| B1 | Show all received robot observations and routes, with simulated voluntary map glances; send no proactive warnings. |
| B2 | Warn about a relevant observed hazard within 4 m of the human. |
| B3 | Warn once for every newly observed relevant incident. |
| B4 | Minimize expected communication cost using posterior outdated-belief probability. |
| oracle | Same costs and robot observations, but access true simulated human belief. |

B2/B4/oracle use a 10 s incident cooldown. B3 warns once per incident ID; a spread
event has a new ID. All policies share the same sensor stream, human response
model, route graph and incident schedule. The oracle is explicitly isolated:
normal `belief_warning` nodes do not subscribe to `/evaluation/human_belief`.

All five conditions include the same **passive map attention model**: the human
voluntarily checks the shared robot-observation display at Poisson-distributed
times with rate `passive_map_rate=.05` per simulated second (mean interval 20 s).
At a glance, only already received `/hri/observations` are incorporated into the
human's private map, possibly triggering a route change. Merely receiving an
observation message does not imply that the human has looked. There is no access
to `/evaluation/truth`. This makes B1 a modeled show-everything/no-alert
condition, rather than a no-communication condition. Setting the rate to zero
explicitly disables voluntary glances for a sensitivity analysis. Exponential
inter-arrival sampling makes the event schedule independent of controller tick
frequency; route choice, warning compliance and map attention use separate,
deterministically seeded RNG streams. A public `passive_map_glance` event records
the glance without publishing hidden belief contents. This attention rate is a
declared simulation assumption, not a measured firefighter behavior rate.

B4's costs are: missed dangerous mismatch 30, false explicit warning 4, soft
interruption .4, explicit interruption 1.5. Expected effective mismatch reduction
is .35 for soft and .85 for explicit messages. These are declared modeling
assumptions, not fitted human data. The simulated human responds with probability
`trust * .55` to soft messages and `trust` to explicit warnings; initial trust is
.8. Every interruption reduces modeled trust .005; a new personal confirmation
increases it .01. This intentionally imperfect policy model can be examined in
the robustness analysis.

`policy_comparison.py` performs value iteration and tabular Q-learning on an
11-state **communication MDP**. It writes a file explicitly labeled
`communication_model_analysis_not_simulation_results`. State bins represent
mismatch probability; actions are silent, soft, explicit and observe. The
observation transition assumes perfect information and costs 1. It does not
physically reposition the UAV and is not advertised as an integrated active
perception policy. The live B4 controller uses the initial cost-sensitive policy.

```bash
python3 evaluation/policy_comparison.py --seed 42 --output results/policy_model.json
```

## Metric definitions

Only complete integrated trials are included in group summaries. Groups isolate
scenario, policy, actual/assumed rationality, initial trust, observation radius, passive-map rate,
requested duration and scenario-configuration hash; robustness conditions are
never silently pooled. Undefined
ratios and unobserved outcomes are `null`, never fabricated zeroes. CSV includes
each run, JSON includes run data and mean/sample standard deviation with the
number of non-null samples. These summaries do not establish significance.

| Metric | Operational definition |
|---|---|
| Belief accuracy | Fraction of posterior samples whose `p(unaware) >= .5` agrees with evaluator-only human knowledge for an active incident. |
| Brier score | Mean squared probability error against that same hidden label. |
| Detection lead time | First physical hazard-volume entry time minus first sensor detection time, for incidents eventually entered. `null` if no such entry; negative means detection was late. |
| Exposure | Integrated actual human pose time inside any active incident radius; overlapping volumes counted once. |
| Entrapment | At least 10 s of near-stationary motion inside an active hazard radius; not equivalent to failing the time limit. |
| Time to safety | Independent Gazebo odometry first enters the .65 m goal tolerance, minus its first logged pose time. Null for a valid trial that times out; `time_to_safety_censored` explicitly flags this. |
| Mission completion | Goal tolerance reached within the valid trial duration. |
| Warning precision | Warnings issued while active incident lies on declared intended path and hidden belief still marks its edge clear, divided by all non-silent warnings. |
| Warning recall | Distinct dangerous-mismatch incidents receiving at least one appropriate warning, divided by all distinct dangerous-mismatch incidents. |
| Missed warnings | Dangerous-mismatch incidents without an appropriate warning. |
| False alarms | Non-silent warnings without a currently dangerous outdated-belief mismatch. |
| Interruptions | Number of soft plus explicit messages. |
| Modeled trust | Last recorded bounded scalar in the simulated response model. |
| Simulated compliance | Complied responses divided by warning responses. |

Exposure, entrapment and goal completion use `/human/odom` directly, independently
of the human behavior node's intentions or completion claims. Entrapment uses a
speed threshold of .05 m/s over the stated 10 s interval. Detected incidents that
are never entered have undefined lead time rather than an invented positive
lead; `lead_time_samples` makes that selection visible. A single mean lead time
therefore cannot measure the benefit of avoiding exposure altogether.

Precision uses messages; recall uses incident-level opportunities. They are
deliberately different units and must not be combined into an undocumented
message-level F1 score. Metric evaluation uses truth topics; policy inference
does not. Logs are sorted by sim stamp, with warnings evaluated before response
belief updates at the same stamp, to limit subscriber arrival-order artifacts.

## Research interpretation and limits

Testable hypotheses: B4 reduces interruptions relative to B3 while preserving
mission completion and reducing exposure relative to B1; benefit decreases when
the inference model assumes the wrong human rationality. The oracle estimates
the value of perfect belief knowledge while holding robot perception fixed.

The logical camera is an ideal semantic sensor with a viewing frustum and no
occlusion model. It recognizes configured incident IDs and associates them with
pre-annotated route edges; there is no learned fire classifier. Human intentions
are observable declared next waypoints/path choices, not private beliefs.
Fire/gas hazards are geometric exposure regions attached to physical Gazebo
entities; they are not CFD or thermal injury models. A responder is represented
by a colliding wheeled proxy with a human-shaped visual, not a biomechanics
model. The UAV uses the simulation asset's declared hover constraint. These
assumptions sharply limit real-world transfer. This is a simulated HRI study,
not a safety certification or a human-subject experiment.
