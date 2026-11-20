# Belief-aware warning and rescue routing

Phase 4 runs inside the existing ROS 2 Jazzy / Gazebo Harmonic simulation. The default is `tom:=true warning_engine:=expected_loss policy:=B4`. No training runs during launch. `warning_engine:=legacy` preserves the Phase 3 policy; `tom:=false` preserves the earlier two-state experiments.

## Information boundary and ROS interfaces

World state, private human knowledge, and the robot's belief distribution remain separate. `bayesian_tom`, `warning_decision`, and `rescue_routing` receive only the sanitized topology (nodes, edge endpoints, start and safety goal), public observations and actual crew/robot odometry. These nodes do not receive incident schedules, private initial knowledge or Gazebo evaluation poses. `scripts/audit_warning_boundary.py` audits their deployed subscriptions and parameters. ROS process separation is a software information boundary, not DDS security isolation.

| Topic | Producer and purpose |
| --- | --- |
| `/hri/belief` | Bayesian ToM: four hypotheses per edge, uncertainty, dangerous mismatch, evidence timestamps, confirmed movement |
| `/hri/observations` | Public perception: hazards measured by robot/UAV logical cameras |
| `/hri/route_observations` | Public perception: which complete trail samples lie in a camera's observation frustum |
| `/hri/decision` | Policy, action, classification, expected losses, posterior, evidence, explanation and current alternative |
| `/hri/warning` | New soft/explicit warnings only, unique ID, observed edge/state and recommended route |
| `/hri/acknowledgement` | Human receipt acknowledgement; does **not** reveal compliance or private belief |
| `/hri/rescue_route` | Risk-weighted crew route with revision, world waypoints, states, costs and verification status |
| `/hri/robot_route` | Same route schema for the ground robot |
| `/uav/observation_goal` | `PoseStamped` in `world`; reposition the scripted UAV for another actual observation |
| `/hri/control` | Operator approval, waypoint replacement and cancellation |
| `/hri/navigation` | Actual Nav2 request, acceptance, cancellation and completion results |

Only human action generation, the event logger/evaluation scripts, and the explicitly selected `oracle_warning` process read `/evaluation/human_belief`. The normal dashboard does not subscribe to it. The oracle uses the same observed hazards, route planner, costs and response model, adding privileged human knowledge. It is labeled `oracle` throughout its outputs.

The warning interface retains legacy `action: soft|explicit` and adds `warning_action: SOFT_CUE|EXPLICIT_WARNING`. `SILENT` and `OBSERVE` appear on the decision topic and do not masquerade as messages received by the crew. One controlling incident per edge is selected: a physical blockage dominates fire on that edge; all measured fire patches still contribute to routing costs. B3's “hazard change” refers to a controlling edge incident/revision, not every visual flame particle.

## Expected-loss policy

All values are dimensionless **research assumptions**, not empirically elicited human costs. Change `hri_reasoning/config/warning_policy.json` or supply `warning_config:=/absolute/file.json`. Let `p` be the posterior probability of an incompatible belief on an intended route; for a blockage this is `P(open_safe)+P(open_hazardous)`, and for fire it is `P(open_safe)`. When the observed intended route avoids the edge, its decision probability is zero. Hazard severity `s` and urgency multiplier `u` yield:

```
H = s × u × (missed_danger + hazard_exposure)
L(SILENT) = p H
L(alert) = p H (1 − effectiveness)
         + (1 − p) unnecessary_alert × alert_scale
         + interruption + evacuation_delay × alert_delay
         + repeated_warning × previous_alert_count
```

`alert_scale` is 0.5 for a soft cue and 1 for an explicit warning; this fixed modeling convention discounts unnecessary soft alerts. Separate configurable interruption costs are 0.4 and 1.5. Default missed-danger cost is 30, exposure 12, unnecessary-alert cost 4, repeated-warning cost 3, observation cost 1, and evacuation-delay cost 0.08 per second. Urgency multipliers are 0.25/1/3; near and emergency distances are 4 m and 1.5 m. Severity is parameterized by hazard kind. These are deliberately visible assumptions and can be varied for ablations.

`OBSERVE` includes its action cost, modeled evacuation delay, exposure during the delay, and discounted expected minimum subsequent warning loss. A hypothetical binary observation with configured accuracy supplies a Bayesian two-outcome information-value approximation. Runtime repositioning does **not** sample that model to manufacture evidence. The actual ToM posterior changes only after new public sensor information or confirmed crew movement. This observation-value model is an approximation: a camera relocation may not actually reveal human knowledge.

B4 requires at least one discriminating, physically confirmed crew action for a non-emergency belief warning. Until then it may observe if observation loss is lower than silence, or wait. A harmless incident injection alone therefore cannot produce a belief-mismatch warning. With evidence, B4 selects the minimum-loss available action. All policies except passive B1 share configurable emergency override, cooldown and acknowledgement hold rules. Immediate physical danger can override a route-avoidance estimate or earlier acknowledgement; repeated emergency warnings remain rate limited. Acknowledgement is receipt, not certainty that the human complied.

Classification distinguishes no relevant observed hazard, likely aware/avoiding, probably unaware, uncertain belief, immediate danger, repeated suppression and previously acknowledged messages. Explanations name the observed hazard, sensor/time, measured movement, inferred probability, alternative and unknown segments. The current map consists entirely of candidate evacuation corridors; “task relevant” includes all these edges until evacuation completes.

## Finite MDP and Q-learning

`warning_model.WarningMDP` is a communication-decision approximation, separate from Gazebo physics and the HMM. With default configuration it has **270 nonterminal states plus an absorbing completion state**:

- Incompatible-belief probability: five bins (0, .25, .5, .75, 1).
- Severity: three bins (.5, 1, 2).
- Urgency: distant, near, immediate.
- Message memory: new, pending, acknowledged.
- Uncertainty: low or high.

Actions are SILENT, SOFT_CUE, EXPLICIT_WARNING and OBSERVE. Reward is negative immediate expected loss; the implementation equivalently minimizes discounted cost-to-go. The default discount is .9 and a conceptual decision step is 3 seconds. Runtime decisions are evaluated every .5 simulation seconds; cooldowns enforce communication timing. MDP steps do not claim to predict actual travel time.

Each state/action has normalized explicit transitions. Completion occurs with probability .12. Otherwise urgency progresses with probability .25, capped at immediate. An alert reduces the incompatible-belief probability according to its configured effectiveness, with a further urgency-dependent local discovery probability. Probabilities between bins are linearly interpolated, preserving their expected value. Alert receipt leads to acknowledged/pending memory with configured probability .9. OBSERVE uses Bayesian binary-signal outcomes and clears the model's uncertainty bit. Severity remains constant for this finite encounter. These are modeling assumptions, not fitted dynamics or human-subject evidence.

Value iteration uses the Bellman cost backup, maximum residual tolerance `1e-8`, and a finite iteration cap. Failure falls back to expected loss. Tabular Q-learning uses seeded epsilon-greedy sampling and cost-minimizing updates; alpha, epsilon, episode count, step limit and discount are configurable. Train with `evaluation/train_warning_policies.py`, then load the immutable `q_policy.json` using `q_policy:=... warning_engine:=q_learning`. A configuration SHA-256 mismatch or missing table falls back to expected loss. Evaluation uses a separate seed, zero exploration and no table updates. The training report includes reproducibility, Bellman residual and agreement with value iteration; reproducible does not imply converged. Runtime emergency/repeat/evidence guards wrap all three engines identically.

## Simulated response and modeled trust

`config/human_response.json` controls acknowledgement probability and delay, compliance scale, soft-cue multiplier, route-change probability, fatigue and verified-information gain. The compliance probability is `base_compliance × modeled_trust × soft_multiplier` (multiplier 1 for explicit alerts). Receipt and compliance use separate seeded random streams; duplicate warning IDs are ignored. Compliance updates private knowledge; a separate configurable draw decides immediate replanning. Acknowledgement can occur without compliance.

Modeled trust decreases by configured alert fatigue and increases when the human directly verifies information. It remains bounded. This variable is a simulation parameter, **not actual human trust**; its value and compliance outcomes are evaluation-only outputs. The robot's assumed compliance and the simulated human's actual compliance parameters may differ.

## Risk-weighted rescue routing

The graph's corridors correspond to the existing Gazebo trail geometry. Known colliding fallen trees/debris exclude their edges. Fire/gas observations add exposure zones; costs integrate sampled exposure along each segment:

```
edge_cost = (distance + exposure_weight × exposure_integral)
            × (unknown_multiplier if unobserved else 1)
```

The default exposure weight is 3 and unknown multiplier 1.5. The route can therefore be longer than the geometric shortest path. Output includes every cost component and an explanation. Public frustum coverage samples 21 positions along each edge; it uses Gazebo's ideal logical sensors and does not model foliage occlusion. Unknown space is explicitly conditional. A connector from the crew's measured position to the graph is separately checked against observed obstruction envelopes. A missing connected route yields `safety: unavailable`; an alternative with unknown segments yields `conditional`, never verified-safe. There is no automatic hazard expiry or claimed clearance of a previously blocked edge in these scenarios.

The crew retains its own belief-based route; the public recommendation does not secretly overwrite private knowledge. A warning communicates the observed state and may produce a compliant replan. Actual crew drive commands still encounter active Gazebo collision geometries.

For industrial/oil-field compatibility the same schema supplies robot waypoints. With `nav2:=true`, the operator sends `{"command":"approve_route","revision":N}` to approve the current route, or `{"command":"waypoint","x":2,"y":0}` to replace it. Approval uses `world` waypoints; manual replacements use `odom`. Stale revisions are rejected. A newly observed blockage/hazard on approved edges cancels the route and asks for renewed approval. Queued replacements wait for Nav2 cancellation before being dispatched. Nav2 uses actual lidar obstacle/inflation costmaps and the existing latched velocity guard. The initial `world → odom` transform uses the configured spawn origin; this is not SLAM or drift-free localization. Full oil-field rescue behavior remains future work.

## Baselines and evaluation

B1 is passive. B2 warns by physical proximity. B3 warns for each relevant controlling hazard revision. B4 uses the inferred posterior. Oracle replaces that estimate with true simulated human knowledge. All share the same public hazard/route inputs and warning response interface; only oracle subscribes to private belief. In the matched test the tree, static robot/UAV poses, seed and sensor-visible hazard geometry are held constant. OBSERVE is disabled for this comparison so policies cannot change robot observations. Sensor receipt timestamps are asynchronous; semantic observations, rather than byte-identical ROS streams, are checked for equality.

Only initial crew knowledge differs between the known and unaware cases. This necessarily changes observable crew behavior, which is the intended ToM evidence. Warning precision uses the human's **pre-response** knowledge, avoiding logger-order bias when compliance updates arrive immediately. Recall is per dangerous belief episode; a miss counts even if later direct observation saves the crew. No-alert precision and no-danger recall are null. Metrics include interruptions, simulated acknowledgement/compliance, time to warning, physical exposure/entrapment, completion/censoring, blocked-route rejection and replanning latency. Exposure is time inside research hazard envelopes; entrapment is ten seconds stalled there. Neither is a validated injury/dose model. Seed-7 paired demonstrations establish mechanism behavior, not statistical superiority or human-subject validity.
