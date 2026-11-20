# Phase 3: human beliefs and Bayesian Theory of Mind

The default reasoning pipeline now uses a four-hypothesis, factorized route HMM. It operates on the existing Gazebo forest and ROS 2 telemetry. `tom:=false` selects the preserved two-state prototype; the old baseline experiment runner explicitly selects that mode. The privileged oracle baseline is available only with `tom:=false policy:=oracle`.

## Run the matched demonstration

After building the existing Docker images and workspace:

```bash
docker compose run --rm --no-deps sim bash scripts/build.sh
python3 scripts/forestctl.py start --belief unaware --seed 7 --play
# Repeat the same world, changing only the crew's private initial knowledge:
python3 scripts/forestctl.py start --belief known --seed 7 --play
```

Open the [native Gazebo viewer](http://127.0.0.1:6080/vnc.html?autoconnect=true&resize=scale) and [dashboard](http://127.0.0.1:5173). Start the dashboard with `cd dashboard && npm ci && npm run dev -- --host 127.0.0.1` if it is not running. Without `--play`, press Play at the bottom left of Gazebo. Seed 7 includes two initial two-second waits; the crew subsequently moves and reaches the fork around 11 simulated seconds. The ground robot stays at its lookout position in this experiment; the crew and scripted UAV move.

`forestctl.py pause`, `resume`, `reset`, `replay`, `stop`, and `status` work for both conditions. Reset/replay recreate the Gazebo and ROS processes, private human state, observation memory, and HMM together. Logs go to a new `results/forest_runs/…` directory. `start --case D` switches back to the Phase 2 combined-hazard scenario.

The comparison fixes the world, positions, seed, incident at 1 s, robot prior, and observation parameters. The known crew starts briefed that `trail_a` is closed; the unaware crew retains its old safe map. The tree physically exists before either crew reaches the fork. Warning policy B1 and passive-map attention zero isolate action evidence. Both crews can still discover the tree with their own limited sensor.

## State boundary

| State | Owner / allowed inputs | Normal outputs |
|---|---|---|
| Actual world W | Gazebo physics, forest incident/fire manager | Physical sensors; `/evaluation/truth` for evaluation |
| Private human B | `human_behavior`, its local sensor, optional map attention, received warnings | Commands and observable route declarations; private map only on `/evaluation/human_belief` |
| Robot posterior | `bayesian_tom`, a sanitized public graph, actual crew odometry, public intent, robot observations, warnings | `/hri/belief`, `/hri/tom_evidence`, warnings and route advice |

`tom.py` contains no ROS subscriptions or scenario-file access. `tom_runtime.py` inherits a public-map-only node, not the old `ResearchNode`. Launch writes an allowlisted map containing only `nodes`, geometric `edges`, `start`, and `goal`. It passes **no** hazard list, event schedule, seed, private initial knowledge, actual human beta, or actual compliance to normal inference. A schema rejects extra configuration fields. The actual beta and compliance are independently assigned to the human; the robot has explicitly assumed parameters.

`public_perception` receives only robot/UAV logical-camera measurements. It recognizes static semantic model classes and assigns measured hazard positions to the nearest trail segment. It does not read scheduled hazard coordinates or incident IDs from a scenario file. The new inference does not subscribe to `/forest/fire_state`, `/evaluation/*`, the human's local camera, or incident-injection events. The dashboard does not subscribe to the human's private map. Unit counterfactual tests and `scripts/audit_tom_boundary.py` check these boundaries, including the running ROS graph and public map.

This is an enforced application input/schema boundary, not an OS/DDS security sandbox: the local developer environment and evaluation process retain access to the full filesystem and ROS domain.

## Human policy and action likelihood

Each edge has one of `open_safe`, `blocked`, `open_hazardous`, or `unknown_stale`. A confidently held safe belief can be stale relative to the world while remaining `open_safe` in the human map. `unknown_stale` explicitly represents an uncertain/unverified belief, not knowledge of the real incident.

Edge costs are geometric length multiplied by 1, infinity, 4, or 1.6 respectively. For a candidate next node, cost is the next edge plus the shortest belief-conditioned suffix to safety. The suffix cannot return through the current node. Waiting has cost equal to the best available route plus two equivalent metres; if no route exists, waiting is the sole deliberate action. At safety, the crew stops.

For finite-cost actions:

`P_deliberate(a | B) = exp[-beta (C_B(a) - min C_B)] / sum exp[-beta (C_B(a') - min C_B)]`.

The final distribution mixes this with a configurable uniform lapse over adjacent choices plus waiting: `(1-lapse) P_deliberate + lapse / |A|`. This gives unexpected or suboptimal actions positive likelihood. The human samples this distribution with a dedicated seeded RNG. A lapse can select a believed-blocked edge, but physical collisions remain active. Beta zero makes finite-cost choices uniform; large beta approaches shortest-cost selection. Subtracting the minimum cost prevents exponential overflow. Likelihood updates use log probabilities and maximum subtraction; impossible all-zero evidence retains the prior rather than producing NaNs.

The human uses actual Gazebo logical-camera detections, limited by the sensor's 1.7 rad frustum and the configured observation radius (default 3 m). Out-of-view map entries remain unchanged. Fire/gas produce a hazardous belief; tree/debris produce a blocked belief. An attempted forward motion with less than 8 cm progress over two simulation seconds is an unexpected obstruction; the human marks that edge blocked and replans. Missing/stale odometry stops motion instead of creating an observation. This is a kinematic obstruction heuristic, not a contact-force classifier.

Warnings change the private map with probability `trust` for explicit warnings and `.55*trust` for soft warnings, using a separate RNG. Trust changes remain private. The inference receives the public warning message and predicts acquisition using **assumed** compliance; it never reads the sampled compliance outcome. A separate Poisson attention clock optionally acquires robot map observations. Private belief snapshots and compliance outcomes are logged only for evaluation.

Human route declarations are an explicit favorable observability assumption. A target declaration is not counted as action evidence until independent `/human/odom` shows at least 15 cm movement along that branch. Each decision is used once. High-rate repeated intent messages do not multiply the same likelihood. A missing, invalid, stationary, or stale action is not treated as proof of knowledge. The currently published destination is the known safety task, not an inferred choice among latent goals.

## HMM and bounded hypothesis space

There is one categorical filter per edge (4E stored probabilities, never an enumerated 4^E joint state). Initial prior is `[.55, .15, .15, .15]` in the order above. For each likelihood calculation, other edges use the previous MAP hypotheses. All edge updates use the same pre-update context. This approximation can lose correlations between routes; it is not exact joint Bayesian inference.

The default transition matrix per whole simulation second is:

```text
               safe   blocked hazardous unknown
safe           .998   0       0         .002
blocked        0      .998    0         .002
hazardous      0      0       .998      .002
unknown        .002   0       0         .998
```

`p_pred(j) = sum_i p_previous(i) T(i,j)` and `p_new(j) ∝ p_pred(j) P(observed action | B_edge=j, context)`.

`tom_prior` and `tom_transition` launch arguments accept JSON distributions / a row-stochastic 4×4 matrix. Invalid or nonfinite distributions are rejected. Transitions advance using `/clock`, not wall time or callback count. A backward clock requires a full scenario restart.

Observation opportunities condition the transition: `p' = (1-c)p + c one_hot(observed_state)`. A public warning uses assumed compliance. An estimated human viewing opportunity uses a 0.9 acquisition probability once per observed edge, based on actual crew pose/heading, assumed visibility, and an already robot-observed hazard. An unexpected blockage opportunity uses 0.95 when stalled travel coincides with an observed nearby obstruction. Geometric opportunity is imperfect evidence of acquisition, not a read of the human camera or private knowledge. Receipt/visibility can alter B, so these are transition events, not repeated independent likelihoods. A robot sighting alone changes the robot's hazard estimate and **does not** alter the posterior over human knowledge.

## Published interpretation

`/hri/belief` is JSON in `std_msgs/String`, with a focused edge plus all edge distributions in `beliefs`. It includes four posteriors, entropy (natural-log nats), confidence `1-H/log(4)`, latest relevant evidence and hazard-observation times, movement-evidence count, declared route, and known task destination. Entropy-derived confidence is not a measured probability of correctness.

For a robot-observed **blockage**, incompatible beliefs are `open_safe` and `open_hazardous` (both assert that the trail is open). For an observed **hazard**, the incompatible hypothesis is `open_safe`. `p_outdated` sums that probability mass. `p_mismatch` is this mass only when the declared intended route includes the affected edge; it is zero when irrelevant. An additional upper bound includes `unknown_stale`. Without a hazard observation, mismatch is **null/unassessed**, not zero/safe. Unknown segments are never certified safe. Displayed route beliefs can be selected independently in the dashboard.

Avoidance often supports both `blocked` and `open_hazardous`. The system retains that ambiguity; it does not claim the action uniquely identifies why the human avoided a trail.

## Experiments and evaluation

```bash
docker compose run --rm --no-deps sim python3 evaluation/run_tom_experiments.py \
  --output results/my-phase3-run
```

Each condition starts a fresh physical Gazebo world and records its complete command, scenario, public posteriors, private evaluation truth, odometry, sensor identities, evidence, shutdown log, input audit, and scores. Output paths must be new. Conditions are:

1. Unaware crew, old safe map; physical tree appears at 1 s.
2. Same world and seed, crew already briefed about the blockage.
3. Beta 0, lapse .15, seed 27: noisy/suboptimal action stress case.
4. Robot at (18,0), tree behind its actual camera; UAV detection link withheld from inference. Crew still has its own sensor.
5. Actual beta .15 versus assumed beta 4, seed 27: rationality misspecification.
6. Initially unaware, explicit warning at 13 s; actual and assumed compliance 1 for a controlled acquisition demonstration.

Conditions 1, 2, 4 and 6 use seed 7 by default. Seeds 27 for the two noise examples are selected demonstration cases, not a random population sample. The CLI seed changes the other conditions. This suite establishes functioning mechanisms and exposes model failures; it is not a powered experiment or evidence of a general HRI benefit.

Only `evaluation/tom_metrics.py` joins private beliefs and physical incident truth. It uses latest-at-or-before timestamps; future private updates cannot label earlier predictions. It scores the changed edge after physical incident creation: classification accuracy, average probability assigned to the true hypothesis, multiclass Brier score (sum of four squared errors), ten-bin confidence calibration/ECE, and dangerous-mismatch precision/recall at threshold .7. Missing hazard observations are reported as abstentions and count as false negatives when a true dangerous mismatch exists. Undefined denominators remain null.

Lead time is first threshold crossing relative to the observable branch decision and to entering a 2 m research proximity zone. Negative decision lead is valid: confirmation from movement can arrive after the decision. Proximity is not a validated exposure dose. Time samples are correlated; reported sample counts are not independent subjects. A larger prespecified multi-seed study and empirical human-response calibration remain necessary.

The inverse-planning principle follows [Baker, Saxe & Tenenbaum (2009), *Action understanding as inverse planning*](https://web.mit.edu/9.s915/www/classes/cognition2009.pdf). The route hypotheses, approximate factorization, costs, observation/compliance assumptions, and Gazebo implementation here are project-specific research choices, not a reproduction of that study.
