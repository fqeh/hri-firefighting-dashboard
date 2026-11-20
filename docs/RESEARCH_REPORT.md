# Research protocol and interpretation

## Question and hypotheses

Can an embodied robot infer that a teammate still considers a changed route clear, and intervene when that mismatch matters? Hazard detection alone does not establish that a warning is useful: the teammate may already know, may intend another route, or may be uncertain.

Prespecified comparisons are B4 versus B2 and B3 for warning count/false alarms at comparable simulated completion and exposure, and B4 versus B1 for simulated safety. The oracle is a reference with privileged simulated human knowledge, not a deployable policy. Null findings are valid outcomes. Do not claim that synthetic results establish real-world firefighter safety.

## Study apparatus

The apparatus uses Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic. The forest has colliding terrain, trees, trails, responder proxy, lookout and injectable fallen-tree/fire models. The fictional industrial site has tanks, pipelines, buildings, corridors, gates, a mobile robot and debris/fire/gas incidents. Scripts schedule physical entities through Gazebo services. Hazard zones define exposure for evaluation; there is no combustion, plume-dispersion or toxicology model.

The robot's logical camera produces object identities inside a frustum. This is an idealized semantic sensor; the prototype does not train fire recognition, and logical-camera occlusion behavior is not equivalent to a real visual detector. RGB images and lidar are independent Gazebo sensor outputs used for supervision and navigation. The UAV is a constrained hover lookout with real pose/sensors, not a validated autonomous aircraft. The responder is a wheeled physical proxy, not biomechanical walking.

## Human and inference models

The human starts with a stale map and chooses next graph actions with a Boltzmann distribution over route cost. The graph's nodes are fixed world coordinates. Local observations and accepted warnings alter the private map. Trust and compliance are explicitly modeled parameters, not psychological measurements.

Normal inference observes declared route intent and sensor evidence. It maintains two hypotheses per observed incident—aware versus outdated clear belief—with uncertainty represented by their posterior and binary entropy. An HMM models acquisition/forgetting; action likelihoods depend on an assumed rationality. The inference process has no subscription to hidden human-belief labels except under the separately selected oracle policy. Static semantic map annotations associate observed model identities with affected trails; event timing and actual existence come from sensor evidence rather than the schedule.

The cost policy minimizes expected missed-warning, false-alarm and interruption losses. The MDP/value-iteration and Q-learning comparison uses an explicit abstract communication transition model. It is an independently tested algorithm comparison and is not an integrated UAV sensing controller or a physical safety result.

## Baselines and fairness

All policies use the same simulator, incident schedule, observation channels and scenario parameters. B1 provides the supervisor with observed information without proactive alerts. B2 warns near a relevant observed hazard. B3 warns once for every relevant newly observed incident. B4 selects warnings from estimated dangerous belief mismatch and asymmetric costs. Oracle selects from actual simulated human belief.

Passive dashboard viewing and warning compliance are separate assumptions. A scripted person does not automatically perceive a browser panel merely because the panel exists. The behavior configuration and experiment metadata must specify the passive viewing process; a zero viewing rate means no passive map consultation. Compare policies under identical viewing assumptions and report sensitivity to that rate.

Use paired seeds across policies, fresh processes per run, separate robustness conditions, and fixed simulated observation windows. Record package versions, source hashes, image ID, seed, rationality, trust, observation radius, viewing rate and trial duration. Same seeds do not guarantee bitwise identical asynchronous ROS schedules or physics across machines. Do not pool different behavior assumptions into the same aggregate.

## Outcomes and validity

Report posterior classification accuracy and Brier score; incident-level warning precision/recall; missed and false warnings; interruptions; directly measured time to safety and completion; geometric exposure duration; explicitly defined entrapment proxy; modeled trust and simulated compliance. Detection lead time exists only when both detection and a later/earlier physical exposure are observed; report the sample count and retain null for never-exposed hazards.

An unreachable or unobserved goal is not a zero-second completion. Trials ending before safety are censored, with a null time-to-safety. Trial completion means the requested observation window finished, not that the mission succeeded. A stalled clock, missing physical pose, failed incident injection or missing required sensors invalidates a run. These are engineering failures, not evidence for or against a warning policy.

The default local observation radius can let the person detect a tree before exposure even under B1. Such scenes can demonstrate belief inference or interruption differences without measurable safety improvement. Report that outcome rather than tuning scenarios after seeing results to favor B4.

## Results

Actual execution evidence and any measured trial summaries are linked from [the validation record](../results/VALIDATION.md). This document does not substitute illustrative numbers for completed runs. The analytic communication-policy output is labeled separately from Gazebo experiments.

The completed engineering matrix contains 20 valid runs (two seeds × five policies × two scenarios, 70 simulated seconds each), with all 60 scheduled incidents created and detected. Every policy reached safety with zero geometric exposure. B4 issued one warning per run, equal to B3, and took about 11 s longer than B1. Thus these settings show no B4 safety or interruption benefit. See [the measured report and figure](../results/report/RESULTS.md).

Two further forest/B4 trials used β=0.2 and β=3.0 while inference assumed β=1.2. Both completed with zero exposure and one warning, with times to safety 56.8 s and 44.2 s. Each condition has only one seed; this verifies sensitivity instrumentation rather than establishing robustness.

The newer interactive operator adviser uses actual waypoint requests, Nav2 plans and robot observations, with explicit acknowledgment and cancellation. It is evaluated through a separate integration check. The responder matrix does not measure operator-belief accuracy or operator decision-making, and there is no hidden ground-truth label for a real dashboard user's belief.

For a substantive term-project study, run at least the prespecified repeated-seed matrix and sensitivity conditions. Report paired differences, uncertainty intervals and failed/censored trials. A small engineering smoke matrix is useful for validating instrumentation but is insufficient for broad statistical conclusions.

## Extensions before a human study

Validate perception under smoke and occlusion, expand belief hypotheses and model uncertainty, use a maintained flight platform and articulated responder, calibrate trust/compliance on data, and validate industrial navigation under changed obstacles. A human-participant study would require appropriate university review, informed consent, training, counterbalanced policy order, workload/trust measures and no exposure to real emergency hazards.
