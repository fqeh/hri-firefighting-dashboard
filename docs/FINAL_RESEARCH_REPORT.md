# Belief-Aware Warnings for Human–Robot Firefighting Teams

## Abstract

This project implements a simulation-based human–robot interaction prototype that estimates a teammate's route beliefs from public hazard observations and route actions, then chooses whether to issue a warning. A ROS 2 Jazzy and Gazebo Harmonic simulator supports a forest evacuation and fictional industrial navigation scenario. The supervisor uses actual ROS/Gazebo telemetry. The research evaluation compares passive, proximity, every-change, belief-aware and privileged-oracle conditions. The archived 20-run baseline is an engineering matrix (two seeds per policy and environment), not a powered study. Every baseline trial reached its configured safety goal with zero geometric exposure; B4 issued the same warning count as B3 and approximately one more warning than B1. These runs do not establish a safety benefit or interruption reduction. A separate single-seed known/unaware comparison demonstrates the mechanism qualitatively. Robustness and human-subject claims remain unestablished.

## Research questions and hypotheses

**RQ1:** How accurately and how early can Bayesian Theory of Mind infer outdated hazard beliefs from observable actions?  
**H1:** Posterior mismatch estimates should increase after an observed hazard and a confirmed action along a conflicting route, while remaining uncertain when avoidance has several plausible causes.

**RQ2:** Can B4 reduce missed warnings and unnecessary interruptions relative to conventional policies?  
**H2:** B4 should be selective for an uninformed, route-committed teammate, while B3 may warn already-informed teammates. Safety and interruption effects require adequate paired samples.

**RQ3:** Does warning behavior remain consistent across forest and industrial environments?  
**H3:** Equivalent information and costs should yield comparable policy behavior, subject to different geometry and navigation dynamics.

These are hypotheses, not established findings. The present evidence tests implementation and mechanism; it is insufficient to confirm H2/H3.

## Methodology

### Simulator and system

The runtime uses Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, ros_gz and Nav2. Gazebo owns physical state, collisions, motion, lidar, odometry and RGB imagery. The forest scenario contains trails, fire, a colliding tree, a responder proxy and constrained scripted UAV lookout. The fictional industrial world contains corridors, a differential-drive robot, Nav2, LiDAR, camera, and configurable debris/fire/gas incidents. The dashboard consumes public ROS data through a local gateway and session supervisor.

### Belief and decision models

The simulated human has a stale private route map, limited observation, probabilistic warning response, modeled trust and Boltzmann route choice. Bayesian inference uses public observations and confirmed actions to estimate a four-hypothesis per-route belief. Private truth is isolated to evaluation and the explicitly chosen oracle. B1 shows information with modeled passive attention and no proactive alerts; B2 warns by proximity; B3 warns on relevant observed changes; B4 selects actions from expected loss; oracle has privileged simulated belief and is not deployable.

Cost-sensitive warning decisions are the primary live B4 path. Value iteration and tabular Q-learning are compared on a finite communication model; the observation action approximates information value and is not a physical UAV controller. The trained Q policy is reproducible but does not meet the recorded Bellman residual tolerance. Do not interpret that output as an optimal policy.

### Experimental setup and outcomes

The preserved Gazebo matrix crosses forest/industrial, B1/B2/B3/B4/oracle and seeds 7/17 at 70 simulated seconds: 20 valid trials. The 60 scheduled incidents were recorded as physically created and detected in the source audit. The matrix includes one baseline scenario configuration per environment and only two seeds per cell. A separate forest B4 robustness check uses actual β=.2 and β=3.0 with assumed β=1.2, one seed each. A seed-7 known/unaware paired comparison holds semantic robot observations constant and demonstrates the selective warning mechanism.

Metrics are derived from timestamped JSONL logs and independent Gazebo odometry/truth topics: belief accuracy/Brier score; episode warning precision/recall, false/missed warnings; alert count; exposure/entrapment; completion/time-to-safety; modeled trust/compliance; and route planning/replanning. Null remains null when an outcome is undefined. The full operational definitions are in [evaluation/README](../evaluation/README.md).

### Reproducibility and analysis

Runs use explicit seeds, fresh ROS/Gazebo processes, stored scenario and behavior configuration, config hashes, and run logs. The archived group summaries and Phase 7 exports are linked from [RESULTS.md](../RESULTS.md). No significance test is reported because the main matrix has n=2 per policy/environment condition; repeated posterior samples within a run are correlated and are not independent subjects.

## Results

Every baseline policy reached safety in every archived trial; every run had 0 measured geometric exposure. In forest, mean time-to-safety was 49.62 s for B1, 49.78 s for B2, 60.83 s for B3, 60.62 s for B4 and 60.68 s for oracle. In industrial, the respective means were 49.48, 49.73, 60.53, 60.83 and 60.73 s. B1 issued no warnings, while B2, B3, B4 and oracle averaged one warning per run. These outcomes show no B4 safety or interruption advantage under these settings.

The standard matrix had no physically entered hazards after detection, so sensor-detection-to-exposure lead time has 0 eligible observations and is undefined. In the separate matched seed-7 forest comparison, B4 warned once in the unaware case and not at all in the informed case; B3 warned in both. The B4 posterior for the “open safe” route hypothesis was 2.42% in the informed condition and 83.23% in the unaware condition. The unaware warning arrived 10.5 simulation seconds after the dangerous mismatch episode began. This is a single mechanism demonstration, not a precise lead-time distribution.

The two one-seed rationality-mismatch runs completed with zero exposure and one warning each, but posterior classification accuracy was ≈0.455 in both. This demonstrates execution of a sensitivity condition, not robustness.

## Discussion by research question

**RQ1:** Inference is implemented and the logs support accuracy/Brier measurement. The 20-run report gives mean accuracy per condition but does not establish accuracy on unseen human behavior. No exposure-based lead time was measurable in the base matrix. The one warning-delay example cannot substitute for a distribution.

**RQ2:** The core selectivity mechanism appears in the matched single-seed comparison, but the wider baseline matrix is neutral/unfavorable for B4 warning counts: it matches B3 and exceeds B1. All policies completed safely in the observed matrix, leaving no measured safety difference to attribute to warning strategy.

**RQ3:** Both environments ran and produced comparable high completion in this limited configuration. Two seeds per policy/environment and differences in scenario geometry do not support general claims of cross-environment consistency.

## Ablations and untested factors

The project can compare legacy/no-ToM paths, expected-loss vs value iteration vs frozen Q-learning, known/unaware initial knowledge, and matched vs mismatched rationality. However, the current archived dataset does not fully cross all of them. The following requested ablations are incomplete: systematic no-inference paired comparisons under equal information, a broad assumption-mismatch grid, trust dynamics on/off, UAV observation actions on/off, cost-threshold trial sweeps, sensor-coverage sweeps, and fire-dynamics sweeps. The cost-threshold figure in the Phase 7 export is algebraic sensitivity from the configured expected-loss equation and explicitly not a simulation outcome.

## Threats to validity and assumptions

- A wheeled visual responder proxy is not a walking firefighter; the UAV motion is scripted/constrained, not validated flight.
- The logical camera supplies ideal semantic identity/pose within its frustum; there is no trained perception, visual occlusion model or smoke degradation.
- Fire/gas are configured geometric hazard/exposure models, not combustion, CFD, toxicology or injury models.
- Human belief, rationality, observation, compliance and trust are synthetic assumptions, not calibrated psychological data.
- Route intent and team tracking are unusually favorable/observable compared with realistic operation.
- Seeds do not make distributed ROS/Gazebo trajectories bit-identical across machines. The two-seed matrix is underpowered, and posterior samples within a run are temporally correlated.
- The oracle sees private simulated belief and is not available to an actual robot.
- The project has no human-participant data and does not certify operational safety.

## Conclusion and future work

The project demonstrates a real Gazebo/ROS system, public-evidence Bayesian inference, multiple warning strategies, risk-aware navigation, a live dashboard and reproducible evaluation tooling. The archived results do **not** show that B4 improves simulated safety or reduces interruptions. A publishable evaluation would need preregistered parameter ranges, at least 30 paired seeds per condition, powered sample planning, paired uncertainty intervals, explicit robustness and ablation grids, and model calibration. Any later human study would require institutional review, consent, counterbalanced protocol and no exposure to real hazards.
