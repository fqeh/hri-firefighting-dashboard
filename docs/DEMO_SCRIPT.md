# 5–10 minute demonstration script

Use the [Phase 6 screenshots and walkthrough](PHASE6_VALIDATION.md) if a live simulator is unavailable. Do not describe a recorded image as a current live run.

## 0:00–1:00 — System and evidence

Open the dashboard and Gazebo viewer. Explain that Gazebo is the physical world and the browser's map is a diagnostic route view. Point out the live ROS connection, timestamps and observation/inference/configuration provenance. State that battery and thermal sensing are not modeled.

## 1:00–3:30 — Forest warning mechanism

Select forest case F, policy B4, seed 7, with manual incident injection. Start/resume the scenario. Show the crew moving and UAV observations. Inject the fallen-tree incident. Wait for actual sensor detection, route blockage/recommendation, posterior update and warning. Expand the evidence explanation, acknowledge the warning, and show that acknowledgment does not itself assert hidden belief or physical compliance. Point out the alternate route and eventual completion in the accepted run evidence.

## 3:30–6:30 — Industrial operator workflow

Switch to oil-facility blockage. Request the assembly route and approve its current revision. Inject debris or use the scheduled blockage. Show real robot odometry, LiDAR/camera and route update; acknowledge the warning and approve the safe alternative. Explain that command acceptance is not navigation success and that Nav2 action feedback is the outcome.

## 6:30–8:00 — Evaluation monitor and results

Show the fixed simulation-time experiment monitor and its log links. If truth evaluation is enabled, explain that it uses evaluator-only simulated labels. Summarize the actual 20-run matrix: two seeds per policy/environment, all complete with zero exposure, no demonstrated B4 safety/alert advantage. The oracle is a privileged reference only.

## 8:00–9:00 — Limitations

Close by distinguishing a working research prototype from validated emergency equipment. Mention ideal semantic perception, proxy human behavior, geometric fire/gas zones, no validated flight, and the need for a larger preregistered repeated-seed study and eventual reviewed human study.
