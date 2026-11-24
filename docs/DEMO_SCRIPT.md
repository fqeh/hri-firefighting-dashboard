# 5–10 minute demonstration script

## Presentation demo (single-screen dashboard, current)

1. **Introduce the actors.** Use the figures in the presentation: lookout (UAV or robot), human (crew or operator), hazards, goal.
2. **Crew unaware.** Scenario → **Crew unaware** → **Load scenario & policy** → wait for the load → Layers: hide **Trees** → **▶ Resume**.
   - About 5 s: the crew walks from Start towards Fork, skipping the north route (Trail D). The ToM counts it as evidence that they think A or B is open.
   - About 9 s: the tree falls on Trail A (t = 8 s), the UAV sees it (*Observe* turns red), and B4 warns at once (P ≈ 69 % above the bar). Pause here to explain the floating belief map (W2: ToM + HMM) and *Should the lookout speak?* (W3).
   - About 13 s: the crew turns to Trail B and the belief map turns Trail A red (`=`). **Acknowledge** the warning.
3. **Crew knows.** Same steps with **Crew knows**: no warning. In *Same moment, four policies*, B3 would warn but B4 stays silent: the false alarm that erodes trust (W1·D2).
4. **Escalation (optional).** In F, inject the tree on Trail B and fire + smoke on Trails C and D. When the fourth route closes, **No escape route · Send operator** appears. Press **Send operator**.
5. **Oil facility.** Case *unaware*, B4. In Robot, set East assembly point via Corridor b → Request → Approve. About 8 s: debris, then a warning to the operator → Acknowledge → approve the new revision. *no_safe* shows the robot refusing to move when no safe route exists.

See [DEMO_DASHBOARD.md](DEMO_DASHBOARD.md) for every control.

## Earlier script (tabbed console)

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
