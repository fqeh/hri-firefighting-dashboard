# Phase 5 validation record

**Phase 5 acceptance passed.** The consolidated machine-readable record is [acceptance.json](../results/phase5-report/acceptance.json). Earlier failures are retained, with explicit references to their passing replacements.

All five project packages build. Colcon reports **125 checks, 0 failures, 0 errors, 0 skipped** (124 pytest cases plus the CTest wrapper). TypeScript and the production dashboard build pass. All seven required demonstration behaviors have passing physical or GUI evidence. No unresolved critical errors appear in the selected acceptance logs.

## Demonstrated results

- Normal Nav2 navigation completed both route goals; independent world pose ended at `(2.769, 0.007)` for target `(3, 0)`.
- Dynamic debris was observed by the robot, canceled an approved route, and triggered a recommendation through Corridor C. Motion resumed only after approval.
- Stationary forward LiDAR changed from about `18.19 m` to `5.22 m` after physical debris appeared.
- The independent contact fixture kept collisions enabled and applied sustained forward velocity. The robot stopped at `x=5.000 m`; 60 fresh contact samples stayed within 4 cm.
- Fire excluded the original route; gas excluded the north alternative; combined blockage/gas/debris produced no sufficiently safe route and approval was refused without motion.
- Value iteration and the existing frozen Q-table both ran live through the industrial interface with no fallback. This establishes integration, not Q-learning convergence.
- The unchanged forest B4 scenario passed a 25-second regression with physical crew movement, posterior, warning, and route outputs. This short regression is not a new full forest evacuation trial.
- The native GUI recording used actual dashboard destination, approval and acknowledgement controls. The robot moved before the incident, stopped at the changed route, followed the approved northern detour and reached the assembly point. Final wheel-odometry pose was `(14.864, 6.131)` for target `(15, 6)`.

[Blocked road and alternative in Gazebo](../results/phase5-visible/blocked-warning-gazebo.png) · [warning/dashboard capture](../results/phase5-visible/blocked-warning-dashboard.png) · [assembly arrival](../results/phase5-visible/assembly-arrival-gazebo.png) · [visible demonstration result](../results/phase5-visible/result.json). Videos and timestamped snapshots are in `results/phase5-visible`.

The GUI is left **paused at the completed assembly arrival**. Use `python3 scripts/scenarioctl.py replay --play` for a fresh run, then select a destination and approve the recommendation in the dashboard. Merely resuming the completed run does not assign a new mission.

## Matched operator comparison

The physical debris, stationary robot and semantic observations matched across selected trials. Seed 7 reproduced the configured event sequence. Only the oracle consumed private operator beliefs.

| Policy | Alerts: operator informed | Alerts: operator uninformed |
|---|---:|---:|
| B1 passive dashboard | 0 | 0 |
| B2 proximity | 0 | 0 |
| B3 hazard change | 1 | 1 |
| B4 belief-aware | 0 | 1 |
| Oracle | 0 | 1 |

B4's first estimated incompatible-belief probability was `0.405` when the operator chose the safe alternative, versus `0.867` when the operator chose Corridor B. The informed case was classified as likely aware or avoiding the hazard and remained silent; the uninformed case produced an explicit warning. The model retains ambiguity between blocked, hazardous and unknown beliefs. B2 stayed silent at the fixed six-metre observation distance; this is not evidence that proximity warning never works. Original comparison logs are in `phase5-policies`; the oracle's corrected shutdown trial is in `phase5-oracle-retest` and is explicitly selected by the consolidated report.

## Platform

Host: macOS 15.3.2 (24D81), Apple M3. Container: Ubuntu 24.04 ARM64, ROS 2 Jazzy, Gazebo Harmonic 8.15.0, Nav2 1.3.13. Headless and native GUI rendering use Mesa llvmpipe in Docker. The installed apt bridge is 1.0.24; the selected local overlay is upstream Jazzy 1.0.25 plus shutdown fix at commit `95e6fdea72fb57ef55a4136292dfa62370ab3fb9`. `results/phase5-report/runtime_manifest.json` records exact packages and source hashes.

## Commands executed

From the repository root unless otherwise stated:

```sh
docker compose ps -a
python3 -m compileall -q ros2_ws/src/hri_industrial
docker compose run --rm --no-deps sim bash scripts/build.sh
docker compose run --rm --no-deps sim python3 -m pytest -q ros2_ws/src/hri_industrial/test
docker stop project-gui-1
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-initial --cases normal
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-routing-fix --cases normal,blockage
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-hazards --cases fire,gas,no_safe,known,unaware
docker compose run --rm --no-deps sim bash scripts/build_bridge.sh
docker compose run --rm --no-deps sim bash scripts/build.sh
docker compose run --rm --no-deps sim python3 -m pytest -q ros2_ws/src/hri_reasoning/test ros2_ws/src/hri_bringup/test ros2_ws/src/hri_dashboard/test ros2_ws/src/hri_industrial/test
docker compose run --rm --no-deps sim python3 scripts/record_environment.py --output results/phase5-report/runtime_manifest.json
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-acceptance --cases normal,blockage,fire,gas,no_safe,known,unaware
```

`npm run build` was run in `dashboard` and passed TypeScript plus the production Vite build. The explicit bridge download was:

```sh
curl -fL --retry 3 https://codeload.github.com/gazebosim/ros_gz/tar.gz/95e6fdea72fb57ef55a4136292dfa62370ab3fb9 -o .runtime/downloads/ros_gz-95e6fde.tar.gz
```

SHA-256, checked with Python: `a8dedb29ee8abf147e8067f9ba4c7817139a871f00de16246dda44adeb2b14c2`.

Additional verification commands executed (individual commands shown where batched in one container):

```sh
docker compose run --rm --no-deps sim python3 scripts/verify_industrial_collision.py --output results/phase5-collision
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-final --cases normal,known
docker compose run --rm --no-deps sim python3 evaluation/run_industrial_experiments.py --output results/phase5-policies
docker compose run --rm --no-deps sim python3 evaluation/run_industrial_experiments.py --output results/phase5-oracle-retest --policies oracle
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-value-iteration --cases unaware --engine value_iteration
docker compose run --rm --no-deps sim python3 scripts/verify_industrial.py --output results/phase5-q-learning --cases unaware --engine q_learning --q-policy /work/results/phase4-mdp/q_policy.json
docker compose run --rm --no-deps sim python3 evaluation/run_warning_experiments.py --output results/phase5-forest-regression --policies B4 --knowledge unaware --duration 25
docker compose run --rm --no-deps sim bash -c 'cd ros2_ws && colcon test --executor sequential --event-handlers console_direct+ && colcon test-result --verbose'
NODE_PATH=/tmp/hri-browser-check/node_modules PLAYWRIGHT_BROWSERS_PATH=/tmp/hri-browser-check/browsers node scripts/record_industrial_demo.cjs results/phase5-visible
docker exec project-gui-1 /hri-entrypoint.sh python3 scripts/record_environment.py --output results/phase5-report/runtime_manifest.json
python3 evaluation/summarize_industrial.py
```

The recording helper uses Playwright; these paths identify the temporary browser installation used here. The project itself runs without Playwright.

## Corrected failures

- Initial Python compile found an unmatched brace in the Gazebo entity-request formatter; corrected before simulator launch.
- Initial unit run: 5 passed, 3 failed. The no-route adapter expected a connector field absent from the common planner's unavailable response; optional connector fields are now handled. The hash test used a duplicated path prefix; corrected without changing the saved reference hashes. Retest: 8 passed.
- `phase5-initial/normal`: Nav2 reached the first route node but refused the second because a 2 Hz route update exceeded the existing 0.5-second freshness bound. Industrial routing now publishes at 5 Hz; subsequent normal and blockage runs passed. A watchdog also cancels motion if route telemetry becomes stale.
- The initial normal run reproduced the pre-existing apt bridge's segmentation fault during shutdown. Rather than discounting that error, the project now builds the pinned upstream Jazzy lifetime/shutdown fix. Original failed logs are retained. Clean final runs must include clean bridge shutdown; these tests do not establish absence of every possible middleware race.
- The bridge build first used an override option absent from this installed colcon; removed. Its next attempt ran eight compiler jobs despite `CMAKE_BUILD_PARALLEL_LEVEL` and exhausted Docker memory. The stopped attempt is recorded in `.runtime/bridge_ws/log`. `MAKEFLAGS=-j1` limits the successful build to one job; it completed in 4 min 19 s.
- Sandboxed DNS prevented the initial archive download; the approved retry succeeded. macOS `shasum` encountered a locale error; Python `hashlib.sha256` verified the archive instead.
- `phase5-acceptance/known` passed all scenario assertions but Gazebo exceeded the five-second SIGINT shutdown deadline. The new process wrapper requests `/server_control` with `stop:true` before using signals as a bounded fallback. `phase5-final/known` subsequently shut down cleanly. This uses Gazebo's [supported server-control service](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1Server.html).
- The normal-navigation probe originally accepted an earlier waypoint success plus proximity to the final destination. It now requires both waypoint success results and proximity within 0.4 m. `phase5-final/normal` passed the stronger check.

- `phase5-policies/oracle/known` completed all scenario assertions but exposed an interrupted Python ROS task constructor during shutdown in the safety node (`Task` missing `_exception`). Safety and dashboard nodes now set a stop flag and finish the current callback before releasing the ROS context, matching the reasoning-node lifecycle. The emergency-stop gate and warning algorithms are unchanged. The original comparison remains failed; a separate oracle retest supplies replacement evidence.

- `phase5-visible-initial` reached the assembly point, but its first approval clicked a stale displayed revision and early screenshots preceded the next SSE/native-render frame. The final recorder waits for the displayed revision, confirms initial physical movement, and waits for the visible warning before capturing. Stale approval rejection remains enforced; no old route is silently approved.

## Research limits

Fire/gas envelopes and semantic recognition are research approximations. No explosion physics, validated gas dispersion, AMCL/SLAM localization, or operational safety certification is claimed. The public initial road survey is an explicit prior. Wheel odometry can drift. The paired examples are one-seed demonstrations, not statistical efficacy claims. Existing Q-learning convergence limits remain unchanged. Hidden operator knowledge is evaluation-only; the normal inference dashboard reports posterior estimates and uncertainty.
