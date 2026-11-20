# Contributing

## Scope and principles

Contributions should preserve the Gazebo-as-physical-world architecture, public/private belief boundary, deterministic scenario configuration, and explicit distinction among observation, inference and simulation truth. Do not add mock values to integrated dashboard mode or treat simulation outcomes as firefighter safety evidence.

## Development

- Python/ROS packages: `ros2_ws/src/`
- Frontend: `dashboard/` (React + TypeScript)
- Evaluation tools: `evaluation/`
- Scenarios and launchers: `ros2_ws/src/hri_sim`, `ros2_ws/src/hri_industrial`, `ros2_ws/src/hri_bringup`

Build and run focused checks:

```sh
bash scripts/test_local.sh
docker compose run --rm --no-deps sim bash -c 'cd ros2_ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose'
cd dashboard && npm test && npm run build
```

For physical behavior changes, add or update an acceptance check that uses a fresh Gazebo/ROS launch and independent sensor/odometry evidence. Preserve failed attempts in their named output directory; do not overwrite historical trial logs or relabel invalid runs as complete. Evaluation scripts must not leak `/evaluation/truth` or `/evaluation/human_belief` into normal policy inference or public dashboard output.

Record scenario, seed, configuration, source/runtime hashes, actual simulation duration and completion/censoring status. Keep analytical calculations clearly separate from executed simulation measurements. Do not make statistical or real-world safety claims from the small engineering matrix.
