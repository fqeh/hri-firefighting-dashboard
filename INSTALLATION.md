# Installation

## Tested runtime

The verified development host is macOS 15.3.2 on Apple Silicon. ROS 2 Jazzy, Gazebo Harmonic, Nav2 and the ROS packages run in the repository's Ubuntu 24.04 Docker image. The recorded runtime is Ubuntu 24.04 ARM64, ROS 2 Jazzy, Gazebo Harmonic 8.15.0 and Nav2 1.3.13. Native Ubuntu 24.04 with the same ROS/Gazebo pairing is supported by the installer, but native RViz interaction has not been validated on this host.

Install Docker Desktop, start it, and from the repository root build the simulator and workspace:

```sh
docker compose build sim
docker compose run --rm --no-deps sim bash scripts/build.sh
```

Start the headless simulator and ROS dashboard gateway:

```sh
docker compose up sim
```

In another terminal, launch the full browser dashboard, including its loopback-only scenario and experiment supervisor:

```sh
bash scripts/run_dashboard.sh
```

Open `http://127.0.0.1:5173`. The GUI compose profile and native Ubuntu instructions are in the [project README](README.md). For native Ubuntu, use `bash scripts/install_ubuntu.sh`, `bash scripts/build.sh`, then `bash scripts/run_demo.sh forest gui:=true`.

## Verification commands

```sh
bash scripts/test_local.sh
docker compose run --rm --no-deps sim bash -c 'cd ros2_ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose'
cd dashboard && npm ci && npm test && npm run build
```

The project is a simulation research prototype, not a certified emergency-response system. See [dependency provenance](docs/DEPENDENCIES.md) and [known limitations](LIMITATIONS.md).
