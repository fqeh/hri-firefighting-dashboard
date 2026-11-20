# Dependency provenance and compatibility

Checked against upstream sources on 2026-11-13. Runtime-selected versions are recorded separately after installation; no source repository is silently replaced with a local simulator.

| Dependency | Chosen series | Upstream / license |
| --- | --- | --- |
| ROS 2 | Jazzy, Ubuntu Noble ARM64/AMD64 | [ros2](https://github.com/ros2), per-package Apache-2.0/BSD licenses |
| Gazebo Sim | Harmonic / gz-sim8, ROS vendor packages | [gz-sim](https://github.com/gazebosim/gz-sim/tree/gz-sim8), Apache-2.0 |
| ros_gz | Jazzy branch/package | [ros_gz](https://github.com/gazebosim/ros_gz/tree/jazzy), Apache-2.0 |
| Navigation2 | Jazzy packages | [Nav2](https://github.com/ros-navigation/navigation2/tree/jazzy), per-package Apache-2.0/BSD/LGPL, [license inventory](https://github.com/ros-navigation/navigation2/blob/jazzy/LICENSE) |
| RViz2 | Jazzy packages | [rviz](https://github.com/ros2/rviz), BSD-3-Clause |
| React / TypeScript / Vite | Exact frontend versions in package-lock.json | [React](https://github.com/facebook/react) MIT; [TypeScript](https://github.com/microsoft/TypeScript) Apache-2.0; [Vite](https://github.com/vitejs/vite) MIT |

The [official Gazebo compatibility table](https://gazebosim.org/docs/latest/ros_installation/) pairs Jazzy with Harmonic. Installing `ros-jazzy-ros-gz` selects the matching Gazebo vendor dependencies. Do not mix Gazebo Classic, Humble binaries, or newer Gazebo major versions into this workspace. The [Jazzy bridge type matrix](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md) includes all standard messages and LogicalCameraImage used here.

The project reuses maintained Gazebo physics, differential-drive, sensor and pose systems, ROS transport, and Nav2 planning/control. The small robot chassis and incident assets are original SDF primitives. This keeps the first build self-contained and avoids unresolved Fuel URLs; it is not a photorealistic or validated commercial robot model. Upstream example API behavior informed configuration; no external mesh licensing is required for the generated terrain.

The Docker base image is pinned by digest. Apt package resolution remains dependent on the selected repository snapshot, so the installed package manifest and image ID must accompany published experiments. The supplied Dockerfile is reproducible as a build recipe, not a guarantee that future apt repositories serve byte-identical packages. Archive the verified image with `docker save` for strict replication.

On the inspected Mac, Docker Desktop's credential helper stalled on public image pulls. An isolated empty Docker configuration allows anonymous public-image access without changing the user's account configuration. Initial Ubuntu and ROS HTTP fetches returned apt checksum mismatches. Ubuntu supports HTTPS directly; the main ROS hostname did not present a matching HTTPS certificate. The Dockerfile therefore uses the [Oregon State ROS mirror](https://ftp.osuosl.org/pub/ros/packages.ros.org/ros2/ubuntu/) with the official ROS signing key. ROS infrastructure maintainers discuss the endpoint's HTTPS limitation in the [signing-key migration thread](https://discourse.ros.org/t/ros-signing-key-migration-guide/43937). No signature or checksum verification is disabled. `ROS_APT_MIRROR` is a build argument for selecting a compatible mirror.
