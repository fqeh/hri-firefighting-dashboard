# Third-party licenses

Original project code and generated SDF/terrain assets are distributed under Apache-2.0; see [LICENSE](LICENSE). Third-party software retains its own license.

| Component | Use | License/source |
|---|---|---|
| ROS 2 Jazzy packages | Middleware, messages, launch | Per-package Apache-2.0/BSD and other upstream terms; [ROS 2](https://github.com/ros2) |
| Gazebo Harmonic / gz-sim8 | Physics, rendering, sensors | Apache-2.0; [Gazebo](https://github.com/gazebosim/gz-sim/tree/gz-sim8) |
| ros_gz | Gazebo/ROS bridge | Apache-2.0; [ros_gz](https://github.com/gazebosim/ros_gz/tree/jazzy) |
| Navigation2 | Planning and navigation | Per-package Apache-2.0/BSD/LGPL; [Nav2 license inventory](https://github.com/ros-navigation/navigation2/blob/jazzy/LICENSE) |
| RViz2 | Optional ROS visualization | BSD-3-Clause; [RViz](https://github.com/ros2/rviz) |
| React / React DOM | Dashboard runtime | MIT; [React](https://github.com/facebook/react) |
| three.js (0.187.0) | Dashboard web 3D view | MIT; [three.js](https://github.com/mrdoob/three.js) |
| cloudflared (optional, not vendored) | `scripts/run_public.sh` remote sharing | Apache-2.0; [cloudflared](https://github.com/cloudflare/cloudflared) |
| TypeScript | Type checking and compilation | Apache-2.0; [TypeScript](https://github.com/microsoft/TypeScript) |
| Vite / Vitest | Frontend build and tests | MIT; pinned versions in `dashboard/package-lock.json` |

The exact ROS apt packages and runtime sources for the verified image are recorded in the result runtime manifests. The bridge overlay is pinned and hash-checked by the build scripts. Generated terrain assets are original; the dependency review found no fetched third-party mesh models. For deployment or redistribution, retain notices from installed ROS and npm packages and review the specific package license files; this table is a project-level summary, not legal advice.

See [DEPENDENCIES.md](docs/DEPENDENCIES.md) for version compatibility, provenance and mirror details.
