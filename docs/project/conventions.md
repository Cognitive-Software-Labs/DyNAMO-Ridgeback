# Project conventions

Cross-cutting rules that do not belong to one subsystem live here. Detailed
behavior stays in the relevant technical reference.

## ROS namespace and time

- Clearpath uses a non-empty namespace, currently `r100_0001`.
- New namespaced nodes remap `('/tf', 'tf')` and
  `('/tf_static', 'tf_static')` when they consume TF.
- A ROS topic namespace does not determine the frame IDs carried inside TF.
  Inspect the actual graph instead of deriving `base_frame` from the namespace.
- Simulation nodes use `use_sim_time: true`.
- Sensor names are auto-indexed: the first camera is `camera_0`; the first 2D
  LiDAR is `lidar2d_0`.

## Entrypoints and launch ownership

- Human-facing docs describe public workflows. Internal include launches are
  documented only when their lifecycle or argument boundary matters.
- Bringup is event-driven through `launch_wait` readiness gates chained by
  process/state events. Do not replace a real readiness condition with a fixed
  `TimerAction` delay.
- `start_exploration.sh` sources the workspace, cleans stale processes, selects
  CycloneDDS only when the caller has not selected an RMW, accepts the world as
  positional argument 1, and forwards later `key:=value` arguments.
- Run `cleanup.sh` once before a benchmark sweep, never between its persistent
  configurations.

## Generated state

- `build/` and `install/` are expected top-level Colcon directories.
- Generated run output belongs under `artifacts/`: `colcon/` for build/test
  logs, `exploration/` for exploration runs, and `benchmarks/` for benchmark
  runs and sweeps.
- Preserve explicit `output_dir`, `LOG_DIR`, and `ROS_LOG_DIR` overrides.
- `clearpath/robot.yaml` is canonical. Generated descriptions and installed
  copies are not editing targets.
- `perception_venv/` supplies OWLv2, segmentation, and monocular-depth
  dependencies; public localization launches prepend its `bin/` directory.

## Dependencies and patches

External ROS repositories are declared in `.repos`. When a dependency changes,
check the root installation instructions, local patches, and any operational
notes affected by that change. If a simulation asset referenced by a Clearpath
patch moves, update the patch in the same change.

The `slam_toolbox` namespace patch is required by this stack; its rationale and
failure signature live in [troubleshooting](../troubleshooting.md).

## Graphify

The repository graph lives in `graphify-out/`.

- Read `graphify-out/GRAPH_REPORT.md` before architecture or broad codebase
  analysis.
- After modifying code, run
  `bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"`.
- Do not use the obsolete direct Python `graphify.watch` one-liner; the helper
  owns the correct environment and output path.

Graphify is required after code changes, not documentation-only edits.
