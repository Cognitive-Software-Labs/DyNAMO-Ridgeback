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
  logs, `exploration/` for exploration runs, `benchmarks/` for benchmark
  runs and sweeps, `benchmark-jobs/` for jobs the configurator writes, and
  `configurator/runs/` for its run records.
- Preserve explicit `output_dir`, `LOG_DIR`, and `ROS_LOG_DIR` overrides.
- `benchmarking/paths.py` owns those roots and `anchored_path`, which anchors
  every operator-supplied benchmark path on the workspace root once, at job
  canonicalisation. Do not add a second anchor: the runner resolves a job's
  relative paths against **the job file's own parent**, so anything resolving
  against its own working directory instead will disagree with execution.
- `--symlink-install` behaves three ways inside `ridgeback_autonomy`, which
  decides what an edit requires afterwards: `configurator_assets/*` are
  symlinked, so reload the browser; modules under `ridgeback_autonomy/` are
  symlinked into `site-packages`, so restart the process that imported them;
  anything listed in an `install(PROGRAMS ... RENAME)` rule — `configurator.py`,
  `target_replay_benchmark.py`, the node scripts — is a real **copy**, so run
  `colcon build`. Verify such a change landed by grepping the executable under
  `install/ridgeback_autonomy/lib/ridgeback_autonomy/`, never `site-packages`,
  which is a symlink and shows the edit whether or not you rebuilt.
- `clearpath/robot.yaml` is canonical. Generated descriptions and installed
  copies are not editing targets.
- `perception_venv/` supplies OWLv2, segmentation, and monocular-depth
  dependencies; public localization launches prepend its `bin/` directory.

## Dependencies and patches

External ROS repositories are declared in `.repos`. When a dependency changes,
check the root installation instructions, local patches, and any operational
notes affected by that change. If a simulation asset referenced by a Clearpath
patch moves, update the patch in the same change.

The [dependency runbook](dependencies.md) owns the import, verification, refresh,
live-checkout migration, and rollback procedure. Run `tools/check_dependencies`
before builds and do not discard a nested-repository change to satisfy it.

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
