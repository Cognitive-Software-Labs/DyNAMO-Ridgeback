# AI Context

Shared repository guidance for AI coding agents working in this repo.

## Documentation Map

- `README.md`: canonical human-facing setup, launch usage, and operator workflows
- `AI_CONTEXT.md`: canonical agent-facing repo conventions, mental model, and documentation rules
- `ISSUES.md`: troubleshooting, historical root causes, and operational gotchas
- `AGENTS.md` / `CLAUDE.md`: thin entrypoints that point agents here

## Documentation Rules

Canonical ownership:
- `README.md` owns human setup and usage instructions
- `AI_CONTEXT.md` owns agent workflow, repo conventions, and implementation-oriented guidance
- `ISSUES.md` owns troubleshooting, debugging recipes, workarounds, and historical root causes
- `AGENTS.md` and `CLAUDE.md` should stay thin and should not grow into parallel full guides

Update rules:
- If a change affects user setup, launch commands, public parameters, or operator workflow, update `README.md`
- If a change affects agent behavior, repo conventions, codebase navigation, graphify workflow, or non-obvious implementation context, update `AI_CONTEXT.md`
- If a change explains a failure mode, workaround, debugging recipe, or historical root cause, update `ISSUES.md`
- If a fact belongs in multiple places, prefer one canonical source and cross-reference it
- Small duplication is acceptable only when the second doc serves a different audience and the fact is stable and high-value
- `AI_CONTEXT.md` may reference a `README.md` section and then expand it when agents need more precision or extra context

Precedence:
- For human workflow truth, `README.md` wins
- For agent workflow truth, `AI_CONTEXT.md` wins
- For troubleshooting and history truth, `ISSUES.md` wins

## Graphify

This project has a graphify knowledge graph at `graphify-out/`.

Rules:
- Before answering architecture or codebase questions, read `graphify-out/GRAPH_REPORT.md` for god nodes and community structure
- After modifying code files, run `bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"` to keep the repo-root graph current
- Do not use the old `python3 -c "from graphify.watch import _rebuild_code ..."` one-liner in this repo; the helper script is the canonical rebuild path

## Human Docs To Reference

Use `README.md` for the full human runbook. In particular, point humans there for:
- installation and workspace build steps
- the 2 public launch entrypoints
- `start_exploration.sh` and `build_and_start_expl.sh` quick-start scripts
- G1 perception setup in `perception_venv`
- public parameters and benchmark usage

Use `ISSUES.md` when the task touches:
- `slam_toolbox` TF namespace behavior
- DDS middleware selection (CycloneDDS default via `cyclonedds.xml`; FastDDS + UDP-only fallback) and the `tools/setup_dds.sh` kernel tuning in `start_exploration.sh`
- stale-process cleanup, diagnostics, or recurring environment failures

## Repo Mental Model

- This repo centers on 2 public launch entrypoints:
  - `ridgeback_exploration.launch.py`
  - `g1_distance_benchmark.launch.py`
- Lower-level launches in `src/ridgeback_autonomy/launch/includes/` are internal building blocks
- Bringup is **event-driven**: stages are sequenced by readiness gates (`ros2 run ridgeback_autonomy launch_wait`, in `common/launch_wait.py`) chained via `OnProcessExit`, not fixed `TimerAction` delays — each stage starts when its prerequisite topic/service exists, with a `--timeout` fallback. See ISSUES.md "Event-Driven Startup". Don't reintroduce timer delays
- The ROS package is `ridgeback_autonomy`
- The main Python package is also `ridgeback_autonomy`
- Simulation assets now live under `src/ridgeback_autonomy/sim/`
- Human-facing docs should talk about public workflows, not internal include files, unless that detail matters

## High-Signal Runtime Architecture

Exploration stack:
- Gazebo simulation
- `slam_toolbox`
- Nav2
- frontier explorer — the in-repo `frontier_explorer_node`, launched by `launch/includes/explore.launch.py` (explore_lite was removed 2026-07-10 after a head-to-head benchmark; findings in ISSUES.md "Exploration Quits Early")
- HUD is a general aggregator: producers publish `rviz_2d_overlay_msgs/OverlayText` "panels" on their own topics (`hud/velocity`, `hud/coverage`, …); `hud_node` merges them in the configured `panels` order into one `hud_overlay` (the single RViz `TextOverlay` display). Add a metric = new publisher + its topic in `panels`; no RViz change.
- `velocity_overlay_node` publishes the velocity panel on `hud/velocity` (4-stage cmd_vel chain: `planned` from MPPI, `capped` from velocity_smoother, `controller` from collision_monitor, `actual` from `platform/odom/filtered`)
- `coverage_overlay_node` publishes the live exploration-coverage panel on `hud/coverage`: compares the SLAM map to the ground-truth map for the current `world` (reusing `common/coverage_utils.py`), reporting `complete` (discovered fraction of gt-free) and `accuracy` (coverage over explored gt-free); worlds without a ground-truth map show `n/a`. Gated by `coverage_overlay_enabled` (default true). Ground-truth maps live in the package at `sim/ground_truth_maps/` (installed to `share/`, alongside `sim/worlds/`); the node resolves them via `get_package_share_directory`. Capture/preview tooling (`capture_ground_truth.sh`, `render_previews.py`) and the README sit alongside the maps in `sim/ground_truth_maps/`
- custom RViz config also enables MPPI trajectory visualization (`/optimal_trajectory` + `/trajectories`) when `mppi_visualize:=true` is forwarded into nav2 via `FollowPath.visualize`

G1 perception stack (on by default — `g1_perception_enabled` defaults to `true`; it gates the entire detection+measurement+overlay positioning stack as one unit and requires `perception_venv`. Running it without the venv makes `g1_detector_node` log one clear error and exit cleanly instead of crashing; disable the stack with `g1_perception_enabled:=false`):
- `g1_detector_node` publishes raw detections on `detections/g1/raw`
- `g1_camera_measurement_node` publishes camera-based measurements on `measurements/g1/camera`
- `g1_lidar_measurement_node` publishes LiDAR-based measurements on `measurements/g1/lidar`
- `g1_overlay_node` renders the separate OpenCV perception window

Benchmark stack:
- simulator
- `g1_detector_node`
- one measurement node chosen by `measurement_backend`
- `g1_distance_benchmark_runner`

## Namespace And Topic Conventions

- Clearpath requires a non-empty namespace, currently `r100_0001`
- Topics are therefore namespaced, for example:
  - `/r100_0001/sensors/lidar2d_0/scan`
  - `/r100_0001/sensors/camera_0/color/image`
  - `/r100_0001/map`
- New nodes in this stack should set the namespace and remap:
  - `('/tf', 'tf')`
  - `('/tf_static', 'tf_static')`
- In simulation, all nodes must use `use_sim_time: true`

## Non-Obvious Conventions

- Always run `bash cleanup.sh` before launching from the repo runbooks or helper scripts
- `start_exploration.sh` is the canonical quick-start: sources the workspace, runs cleanup, and forwards `world` (positional 1), `DEPTH_ANYTHING_ENABLED` (env), and the DDS selection (`RMW_IMPLEMENTATION`, default CycloneDDS; `FASTRTPS_NO_SHM` applies only on the FastDDS fallback) to the public launch
- `build_and_start_expl.sh` rebuilds the workspace before forwarding to `start_exploration.sh`; pass-through args are positional in the same order
- The G1 overlay is a separate OpenCV window, not an RViz panel
- The `mock_hospital` world is the main exploration scenario; `warehouse` is the larger exploration test; `office` is the common perception-debug world
- `perception_venv/` is expected for OWLv2 and Depth-Anything dependencies; the public launches prepend its `bin/` directory to `PATH`
- If you move or rename sim assets that are referenced by patched Clearpath files, update `patches/clearpath_gz_customizations.patch` in the same change

## Project Subagents

Committed under `.claude/agents/` (Claude Code picks them up automatically):

- `sim-runner` — launches/monitors/stops instrumented benchmark runs (full recipe incl. probe attach, `tools/benchmark/`)
- `log-triage` — read-only launch-log diagnosis against the ISSUES.md failure signatures
- `box-health` — read-only shared-box triage (GPU seat ACL, co-tenant load, stale processes)

Keep their recipes in sync when the underlying scripts (`cleanup.sh`, `start_exploration.sh`, `tools/benchmark/`) or ISSUES.md sections change.

## Key Repo Facts Agents Should Remember

- `clearpath/robot.yaml` must also exist at `~/clearpath/robot.yaml` for the simulator default path
- Sensor names are auto-indexed by Clearpath, so the first camera becomes `camera_0` and the first 2D lidar becomes `lidar2d_0`
- `slam_toolbox` is sourced from `.repos` and still needs the local TF namespace patch described in `ISSUES.md`
- DDS defaults to **CycloneDDS** in `start_exploration.sh` (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `CYCLONEDDS_URI=cyclonedds.xml`): loopback-only, `MaxAutoParticipantIndex` raised (40+ node stack), large socket buffers. Needs the host kernel tuning from `tools/setup_dds.sh` (one-time sudo; persisted in `/etc/sysctl.d/60-ros-dds.conf`). Set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` to fall back to FastDDS (then `FASTRTPS_NO_SHM` defaults to `true` = UDP-only profile). The public launch files (`ridgeback_exploration`, `g1_distance_benchmark`) also `SetEnvironmentVariable` the same RMW/`CYCLONEDDS_URI` defaults (respecting an explicit override), so `ros2 launch` directly is consistent with the script — no RMW mismatch. Keep the historical SHM rationale in `ISSUES.md`, not `README.md`

## Key Config And Entry Files

- `clearpath/robot.yaml`: robot platform, namespace, and sensor declarations
- `src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py`: main public exploration launch
- `src/ridgeback_autonomy/launch/g1_distance_benchmark.launch.py`: public benchmark launch
- `start_exploration.sh`: canonical exploration quick-start (cleanup + launch + arg/env forwarding)
- `build_and_start_expl.sh`: rebuild then forward to `start_exploration.sh`
- `src/ridgeback_autonomy/config/nav2_params.yaml`: Nav2 config
- `src/ridgeback_autonomy/config/slam_toolbox_params.yaml`: SLAM config
- `src/ridgeback_autonomy/config/frontier_explorer_params.yaml`: `frontier_explorer_node` config
- `src/ridgeback_autonomy/config/camera_config.json`: shared camera geometry
- `cyclonedds.xml`: default DDS config (CycloneDDS) exported by `start_exploration.sh` — loopback, raised participant limit, large socket buffers
- `fastrtps_no_shm.xml`: UDP-only FastDDS profile, used only when falling back with `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `tools/setup_dds.sh`: one-time sudo host tuning (`/etc/sysctl.d/60-ros-dds.conf`) for large-message DDS buffers

## External Dependencies

Managed through `.repos`:
- `clearpath_simulator`
- `clearpath_common`
- `clearpath_config`
- `clearpath_msgs`
- `slam_toolbox`

If you remove, rename, or add repo dependencies, check:
- `.repos`
- `README.md` install instructions
- any local patches under `patches/`
- `ISSUES.md` if the dependency change resolves or creates a notable operational issue
