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
- If `graphify-out/wiki/index.md` exists, navigate it instead of reading raw files
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
- the FastDDS shared-memory workaround (`FASTRTPS_NO_SHM` toggle in `start_exploration.sh`)
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
- frontier explorer — either `explore_lite` (default) or the in-repo `frontier_explorer_node`, dispatched in `launch/includes/explore.launch.py` based on the `explorer` arg
- HUD is a general aggregator: producers publish `rviz_2d_overlay_msgs/OverlayText` "panels" on their own topics (`hud/velocity`, `hud/coverage`, …); `hud_node` merges them in the configured `panels` order into one `hud_overlay` (the single RViz `TextOverlay` display). Add a metric = new publisher + its topic in `panels`; no RViz change.
- `velocity_overlay_node` publishes the velocity panel on `hud/velocity` (4-stage cmd_vel chain: `planned` from MPPI, `capped` from velocity_smoother, `controller` from collision_monitor, `actual` from `platform/odom/filtered`)
- `coverage_overlay_node` publishes the live exploration-coverage panel on `hud/coverage`: compares the SLAM map to the ground-truth map for the current `world` (reusing `common/coverage_utils.py`), reporting `complete` (discovered fraction of gt-free) and `accuracy` (coverage over explored gt-free); worlds without a ground-truth map show `n/a`. Gated by `coverage_overlay_enabled` (default true). Ground-truth maps live in the package at `sim/ground_truth_maps/` (installed to `share/`, alongside `sim/worlds/`); the node resolves them via `get_package_share_directory`. Capture/preview tooling (`capture_ground_truth.sh`, `render_previews.py`) and the README sit alongside the maps in `sim/ground_truth_maps/`
- custom RViz config also enables MPPI trajectory visualization (`/optimal_trajectory` + `/trajectories`) when `mppi_visualize:=true` is forwarded into nav2 via `FollowPath.visualize`

G1 perception stack (on by default — `g1_perception_enabled` defaults to `true`; it gates the entire detection+measurement+overlay positioning stack as one unit and requires `perception_venv`. Running it without the venv makes `g1_detector_node` log one clear error and exit cleanly instead of crashing; disable the stack with `g1_perception_enabled:=false`):
- `g1_detector_node` publishes raw detections on `detections/g1/raw`
- `g1_camera_measurement_node` publishes camera-based measurements on `measurements/g1/camera`
- `g1_lidar_measurement_node` publishes LiDAR-based measurements on `measurements/g1/lidar`
- `g1_mask_measurement_node` publishes the three mask-based rows on `measurements/g1/mask`, and the beams polar profiling reduced on `visualization/g1/polar_rays` (`MarkerArray`)
- `g1_overlay_node` renders the perception panels. It **publishes the composite on `debug/g1/overlay`** (`bgr8`) so RViz can hold it as an Image display, and separately opens the OpenCV window when `show_window` is true (default). `max_cols` sets panels per row; `rgb_panel_labels` toggles the per-detection label block

Visualization conventions:
- Enable/disable of the G1 visualizations is done with **RViz Displays checkboxes only** — no `add_on_set_parameters_callback` anywhere in this repo, and none was added. A `MarkerArray` display renders one checkbox per marker namespace, which is why `polar_rays` splits into `polar/used`, `polar/dropped` and `polar/wedge`
- Marker builders live in `common/markers.py` and emit markers in the **scan's own frame**, so beam geometry is `angle_min + i*angle_increment` with no extrinsics; TF places them
- `g1_estimate_viz_node` drives its rings and its HUD panel from `benchmarking/estimators.py`, so registering an estimator there is enough to make it appear in both. HUD rows are coloured from the same `ESTIMATOR_COLOURS` table as the rings, lightened only as far as `HUD_MIN_LUMINANCE` needs
- **The rings, the HUD and the `polar_rays` layers all show one instance: the nearest.** Ranked by `nearest_instance_index` in `benchmarking/estimators.py`, where a detection's distance is the first `PUBLIC_ESTIMATOR_ORDER` entry that produced one and ties break on the lower index. Marker ids are therefore fixed per estimator (rings) and per namespace (rays), not per detection — an id that moved with the detection index would strand the previous instance's markers on screen for a full lifetime. The OpenCV overlay is the exception and still labels every detection
- **Rings are placed off the robot FRONT, not the base origin.** Every estimator subtracts `ROBOT_FRONT_OFFSET_M` before publishing (`geometry.apply_vehicle_front_offset`, and `optical_to_base_planar`'s `front_offset_m` for the mask paths), but TF returns the base pose — so `_add_estimator_markers` calls `geometry.remove_vehicle_front_offset` before rotating a measurement out through `world_marker_point`. Skipping it puts every ring a constant 0.25 m nearer the robot than the distance printed beside it, which reads as sensor error rather than as a plotting fault. `world_marker_point` is module-level and pure so a ring's world position can be asserted without a node — see `test_estimate_viz.py`, and add a placement assertion to any new marker path
- **`sensor_depth` and `depth_anything` rings borrow their bearing.** Those two publish a planar distance and no position (they are absent from `ESTIMATOR_POSITION_ATTRS` — the *scoring* locator, which must stay that way), so the ring direction comes from `display_bearing`: the first `PUBLIC_ESTIMATOR_ORDER` row that placed the same detection, matching `display_distance`'s rule. The radius is still the row's own number. Those rings are drawn at `BORROWED_BEARING_LINE_WIDTH_M` — a thin ring means the direction is second-hand. With no bearing available they fall back to the boresight
- **The truth line expires on the message stamp, never on when it arrived, and names its own trial.** `estimators.truth_reading` is the single gate (`TRUTH_MAX_AGE_S`, one definition, both the viz node and the OpenCV overlay), and the runner puts the trial id in `header.frame_id` — not a TF frame, no more than the point is a point in one — so the HUD renders `truth 3.250 m  [bed_occluder_single]`. A receipt-time gate cannot tell a message delayed behind a full subscription queue from a fresh one: it reported the delay as freshness and pinned the *previous* trial's truth under the current trial's scene for a whole capture window, while the estimator rows moved on. Both consumers subscribe at depth 1 for the same reason, and the runner republishes every `TRUTH_PUBLISH_PERIOD_SEC` (well inside the gate) so one dropped message cannot blank the line
- The mask node ranks from its own three estimators only (nothing else has filled the batch yet) while the viz node ranks from `rgb` onward, so **two near-equidistant robots can split the ray layer from the ring layer** for a frame. Pinning both to the canonical order bounds this to near-ties; closing it entirely would need a published nearest-index topic
- `hud_node` has `horizontal_alignment` / `vertical_alignment` (default `left`/`top`, benchmark uses `right`/`top`) and `rich_text`. **`rich_text` changes the text contract**: the overlay renders via `QStaticText`, which switches to rich text as soon as any HTML tag appears — there `\n` stops breaking lines and runs of spaces collapse, so panels must use `<br/>` and `&nbsp;`. Exploration's panels are plain and keep the default
- **Dock placement is only expressible as a `QMainWindow State` hex blob** in the `.rviz` file. `rviz_common`'s `addPane()` hardcodes `Qt::LeftDockWidgetArea` and the config format has no per-display area key, so a wide panel with no blob lands in the narrow left dock. `benchmark.rviz` carries one (generated via `QMainWindow::saveState()`, dock objectNames = the pane names = the keys in that same block); `restoreState` fails silently on a name mismatch, so any change to it needs a screenshot, not a build

Benchmark stack:
- simulator
- `g1_detector_node`
- one measurement node chosen by `measurement_backend`
- `g1_estimate_viz_node` + `hud_node` (the rings and the distance readout, `estimate_viz:=true` by default)
- `g1_distance_benchmark_runner` — also screen-records the RViz window to `video/run.mp4` via `benchmarking/recording.py` (best-effort; never fails a run)

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
- `start_exploration.sh` is the canonical quick-start: sources the workspace, runs cleanup, and forwards `world` (positional 1), `EXPLORER` / explorer (positional 2 or env), `DEPTH_ANYTHING_ENABLED` (env), and `FASTRTPS_NO_SHM` (env) to the public launch
- `build_and_start_expl.sh` rebuilds the workspace before forwarding to `start_exploration.sh`; pass-through args are positional in the same order
- The G1 overlay is a separate OpenCV window, not an RViz panel
- The `mock_hospital` world is the main exploration scenario; `warehouse` is the larger exploration test; `office` is the common perception-debug world
- `perception_venv/` is expected for OWLv2 and Depth-Anything dependencies; the public launches prepend its `bin/` directory to `PATH`
- If you move or rename sim assets that are referenced by patched Clearpath files, update `patches/clearpath_gz_customizations.patch` in the same change

## Key Repo Facts Agents Should Remember

- `clearpath/robot.yaml` must also exist at `~/clearpath/robot.yaml` for the simulator default path
- Sensor names are auto-indexed by Clearpath, so the first camera becomes `camera_0` and the first 2D lidar becomes `lidar2d_0`
- `slam_toolbox` is sourced from `.repos` and still needs the local TF namespace patch described in `ISSUES.md`
- The UDP-only FastDDS profile is a `start_exploration.sh` toggle (`FASTRTPS_NO_SHM`, default `false` — shared memory on); the public launches do not set FastDDS env vars themselves, so `ros2 launch` invocations honor whatever is in your shell. Keep the historical rationale and toggle docs in `ISSUES.md`, not in `README.md`

## Key Config And Entry Files

- `clearpath/robot.yaml`: robot platform, namespace, and sensor declarations
- `src/ridgeback_autonomy/launch/ridgeback_exploration.launch.py`: main public exploration launch
- `src/ridgeback_autonomy/launch/g1_distance_benchmark.launch.py`: public benchmark launch
- `start_exploration.sh`: canonical exploration quick-start (cleanup + launch + arg/env forwarding)
- `build_and_start_expl.sh`: rebuild then forward to `start_exploration.sh`
- `src/ridgeback_autonomy/config/nav2_params.yaml`: Nav2 config
- `src/ridgeback_autonomy/config/slam_toolbox_params.yaml`: SLAM config
- `src/ridgeback_autonomy/config/explore_lite_params.yaml`: `explore_lite` frontier exploration config
- `src/ridgeback_autonomy/config/frontier_explorer_params.yaml`: in-repo `frontier_explorer_node` config (used when `explorer:=custom`)
- `src/ridgeback_autonomy/config/camera_config.json`: shared camera geometry
- `fastrtps_no_shm.xml`: UDP-only FastDDS profile exported by `start_exploration.sh` when `FASTRTPS_NO_SHM=true`

## External Dependencies

Managed through `.repos`:
- `clearpath_simulator`
- `clearpath_common`
- `clearpath_config`
- `clearpath_msgs`
- `m-explore-ros2`
- `slam_toolbox`

If you remove, rename, or add repo dependencies, check:
- `.repos`
- `README.md` install instructions
- any local patches under `patches/`
- `ISSUES.md` if the dependency change resolves or creates a notable operational issue
