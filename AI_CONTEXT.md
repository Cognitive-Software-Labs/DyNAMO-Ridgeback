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
- the public launch/workflow entrypoints
- `start_exploration.sh` and `build_and_start_expl.sh` quick-start scripts
- G1 perception setup in `perception_venv`
- public parameters and benchmark usage

Use `ISSUES.md` when the task touches:
- `slam_toolbox` TF namespace behavior
- the FastDDS shared-memory workaround (`FASTRTPS_NO_SHM` toggle in `start_exploration.sh`)
- stale-process cleanup, diagnostics, or recurring environment failures

## Repo Mental Model

- This repo centers on 2 normal human launch workflows:
  - `ridgeback_exploration.launch.py`
  - `g1_distance_benchmark.launch.py`
- Benchmarking also exposes `g1_benchmark_env.launch.py` and
  `g1_benchmark_config.launch.py` as public process-level layers because
  `g1_benchmark_sweep` invokes them separately. The compatibility
  `g1_distance_benchmark.launch.py` includes both and inherits their arguments.
- `manual_mapping.launch.py` is the fifth top-level public launch file.
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
- `g1_pointcloud_measurement_node` publishes the `pointcloud` row on `measurements/g1/pointcloud` — the one path that reads the organized `PointCloud2` directly, which is what the mask stack's deprojection was validated against
- **Exploration runs the pointcloud row only; the mask node is benchmark-only for now.** `ridgeback_exploration.launch.py` starts no `g1_mask_measurement_node`, so it draws exactly one ring, and both the viz node and the overlay there are pinned to `estimators: 'pointcloud'` rather than the `all` default — see the `g1_estimate_viz_node` paragraph below for why an unproduced row is worse than no row. Wiring the mask rows into exploration is a later change; when it happens it inherits two pre-existing gaps: `hud_node`'s `panels` in exploration is `['hud/velocity', 'hud/coverage']`, so `hud/g1_distances` has always been published into the void there (rings render, the panel naming them does not), and `exploration.rviz`'s `G1 Polar Rays` display has never had a publisher in exploration. On the **real** robot the D435 publishes no cloud by default, so exploration draws no rings at all until the mask rows arrive (`object_localization_documentation/object_localization_pipeline.md`)
- `g1_mask_measurement_node` publishes the three mask-based rows on `measurements/g1/mask`, and the beams polar profiling reduced on `visualization/g1/polar_rays` (`MarkerArray`)
- **Depth is obtained inside the mask node, at the detection stamp — there is no depth producer process.** `depth_source` picks a strategy from `perception/core/depth_sources.py`; the node subscribes to that strategy's *input* stream (raw camera depth, or the color frame for monocular), buffers it undecoded, and converts only the frame the detections were made on. A separate producer had to guess which stamp the detector would pick, and its independent thinning cost the depth rows ~90% of their coverage under sim load (`object_localization_documentation/aligned_depth_coverage.md`). Consequence for new code: receiving is cheap and unconditional, converting is deferred and demand-driven — do not reintroduce a cadence cap or a latest-wins slot on a stream something downstream matches by stamp. The frame is still published on `debug/g1/mask/aligned_depth` for the overlay panel, but that is debug-only and encoded only when subscribed
- `g1_overlay_node` renders the perception panels. It **publishes the composite on `debug/g1/overlay`** (`bgr8`) so RViz can hold it as an Image display, and separately opens the OpenCV window when `show_window` is true (default). `max_cols` sets panels per row; `rgb_panel_labels` toggles the per-detection label block

Visualization conventions:
- Enable/disable of the G1 visualizations is done with **RViz Displays checkboxes only** — no `add_on_set_parameters_callback` anywhere in this repo, and none was added. A `MarkerArray` display renders one checkbox per marker namespace, which is why `polar_rays` splits into `polar/used`, `polar/dropped` and `polar/wedge`
- Marker builders live in `common/markers.py` and emit markers in the **scan's own frame**, so beam geometry is `angle_min + i*angle_increment` with no extrinsics; TF places them
- `g1_estimate_viz_node` drives its rings and its HUD panel from `benchmarking/estimators.py`, so registering an estimator there is enough to make it appear in both. **Which of them a run shows is the `estimators` parameter**, resolved by the shared `parse_estimators` (so canonical order however the launch argument named them, and `all` — the declared default, which is what `ridgeback_exploration.launch.py` gets since it has no such argument — means the full registry). `g1_distance_benchmark.launch.py` hands over the set it already resolved for the measurement nodes, so both surfaces are filtered from one set: an unselected row draws no ring, and a ring can never appear without a row beside it to name it. A row for a path the run never launched can only ever print `-- miss` — the same word an estimator that ran and found nothing prints, so the two are indistinguishable — which is why `ridgeback_exploration.launch.py` pins its own set rather than taking the `all` default. `_ESTIMATOR_ID_BASE` still enumerates the whole registry: marker ids that shifted with the selection would let a leftover marker collide with another estimator's ring. HUD rows are coloured from the same `ESTIMATOR_COLOURS` table as the rings, at the luminance the row's state calls for: a fresh row is lifted only as far as `HUD_MIN_LUMINANCE` needs and otherwise untouched, an aged row is driven to `HUD_AGED_LUMINANCE` from whichever side it started on. `colour_at_luminance` is the one helper — blend toward white going up, scale all three channels going down (which holds hue *and* saturation). A dim multiplier would not work: a colour sitting exactly at the floor after its lift would clamp straight back to its fresh appearance
- **The rings, the HUD and the `polar_rays` layers all show one instance: the nearest.** Ranked by `nearest_instance_index` in `benchmarking/estimators.py`, where a detection's distance is the first `PUBLIC_ESTIMATOR_ORDER` entry that produced one and ties break on the lower index. `pointcloud` heads that order, so it is the row that normally speaks for a detection; on the **real** robot the D435 publishes no cloud by default, so the fallthrough there is `projective_ranging`. Marker ids are therefore fixed per estimator (rings) and per namespace (rays), not per detection — an id that moved with the detection index would strand the previous instance's markers on screen for a full lifetime. The OpenCV overlay is the exception and still labels every detection
- **Rings are placed off the robot FRONT, not the base origin.** Every estimator subtracts `ROBOT_FRONT_OFFSET_M` before publishing (`geometry.apply_vehicle_front_offset`, and `optical_to_base_planar`'s `front_offset_m` for the mask paths), but TF returns the base pose — so `_add_estimator_markers` calls `geometry.remove_vehicle_front_offset` before rotating a measurement out through `world_marker_point`. Skipping it puts every ring a constant 0.25 m nearer the robot than the distance printed beside it, which reads as sensor error rather than as a plotting fault. `world_marker_point` is module-level and pure so a ring's world position can be asserted without a node — see `test_estimate_viz.py`, and add a placement assertion to any new marker path
- **Every registered estimator places its own detection**, so every ring has a bearing of its own and there is one ring line width. `ESTIMATOR_POSITION_ATTRS` covering `PUBLIC_ESTIMATOR_ORDER` is what licenses that, and `test_estimate_viz` asserts it: a distance-only row added back would silently draw its ring down the boresight, wrong by the whole lateral component, with nothing marking it as a guess
- **The truth line expires on the message stamp, never on when it arrived, and names its own trial.** `estimators.truth_reading` is the single gate (`TRUTH_MAX_AGE_S`, one definition, both the viz node and the OpenCV overlay), and the runner puts the trial id in `header.frame_id` — not a TF frame, no more than the point is a point in one — so the HUD renders `truth 3.250 m  [bed_occluder_single]`. A receipt-time gate cannot tell a message delayed behind a full subscription queue from a fresh one: it reported the delay as freshness and pinned the *previous* trial's truth under the current trial's scene for a whole capture window, while the estimator rows moved on. Both consumers subscribe at depth 1 for the same reason, and the runner republishes every `TRUTH_PUBLISH_PERIOD_SEC` (well inside the gate) so one dropped message cannot blank the line
- **Estimator rows expire on two gates, and neither is the truth line's gate.** `partition_measurements` in `g1_estimate_viz_node` caches `(message, receipt_nanoseconds)` and asks two separate questions: *liveness*, on the RECEIPT time against `MARKER_LIFETIME_SEC` — has this producer gone quiet; and *validity*, on the STAMP against `MAX_OBSERVATION_AGE_S` (3 s, same order as `TRUTH_MAX_AGE_S`) — can this observation still describe the scene. This deliberately differs from `truth_reading`'s stamp-only rule, and the difference is load-bearing both ways: truth is trial-keyed, so no receipt gate can tell trial N-1's truth from trial N's, whereas estimator rows only ever ask "is this producer still running", which receipt time answers correctly. A mask measurement is stamped with the *detection* instant and only arrives after inference, segmentation and the depth lookup, so it is genuinely ~1.2 s old on arrival; charging that pipeline latency against a single 1.5 s stamp gate is what made the three mask rows blink — the pointcloud row shares the stamp with no pipeline behind it and repainted the row as `-- miss` at detector rate
- **Survivors split into the current batch and an aged remainder; only the batch places rings.** The same-batch set is the messages carrying the newest stamp present, and it alone drives `nearest_detection_index` and every marker — `merged_distance_reader`'s premise that indices name the same detections holds only *within* a detector batch, so an older message's index 0 may be a different robot. Aged messages fill HUD rows only, ranked against themselves, printed with an age column and dimmed. A stale number the panel labels as stale is honest; a stale ring is a false claim about where a robot is now, with nothing on the floor plan able to contradict it. A lone lagging producer (mask-only run) is its own current batch and still draws rings — there is nothing newer to contradict it
- **The HUD renders on a timer, the markers stay event-driven, and the HUD's tick is unconditional.** `hud_publish_rate_hz` (5.0, matching `hud_node`'s own sampler) drives `_render_hud`; `_measurement_cb` still calls `_publish_markers` directly. Markers must not move to the timer — they carry `lifetime`, so republishing unchanged data against a fresh stamp would renew it forever and no ring could expire. The HUD has the opposite problem: `hud_node._on_panel` only overwrites its cached text and never expires, so a section that stops publishing latches its last numbers on screen while the rings correctly vanish — which reads as a broken ring layer rather than a dead detector. An empty cache therefore publishes `''`, which `hud_node._tick`'s falsy-panel filter collapses. The timer is also the only thing that re-evaluates `truth_reading`: reachable only from a measurement callback it was a gate with no puller, and when detections stopped the previous trial's truth sat under an already-teleported target. `create_timer` uses the node clock, so a paused sim freezes the panel and the ages together
- The mask node ranks from its own three estimators only (nothing else has filled the batch yet) while the viz node ranks from `pointcloud` onward, so **two near-equidistant robots can split the ray layer from the ring layer** for a frame. Pinning both to the canonical order bounds this to near-ties; closing it entirely would need a published nearest-index topic
- `hud_node` has `horizontal_alignment` / `vertical_alignment` (default `left`/`top`, benchmark uses `right`/`top`) and `rich_text`. **`rich_text` changes the text contract**: the overlay renders via `QStaticText`, which switches to rich text as soon as any HTML tag appears — there `\n` stops breaking lines and runs of spaces collapse, so panels must use `<br/>` and `&nbsp;`. Exploration's panels are plain and keep the default
- **Dock placement is only expressible as a `QMainWindow State` hex blob** in the `.rviz` file. `rviz_common`'s `addPane()` hardcodes `Qt::LeftDockWidgetArea` and the config format has no per-display area key, so a wide panel with no blob lands in the narrow left dock. `benchmark.rviz` carries one (generated via `QMainWindow::saveState()`, dock objectNames = the pane names = the keys in that same block); `restoreState` fails silently on a name mismatch, so any change to it needs a screenshot, not a build

Benchmark stack:
- Persistent environment layer (`g1_benchmark_env.launch.py`): simulator,
  Ridgeback spawn, camera optical TF, benchmark RViz, and `g1_detector_node`.
  The detector publishes only after OWLv2 has loaded, so its publisher is the
  warm-model readiness signal.
- Per-config layer (`g1_benchmark_config.launch.py`): the selected pointcloud
  and/or mask measurement nodes; `g1_estimate_viz_node`; `hud_node`;
  overlay; and `g1_distance_benchmark_runner`. It gates startup on both the
  color and raw-detections publishers through `launch_wait`.
- The runner screen-records persistent RViz to `video/run.mp4` through
  `benchmarking/recording.py` (best-effort; never fails a run).
- `g1_distance_benchmark.launch.py` is the backwards-compatible single-run
  wrapper. `shutdown_on_complete` defaults false, so the completed single-run
  stack stays open for inspection.
- `g1_benchmark_sweep` validates a sweep YAML, starts one environment launch,
  runs one config launch process at a time with `shutdown_on_complete:=true`,
  resumes valid config outputs by `run.json`, records RTF, and writes
  sweep-level `sweep.json` + `summary.md`. Never add runtime parameter mutation
  to avoid the per-config process restart; node existence is estimator-driven.

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

- Always run `bash cleanup.sh` before launching from the repo runbooks or helper scripts. For a benchmark sweep, run it once before the supervisor; never run it between configs because the simulator and RViz are intentionally persistent
- `start_exploration.sh` is the canonical quick-start: sources the workspace, runs cleanup, and forwards `world` (positional 1), `EXPLORER` / explorer (positional 2 or env), and `FASTRTPS_NO_SHM` (env) to the public launch
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
- `src/ridgeback_autonomy/launch/g1_benchmark_env.launch.py`: persistent benchmark simulator/RViz/detector layer
- `src/ridgeback_autonomy/launch/g1_benchmark_config.launch.py`: restartable per-config benchmark layer
- `src/ridgeback_autonomy/config/benchmark_sweep_baseline.yaml`: shipped 4-config baseline sweep
- `src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/g1_benchmark_sweep.py`: sweep supervisor executable
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
