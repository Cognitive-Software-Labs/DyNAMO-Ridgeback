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
- **Exploration and the benchmark run the same four rows, from the same node specs.** `perception/g1_launch.py` owns the whole G1 stack's specs — `pointcloud_measurement_node()`, `mask_measurement_node()`, `estimate_viz_node()`, `overlay_node()`, `distance_hud_node()` — plus the topic constants they wire (`POINTCLOUD_MEASUREMENT_TOPIC` / `MASK_MEASUREMENT_TOPIC` / `MASK_ALIGNED_DEPTH_DEBUG_TOPIC` / `HUD_DISTANCES_PANEL_TOPIC`), `OVERLAY_SINGLE_ROW`, `SIMULATION_CAMERA_INPUTS` and `resolved_camera_inputs()`. **No launch file calls `resolve_camera_inputs` itself and none writes out a G1 node inline** (`test_launch_layout` asserts both), so the two stacks cannot drift into measuring off different topics or rendering to different panels. The only G1 node still written out at a call site is exploration's `g1_detector_node`, because the benchmark's lives in its persistent environment layer and is started once for a whole sweep rather than per config — a genuinely different lifetime, not a duplicated spec. The factories take the varying axes as keyword arguments and **omit any left `None`**, which is the difference between the callers: the benchmark passes every axis it sweeps, exploration only the ones it declares an argument for (`estimators`, `depth_source`, `mask_gate`) and lets the node's own defaults stand for the rest. Setting a parameter to the value the node already defaults to would be a second copy of that default, free to drift
- **`base_frame` is the one mask-node parameter every caller must pass.** Its default is the bare string `base_link` (`g1_mask_measurement_node.BASE_FRAME_DEFAULT`), unlike the pointcloud and viz nodes which derive `<namespace>/robot/base_link` from `get_namespace()`. Under a namespace the bare default names a frame nothing publishes and polar profiling's scan→base lookup fails silently — it reports as that row missing, not as a bad frame. `mask_measurement_node()` therefore makes it a required keyword rather than an optional one
- **Exploration selects its rows through an `OpaqueFunction`** (`build_g1_perception_nodes`), because `parse_estimators` decides which measurement nodes exist at all and a substitution cannot be branched on until a context resolves it. Same helpers the benchmark uses. Both display surfaces are filtered from that one set, so a ring can never appear without a HUD column to name it. Exploration also carries **two `hud_node` instances**: the original one merges the plain-text `hud/velocity` and `hud/coverage` panels onto `hud_overlay`, and `hud_g1_node` merges the rich-text `hud/g1_distances` onto `hud_g1_overlay`. They cannot be one node — see the `rich_text` paragraph below; the g1 panel's `<span>` colours would switch the whole overlay to rich text and collapse the space-padded columns the other two align with. On the **real** robot the D435 publishes no cloud by default, so the `pointcloud` column there is a permanent `--` and the three mask rows carry the readout (`object_localization_documentation/object_localization_pipeline.md`)
- `g1_mask_measurement_node` publishes the three mask-based rows on `measurements/g1/mask`, and the beams polar profiling reduced on `visualization/g1/polar_rays` (`MarkerArray`)
- **Depth is obtained inside the mask node, at the detection stamp — there is no depth producer process.** `depth_source` picks a strategy from `perception/core/depth_sources.py`; the node subscribes to that strategy's *input* stream (raw camera depth, or the color frame for monocular), buffers it undecoded, and converts only the frame the detections were made on. A separate producer had to guess which stamp the detector would pick, and its independent thinning cost the depth rows ~90% of their coverage under sim load (`object_localization_documentation/aligned_depth_coverage.md`). Consequence for new code: receiving is cheap and unconditional, converting is deferred and demand-driven — do not reintroduce a cadence cap or a latest-wins slot on a stream something downstream matches by stamp. The frame is still published on `debug/g1/mask/aligned_depth` for the overlay panel, but that is debug-only and encoded only when subscribed
- `g1_overlay_node` renders the perception panels and **publishes the composite on `debug/g1/overlay`** (`bgr8`) for RViz's Image display. It has no standalone GUI. `max_cols` sets panels per row; `rgb_panel_labels` toggles the per-detection label block
- `clearpath/robot.yaml` deliberately leaves the RealSense color/depth stream profiles unspecified, preserving Clearpath's current 640x480 @ 30 defaults for both simulation and hardware. A higher-resolution hardware profile needs a backend-specific configuration so it does not silently triple the simulation pixel load
- `common/camera_inputs.py` is the sole pure camera-topic contract. `resolve_camera_inputs('simulation')` preserves the existing Gazebo topics; `resolve_camera_inputs('realsense')` selects the driver-owned `aligned_depth_to_color/image_raw` stream and leaves organized points optional. It imports no ROS/launch modules, keeps names namespace-relative, and applies the established `color_topic`, `camera_info_topic`, `depth_topic`, and `pointcloud_topic` arguments as validation-backed compatibility overrides. A hardware entrypoint still requires robot validation; do not claim it has been exercised from simulation.

Visualization conventions:
- Enable/disable of the G1 visualizations is done with **RViz Displays checkboxes only** — no `add_on_set_parameters_callback` anywhere in this repo, and none was added. A `MarkerArray` display renders one checkbox per marker namespace, which is why `polar_rays` splits into `polar/used`, `polar/dropped` and `polar/wedge`
- Marker builders live in `common/markers.py` and emit markers in the **scan's own frame**, so beam geometry is `angle_min + i*angle_increment` with no extrinsics; TF places them
- `g1_estimate_viz_node` drives its rings and its HUD panel from `perception/estimators.py`, so registering an estimator there is enough to make it appear in both. **Which of them a run shows is the `estimators` parameter**, resolved by the shared `parse_estimators` (so canonical order however the launch argument named them, and `all` — the declared default of both entrypoints — means the full registry). Both `g1_distance_benchmark.launch.py` and `ridgeback_exploration.launch.py` hand over the set they already resolved for the measurement nodes, so both surfaces are filtered from one set: an unselected row draws no ring, and a ring can never appear without a row beside it to name it. That filtering is what makes `all` safe as a default in either stack — a row for a path the run never launched could only ever print `-- miss`, the same word an estimator that ran and found nothing prints. `_ESTIMATOR_ID_BASE` still enumerates the whole registry: marker ids that shifted with the selection would let a leftover marker collide with another estimator's ring. HUD rows are coloured from the same `ESTIMATOR_COLOURS` table as the rings, at the luminance the row's state calls for: a fresh row is lifted only as far as `HUD_MIN_LUMINANCE` needs and otherwise untouched, an aged row is driven to `HUD_AGED_LUMINANCE` from whichever side it started on. `colour_at_luminance` is the one helper — blend toward white going up, scale all three channels going down (which holds hue *and* saturation). A dim multiplier would not work: a colour sitting exactly at the floor after its lift would clamp straight back to its fresh appearance
- **The rings, the HUD and the `polar_rays` layers all show one instance: the nearest.** Ranked by `nearest_instance_index` in `perception/estimators.py`, where a detection's distance is the first `PUBLIC_ESTIMATOR_ORDER` entry that produced one and ties break on the lower index. `pointcloud` heads that order, so it is the row that normally speaks for a detection; on the **real** robot the D435 publishes no cloud by default, so the fallthrough there is `projective_ranging`. Marker ids are therefore fixed per estimator (rings) and per namespace (rays), not per detection — an id that moved with the detection index would strand the previous instance's markers on screen for a full lifetime. The camera overlay is the exception and still labels every detection
- **Rings are placed off the robot FRONT, not the base origin.** Every estimator subtracts `ROBOT_FRONT_OFFSET_M` before publishing (`geometry.apply_vehicle_front_offset`, and `optical_to_base_planar`'s `front_offset_m` for the mask paths), but TF returns the base pose — so `_add_estimator_markers` calls `geometry.remove_vehicle_front_offset` before rotating a measurement out through `world_marker_point`. Skipping it puts every ring a constant 0.25 m nearer the robot than the distance printed beside it, which reads as sensor error rather than as a plotting fault. `world_marker_point` is module-level and pure so a ring's world position can be asserted without a node — see `test_estimate_viz.py`, and add a placement assertion to any new marker path
- **Every registered estimator places its own detection**, so every ring has a bearing of its own and there is one ring line width. `ESTIMATOR_POSITION_ATTRS` covering `PUBLIC_ESTIMATOR_ORDER` is what licenses that, and `test_estimate_viz` asserts it: a distance-only row added back would silently draw its ring down the boresight, wrong by the whole lateral component, with nothing marking it as a guess
- **The truth line expires on the message stamp, never on when it arrived, and names its own trial.** `perception/ground_truth.py` owns the single `truth_reading` gate (`TRUTH_MAX_AGE_S`, one definition, both the viz node and the camera overlay), and the runner puts the trial id in `header.frame_id` — not a TF frame, no more than the point is a point in one — so the HUD renders `truth 3.250 m  [bed_occluder_single]`. A receipt-time gate cannot tell a message delayed behind a full subscription queue from a fresh one: it reported the delay as freshness and pinned the *previous* trial's truth under the current trial's scene for a whole capture window, while the estimator rows moved on. Both consumers subscribe at depth 1 for the same reason, and the runner republishes every `TRUTH_PUBLISH_PERIOD_SEC` (well inside the gate) so one dropped message cannot blank the line
- **Estimator rows expire on two gates, and neither is the truth line's gate.** `partition_measurements` in `g1_estimate_viz_node` caches `(message, receipt_nanoseconds)` and asks two separate questions: *liveness*, on the RECEIPT time against `MARKER_LIFETIME_SEC` — has this producer gone quiet; and *validity*, on the STAMP against `MAX_OBSERVATION_AGE_S` (3 s, same order as `TRUTH_MAX_AGE_S`) — can this observation still describe the scene. This deliberately differs from `truth_reading`'s stamp-only rule, and the difference is load-bearing both ways: truth is trial-keyed, so no receipt gate can tell trial N-1's truth from trial N's, whereas estimator rows only ever ask "is this producer still running", which receipt time answers correctly. A mask measurement is stamped with the *detection* instant and only arrives after inference, segmentation and the depth lookup, so it is genuinely ~1.2 s old on arrival; charging that pipeline latency against a single 1.5 s stamp gate is what made the three mask rows blink — the pointcloud row shares the stamp with no pipeline behind it and repainted the row as `-- miss` at detector rate
- **Survivors split into the current batch and an aged remainder; both place rings, each against the pose at its own stamp.** The same-batch set is the messages carrying the newest stamp present, and it alone drives the shared `nearest_detection_index` — `merged_distance_reader`'s premise that indices name the same detections holds only *within* a detector batch, so an older message's index 0 may be a different robot. An aged message is therefore ranked **against itself**, for its ring exactly as for its HUD column. A lone lagging producer (mask-only run) is its own current batch. **Aged messages used to be barred from placing rings, and that was wrong** — it did not cost the mask rows an occasional ring, it cost them *every* ring: a mask measurement carries the detection instant and only arrives ~1.2 s later, after inference, segmentation and the depth lookup, so it is never the newest stamp while the pointcloud row is also running. Measured in exploration, 19 of 19 detected mask messages landed in the aged remainder and none in the batch, so the floor plan showed one ring out of four. What made a stale ring dishonest was its *placement*, not its age: `_publish_markers` now looks the robot pose up at each message's own stamp rather than taking the latest TF, so a 1.2 s old reading lands on the world point the sensor actually saw instead of being dragged and swung by however far the base has since driven and turned. `_robot_pose_in_world`'s zero timeout still stands (the stamp is always in the past, so waiting cannot help), and a lookup that falls outside the TF cache skips that one message rather than the whole frame
- **Both surfaces render on one timer, from one snapshot, and the tick is unconditional.** `hud_publish_rate_hz` (5.0, matching `hud_node`'s own sampler) drives `_render`, which calls `collect_readings` **once** and hands the result to both the panel and `_publish_markers`. That single `dict[str, EstimatorReading]` is the load-bearing part: a column and its ring are two renderings of one claim, and while the markers were event-driven off `_measurement_cb` and the panel ran on the timer, each re-read the cache — so a path's number and its ring could describe different instants, by a tick at best and by a whole marker lifetime at worst. **The old objection to putting markers on the timer is now answered:** they carry `lifetime`, so republishing unchanged data against a fresh stamp would renew it forever and no ring could expire — which is why an estimator with no reading is now sent an explicit `Marker.DELETE` (`_add_delete_markers`, ring and dot ids) instead of being waited out. `lifetime` stays only as the backstop for this node dying outright, which no delete could cover. The panel has always needed a tick of its own: `hud_node._on_panel` only overwrites its cached text and never expires, so a section that stops publishing latches its last numbers on screen while the rings correctly vanish. An empty cache therefore publishes `''`, which `hud_node._tick`'s falsy-panel filter collapses, and the marker side deletes every ring in the same breath. `has_messages`, not an empty `readings`, is what decides that empty string — a live producer whose fields are all NaN must still print `-- miss`. The timer is also the only thing that re-evaluates `truth_reading`: reachable only from a measurement callback it was a gate with no puller, and when detections stopped the previous trial's truth sat under an already-teleported target. `create_timer` uses the node clock, so a paused sim freezes the panel, the ages and the rings together
- The mask node ranks from its own three estimators only (nothing else has filled the batch yet) while the viz node ranks from `pointcloud` onward, so **two near-equidistant robots can split the ray layer from the ring layer** for a frame. Pinning both to the canonical order bounds this to near-ties; closing it entirely would need a published nearest-index topic
- **The distance HUD has two layouts, chosen by `hud_layout` (`rows` | `wide`).** `rows` is the default and the benchmark's: one labelled row per estimator, with the truth and error columns only a run with a truth source can fill. `wide` is exploration's — `ESTIMATOR_SHORT_LABELS` across the top, distances under them, ages under those, in fixed `HUD_WIDE_CELL_COLUMNS`-wide right-aligned cells, one `<span>` per **cell** so a column keeps its ring's hue and dims whole when its reading is aged. No truth line and no error column, because nothing publishes truth in exploration. Both layouts read the same `collect_readings` map, so they cannot disagree about which estimator is fresh, aged or missing, and both keep the `''`-when-empty contract. `partition_measurements` returns `(message, age_seconds)` for the **current batch too**, not only the aged remainder: a same-batch reading's age is the pipeline latency behind that row, which the wide layout prints under every column — a blank age cell is how it marks a miss, so a fresh column left blank would be indistinguishable from one
- `hud_node` has `horizontal_alignment` / `vertical_alignment` (default `left`/`top`, both the benchmark and exploration's `hud_g1_node` use `right`/`top`) and `rich_text`. **`rich_text` changes the text contract**: the overlay renders via `QStaticText`, which switches to rich text as soon as any HTML tag appears — there `\n` stops breaking lines and runs of spaces collapse, so panels must use `<br/>` and `&nbsp;`. This is why exploration runs two aggregators rather than one: `hud/velocity` and `hud/coverage` are plain and align their columns with runs of spaces, `hud/g1_distances` is unconditionally rich, and merging them would collapse the first two. `overlay_width` must cover the widest panel — the overlay **clips rather than wraps**, and the clipped column is the last one, so a box that is too narrow silently deletes exactly the information the layout was widened for
- **Dock placement is only expressible as a `QMainWindow State` hex blob** in the `.rviz` file. `rviz_common`'s `addPane()` hardcodes `Qt::LeftDockWidgetArea` and the config format has no per-display area key, so a wide panel with no blob lands in the narrow left dock. `benchmark.rviz` carries one (generated via `QMainWindow::saveState()`, dock objectNames = the pane names = the keys in that same block); `restoreState` fails silently on a name mismatch, so any change to it needs a screenshot, not a build. **`exploration.rviz`'s blob predates its `Perception overlay` pane and does not place it** — the overlay lands in the left dock until the blob is regenerated by dragging the pane to the bottom dock and saving the config. Never hand-write it and never adapt benchmark's: exploration has a different pane set (it carries `Rendering` and a bottom `Time`), and `restoreState` returning false restores *nothing*, so a near-miss loses the whole layout rather than just the new dock
- **`G1 Estimates` in `exploration.rviz` lists its four marker namespaces explicitly**, so the per-estimator checkboxes are configured rather than discovered at runtime. `ring.ns` and `dot.ns` are the same string (`g1_estimates/<estimator>`), which is what makes one checkbox drop both. The cost is that the names are now pinned in a config file: rename an estimator in `PUBLIC_ESTIMATOR_ORDER` and the block keeps a checkbox for a namespace nothing publishes while the new row arrives unconfigured. `test_launch_layout` guards it by asserting the block's keys equal `{f'g1_estimates/{e}' for e in PUBLIC_ESTIMATOR_ORDER}`

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
- The G1 overlay is published on `debug/g1/overlay` for the RViz Image display; it has no standalone GUI
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
