# Ridgeback Exploration, Target Localization, and Benchmarking

Autonomous exploration, reusable target localization, and distance benchmarking for a [Clearpath Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/) robot in Gazebo Harmonic with ROS 2 Jazzy. The shipped simulation benchmark currently uses a Unitree G1 as its target model; the ROS interfaces and localization package are target-generic.

This workspace supports 3 main human workflows:
- full autonomous exploration with target localization (any world)
- repeatable target-distance benchmarking across the point-cloud and mask-based measurements
- manual mapping

## Docs

- [Documentation index](docs/index.md): project context, architecture, benchmarking, plans, and historical validation
- `README.md`: installation, public launch usage, and normal human workflows
- `docs/ISSUES.md`: troubleshooting, resolved root causes, and operational gotchas
- `docs/project/`: project context, documentation ownership, and repository conventions
- `AGENTS.md` / `CLAUDE.md`: thin entrypoints into the shared agent guidance

## Stack

| Layer | Package | Purpose |
|-------|---------|---------|
| Simulation | Gazebo Harmonic + clearpath_simulator | Physics, sensors, world |
| Perception | Hokuyo UST-10LX 2D lidar | Obstacle detection + SLAM input |
| Perception | Intel RealSense D455 (~1m height) | Depth/RGB for overlay and distance estimation |
| SLAM | slam_toolbox (online async, from source) | Map building + localization |
| Navigation | Nav2 (MPPI omni controller) | Path planning + obstacle avoidance |
| Exploration | in-repo `frontier_explorer_node` | Frontier detection + goal selection |
| Perception | Staged target-localization pipeline | Raw OWLv2 detections plus the point-cloud and mask measurement nodes and an optional overlay |
| Benchmarking | Target-distance benchmark runner | Controlled evaluation of the point-cloud and mask-based localization paths |

## Prerequisites

- Ubuntu 24.04
- GPU with OpenGL support (for Gazebo)

## Installation

### 1. Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

```bash
sudo apt install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-ros-gz
```

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

### 2. Build the workspace

```bash
cd /path/to/DyNAMO-Ridgeback

# Clone external dependencies
vcs import < .repos

# Apply clearpath_gz patch (adds custom worlds/models + SpawnG1 Gazebo GUI plugin)
cd src/clearpath_simulator/clearpath_gz && git apply ../../../patches/clearpath_gz_customizations.patch && cd ../../..

# Apply slam_toolbox patch (fixes TF namespace issue)
cd src/slam_toolbox && git apply ../../patches/slam_toolbox_tf_namespace.patch && cd ../..

# Apply clearpath_common patch (camera model owns its own frames in simulation)
cd src/clearpath_common && git apply ../../patches/clearpath_realsense_sim_frames.patch && cd ../..

# Install any remaining deps
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --base-paths src
source install/setup.bash
```

If you rename or move the workspace directory later, regenerate `build/` and `install/` before rebuilding so the generated setup files do not keep stale absolute paths. Preserve `artifacts/`: its logs and benchmark results are independent evidence, not build products.

### 3. (Optional) Set up target localization venv

The target detection/localization stack (detection + point-cloud measurement +
overlay) is **on by default** — `ridgeback_exploration.launch.py` ships with
`target_localization_enabled:=true`, which **requires** the venv below; without it the
detector logs a single clear error and exits cleanly. Disable the whole stack
with `target_localization_enabled:=false` for a normal exploration run that needs no
venv and will not try to load any models.

The stack follows a staged pipeline inside the installable Python package `ridgeback_autonomy/`:
- raw detections on `detections/target/raw`
- point-cloud measurements on `measurements/target/pointcloud`
- mask-based measurements on `measurements/target/mask`

It requires a Python venv with PyTorch and Transformers. All dependencies
(pinned, with the correct CUDA torch index) live in
[`requirements-perception.txt`](requirements-perception.txt):

```bash
# Create venv that can see ROS 2 packages
python3 -m venv --system-site-packages perception_venv
echo "/opt/ros/jazzy/lib/python3.12/site-packages" > perception_venv/lib/python3.12/site-packages/ros2.pth

# Install pinned perception dependencies
perception_venv/bin/python3 -m pip install -U pip
perception_venv/bin/python3 -m pip install -r requirements-perception.txt
```

> **GPU note:** `requirements-perception.txt` pins the **CUDA 12.8 (`cu128`)**
> torch build, which carries the `sm_120` kernels needed for Blackwell GPUs
> (e.g. RTX PRO 6000). Installing plain `torch` from PyPI gives a CPU-only build
> and silently runs Depth-Anything / OWLv2 on the CPU. For a different
> GPU/CUDA, change the index URL + torch pins in that file.

The exploration and benchmark-environment launches automatically prepend `perception_venv/bin` to `PATH` and set `VIRTUAL_ENV` for the perception nodes. When you launch with `target_localization_enabled:=true`, the first run will download the OWLv2 detector from Hugging Face. A benchmark run with `depth_source:=monocular` downloads the Depth-Anything V2 metric checkpoint on first use, and one with `mask_gate:=silhouette` the SlimSAM segmentation checkpoint.

### 4. (Optional) Install graphify post-commit hook

If you use the graphify knowledge graph, install the post-commit hook to auto-rebuild it after each commit:

```bash
perception_venv/bin/python3 -m pip install graphifyy==0.8.35
bash tools/install_hooks
```

The hook only triggers on code-file changes and calls `tools/rebuild_graphify`. The hook source lives in `tools/hooks/post-commit`.

### 5. Set up robot config

The Clearpath simulator expects the robot config at `~/clearpath/`:

```bash
mkdir -p ~/clearpath
cp clearpath/robot.yaml ~/clearpath/robot.yaml
```

**If `~/clearpath/robot.yaml` already exists, diff before you overwrite it**, and
work out whether the differences are local customization or just staleness:

```bash
diff -u ~/clearpath/robot.yaml clearpath/robot.yaml
cp ~/clearpath/robot.yaml ~/clearpath/robot.yaml.bak   # rollback point before writing
```

This directory is a *deployment* location, so a copy sitting there can legitimately
carry host, namespace, or mount settings the repo does not know about — but it can
equally be an old `cp` that simply never got refreshed. The two cases call for
opposite actions, and only the diff tells them apart. Copying the repo file
wholesale replaces every setting at once; to change one sensor, edit that block in
place. Regenerate the setup's outputs afterwards, and treat writing to a *robot's*
deployed configuration as a separate, explicitly authorized step.

## Usage

Start every run from a sourced workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/DyNAMO-Ridgeback
source install/setup.bash
```

The package has 5 top-level launch files. The first two are the normal human
entrypoints; the benchmark environment/config pair are also public because the
sweep supervisor invokes them as separate processes:

- `ridgeback_exploration.launch.py`
- `target_distance_benchmark.launch.py`
- `target_benchmark_env.launch.py`
- `target_benchmark_config.launch.py`
- `manual_mapping.launch.py`

The lower-level simulation, SLAM, Nav2, and frontier-exploration launches live under `launch/includes/` and are treated as internal launch building blocks rather than public entrypoints.

### `ridgeback_exploration.launch.py`

Launches Gazebo, SLAM, Nav2, frontier exploration, and the target-localization stack in sequence:

1. Gazebo + Ridgeback spawn
2. Exploration RViz config
3. `slam_toolbox` (once the scan and filtered-odometry topics publish)
4. Nav2 (once `/map` publishes)
5. The in-repo `frontier_explorer_node` (once the global costmap publishes)
6. Target-localization nodes: `target_detector_node`, the measurement nodes for the selected `estimators` (`target_pointcloud_measurement_node` and/or `target_mask_measurement_node`), `target_visualization_node`, `target_overlay_node`, and a second `hud_node` for the distance panel

All four distance estimator rows run by default, the same set the benchmark compares — one RViz ring per row, each with its own bearing, plus a wide distance HUD top-right (`hud_target_overlay`) listing the four side by side with the age of each reading. There is no ground truth in exploration, so that panel carries no truth line and no error column. Per-row visibility is an RViz Displays checkbox under `Target Estimates`, one per estimator.

The perception overlay is published on `debug/target/overlay` and shown by the configured RViz Image display.

Bringup is **event-driven** (readiness gates), not fixed timers — each stage starts when its prerequisite exists, with a `--timeout` fallback. See [docs/ISSUES.md](docs/ISSUES.md), "Event-Driven Startup".

Available worlds:

| World | Source | Notes |
|-------|--------|-------|
| `mock_hospital` | Custom (`sim/worlds/`) | Default; detailed multi-room clinical layout |
| `warehouse` | Clearpath | Large open floor plan |
| `office` | Clearpath | Smaller rooms and corridors |
| `construction` | Clearpath | Outdoor construction site |
| `orchard` | Clearpath | Outdoor orchard rows |
| `solar_farm` | Clearpath | Outdoor solar panel array |
| `pipeline` | Clearpath | Outdoor pipeline facility |

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `world` | `mock_hospital` | Gazebo world to load (see table above) |
| `namespace` | `r100_0001` | ROS namespace for all nodes |
| `use_sim_time` | `true` | Use Gazebo `/clock` |
| `setup_path` | `~/clearpath/` | Directory containing `robot.yaml` and generated Clearpath files |
| `color_topic` | `sensors/camera_0/color/image` | Compatibility override for the shared camera contract's color image |
| `camera_info_topic` | `sensors/camera_0/color/camera_info` | Compatibility override for color-grid camera intrinsics |
| `depth_topic` | `sensors/camera_0/depth/image` | Compatibility override for aligned depth; a RealSense launch must set its driver-aligned topic |
| `pointcloud_topic` | `sensors/camera_0/points` | Compatibility override for organized points; pass an empty value only when no pointcloud estimator is selected |
| `exploration_rviz` | `true` | Launch the custom exploration RViz config |
| `target_localization_enabled` | `true` | Launch the target-localization stack |
| `estimators` | `all` | Distance estimator rows to run — `all`, or a comma-separated subset of `pointcloud`, `projective_ranging`, `euclidean_reconstruction`, `polar_profiling`. Selects the measurement nodes, the rings and the HUD columns from one list |
| `estimate_viz` | `true` | Publish the estimator rings and the wide distance HUD |
| `depth_source` | `stereoscopic` | Aligned depth source for the mask rows — `stereoscopic` or `monocular` (Depth-Anything V2; downloads a checkpoint on first use) |
| `mask_gate` | `box` | Mask front-end — `box` (no segmentation model) or `silhouette` (SlimSAM) |
| `mppi_visualize` | `false` | Publish MPPI trajectory visualization topics (RViz already has `MPPI Optimal` and `MPPI Samples` displays subscribed to `/r100_0001/optimal_trajectory` and `/r100_0001/trajectories`) |
| `headless_rendering` | `false` | Optional server-only EGL sensor rendering for SSH/non-seat sessions; the existing `tools/gpu-run` NVIDIA GLX workflow remains the default GUI-capable path (see [docs/ISSUES.md](docs/ISSUES.md), "Camera rate collapses") |

Examples:

```bash
# Always clean up stale processes first
bash cleanup.sh

ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=mock_hospital
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=warehouse exploration_rviz:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=mock_hospital target_localization_enabled:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=office headless_rendering:=true

# One ring instead of four: the pre-mask-row exploration stack
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py estimators:=pointcloud
```

The explorer is the in-repo `frontier_explorer_node` (sources under `src/ridgeback_autonomy/ridgeback_autonomy/frontier_explorer/`). It consumes the Nav2 global costmap and sends goals via `NavigateToPose`. It replaced `explore_lite` after a head-to-head benchmark — see [docs/ISSUES.md](docs/ISSUES.md), "Exploration Quits Early", for the findings.

#### Quick-start script

`start_exploration.sh` sources the workspace, runs cleanup, and launches exploration. It accepts an optional world as the first positional argument and forwards later `key:=value` launch arguments:

```bash
bash start_exploration.sh                                # mock_hospital
bash start_exploration.sh office                         # office world
bash start_exploration.sh headless_rendering:=true       # default world, EGL server rendering
bash start_exploration.sh office estimators:=pointcloud  # office with one estimator row
RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash start_exploration.sh office  # explicit override
```

The script defaults to CycloneDDS, selected by the exact-stamp image-delivery
benchmark. An explicit `RMW_IMPLEMENTATION` remains authoritative. The rejected
UDP-only FastDDS experiment is retained in the
[investigation history](docs/history/exact_stamp_depth_availability.md).

Every launch entrypoint also points `CYCLONEDDS_URI` at
`config/cyclonedds.xml`, which raises the participant-index ceiling. Exploration
starts 51 processes and aborts `slam_toolbox`, the whole Nav2 stack and the
explorer on Cyclone's default — see
[docs/ISSUES.md](docs/ISSUES.md), "Nav2 and slam_toolbox abort on CycloneDDS's
participant-index ceiling". Setting `CYCLONEDDS_URI` yourself overrides it; the
launch files only fill in a value when none exists.

`build_and_start_expl.sh` rebuilds the workspace first, then runs the same exploration quick-start (extra args are forwarded to `start_exploration.sh`):

```bash
bash build_and_start_expl.sh
bash build_and_start_expl.sh office
bash build_and_start_expl.sh office headless_rendering:=true
```

### `target_distance_benchmark.launch.py`

```bash
# Default run: compare all four estimators
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py

# Point cloud only
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=pointcloud

# Point cloud + the mask reference row
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=pointcloud,projective_ranging

# Mask-based localization rows, stereoscopic aligned depth
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction,polar_profiling

# Same rows on monocular (Depth-Anything) aligned depth — compare depth
# sources by running the benchmark twice, once per depth_source
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction depth_source:=monocular

# Silhouette mask front-end (segmentation model) instead of the rasterized
# box — compare gates by running the benchmark twice, once per mask_gate
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=projective_ranging,euclidean_reconstruction,polar_profiling mask_gate:=silhouette

# One mask row on its own — polar profiling needs no depth at all, so this run
# builds no depth source and subscribes to no depth stream
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py estimators:=polar_profiling
```

This compatibility launch composes two layers. `target_benchmark_env.launch.py` owns the simulator, camera TF, RViz, and `target_detector_node`; `target_benchmark_config.launch.py` waits for the color and warm-detector topics, then starts the selected measurement nodes, visualizations, HUD, overlay, and `target_distance_benchmark_runner`. Every row is individually selectable: `estimators` is split per stack and each measurement node is passed only the rows it owns, so a path that was not selected is never run — its fields stay NaN, it gets no CSV, and the inputs only it needs are never subscribed to. Selecting any mask row launches `target_mask_measurement_node`, which for `projective_ranging`/`euclidean_reconstruction` obtains the aligned depth frame itself, at the detection stamp, through the source `depth_source` selects; `polar_profiling` needs no depth at all, so a polar-only run builds no depth source (and under `depth_source:=monocular`, loads no model). The mask node builds one mask per detection (`mask_gate:=box` rasterizes the detection box; `mask_gate:=silhouette` prompts a segmentation model with the boxes, needs `perception_venv`) and runs the localization paths from `docs/localization/`.

The mask rows are identified by their config axes — the mask gate (`mask_gate`), the aligned-depth source (`depth_source`), the path, and on the box gate the foreground-isolation recipe (`isolation_2d` for projective ranging, `isolation_3d` for euclidean reconstruction) — so their CSV files fold the axes into a self-describing name. The stereoscopic box-gate run above writes `box_gated_stereoscopic_projective_ranging_nearest_mode_histogram.csv` and `box_gated_stereoscopic_euclidean_reconstruction_height_crop_nearest_mode_band.csv`; silhouette rows drop the isolation token (the tight branches never run a recipe), e.g. `silhouette_gated_stereoscopic_projective_ranging.csv`; polar profiling folds the gate only (`box_gated_polar_profiling.csv`). The `pointcloud` row keeps its plain name.

Each benchmark run writes under
`artifacts/benchmarks/<timestamp>_<scenario>[_<gate>][_<depth_source>]/` by default
(e.g. `20260822_202851_v2_silhouette_stereoscopic`):
- `summary.md` — the readable report; start here
- one trial-level CSV per selected estimator
- one `run.json` — the run-level numbers for machines, plus provenance
- one shared collage image per included trial under `images/`
- one best-effort RViz recording at `video/run.mp4`

Pass `output_dir:=...` to write the timestamped run folder somewhere else.

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `namespace` | `r100_0001` | Namespace for simulation, detector, measurement node, and benchmark runner |
| `use_sim_time` | `true` | Use Gazebo `/clock` |
| `setup_path` | `~/clearpath/` | Directory containing `robot.yaml` and generated Clearpath files |
| `world` | `target_distance_calibration` | Gazebo world used for the benchmark run |
| `gz_gui` | `true` | Launch Gazebo's GUI; set `false` for unattended benchmark runs |
| `estimators` | `all` | Comma-separated estimator subset to compare in one run: `pointcloud`, `projective_ranging`, `euclidean_reconstruction`, `polar_profiling`. Any subset works, rows individually — `estimators:=polar_profiling` runs that row alone |
| `depth_source` | `stereoscopic` | Aligned-depth producer for the `projective_ranging`/`euclidean_reconstruction` rows: `stereoscopic` or `monocular`; comparing sources = two runs |
| `mask_gate` | `box` | Mask front-end for the mask-based rows: `box` (rasterized detection box, no model) or `silhouette` (segmentation model prompted with the boxes; needs `perception_venv`); comparing gates = two runs |
| `depth_match_debug` | `false` | Default-off mask-worker evidence logging: exact-depth delivery, pending replacements, batch completion/publication age, and bounded cold/warm percentiles for RGB, SlimSAM, depth production, mask preparation, estimator reduction, worker, lock, and CUDA-synchronization timing |
| `detector_fps` | `10.0` | Upper bound on the detection rate every measurement row inherits. Belongs to the persistent environment layer, so in a sweep it is a `defaults` key and cannot vary per configuration |
| `detector_debug` | `false` | Default-off detector evidence logging: achieved cadence, superseded frames, and bounded cold/warm percentiles for the throttle wait, decode, inference, parse, publish and CUDA synchronization. Environment-layer, like `detector_fps` |
| `headless_rendering` | `false` | Optional default-off server-only EGL sensor rendering. In sweeps this is a persistent-environment `defaults` key and cannot vary per configuration; `tools/gpu-run` remains the GUI-capable NVIDIA GLX path |
| `isolation_2d` | `nearest_mode_histogram` | Projective-ranging box-gate foreground recipe: `nearest_mode_histogram` or `otsu` |
| `isolation_3d` | `height_crop_nearest_mode_band` | Euclidean-reconstruction box-gate foreground recipe: `height_crop_nearest_mode_band`, `height_crop_range_band`, `height_crop`, `nearest_mode_band`, or `range_band`. The two chains differ only in how the background separator anchors — nearest mode vs. percentile; the percentile one slides as background grows and is what the `pointcloud` row does |
| `isolation_3d_floor_margin_m` | `0.05` | Height above the floor a point must clear in Euclidean recipes containing `height_crop` |
| `isolation_3d_percentile` | `25.0` | Range percentile used as the anchor by the Euclidean `range_band` step |
| `isolation_3d_ahead_m` | `0.10` | Euclidean inlier window retained in front of its range anchor |
| `isolation_3d_behind_m` | `0.35` | Euclidean inlier window retained behind its range anchor |
| `isolation_3d_bin_width_m` | `0.05` | Euclidean range-histogram bin width used by `nearest_mode_band` |
| `isolation_3d_min_bin_fraction` | `0.05` | Fraction of Euclidean ranges a histogram bin must hold before it may anchor `nearest_mode_band` |
| `mask_depth_max_meters` | `0.0` | Working depth gate for the mask rows; `0` means no gate, leaving each row bounded only by what its depth source declares it can resolve |
| `isolation_2d_bin_width_m` | `0.05` | Depth-histogram bin width of the selected `isolation_2d` recipe. Applies to whichever recipe is chosen — both bin the masked depths |
| `isolation_2d_band_m` | `0.35` | Depth band kept around the near-surface anchor. `nearest_mode_histogram` only; `otsu` has no band and ignores it |
| `isolation_2d_min_bin_fraction` | `0.05` | Fraction of the masked depths a histogram bin must hold before the anchor may sit on it. `nearest_mode_histogram` only |
| `min_valid_pixels` | `10` | Shared foreground-sample floor for projective ranging and Euclidean reconstruction; both depth rows reject a prepared selection below it |
| `camera_info_topic` | `sensors/camera_0/color/camera_info` | Compatibility override for the shared camera contract's color-grid intrinsics |
| `repeats` | `5` | Number of positive-trial repeats per spawn pose |
| `output_dir` | `<repo-root>/artifacts/benchmarks` | Root directory that will receive one timestamped subfolder per run |
| `run_dir_name` | empty | Optional exact run-folder name under `output_dir`; sweeps use the configuration name. Empty preserves the timestamped single-run naming |
| `shutdown_on_complete` | `false` | Shut down the config launch service when the runner exits. The sweep sets this to `true`; the compatibility launch leaves the completed stack open for inspection |
| `settle_sec` | `2.0` | Delay after spawning the target before sampling |
| `capture_sec` | `10.0` | Sampling window length for collecting usable detections |
| `replay_dataset_dir` | empty | New directory for the compact legacy measurement replay dataset. Setting it switches this run from elapsed-time capture to an exact raw-batch quota and requires projective ranging, box gating, and stereoscopic depth |
| `sensor_capture_dir` | empty | New immutable full sensor-capture artifact for `mask-output`/`mask-model` replay. Stores exact RGB, float32 aligned depth, raw detections, calibration, and transforms; mutually exclusive with `replay_dataset_dir` |
| `capture_batches` | `5` | Raw detector batches captured per trial when either replay output is set; empty batches count. Five was selected by the 2026-09-09 convergence run |
| `capture_drain_sec` | `2.0` | Maximum post-quota drain for matching exact depth, camera context, and live measurement messages. Capture ends early when every selected stamp is complete and preserves missing matches when the bound expires |
| `capture_timeout_sec` | `30.0` | Hard wall-time bound for obtaining the raw detector-batch quota; it is a stall guard, not the normal capture duration |
| `color_topic` | `sensors/camera_0/color/image` | Compatibility override for the shared camera contract's color image; feeds detector, measurements, overlay, runner, and readiness gate |
| `depth_topic` | `sensors/camera_0/depth/image` | Simulation's color-aligned depth. A hardware RealSense launch must override this to `sensors/camera_0/aligned_depth_to_color/image_raw` |
| `pointcloud_topic` | `sensors/camera_0/points` | Simulation's organized point cloud. It is optional on RealSense; omit the pointcloud estimator or override this only after confirming driver output |
| `scan_topic` | `sensors/lidar2d_0/scan` | LaserScan topic used by polar profiling |
| `base_frame` | `<namespace>/robot/base_link` | Vehicle frame used for point-cloud and scan projection |

Benchmark semantics:
- only positive spawned-target trials are kept
- only successful single-target detections are used
- a trial is included only if every selected estimator has a usable aligned event
- each estimator CSV stores one row per included trial, using the median estimate over that trial’s aligned usable detections
- the shared collage image for each trial is built from one representative aligned detection event that is closest to the per-trial medians across the selected estimators

### Local benchmark configurator

Use the local configurator to build and validate a benchmark job without
starting Gazebo or writing benchmark artifacts. It binds only to loopback and
opens a browser by default; use `--no-open` on a remote or headless session.

```bash
ros2 run ridgeback_autonomy target_benchmark_configurator
ros2 run ridgeback_autonomy target_benchmark_configurator --no-open
```

The UI exports a canonical replay-job YAML plus its exact command. It supports
legacy measurement replay, frozen mask-output comparison, rerunnable
mask-model materialization, and live-system sweeps. It validates selected
artifacts against the same replay-job contract used by the CLI. See
[benchmarking reference](docs/benchmarking/target_distance_benchmarking.md).

### Benchmark sweeps

Use the sweep supervisor when comparing configurations. It starts Gazebo,
RViz, the Ridgeback, camera TF, and the detector once, then restarts only the
measurement/configuration layer for each entry. The detector therefore loads
OWLv2 once per sweep; silhouette configurations still load their segmentation
model inside their per-config mask node.

```bash
SWEEP="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_sweep_baseline.yaml"

# Validate all configs and print the trial/time estimate without launching.
ros2 run ridgeback_autonomy target_benchmark_sweep "$SWEEP" --dry-run

# Run all 4 baseline configurations against one simulator boot.
ros2 run ridgeback_autonomy target_benchmark_sweep "$SWEEP"

# Run a named subset.
ros2 run ridgeback_autonomy target_benchmark_sweep "$SWEEP" \
  --only pointcloud,mask_polar_profiling

# Rebuild the cross-config report for a partial or completed sweep.
ros2 run ridgeback_autonomy target_benchmark_sweep \
  --report-only artifacts/benchmarks/20260828_141530_baseline
```

The shipped `benchmark_sweep_model_concurrency.yaml` is the controlled
sequential evidence run for the SlimSAM/Depth-Anything question. It starts with
a diagnostics-off/on A/A pair and then runs three rotated replications of the
box/silhouette by stereoscopic/monocular matrix. Run it only from a clean,
committed checkout; its `sweep.json` records the exact sweep and scenario file
hashes. See [the model-concurrency evidence plan](docs/plans/model_concurrency_evidence.md)
for the decision gates and interpretation.

The shipped `benchmark_sweep_projective_parameters.yaml` varies one projective
ranging tuning constant at a time — `isolation_2d_band_m`,
`isolation_2d_min_bin_fraction`, `isolation_2d_bin_width_m` and
`min_valid_pixels` — around their current values, and repeats the default
configuration twice as an in-sweep noise control. See
[projective parameter sensitivity](docs/history/projective_parameter_sensitivity.md)
for what is already settled and what those runs are meant to answer.

Do **not** run `cleanup.sh` between configurations: it kills Gazebo and RViz,
which are deliberately persistent. The supervisor runs it once, immediately
before starting that environment. It then owns each config process group,
removes any orphan `bench_*` entities after an unclean exit, and shuts the
environment down at the end. If a sweep is interrupted, rerun the same command:
the latest incomplete sweep with the same YAML and `--only` selection is resumed,
valid `run.json` configurations are skipped, and partial config folders are
preserved with an `.incomplete_<timestamp>` suffix before retry.

The YAML format is a metadata block, sweep-wide defaults, and named configs:

```yaml
sweep:
  name: baseline
  description: Every estimator once at its defaults.

defaults:
  scenario: ''       # packaged benchmark_scenarios_full.yaml
  repeats: 1

configs:
  - name: pointcloud
    estimators: pointcloud
  - name: mask_projective_ranging
    estimators: projective_ranging
    mask_gate: box
    depth_source: stereoscopic
    isolation_2d: nearest_mode_histogram
```

Config values override `defaults`. Validation happens before Gazebo starts:
unknown arguments, duplicate/unsafe names, bad estimators, missing scenario
files, environment keys on individual configs, and estimator-inapplicable
knobs are rejected. `world`, `setup_path`, `namespace`, `use_sim_time`, and
`color_topic` may be set only in `defaults` because the environment cannot
change mid-sweep.

Use `--skip-preflight-cleanup` only when another same-user ROS/Gazebo session
must remain alive. The sweep then skips the aggressive repository cleanup and
warns that any stale processes are the operator's responsibility.

Outputs use one timestamped sweep folder with a resumable manifest and a
comparison report:

```text
artifacts/benchmarks/20260828_141530_baseline/
  sweep.json
  summary.md
  pointcloud/
    run.json  summary.md  pointcloud.csv  images/  video/run.mp4
  mask_projective_ranging/
    ...
```

`summary.md` contains one row per `(config, estimator)`, sorted by MAE. It also
records each configuration's wall time and a pre-run real-time-factor sample;
RTF is diagnostic only, but helps distinguish configuration effects from
observation-coverage drift as a long-lived simulator slows down.

### Legacy measurement replay

Use replay when only the box-gated stereoscopic `projective_ranging` recipe or
its numeric parameters change. One live run freezes raw detections, exact-stamp
depth ROIs, intrinsics, transforms, and ground truth; the offline command then
evaluates every configuration in the existing projective sweep over those same
inputs. It does not replace live runs for detector, transport, throughput,
latency, model, or final-integration questions.

Both the dataset directory and offline output directory must be new. This
five-scene command is a smoke workflow and uses the selected five-batch default.

```bash
SCENARIO="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_scenarios_examples.yaml"
SWEEP="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_sweep_projective_parameters.yaml"
DATASET="$PWD/artifacts/benchmarks/replay_projective_smoke_dataset"
LIVE_OUTPUT="$PWD/artifacts/benchmarks/replay_projective_smoke_live"
OFFLINE_OUTPUT="$PWD/artifacts/benchmarks/replay_projective_smoke_offline"

ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  scenario:="$SCENARIO" repeats:=1 estimators:=projective_ranging \
  mask_gate:=box depth_source:=stereoscopic record_video:=false gz_gui:=false \
  output_dir:="$LIVE_OUTPUT" run_dir_name:=matching_live \
  replay_dataset_dir:="$DATASET" \
  capture_drain_sec:=2.0 capture_timeout_sec:=30.0 \
  shutdown_on_complete:=true

ros2 run ridgeback_autonomy target_offline_replay_benchmark \
  "$DATASET" "$SWEEP" --output-dir "$OFFLINE_OUTPUT" --workers 4
```

Capture counts raw detector batches rather than elapsed seconds. Once the quota
arrives, it drains only until every selected stamp has matching depth, camera
context, and a live measurement or `capture_drain_sec` expires. Missing matches
remain explicit replay evidence. Any skipped trial leaves the dataset `incomplete`, and the
offline loader refuses it instead of silently comparing a reduced scenario set.

The offline root contains `summary.md` and `sweep.json` for the cross-variant
comparison, `replay.json` for dataset/evaluator provenance and total evaluation
time, and one normal CSV/`run.json`/`summary.md` set per variant. The clean
2026-09-09 full-scenario validation captured 109/109 trials, matched all 135
live rows, and reduced the estimated 6.99-hour 15-variant sweep to 8 minutes 22
seconds end to end (about 50x). Five batches is therefore the legacy capture default. On
the measured 32-thread host, 16 workers was the replay knee; choose workers for
the machine rather than blindly using every logical CPU. See
[`docs/history/offline_measurement_replay_validation.md`](docs/history/offline_measurement_replay_validation.md).

### Layered mask replay

Use the typed replay path when masks themselves must be compared or rerun. The
four profiles and their legal axes are available without ROS or model imports:

```bash
ros2 run ridgeback_autonomy target_replay_describe --json
```

Capture one full sensor parent with the existing benchmark launch. The capture
still counts raw batches, including empty batches, and stores missing exact RGB,
depth, or context explicitly after the bounded drain.

```bash
SCENARIO="$(ros2 pkg prefix ridgeback_autonomy)/share/ridgeback_autonomy/config/benchmark_scenarios_examples.yaml"
SENSOR="$PWD/artifacts/benchmarks/sensor_capture"
LIVE_OUTPUT="$PWD/artifacts/benchmarks/sensor_capture_live"

ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  scenario:="$SCENARIO" repeats:=1 estimators:=projective_ranging \
  mask_gate:=box depth_source:=stereoscopic record_video:=false gz_gui:=false \
  output_dir:="$LIVE_OUTPUT" run_dir_name:=sensor_capture_live \
  sensor_capture_dir:="$SENSOR" capture_batches:=5 \
  capture_drain_sec:=2.0 capture_timeout_sec:=30.0 \
  shutdown_on_complete:=true
```

Derive immutable caches independently. A changed checkpoint, revision, prompt
padding, IoU floor, device/dtype, dependency version, or code provenance yields
a different producer signature and artifact identity.

```bash
ros2 run ridgeback_autonomy target_replay_materialize_masks \
  "$SENSOR" --producer box \
  --output-dir "$PWD/artifacts/benchmarks/masks_box"

ros2 run ridgeback_autonomy target_replay_materialize_masks \
  "$SENSOR" --producer slimsam \
  --model Zigeng/SlimSAM-uniform-50 \
  --output-dir "$PWD/artifacts/benchmarks/masks_slimsam"
```

The generalized executor accepts a canonical job YAML/JSON. `mask-output`
requires the sensor parent plus one or more caches; `mask-model` requires the
sensor parent plus `materializations` and creates its caches once before running
all measurement variants.

```yaml
job_version: 1
question: compare-mask-outputs
profile: mask-output
inputs:
  sensor_capture: ./artifacts/benchmarks/sensor_capture
  mask_caches:
    - ./artifacts/benchmarks/masks_box
    - ./artifacts/benchmarks/masks_slimsam
sweep:
  sweep:
    name: paired_mask_measurements
    description: Compare two measurement bands over every frozen mask cache.
  configs:
    - name: baseline
      estimators: projective_ranging,euclidean_reconstruction
      isolation_2d_band_m: 0.35
    - name: wider_2d_band
      estimators: projective_ranging,euclidean_reconstruction
      isolation_2d_band_m: 0.50
resources:
  measurement_workers: 4
comparison_baseline: masks_box:baseline
output_dir: ./artifacts/benchmarks/paired_masks
```

```bash
ros2 run ridgeback_autonomy target_replay_benchmark replay-job.yaml
```

Outputs appear only after the whole job succeeds. Offline reports state their
profile, supported claims, artifact hashes, cache lineage, worker count, and
limitations. Mask-production time is diagnostic; only the existing live-system
benchmark can support transport, throughput, end-to-end latency, GPU-contention,
or integration claims.

### Perception interfaces

| Node | Output topic | Key params |
|------|--------------|------------|
| `target_detector_node` | `detections/target/raw` | `color_topic`, `detection_model`, `detection_threshold`, `detector_fps` (default `10.0`; an upper bound on *step starts*, so the achieved rate matches the setpoint until inference alone exceeds the period), `detector_debug` |
| `target_pointcloud_measurement_node` | `measurements/target/pointcloud` | `color_topic`, `pointcloud_topic`, `base_frame`, `enabled_estimators` |
| `target_mask_measurement_node` | `measurements/target/mask` (+ `debug/target/mask` on the silhouette gate, + `debug/target/mask/aligned_depth` when a depth path is enabled, + `visualization/target/polar_rays` when polar profiling is) | `enabled_estimators`, `depth_source`, `depth_topic`, `camera_info_topic`, `scan_topic`, `base_frame` (**must be passed** — its own default is the bare `base_link`, unlike the other nodes', which are namespace-derived), `pitch_deg`, `front_offset_m`, `isolation_2d` and its three numeric settings, `min_valid_pixels`, `isolation_3d` and its six numeric settings, `mask_gate`, `segmentation_model`, `color_topic`, `ray_marker_topic` |
| `target_overlay_node` | `debug/target/overlay` | `measurement_topic`, `mask_measurement_topic`, `color_topic`, `aligned_depth_topic`, `estimators`, `max_cols`, `rgb_panel_labels` |
| `target_visualization_node` | `visualization/target/estimates` + `hud/target_distances` | `base_frame`, `world_frame`, `marker_lifetime_sec`, `ground_truth_topic`, `estimators` (gates both the HUD rows and the rings; defaults to `all`), `hud_layout` (`rows` — the benchmark's, with truth and error columns — or `wide`, exploration's estimator columns with an age under each) |
| `target_distance_benchmark_runner` | per-estimator CSVs + summary CSV + trial collage images + `video/run.mp4` | `estimators`, `output_dir`, `pointcloud_measurement_topic`, `mask_measurement_topic`, `color_topic`, `record_video`, `record_fps` |

Camera intrinsics come from the colour camera's `CameraInfo`. There is no intrinsics config file to tune — `config/camera_config.json` and its loader were deleted once the last estimator that read their FoV constants was removed.

## Configuration

### Robot sensors (`clearpath/robot.yaml`)

- **ROS middleware**: The checked-in Clearpath setup selects CycloneDDS
  (`rmw_cyclonedds_cpp`), matching the measured exact-stamp delivery default.
  Deploy it to the robot only through an approved configuration diff.
- **Hokuyo UST-10LX**: Mounted at front of chassis, provides 2D laser scan for SLAM and costmaps
- **Intel RealSense D455**: Mounted on a riser bracket. `device_type: d455` selects both the driver's device filter and the URDF model, so it decides the camera's geometry as well as which device the driver binds to. Configured streams are RGB and depth with `align_depth.enable: true` and `enable_sync: true`; stream profiles are left unspecified so Clearpath's 640x480 @ 30 defaults apply. It supplies RGB and aligned depth to the target-localization stack. An organized point cloud is enabled in the checked-out Clearpath defaults but is **not** wired as an application input — its layout, frame, and timestamps are unverified on hardware. The D455 also has an IMU, which this stack neither enables nor consumes.

### Key parameters to tune

| File | Parameter | Effect |
|------|-----------|--------|
| `config/frontier_explorer_params.yaml` | `min_frontier_size` / `near_frontier_radius` / `goal_advance_cells` | Frontier clustering, near-tier preference, and goal placement past the centroid |
| `config/frontier_explorer_params.yaml` | `progress_timeout` / `abort_blacklist_threshold` | Stall timeout (cancels only when no progress) and failures required before a frontier is blacklisted |
| `config/frontier_explorer_params.yaml` | `distance_weight` / `size_weight` | Scoring trade-off between how close vs. how large a far-tier frontier is |
| `config/frontier_explorer_params.yaml` | `goal_cost_threshold` / `goal_safety_margin` / `lethal_cost_threshold` | Goal-safety filtering against the costmap |
| `config/nav2_params.yaml` | `vx_max` / `vy_max` / `wz_max` | Robot linear and turn-rate limits |
| `config/nav2_params.yaml` | `max_accel` / `max_decel` | Velocity smoother acceleration and braking limits |
| `config/slam_toolbox_params.yaml` | `resolution` | Map resolution (m/pixel) |

## Generated artifacts

Repository-owned logs and results live under the Git-ignored `artifacts/` directory:

| Directory | Contents |
|-----------|----------|
| `artifacts/colcon/` | Colcon build/test logs; older logs are preserved under `history/` |
| `artifacts/exploration/<run>/` | `console.log` and `ros/` for each quick-start launch; older flat `.log` files remain alongside these folders |
| `artifacts/benchmarks/<run-or-sweep>/` | Benchmark JSON/CSV reports, images, video, and child-process logs |

`colcon_defaults.yaml` sets the log base for Colcon commands run from the workspace root. Use `colcon --log-base /somewhere/else build ...` to override it. The build helper also uses an absolute log base when invoked from elsewhere and honors `COLCON_LOG_PATH`.

`LOG_DIR` overrides the exploration output root, and `output_dir:=...` (or a sweep's `defaults.output_dir`) overrides benchmark output. `ROS_LOG_DIR` always overrides the managed child ROS-log locations. Without that override, sweep environment logs live in `<sweep>/environment/logs/ros/`, config logs in `<sweep>/<config>/logs/ros/`, and standalone runner command logs in `<run>/logs/ros/`.

Direct `ros2 launch` commands still use ROS's normal log destination unless you set `ROS_LOG_DIR` before launching, for example `ROS_LOG_DIR="$PWD/artifacts/ros" ros2 launch ...`. Existing `~/.ros/log` and `/tmp` logs are not moved.

Existing runs were relocated without rewriting their contents: absolute paths recorded in old run metadata describe the original execution location. Reports and sweep resume use the relocated folders. Do not treat benchmark results as disposable logs; no automatic pruning is enabled.

## Patches and Issue History

This project still relies on three local patches:

1. `patches/clearpath_gz_customizations.patch` patches `src/clearpath_simulator/clearpath_gz` to add this repo's Gazebo worlds/models to the simulator search path and to expose the custom `SpawnG1` Gazebo GUI plugin.
2. `patches/slam_toolbox_tf_namespace.patch` patches `src/slam_toolbox` so `slam_toolbox` respects namespaced TF remappings.
3. `patches/clearpath_realsense_sim_frames.patch` patches `src/clearpath_common` (recorded against revision `9960354`) so `intel_realsense.urdf.xacro` forwards `is_sim` into the camera macro as `use_nominal_extrinsics`, and so the Gazebo render sensor sits on the model's colour frame. Without it, simulation has no `camera_0_color_optical_frame` TF and the mask estimators report `TF_MISS_EXTRINSIC`.

Because the patches are recorded against specific upstream revisions, `.repos`
pins every dependency to an exact commit rather than to a branch tip. Do not
change a pin to `jazzy`/`main` to pick up a fix: a fresh `vcs import` would then
clone commits the patches were never rebased onto. As of 2026-09-07 the pinned
`clearpath_common` was 39 commits behind its branch tip, and the realsense patch
did not apply to that tip. Refreshing the pins is a deliberate, gated task — see
"Upstream dependency refresh" in [BACKLOG.md](docs/BACKLOG.md).

Each patch is a plain `git apply`, which is not idempotent. Check before re-running:

```bash
# "not applied" -> apply it; "already applied" -> skip; anything else -> local conflict, resolve by hand
cd src/clearpath_common
git apply --check      ../../patches/clearpath_realsense_sim_frames.patch && echo "not applied"
git apply --reverse --check ../../patches/clearpath_realsense_sim_frames.patch && echo "already applied"
```

Never re-run an apply that failed, and never resolve a conflict by discarding unrelated changes in the dependency checkout.

The deeper root-cause notes, previous middleware workarounds, namespace gotchas, and troubleshooting tips now live in [ISSUES.md](docs/ISSUES.md).
