# Ridgeback Exploration, Perception, and G1 Benchmarking

Autonomous exploration, G1 perception, and distance benchmarking for a [Clearpath Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/) robot in Gazebo Harmonic with ROS 2 Jazzy.

This workspace supports 2 main human workflows:
- full autonomous exploration with G1 perception (any world)
- repeatable G1 distance benchmarking across camera and LiDAR measurements

## Docs

- `README.md`: installation, public launch usage, and normal human workflows
- `ISSUES.md`: troubleshooting, resolved root causes, and operational gotchas
- `AI_CONTEXT.md`: agent-facing repo conventions, mental model, and documentation rules
- `AGENTS.md` / `CLAUDE.md`: thin entrypoints into the shared agent guidance

## Stack

| Layer | Package | Purpose |
|-------|---------|---------|
| Simulation | Gazebo Harmonic + clearpath_simulator | Physics, sensors, world |
| Perception | Hokuyo UST-10LX 2D lidar | Obstacle detection + SLAM input |
| Perception | Intel RealSense D455 (~1m height) | Depth/RGB for overlay and distance estimation |
| SLAM | slam_toolbox (online async, from source) | Map building + localization |
| Navigation | Nav2 (MPPI omni controller) | Path planning + obstacle avoidance |
| Exploration | explore_lite (m-explore-ros2) or in-repo `frontier_explorer_node` | Frontier detection + goal selection (selectable via `explorer:=`) |
| Perception | Staged G1 perception pipeline | Raw OWLv2 detections plus camera/LiDAR measurement nodes and an optional overlay |
| Benchmarking | G1 distance benchmark runner | Controlled evaluation of RGB, depth, mono-depth, pointcloud, and LiDAR measurements |

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

# Install any remaining deps
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --base-paths src
source install/setup.bash
```

If you rename or move the workspace directory later, wipe `build/`, `install/`, and `log/` before rebuilding so the generated setup files do not keep stale absolute paths.

### 3. (Optional) Set up G1 perception venv

The G1 perception/positioning stack (detection + camera/lidar measurement +
overlay) is **on by default** — `ridgeback_exploration.launch.py` ships with
`g1_perception_enabled:=true`, which **requires** the venv below; without it the
detector logs a single clear error and exits cleanly. Disable the whole stack
with `g1_perception_enabled:=false` for a normal exploration run that needs no
venv and will not try to load any models.

The stack follows a staged pipeline inside the installable Python package `ridgeback_autonomy/`:
- raw detections on `detections/g1/raw`
- camera measurements on `measurements/g1/camera`
- lidar measurements on `measurements/g1/lidar`
- mono-depth debug images on `debug/g1/camera/mono_depth`

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

Both public launch files automatically prepend `perception_venv/bin` to `PATH` and set `VIRTUAL_ENV` for the perception nodes. When you launch with `g1_perception_enabled:=true`, the first run will download the OWLv2 detector from Hugging Face. If you also pass `depth_anything_enabled:=true`, the first run will download the Depth-Anything V2 metric checkpoint.

### 4. (Optional) Install graphify post-commit hook

If you use the graphify knowledge graph, install the post-commit hook to auto-rebuild it after each commit:

```bash
pip install graphify              # or: pipx install graphify
bash tools/install_hooks
```

The hook only triggers on code-file changes and calls `tools/rebuild_graphify`. The hook source lives in `tools/hooks/post-commit`.

### 5. Set up robot config

The Clearpath simulator expects the robot config at `~/clearpath/`:

```bash
mkdir -p ~/clearpath
cp clearpath/robot.yaml ~/clearpath/robot.yaml
```

## Usage

Start every run from a sourced workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/DyNAMO-Ridgeback
source install/setup.bash
```

The package now exposes 2 public launch entrypoints:
- `ridgeback_exploration.launch.py`
- `g1_distance_benchmark.launch.py`

The lower-level simulation, SLAM, Nav2, and frontier-exploration launches live under `launch/includes/` and are treated as internal launch building blocks rather than public entrypoints.

### `ridgeback_exploration.launch.py`

Launches Gazebo, SLAM, Nav2, frontier exploration, and the G1 perception stack in sequence:

1. Gazebo + Ridgeback spawn
2. Exploration RViz config
3. `slam_toolbox` (once the scan + filtered-odom topics publish)
4. Nav2 (once `/map` publishes)
5. The selected explorer — `explore_lite` (default) or the in-repo `frontier_explorer_node` (once the global costmap publishes)
6. G1 perception nodes: `g1_detector_node`, `g1_camera_measurement_node`, `g1_lidar_measurement_node`, `g1_overlay_node`

Bringup is **event-driven** (readiness gates), not fixed timers — each stage starts when its prerequisite exists, with a `--timeout` fallback. See [ISSUES.md](ISSUES.md) "Event-Driven Startup".

The perception overlay appears in a separate OpenCV window named `G1 Perception`; it is not embedded in RViz.

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
| `exploration_rviz` | `true` | Launch the custom exploration RViz config |
| `g1_perception_enabled` | `true` | Launch the G1 perception stack |
| `depth_anything_enabled` | `false` | Enable Depth-Anything in the camera measurement node |
| `mppi_visualize` | `false` | Publish MPPI trajectory visualization topics (RViz already has `MPPI Optimal` and `MPPI Samples` displays subscribed to `/r100_0001/optimal_trajectory` and `/r100_0001/trajectories`) |
| `explorer` | `explore_lite` | Frontier explorer to dispatch — `explore_lite` or `custom` (the in-repo `frontier_explorer_node`) |
| `headless_rendering` | `false` | Render Gazebo server sensors via EGL without an X display — GPU-accelerated sensor rendering for SSH/non-seat sessions (see [ISSUES.md](ISSUES.md) "Simulation RTF Collapse") |

Examples:

```bash
# Always clean up stale processes first
bash cleanup.sh

ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=mock_hospital
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=warehouse exploration_rviz:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=office depth_anything_enabled:=true
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=mock_hospital g1_perception_enabled:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=office explorer:=custom
```

The `custom` explorer is the in-repo `frontier_explorer_node` (sources under `src/ridgeback_autonomy/ridgeback_autonomy/frontier_explorer/`). It and `explore_lite` both consume the Nav2 global costmap and send goals via `NavigateToPose`; pick whichever you want to evaluate.

#### Quick-start script

`start_exploration.sh` sources the workspace, runs cleanup, and launches exploration. Depth-Anything stays disabled unless explicitly enabled. The script accepts the world as the first positional arg and the explorer (`explore_lite` or `custom`) as the second; `EXPLORER` works as an env-var alternative:

```bash
bash start_exploration.sh                                # mock_hospital + explore_lite
bash start_exploration.sh office                         # office + explore_lite
bash start_exploration.sh mock_hospital custom           # mock_hospital + custom explorer
EXPLORER=custom bash start_exploration.sh office         # office + custom explorer
DEPTH_ANYTHING_ENABLED=true bash start_exploration.sh office
RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash start_exploration.sh office   # fall back to FastDDS
```

By default the script uses **CycloneDDS** (`cyclonedds.xml`: loopback interface, raised participant limit, large socket buffers) — more robust on this multi-NIC host than FastDDS shared memory, which gets stale locks. Run `tools/setup_dds.sh` **once** (sudo) to install the persistent large-message kernel tuning (`net.core.rmem_max` etc.); without it the script warns and big messages (camera/costmap) may drop. To fall back to FastDDS set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (defaults to the UDP-only profile) — see [ISSUES.md](ISSUES.md) for the rationale.

`build_and_start_expl.sh` rebuilds the workspace first, then runs the same exploration quick-start (extra args are forwarded to `start_exploration.sh`):

```bash
bash build_and_start_expl.sh
bash build_and_start_expl.sh office
bash build_and_start_expl.sh office custom
```

### `g1_distance_benchmark.launch.py`

```bash
# Default run: compare all available estimators
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py

# RGB only
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=rgb

# RGB + point cloud
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=rgb,pointcloud

# LiDAR only
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=lidar

# Mixed camera + LiDAR comparison
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py estimators:=rgb,lidar
```

This launch composes the simulator, `g1_detector_node`, the camera measurement node and/or the LiDAR measurement node depending on `estimators`, and `g1_distance_benchmark_runner`.

Each benchmark run writes under `benchmark-results/<timestamp>/` by default:
- one trial-level CSV per selected estimator
- one `comparison_summary.csv`
- one shared collage image per included trial under `images/`

Pass `output_dir:=...` to write the timestamped run folder somewhere else.

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `namespace` | `r100_0001` | Namespace for simulation, detector, measurement node, and benchmark runner |
| `use_sim_time` | `true` | Use Gazebo `/clock` |
| `setup_path` | `~/clearpath/` | Directory containing `robot.yaml` and generated Clearpath files |
| `world` | `g1_distance_calibration` | Gazebo world used for the benchmark run |
| `estimators` | `rgb,sensor_depth,depth_anything,pointcloud,lidar` | Comma-separated estimator subset to compare in one run |
| `repeats` | `5` | Number of positive-trial repeats per spawn pose |
| `output_dir` | `<repo-root>/benchmark-results` | Root directory that will receive one timestamped subfolder per run |
| `settle_sec` | `2.0` | Delay after spawning the target before sampling |
| `capture_sec` | `10.0` | Sampling window length for collecting usable detections |
| `color_topic` | `sensors/camera_0/color/image` | RGB topic used by the detector, camera measurement node, and benchmark snapshots |
| `depth_topic` | `sensors/camera_0/depth/image` | Depth topic used by the camera measurement node and benchmark collages |
| `pointcloud_topic` | `sensors/camera_0/points` | Camera-aligned point cloud used by the camera measurement node |
| `scan_topic` | `sensors/lidar2d_0/scan` | LaserScan topic used by the LiDAR measurement node |
| `base_frame` | `<namespace>/robot/base_link` | Vehicle frame used for point-cloud and LiDAR projection |

Benchmark semantics:
- only positive spawned-target trials are kept
- only successful single-target detections are used
- a trial is included only if every selected estimator has a usable aligned event
- each estimator CSV stores one row per included trial, using the median estimate over that trial’s aligned usable detections
- the shared collage image for each trial is built from one representative aligned detection event that is closest to the per-trial medians across the selected estimators

### Perception interfaces

| Node | Output topic | Key params |
|------|--------------|------------|
| `g1_detector_node` | `detections/g1/raw` | `color_topic`, `detection_model`, `detection_threshold`, `detector_fps` |
| `g1_camera_measurement_node` | `measurements/g1/camera` | `camera_config_path`, `color_topic`, `depth_topic`, `pointcloud_topic`, `base_frame`, `enabled_estimators`, `depth_anything_enabled` |
| `g1_lidar_measurement_node` | `measurements/g1/lidar` | `camera_config_path`, `scan_topic`, `base_frame` |
| `g1_overlay_node` | OpenCV window only | `measurement_topic`, `color_topic`, `depth_topic`, `mono_depth_debug_topic` |
| `g1_distance_benchmark_runner` | per-estimator CSVs + summary CSV + trial collage images | `estimators`, `output_dir`, `camera_measurement_topic`, `lidar_measurement_topic`, `color_topic`, `depth_topic` |

The shared camera geometry lives in `config/camera_config.json`, and the measurement nodes use the `camera_config_path` parameter.

## Configuration

### Robot sensors (`clearpath/robot.yaml`)

- **Hokuyo UST-10LX**: Mounted at front of chassis, provides 2D laser scan for SLAM and costmaps
- **Intel RealSense D455**: Mounted on a riser bracket, provides RGB, aligned depth, and camera-aligned point cloud to the G1 perception stack

### Key parameters to tune

| File | Parameter | Effect |
|------|-----------|--------|
| `config/explore_lite_params.yaml` | `min_frontier_size` | Minimum frontier size (m) to consider — increase to skip small gaps |
| `config/explore_lite_params.yaml` | `planner_frequency` | How often (Hz) to re-evaluate frontiers; lower values reduce goal preemption churn |
| `config/frontier_explorer_params.yaml` | `min_frontier_size` / `near_frontier_radius` / `goal_advance_cells` | Custom-explorer frontier clustering, near-tier preference, and goal placement past the centroid |
| `config/frontier_explorer_params.yaml` | `distance_weight` / `size_weight` | Scoring trade-off between how close vs. how large a far-tier frontier is |
| `config/frontier_explorer_params.yaml` | `goal_cost_threshold` / `goal_safety_margin` / `lethal_cost_threshold` | Goal-safety filtering against the costmap |
| `config/nav2_params.yaml` | `vx_max` / `vy_max` / `wz_max` | Robot linear and turn-rate limits |
| `config/nav2_params.yaml` | `max_accel` / `max_decel` | Velocity smoother acceleration and braking limits |
| `config/slam_toolbox_params.yaml` | `resolution` | Map resolution (m/pixel) |

## Patches and Issue History

This project still relies on two local patches:

1. `patches/clearpath_gz_customizations.patch` patches `src/clearpath_simulator/clearpath_gz` to add this repo's Gazebo worlds/models to the simulator search path and to expose the custom `SpawnG1` Gazebo GUI plugin.
2. `patches/slam_toolbox_tf_namespace.patch` patches `src/slam_toolbox` so `slam_toolbox` respects namespaced TF remappings.

The deeper root-cause notes, previous middleware workarounds, namespace gotchas, and troubleshooting tips now live in [ISSUES.md](ISSUES.md).
