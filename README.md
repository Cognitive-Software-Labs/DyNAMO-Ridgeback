# Ridgeback Exploration, Perception, and G1 Benchmarking

Autonomous exploration, G1 perception, and distance benchmarking for a [Clearpath Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/) robot in Gazebo Harmonic with ROS 2 Jazzy.

This workspace supports 3 main human workflows:
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
| Exploration | explore_lite (m-explore-ros2) | Frontier detection + goal selection |
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
colcon build --symlink-install
source install/setup.bash
```

If you rename or move the workspace directory later, wipe `build/`, `install/`, and `log/` before rebuilding so the generated setup files do not keep stale absolute paths.

### 3. Set up G1 perception venv (first time only)

The G1 perception stack now follows a staged pipeline inside the installable Python package `ridgeback_autonomy/`:
- raw detections on `detections/g1/raw`
- camera measurements on `measurements/g1/camera`
- lidar measurements on `measurements/g1/lidar`
- mono-depth debug images on `debug/g1/camera/mono_depth`

It requires a Python venv with PyTorch and Transformers:

```bash
# Create venv that can see ROS 2 packages
python3 -m venv --system-site-packages perception_venv
echo "/opt/ros/jazzy/lib/python3.12/site-packages" > perception_venv/lib/python3.12/site-packages/ros2.pth

# Install perception dependencies
perception_venv/bin/python3 -m pip install torch torchvision transformers accelerate Pillow
```

Both public launch files automatically prepend `perception_venv/bin` to `PATH` and set `VIRTUAL_ENV` for the perception nodes. The first run will download the OWLv2 detector from Hugging Face. If you enable `depth_anything_enabled:=true`, the first run will also download the Depth-Anything V2 metric checkpoint.

### 4. Set up robot config

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
3. `slam_toolbox` (after 20 s)
4. Nav2 (after 30 s)
5. `explore_lite` (after 45 s)
6. G1 perception nodes: `g1_detector_node`, `g1_camera_measurement_node`, `g1_lidar_measurement_node`, `g1_overlay_node`

The perception overlay appears in a separate OpenCV window named `G1 Perception`; it is not embedded in RViz.

Available worlds:

| World | Source | Notes |
|-------|--------|-------|
| `warehouse` | Clearpath | Default; large open floor plan |
| `office` | Clearpath | Smaller rooms and corridors |
| `hospital` | Custom (`sim/worlds/`) | Multi-room clinical layout |
| `construction` | Clearpath | Outdoor construction site |
| `orchard` | Clearpath | Outdoor orchard rows |
| `solar_farm` | Clearpath | Outdoor solar panel array |
| `pipeline` | Clearpath | Outdoor pipeline facility |

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `world` | `warehouse` | Gazebo world to load (see table above) |
| `namespace` | `r100_0001` | ROS namespace for all nodes |
| `use_sim_time` | `true` | Use Gazebo `/clock` |
| `setup_path` | `~/clearpath/` | Directory containing `robot.yaml` and generated Clearpath files |
| `exploration_rviz` | `true` | Launch the custom exploration RViz config |
| `g1_perception_enabled` | `true` | Launch the G1 perception stack |
| `depth_anything_enabled` | `false` | Enable Depth-Anything in the camera measurement node |

Examples:

```bash
# Always clean up stale processes first
bash cleanup.sh

ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=hospital
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=warehouse exploration_rviz:=false
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=office depth_anything_enabled:=true
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=hospital g1_perception_enabled:=false
```

#### Quick-start script

`start_exploration.sh` sources the workspace, runs cleanup, and launches with Depth-Anything enabled:

```bash
bash start_exploration.sh              # defaults to warehouse
bash start_exploration.sh office       # any world name as the first arg
bash start_exploration.sh hospital
```

### `g1_distance_benchmark.launch.py`

```bash
# Camera-stack benchmark (RGB / sensor depth / mono depth / pointcloud)
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py measurement_backend:=camera

# LiDAR benchmark
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py measurement_backend:=lidar

# Camera backend scored on pointcloud instead of RGB depth
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py measurement_backend:=camera primary_metric:=pointcloud
```

This launch composes the simulator, `g1_detector_node`, one selected measurement node, and `g1_distance_benchmark_runner`.

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `namespace` | `r100_0001` | Namespace for simulation, detector, measurement node, and benchmark runner |
| `use_sim_time` | `true` | Use Gazebo `/clock` |
| `setup_path` | `~/clearpath/` | Directory containing `robot.yaml` and generated Clearpath files |
| `world` | `g1_distance_calibration` | Gazebo world used for the benchmark run |
| `measurement_backend` | `camera` | Measurement node to launch: `camera` or `lidar` |
| `primary_metric` | `auto` | Metric column to score; defaults from the selected backend |
| `repeats` | `5` | Number of positive-trial repeats per spawn pose |
| `output_csv` | empty | Optional CSV path override; defaults from the selected backend |
| `settle_sec` | `2.0` | Delay after spawning the target before sampling |
| `capture_sec` | `3.0` | Sampling window length |
| `depth_anything_enabled` | `false` | Enable the optional mono-depth branch when `measurement_backend:=camera` |
| `color_topic` | `sensors/camera_0/color/image` | RGB topic used by the detector, camera measurement node, and benchmark snapshots |
| `depth_topic` | `sensors/camera_0/depth/image` | Depth topic used by the camera measurement node |
| `pointcloud_topic` | `sensors/camera_0/points` | Camera-aligned point cloud used by the camera measurement node |
| `scan_topic` | `sensors/lidar2d_0/scan` | LaserScan topic used by the LiDAR measurement node |
| `base_frame` | `<namespace>/robot/base_link` | Vehicle frame used for point-cloud and LiDAR projection |
| `failed_frame_dir` | empty | Optional override for saved failed-frame directory |
| `save_failed_frames` | `true` | Save annotated RGB frames for missed/ambiguous positive trials |

### Perception interfaces

| Node | Output topic | Key params |
|------|--------------|------------|
| `g1_detector_node` | `detections/g1/raw` | `color_topic`, `detection_model`, `detection_threshold` |
| `g1_camera_measurement_node` | `measurements/g1/camera` | `camera_config_path`, `color_topic`, `depth_topic`, `pointcloud_topic`, `base_frame`, `depth_anything_enabled` |
| `g1_lidar_measurement_node` | `measurements/g1/lidar` | `camera_config_path`, `scan_topic`, `base_frame` |
| `g1_overlay_node` | OpenCV window only | `measurement_topic`, `color_topic`, `depth_topic`, `mono_depth_debug_topic` |
| `g1_distance_benchmark_runner` | CSV + optional failed frames | `measurement_topic`, `primary_metric`, `color_topic`, `output_csv` |

The shared camera geometry lives in `config/camera_config.json`, and the measurement nodes use the `camera_config_path` parameter.

## Configuration

### Robot sensors (`clearpath/robot.yaml`)

- **Hokuyo UST-10LX**: Mounted at front of chassis, provides 2D laser scan for SLAM and costmaps
- **Intel RealSense D455**: Mounted on a riser bracket, provides RGB, aligned depth, and camera-aligned point cloud to the G1 perception stack

### Key parameters to tune

| File | Parameter | Effect |
|------|-----------|--------|
| `config/explore_lite_params.yaml` | `min_frontier_size` | Minimum frontier size (m) to consider — increase to skip small gaps |
| `config/explore_lite_params.yaml` | `planner_frequency` | How often (Hz) to re-evaluate frontiers |
| `config/nav2_params.yaml` | `vx_max` / `vy_max` | Robot speed limits |
| `config/slam_toolbox_params.yaml` | `resolution` | Map resolution (m/pixel) |

## Patches and Issue History

This project still relies on two local patches:

1. `patches/clearpath_gz_customizations.patch` patches `src/clearpath_simulator/clearpath_gz` to add this repo's Gazebo worlds/models to the simulator search path and to expose the custom `SpawnG1` Gazebo GUI plugin.
2. `patches/slam_toolbox_tf_namespace.patch` patches `src/slam_toolbox` so `slam_toolbox` respects namespaced TF remappings.

The deeper root-cause notes, previous middleware workarounds, namespace gotchas, and troubleshooting tips now live in [ISSUES.md](ISSUES.md).
