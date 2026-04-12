# Ridgeback SLAM & Frontier Exploration

Autonomous frontier exploration for a [Clearpath Ridgeback](https://clearpathrobotics.com/ridgeback-indoor-robot-platform/) robot in Gazebo Harmonic with ROS 2 Jazzy.

The integrated launch path brings up simulation, SLAM, Nav2, frontier exploration, a custom RViz layout, and camera preview windows. The main scenario in this workspace is `hospital`, while `warehouse` is used as an extended SLAM and exploration test.

## Stack

| Layer | Package | Purpose |
|-------|---------|---------|
| Simulation | Gazebo Harmonic + clearpath_simulator | Physics, sensors, world |
| Perception | Hokuyo UST-10LX 2D lidar | Obstacle detection + SLAM input |
| Perception | Intel RealSense D455 (~1m height) | Depth/RGB for overlay and distance estimation |
| SLAM | slam_toolbox (online async, from source) | Map building + localization |
| Navigation | Nav2 (MPPI omni controller) | Path planning + obstacle avoidance |
| Exploration | explore_lite (m-explore-ros2) | Frontier detection + goal selection |
| Detection | Staged G1 perception pipeline | Raw OWLv2 detections plus camera/LiDAR measurement nodes and an optional overlay |

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

### 3. Set up robot config

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

The lower-level simulation, SLAM, Nav2, and frontier-exploration launches live under `launch/includes/` and are treated as internal building blocks.

### `ridgeback_exploration.launch.py`

```bash
# Always clean up stale processes first
bash cleanup.sh

# Main hospital scenario
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=hospital

# Extended SLAM / exploration test
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=warehouse
```

This integrated launch:
1. Starts Gazebo and spawns the Ridgeback.
2. Launches the exploration RViz config.
3. Starts `slam_toolbox` after 20 seconds.
4. Starts Nav2 after 30 seconds.
5. Starts `explore_lite` after 45 seconds.
6. Optionally starts the G1 perception stack:
   `g1_detector_node`, `g1_camera_measurement_node`, and `g1_overlay_node`

Useful toggles:

```bash
# Disable the custom RViz instance
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=warehouse exploration_rviz:=false

# Enable the staged G1 perception stack
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=hospital g1_perception_enabled:=true

# Optional: enable the Depth-Anything branch inside the camera measurement node
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py world:=hospital g1_perception_enabled:=true depth_anything_enabled:=true
```

Arguments:

| Argument | Default | Meaning |
|----------|---------|---------|
| `namespace` | `r100_0001` | ROS namespace for RViz, Nav2, explore_lite, and optional perception nodes |
| `use_sim_time` | `true` | Use Gazebo `/clock` |
| `setup_path` | `~/clearpath/` | Directory containing `robot.yaml` and generated Clearpath files |
| `world` | `warehouse` | Gazebo world to load |
| `exploration_rviz` | `true` | Launch the custom exploration RViz config |
| `g1_perception_enabled` | `false` | Launch the detector, camera measurement, and overlay nodes |
| `depth_anything_enabled` | `false` | Enable Depth-Anything in the camera measurement node |

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

### G1 perception setup (first time only)

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

## Why slam_toolbox is built from source

This project requires two local patches:

1. `patches/clearpath_gz_customizations.patch` patches `src/clearpath_simulator/clearpath_gz` to add this repo's Gazebo worlds/models to the simulator search path and to expose the custom `SpawnG1` Gazebo GUI plugin.
2. `patches/slam_toolbox_tf_namespace.patch` patches `src/slam_toolbox` so `slam_toolbox` respects namespaced TF remappings.

Without the `slam_toolbox` patch, SLAM completely fails in a namespaced Clearpath setup.

### The problem

Clearpath robots require a non-empty ROS 2 namespace (e.g. `/r100_0001/`). All topics — including TF — live under that namespace: `/r100_0001/tf`, `/r100_0001/tf_static`. The standard way to handle this in ROS 2 is to remap absolute topic names at the node level:

```python
remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
```

This works perfectly for Nav2, explore_lite, and every other node in the stack. But it doesn't work for slam_toolbox.

### Root cause

In `slam_toolbox_common.cpp` (line 123), the `TransformListener` is created with a simple constructor:

```cpp
tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
```

This constructor only takes a `BufferCore` reference. Internally, `tf2_ros::TransformListener` creates a **hidden internal node** and subscribes to the hardcoded absolute topics `/tf` and `/tf_static` (`transform_listener.hpp` lines 191-198). Since this internal node is separate from slam_toolbox's own node, it ignores all launch-level remappings.

The result: slam_toolbox subscribes to the global `/tf` (which is empty) instead of `/r100_0001/tf` (where the robot publishes). SLAM never receives transforms, its `MessageFilter` drops every single laser scan with "queue is full" warnings, and no map is ever produced.

Ironically, the `TransformBroadcaster` on the very next line already does it correctly:

```cpp
tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(shared_from_this());
```

### The fix

The patch (`patches/slam_toolbox_tf_namespace.patch`) changes one line:

```cpp
// Before:
tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
// After:
tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_, shared_from_this());
```

Passing `shared_from_this()` makes the `TransformListener` use slam_toolbox's own node interface for subscriptions. This means it respects the `/tf` -> `tf` remapping, subscribing to `/r100_0001/tf` as intended. This is the same pattern Nav2 uses for all its nodes.

### What we tried before finding this

1. **tf_relay node** — A Python node that forwarded messages between `/r100_0001/tf` and `/tf`. This caused infinite relay loops (messages echoed back endlessly) and even when the loops were fixed by filtering on `frame_id`, slam_toolbox's `MessageFilter` still dropped all scans.

2. **Process-level remapping** — Launching slam_toolbox via `ExecuteProcess` with `-r /tf:=/r100_0001/tf`. TF worked (configure was instant), but `MessageFilter` still dropped scans after the first one.

3. **Queue size increases** — Bumping `scan_queue_size` from 1 to 10. Didn't help with the fundamental subscription problem.

The 1-line source patch was the only approach that actually resolved the issue. Zero `MessageFilter` drops, instant lifecycle configure, map publishing immediately.

### Namespace gotchas (general)

If you add new nodes to this project, always:

1. Set `namespace=namespace` on the node
2. Add `remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]`
3. Use `/**/node_name:` as the YAML root key in parameter files (the `/**/` glob matches any namespace prefix — without it, a namespaced node like `/r100_0001/explore_node` won't find its params under a bare `explore_node:` key)
4. Set `use_sim_time: true` in simulation

## Troubleshooting

| Problem | Check |
|---------|-------|
| Robot doesn't move | `ros2 topic echo /r100_0001/cmd_vel` — if empty, Nav2 may not be active |
| No map in RViz | `ros2 topic hz /r100_0001/map` — if 0, check slam_toolbox logs and scan topic |
| Detection overlay does not appear | Make sure `g1_perception_enabled:=true` and check `/r100_0001/sensors/camera_0/color/image` |
| explore_lite not finding frontiers | Verify `track_unknown_space: true` in global costmap config |
| TF errors | Ensure all nodes use `use_sim_time: true` |
| Gazebo slow to start | Increase `TimerAction` delays in `ridgeback_exploration.launch.py` |
| Stale processes from previous runs | Run `bash cleanup.sh` before each launch |
| Diagnostics | Run `bash diag.sh /tmp/logfile.log hospital` or `bash diag.sh /tmp/logfile.log warehouse` |
