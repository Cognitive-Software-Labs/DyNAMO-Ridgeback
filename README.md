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
| Detection | g1_detection_node (OWLv2) | VLM-based humanoid robot detection + geometry/depth distance estimation |

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

### Full autonomous exploration (recommended)

Start every run from a sourced workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/DyNAMO-Ridgeback
source install/setup.bash
```

### Main scenarios

```bash
# Always clean up stale processes first
bash cleanup.sh

# Main hospital scenario
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital

# Extended SLAM / exploration test
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse
```

The integrated launch does all of this for you:
1. Starts Gazebo and spawns the Ridgeback.
2. Launches the exploration RViz config.
3. Starts `slam_toolbox` after 20 seconds.
4. Starts Nav2 after 30 seconds.
5. Starts `explore_lite` after 45 seconds.

By default, RViz shows the exploration view plus the camera feeds under the `Cameras` group. When `g1_detection:=true`, `g1_detection_node` opens the combined RGB/depth detection window and can optionally add the Depth-Anything comparison pane.

### Useful launch toggles

```bash
# Disable the custom RViz instance
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse exploration_rviz:=false

# Enable VLM-based humanoid robot detection
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital g1_detection:=true

# Optional: compare live sensor depth with Depth-Anything metric depth
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital g1_detection:=true depth_anything_enabled:=true
```

### VLM detection setup (first time only)

The `g1_detection_node` uses a local OWLv2 detector and the live depth stream to produce a side-by-side detection overlay. It publishes the original geometry-based `distance` estimate plus a sensor-depth `depth_distance`, and it can optionally compare against a metric Depth-Anything branch in a 3-pane RGB / sensor depth / mono depth view. It requires a Python venv with PyTorch and Transformers:

```bash
# Create venv that can see ROS 2 packages
python3 -m venv --system-site-packages perception_venv
echo "/opt/ros/jazzy/lib/python3.12/site-packages" > perception_venv/lib/python3.12/site-packages/ros2.pth

# Install VLM dependencies
perception_venv/bin/python3 -m pip install torch torchvision transformers accelerate Pillow
```

The node's shebang points to `perception_venv/bin/python3` directly. The first launch will download the OWLv2 detector from Hugging Face. If you enable `depth_anything_enabled:=true`, the first run will also download the Depth-Anything V2 metric checkpoint into the local Hugging Face cache.

### Simulation only

```bash
ros2 launch ridgeback_slam_exploration simulation.launch.py world:=hospital
ros2 launch ridgeback_slam_exploration simulation.launch.py world:=warehouse
```

The per-component launch files still exist for debugging and development, but the intended workflow for normal use is the integrated `full_exploration.launch.py` entrypoint.

## Configuration

### Robot sensors (`clearpath/robot.yaml`)

- **Hokuyo UST-10LX**: Mounted at front of chassis, provides 2D laser scan for SLAM and costmaps
- **Intel RealSense D455**: Mounted on a riser bracket, provides depth + RGB to RViz and the `g1_detection_node` overlay

### Key parameters to tune

| File | Parameter | Effect |
|------|-----------|--------|
| `config/explore_lite_params.yaml` | `min_frontier_size` | Minimum frontier size (m) to consider — increase to skip small gaps |
| `config/explore_lite_params.yaml` | `planner_frequency` | How often (Hz) to re-evaluate frontiers |
| `config/nav2_params.yaml` | `vx_max` / `vy_max` | Robot speed limits |
| `config/slam_toolbox_params.yaml` | `resolution` | Map resolution (m/pixel) |

## Why slam_toolbox is built from source

This project requires a 1-line patch to slam_toolbox. Without it, SLAM completely fails in any namespaced Clearpath setup.

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
| Detection overlay does not appear | Make sure `g1_detection:=true` and check `/r100_0001/sensors/camera_0/color/image` |
| explore_lite not finding frontiers | Verify `track_unknown_space: true` in global costmap config |
| TF errors | Ensure all nodes use `use_sim_time: true` |
| Gazebo slow to start | Increase `TimerAction` delays in `full_exploration.launch.py` |
| Stale processes from previous runs | Run `bash cleanup.sh` before each launch |
| Diagnostics | Run `bash diag.sh /tmp/logfile.log hospital` or `bash diag.sh /tmp/logfile.log warehouse` |
