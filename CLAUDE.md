# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Jazzy project for autonomous SLAM-based frontier exploration of a Clearpath Ridgeback (R100) robot in Gazebo Harmonic. The robot explores a static environment with no prior map using slam_toolbox + explore_lite + Nav2.

## Build & Source

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/DyNAMO-Ridgeback

# Clone external deps (first time only)
vcs import < .repos

# Apply slam_toolbox patch (required for namespace TF fix)
cd src/slam_toolbox && git apply ../../patches/slam_toolbox_tf_namespace.patch && cd ../..

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build (slam_toolbox is built from source with a patch)
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Launch Commands

```bash
# Always clean up stale processes first
bash cleanup.sh

# Main hospital scenario
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital

# Extended SLAM / exploration test
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse

# Disable the exploration RViz instance
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse exploration_rviz:=false

# Disable the OpenCV camera viewer windows
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse camera_windows:=false

# Individual components (run in separate terminals)
ros2 launch ridgeback_slam_exploration simulation.launch.py
ros2 launch ridgeback_slam_exploration slam.launch.py
ros2 launch ridgeback_slam_exploration nav2.launch.py
ros2 launch ridgeback_slam_exploration explore.launch.py
```

## Namespace Convention

All topics are prefixed with `/r100_0001/` (set in `clearpath/robot.yaml` under `system.ros2.namespace`). Clearpath requires a non-empty namespace. Examples:
- `/r100_0001/sensors/lidar2d_0/scan` — Hokuyo lidar
- `/r100_0001/sensors/camera_0/...` — RealSense D455
- `/r100_0001/platform/odom/filtered` — odometry
- `/r100_0001/map` — SLAM map
- `/r100_0001/cmd_vel` — velocity commands to robot

All nodes remap `/tf` → `tf` and `/tf_static` → `tf_static` so TF stays within the namespace. When adding new nodes, always include these remappings and set the namespace.

## Architecture & Topic Flow

```
Gazebo → lidar2d_0/scan → slam_toolbox → /r100_0001/map
                        → Nav2 costmaps → explore_lite → NavigateToPose → Nav2 → cmd_vel → Gazebo
```

- **slam_toolbox**: Online async mode, publishes map→odom TF. Built from source with a 1-line patch (see below).
- **Nav2**: MPPI controller with `motion_model: "Omni"` (Ridgeback is omnidirectional)
- **explore_lite**: Reads Nav2 global costmap, sends frontier goals via NavigateToPose action
- **Global costmap**: `track_unknown_space: true` and planner `allow_unknown: true` — both required for frontier exploration

## slam_toolbox Patch

slam_toolbox is cloned from source (via `.repos`) with a 1-line patch in `src/slam_toolbox/src/slam_toolbox_common.cpp` line 123:

```cpp
// Changed from: tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
// Changed to:
tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_, shared_from_this());
```

**Why:** The original simple constructor creates an internal node that ignores launch-level `/tf` remappings. With Clearpath's namespace, this means slam_toolbox subscribes to empty global `/tf` instead of namespaced `/r100_0001/tf`. Passing `shared_from_this()` makes the TransformListener use the slam_toolbox node's topic interface, respecting remappings — the same pattern Nav2 and the TransformBroadcaster (next line) already use.

## Key Config Files

- `clearpath/robot.yaml` — Robot platform, sensors, namespace. Must also exist at `~/clearpath/robot.yaml` for the simulator.
- `src/ridgeback_slam_exploration/config/nav2_params.yaml` — Nav2 stack params. Footprint matches Ridgeback dimensions.
- `src/ridgeback_slam_exploration/config/slam_toolbox_params.yaml` — SLAM params. `scan_topic` uses absolute path `/r100_0001/sensors/lidar2d_0/scan`.
- `src/ridgeback_slam_exploration/config/explore_lite_params.yaml` — Frontier exploration params. Uses `/**/` YAML prefix for namespace compatibility.

## Clearpath Sensor Naming

Sensors in `robot.yaml` are auto-indexed: first `lidar2d` → `lidar2d_0`, first `camera` → `camera_0`. Reordering sensors in the YAML changes topic names.

## External Dependencies

Managed via `.repos` file (not committed to git, listed in `.gitignore`):
- clearpath_simulator, clearpath_common, clearpath_config, clearpath_msgs, clearpath_nav2_demos (jazzy branches)
- m-explore-ros2 (main branch) — frontier exploration
- slam_toolbox (jazzy branch) — built from source with namespace patch

## Important Notes

- The primary scenario is `hospital`; `warehouse` is used as a larger extended SLAM and exploration test.
- All nodes must use `use_sim_time: true` in simulation — a single wall-clock node breaks TF.
- The integrated `full_exploration.launch.py` also launches the custom RViz config and the `camera_windows_node` viewer by default.
- The `full_exploration.launch.py` uses `TimerAction` delays (20s/30s/45s from startup) to sequence SLAM, Nav2, and exploration. If Gazebo is slow to load, increase these.
- Robot config must be symlinked/copied to `~/clearpath/` for the Clearpath simulator default path.
- **Always run `bash cleanup.sh` before launching** — Gazebo and ROS 2 processes leak across launches, causing topic conflicts, stale TF, and duplicate publishers.
- The `diag.sh` script provides comprehensive diagnostics: `bash diag.sh [logfile] [world]`
