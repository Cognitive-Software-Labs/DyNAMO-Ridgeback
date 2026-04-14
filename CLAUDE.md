# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Jazzy project for autonomous SLAM-based frontier exploration of a Clearpath Ridgeback (R100) robot in Gazebo Harmonic. The robot explores a static environment with no prior map using slam_toolbox + Nav2. Two exploration backends are available: **explore_lite** (default) and a **custom frontier explorer**.

## Build & Source

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/DyNAMO-Ridgeback

# Clone external deps (first time only)
vcs import < .repos

# Apply patches (required for namespace TF and custom GUI)
cd src/slam_toolbox && git apply ../../patches/slam_toolbox_tf_namespace.patch && cd ../..
cd src/clearpath_simulator && git apply ../../patches/clearpath_gz_customizations.patch && cd ../..

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

# Main hospital scenario (uses explore_lite by default)
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital

# Use custom frontier explorer instead of explore_lite
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital explorer:=custom

# Headless (no Gazebo GUI, no camera windows) — saves CPU on slower machines
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital explorer:=custom gz_gui:=false camera_windows:=false

# Disable only the Gazebo GUI window (simulation still runs headless)
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=hospital gz_gui:=false

# Extended SLAM / exploration test
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse

# Warehouse with custom explorer
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse explorer:=custom

# Disable the exploration RViz instance
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse exploration_rviz:=false

# Disable the OpenCV camera viewer windows
ros2 launch ridgeback_slam_exploration full_exploration.launch.py world:=warehouse camera_windows:=false

# Individual components (run in separate terminals)
ros2 launch ridgeback_slam_exploration simulation.launch.py
ros2 launch ridgeback_slam_exploration slam.launch.py
ros2 launch ridgeback_slam_exploration nav2.launch.py
ros2 launch ridgeback_slam_exploration explore.launch.py                    # explore_lite
ros2 launch ridgeback_slam_exploration explore.launch.py explorer:=custom   # custom frontier explorer
```

## Explorer Selection

The `explorer` launch parameter controls which frontier exploration backend is used:

| Value | Backend | Description |
|-------|---------|-------------|
| `explore_lite` (default) | m-explore-ros2 | Mature ROS 2 frontier explorer |
| `custom` | frontier_explorer_node.py | Custom Python frontier explorer with flood-fill clustering, A* pathfinding, and configurable scoring |

Both explorers read the Nav2 global costmap and send goals via the NavigateToPose action to Nav2.

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
                        → Nav2 costmaps → [explore_lite OR custom frontier_explorer] → NavigateToPose → Nav2 → cmd_vel → Gazebo
```

- **slam_toolbox**: Online async mode, publishes map→odom TF. Built from source with a 1-line patch (see below).
- **Nav2**: MPPI controller with `motion_model: "Omni"` (Ridgeback is omnidirectional)
- **explore_lite** (default): Reads Nav2 global costmap, sends frontier goals via NavigateToPose action
- **custom frontier_explorer**: Custom Python node with flood-fill frontier clustering, A* pathfinding, configurable distance/size scoring
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
- `src/ridgeback_slam_exploration/config/explore_lite_params.yaml` — explore_lite params. Uses `/**/` YAML prefix for namespace compatibility.
- `src/ridgeback_slam_exploration/config/frontier_explorer_params.yaml` — Custom frontier explorer params.

## Custom Frontier Explorer

**Location:** `src/ridgeback_slam_exploration/frontier_explorer/`

**Key Components:**
- `frontier_explorer_node.py` — ROS 2 node that orchestrates exploration (installed as executable)
- `navigator.py` — Frontier detection and clustering using flood-fill algorithm
- `path_finding.py` — A* pathfinding for frontier navigation
- `params.py` — Constants and configuration (UNKNOWN=-1, FREE=0, OBSTACLE=1)

**Algorithm Overview:**
1. **Costmap Subscription:** Receives Nav2 global costmap, vectorized conversion to awareness map
2. **Frontier Detection:** Identifies FREE cells adjacent to UNKNOWN cells
3. **Frontier Clustering:** Groups nearby frontier points using flood-fill (8-way connectivity)
4. **Frontier Scoring:** Ranks clusters by `distance_weight × (1 - normalized_dist) + size_weight × normalized_size`
5. **Safety Filtering:** Rejects goals too close to obstacles (configurable `lethal_cost_threshold` and `goal_safety_margin`)
6. **Goal Sending:** Sends best safe frontier centroid as NavigateToPose goal to Nav2
7. **Progress Monitoring:** Times out after `progress_timeout` seconds, selects new frontier

**Configuration:** `config/frontier_explorer_params.yaml`
- `min_frontier_size: 4` — Minimum frontier cluster size
- `distance_weight: 0.7` — Distance priority in frontier selection (higher = prefer closer)
- `size_weight: 0.3` — Cluster size priority in frontier selection
- `lethal_cost_threshold: 90` — OccupancyGrid cost (0-100) above which cells are considered obstacles
- `goal_safety_margin: 3` — Cells around goal that must be clear of lethal obstacles
- `planner_frequency: 2.0 Hz` — Exploration loop rate
- `progress_timeout: 20.0 s` — Goal timeout before selecting new frontier

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
