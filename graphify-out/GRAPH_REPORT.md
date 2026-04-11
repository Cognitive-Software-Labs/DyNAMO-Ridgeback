# Graph Report - .  (2026-04-11)

## Corpus Check
- 16 files · ~10,835 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 95 nodes · 107 edges · 18 communities detected
- Extraction: 81% EXTRACTED · 19% INFERRED · 0% AMBIGUOUS · INFERRED: 20 edges (avg confidence: 0.81)
- Token cost: 46,050 input · 17,584 output

## God Nodes (most connected - your core abstractions)
1. `full_exploration launch` - 9 edges
2. `g1_pointcloud_distance_benchmark launch` - 6 edges
3. `g1_lidar_distance_benchmark launch` - 6 edges
4. `g1_distance_benchmark (RGB) launch` - 6 edges
5. `g1_distance_benchmark_runner` - 6 edges
6. `Project README` - 6 edges
7. `g1_detection_node (RGB/OWLv2)` - 5 edges
8. `ridgeback_slam_exploration CMake Project` - 4 edges
9. `G1Detection.msg Custom Message` - 4 edges
10. `decode_image_message()` - 4 edges

## Surprising Connections (you probably didn't know these)
- `TimerAction staged launch sequencing` --rationale_for--> `full_exploration launch`  [EXTRACTED]
  CLAUDE.md → src/ridgeback_slam_exploration/launch/full_exploration.launch.py
- `CLAUDE.md project instructions` --references--> `full_exploration launch`  [EXTRACTED]
  CLAUDE.md → src/ridgeback_slam_exploration/launch/full_exploration.launch.py
- `VLM-based G1 detection + distance estimation` --references--> `g1_detection_node (RGB/OWLv2)`  [EXTRACTED]
  README.md → src/ridgeback_slam_exploration/launch/g1_distance_benchmark.launch.py
- `OWLv2 model constant (google/owlv2-base-patch16-ensemble)` --references--> `OWLv2 detector (Hugging Face)`  [EXTRACTED]
  src/ridgeback_slam_exploration/scripts/g1_detection_common.py → README.md
- `AGENTS.md graphify rules` --references--> `Project README`  [INFERRED]
  AGENTS.md → README.md

## Hyperedges (group relationships)
- **Three distance benchmark modalities (RGB/LiDAR/Pointcloud)** — launch_g1_distance_benchmark, launch_g1_lidar_benchmark, launch_g1_pointcloud_benchmark, node_g1_distance_benchmark_runner, g1_distance_calibration_world [EXTRACTED 0.95]
- **Staged TimerAction startup flow (sim->SLAM->Nav2->explore)** — launch_full_exploration, simulation_launch, slam_launch, nav2_launch, explore_launch [EXTRACTED 0.95]
- **OWLv2 detection decode/parse/NMS pipeline** — g1_detection_common_decode_image, g1_detection_common_convert_color, g1_detection_common_parse_owl_detections, g1_detection_common_nms, g1_detection_common_focus_bbox [INFERRED 0.85]

## Communities

### Community 0 - "OWLv2 Detection Utilities"
Cohesion: 0.16
Nodes (14): compute_iou(), convert_color_image_message, convert_color_image_message(), convert_depth_to_meters_message, convert_depth_to_meters_message(), decode_buffer(), decode_image_message, decode_image_message() (+6 more)

### Community 1 - "Launch Orchestration and Docs"
Cohesion: 0.17
Nodes (5): CLAUDE.md project instructions, TimerAction staged launch sequencing, fastrtps_no_shm.xml, full_exploration launch, perception_venv (local Python venv)

### Community 2 - "G1 Distance Benchmark Pipeline"
Cohesion: 0.38
Nodes (10): g1_detection_common utilities, g1_distance_calibration world, g1_distance_common utilities, g1_distance_benchmark (RGB) launch, g1_lidar_distance_benchmark launch, g1_pointcloud_distance_benchmark launch, g1_detection_node (RGB/OWLv2), g1_detection_lidar_node (+2 more)

### Community 3 - "Triple-View Depth Comparison (image)"
Cohesion: 0.24
Nodes (10): Depth-Anything Monocular Depth Model, Depth-Anything Panel, Distance Estimate Overlay (Geo/Sensor/Mono), g1_detection_common.py, G1 Detection Triple-View Comparison, g1_distance_common.py, G1 Humanoid Detection Target, RealSense D455 Depth Sensor (+2 more)

### Community 4 - "ROS Detection Nodes and Build"
Cohesion: 0.29
Nodes (8): camera_windows_node Script, g1_detection_node Script, G1Detection.msg Custom Message, ridgeback_slam_exploration CMake Project, SpawnG1 Gazebo GUI Plugin, gz-gui8 (Gazebo GUI), rosidl_default_generators, std_msgs

### Community 5 - "External ROS 2 Dependencies"
Cohesion: 0.29
Nodes (7): AGENTS.md graphify rules, clearpath_simulator, explore_lite (m-explore-ros2), Nav2 MPPI omni controller, Project README, Namespace gotchas guidance, slam_toolbox namespace patch rationale

### Community 6 - "Distance Math (AST view)"
Cohesion: 0.6
Nodes (3): apply_vehicle_front_offset(), planar_distance_from_vehicle_origin(), world_to_vehicle_planar()

### Community 7 - "SpawnG1 Gazebo Plugin"
Cohesion: 0.67
Nodes (0): 

### Community 8 - "Distance Math (semantic view)"
Cohesion: 0.5
Nodes (4): apply_vehicle_front_offset, ROBOT_FRONT_OFFSET_M constant, planar_distance_from_vehicle_origin, world_to_vehicle_planar

### Community 9 - "Vision Model Stack"
Cohesion: 0.5
Nodes (4): Depth-Anything V2 metric depth, OWLv2 model constant (google/owlv2-base-patch16-ensemble), OWLv2 detector (Hugging Face), VLM-based G1 detection + distance estimation

### Community 10 - "full_exploration Launch"
Cohesion: 1.0
Nodes (0): 

### Community 11 - "g1_pointcloud Benchmark Launch"
Cohesion: 1.0
Nodes (0): 

### Community 12 - "g1_lidar Benchmark Launch"
Cohesion: 1.0
Nodes (0): 

### Community 13 - "g1_distance Benchmark Launch"
Cohesion: 1.0
Nodes (0): 

### Community 14 - "bgr_frame_to_pil (orphan)"
Cohesion: 1.0
Nodes (1): bgr_frame_to_pil

### Community 15 - "yaw_from_quaternion (orphan)"
Cohesion: 1.0
Nodes (1): yaw_from_quaternion

### Community 16 - "G1Detection msg (orphan)"
Cohesion: 1.0
Nodes (1): G1Detection.msg interface

### Community 17 - "SpawnG1 plugin ref (orphan)"
Cohesion: 1.0
Nodes (1): SpawnG1 Gazebo GUI plugin

## Knowledge Gaps
- **28 isolated node(s):** `camera_windows_node Script`, `gz-gui8 (Gazebo GUI)`, `rosidl_default_generators`, `std_msgs`, `bgr_frame_to_pil` (+23 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `full_exploration Launch`** (2 nodes): `full_exploration.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `g1_pointcloud Benchmark Launch`** (2 nodes): `g1_pointcloud_distance_benchmark.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `g1_lidar Benchmark Launch`** (2 nodes): `g1_lidar_distance_benchmark.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `g1_distance Benchmark Launch`** (2 nodes): `g1_distance_benchmark.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `bgr_frame_to_pil (orphan)`** (1 nodes): `bgr_frame_to_pil`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `yaw_from_quaternion (orphan)`** (1 nodes): `yaw_from_quaternion`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `G1Detection msg (orphan)`** (1 nodes): `G1Detection.msg interface`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `SpawnG1 plugin ref (orphan)`** (1 nodes): `SpawnG1 Gazebo GUI plugin`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `g1_detection_node (RGB/OWLv2)` connect `G1 Distance Benchmark Pipeline` to `Launch Orchestration and Docs`, `Vision Model Stack`?**
  _High betweenness centrality (0.125) - this node is a cross-community bridge._
- **Why does `VLM-based G1 detection + distance estimation` connect `Vision Model Stack` to `G1 Distance Benchmark Pipeline`?**
  _High betweenness centrality (0.114) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `g1_pointcloud_distance_benchmark launch` (e.g. with `g1_lidar_distance_benchmark launch` and `g1_distance_benchmark (RGB) launch`) actually correct?**
  _`g1_pointcloud_distance_benchmark launch` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `g1_lidar_distance_benchmark launch` (e.g. with `g1_pointcloud_distance_benchmark launch` and `g1_distance_benchmark (RGB) launch`) actually correct?**
  _`g1_lidar_distance_benchmark launch` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `g1_distance_benchmark (RGB) launch` (e.g. with `g1_pointcloud_distance_benchmark launch` and `g1_lidar_distance_benchmark launch`) actually correct?**
  _`g1_distance_benchmark (RGB) launch` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `g1_distance_benchmark_runner` (e.g. with `g1_detection_pointcloud_node` and `g1_detection_lidar_node`) actually correct?**
  _`g1_distance_benchmark_runner` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `camera_windows_node Script`, `gz-gui8 (Gazebo GUI)`, `rosidl_default_generators` to the rest of the system?**
  _28 weakly-connected nodes found - possible documentation gaps or missing edges._