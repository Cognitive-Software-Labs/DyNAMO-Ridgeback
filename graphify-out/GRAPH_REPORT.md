# Graph Report - /home/deivid/dev/DyNAMO-Ridgeback  (2026-04-12)

## Corpus Check
- Corpus is ~14,546 words - fits in a single context window. You may not need a graph.

## Summary
- 215 nodes · 305 edges · 30 communities detected
- Extraction: 98% EXTRACTED · 2% INFERRED · 0% AMBIGUOUS · INFERRED: 6 edges (avg confidence: 0.5)
- Token cost: 0 input · 0 output

## God Nodes (most connected - your core abstractions)
1. `G1DistanceBenchmarkRunner` - 34 edges
2. `G1CameraMeasurementNode` - 17 edges
3. `G1OverlayNode` - 16 edges
4. `G1LidarMeasurementNode` - 13 edges
5. `RgbdOverlayRenderer` - 11 edges
6. `G1DetectorNode` - 9 edges
7. `OwlV2Detector` - 7 edges
8. `apply_vehicle_front_offset()` - 6 edges
9. `add_depth_source_measurements()` - 5 edges
10. `compute_lidar_measurement()` - 5 edges

## Surprising Connections (you probably didn't know these)
- `OwlV2Detector` --uses--> `Detection`  [INFERRED]
  /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/core/detection.py → /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `RgbdOverlayRenderer` --uses--> `DetectionBatch`  [INFERRED]
  /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/core/rendering.py → /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `G1OverlayNode` --uses--> `RgbdOverlayRenderer`  [INFERRED]
  /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_overlay_node.py → /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/core/rendering.py
- `G1DetectorNode` --uses--> `OwlV2Detector`  [INFERRED]
  /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_detector_node.py → /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/core/detection.py
- `G1CameraMeasurementNode` --uses--> `DepthAnythingEstimator`  [INFERRED]
  /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/g1_camera_measurement_node.py → /home/deivid/dev/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/core/depth_anything.py

## Communities

### Community 0 - "Community 0"
Cohesion: 0.12
Nodes (2): G1DistanceBenchmarkRunner, main()

### Community 1 - "Community 1"
Cohesion: 0.19
Nodes (16): add_depth_measurements(), add_depth_source_measurements(), add_lidar_measurements(), add_pointcloud_measurements(), add_rgb_measurements(), apply_vehicle_front_offset(), compute_camera_bearing_window(), compute_lidar_measurement() (+8 more)

### Community 2 - "Community 2"
Cohesion: 0.19
Nodes (2): G1CameraMeasurementNode, main()

### Community 3 - "Community 3"
Cohesion: 0.17
Nodes (9): compute_iou(), non_maximum_suppression(), OwlV2Detector, parse_owl_detections(), resolve_torch_device(), CameraConfig, Detection, DetectionBatch (+1 more)

### Community 4 - "Community 4"
Cohesion: 0.24
Nodes (2): G1OverlayNode, main()

### Community 5 - "Community 5"
Cohesion: 0.23
Nodes (2): G1LidarMeasurementNode, main()

### Community 6 - "Community 6"
Cohesion: 0.29
Nodes (10): batch_from_detections_message(), batch_from_measurements_message(), build_detections_message(), build_measurements_message(), decode_bbox_quads(), decode_optional_float(), first_finite_positive(), optional_float() (+2 more)

### Community 7 - "Community 7"
Cohesion: 0.33
Nodes (1): RgbdOverlayRenderer

### Community 8 - "Community 8"
Cohesion: 0.31
Nodes (3): G1DetectorNode, main(), Node

### Community 9 - "Community 9"
Cohesion: 0.25
Nodes (0): 

### Community 10 - "Community 10"
Cohesion: 0.48
Nodes (5): convert_color_image_message(), convert_depth_to_meters_message(), decode_buffer(), decode_image_message(), normalize_to_uint8()

### Community 11 - "Community 11"
Cohesion: 0.47
Nodes (3): make_image_msg(), test_convert_color_image_message_rgb8_to_bgr(), test_convert_depth_to_meters_from_16uc1()

### Community 12 - "Community 12"
Cohesion: 0.5
Nodes (1): DepthAnythingEstimator

### Community 13 - "Community 13"
Cohesion: 0.4
Nodes (0): 

### Community 14 - "Community 14"
Cohesion: 0.83
Nodes (3): candidate_base_frames(), lookup_transform_components(), rotation_matrix_from_quaternion()

### Community 15 - "Community 15"
Cohesion: 0.67
Nodes (0): 

### Community 16 - "Community 16"
Cohesion: 0.67
Nodes (0): 

### Community 17 - "Community 17"
Cohesion: 0.67
Nodes (0): 

### Community 18 - "Community 18"
Cohesion: 0.67
Nodes (0): 

### Community 19 - "Community 19"
Cohesion: 0.67
Nodes (0): 

### Community 20 - "Community 20"
Cohesion: 1.0
Nodes (0): 

### Community 21 - "Community 21"
Cohesion: 1.0
Nodes (1): Shared perception internals used by the perception node entrypoints.

### Community 22 - "Community 22"
Cohesion: 1.0
Nodes (0): 

### Community 23 - "Community 23"
Cohesion: 1.0
Nodes (0): 

### Community 24 - "Community 24"
Cohesion: 1.0
Nodes (0): 

### Community 25 - "Community 25"
Cohesion: 1.0
Nodes (0): 

### Community 26 - "Community 26"
Cohesion: 1.0
Nodes (0): 

### Community 27 - "Community 27"
Cohesion: 1.0
Nodes (0): 

### Community 28 - "Community 28"
Cohesion: 1.0
Nodes (0): 

### Community 29 - "Community 29"
Cohesion: 1.0
Nodes (0): 

## Knowledge Gaps
- **3 isolated node(s):** `Shared perception internals used by the perception node entrypoints.`, `CameraConfig`, `LidarScanPoints`
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Community 20`** (2 nodes): `rebuild_graphify.py`, `main()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 21`** (2 nodes): `__init__.py`, `Shared perception internals used by the perception node entrypoints.`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 22`** (2 nodes): `camera_config.py`, `load_camera_config()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 23`** (2 nodes): `test_imports.py`, `test_packaged_modules_import()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 24`** (2 nodes): `test_benchmark_runner.py`, `test_benchmark_runner_on_measurement_uses_new_message_fields()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 25`** (2 nodes): `ridgeback_exploration.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 26`** (2 nodes): `explore.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 27`** (2 nodes): `nav2.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 28`** (2 nodes): `simulation.launch.py`, `generate_launch_description()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 29`** (1 nodes): `metrics.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `G1DistanceBenchmarkRunner` connect `Community 0` to `Community 8`?**
  _High betweenness centrality (0.141) - this node is a cross-community bridge._
- **Why does `G1OverlayNode` connect `Community 4` to `Community 8`, `Community 7`?**
  _High betweenness centrality (0.105) - this node is a cross-community bridge._
- **Why does `G1CameraMeasurementNode` connect `Community 2` to `Community 8`, `Community 12`?**
  _High betweenness centrality (0.093) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `RgbdOverlayRenderer` (e.g. with `G1OverlayNode` and `DetectionBatch`) actually correct?**
  _`RgbdOverlayRenderer` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Shared perception internals used by the perception node entrypoints.`, `CameraConfig`, `LidarScanPoints` to the rest of the system?**
  _3 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.12 - nodes in this community are weakly interconnected._