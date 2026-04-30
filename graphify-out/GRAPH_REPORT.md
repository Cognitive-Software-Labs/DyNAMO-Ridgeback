# Graph Report - /home/deivid/dev/DyNAMO-Ridgeback  (2026-04-30)

## Corpus Check
- Corpus is ~41,049 words - fits in a single context window. You may not need a graph.

## Summary
- 284 nodes · 543 edges · 15 communities detected
- Extraction: 72% EXTRACTED · 28% INFERRED · 0% AMBIGUOUS · INFERRED: 154 edges (avg confidence: 0.78)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 7|Community 7]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 17|Community 17]]

## God Nodes (most connected - your core abstractions)
1. `G1DistanceBenchmarkRunner` - 36 edges
2. `BenchmarkCollageRenderer` - 21 edges
3. `G1CameraMeasurementNode` - 17 edges
4. `G1OverlayNode` - 16 edges
5. `RgbdOverlayRenderer` - 15 edges
6. `G1LidarMeasurementNode` - 13 edges
7. `DetectionBatch` - 11 edges
8. `Detection` - 10 edges
9. `MeasurementEvent` - 10 edges
10. `G1DetectorNode` - 9 edges

## Surprising Connections (you probably didn't know these)
- `load_camera_config()` --calls--> `CameraConfig`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/common/camera_config.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `OwlV2Detector` --uses--> `Detection`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/perception/core/detection.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `parse_owl_detections()` --calls--> `Detection`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/perception/core/detection.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `RgbdOverlayRenderer` --uses--> `DetectionBatch`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/perception/core/rendering.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `OwlV2Detector` --uses--> `DetectionBatch`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/perception/core/detection.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py

## Communities

### Community 0 - "Community 0"
Cohesion: 0.09
Nodes (13): ensure_measurement_event(), measurement_message_key(), stamp_to_nanoseconds(), update_measurement_event(), G1DistanceBenchmarkRunner, main(), build_summary_rows(), write_summary_csv() (+5 more)

### Community 1 - "Community 1"
Cohesion: 0.13
Nodes (3): BenchmarkCollageRenderer, PanelContext, RgbdOverlayRenderer

### Community 2 - "Community 2"
Cohesion: 0.15
Nodes (23): build_float32_image_message(), LidarScanPoints, add_depth_measurements(), add_depth_source_measurements(), add_lidar_measurements(), add_pointcloud_measurements(), add_rgb_measurements(), apply_vehicle_front_offset() (+15 more)

### Community 3 - "Community 3"
Cohesion: 0.15
Nodes (21): extract_public_estimator_values(), batch_from_detections_message(), batch_from_measurements_message(), build_detections_message(), build_measurements_message(), decode_bbox_quads(), decode_optional_float(), first_finite_positive() (+13 more)

### Community 4 - "Community 4"
Cohesion: 0.18
Nodes (21): event_has_all_panel_previews(), event_has_panel_preview(), EventPreview, find_exact_preview_match(), find_nearest_preview_match(), has_all_selected_estimates(), MeasurementEvent, nearest_preview_metadata() (+13 more)

### Community 5 - "Community 5"
Cohesion: 0.12
Nodes (6): load_camera_config(), DepthAnythingEstimator, resolve_torch_device(), extract_organized_xyz(), G1CameraMeasurementNode, main()

### Community 6 - "Community 6"
Cohesion: 0.14
Nodes (10): compute_iou(), non_maximum_suppression(), OwlV2Detector, parse_owl_detections(), bgr_frame_to_pil(), G1DetectorNode, main(), test_compute_iou_handles_overlap() (+2 more)

### Community 7 - "Community 7"
Cohesion: 0.16
Nodes (10): parse_estimators(), selected_camera_estimators(), uses_camera_estimators(), uses_lidar_estimators(), generate_launch_description(), generate_launch_description(), build_benchmark_nodes(), generate_launch_description() (+2 more)

### Community 8 - "Community 8"
Cohesion: 0.18
Nodes (5): candidate_base_frames(), lookup_transform_components(), rotation_matrix_from_quaternion(), G1LidarMeasurementNode, main()

### Community 9 - "Community 9"
Cohesion: 0.26
Nodes (2): G1OverlayNode, main()

### Community 10 - "Community 10"
Cohesion: 0.3
Nodes (10): convert_color_image_message(), convert_depth_to_meters_message(), decode_buffer(), decode_image_message(), normalize_to_uint8(), make_image_msg(), test_convert_color_image_message_rgb8_to_bgr(), test_convert_color_image_message_supports_padded_rows() (+2 more)

### Community 14 - "Community 14"
Cohesion: 1.0
Nodes (1): Shared Python package for Ridgeback autonomy helpers and nodes.

### Community 15 - "Community 15"
Cohesion: 1.0
Nodes (1): Benchmarking nodes plus helper libraries for autonomy evaluation workflows.

### Community 16 - "Community 16"
Cohesion: 1.0
Nodes (1): Perception nodes plus helper libraries for detection, fusion, and visualization.

### Community 17 - "Community 17"
Cohesion: 1.0
Nodes (1): Shared perception internals used by the perception node entrypoints.

## Knowledge Gaps
- **4 isolated node(s):** `Shared Python package for Ridgeback autonomy helpers and nodes.`, `Benchmarking nodes plus helper libraries for autonomy evaluation workflows.`, `Perception nodes plus helper libraries for detection, fusion, and visualization.`, `Shared perception internals used by the perception node entrypoints.`
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Community 9`** (14 nodes): `G1OverlayNode`, `.color_callback()`, `.depth_callback()`, `.destroy_node()`, `.lidar_measurement_callback()`, `.log_warning_once()`, `.measurement_callback()`, `.measurement_message_key()`, `.merge_lidar_measurements()`, `.mono_depth_callback()`, `.render_callback()`, `.render_latest()`, `main()`, `g1_overlay_node.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 14`** (2 nodes): `Shared Python package for Ridgeback autonomy helpers and nodes.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 15`** (2 nodes): `Benchmarking nodes plus helper libraries for autonomy evaluation workflows.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 16`** (2 nodes): `Perception nodes plus helper libraries for detection, fusion, and visualization.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 17`** (2 nodes): `Shared perception internals used by the perception node entrypoints.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `G1DistanceBenchmarkRunner` connect `Community 0` to `Community 1`, `Community 7`?**
  _High betweenness centrality (0.283) - this node is a cross-community bridge._
- **Why does `BenchmarkCollageRenderer` connect `Community 1` to `Community 0`, `Community 4`, `Community 7`?**
  _High betweenness centrality (0.151) - this node is a cross-community bridge._
- **Why does `RgbdOverlayRenderer` connect `Community 1` to `Community 9`, `Community 3`?**
  _High betweenness centrality (0.132) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `BenchmarkCollageRenderer` (e.g. with `G1DistanceBenchmarkRunner` and `MeasurementEvent`) actually correct?**
  _`BenchmarkCollageRenderer` has 6 INFERRED edges - model-reasoned connections that need verification._
- **Are the 6 inferred relationships involving `RgbdOverlayRenderer` (e.g. with `PanelContext` and `BenchmarkCollageRenderer`) actually correct?**
  _`RgbdOverlayRenderer` has 6 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Shared Python package for Ridgeback autonomy helpers and nodes.`, `Benchmarking nodes plus helper libraries for autonomy evaluation workflows.`, `Perception nodes plus helper libraries for detection, fusion, and visualization.` to the rest of the system?**
  _4 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.09 - nodes in this community are weakly interconnected._