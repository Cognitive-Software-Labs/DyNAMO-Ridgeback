# Graph Report - /home/deivid/dev/DyNAMO-Ridgeback  (2026-05-28)

## Corpus Check
- Corpus is ~37,138 words - fits in a single context window. You may not need a graph.

## Summary
- 434 nodes · 806 edges · 22 communities detected
- Extraction: 80% EXTRACTED · 20% INFERRED · 0% AMBIGUOUS · INFERRED: 160 edges (avg confidence: 0.78)
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
- [[_COMMUNITY_Community 11|Community 11]]
- [[_COMMUNITY_Community 12|Community 12]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 18|Community 18]]
- [[_COMMUNITY_Community 19|Community 19]]
- [[_COMMUNITY_Community 20|Community 20]]
- [[_COMMUNITY_Community 21|Community 21]]
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 27|Community 27]]

## God Nodes (most connected - your core abstractions)
1. `G1DistanceBenchmarkRunner` - 36 edges
2. `BenchmarkCollageRenderer` - 21 edges
3. `FrontierExplorerNode` - 20 edges
4. `onInitialize()` - 20 edges
5. `OverlayTextDisplay()` - 20 edges
6. `onInitialize()` - 20 edges
7. `PieChartDisplay()` - 19 edges
8. `G1OverlayNode` - 17 edges
9. `G1CameraMeasurementNode` - 17 edges
10. `RgbdOverlayRenderer` - 15 edges

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
Cohesion: 0.08
Nodes (44): batch_from_detections_message(), batch_from_measurements_message(), build_detections_message(), build_float32_image_message(), build_measurements_message(), decode_bbox_quads(), decode_optional_float(), first_finite_positive() (+36 more)

### Community 1 - "Community 1"
Cohesion: 0.06
Nodes (28): get_best_frontier(), get_frontier_clusters(), is_frontier_point(), Update robot awareness map from costmap data.     Used for ROS 2 integration., Checks if a point is a valid frontier (A FREE cell next to an UNKNOWN cell)., Uses Flood Fill to group adjacent frontier points into clusters., Selects frontier cluster based on both distance and size.          Args:, Sensor Model: Discovers the area around the robot based on its VIEW_RADIUS. (+20 more)

### Community 2 - "Community 2"
Cohesion: 0.12
Nodes (7): stamp_to_nanoseconds(), G1DistanceBenchmarkRunner, main(), build_summary_rows(), write_summary_csv(), write_trial_csv(), test_build_summary_rows_aggregates_trial_level_estimator_rows()

### Community 3 - "Community 3"
Cohesion: 0.12
Nodes (27): ensure_measurement_event(), event_has_all_panel_previews(), event_has_panel_preview(), EventPreview, extract_public_estimator_values(), find_exact_preview_match(), find_nearest_preview_match(), has_all_selected_estimates() (+19 more)

### Community 4 - "Community 4"
Cohesion: 0.12
Nodes (13): convert_color_image_message(), convert_depth_to_meters_message(), decode_buffer(), decode_image_message(), normalize_to_uint8(), G1OverlayNode, main(), stamp_seconds() (+5 more)

### Community 5 - "Community 5"
Cohesion: 0.17
Nodes (24): drawPlot(), onEnable(), onInitialize(), PieChartDisplay(), setPosition(), update(), updateAutoColorChange(), updateBGAlpha() (+16 more)

### Community 6 - "Community 6"
Cohesion: 0.13
Nodes (3): BenchmarkCollageRenderer, PanelContext, RgbdOverlayRenderer

### Community 7 - "Community 7"
Cohesion: 0.24
Nodes (24): onDisable(), onEnable(), onInitialize(), OverlayTextDisplay(), processMessage(), reset(), updateAlignBottom(), updateBGAlpha() (+16 more)

### Community 8 - "Community 8"
Cohesion: 0.11
Nodes (12): parse_estimators(), selected_camera_estimators(), uses_camera_estimators(), uses_lidar_estimators(), main(), VelocityOverlayNode, generate_launch_description(), generate_launch_description() (+4 more)

### Community 9 - "Community 9"
Cohesion: 0.12
Nodes (6): load_camera_config(), DepthAnythingEstimator, resolve_torch_device(), extract_organized_xyz(), G1CameraMeasurementNode, main()

### Community 10 - "Community 10"
Cohesion: 0.14
Nodes (10): compute_iou(), non_maximum_suppression(), OwlV2Detector, parse_owl_detections(), bgr_frame_to_pil(), G1DetectorNode, main(), test_compute_iou_handles_overlap() (+2 more)

### Community 11 - "Community 11"
Cohesion: 0.17
Nodes (6): candidate_base_frames(), lookup_transform_components(), rotation_matrix_from_quaternion(), extract_scan_points_base(), G1LidarMeasurementNode, main()

### Community 12 - "Community 12"
Cohesion: 0.21
Nodes (12): getBuffer(), getName(), getQImage(), getTextureHeight(), getTextureWidth(), hide(), isTextureReady(), isVisible() (+4 more)

### Community 14 - "Community 14"
Cohesion: 0.67
Nodes (3): a_star(), heuristic(), Optimized A* Algorithm to find the shortest path avoiding obstacles.

### Community 16 - "Community 16"
Cohesion: 0.5
Nodes (1): Rviz2dString

### Community 18 - "Community 18"
Cohesion: 0.67
Nodes (2): OverlayObject, ScopedPixelBuffer

### Community 19 - "Community 19"
Cohesion: 1.0
Nodes (1): Shared Python package for Ridgeback autonomy helpers and nodes.

### Community 20 - "Community 20"
Cohesion: 1.0
Nodes (1): Benchmarking nodes plus helper libraries for autonomy evaluation workflows.

### Community 21 - "Community 21"
Cohesion: 1.0
Nodes (1): Frontier exploration module for ROS 2.

### Community 22 - "Community 22"
Cohesion: 1.0
Nodes (1): Perception nodes plus helper libraries for detection, fusion, and visualization.

### Community 23 - "Community 23"
Cohesion: 1.0
Nodes (1): Shared perception internals used by the perception node entrypoints.

### Community 27 - "Community 27"
Cohesion: 1.0
Nodes (1): OverlayTextDisplay

## Knowledge Gaps
- **30 isolated node(s):** `Shared Python package for Ridgeback autonomy helpers and nodes.`, `Receive and process occupancy grid (costmap).`, `Convert occupancy grid to robot awareness map (vectorized).                  Nav`, `Get robot position in map frame.`, `Main exploration loop.` (+25 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **Thin community `Community 16`** (4 nodes): `string_to_overlay_text.cpp`, `main()`, `Rviz2dString`, `.Rviz2dString()`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 18`** (3 nodes): `OverlayObject`, `ScopedPixelBuffer`, `overlay_utils.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 19`** (2 nodes): `Shared Python package for Ridgeback autonomy helpers and nodes.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 20`** (2 nodes): `Benchmarking nodes plus helper libraries for autonomy evaluation workflows.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 21`** (2 nodes): `Frontier exploration module for ROS 2.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 22`** (2 nodes): `Perception nodes plus helper libraries for detection, fusion, and visualization.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 23`** (2 nodes): `Shared perception internals used by the perception node entrypoints.`, `__init__.py`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.
- **Thin community `Community 27`** (2 nodes): `OverlayTextDisplay`, `overlay_text_display.hpp`
  Too small to be a meaningful cluster - may be noise or needs more connections extracted.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `G1DistanceBenchmarkRunner` connect `Community 2` to `Community 8`, `Community 3`, `Community 6`?**
  _High betweenness centrality (0.178) - this node is a cross-community bridge._
- **Why does `FrontierExplorerNode` connect `Community 1` to `Community 8`?**
  _High betweenness centrality (0.139) - this node is a cross-community bridge._
- **Why does `G1CameraMeasurementNode` connect `Community 9` to `Community 8`, `Community 0`?**
  _High betweenness centrality (0.079) - this node is a cross-community bridge._
- **Are the 6 inferred relationships involving `BenchmarkCollageRenderer` (e.g. with `G1DistanceBenchmarkRunner` and `MeasurementEvent`) actually correct?**
  _`BenchmarkCollageRenderer` has 6 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Shared Python package for Ridgeback autonomy helpers and nodes.`, `Receive and process occupancy grid (costmap).`, `Convert occupancy grid to robot awareness map (vectorized).                  Nav` to the rest of the system?**
  _30 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.08 - nodes in this community are weakly interconnected._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.06 - nodes in this community are weakly interconnected._