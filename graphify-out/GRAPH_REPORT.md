# Graph Report - /home/deivid/dev/DyNAMO-Ridgeback/.claude/worktrees/jolly-borg-f48cab  (2026-07-11)

## Corpus Check
- 136 files · ~68,081 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 867 nodes · 1455 edges · 62 communities (45 shown, 17 thin omitted)
- Extraction: 87% EXTRACTED · 13% INFERRED · 0% AMBIGUOUS · INFERRED: 193 edges (avg confidence: 0.71)
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
- [[_COMMUNITY_Community 13|Community 13]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 17|Community 17]]
- [[_COMMUNITY_Community 18|Community 18]]
- [[_COMMUNITY_Community 19|Community 19]]
- [[_COMMUNITY_Community 20|Community 20]]
- [[_COMMUNITY_Community 21|Community 21]]
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 24|Community 24]]
- [[_COMMUNITY_Community 25|Community 25]]
- [[_COMMUNITY_Community 26|Community 26]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]
- [[_COMMUNITY_Community 31|Community 31]]
- [[_COMMUNITY_Community 32|Community 32]]
- [[_COMMUNITY_Community 33|Community 33]]
- [[_COMMUNITY_Community 34|Community 34]]
- [[_COMMUNITY_Community 35|Community 35]]
- [[_COMMUNITY_Community 37|Community 37]]
- [[_COMMUNITY_Community 38|Community 38]]
- [[_COMMUNITY_Community 39|Community 39]]
- [[_COMMUNITY_Community 40|Community 40]]
- [[_COMMUNITY_Community 41|Community 41]]
- [[_COMMUNITY_Community 43|Community 43]]
- [[_COMMUNITY_Community 44|Community 44]]
- [[_COMMUNITY_Community 45|Community 45]]
- [[_COMMUNITY_Community 52|Community 52]]
- [[_COMMUNITY_Community 53|Community 53]]
- [[_COMMUNITY_Community 54|Community 54]]

## God Nodes (most connected - your core abstractions)
1. `OverlayTextDisplay` - 77 edges
2. `G1DistanceBenchmarkRunner` - 36 edges
3. `FrontierExplorerNode` - 26 edges
4. `BenchmarkCollageRenderer` - 25 edges
5. `OverlayObject` - 22 edges
6. `DetectionBatch` - 21 edges
7. `OverlayTextDisplay()` - 20 edges
8. `onInitialize()` - 20 edges
9. `onInitialize()` - 20 edges
10. `PieChartDisplay()` - 19 edges

## Surprising Connections (you probably didn't know these)
- `CameraConfig` --uses--> `CameraConfig`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/common/camera_config.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `DetectionBatch` --uses--> `DetectionBatch`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/perception/core/rendering.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `run()` --calls--> `RidgebackRig`  [INFERRED]
  tools/isaac/diag_rig.py → src/ridgeback_autonomy/sim/isaac/robot_rig.py
- `run()` --calls--> `resolve_world()`  [INFERRED]
  tools/isaac/diag_rig.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `BenchmarkCollageRenderer` --uses--> `MeasurementEvent`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/rendering.py → src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/alignment.py

## Import Cycles
- None detected.

## Communities (62 total, 17 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.03
Nodes (77): BoolProperty, ColorProperty, EnumProperty, FloatProperty, IntProperty, Q_OBJECT, QStringList, RosTopicDisplay<rviz_2d_overlay_msgs::msg::OverlayText> (+69 more)

### Community 1 - "Community 1"
Cohesion: 0.08
Nodes (34): batch_from_detections_message(), batch_from_measurements_message(), build_detections_message(), build_float32_image_message(), build_measurements_message(), decode_bbox_quads(), decode_optional_float(), first_finite_positive() (+26 more)

### Community 2 - "Community 2"
Cohesion: 0.06
Nodes (23): FrontierExplorerNode, main(), Receive and process occupancy grid (costmap)., Convert occupancy grid to robot awareness map (vectorized).          Nav2 publis, Get robot position in map frame., Main exploration loop., Check if a goal position has enough clearance from obstacles.                  C, Return True if pos is too close to a recently-visited goal. (+15 more)

### Community 3 - "Community 3"
Cohesion: 0.08
Nodes (25): DepthAnythingEstimator, resolve_torch_device(), bgr_frame_to_pil(), convert_color_image_message(), convert_depth_to_meters_message(), decode_buffer(), decode_image_message(), normalize_to_uint8() (+17 more)

### Community 4 - "Community 4"
Cohesion: 0.12
Nodes (12): stamp_to_nanoseconds(), extract_json_payload(), G1DistanceBenchmarkRunner, main(), build_summary_rows(), write_summary_csv(), write_trial_csv(), CompletedProcess (+4 more)

### Community 5 - "Community 5"
Cohesion: 0.13
Nodes (35): CameraConfig, DetectionBatch, LidarScanPoints, add_depth_measurements(), add_depth_source_measurements(), add_lidar_measurements(), add_pointcloud_measurements(), add_rgb_measurements() (+27 more)

### Community 6 - "Community 6"
Cohesion: 0.11
Nodes (31): ensure_measurement_event(), event_has_all_panel_previews(), event_has_panel_preview(), EventPreview, extract_public_estimator_values(), find_exact_preview_match(), find_nearest_preview_match(), has_all_selected_estimates() (+23 more)

### Community 7 - "Community 7"
Cohesion: 0.08
Nodes (22): _body_vel(), main(), Step n render frames, refreshing cmd every frame like teleop would., run(), run_battery(), _step_frames(), main(), parse_args() (+14 more)

### Community 8 - "Community 8"
Cohesion: 0.09
Nodes (13): _color_msg(), G1EstimateVizNode, _get(), main(), _ring_points(), main(), _quat_from_yaw(), In-process ROS 2 I/O for the Isaac runner (rclpy side).  Owns everything that is (+5 more)

### Community 9 - "Community 9"
Cohesion: 0.06
Nodes (30): MaterialPtr, namespace, Overlay, PanelOverlayElement, QImage, OverlayObject, getBuffer, getName (+22 more)

### Community 10 - "Community 10"
Cohesion: 0.14
Nodes (11): parse_estimators(), selected_camera_estimators(), uses_camera_estimators(), uses_lidar_estimators(), BenchmarkCollageRenderer, PanelContext, build_benchmark_nodes(), OrderedDict (+3 more)

### Community 11 - "Community 11"
Cohesion: 0.16
Nodes (26): drawPlot(), onEnable(), onInitialize(), PieChartDisplay(), processMessage(), setPosition(), update(), updateAutoColorChange() (+18 more)

### Community 12 - "Community 12"
Cohesion: 0.12
Nodes (8): main(), Probe, Preemption: another goal accepted within PREEMPT_WINDOW of abort., HudNode, main(), main(), VelocityOverlayNode, Node

### Community 13 - "Community 13"
Cohesion: 0.22
Nodes (25): onDisable(), onEnable(), onInitialize(), OverlayTextDisplay(), processMessage(), reset(), updateAlignBottom(), updateBGAlpha() (+17 more)

### Community 14 - "Community 14"
Cohesion: 0.13
Nodes (10): load_camera_config(), candidate_base_frames(), lookup_transform_components(), rotation_matrix_from_quaternion(), G1LidarMeasurementNode, main(), CameraConfig, ndarray (+2 more)

### Community 15 - "Community 15"
Cohesion: 0.10
Nodes (14): LoadConfig(), Parameter, SetParametersResult, SharedPtr, String, Rviz2dString, callback_handle_, fg_color (+6 more)

### Community 16 - "Community 16"
Cohesion: 0.19
Nodes (21): _apply(), check_usd(), emit_usd(), _floats(), Geom, Include, Light, main() (+13 more)

### Community 17 - "Community 17"
Cohesion: 0.15
Nodes (20): OverlayObject, getBuffer(), getName(), getPixelBuffer(), getQImage(), getTextureHeight(), getTextureWidth(), hide() (+12 more)

### Community 18 - "Community 18"
Cohesion: 0.13
Nodes (13): align_grids(), compute_stats(), occupancy_msg_to_grid(), pgm_to_grid(), Coverage comparison helpers: SLAM occupancy grid vs a ground-truth map.  Both gr, Compare two normalised grids cell-by-cell. Returns a dict of statistics.      Tw, Load a PGM + companion YAML and return a normalised int8 grid + meta dict., Convert a nav_msgs/OccupancyGrid to a normalised int8 grid + meta dict.      Occ (+5 more)

### Community 19 - "Community 19"
Cohesion: 0.18
Nodes (18): frontier_mask(), get_frontier_clusters(), is_frontier_point(), Checks if a point is a valid frontier (A FREE cell next to an UNKNOWN cell)., Group adjacent frontier points into clusters (8-connectivity flood fill).      R, Boolean mask of frontier cells: FREE cells 4-adjacent to an UNKNOWN cell.      V, _as_cluster_sets(), _grid() (+10 more)

### Community 20 - "Community 20"
Cohesion: 0.21
Nodes (4): G1OverlayNode, main(), G1Measurements, Image

### Community 21 - "Community 21"
Cohesion: 0.24
Nodes (15): add_planar_rig(), _author_chassis_collider(), clearpath_package_paths(), generate_flat_urdf(), import_urdf_to_usd(), main(), package:// roots the URDF references, for the importer's resolver., # NOTE: app.close() fast-shuts the process — nothing after it runs. (+7 more)

### Community 22 - "Community 22"
Cohesion: 0.28
Nodes (3): RgbdOverlayRenderer, DetectionBatch, ndarray

### Community 23 - "Community 23"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 24 - "Community 24"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 25 - "Community 25"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 28 - "Community 28"
Cohesion: 0.33
Nodes (5): camera, depth_hfov_deg, depth_vfov_deg, height_m, pitch_deg

### Community 29 - "Community 29"
Cohesion: 0.50
Nodes (3): start_exploration.sh script, RMW_IMPLEMENTATION, ROS_DOMAIN_ID

### Community 30 - "Community 30"
Cohesion: 0.83
Nodes (3): mcp_query.sh script, _data(), _post()

## Knowledge Gaps
- **170 isolated node(s):** `PreToolUse`, `allow`, `PreToolUse`, `isaac-sim-mcp`, `build_and_start_expl.sh script` (+165 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **17 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `G1DistanceBenchmarkRunner` connect `Community 4` to `Community 10`, `Community 12`, `Community 6`?**
  _High betweenness centrality (0.107) - this node is a cross-community bridge._
- **Why does `FrontierExplorerNode` connect `Community 2` to `Community 12`?**
  _High betweenness centrality (0.084) - this node is a cross-community bridge._
- **Why does `Time` connect `Community 8` to `Community 1`, `Community 2`, `Community 4`, `Community 7`, `Community 12`?**
  _High betweenness centrality (0.070) - this node is a cross-community bridge._
- **Are the 10 inferred relationships involving `BenchmarkCollageRenderer` (e.g. with `G1DistanceBenchmarkRunner` and `.__init__()`) actually correct?**
  _`BenchmarkCollageRenderer` has 10 INFERRED edges - model-reasoned connections that need verification._
- **What connects `PreToolUse`, `allow`, `PreToolUse` to the rest of the system?**
  _227 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.025974025974025976 - nodes in this community are weakly interconnected._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.07955596669750231 - nodes in this community are weakly interconnected._