# Graph Report - /home/deivid/dev/DyNAMO-Ridgeback/.claude/worktrees/isaac  (2026-09-11)

## Corpus Check
- 197 files · ~129,689 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1411 nodes · 2140 edges · 85 communities (67 shown, 18 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 213 edges (avg confidence: 0.72)
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
- [[_COMMUNITY_Community 27|Community 27]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]
- [[_COMMUNITY_Community 31|Community 31]]
- [[_COMMUNITY_Community 32|Community 32]]
- [[_COMMUNITY_Community 33|Community 33]]
- [[_COMMUNITY_Community 34|Community 34]]
- [[_COMMUNITY_Community 35|Community 35]]
- [[_COMMUNITY_Community 36|Community 36]]
- [[_COMMUNITY_Community 37|Community 37]]
- [[_COMMUNITY_Community 38|Community 38]]
- [[_COMMUNITY_Community 39|Community 39]]
- [[_COMMUNITY_Community 40|Community 40]]
- [[_COMMUNITY_Community 41|Community 41]]
- [[_COMMUNITY_Community 42|Community 42]]
- [[_COMMUNITY_Community 43|Community 43]]
- [[_COMMUNITY_Community 44|Community 44]]
- [[_COMMUNITY_Community 45|Community 45]]
- [[_COMMUNITY_Community 46|Community 46]]
- [[_COMMUNITY_Community 47|Community 47]]
- [[_COMMUNITY_Community 48|Community 48]]
- [[_COMMUNITY_Community 49|Community 49]]
- [[_COMMUNITY_Community 50|Community 50]]
- [[_COMMUNITY_Community 51|Community 51]]
- [[_COMMUNITY_Community 52|Community 52]]
- [[_COMMUNITY_Community 53|Community 53]]
- [[_COMMUNITY_Community 54|Community 54]]
- [[_COMMUNITY_Community 55|Community 55]]
- [[_COMMUNITY_Community 56|Community 56]]
- [[_COMMUNITY_Community 57|Community 57]]
- [[_COMMUNITY_Community 58|Community 58]]
- [[_COMMUNITY_Community 60|Community 60]]
- [[_COMMUNITY_Community 61|Community 61]]
- [[_COMMUNITY_Community 62|Community 62]]
- [[_COMMUNITY_Community 63|Community 63]]
- [[_COMMUNITY_Community 64|Community 64]]
- [[_COMMUNITY_Community 66|Community 66]]
- [[_COMMUNITY_Community 67|Community 67]]
- [[_COMMUNITY_Community 68|Community 68]]
- [[_COMMUNITY_Community 75|Community 75]]
- [[_COMMUNITY_Community 76|Community 76]]
- [[_COMMUNITY_Community 77|Community 77]]

## God Nodes (most connected - your core abstractions)
1. `OverlayTextDisplay` - 77 edges
2. `G1DistanceBenchmarkRunner` - 36 edges
3. `attributes` - 29 edges
4. `FrontierExplorerNode` - 27 edges
5. `BenchmarkCollageRenderer` - 25 edges
6. `OverlayObject` - 22 edges
7. `DetectionBatch` - 21 edges
8. `OverlayTextDisplay()` - 20 edges
9. `onInitialize()` - 20 edges
10. `onInitialize()` - 20 edges

## Surprising Connections (you probably didn't know these)
- `generate()` --calls--> `get_assets_root()`  [INFERRED]
  tools/isaac/generate_gt_map.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `generate()` --calls--> `resolve_world()`  [INFERRED]
  tools/isaac/generate_gt_map.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `test_parse_estimators_uses_canonical_order_and_depth_anything_name()` --calls--> `parse_estimators()`  [INFERRED]
  src/ridgeback_autonomy/test/test_benchmark_runner.py → src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/estimators.py
- `CameraConfig` --uses--> `CameraConfig`  [INFERRED]
  src/ridgeback_autonomy/ridgeback_autonomy/common/camera_config.py → src/ridgeback_autonomy/ridgeback_autonomy/common/models.py
- `run()` --calls--> `RidgebackRig`  [INFERRED]
  tools/isaac/diag_rig.py → src/ridgeback_autonomy/sim/isaac/robot_rig.py

## Import Cycles
- None detected.

## Communities (85 total, 18 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.05
Nodes (48): ensure_measurement_event(), event_has_all_panel_previews(), event_has_panel_preview(), EventPreview, extract_public_estimator_values(), find_exact_preview_match(), find_nearest_preview_match(), has_all_selected_estimates() (+40 more)

### Community 1 - "Community 1"
Cohesion: 0.03
Nodes (77): BoolProperty, ColorProperty, EnumProperty, FloatProperty, IntProperty, Q_OBJECT, QStringList, RosTopicDisplay<rviz_2d_overlay_msgs::msg::OverlayText> (+69 more)

### Community 2 - "Community 2"
Cohesion: 0.05
Nodes (39): _body_vel(), main(), Does the LIDAR prim's world transform keep up with the chassis body?      The de, Step n render frames, refreshing cmd every frame like teleop would., run(), run_battery(), run_spin_transforms(), _step_frames() (+31 more)

### Community 3 - "Community 3"
Cohesion: 0.09
Nodes (38): CameraConfig, DetectionBatch, LidarScanPoints, add_depth_measurements(), add_depth_source_measurements(), add_lidar_measurements(), add_pointcloud_measurements(), add_rgb_measurements() (+30 more)

### Community 4 - "Community 4"
Cohesion: 0.08
Nodes (34): batch_from_detections_message(), batch_from_measurements_message(), build_detections_message(), build_float32_image_message(), build_measurements_message(), decode_bbox_quads(), decode_optional_float(), first_finite_positive() (+26 more)

### Community 5 - "Community 5"
Cohesion: 0.08
Nodes (25): DepthAnythingEstimator, resolve_torch_device(), bgr_frame_to_pil(), convert_color_image_message(), convert_depth_to_meters_message(), decode_buffer(), decode_image_message(), normalize_to_uint8() (+17 more)

### Community 6 - "Community 6"
Cohesion: 0.07
Nodes (16): HudNode, main(), _ang_norm(), LocalizationOverlayNode, main(), _yaw_of(), main(), VelocityOverlayNode (+8 more)

### Community 7 - "Community 7"
Cohesion: 0.09
Nodes (13): _color_msg(), G1EstimateVizNode, _get(), main(), _ring_points(), main(), Demo, main() (+5 more)

### Community 8 - "Community 8"
Cohesion: 0.06
Nodes (30): MaterialPtr, namespace, Overlay, PanelOverlayElement, QImage, OverlayObject, getBuffer, getName (+22 more)

### Community 9 - "Community 9"
Cohesion: 0.10
Nodes (23): align_grids(), compute_stats(), occupancy_msg_to_grid(), pgm_to_grid(), Coverage comparison helpers: SLAM occupancy grid vs a ground-truth map.  Both gr, Crop both grids to their overlapping world region at the ground-truth     resolu, Compare two normalised grids cell-by-cell. Returns a dict of statistics.      Tw, Load a PGM + companion YAML and return a normalised int8 grid + meta dict. (+15 more)

### Community 10 - "Community 10"
Cohesion: 0.12
Nodes (30): add_planar_rig(), add_sensor_prims(), attach_visual_meshes(), _author_camera_mast(), _author_chassis_collider(), _bind(), _camera_mesh_bounds(), clearpath_package_paths() (+22 more)

### Community 11 - "Community 11"
Cohesion: 0.06
Nodes (30): attributes, omni:sensor:Core:accumulateOutputs, omni:sensor:Core:azimuthErrorStd, omni:sensor:Core:elementsCoordsType, omni:sensor:Core:elevationErrorStd, omni:sensor:Core:emitterState:s001:azimuthDeg, omni:sensor:Core:emitterState:s001:channelId, omni:sensor:Core:emitterState:s001:elevationDeg (+22 more)

### Community 12 - "Community 12"
Cohesion: 0.09
Nodes (27): main(), observable_fraction(), _dominant_bounds(), generate(), main(), parse_args(), _rasterize(), Bounds (xmin,ymin,xmax,ymax) of the spatially largest connected cluster     of s (+19 more)

### Community 13 - "Community 13"
Cohesion: 0.16
Nodes (26): drawPlot(), onEnable(), onInitialize(), PieChartDisplay(), processMessage(), setPosition(), update(), updateAutoColorChange() (+18 more)

### Community 14 - "Community 14"
Cohesion: 0.12
Nodes (10): parse_estimators(), selected_camera_estimators(), uses_camera_estimators(), uses_lidar_estimators(), build_benchmark_nodes(), OrderedDict, G1OverlayNode, main() (+2 more)

### Community 15 - "Community 15"
Cohesion: 0.09
Nodes (10): LidarScanAssembler, _quat_from_yaw(), In-process ROS 2 I/O for the Isaac runner (rclpy side).  Owns everything that is, Return and clear the newest cmd (vx, vy, wz), or None., True once if a reset was requested since the last call (consumed         by the, Bin the bridge's RTX lidar PointCloud2 into the contract LaserScan.      The 6.0, Raw IMU (no orientation estimate, like a real driver's         data_raw): body-f, Identity base_link -> <ns>/robot/base_link (perception default). (+2 more)

### Community 16 - "Community 16"
Cohesion: 0.17
Nodes (13): ang_norm(), closure_counts(), correction_events(), dilate(), main(), map_metrics(), pose_errors(), Probe (+5 more)

### Community 17 - "Community 17"
Cohesion: 0.22
Nodes (25): onDisable(), onEnable(), onInitialize(), OverlayTextDisplay(), processMessage(), reset(), updateAlignBottom(), updateBGAlpha() (+17 more)

### Community 18 - "Community 18"
Cohesion: 0.17
Nodes (23): _apply(), check_usd(), emit_usd(), _floats(), _freeze_include_dynamics(), Geom, Include, Light (+15 more)

### Community 19 - "Community 19"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 20 - "Community 20"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 21 - "Community 21"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 22 - "Community 22"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 23 - "Community 23"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 24 - "Community 24"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 25 - "Community 25"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 26 - "Community 26"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 27 - "Community 27"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 28 - "Community 28"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 29 - "Community 29"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 30 - "Community 30"
Cohesion: 0.17
Nodes (7): main(), Probe, Clear per-run accumulators and re-base the clock for the next run., Preemption: another goal accepted within PREEMPT_WINDOW of abort., Return the robot to spawn + clear SLAM/costmaps for a fresh run., Observe one exploration cycle; returns when it completes (+10 s settle,     hono, _run_once()

### Community 31 - "Community 31"
Cohesion: 0.13
Nodes (10): load_camera_config(), candidate_base_frames(), lookup_transform_components(), rotation_matrix_from_quaternion(), G1LidarMeasurementNode, main(), CameraConfig, ndarray (+2 more)

### Community 32 - "Community 32"
Cohesion: 0.10
Nodes (14): LoadConfig(), Parameter, SetParametersResult, SharedPtr, String, Rviz2dString, callback_handle_, fg_color (+6 more)

### Community 33 - "Community 33"
Cohesion: 0.15
Nodes (20): OverlayObject, getBuffer(), getName(), getPixelBuffer(), getQImage(), getTextureHeight(), getTextureWidth(), hide() (+12 more)

### Community 34 - "Community 34"
Cohesion: 0.18
Nodes (18): frontier_mask(), get_frontier_clusters(), is_frontier_point(), Checks if a point is a valid frontier (A FREE cell next to an UNKNOWN cell)., Group adjacent frontier points into clusters (8-connectivity flood fill).      R, Boolean mask of frontier cells: FREE cells 4-adjacent to an UNKNOWN cell.      V, _as_cluster_sets(), _grid() (+10 more)

### Community 35 - "Community 35"
Cohesion: 0.19
Nodes (7): main(), Points from a raw LaserScan, in the frame `lidar_pose` (x,y,yaw)         is expr, Try to resolve every buffered, not-yet-published front scan.         Called from, ScanMergerNode, stamp_s(), yaw_of(), LaserScan

### Community 36 - "Community 36"
Cohesion: 0.21
Nodes (12): aborts(), check_gate(), cov(), is_rtf0(), load_runs(), main(), Return a list of (name, passed | None, detail). None = undecidable., Load every *_summary.json under a dir (or matching a glob), tagged by name. (+4 more)

### Community 37 - "Community 37"
Cohesion: 0.14
Nodes (13): camera, comment, cx, cy, depth_hfov_deg, depth_vfov_deg, fps, fx (+5 more)

### Community 38 - "Community 38"
Cohesion: 0.21
Nodes (6): FrontierExplorerNode, main(), Receive and process occupancy grid (costmap)., Convert occupancy grid to robot awareness map (vectorized).          Nav2 publis, Reset the stall timer whenever distance to goal improves., OccupancyGrid

### Community 39 - "Community 39"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 40 - "Community 40"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 41 - "Community 41"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 43 - "Community 43"
Cohesion: 0.20
Nodes (5): Get robot position in map frame., Main exploration loop., Cancel the active Nav2 goal, if any., Best-effort clear of both costmaps (fire-and-forget) on a stall., Cancel the goal if the robot has made no progress toward it.          Stall-base

### Community 44 - "Community 44"
Cohesion: 0.22
Nodes (5): Tally a navigation failure for goal, merging nearby entries.          Returns th, Drop failure tallies near goal (called on success or blacklist)., Count a failure; blacklist the goal once it keeps failing., Handle goal acceptance; attach result callback., Handle goal completion — immediately pick a new frontier.

### Community 45 - "Community 45"
Cohesion: 0.22
Nodes (7): frame_camera(), place(), Point the viewport at the robot from a three-quarter view.      The camera-to-wo, Reference an asset under a plain offset parent.      The referenced layers autho, set_approximation(), Prim, Path

### Community 47 - "Community 47"
Cohesion: 0.31
Nodes (7): bin_degrees(), Contract tests for the Isaac RTX-lidar scan assembler (no rclpy needed).  Covers, Each masked sector must be covered by the opposite, back-to-back unit,     so no, test_contract_scan_geometry_is_the_ust_10lx_window(), test_edge_mask_covers_the_measured_phantom_band_on_both_ends(), test_edge_mask_is_symmetric_and_only_touches_the_arc_ends(), test_masked_sector_is_inside_the_other_lidar_arc()

### Community 48 - "Community 48"
Cohesion: 0.25
Nodes (4): Check if a goal position has enough clearance from obstacles.                  C, Return True if pos is too close to a recently-visited goal., Return the goal position for a cluster.          Advances PAST the frontier boun, Find the best safe, non-blacklisted frontier.          Selection strategy (tier-

### Community 49 - "Community 49"
Cohesion: 0.29
Nodes (4): Select a new frontier goal., Publish frontier cluster centroids as spheres for RViz., Convert grid coordinates to world coordinates., Send goal to Nav2 NavigateToPose action. Returns True if sent.

### Community 50 - "Community 50"
Cohesion: 0.36
Nodes (7): cast(), main(), Nearest segment hit per bearing. segs: (n,4) array., Render-purpose, visible Mesh prims — what the RTX lidar can hit.      Excludes c, Plane-crossing segments [(x0,y0,x1,y1)] for one mesh, in base_link., slice_prim(), visible_render_meshes()

### Community 51 - "Community 51"
Cohesion: 0.70
Nodes (4): fetch(), main(), parse_args(), Path

### Community 52 - "Community 52"
Cohesion: 0.50
Nodes (3): start_exploration.sh script, RMW_IMPLEMENTATION, ROS_DOMAIN_ID

### Community 53 - "Community 53"
Cohesion: 0.83
Nodes (3): mcp_query.sh script, _data(), _post()

## Knowledge Gaps
- **463 isolated node(s):** `PreToolUse`, `PreToolUse`, `isaac-sim-mcp`, `build_and_start_expl.sh script`, `tag` (+458 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **18 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Time` connect `Community 7` to `Community 0`, `Community 2`, `Community 35`, `Community 4`, `Community 38`, `Community 6`, `Community 15`, `Community 16`, `Community 30`?**
  _High betweenness centrality (0.101) - this node is a cross-community bridge._
- **Why does `FrontierExplorerNode` connect `Community 38` to `Community 6`, `Community 43`, `Community 44`, `Community 48`, `Community 49`?**
  _High betweenness centrality (0.069) - this node is a cross-community bridge._
- **Why does `G1DistanceBenchmarkRunner` connect `Community 0` to `Community 14`, `Community 6`?**
  _High betweenness centrality (0.060) - this node is a cross-community bridge._
- **Are the 10 inferred relationships involving `BenchmarkCollageRenderer` (e.g. with `G1DistanceBenchmarkRunner` and `.__init__()`) actually correct?**
  _`BenchmarkCollageRenderer` has 10 INFERRED edges - model-reasoned connections that need verification._
- **What connects `PreToolUse`, `PreToolUse`, `isaac-sim-mcp` to the rest of the system?**
  _572 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.05112347969490827 - nodes in this community are weakly interconnected._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.025974025974025976 - nodes in this community are weakly interconnected._