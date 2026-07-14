# Graph Report - /home/deivid/dev/DyNAMO-Ridgeback  (2026-07-14)

## Corpus Check
- 182 files · ~101,241 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1314 nodes · 1987 edges · 82 communities (64 shown, 18 thin omitted)
- Extraction: 90% EXTRACTED · 10% INFERRED · 0% AMBIGUOUS · INFERRED: 204 edges (avg confidence: 0.71)
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
- [[_COMMUNITY_Community 57|Community 57]]
- [[_COMMUNITY_Community 58|Community 58]]
- [[_COMMUNITY_Community 59|Community 59]]
- [[_COMMUNITY_Community 60|Community 60]]
- [[_COMMUNITY_Community 61|Community 61]]
- [[_COMMUNITY_Community 63|Community 63]]
- [[_COMMUNITY_Community 64|Community 64]]
- [[_COMMUNITY_Community 65|Community 65]]
- [[_COMMUNITY_Community 72|Community 72]]
- [[_COMMUNITY_Community 73|Community 73]]
- [[_COMMUNITY_Community 74|Community 74]]

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
- `run()` --calls--> `RidgebackRig`  [INFERRED]
  tools/isaac/diag_rig.py → src/ridgeback_autonomy/sim/isaac/robot_rig.py
- `generate()` --calls--> `get_assets_root()`  [INFERRED]
  tools/isaac/generate_gt_map.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `run()` --calls--> `resolve_world()`  [INFERRED]
  tools/isaac/diag_rig.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `generate()` --calls--> `resolve_world()`  [INFERRED]
  tools/isaac/generate_gt_map.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `test_parse_estimators_uses_canonical_order_and_depth_anything_name()` --calls--> `parse_estimators()`  [INFERRED]
  src/ridgeback_autonomy/test/test_benchmark_runner.py → src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/estimators.py

## Import Cycles
- None detected.

## Communities (82 total, 18 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.03
Nodes (77): BoolProperty, ColorProperty, EnumProperty, FloatProperty, IntProperty, Q_OBJECT, QStringList, RosTopicDisplay<rviz_2d_overlay_msgs::msg::OverlayText> (+69 more)

### Community 1 - "Community 1"
Cohesion: 0.08
Nodes (58): batch_from_detections_message(), batch_from_measurements_message(), build_detections_message(), build_float32_image_message(), build_measurements_message(), decode_bbox_quads(), decode_optional_float(), first_finite_positive() (+50 more)

### Community 2 - "Community 2"
Cohesion: 0.08
Nodes (36): ensure_measurement_event(), event_has_all_panel_previews(), event_has_panel_preview(), EventPreview, extract_public_estimator_values(), find_exact_preview_match(), find_nearest_preview_match(), has_all_selected_estimates() (+28 more)

### Community 3 - "Community 3"
Cohesion: 0.06
Nodes (17): _color_msg(), G1EstimateVizNode, _get(), main(), _ring_points(), main(), HudNode, main() (+9 more)

### Community 4 - "Community 4"
Cohesion: 0.06
Nodes (18): load_camera_config(), candidate_base_frames(), lookup_transform_components(), rotation_matrix_from_quaternion(), DepthAnythingEstimator, resolve_torch_device(), G1CameraMeasurementNode, main() (+10 more)

### Community 5 - "Community 5"
Cohesion: 0.06
Nodes (24): FrontierExplorerNode, main(), Receive and process occupancy grid (costmap)., Convert occupancy grid to robot awareness map (vectorized).          Nav2 publis, Get robot position in map frame., Main exploration loop., Check if a goal position has enough clearance from obstacles.                  C, Return True if pos is too close to a recently-visited goal. (+16 more)

### Community 6 - "Community 6"
Cohesion: 0.12
Nodes (12): stamp_to_nanoseconds(), extract_json_payload(), G1DistanceBenchmarkRunner, main(), build_summary_rows(), write_summary_csv(), write_trial_csv(), CompletedProcess (+4 more)

### Community 7 - "Community 7"
Cohesion: 0.09
Nodes (37): main(), observable_fraction(), build_grid(), check_waypoints(), _footprint_at_plane(), _geom_world_pose(), load_grid(), main() (+29 more)

### Community 8 - "Community 8"
Cohesion: 0.11
Nodes (20): bgr_frame_to_pil(), convert_color_image_message(), convert_depth_to_meters_message(), decode_buffer(), decode_image_message(), normalize_to_uint8(), dtype, G1OverlayNode (+12 more)

### Community 9 - "Community 9"
Cohesion: 0.06
Nodes (30): MaterialPtr, namespace, Overlay, PanelOverlayElement, QImage, OverlayObject, getBuffer, getName (+22 more)

### Community 10 - "Community 10"
Cohesion: 0.06
Nodes (30): attributes, omni:sensor:Core:accumulateOutputs, omni:sensor:Core:azimuthErrorStd, omni:sensor:Core:elementsCoordsType, omni:sensor:Core:elevationErrorStd, omni:sensor:Core:emitterState:s001:azimuthDeg, omni:sensor:Core:emitterState:s001:channelId, omni:sensor:Core:emitterState:s001:elevationDeg (+22 more)

### Community 11 - "Community 11"
Cohesion: 0.16
Nodes (26): drawPlot(), onEnable(), onInitialize(), PieChartDisplay(), processMessage(), setPosition(), update(), updateAutoColorChange() (+18 more)

### Community 12 - "Community 12"
Cohesion: 0.17
Nodes (13): ang_norm(), closure_counts(), correction_events(), dilate(), main(), map_metrics(), pose_errors(), Probe (+5 more)

### Community 13 - "Community 13"
Cohesion: 0.22
Nodes (25): onDisable(), onEnable(), onInitialize(), OverlayTextDisplay(), processMessage(), reset(), updateAlignBottom(), updateBGAlpha() (+17 more)

### Community 14 - "Community 14"
Cohesion: 0.10
Nodes (9): LidarScanAssembler, _quat_from_yaw(), In-process ROS 2 I/O for the Isaac runner (rclpy side).  Owns everything that is, Return and clear the newest cmd (vx, vy, wz), or None., True once if a reset was requested since the last call (consumed         by the, Raw IMU (no orientation estimate, like a real driver's         data_raw): body-f, Identity base_link -> <ns>/robot/base_link (perception default)., Bin the bridge's RTX lidar PointCloud2 into the contract LaserScan.      The 6.0 (+1 more)

### Community 15 - "Community 15"
Cohesion: 0.14
Nodes (9): parse_estimators(), selected_camera_estimators(), uses_camera_estimators(), uses_lidar_estimators(), RgbdOverlayRenderer, build_benchmark_nodes(), OrderedDict, DetectionBatch (+1 more)

### Community 16 - "Community 16"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 17 - "Community 17"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 18 - "Community 18"
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

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
Cohesion: 0.17
Nodes (7): main(), Probe, Clear per-run accumulators and re-base the clock for the next run., Preemption: another goal accepted within PREEMPT_WINDOW of abort., Return the robot to spawn + clear SLAM/costmaps for a fresh run., Observe one exploration cycle; returns when it completes (+10 s settle,     hono, _run_once()

### Community 28 - "Community 28"
Cohesion: 0.13
Nodes (12): compute_iou(), non_maximum_suppression(), OwlV2Detector, parse_owl_detections(), Detection, G1DetectorNode, main(), DetectionBatch (+4 more)

### Community 29 - "Community 29"
Cohesion: 0.10
Nodes (14): LoadConfig(), Parameter, SetParametersResult, SharedPtr, String, Rviz2dString, callback_handle_, fg_color (+6 more)

### Community 30 - "Community 30"
Cohesion: 0.15
Nodes (20): OverlayObject, getBuffer(), getName(), getPixelBuffer(), getQImage(), getTextureHeight(), getTextureWidth(), hide() (+12 more)

### Community 31 - "Community 31"
Cohesion: 0.13
Nodes (13): align_grids(), compute_stats(), occupancy_msg_to_grid(), pgm_to_grid(), Coverage comparison helpers: SLAM occupancy grid vs a ground-truth map.  Both gr, Compare two normalised grids cell-by-cell. Returns a dict of statistics.      Tw, Load a PGM + companion YAML and return a normalised int8 grid + meta dict., Convert a nav_msgs/OccupancyGrid to a normalised int8 grid + meta dict.      Occ (+5 more)

### Community 32 - "Community 32"
Cohesion: 0.18
Nodes (18): frontier_mask(), get_frontier_clusters(), is_frontier_point(), Checks if a point is a valid frontier (A FREE cell next to an UNKNOWN cell)., Group adjacent frontier points into clusters (8-connectivity flood fill).      R, Boolean mask of frontier cells: FREE cells 4-adjacent to an UNKNOWN cell.      V, _as_cluster_sets(), _grid() (+10 more)

### Community 33 - "Community 33"
Cohesion: 0.19
Nodes (19): add_planar_rig(), add_sensor_prims(), attach_visual_meshes(), _author_chassis_collider(), clearpath_package_paths(), generate_flat_urdf(), import_urdf_to_usd(), main() (+11 more)

### Community 34 - "Community 34"
Cohesion: 0.14
Nodes (8): OdomState, Kinematic-holonomic drive rig control + odometry for the Ridgeback.  The importe, Exact planar pose (x, y, yaw) from the rig joints., Integrate noisy odometry from true pose increments; return         (OdomState, b, odom_noise scales the default drift model (0 = perfect odom)., Call once ready() is true (physics playing, backend attached)., Accel-limit toward the commanded body twist, write joint targets., RidgebackRig

### Community 35 - "Community 35"
Cohesion: 0.21
Nodes (12): aborts(), check_gate(), cov(), is_rtf0(), load_runs(), main(), Return a list of (name, passed | None, detail). None = undecidable., Load every *_summary.json under a dir (or matching a glob), tagged by name. (+4 more)

### Community 36 - "Community 36"
Cohesion: 0.14
Nodes (13): camera, comment, cx, cy, depth_hfov_deg, depth_vfov_deg, fps, fx (+5 more)

### Community 37 - "Community 37"
Cohesion: 0.22
Nodes (13): _dominant_bounds(), _flood_free(), generate(), main(), parse_args(), _rasterize(), Bounds (xmin,ymin,xmax,ymax) of the spatially largest connected cluster     of s, Segments -> (occupied bool grid [iy,ix], origin (xmin,ymin)). Segments     outsi (+5 more)

### Community 38 - "Community 38"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 39 - "Community 39"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 40 - "Community 40"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 41 - "Community 41"
Cohesion: 0.22
Nodes (5): Demo, main(), save_camera(), save_map(), save_scan()

### Community 43 - "Community 43"
Cohesion: 0.39
Nodes (6): _body_vel(), main(), Step n render frames, refreshing cmd every frame like teleop would., run(), run_battery(), _step_frames()

### Community 44 - "Community 44"
Cohesion: 0.39
Nodes (7): attach_camera(), attach_lidars(), _find_prim_by_name(), Runtime ROS wiring for the robot package's GPU sensor prims (P4).  The sensor PR, Bind one render product + color/depth/points/camera_info publishers     to the b, Bind render products + bridge laser_scan publishers to the two     baked UST-10L, _render_product()

### Community 46 - "Community 46"
Cohesion: 0.47
Nodes (5): main(), parse_args(), run(), get_assets_root(), The configured NVIDIA asset root (S3/Nucleus), or None if unavailable.      Stoc

### Community 47 - "Community 47"
Cohesion: 0.47
Nodes (5): _candidate_dirs(), World-name resolution for the Isaac Sim runner.  Keeps the gz-era operator contr, Return a loadable stage path/URL for a world name or explicit path., resolve_world(), Path

### Community 48 - "Community 48"
Cohesion: 0.50
Nodes (3): start_exploration.sh script, RMW_IMPLEMENTATION, ROS_DOMAIN_ID

### Community 49 - "Community 49"
Cohesion: 0.83
Nodes (3): mcp_query.sh script, _data(), _post()

## Knowledge Gaps
- **461 isolated node(s):** `PreToolUse`, `allow`, `PreToolUse`, `isaac-sim-mcp`, `build_and_start_expl.sh script` (+456 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **18 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Time` connect `Community 3` to `Community 5`, `Community 6`, `Community 41`, `Community 12`, `Community 46`, `Community 14`, `Community 27`, `Community 28`?**
  _High betweenness centrality (0.111) - this node is a cross-community bridge._
- **Why does `G1DistanceBenchmarkRunner` connect `Community 6` to `Community 2`, `Community 3`, `Community 15`?**
  _High betweenness centrality (0.084) - this node is a cross-community bridge._
- **Why does `run()` connect `Community 46` to `Community 34`, `Community 44`, `Community 14`, `Community 47`?**
  _High betweenness centrality (0.057) - this node is a cross-community bridge._
- **Are the 10 inferred relationships involving `BenchmarkCollageRenderer` (e.g. with `G1DistanceBenchmarkRunner` and `.__init__()`) actually correct?**
  _`BenchmarkCollageRenderer` has 10 INFERRED edges - model-reasoned connections that need verification._
- **What connects `PreToolUse`, `allow`, `PreToolUse` to the rest of the system?**
  _547 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.025974025974025976 - nodes in this community are weakly interconnected._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.07505827505827506 - nodes in this community are weakly interconnected._