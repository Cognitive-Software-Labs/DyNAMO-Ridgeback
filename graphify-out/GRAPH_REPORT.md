# Graph Report - /home/deivid/dev/DyNAMO-Ridgeback/.claude/worktrees/isaac  (2026-09-11)

## Corpus Check
- 197 files · ~127,856 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1409 nodes · 2137 edges · 86 communities (68 shown, 18 thin omitted)
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
- [[_COMMUNITY_Community 59|Community 59]]
- [[_COMMUNITY_Community 61|Community 61]]
- [[_COMMUNITY_Community 62|Community 62]]
- [[_COMMUNITY_Community 63|Community 63]]
- [[_COMMUNITY_Community 64|Community 64]]
- [[_COMMUNITY_Community 65|Community 65]]
- [[_COMMUNITY_Community 67|Community 67]]
- [[_COMMUNITY_Community 68|Community 68]]
- [[_COMMUNITY_Community 69|Community 69]]
- [[_COMMUNITY_Community 76|Community 76]]
- [[_COMMUNITY_Community 77|Community 77]]
- [[_COMMUNITY_Community 78|Community 78]]

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
- `run()` --calls--> `get_assets_root()`  [INFERRED]
  tools/isaac/world_probe.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `run()` --calls--> `resolve_world()`  [INFERRED]
  tools/isaac/diag_rig.py → src/ridgeback_autonomy/sim/isaac/worlds.py
- `generate()` --calls--> `resolve_world()`  [INFERRED]
  tools/isaac/generate_gt_map.py → src/ridgeback_autonomy/sim/isaac/worlds.py

## Import Cycles
- None detected.

## Communities (86 total, 18 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.05
Nodes (46): ensure_measurement_event(), event_has_all_panel_previews(), event_has_panel_preview(), EventPreview, find_exact_preview_match(), find_nearest_preview_match(), has_all_selected_estimates(), measurement_message_key() (+38 more)

### Community 1 - "Community 1"
Cohesion: 0.03
Nodes (77): BoolProperty, ColorProperty, EnumProperty, FloatProperty, IntProperty, Q_OBJECT, QStringList, RosTopicDisplay<rviz_2d_overlay_msgs::msg::OverlayText> (+69 more)

### Community 2 - "Community 2"
Cohesion: 0.07
Nodes (59): extract_public_estimator_values(), batch_from_detections_message(), batch_from_measurements_message(), build_detections_message(), build_float32_image_message(), build_measurements_message(), decode_bbox_quads(), decode_optional_float() (+51 more)

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
Cohesion: 0.09
Nodes (27): main(), observable_fraction(), build_grid(), check_waypoints(), _footprint_at_plane(), _geom_world_pose(), load_grid(), main() (+19 more)

### Community 7 - "Community 7"
Cohesion: 0.11
Nodes (20): bgr_frame_to_pil(), convert_color_image_message(), convert_depth_to_meters_message(), decode_buffer(), decode_image_message(), normalize_to_uint8(), dtype, G1OverlayNode (+12 more)

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
Cohesion: 0.16
Nodes (26): drawPlot(), onEnable(), onInitialize(), PieChartDisplay(), processMessage(), setPosition(), update(), updateAutoColorChange() (+18 more)

### Community 13 - "Community 13"
Cohesion: 0.09
Nodes (10): LidarScanAssembler, _quat_from_yaw(), In-process ROS 2 I/O for the Isaac runner (rclpy side).  Owns everything that is, Return and clear the newest cmd (vx, vy, wz), or None., True once if a reset was requested since the last call (consumed         by the, Bin the bridge's RTX lidar PointCloud2 into the contract LaserScan.      The 6.0, Raw IMU (no orientation estimate, like a real driver's         data_raw): body-f, Identity base_link -> <ns>/robot/base_link (perception default). (+2 more)

### Community 14 - "Community 14"
Cohesion: 0.22
Nodes (25): onDisable(), onEnable(), onInitialize(), OverlayTextDisplay(), processMessage(), reset(), updateAlignBottom(), updateBGAlpha() (+17 more)

### Community 15 - "Community 15"
Cohesion: 0.13
Nodes (10): parse_estimators(), selected_camera_estimators(), uses_camera_estimators(), uses_lidar_estimators(), RgbdOverlayRenderer, build_benchmark_nodes(), OrderedDict, DetectionBatch (+2 more)

### Community 16 - "Community 16"
Cohesion: 0.17
Nodes (23): _apply(), check_usd(), emit_usd(), _floats(), _freeze_include_dynamics(), Geom, Include, Light (+15 more)

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
Cohesion: 0.08
Nodes (23): abort_reason, aborted, closure_log_available, closures_accepted, closures_rejected, drive_duration_s, gt_occupied_cells_observed, loop_error_max_m (+15 more)

### Community 28 - "Community 28"
Cohesion: 0.17
Nodes (7): main(), Probe, Clear per-run accumulators and re-base the clock for the next run., Preemption: another goal accepted within PREEMPT_WINDOW of abort., Return the robot to spawn + clear SLAM/costmaps for a fresh run., Observe one exploration cycle; returns when it completes (+10 s settle,     hono, _run_once()

### Community 29 - "Community 29"
Cohesion: 0.10
Nodes (14): LoadConfig(), Parameter, SetParametersResult, SharedPtr, String, Rviz2dString, callback_handle_, fg_color (+6 more)

### Community 30 - "Community 30"
Cohesion: 0.15
Nodes (20): OverlayObject, getBuffer(), getName(), getPixelBuffer(), getQImage(), getTextureHeight(), getTextureWidth(), hide() (+12 more)

### Community 31 - "Community 31"
Cohesion: 0.15
Nodes (11): compute_iou(), non_maximum_suppression(), OwlV2Detector, parse_owl_detections(), Detection, G1DetectorNode, main(), DetectionBatch (+3 more)

### Community 32 - "Community 32"
Cohesion: 0.18
Nodes (18): frontier_mask(), get_frontier_clusters(), is_frontier_point(), Checks if a point is a valid frontier (A FREE cell next to an UNKNOWN cell)., Group adjacent frontier points into clusters (8-connectivity flood fill).      R, Boolean mask of frontier cells: FREE cells 4-adjacent to an UNKNOWN cell.      V, _as_cluster_sets(), _grid() (+10 more)

### Community 33 - "Community 33"
Cohesion: 0.19
Nodes (7): main(), Points from a raw LaserScan, in the frame `lidar_pose` (x,y,yaw)         is expr, Try to resolve every buffered, not-yet-published front scan.         Called from, ScanMergerNode, stamp_s(), yaw_of(), LaserScan

### Community 34 - "Community 34"
Cohesion: 0.14
Nodes (8): OdomState, Kinematic-holonomic drive rig control + odometry for the Ridgeback.  The importe, Exact planar pose (x, y, yaw) from the rig joints., Integrate noisy odometry from true pose increments; return         (OdomState, b, odom_noise scales the default drift model (0 = perfect odom)., Call once ready() is true (physics playing, backend attached)., Accel-limit toward the commanded body twist, write joint targets., RidgebackRig

### Community 35 - "Community 35"
Cohesion: 0.21
Nodes (12): aborts(), check_gate(), cov(), is_rtf0(), load_runs(), main(), Return a list of (name, passed | None, detail). None = undecidable., Load every *_summary.json under a dir (or matching a glob), tagged by name. (+4 more)

### Community 36 - "Community 36"
Cohesion: 0.17
Nodes (14): _dominant_bounds(), generate(), main(), parse_args(), _rasterize(), Bounds (xmin,ymin,xmax,ymax) of the spatially largest connected cluster     of s, Segments -> (occupied bool grid [iy,ix], origin (xmin,ymin)). Segments     outsi, World-space plane-crossing segments [(x0,y0,x1,y1), ...] from every     visible (+6 more)

### Community 37 - "Community 37"
Cohesion: 0.14
Nodes (13): camera, comment, cx, cy, depth_hfov_deg, depth_vfov_deg, fps, fx (+5 more)

### Community 38 - "Community 38"
Cohesion: 0.23
Nodes (7): lidar_offset(), main(), point_in_poly(), poly_extent(), Static front/rear lidar pose in base_link, per docs/isaac/robot-model.md     (co, Standard ray-crossing test; polygon is a list of geometry_msgs Point32., StallProbe

### Community 39 - "Community 39"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 40 - "Community 40"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 41 - "Community 41"
Cohesion: 0.15
Nodes (12): aborted, aborts_genuine, aborts_preempted, canceled, coverage_peak_pct, frontier_peak_black, frontiers_at_end, avail (+4 more)

### Community 42 - "Community 42"
Cohesion: 0.22
Nodes (5): Demo, main(), save_camera(), save_map(), save_scan()

### Community 44 - "Community 44"
Cohesion: 0.27
Nodes (10): main(), parse_args(), run(), attach_camera(), attach_lidars(), _find_prim_by_name(), Runtime ROS wiring for the robot package's GPU sensor prims (P4).  The sensor PR, Bind one render product + color/depth/points/camera_info publishers     to the b (+2 more)

### Community 45 - "Community 45"
Cohesion: 0.22
Nodes (7): frame_camera(), place(), Point the viewport at the robot from a three-quarter view.      The camera-to-wo, Reference an asset under a plain offset parent.      The referenced layers autho, set_approximation(), Prim, Path

### Community 47 - "Community 47"
Cohesion: 0.31
Nodes (7): bin_degrees(), Contract tests for the Isaac RTX-lidar scan assembler (no rclpy needed).  Covers, Each masked sector must be covered by the opposite, back-to-back unit,     so no, test_contract_scan_geometry_is_the_ust_10lx_window(), test_edge_mask_covers_the_measured_phantom_band_on_both_ends(), test_edge_mask_is_symmetric_and_only_touches_the_arc_ends(), test_masked_sector_is_inside_the_other_lidar_arc()

### Community 48 - "Community 48"
Cohesion: 0.39
Nodes (6): _body_vel(), main(), Step n render frames, refreshing cmd every frame like teleop would., run(), run_battery(), _step_frames()

### Community 49 - "Community 49"
Cohesion: 0.36
Nodes (7): cast(), main(), Nearest segment hit per bearing. segs: (n,4) array., Render-purpose, visible Mesh prims — what the RTX lidar can hit.      Excludes c, Plane-crossing segments [(x0,y0,x1,y1)] for one mesh, in base_link., slice_prim(), visible_render_meshes()

### Community 50 - "Community 50"
Cohesion: 0.32
Nodes (7): _candidate_dirs(), get_assets_root(), World-name resolution for the Isaac Sim runner.  Keeps the gz-era operator contr, The configured NVIDIA asset root (S3/Nucleus), or None if unavailable.      Stoc, Return a loadable stage path/URL for a world name or explicit path., resolve_world(), Path

### Community 51 - "Community 51"
Cohesion: 0.48
Nodes (6): cast(), main(), parse_args(), {prim_path: [(x0,y0,x1,y1)]} for segments within radius of (cx,cy).      Mirrors, run(), slice_near()

### Community 52 - "Community 52"
Cohesion: 0.70
Nodes (4): fetch(), main(), parse_args(), Path

### Community 53 - "Community 53"
Cohesion: 0.50
Nodes (3): start_exploration.sh script, RMW_IMPLEMENTATION, ROS_DOMAIN_ID

### Community 54 - "Community 54"
Cohesion: 0.83
Nodes (3): mcp_query.sh script, _data(), _post()

## Knowledge Gaps
- **463 isolated node(s):** `PreToolUse`, `PreToolUse`, `isaac-sim-mcp`, `build_and_start_expl.sh script`, `tag` (+458 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **18 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Time` connect `Community 3` to `Community 0`, `Community 33`, `Community 5`, `Community 6`, `Community 42`, `Community 44`, `Community 13`, `Community 28`, `Community 31`?**
  _High betweenness centrality (0.102) - this node is a cross-community bridge._
- **Why does `FrontierExplorerNode` connect `Community 5` to `Community 3`?**
  _High betweenness centrality (0.069) - this node is a cross-community bridge._
- **Why does `G1DistanceBenchmarkRunner` connect `Community 0` to `Community 3`, `Community 15`?**
  _High betweenness centrality (0.061) - this node is a cross-community bridge._
- **Are the 10 inferred relationships involving `BenchmarkCollageRenderer` (e.g. with `G1DistanceBenchmarkRunner` and `.__init__()`) actually correct?**
  _`BenchmarkCollageRenderer` has 10 INFERRED edges - model-reasoned connections that need verification._
- **What connects `PreToolUse`, `PreToolUse`, `isaac-sim-mcp` to the rest of the system?**
  _571 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.05240549828178694 - nodes in this community are weakly interconnected._
- **Should `Community 1` be split into smaller, more focused modules?**
  _Cohesion score 0.025974025974025976 - nodes in this community are weakly interconnected._