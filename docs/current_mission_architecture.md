# Current mission architecture

## Authoritative entrypoint

The complete thesis mission is launched with:

```bash
ros2 launch ridgeback_autonomy ridgeback_mission.launch.py
```

`mission_manager_node` is the sole parking owner. The older
`humanoid_parking_node` remains installable for historical/debug use but is not
started by a public launch. `ridgeback_exploration.launch.py` runs exploration
and perception only.

## Active stack

The canonical launch starts Gazebo with the `park_robot` world, SLAM Toolbox in
mapping mode, Nav2, the in-repository frontier explorer, OWLv2 G1 detection,
camera and LiDAR measurement nodes, the perception overlay/estimate marker
nodes, `mission_manager_node`, and the clean thesis RViz configuration.

Important names under `/r100_0001` include:

- RGB: `sensors/camera_0/color/image`
- depth: `sensors/camera_0/depth/image`
- organized cloud: `sensors/camera_0/points`
- detections: `detections/g1/raw`
- camera measurements: `measurements/g1/camera`
- map: `map`
- exploration control: `explore/set_enabled`
- exploration ownership status: `explore/active`
- Nav2 actions: `compute_path_to_pose` and `navigate_to_pose`
- Nav2 plan: `plan`

All mission nodes use simulated time and the namespaced TF topics. Parking
geometry is expressed in `map`; the robot frame is `base_link` with +x forward
and +y left.

## State sequence

The normal path is:

```text
WAITING_FOR_START_POSE
  -> SEARCH_G1
  -> STOPPING_EXPLORATION
  -> LOCALIZE_G1
  -> TRANSFORM_G1_TO_MAP
  -> G1_LOCALIZED
  -> DONUT_READY
  -> DONUT_CLASSIFIED
  -> FOOTPRINT_CHECKED
  -> PARKING_POSES_VALIDATED
  -> NAV2_FEASIBILITY_CHECKED
  -> PARKING_POSE_SELECTED
  -> NAVIGATING_TO_PREPARKING_POSE
  -> NAVIGATING_TO_PARKING_POSE
  -> PARKED
  -> RETURNING_TO_START
  -> MISSION_COMPLETE
```

The initial `map -> base_link` pose is captured once and never overwritten.
G1 is confirmed by the existing three-positive-votes-in-five-results window.
Confirmation commits the mission: frontier exploration is disabled and its
active Nav2 goal must reach a terminal result before mission navigation begins.
The canonical mission never re-enables exploration after commitment.

A bounded side-observation branch remains available if direct localization
cannot produce a trustworthy position. It uses map/footprint/path checks and
does not resume frontier exploration after commitment. Expected environmental
or perception failures are logged distinctly; an empty safe candidate set
enters `MISSION_FAILED` with the robot stopped.

## G1 localization

The preferred estimate is the lower-leg/foot midpoint. A sample is accepted
only when both clusters have enough points, balanced support, 0.18-0.35 m
separation, and agreement with the coarse camera estimate. Five accepted
samples are transformed at their measurement timestamps into `map` and combined
with a median.

If reliable feet are unavailable, the RGB-D body fallback uses the central
35-65% of bbox width and 25-60% of bbox height. It rejects invalid points,
keeps base-frame heights from 0.60-1.80 m, trims range/lateral outliers, takes a
3D median, transforms the measurement into `map`, checks it against the coarse
G1 estimate, rejects jumps over 0.75 m, and stabilizes five map-frame samples.
Detection-to-depth/cloud age is limited to 0.50 s, and camera intrinsics are
scaled if the depth resolution differs from CameraInfo.

The body sources are attempted in this order: conventionally transformed
organized cloud, aligned depth back-projection, then the existing simulation
fallback for Gazebo clouds whose numeric axes are camera-link-like despite an
optical-frame header. The final source is logged as `foot_midpoint` or
`depth_body_projection`. Gazebo model truth is never used for mission control.

## Reachability and candidate geometry

Reachability is a simplified planar annulus, not a G1 FK/IK or capability map:

- minimum reach: 0.30 m
- maximum reach: 0.60 m
- preferred reach: 0.45 m

The task point is `(-0.280, 0.000)` m in `base_link`. Candidate base poses are
derived so this rear task point lies in the annulus. The base +x direction
points away from G1, leaving the rear task point toward G1. Pre-parking is
0.50 m outward along the final base +x direction with the same yaw.

RViz shows the annulus, its inner/outer boundaries, and the estimated G1 center.
Free task cells are green. Occupied, unknown, footprint-invalid, out-of-map, and
path-invalid task cells are overlaid in red shades. The selected final footprint,
task point, task offset, and pre-parking footprint remain in the clean thesis
view.

## Safety filtering

The authoritative rectangular Ridgeback footprint is loaded from
`config/nav2_params.yaml`; mission code does not maintain a second polygon.
Each task point passes these stages:

1. task point lies in a free map cell inside the annulus;
2. task point is converted to a base pose;
3. the complete rotated footprint is rejected on occupied, unknown, or
   out-of-bounds map cells;
4. obstacle/map-boundary clearance is computed for valid footprints;
5. Nav2 `ComputePathToPose` must succeed with no error and a non-empty path;
6. the returned path is sampled at no more than map resolution/0.10 m spacing,
   and the complete footprint must remain in known free map space at every
   sample.

NavFn keeps `allow_unknown: true` for frontier exploration. The stricter
mission-level path check prevents parking from relying on that permissive
exploration setting.

## Ranking and execution

There is no reliable sensor-derived G1 yaw or hand identity. True humanoid
left/right classification is therefore not implemented. When G1 orientation
is unavailable, candidates are grouped relative to the observed approach
direction: lateral candidates first, then approach-front, then approach-back.
Within the best available class, selection favors preferred reach, greater
clearance, shorter path, and deterministic input order.

The selected pre-parking and final poses are sent through Nav2. Success is
measured against runtime `map -> base_link`. The robot waits 5 seconds after
parking, then navigates to the exact saved start pose and completes the mission.

## Key parameters

- `reach_min_m`, `reach_max_m`
- `task_point_x_base_m`, `task_point_y_base_m`
- `preparking_offset_m`, `parking_wait_duration_s`
- `occupied_threshold`, `parking_path_sample_spacing_m`
- `g1_sensor_max_age_s`
- foot cluster point, balance, separation, and consistency thresholds
- body ROI, height, jump, and sample-count thresholds
- localization/observation timeouts

## Known limitations

- G1 orientation and hand identity are not estimated reliably.
- Reachability is a planar distance model; G1 FK/IK is not implemented.
- The Gazebo organized-cloud frame mismatch still requires a simulation-only
  last-resort interpretation; aligned depth is preferred ahead of it.
- Depth is used for G1 localization, not as a Nav2 costmap source.
- Full end-to-end behavior remains sensitive to detector visibility, SLAM map
  quality, and simulation timing; successful compilation/unit tests are not a
  substitute for a complete Gazebo mission run.
