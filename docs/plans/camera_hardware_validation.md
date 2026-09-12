# D455 hardware validation plan

Status: **pending physical deployment validation.** The camera input contract is
implemented, but neither parser tests nor simulation prove the robot-side device,
topics, timing, TF ownership, or application behaviour. Coordinate physical
access and obtain authorization before changing a robot-side setup. Do not
overwrite `~/clearpath/robot.yaml` wholesale.

This is a validation assignment. A failed gate identifies a precise follow-up;
it does not authorize a speculative product-code, driver, QoS, TF, or launch
change during the evidence run.

## Current preflight findings

Resolve these explicitly before execution:

1. The checked-in `clearpath/robot.yaml` selects `rmw_cyclonedds_cpp`, matching
   the repository's approved runtime direction. The robot-side setup must still
   receive and approve that one-line configuration change before validation;
   then prove every participant uses the intended ROS domain and RMW.
2. The camera configuration uses `serial_no: "0"`. That does not prove which
   physical D455 the driver selected. Enumerate the connected device, record its
   serial, and pin or otherwise prove selection through an approved config diff.
3. `ridgeback_exploration.launch.py` is a simulation entrypoint and always starts
   Gazebo. Do not run it on the robot as the hardware smoke. Use the robot's
   approved Clearpath hardware bringup plus the existing perception nodes
   directly for the first validation. A reusable hardware localization launch
   is a separate implementation decision after the raw camera contract passes.

## Scope and non-goals

Required scope:

- D455 identity and exclusive driver ownership;
- effective driver configuration and active profiles;
- color, color `CameraInfo`, and aligned-depth topic contracts;
- stamp delivery and exact color/depth matching under CycloneDDS;
- camera-internal TF ownership and color-optical-to-base availability;
- a minimal application smoke using the two aligned-depth estimators.

Not part of the required pass:

- camera-to-LiDAR calibration;
- enabling or accepting the optional organized point cloud;
- changing estimator recipes, thresholds, exact-stamp semantics, or scoring;
- claiming estimator accuracy from a visual overlay;
- implementing a hardware launch file or fixing a failed gate without separate
  approval.

## Phase 0: freeze provenance and evidence storage

Create one ignored artifact root such as
`artifacts/hardware/d455_<YYYYMMDD_HHMMSS>/`. Record before launch:

- repository revision and worktree status;
- installed workspace paths and package versions;
- Clearpath dependency revisions;
- effective robot-side `setup_path` and a checksum/copy of the approved config;
- the exact repository-to-robot configuration diff;
- ROS domain, `RMW_IMPLEMENTATION`, host/kernel, USB topology, and load;
- D455 firmware and `realsense2_camera` version.

Keep at least these evidence groups: `provenance`, `device`, `parameters`,
`topics`, `stamps`, `tf`, `rosbag`, `application`, and `logs`. Raw bags may stay
outside Git; record their path, duration, size, and checksum in the completion
document.

Stop if physical access, safety authorization, the robot-side config diff, or
exclusive camera ownership is unavailable.

## Phase 1: prove identity, ownership, and effective configuration

1. Enumerate RealSense devices with the vendor tooling and record product,
   product line, serial, firmware, USB mode, and advertised sensors.
2. Start the approved Clearpath hardware bringup with CycloneDDS and
   `use_sim_time:=false`.
3. Record the camera node's fully qualified name, publishers, subscriptions,
   effective parameter dump, and process ownership.
4. Confirm the resolved configuration, rather than only source YAML, includes:
   `device_type: d455`, the intended serial, color and depth enabled,
   `align_depth.enable: true`, and `enable_sync: true`.
5. Record the active color/depth profiles the driver reports. Do not infer them
   from the checked-in defaults.

Pass requires exactly one intended D455, one driver owner, selection of the
recorded serial, and the required alignment/synchronization parameters. A
fallback to another device, duplicate driver, USB downgrade that changes the
profile, or remaining Fast DDS participant fails this phase.

## Phase 2: prove the stream and timing contract

After a warm-up period, record at least 60 seconds of:

- `sensors/camera_0/color/image_raw`;
- `sensors/camera_0/color/camera_info`;
- `sensors/camera_0/aligned_depth_to_color/image_raw`;
- `/tf` and `/tf_static`.

For each camera topic, record its resolved absolute name, type, publisher,
subscriber set, offered/requested QoS, rate, bandwidth, encoding, width, height,
`frame_id`, and header stamps. Record the color `CameraInfo` distortion model,
dimensions, `K`, `D`, `R`, and `P`.

Analyze the steady-state color and aligned-depth stamp sets, excluding the
declared warm-up interval. Report:

- sample counts and effective rates;
- exact intersection and color-only/depth-only stamps;
- exact-match percentage;
- duplicate, regressing, and out-of-order stamps;
- longest receive gap and any middleware lost-message reports;
- observer effect from recording or additional subscribers.

Proposed pass criteria, to freeze before collecting data:

- every aligned-depth image has the same grid as color and color `CameraInfo`;
- encodings and units are supported by the current source contract;
- stamps are monotonic with no duplicates or regressions;
- steady-state raw color/aligned-depth exact matching is at least 99%; a lower
  result is a failed transport/driver contract requiring diagnosis;
- Phase 4 must still show 100% exact depth availability for every processed
  detection stamp. Raw-stream percentage does not waive that application gate.

Do not rewrite timestamps, widen matching tolerance, or substitute nearest
frames to make this phase pass.

## Phase 3: prove TF ownership

Capture the TF graph and directly query the transform from the observed color
optical frame to `r100_0001/robot/base_link`. Record both static and dynamic TF
publishers and the transform values.

Pass requires:

- the hardware RealSense driver is the sole owner of camera-internal calibrated
  frames;
- no simulation nominal camera chain or static compatibility publisher exists;
- the color optical frame has a stable, finite transform to the configured base
  frame;
- frame IDs on color, aligned depth, and `CameraInfo` agree with the TF graph.

The nominal mount may be used as a sanity check, not as proof of camera-to-LiDAR
calibration. LiDAR integration remains owned by the separate
[calibration backlog](../BACKLOG.md#camera-lidar-calibration).

## Phase 4: minimal target-localization smoke

Keep the approved hardware bringup running and start the existing detector and
mask-measurement nodes without the simulation launch. Freeze these settings:

- CycloneDDS and the robot's ROS domain;
- `use_sim_time:=false`;
- RealSense color, color `CameraInfo`, and aligned-depth topics from Phase 2;
- `depth_source:=stereoscopic`;
- `mask_gate:=box`;
- `enabled_estimators:=projective_ranging,euclidean_reconstruction`;
- namespaced `base_frame:=r100_0001/robot/base_link`;
- `depth_match_debug:=true` for bounded stamp/timing evidence;
- pointcloud and polar estimators disabled.

Predeclare measured near, middle, and farther target positions that fit the
approved space and device profile. Include one off-centre/image-edge position.
Record target placement method, duration, camera motion, host load, detections,
mask measurements, status histograms, depth diagnostics, and logs. Overlay/HUD
images may corroborate the run but never replace topic evidence.

Required pass:

- detection and mask-measurement topics remain live;
- every processed detection stamp has its exact aligned-depth frame;
- zero `NO_DEPTH_FRAME`, `NO_CAMERA_INFO`, `GRID_MISMATCH`, and
  `TF_MISS_EXTRINSIC` results;
- both estimators produce finite, plausible measurements whenever the detector
  identifies the target;
- no repeated driver restart, DDS loss, callback exception, or stale-frame
  warning occurs.

Distance comparisons here are sanity checks. They do not establish a calibrated
accuracy claim. If the target GPU supports SlimSAM, run `mask_gate:=silhouette`
as a separately labelled optional smoke only after box mode passes.

## Phase 5: optional organized-pointcloud gate

Leave the RealSense pointcloud unwired unless there is an approved need. If it
is evaluated, prove before launching the pointcloud estimator:

- `PointCloud2.height > 1` and dimensions correspond to the color grid;
- row-major point ordering is indexable by color pixel;
- the cloud frame and TF path are correct;
- timestamps are compatible with the color/detection contract;
- invalid-point representation and density are recorded.

Only after all checks pass may a RealSense pointcloud input be proposed. Failure
does not fail the required D455 color/aligned-depth validation unless pointcloud
support was explicitly made part of the deployment acceptance criteria.

## Stop conditions and failure ownership

Stop and record the exact failed boundary when any required gate fails. Examples:

- wrong/ambiguous serial or competing device owner: deployment configuration;
- expected aligned-depth topic absent: driver/effective configuration;
- grid mismatch: driver profile/alignment configuration;
- raw stamps missing before application startup: driver/DDS delivery;
- raw stamps present but absent from the mask callback: subscriber/DDS boundary;
- transform absent or multiply owned: robot description/driver TF ownership;
- streams and TF pass but measurements fail: target-localization integration.

Do not proceed to downstream application or pointcloud checks to obscure an
upstream failure. Any fix is a separately approved task followed by a rerun from
the failed phase and the relevant earlier control.

## Completion record and backlog decision

Write a dated `docs/history/d455_hardware_validation_<YYYY-MM-DD>.md` containing:

- commands and frozen provenance;
- device identity and effective parameter evidence;
- topic/QoS/profile/grid/encoding/rate tables;
- stamp-set analysis and application status histograms;
- TF graph, ownership, and optical-to-base transform;
- required and optional smoke results;
- artifact paths/checksums and observer limitations;
- every check not run, failed, or unavailable.

Update current technical references only with demonstrated facts; do not turn
datasheet specifications or recommended ranges into runtime guarantees. Remove
the physical-camera backlog item only if Phases 0--4 pass. Otherwise narrow it
to the precise unresolved boundary and retain the evidence needed for the next
task.
