# D455 hardware validation plan

Status: pending physical deployment validation. Repository implementation is
complete; this plan must not be used to claim hardware readiness from simulation
or parser tests. Coordinate access and obtain authorization before changing a
robot-side setup. Do not overwrite `~/clearpath/robot.yaml` wholesale.

## Preconditions

- Record repository revision, installed workspace, Clearpath dependency
  revisions, the launch's effective `setup_path`, and the approved robot-side
  configuration diff.
- Ensure the intended physical device is connected and no competing process owns it.
- Preserve exact matching, driver-owned alignment, current stream-profile
  omission, camera mount, estimator defaults, and optional pointcloud policy.

## Validation procedure

1. **Identity/configuration:** record enumerated product and serial; prove
   `device_type: d455` and the configured `serial_no` select that device in the
   installed wrapper. Record resolved driver parameters, including alignment and
   synchronization.
2. **Streams:** record effective color/depth profiles, rates, encodings, frame IDs,
   and `CameraInfo`. Confirm `aligned_depth_to_color/image_raw` exists and its grid
   matches color. Compare color/depth header stamps and quantify exact matches.
3. **TF ownership:** confirm the driver is the only publisher of internal camera
   frames on hardware; record the camera-optical to configured base transform.
   Ensure the simulation-only model/render path is absent.
4. **Application smoke:** start the approved target-localization workflow and
   observe detections, mask measurements, statuses, overlay, HUD, and markers.
   Record both box and silhouette evidence if the target GPU supports the model.
   A visible UI without measurement topics is not a pass.
5. **Optional pointcloud:** leave unwired unless needed. If evaluated, prove
   `height > 1`, color-grid correspondence, ordering, frame, stamps, and
   estimator behaviour before defining a RealSense pointcloud input.
6. **LiDAR integration:** follow the separate
   [calibration backlog](../BACKLOG.md#camera-lidar-calibration); nominal URDF or
   visually plausible projection is insufficient.

## Completion record

Store commands, resolved parameters, topic/TF samples, dimensions/rates, logs,
and observed limitations in a dated history document. State explicitly which
checks were not run. Update current references only with demonstrated facts;
do not turn product specifications or recommended ranges into runtime guarantees.
