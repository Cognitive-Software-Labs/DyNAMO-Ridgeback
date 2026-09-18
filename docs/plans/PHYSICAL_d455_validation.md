# PHYSICAL — D455 camera validation

Status: **pending physical access and validation.** The
[project backlog](../BACKLOG.md#physical-camera-validation) owns this deployment
gate. This plan owns the execution procedure; the [camera reference](../target_localization/camera_stack.md)
owns the implemented input contract, nominal profiles, and TF ownership.
Passing establishes integration readiness for the tested device and profiles.
Calibrated accuracy, camera–LiDAR calibration, and motion validation are separate.

## Preparation and evidence

Coordinate physical access and permission for any robot-side configuration
changes. Inspect the deployed setup before proposing a diff; the checked-in
CycloneDDS setting does not establish what is running on the robot or imply
that it still needs changing. Preserve the robot's configuration and services.
Do not overwrite its setup from the repository or run simulator cleanup.

Record code/installed-workspace/dependency revisions, effective setup path and
configuration, ROS domain/RMW, host load, USB topology, device firmware, driver
version, commands, and any applied configuration diff. Use a unique artifact
root under `artifacts/hardware/` and preserve logs, topic captures, parameter
dumps, timestamps, and TF evidence with checksums.

Freeze the tested profiles, target placements, warm-up interval, recording
window, stream-rate/gap criteria, and timestamp acceptance criteria before
collecting results. Keep the robot stationary, autonomous motion disabled, and
platform/sensor bringup owned by its existing approved services. A failed gate
should identify a specific follow-up; do not tune recipes, alter timestamps,
or change drivers/TF during the evidence run to turn it into a pass.

## 1. Device identity and effective configuration

- Enumerate the connected cameras. Record the intended D455 serial, firmware,
  USB mode, and advertised profiles; `serial_no: "0"` alone proves no identity.
- Identify the camera process/node and confirm one driver owns the intended
  device. Verify the participating stack's domain, RMW, and wall-time settings.
- Record effective parameters for colour/depth enablement, alignment, and
  synchronization. Inspect active driver profiles rather than inferring them
  from repository YAML.
- Request `640x480@30` and `1280x720@30` as separate runs. Record the actual
  colour and native-depth profiles and resulting aligned-depth grid. Unsupported
  requests or fallback profiles are unresolved findings, not successful tests
  of the requested profile.

Pass requires unambiguous device selection, exclusive ownership, and the frozen
configuration actually running. Propose only the configuration changes the
inspection demonstrates are necessary.

## 2. Image, depth, and calibration contract

Discover resolved colour, colour `CameraInfo`, and colour-aligned depth topics
from the running graph. Compare them with the configured application inputs.
For each topic record type, publisher/subscriber ownership, QoS, encoding,
units, dimensions, `frame_id`, rate, and bandwidth. Preserve live `K`, `D`, `R`,
and `P` calibration fields for each profile.

Pass requires supported encodings/units, compatible colour/aligned-depth/
calibration grids, and the expected publishing ownership. Compare the observed
calibration with the nominal simulation profiles and record a keep/update
recommendation; one device's calibration does not automatically redefine the
shared simulation model.

## 3. Timing and exact-frame availability

After the frozen warm-up, capture at least 60 seconds per profile using
QoS-compatible observers. Preserve colour and aligned-depth header stamp sets
and the resolved TF topics. Report received counts, unique stamps, duplicates,
regressions, out-of-order delivery, receive gaps, middleware loss reports, and
recording/subscriber overhead.

For unique colour stamps `C` and aligned-depth stamps `D`, report both
`|C ∩ D| / |C|` and `|C ∩ D| / |D|`. An empty stream fails the stream gate;
it does not receive a perfect matching score. Use the same declared steady-state
capture interval and document boundary handling.

The previous 99% raw-match target is a **proposed acceptance threshold** for
both directions, to confirm before measurement. It is not an observed device
guarantee. Preserve no-duplicate/no-regression criteria and the predeclared
rate/gap requirements; diagnose observer losses before attributing them to the
producer. The application smoke still requires exact depth availability for
every processed detection stamp, regardless of the raw-stream percentage.

Do not rewrite timestamps, widen matching tolerance, or substitute nearby frames.

## 4. Transform ownership and availability

Inspect the live TF graph, including namespaced topics, and record its publishers.
Query the observed colour-optical frame to the base frame the application
actually requests. The public exploration launch currently derives that frame
as `<namespace>/robot/base_link`; a ROS topic namespace alone does not establish
that the frame exists. Record a mismatch as an integration failure.

Pass requires a finite, stable camera-to-base transform and one owner of the
camera's internal calibrated transforms: the hardware RealSense driver. Verify
image/calibration frame IDs against the live graph and reject competing nominal
simulation or compatibility publishers. The configured mount is a sanity check;
it does not prove camera–LiDAR calibration.

## 5. Stationary application smoke

Attach the shared stack to the existing hardware services using
`backend:=hardware autonomous_motion_enabled:=false start_hardware_platform:=false`.
Use the public launch argument
`estimators:=projective_ranging,euclidean_reconstruction`, with
`depth_source:=stereoscopic mask_gate:=box depth_match_debug:=true`.
Use the device's domain, wall time, and the verified camera input topics.
Keep pointcloud and polar estimators out of this smoke.

Use predeclared visible, in-range targets at near, middle, farther, and one
off-centre position. Record target placement, duration, detections, measurements,
status histograms, exact-depth diagnostics, and host load. Images can illustrate
the run but cannot substitute for topic evidence.

Pass requires live detection/measurement topics, exact depth for every processed
detection, zero `NO_DEPTH_FRAME`, `NO_CAMERA_INFO`, `GRID_MISMATCH`, or
`TF_MISS_EXTRINSIC` results, and finite plausible outputs for the declared valid
target cases. Record driver restarts, callback failures, receive losses, and
stale-data warnings. These distance checks are integration sanity checks, not
an accuracy benchmark. Silhouette mode may be a separately labelled follow-up.

## Completion and separate work

Write a dated engineering record with provenance, profile results, calibration,
timing analysis, TF ownership, application outcomes, artifacts/checksums,
observer limitations, and every unavailable/failed check. Update the camera
reference only with demonstrated facts. Apply the project backlog's completion
criteria; unsupported profiles or failed checks remain explicit open work.

[Organized-pointcloud qualification](../BACKLOG.md#optional-physical-organized-pointcloud-qualification)
and [camera–LiDAR calibration](../BACKLOG.md#camera-lidar-calibration) are separate
items. Neither is required to claim completion of the core colour/aligned-depth
validation. A fix to a failed boundary requires a separately scoped change and
rerunning the affected checks.
