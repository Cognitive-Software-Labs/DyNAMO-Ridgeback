# Documentation-backed engineering backlog

Only active, evidenced gaps belong here. Implemented behaviour belongs in the
technical references; completed experiments and migrations belong in history;
unselected alternatives belong in `docs/do_not_try_again/`. Closing an item means satisfying
its completion criteria and moving durable results to the relevant reference or
history document—not retaining a crossed-out entry here.

## Isaac stock-world seating and ground-truth map regeneration

**Gap.** The robot is seated correctly in `mock_hospital`, whose floor top is
at z = 0.05 m, but floats 49.8 mm above every stock Isaac world because the
runner's fixed `spawn_z=0.076` assumes that raised floor. The drive rig has no
vertical degree of freedom and the wheels have no colliders, so physics cannot
settle it. The stock-world maps are internally consistent with the floating
robot, but their 0.3024 m slice plane is 49.8 mm above the physically seated
UST-10LX plane.

**Completion criteria.** Derive each world's spawn height from its floor level
using the measured wheel-mesh bottom: `spawn_z = floor_z + 0.02617`. Derive the
map slice from the same source as `floor_z + 0.25257`; do not introduce a second
independent constant. Verify wheel contact height and scan height in
`mock_hospital` and at least one stock world, then regenerate all affected
stock-world maps and previews once.

**Context.** [Isaac robot model](isaac/robot-model.md), the
[ground-truth map runbook](../src/ridgeback_autonomy/sim/ground_truth_maps/README.md),
and the original phase evidence in the [Isaac port plan](isaac/port-plan.md).

## Isaac lidar and exploration recertification

**Gap.** No statistically valid Isaac exploration baseline exists for the
current robot. The measured sensor geometry changed repeatedly on 2026-09-10,
and the 2026-09-11 lidar reparent fixed an orphan-body defect that left the
emitters stationary while the chassis rotated. All earlier coverage and SLAM
quality numbers are therefore void. The front/rear merged-scan path also still
needs a fresh-boot live validation after its TF-remap and range-bound fixes.

**Completion criteria.** After the seating and map regeneration above, validate
the raw 270-degree scans and the merged SLAM scan from a clean Isaac 6.1 boot,
first with zero odometry noise and then with the configured noise. Re-measure
SLAM quality, confirm the front/rear stamp and motion-compensation behavior,
and run 3–5 seeds per comparison condition on a suitably quiet host. P5 passes
only when 3/3 runs complete, coverage is at least the Gazebo mean minus 10
points, genuine aborts do not exceed the Gazebo maximum, and throttled headless
RTF is at least 0.8.

**Context.** [Exploration benchmark runbook](exploration/benchmarking.md),
[Isaac lidar pipeline](isaac/lidar-pipeline.md), and the resolved orphan-lidar
investigation in [Isaac port history](isaac/port-history.md#lidars-detached-from-the-articulation-2026-09-11).

## Isaac hull-collider validation

**Gap.** The vendor chassis collider changed from an AABB cube that overclaimed
volume by 25.4% to a `convexHull` that overclaims it by 6.9%. Static inspection
and lidar self-occlusion checks passed, but no live contact test has driven the
robot into representative walls to prove that the new hull stops at the right
place without instability or tunnelling.

**Completion criteria.** Run controlled low-speed frontal, lateral, and angled
contacts against known geometry. Compare the observed stop pose with the
rendered hull and collider, verify stable contacts and recovery, and record the
accepted tolerance in the robot-model reference.

**Context.** [Isaac robot model](isaac/robot-model.md#colliders) and the
[vendor-chassis evidence](isaac/port-history.md#vendor-chassis-graft-2026-09-10).

## Isaac documentation lifecycle cleanup

**Gap.** The Isaac documentation still mixes current contracts with port-era
investigation and completed migration material. In particular,
`lidar-pipeline.md` is indexed as a current reference but is structured as a
dated Isaac 6.0 investigation; `port-plan.md` contains both the active phase
gate and extensive completed-phase history; and `migration-6.1.md` combines a
completed execution record with still-useful rollback instructions.

**Completion criteria.** Make `lidar-pipeline.md` describe only the current
Isaac 6.1 lidar-to-ROS contract and live verification procedure, moving dated
bug evidence to history. Reduce `port-plan.md` to unfinished gates or mark it
entirely historical. Split the completed 6.1 migration evidence from any live
rollback/runbook material. Update the index and cross-links, and add an
automated internal Markdown-link check so deleted or moved owners cannot leave
dangling references.

**Context.** [Documentation ownership](project/documentation.md), the
[Isaac document map](isaac/README.md), and [Isaac port history](isaac/port-history.md).

## Physical command-chain validation

**Deployment gate.** The hardware adapter is attach-only and autonomous motion
defaults off, but the repository has not proven the deployed base controller's
velocity message contract. The generated setup currently mixes stamped Nav2,
velocity-smoother, collision-monitor, and mux commands with a generated R100
mecanum controller setting that reports `use_stamped_vel: false`.

**Completion criteria.** On the exact approved robot configuration, trace the
message type, topic, QoS, frame/timeout semantics, mux ownership, collision
monitor, controller input, e-stop, and deadman behavior from Nav2 to the base.
Resolve any mismatch at the narrowest owning layer; then perform stationary,
lifted/blocked-wheel where approved, low-speed clear-space, stop, timeout, and
e-stop tests before enabling `autonomous_motion_enabled:=true`.

**Context.** [Exploration architecture](exploration/architecture.md#hardware-safety-boundary)
and the public hardware workflow in the [README](../README.md).

## Gazebo collision-stack parity for the measured lidar mounts

**Gap.** The shared robot description now carries the physically measured,
coplanar UST-10LX positions and explicit ±135° scan windows. Gazebo accepts
them, preserves their poses in the reduced SDF, and produces clean stationary
and moving raw scans. The full `collision_monitor` and SLAM consumers still
need an integration run with that description.

**Completion criteria.** Verify `collision_monitor` does not stop clear-space
motion and that SLAM still receives the selected front scan. If behavior fails,
implement a Gazebo-only collision-mask correction while keeping
`clearpath/robot.yaml`, the ±135° aperture, and published TF at the measured
poses. Repeat the scan, motion, and camera topic gates after the correction.

**Context.** [Isaac robot and sensor model](isaac/robot-model.md) and
`test_camera_description.py`'s fixed-joint-reduction guard.

## Physical camera validation

**Gap.** The repository selects a D455 and has parser/description tests, but no
recorded robot-side proof of the actual device, active profiles, topics, grids,
stamps, TF ownership, or optional organized cloud.

**Completion criteria.** Execute the [hardware plan](plans/camera_hardware_validation.md)
against the approved robot setup; validate 640x480@30 and 1280x720@30 in
separate runs; record device identity, effective configuration,
color/depth/camera-info topics, dimensions/encodings/rates/stamps, exact-match
observations, and optical-to-base TF; verify there is one internal-camera TF
owner. Compare each live `CameraInfo` with the nominal shared simulation
profile and record an explicit keep/update decision. If enabling `pointcloud`,
additionally prove organization, color-grid indexing, frame, and timestamps
before wiring it as a RealSense input.

**Context.** [Aligned depth](target_localization/aligned_depth.md), the
[removed D435 transform](history/operational_incidents.md#d435-static-camera-transform--removed-2026-08-31),
and `common/camera_inputs.py`.

## Camera-LiDAR calibration

**Deployment gate.** Resolve this before treating polar-profiling measurements
from the physical robot as valid. Hardware bring-up and data collection may
precede it, but deployment must not rely on camera-to-LiDAR projection results
until the completion criteria below have been satisfied.

**Gap.** Simulation has exact model transforms; a documented physical
camera-to-LiDAR calibration and clock validation is absent.

**Why it matters.** Polar mask membership projects a free-running scan through
that transform. Spatial or temporal error can select the wrong beams.

**Completion criteria.** Define and execute a repeatable target/measurement
procedure; record transform convention, uncertainty, storage/ownership, and
timestamp evidence; validate projected beams on independent poses/ranges and
under robot motion; document recalibration triggers. Do not silently tune polar
segmentation to absorb an extrinsic error.

**Context.** [Polar profiling](target_localization/polar_profiling.md) and the
[hardware plan](plans/camera_hardware_validation.md).

## Camera-geometry benchmark recertification

**Gap.** The current measured camera and LiDAR mounts supersede the geometry
used to generate the visibility fractions and pixel-grid certifications in
`benchmark_scenarios_full.yaml`. The scenes remain a stable A/B input set, but
their old certified fractions are not current evidence.

**Completion criteria.** Regenerate the pixel/visibility audit from the current
robot description for both shared camera profiles and verify Gazebo/Isaac
agreement; update scenario certifications and the gallery; rerun every
benchmark whose inputs or measured behavior depend on camera or LiDAR geometry
before quoting its numbers.

**Context.** [Camera stack](target_localization/camera_stack.md) and
[Isaac robot and sensor model](isaac/robot-model.md).

## Isolation validation

**Gap.** 2D/3D recipes and current defaults are implemented, but the
2026-08-28 default/depth-gate transition and cross-recipe accuracy have not been
established by a controlled current comparison, especially with deep backgrounds.

**Completion criteria.** Fix one scenario YAML and revision; include clutter,
near occluders, long backgrounds, ranges, and viewpoints; compare registered
recipes and box/silhouette controls without regenerating scenes; record accuracy,
misses/coverage, runtime, memory, and parameters; include exploration only as a
qualitative follow-up. Treat a default change as a separate decision.

**Context.** [Projective-ranging isolation](target_localization/projective_ranging.md#implemented-2d-recipes),
[euclidean-reconstruction isolation](target_localization/euclidean_reconstruction.md#implemented-3d-recipes),
and the [candidate protocol](do_not_try_again/foreground_isolation.md#evaluation-protocol).

## Occlusion characterization

**Gap.** Dedicated `interocc_*`, `objocc_*`, and `objpartial_*` scenes exist,
but current defaults have not first been characterized well enough to select a
recovery design. The [depth-clustering plan](plans/occlusion_handling.md) is a
proposal, not an approved algorithm or current miss reason.

**Completion criteria.** Run current defaults on the fixed occlusion families;
separate detector, segmentation, depth/scan availability, isolation, association,
and wrong-near estimates; identify which estimator/precision/scenes require
change; decide whether the failure merits a new recipe, a wrong-object guard, an
explicit occlusion reason, or no change. Only then approve a scoped implementation
and require non-occluded regression evidence.

**Context.** [Occlusion proposal](plans/occlusion_handling.md),
[segmentation candidates](do_not_try_again/segmentation.md), and the
[scenario gallery](benchmarking/benchmark_scenarios_v2_gallery.html).
