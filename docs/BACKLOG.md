# Documentation-backed engineering backlog

This backlog owns robot, exploration, deployment, and cross-cutting gaps.
The [target-distance benchmarking backlog](target_distance_benchmarking/BACKLOG.md)
owns benchmark tooling and controlled estimator-evaluation work. Each item
has one owner; cross-cutting prerequisites are linked rather than copied.

Only active, evidenced gaps belong here. Implemented behaviour belongs in the
technical references; dated experiments and migrations belong in the archive;
unselected alternatives belong in `docs/do_not_try_again/`. Closing an item means satisfying
its completion criteria and moving durable results to the relevant reference or
archival record, rather than retaining a crossed-out entry here.

## Isaac exploration recertification / P5

**Gap.** No statistically valid autonomous Isaac exploration baseline exists
for the current robot. The September 17–18, 2026 closed-loop qualification
passed its bounded gates; it did not qualify frontier exploration or supply a
matching Gazebo coverage comparison. Its noisy-seed tradeoffs motivate the
SLAM-source follow-up below. Earlier invalidated exploration numbers remain void.

**Completion criteria.** With corrected seating, regenerated maps, and the
qualified LiDAR pipeline, run autonomous exploration over 3–5 seeds per
comparison condition on a suitably quiet host. Establish a current matching
Gazebo baseline. P5 passes
only when 3/3 runs complete, coverage is at least the Gazebo mean minus 10
points, genuine aborts do not exceed the Gazebo maximum, and throttled headless
RTF is at least 0.8.

**SLAM-source follow-up.** Compare `front_only` and `merged` with matched
seeds at zero and configured noise during exploration. Investigate the noisy
seed-0 regression and larger EKF/SLAM yaw components using signed yaw traces
and total yaw error; component RMS values are not additive. Report per-seed
pose/map quality, coverage, aborts, and timing alongside aggregate results.
Keep `front_only` as the public default until a separate, evidence-backed
default decision addresses these tradeoffs and the exploration/P5 results.

**Context.** [Exploration evaluation](exploration/evaluation.md),
[Isaac lidar pipeline](isaac/lidar-pipeline.md), and the resolved orphan-lidar
investigation in the archived evidence below.

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

## Optional: restore Gazebo exploration worlds and backend-specific maps

**Opportunity.** Gazebo remains a supported backend, but the adapter currently
ships only `mock_hospital` for exploration and `target_distance_calibration`
for target benchmarking. The only Gazebo `mock_hospital`, `warehouse`, and
`office` coverage maps are archived under `historical/`; the latter two source
worlds are no longer present. Meanwhile the coverage overlay looks up
`<world>.pgm` without considering the backend. Restoring same-named Gazebo and
Isaac worlds under that contract could silently score one simulator against
the other simulator's geometry.

**Completion criteria.** Regenerate a complete analytical map set for the
current Gazebo `mock_hospital` from its SDF. Restore or re-import the exact
Gazebo `warehouse` and `office` world sources with recorded provenance before
promoting their maps; do not promote the archived captures on name alone.
Make map identity explicitly backend-qualified—for example,
`ground_truth_maps/gz/<world>` and `ground_truth_maps/isaac/<world>`—and pass
the selected backend into coverage-map resolution. Preserve convenient
operator world names, but treat `(backend, world)` as the unique identity in
storage, diagnostics, artifacts, and documentation. Add tests proving that a
Gazebo run cannot fall back to an Isaac map, or vice versa, even when both
worlds are named `warehouse` or `office`. Generate `.pgm`, `.yaml`, `.png`, and
`.npz` artifacts for every restored map and visually verify them against their
own simulator world.

**Context.** [Ground-truth map runbook](../src/ridgeback_autonomy/sim/ground_truth_maps/README.md),
[exploration evaluation runbook](exploration/evaluation.md), and the Gazebo
adapter worlds under `src/ridgeback_autonomy_gz/sim/worlds/`.

## Physical camera validation

**Gap.** The D455 input contract is implemented, but device identity, effective
profiles, colour/aligned-depth delivery, exact timestamps, TF ownership, and
application behavior still need proof on the deployed robot.

**Completion criteria.** Execute the [hardware validation procedure](plans/camera_hardware_validation.md):
verify the intended device and driver, inspect effective configuration, validate
image/depth/calibration contracts and timing, confirm the actual camera-to-base
TF chain, and run a stationary projective/Euclidean application smoke. Test
640x480@30 and 1280x720@30 separately; record unsupported/fallback profiles as
unresolved rather than silently accepting them. Freeze acceptance criteria
before measurement. Preserve evidence and compare live calibration with the
nominal simulation model without treating nominal values as hardware guarantees.

Close only after both required profiles and the core checks pass, or after an
explicit deployment-scope decision revises the required profile set. Pointcloud,
camera–LiDAR calibration, calibrated accuracy, and motion validation are separate.

**Context.** [Camera stack](target_localization/camera_stack.md) and
[aligned depth](target_localization/aligned_depth.md).

## Optional physical organized-pointcloud qualification

**Status.** Separate optional follow-up; not a blocker for core D455 colour and
aligned-depth integration. The hardware input contract leaves pointcloud unwired
until its organization and correspondence are demonstrated.

**Completion criteria if needed.** Before wiring a RealSense pointcloud input,
prove `PointCloud2.height > 1`, colour-grid dimensions and row-major pixel
correspondence, frame/TF correctness, compatible timestamps, and documented
invalid-point representation/density. Preserve device/profile provenance and
application evidence. An unverified or unsuitable cloud remains disabled.

**Context.** [Camera pointcloud contract](target_localization/camera_stack.md)
and [hardware validation](plans/camera_hardware_validation.md).

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

## Shared camera mast and bracket geometry

**Gap.** `tools/isaac/import_ridgeback_urdf.py` authors the camera mast and
standoff as collision-enabled USD cubes. `clearpath/robot.yaml` supplies the
camera mount but declares no shared mast/bracket attachment. The Isaac model
therefore contains structural geometry outside the shared robot description.

**Completion criteria.** Establish the measured attachment geometry in one
shared description or asset source consumed by Gazebo and Isaac. Remove the
independent Isaac authoring after equivalence checks. Preserve the configured
camera pose; compare rendered and collision geometry, TF, and sensor
self-occlusion in both simulators, including motion. Record physical dimensions
and measurement uncertainty. Rerun affected benchmarks before quoting results.

**Context.** [Shared robot geometry](robot/geometry.md#known-representation-differences)
and [Isaac additions](isaac/robot-model.md#hand-authored-parts).

## Robot geometry and drawing audit

**Gap.** The shared reference retains dimensions and a hand-plotted drawing
from the Isaac USD. It does not establish agreement with the generated Gazebo
model or physical hardware, and the drawing cannot detect configuration drift.

**Completion criteria.** Compare frame origins, sensor emission/optical origins,
body bounds, attachment bounds, and floor-contact references in a common frame.
Record source revisions and physical measurement uncertainty; distinguish
configured, model-derived, and measured values. Correct discrepancies at their
source. Regenerate the drawing from authoritative geometry where practical,
and add a repeatable check for its declared dimensions and mount annotations.
Rerun any benchmarks affected by geometry corrections.

**Context.** [Geometry and dimensional provenance](robot/geometry.md).

## Collision-envelope and navigation-footprint validation

**Drive scope.** Retain the [Isaac planar drive](isaac/robot-model.md#why-this-abstraction-fits-the-current-work)
for flat-floor navigation/perception work. Physical roller simulation is deferred
until a traction, terrain, traversal, or wheel-control requirement justifies it.
Geometry and footprint validation can proceed independently; shared outlines do
not establish wheel-contact parity.

**Gap.** Nav2 defines its planar footprint separately from simulator collision
geometry. Its comments claim a circumscribing body polygon, while the available
contact-envelope illustration qualifies only Isaac representations. Agreement
with Gazebo and the deployed hardware, including attachments, remains unproven.

**Completion criteria.** Overlay the configured and effective Nav2 footprints,
each simulator's collision geometry, and the measured physical envelope in the
same frame, with height-dependent attachment limits and explicit uncertainty.
Choose and document the required containment and clearance policy; fix any
violations. Validate frontal, lateral, and angled contacts plus stop/reverse
recovery separately in each simulator. Qualify hardware clearance under the
physical command-chain validation gate; simulator contact results do not close
that gate. Preserve backend-specific tolerances and rerun affected benchmarks.

**Context.** [Collision model and footprint](robot/collision_model.md),
[Isaac colliders](isaac/robot-model.md#colliders), and
[physical command-chain validation](#physical-command-chain-validation).

## Archived evidence

- [LiDAR/closed-loop SLAM qualification](../archive/engineering/2026-09-17-isaac-lidar-qualification.md)
- [paired results and interpretation](../archive/engineering/2026-09-17-isaac-lidar-qualification.md#interpretation)
- [Isaac port history](../archive/engineering/port-history.md#lidars-detached-from-the-articulation-2026-09-11)
- [removed D435 transform](../archive/engineering/operational_incidents.md#d435-static-camera-transform--removed-2026-08-31)
