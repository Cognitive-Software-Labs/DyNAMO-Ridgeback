# Documentation-backed engineering backlog

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

## Deferred: Isaac target-distance benchmark port

**Status.** Deferred (former P6); this records scope, not approval to implement.
The target-distance benchmark still requires Gazebo-specific entity control.

**Scope if resumed.** Replace Gazebo CLI spawn/remove/pose plumbing with a
backend interface and Isaac simulation-control services. Preserve the current
scenario schema, registered estimators, layered replay and configurator.
Forward backend selection and Isaac arguments through the benchmark environment
launch; cover process startup and teardown for both backends.

**Acceptance.** An Isaac run with all registered estimators and one repeat
completes the configured scenes, comparison report and collages; estimates are
compared against a current matching Gazebo baseline, and repeated cleanup leaves
no residual prims. Historical geometry-dependent numbers are not acceptance evidence.

**Context.** [Target-distance benchmarking](target_distance_benchmarking/overview.md).

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

**Context.** [Aligned depth](target_localization/aligned_depth.md) and
`common/camera_inputs.py`.

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
[scenario gallery](target_distance_benchmarking/benchmark_scenarios_v2_gallery.html).

## Archived evidence

- [LiDAR/closed-loop SLAM qualification](../archive/engineering/2026-09-17-isaac-lidar-qualification.md)
- [paired results and interpretation](../archive/engineering/2026-09-17-isaac-lidar-qualification.md#interpretation)
- [Isaac port history](../archive/engineering/port-history.md#lidars-detached-from-the-articulation-2026-09-11)
- [removed D435 transform](../archive/engineering/operational_incidents.md#d435-static-camera-transform--removed-2026-08-31)
