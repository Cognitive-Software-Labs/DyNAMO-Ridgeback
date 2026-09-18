# Target-distance benchmarking backlog

This is the canonical list of open target-distance benchmarking work: replay
qualification, benchmark tooling, scenario certification, and controlled
estimator comparisons. Exploration evaluation and shared robot/deployment gaps
remain in the [project backlog](../BACKLOG.md).

Each item states the remaining gap and its completion criteria. Implemented
contracts belong in the [benchmark references](overview.md); detailed execution
procedures may live in linked plans. Completed evidence goes in the engineering
archive, and the closed item is removed. A deferred item or a linked proposal
is not approval to implement it.

## Layered replay validation

**Gap.** Layered replay is implemented; fresh live parity, real segmentation
model, representative storage, and scaling evidence remain incomplete. The
[profile reference](profiles.md) owns the implemented contract.

**Remaining work.**

- Capture a fresh live sensor dataset with exact RGB, depth, detections,
  calibration, and transforms. Verify event identity, empty detections,
  explicit missing evidence, and absence of skipped trials.
- Compare live and offline box/current-SlimSAM measurements over the same
  captured events: numeric results, outcomes, miss reasons, and association.
- Materialize a candidate checkpoint or changed segmentation parameter from
  that same capture, then reuse both baseline and candidate caches in a paired
  measurement sweep.
- Measure representative full-frame capture and cache sizes, encoding/decoding
  cost, materialization time, and peak RAM/VRAM.
- Verify sequential and parallel replay results agree; measure scaling and
  identify where additional workers stop helping.

**Constraints.** Preserve legacy measurement-replay compatibility, exact stamps,
and artifact lineage. Keep ground truth outside mask generation. Offline
measurements establish offline costs; whole-system performance claims require
live runs. Production default changes and additional replay profiles are outside
this assignment.

**Completion criteria.** Record revisions, commands, artifact hashes, parity
results, candidate comparison, storage/resource measurements, scaling, and
remaining limitations in a dated evidence record. Update maintained references
with demonstrated facts and remove this item only when the listed gates pass.

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

**Context.** [Camera stack](../target_localization/camera_stack.md) and
[shared robot geometry](../robot/geometry.md).

## Isolation validation

**Gap.** 2D/3D recipes and current defaults are implemented, but the
2026-08-28 default/depth-gate transition and cross-recipe accuracy have not been
established by a controlled current comparison, especially with deep backgrounds.

**Completion criteria.** Fix one scenario YAML and revision; include clutter,
near occluders, long backgrounds, ranges, and viewpoints; compare registered
recipes and box/silhouette controls without regenerating scenes; record accuracy,
misses/coverage, runtime, memory, and parameters; include exploration only as a
qualitative follow-up. Treat a default change as a separate decision.

**Context.** [Projective-ranging isolation](../target_localization/projective_ranging.md#implemented-2d-recipes),
[euclidean-reconstruction isolation](../target_localization/euclidean_reconstruction.md#implemented-3d-recipes),
and the [candidate protocol](../do_not_try_again/foreground_isolation.md#evaluation-protocol).

## Occlusion characterization

**Gap.** Dedicated `interocc_*`, `objocc_*`, and `objpartial_*` scenes exist,
but current defaults have not first been characterized well enough to select a
recovery design. The [depth-clustering plan](../plans/occlusion_handling.md) is a
proposal, not an approved algorithm or current miss reason.

**Completion criteria.** Run current defaults on the fixed occlusion families;
separate detector, segmentation, depth/scan availability, isolation, association,
and wrong-near estimates; identify which estimator/precision/scenes require
change; decide whether the failure merits a new recipe, a wrong-object guard, an
explicit occlusion reason, or no change. Only then approve a scoped implementation
and require non-occluded regression evidence.

**Context.** [Occlusion proposal](../plans/occlusion_handling.md),
[segmentation candidates](../do_not_try_again/segmentation.md), and the
[scenario gallery](benchmark_scenarios_v2_gallery.html).

## Other benchmarking execution plans

These plans retain their execution detail; listing them here does not change
their approval or completion status.

- [Live sweeps from the configurator](../plans/benchmark_gui_direct_run.md)
- [Model-concurrency evidence](../plans/model_concurrency_evidence.md)
- [Remote versus physical-seat comparison](../plans/remote_vs_physical_seat_validation.md)

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

**Context.** [Target-distance benchmarking](overview.md).

## Archived evidence

- [Layered replay implementation and preliminary storage evidence](../../archive/engineering/layered_replay_implementation_validation.md)
