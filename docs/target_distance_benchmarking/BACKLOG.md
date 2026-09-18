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

## Benchmark execution-environment qualification

**Priority and dependency.** Complete this gate before parameter-tuning campaigns
or treating new benchmark comparisons as reportable results. Qualification and
bounded diagnostic runs are allowed to establish the gate; their outputs remain
qualification evidence. This applies to live runs and the provenance of captures
used for replay tuning. It is a documented workflow gate, not a CLI-enforced lock.

**Gap.** Remote-versus-physical-seat equivalence has not been established for the
current Gazebo GLX workflow. Rendering load may affect sensor delivery, processing,
and which events are scored. Neither session is automatically a valid baseline.

**Remaining work.** Execute the [environment qualification plan](../plans/remote_vs_physical_seat_validation.md):
verify actual rendering paths; freeze conditions and practical acceptance margins;
compare matched light/heavy workloads with at least three independent replications
per condition; assess sensor delivery, processing, scored outcomes, and uncertainty.

**Completion criteria.** Preserve reproducible evidence showing that an identified
execution environment meets declared stream/application requirements and is
repeatable enough for the intended claims. Interchangeable remote/seat results
additionally require practical equivalence within predeclared margins. If only
one environment is qualified, restrict tuning and claims to it and record the
unresolved comparison separately. Display-only limitations must be scoped;
measurement differences or inconclusive evidence keep the affected use blocked.
Gazebo GLX qualification does not establish Isaac, EGL, hardware, or untested
workload/profile equivalence. Record the qualified conditions in run guidance.

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

## Combined segmentation and monocular-depth throughput

**Gap.** The combined SlimSAM/Depth-Anything path runs its models sequentially.
Timing diagnostics and the fourteen-configuration sweep exist, but a controlled
current comparison has not established whether this path keeps up or whether
concurrency is needed. This is an evidence gap, not a confirmed performance bug.

**Remaining work.** Execute the [measurement plan](../plans/model_concurrency_evidence.md):
compare neither model, each model alone, and both together over three process
replications, with a diagnostics-off/on control. Separate cold startup from
steady-state completion, latency, scored outcomes, and resource use.

**Completion criteria.** Preserve a reproducible evidence record and classify
whether the current path keeps up, has a cold-start issue, suffers warm overload,
or shows resource interaction. If results are inconclusive, retain the precise
unresolved measurement. If it keeps up, close this item without redesign. Only
a demonstrated bottleneck justifies proposing a separate concurrency experiment;
no production architecture change is included here.

## Live sweeps from the configurator

**Status and priority.** Blocked on cleanup/process ownership; prioritize
[execution-environment qualification](#benchmark-execution-environment-qualification)
before this convenience feature. Qualification can use the terminal workflow.

**Gap.** The configurator authors live sweeps but cannot start them. Sweep
preflight cleanup can kill the configurator through two broad install-path
matches, and live process ownership/conflict handling is not qualified.

**Remaining work.** Follow the [implementation plan](../plans/benchmark_gui_direct_run.md):
fix owned-process cleanup; reuse the CLI dry-run estimate; show the actual child
renderer and qualification status; detect genuine conflicting runs; support live
start, progress, cancellation, and restart/reattachment through existing records.

**Completion criteria.** Demonstrate that a live sweep starts without killing
the configurator or unrelated processes, reports actual progress, reconnects
after browser/server restart, and cancels its owned processes cleanly. Verify
dry run has no launch/cleanup side effects and existing offline workflows remain
intact. Record the lifecycle evidence in the qualified environment and update
operator guidance. UI availability does not itself qualify benchmark results.

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
