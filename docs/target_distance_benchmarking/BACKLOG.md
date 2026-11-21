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

The [sampled monocular capture plan](../plans/BENCHMARKING_sampled_monocular_capture.md)
defines the current handoff for collecting bounded stereo/monocular samples
during the 16 live qualification runs, with matching live references for later replay.
The [accepted isolated-condition pairs](semantics.md#accepted-isaac-repeatability-scope)
also supply fresh full-scene captures with passing collection checks; offline
box/real-SlimSAM parity on those datasets is still unperformed.

**Remaining work.**

- Select and freeze the live sensor dataset for each replay claim, including
  exact RGB, depth, detections, calibration, and transforms. Retain the passing
  collection checks, event identity, empty detections and explicit missing evidence.
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

## Isaac qualification beyond isolated repeatability

**Status.** The [isolated-condition workflow](semantics.md#accepted-isaac-repeatability-scope)
is accepted for its preserved experimental build. It closes that repeatability
question; small RGB differences and the earlier proposed numerical alert limits
are not new blockers for it.

**Remaining scope.** Before claiming repeatability for persistent multi-condition
sweeps, execute the [P0 handoff's original A5](../plans/handoffs/BENCHMARKING_isaac_fabric_leftover_p0_claude.md#7-acceptance-tests):
two fresh sessions, each switching through all four conditions. Check cross-condition history, scene state,
persistent residue, repairs and per-instance outputs. The isolated pairs do not
exercise those transitions. Earlier P0 stress/control/persistent-injection tests
already have evidence; they are not wholly untested requirements.

A same-build, same-input Gazebo pair is required only before making controlled
backend-causal or equivalence claims; the existing historical comparison cannot
support those claims.

**Completion criteria.** For each proposed scope extension, preserve the exact
integrated build and matched inputs, execute its missing comparison, report
coverage/outcomes beside numeric differences, and record the owner's conclusion.
Replay,
[shutdown](#benchmark-shutdown-qualification), and
[geometry-dependent comparisons](#camera-geometry-benchmark-recertification)
retain their own completion criteria in this backlog.

## Hardware-bag experiment qualification

**Gap.** The [bridge tooling](hardware_bag_validation.md) completed the
48-arrangement, ten-path diagnostic. Final comparisons still need independently
reviewed target annotations/association, an agreed error and evaluation protocol,
and justified treatment of the existing reference uncertainty. Numerical return
rates include false objects and duplicate fragments and are not accuracy scores.

**Completion criteria.** Freeze annotation/matching and metric rules; retain
missed detections and explicit missing inputs; diagnose selection/depth behavior
from caches before agreed tuning/ablation; report per-configuration coverage and
reference-qualified accuracy with reproducible lineage. Group shared-reference
placements appropriately. Do not infer a re-recording need from model failures
or treat a review of existing measurement records as new physical evidence.
The diagnostic identified no new onsite requirement. Future live hardware
throughput/autonomy qualification remains separate.

## Camera-geometry benchmark recertification

**Status.** Certification is complete: `scene_audit` certifies every scene at
640x480 from the current description, Gazebo and Isaac agree statically on
camera pose, intrinsics, LiDAR pose and object geometry, and rendered depth
captures in both backends match the audit's masks.
[Scene certification](semantics.md#scene-certification) owns the method.

**Gap.** Benchmark results recorded before the 2026-10-29 scene recertification
predate the floor-standing assets and replaced `edge640_*` scenes. The accepted
isolated-condition captures provide repeatability evidence on the newer geometry,
but do not replace older accuracy results or complete the controlled isolation,
occlusion-attribution and model-throughput comparisons.
[BENCHMARKING — Geometry reruns](../plans/BENCHMARKING_geometry_reruns.md)
owns the order and preconditions.

**Completion criteria.** Rerun every benchmark whose inputs or measured
behavior depend on camera or LiDAR geometry or asset seating, in a
[qualified execution environment](running_benchmarks.md#qualified-execution-environments),
before quoting its numbers as current. Pointcloud was excluded from the
repeatability campaign; its rerun is conditional on reopening that scope.

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
recovery design. Recovery candidates remain deferred until this evidence
identifies the responsible failure boundaries.

**Completion criteria.** Run current defaults on the fixed occlusion families;
separate detector, segmentation, depth/scan availability, isolation, association,
and wrong-near estimates; identify which estimator/precision/scenes require
change; decide whether the failure merits a new recipe, a wrong-object guard, an
explicit occlusion reason, or no change. Only then approve a scoped implementation
and require non-occluded regression evidence.

**Context.** [Deferred recovery candidates](../do_not_try_again/occlusion_recovery.md),
[segmentation candidates](../do_not_try_again/segmentation.md), and the
[scenario gallery](benchmark_scenarios_v2_gallery.html).

## Combined segmentation and monocular-depth throughput

**Gap.** The combined SlimSAM/Depth-Anything path runs its models sequentially.
Timing diagnostics and the fourteen-configuration sweep exist, but a controlled
current comparison has not established whether this path keeps up or whether
concurrency is needed. This is an evidence gap, not a confirmed performance bug.

**Remaining work.** Execute the [BENCHMARKING — Combined-model throughput](../plans/BENCHMARKING_model_throughput.md):
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

**Status and priority.** Cleanup ownership is fixed; live start, conflict
detection and supervision remain.

**Gap.** The configurator authors live sweeps but cannot start them. Live
conflict handling, progress and cancellation are not implemented or qualified.

**Remaining work.** Follow the [BENCHMARKING — Live configurator sweeps](../plans/BENCHMARKING_live_configurator_sweeps.md):
reuse the CLI dry-run estimate; show the actual child
renderer and qualification status; detect genuine conflicting runs; support live
start, progress, cancellation, and restart/reattachment through existing records.

**Completion criteria.** Demonstrate that a live sweep starts without killing
the configurator or unrelated processes, reports actual progress, reconnects
after browser/server restart, and cancels its owned processes cleanly. Verify
dry run has no launch/cleanup side effects and existing offline workflows remain
intact. Record the lifecycle evidence in the qualified environment and update
operator guidance. UI availability does not itself qualify benchmark results.

## Benchmark shutdown qualification

**Status.** Configuration-first ownership is implemented for single runs and
sweeps. The [shutdown plan](../plans/BENCHMARKING_shutdown_lifecycle.md) owns the
remaining qualification and dependency investigations.

**Gap.** The RViz DDS unload crash has a reproduced view-controller ownership
defect and passing ownership regression. Its first traced active Isaac stop is
clean at the application boundary; the strict whole-process matrix stopped on
unexplained SDK startup exit/termination outcomes; environment-only Isaac
scan captures on 2026-10-25 also escalated to SIGTERM/SIGKILL after
`RUNNER EXIT` (`artifacts/benchmarks/ust10lx_review_20261025_base_39fb8c1/`).
The rare image-bridge teardown failure and safe provider destruction with
outstanding blocking scene requests also remain open. Application shutdown and complete SDK qualification are
separate outcomes.

**Completion criteria.** Preserve reproducible failing schedules and corrected
behavior for each remaining signature, then exercise startup, normal completion,
mid-trial cancellation, pending scene requests, repeated interrupts, and child
failure on both backends. Record child exits, escalation, cleanup outcomes, and
owned leftovers separately from saved measurements.

## Archived evidence

- [Isolated-condition repeatability, acceptance and coverage audit](../../archive/engineering/2026-11-21-isaac-condition-repeatability.md)
- [Gazebo and Isaac backend qualification](../../archive/engineering/2026-10-26-benchmark-gazebo-isaac-qualification.md)
- [Scene recertification for the current camera](../../archive/engineering/2026-10-29-scene-recertification.md)
- [Layered replay implementation and preliminary storage evidence](../../archive/engineering/layered_replay_implementation_validation.md)
