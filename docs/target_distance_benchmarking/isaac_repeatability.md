# Isaac repeatability: fixes and evidence

This overview connects the fixes used for the accepted Isaac repeatability
tests: what each addresses, what evidence supports it, and what remains outside
the conclusion. Detailed mechanisms stay in their existing topic references;
dated measurements stay in the records under [Archived evidence](#archived-evidence).

It belongs to target-distance benchmarking because the result depends on the
renderer, scene lifecycle, model readiness and measurement protocol together.
The [documentation ownership rules](../project/documentation.md#canonical-owners)
place these maintained explanations under `docs/` and experiment narratives in
`archive/engineering/`.

## Where the repeatability test is documented

| Question | Owner |
|---|---|
| What workflow was accepted, and what may it claim? | [Accepted Isaac repeatability scope](semantics.md#accepted-isaac-repeatability-scope) |
| Where are the exact method, build, per-condition means/maxima, Gazebo comparison and exclusions? | The accepted isolated-condition report under [Archived evidence](#archived-evidence) |
| How does the campaign helper collect and validate repeats? | [Unattended qualification](unattended_qualification.md) |
| Which additional experiments remain unperformed? | [Qualification beyond isolated repeatability](BACKLOG.md#isaac-qualification-beyond-isolated-repeatability), with separate replay/lifecycle/comparison items in that backlog |

## Rendering fixes

### Expose and select an explicit rendering recipe

**Problem.** Fixed simulation stepping alone does not make independent renderer
sessions identical. Temporal AA, denoising and render history can change camera
inputs even when the authored scene and poses match.

**Change.** Expose renderer and AA selection through the benchmark environment
and sweep path. The repeatability recipe selects deterministic simulation,
PathTracing, AA off and no PathTracing denoiser. Deterministic stepping was
already present; this work did not invent a new simulation clock. The recipe
also does not change every public launch to PathTracing by default.

**Contribution.** This makes the rendering conditions explicit and testable and
provides a less history-dependent starting point. It was insufficient on its
own: early PathTracing pairs still had different RGB and monocular depth.
Do not assign all later improvement to the renderer selection alone.

Owners: [environment arguments](launch_arguments.md#environment-layer) and
[camera timing/rendering](../isaac/camera-depth.md#renderer-repeatability).

### Disable the PathTracing result cache

**Problem.** PathTracing's lighting-result cache evolves with render history,
allowing independent sessions to produce different RGB from a frozen scene.

**Change.** At startup, disable `/rtx/pathtracing/cached/enabled` specifically
for deterministic PathTracing. Preserve the separate many-light sampling cache
and other renderer/mode combinations. GUI presentation flags are appended so
windowed launches keep the cache and AA controls.

**Contribution.** This has direct controlled evidence: disabling this cache
eliminated the frozen probe's RGB discrepancy and greatly improved the furnished
pipeline. Small live RGB differences remained. It also changes image content,
so cache-on and cache-off data cannot be mixed as equivalent inputs.

Owner: [renderer repeatability policy](../isaac/camera-depth.md#renderer-repeatability).

### Prevent Isaac from restoring DLSS over the requested AA mode

**Problem.** A requested AA value of zero did not reliably stay effective after
renderer initialization; isolated probes observed AA returning to DLSS.

**Change.** For deterministic PathTracing with a non-DLSS AA request, clear
`/rtx-transient/post/aa/limitedOps` at startup. An explicit DLSS request remains
supported. Verify effective settings rather than treating requested arguments
as proof that the renderer obeyed them.

**Contribution.** The setting now holds in verified cache-policy probes. This
removes a configuration confound; confirmed AA off still did not produce
universal image identity. Its separate contribution to final distance deltas
was not measured in a full-matrix ablation.

Owner: [renderer repeatability policy](../isaac/camera-depth.md#renderer-repeatability).

## Monocular startup fix

**Problem.** Empty measurement messages could satisfy startup checks before
Depth Anything had performed useful inference. Lazy model initialization then
occupied the beginning of capture, while the newest-pending-batch policy
replaced work. Some saved batches had neither model depth nor live measurements.

**Change.** Warm the selected monocular depth source on a real buffered camera
frame, even if that frame has no detections. Publish readiness only after
successful inference with the expected image grid. The benchmark waits for
that producer state before scene setup and timed capture. A detected target,
TF or a successful range estimate is not required. Stereo and polar-only
configurations bypass this monocular gate.

**Contribution.** The targeted gated pair eliminated the observed cold-start
context gaps, and the accepted full pairs passed their readiness/context
guards. This improves capture validity; it is not evidence of a changed model
or improved distance accuracy. It cannot recover later batches replaced under
load, and it does not extend a measurement deadline to fill a sample quota.

Owners: [monocular producer lifecycle](../target_localization/aligned_depth.md#3-source-2-rgb-frame--aligned-depth-depth-anything),
[producer-state fields](../target_localization/distributed_deployment.md#topics-types-and-acquisition-semantics)
and [benchmark readiness](semantics.md#scenario-and-comparison-discipline).

## Fabric scene-cleanup fix: P0

**Problem.** A deleted target could disappear from USD while a prim survived in
Fabric, the renderer's scene. That leftover could keep rendering in RGB/depth
and affect later trials. USD/registry-only state checks could report a clean
scene despite the visible residue. This explains contamination in older pairs
that was initially interpreted as intrinsic monocular/rendering variation.

**Change.** Audit Fabric after deletion, repair leftover prims with bounded
retries and check the repair on subsequent normal updates. Scene queries also
discover residue outside the live entity registry. Keep unrepaired residue
discoverable so the existing `state_mismatch` and `CLEANUP_INCOMPLETE` paths
expose the failure. The clean path does not insert extra simulator updates.
P0 is integrated with the render/readiness changes in current `dev`.

**Contribution.** Stress tests exercised repair, reuse controls and deliberately
persistent residue. The accepted box/stereo run also exercised a real repair
without changing later projective/Euclidean outcomes or paired numeric values.
This addresses an observed scene-contamination mechanism. It does not imply
that every remaining RGB difference is a Fabric leak, or that all originally
proposed persistent-session qualification has been performed.

Owners: [scene-state checks](semantics.md#background-props) and
[persistent-residue troubleshooting](../troubleshooting.md#isaac-logs-persistent-fabric-residue).

## Measurement and execution safeguards

These changes make the evidence interpretable and keep collection reliable.
They do not independently make the renderer more deterministic.

| Safeguard | Why it matters and what it establishes | Owner |
|---|---|---|
| Fresh sessions and matched conditions | Repeating a pose inside one session can reuse the same render state. The accepted route tests each condition first in independent sessions; it does not test switching all conditions in one persistent session. | [Comparison discipline](semantics.md#scenario-and-comparison-discipline), [Isaac-only routes](unattended_qualification.md#isaac-only-repeatability) |
| Source, installed executable and input verification | Early targeted attempts accidentally used stale installed copies and were excluded. Rebuild copied entrypoints, check package prefixes, and freeze source/install/model/input identities before collection. | [Copied-install convention](../project/conventions.md#generated-state), [frozen-input checks](unattended_qualification.md#deadlines-ownership-and-failure-behavior) |
| Readiness and sampled-live guards | Require warmup before sampled stamps and a live measurement record for sampled events; retain empty detections and legitimate absent unused inputs. This prevents incomplete startup captures being mistaken for model variation. | [Sampled-window capture](unattended_qualification.md#sampled-window-capture), [readiness checks](unattended_qualification.md#readiness-checks) |
| Per-instance comparison and retained misses | Preserve each trial/instance's two estimates, outcomes and miss reasons. Earlier scratch comparisons could overwrite multi-instance results; means over numeric pairs must be shown with coverage. | [Result pairing](unattended_qualification.md#isaac-only-repeatability), [scoring semantics](semantics.md#miss-reasons-and-attribution-boundary) |
| Realistic bounded campaign startup/admission limits | The earlier readiness stage refused the third configuration before any full run. Revised limits allow cold startup while preserving measurement windows and cleanup reserves. This repairs execution admission, not sensor repeatability. | [Campaign deadlines](unattended_qualification.md#deadlines-ownership-and-failure-behavior) |
| Owned-process monitoring and quiet-host checks | Record contention, progress, process exits and cleanup; freeze checks reject drift. The campaign helper has its own heartbeat route, separate from the accepted condition-first harness. | [Receiving-agent heartbeat](unattended_qualification.md#receiving-agent-heartbeat), [ownership and failures](unattended_qualification.md#deadlines-ownership-and-failure-behavior) |

The condition-first harness also corrected a workstation-lock handover: its
host sampler closes inherited lock FD 9, and the next driver allows a bounded
handover. This avoids an orphan sampler sleep retaining the previous driver's
lock. It was a local execution-harness change, not a production renderer,
estimator or measurement-protocol fix; the accepted report retains its receipt.

## What the combined result establishes

The accepted scope covers the full scenario expansion for box/silhouette ×
monocular/ideal-stereo with the recorded models, GUI and diagnostics workload.
Projective/Euclidean outcomes were stable across those isolated pairs; the
report retains numerical differences, the cabinet-association residual, RGB
exceptions, polar outcome changes and LiDAR spread, and the excluded graphics
crash.

Image comparisons cover the first five stored batches of each 10-second
window; numerical trial estimates use the full window. Matching early images
does not prove equality of every later frame. Two fresh sessions per condition
provide paired differences, not a population variance estimate.

This is a combined-build result. Cache controls have their own controlled
probe, warmup has targeted before/after evidence, and Fabric removal has direct
repair/control evidence. There is no full-scene factorial experiment measuring
each fix's independent contribution. Older contaminated persistent sessions
and newer isolated sessions are different populations; a ratio of their maxima
would not be a defensible improvement factor.

## Controls and proposals that are not additional implemented fixes

| Investigation or proposal | Evidence boundary |
|---|---|
| More render warmup, zero-delay mode or explicit zero dither | Did not cure the frozen-scene cache-on RGB discrepancy. A frame-freshness probe still observed a one-update lag under its mutation ordering; no new production frame-delivery fix follows from it. |
| Disable many-light sampling or texture streaming | Residual live/static differences remained; neither was a demonstrated cure, and these extra switches were not adopted in the production repeatability policy. |
| Hold physics/poses, deactivate before deletion or add more settle frames | These controls did not establish a new general renderer fix. Deactivation did not prevent Fabric leaks; absence of leaks in the longer-settle control was inconclusive. |
| Fixed prim slots or a spawn-once pool | Prevention candidates had reproducer evidence, but were not implemented as P0. The accepted condition-first protocol likewise does not add automatic per-condition restarts to persistent production sweeps. |
| Estimator changes, LiDAR noise removal or a new equivalence margin | No estimator/model tuning was used to hide the repeatability differences. LiDAR noise remains enabled. The owner's acceptance does not invent a universal numerical margin or require pixel-perfect RGB. |

Remaining evidence requirements are owned by the
[benchmarking backlog](BACKLOG.md#isaac-qualification-beyond-isolated-repeatability).
They include cross-condition history, new-capture offline parity and conditional
controlled backend comparisons; broader lifecycle, isolation, occlusion,
throughput and accuracy questions retain their own entries.

## Archived evidence

These records own the measurements and their revisions, conditions and limits.
The figures below summarize those records; they are not interchangeable
populations or a new ablation study.

| Record | What it demonstrates about the fixes |
|---|---|
| [Accepted isolated-condition repeatability and Gazebo comparison](../../archive/engineering/2026-11-21-isaac-condition-repeatability.md) | Main test report: eight valid fresh sessions, 872 trials and 4,360 sampled batches; per-condition means/maxima and coverage. Largest retained projective/Euclidean delta: 20.527 mm, tied to a cabinet detection/association case. One real Fabric repair, zero persistent events, and 59/60 later trials with identical sampled RGB; later paired numeric projective/Euclidean values stayed equal. Earlier P0 evidence includes four repairs in 3,000 stress cycles, a clean 1,000-cycle reuse control and persistent-injection failure handling. |
| [PathTracing cache diagnosis](../../archive/engineering/2026-11-05-isaac-render-cache-diagnosis.md) | Controlled frozen scene: cache-on RGB 0/10 identical pairs, result-cache-off 10/10. Furnished result-cache-off RGB matched in 112/115 stereo and 118/118 monocular pairs. The latter is a small timestamp-matched probe, not the later full-scene dataset. |
| [Cache policy and effective-AA verification](../../archive/engineering/2026-11-05-isaac-cache-policy-and-residual-probes.md) | All 510 settings snapshots had result cache off, many-light sampling on and effective AA off. Residual live pixels and startup HDR transitions remained; texture/many-light changes did not establish a cure. |
| [Monocular readiness and targeted output repeatability](../../archive/engineering/2026-11-05-monocular-readiness-and-isaac-repeatability.md) | Gated pair: zero missing depth/live measurements in 356 monocular batches; all 24 outcome/reason rows matched. Maximum monocular projective/Euclidean deltas were 0.412/0.626 mm. Stale-installed attempts are explicitly excluded. |
| [Older full-matrix pair and subsequent contamination correction](../../archive/engineering/2026-11-06-isaac-full-repeatability.md) | Collection completed after the admission-limit repair, but the 1.376 m monocular tail was later attributed to stale Fabric geometry. It is evidence for the need to distinguish scene integrity from intrinsic model/render variation, not a clean noise baseline. |
