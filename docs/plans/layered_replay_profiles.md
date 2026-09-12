# Plan: layered replay profiles and derived evidence caches

Status: **IMPLEMENTED — LIVE VALIDATION GATES REMAIN.** This plan extends the validated offline
measurement replay so an operator can choose the earliest stage to rerun. It is
not authorization to change a production measurement default or to treat
offline results as latency, throughput, transport, or integration evidence.

Prepared 2026-09-09 against `aafa4385`.

Implemented 2026-09-10 in the benchmark core and local configurator: shared
profiles/jobs, typed sensor and mask-cache artifacts, exact-evidence runner
capture, box and SlimSAM materializers, projective/Euclidean cache replay,
atomic outputs, and the three generalized commands. Unit/compatibility/import/determinism tests and
a preliminary lossless storage spike pass. A fresh full live sensor capture,
box/SlimSAM live parity, real full-frame storage/scaling measurements, and a
candidate checkpoint run remain required before this plan can move to history;
see `docs/history/layered_replay_implementation_validation.md`.

## Decision

Implement three explicit offline profiles over immutable, lineage-tracked
artifacts:

| Profile | Frozen evidence | Work rerun | Question it can answer |
|---|---|---|---|
| `measurement` | detections, box-local depth, calibration and transforms | deterministic box selection plus measurement recipes and numeric parameters | Which measurement strategy is best for these exact detections? |
| `mask-output` | detections, one or more materialized mask sets, depth, calibration and transforms | measurement from each frozen mask set | Are the current SlimSAM masks better than box masks for distance accuracy and coverage? |
| `mask-model` | detections, exact RGB, aligned depth, calibration and transforms | mask generation, then measurement | Does another segmentation checkpoint or segmentation parameter improve the result? |

The profiles form one artifact graph rather than three unrelated capture
formats:

```text
sensor capture (detections + exact RGB/depth/context)
      |
      +-- box mask cache --------------------+
      |                                      |
      +-- SlimSAM current mask cache --------+--> measurement sweeps
      |                                      |
      `-- SlimSAM candidate mask cache ------+

legacy measurement capture ---------------------> measurement sweeps
```

The existing compact replay dataset remains a valid `measurement` input. A new
sensor capture is needed only when the experiment must rerun segmentation. Mask
caches are derived children of that capture and may be reused by any number of
measurement sweeps.

## What each profile does and does not prove

Offline profiles answer accuracy, coverage, miss-reason, and paired-result
questions over frozen events. Their reports must carry the claim boundary:

- `measurement` and `mask-output` do not time OWLv2 or SlimSAM and cannot
  establish their runtime efficiency.
- `mask-model` may report isolated mask-production time on the replay host, but
  it still does not establish ROS scheduling, camera delivery, end-to-end
  latency, throughput, GPU contention with the detector, or simulator real-time
  factor.
- `live-system` remains the existing live benchmark. It is not an offline
  profile and is required for all whole-system efficiency and integration
  claims.

Detector-output and detector-model replay are deliberately deferred. The
profile registry should make them possible later, but this implementation stops
after the two requested mask-level designs work.

## Scope boundaries

- Preserve the current 109-trial replay dataset and byte-stable result contract.
- Keep ground truth scoring-only. Mask production must never receive truth or
  use it to retain, reject, or crop an event.
- Capture by raw detection-batch quota, including empty batches. Keep
  `capture_timeout_sec` as a stall guard and the bounded drain as an
  exact-evidence completion step; elapsed seconds are not the sample count for
  offline replay.
- Continue to support `capture_sec` for the live-system benchmark.
- Reuse `MaskRegion`, `MaskPrecision`, the live measurement pipeline, sweep
  parsing, scoring, reduction, trial output, and report generation. Do not add
  parallel definitions of masks, parameters, misses, or accuracy.
- Keep trial-local CPU measurement work parallel. Do not interpret “50
  variants” as permission to load 50 GPU models simultaneously.
- Add no storage dependency until a measured format spike proves NumPy plus
  lossless image encoding inadequate.
- Do not add a sixth public ROS launch file. Extend the existing benchmark
  config/runner surfaces and add offline commands where required.

## Shared profile and parameter contract

Create a ROS-free benchmark profile registry as the single owner of:

- stable profile ID and human label;
- earliest mutable stage;
- required artifact kinds and fields;
- frozen stages and rerun stages;
- allowed sweep axes, their types, defaults, ranges, and consuming stage;
- compatible estimators, depth sources, and mask gates;
- supported claims and explicit limitations;
- executor/materializer entry point;
- storage and duration estimate inputs.

Extend the existing sweep validator with stage ownership rather than creating a
GUI-only or replay-only parameter list. Validation must reject an upstream knob
with an actionable correction, for example:

```text
segmentation_model is frozen by profile "mask-output";
choose profile "mask-model" to vary it.
```

Expose the resolved contract as JSON-compatible data so command-line tools, the
configurator UI, tests, and reports all consume the same facts.
Describe the existing `live-system` benchmark through the same capability
surface for configuration purposes, but route it to the existing live workflow,
not through a replay executor.

## Artifact contract

Introduce a typed manifest envelope without rewriting legacy data:

```yaml
format: dynamo-replay
manifest_version: 2
artifact:
  id: <stable content-derived identity>
  kind: sensor-capture | mask-cache
  state: complete
parent:                         # mask-cache only
  artifact_id: <sensor capture id>
  manifest_sha256: <hash>
producer:
  commit: <git commit>
  dirty_count: 0
  model: <checkpoint or box rasterizer>
  model_revision: <resolved revision>
  dependencies: <relevant versions>
  parameters: <complete producer arguments>
trials: [...]
```

Use a manifest dispatcher:

- current `schema_version: 1` datasets load unchanged and are interpreted as
  legacy `measurement` evidence;
- new sensor captures and mask caches use the typed envelope and their own
  payload schema versions;
- unknown kinds, versions, incomplete state, missing parents, parent-hash
  mismatch, and payload-hash mismatch fail before evaluation;
- derived artifacts never silently locate a “similar” parent. The declared
  parent must be supplied and its manifest hash must match.

### Sensor capture payload

For every frozen raw detection batch, including `count == 0`, store:

- trial/event identity, source stamp and frame;
- detection count, labels, scores, and boxes;
- the exact RGB frame on the detection grid, losslessly;
- the exact aligned-depth frame in metres, preserving invalid values;
- camera intrinsics, camera-to-base transform, front offset, and depth usable
  range;
- explicit absence when an exact RGB, depth, or context match did not arrive.

At the current 640 x 480 grid and five batches per 109 trials, uncompressed RGB
plus float32 depth is approximately 1.1 GiB before metadata. Phase 0 must
measure lossless PNG/NumPy and compressed-NPZ alternatives on representative
events, including encode/decode time, before fixing the on-disk layout. The
format must preserve exact RGB bytes and float32 depth semantics; lossy JPEG is
not eligible.

### Mask cache payload

A mask cache stores one index-aligned result for every parent detection:

- `None` as the explicit no-mask outcome, distinct from an empty mask;
- `MaskRegion.origin_u`, `origin_v`, full image dimensions and precision;
- packed boolean region data and its shape;
- producer status, such as oversized box or rejected/empty segmentation;
- materialization duration as diagnostic metadata, not an accuracy input.

Store the region's actual extent, not the detector box. SlimSAM prompt padding
can legitimately select pixels outside the original box. Use the existing
`MaskRegion` representation and round-trip it exactly; do not invent a cache
mask type.

Mask caches reference parent depth instead of duplicating it. A future bundle
or export command may make an artifact graph portable, but it is not required
for this slice.

## Execution model

Split execution into two stages with different parallelism:

1. **Materialize masks.** Load a model once, stream parent trials through it,
   and write an immutable child cache. Box materialization uses no model.
   Model-level concurrency is explicit and capacity-aware; default to one model
   worker per available device rather than one model load per trial.
2. **Evaluate measurements.** Load one trial and its selected mask caches per
   CPU worker, then apply every compatible numeric measurement variant while
   the arrays are hot. Preserve deterministic variant/trial ordering in output.

This is what makes a large parameter grid cheap: a SlimSAM cache is produced
once, while 5, 50, or more measurement strategies can consume it without
running SlimSAM again. Changing the checkpoint, prompt padding, predicted-IoU
floor, preprocessing, or relevant dependency revision produces a different
cache rather than mutating an existing one.

GPU inference can vary across hardware and library versions. The cache makes
the measurement comparison deterministic by freezing the observed masks; it
must not claim that recomputing a model on another stack is bit-deterministic.

## Command surfaces

Add generalized, ROS-free commands backed by importable functions:

- `target_replay_describe --json` — resolved profiles and parameter contract;
- `target_replay_materialize_masks <sensor-capture> ... --output-dir <new>` —
  create a box or segmentation mask cache;
- `target_replay_benchmark <job-spec> --output-dir <new>` — validate lineage,
  resolve artifacts, and evaluate the selected profile.

Keep `target_offline_replay_benchmark` as a compatible measurement-replay entry
point while existing scripts and datasets depend on it. Remove user-facing
“V1” wording from help and validation errors; `schema_version: 1` remains the
correct internal compatibility marker.

The canonical job specification should name the profile, input artifacts,
sweep, worker limits, output location, and comparison baseline. Commands must
accept only structured arguments and never evaluate shell text.

## Implementation phases

### Phase 0: contract fixtures and storage spike

- Freeze one legacy replay dataset/output as a compatibility fixture.
- Capture a small RGB/depth/detection corpus containing empty detection,
  multiple detections, missing exact evidence, an oversized box, a mask that
  extends outside its prompt box, and a rejected segmentation.
- Measure candidate lossless storage layouts and choose the smallest layout
  whose decode cost does not dominate the expected offline work.
- Write the manifest and job-spec reference before writing executors.

Gate: reviewers can determine from the documents and fixtures exactly which
bytes are frozen and which stages each profile reruns.

### Phase 1: profile registry and compatibility loader

- Add the pure profile/axis registry and structured validation errors.
- Add the typed manifest dispatcher and legacy schema adapter.
- Move the current replay-specific allowed-argument checks behind the shared
  registry without changing accepted legacy sweeps or outputs.
- Expose a JSON description consumed by the configurator UI.

Gate: every legacy replay test and golden result remains unchanged; every axis
is owned by one stage and invalid profile/axis combinations fail before work.

### Phase 2: sensor capture

- Add a capture profile to the existing runner rather than another runner.
- Reuse exact-stamp buffering so raw detections, RGB, depth, intrinsics and TF
  belong to the same event.
- Preserve the batch quota, bounded drain, timeout, empty batches, and
  incomplete-dataset rules.
- Move serialization details out of the ROS node so the runner remains
  orchestration-only.

Gate: a small live capture has no skipped trials, every present payload matches
its declared grid/stamp, and any absent exact evidence remains explicit rather
than being substituted with a nearby frame.

### Phase 3: derived mask caches

- Implement box and SlimSAM producers over the sensor payload.
- Resolve and record checkpoint revision, preprocessing, prompt padding,
  predicted-IoU floor, device/dtype settings, code provenance, and dependency
  versions.
- Serialize `MaskRegion` and no-mask outcomes with complete parent lineage.
- Make output creation atomic and refuse overwrite/resume from a mismatched
  producer signature.

Gate: box masks match `region_from_bbox`; loaded SlimSAM regions select exactly
the pixels produced before serialization; ground truth is inaccessible to both
producers.

### Phase 4: generalized measurement replay

- Extract the current hard-coded projective evaluator behind a mask-provider
  interface and call the shared `fill_path_measurements` path.
- First preserve projective-ranging parity, then add euclidean reconstruction
  over the same prepared depth region. Polar replay remains deferred because it
  needs scan evidence and time-resolved transforms.
- Compare multiple mask caches and measurement variants as paired results over
  identical parent events.
- Keep trial-local CPU multiprocessing and deterministic output ordering.

Gate: the current projective result is unchanged, and one sensor capture can
produce a paired box-versus-SlimSAM accuracy/coverage report without Gazebo,
OWLv2, ROS, or model inference during measurement evaluation.

### Phase 5: evidence and rollout

- Run live/offline parity for box and SlimSAM on the same captured events.
- Confirm sequential and parallel measurement evaluation are byte-identical.
- Measure sensor-capture size, mask-cache size, materialization time, replay
  scaling, peak RSS/VRAM, and the worker knee.
- Run at least one segmentation candidate through `mask-model`, then reuse both
  resulting mask caches in a multi-variant measurement sweep.
- Update the benchmark reference, README workflow, docs index, and dated
  history only after all gates pass.

Gate: the report states both the accuracy conclusion and the exact profile
boundary, so a replay timing number cannot be mistaken for system efficiency.

## Verification matrix

Automated proof must cover:

- legacy schema loading and golden output;
- new manifest/payload versions, hashes, incomplete state and parent lineage;
- RGB/depth exact-stamp and grid mismatch handling;
- `MaskRegion` round-trip for rect, tight, empty, `None`, holes, disconnected
  components, and regions extending beyond the detector box;
- producer signature changes for checkpoint, model revision, prompt padding,
  predicted-IoU floor, preprocessing, dependency and code changes;
- profile-aware parameter acceptance and rejection;
- box and SlimSAM live/offline outcome, miss-reason, association and numeric
  parity;
- projective and euclidean reuse of one prepared depth region;
- sequential/parallel determinism and interrupted-output rejection;
- import proof that measurement and mask-output evaluation do not import ROS,
  Gazebo, transformers, torch, or the detector;
- report provenance and supported-claim labeling.

## Stop condition

Stop this implementation when all of the following are true:

- existing measurement replay remains compatible and equally fast;
- one frozen sensor capture derives a box cache and the current SlimSAM cache;
- those caches produce a paired box-versus-SlimSAM measurement report;
- a changed segmentation model or segmentation parameter produces a separately
  identified cache from the same sensor capture;
- many measurement variants consume each cache without rerunning segmentation;
- invalid upstream parameters fail before execution with the correct profile
  recommendation;
- live parity, lineage, corruption, determinism, storage, and scaling evidence
  are recorded.

At that point, do not add detector replay, a distributed artifact store, remote
execution, automatic model search, or a general DAG scheduler. Each requires a
separate need and plan.

## Deliverables

- Shared profile/axis registry and canonical job specification.
- Typed sensor-capture and mask-cache manifests with legacy replay support.
- Exact RGB/depth/detection capture mode in the existing runner.
- Box and SlimSAM mask materializers with immutable provenance.
- Generalized projective/euclidean measurement replay and paired reports.
- Unit, compatibility, parity, corruption, determinism, import, and performance
  tests.
- Updated technical reference and operator workflow plus dated validation
  evidence.
