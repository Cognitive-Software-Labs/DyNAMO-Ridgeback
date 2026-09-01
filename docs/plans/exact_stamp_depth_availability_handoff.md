# Handoff: locate and fix exact-stamp depth availability loss

Status: **COMPLETED 2026-09-01.** The current-source trace located the loss at
Fast DDS delivery to the mask-node subscriber. CycloneDDS produced 100% exact
depth coverage in the matched post-run and is now the quick-start default.
Results, rejected hypotheses, artifacts, and limitations are recorded in
[`docs/history/exact_stamp_depth_availability.md`](../history/exact_stamp_depth_availability.md).
The handoff below is retained as the investigation contract.

## Objective

Trace individual color/detection/depth stamps across the complete simulation
path until the stage that loses or delays each requested depth frame is proven.
Test multiple competing explanations, including new ones suggested by the
evidence. Only after one mechanism is established, implement its narrowest
responsible fix and run a matched pre/post benchmark containing exactly the two
depth-based estimators:

- `projective_ranging`
- `euclidean_reconstruction`

This assignment includes investigation, a justified narrow fix, regression
tests, the matched benchmark, and documentation. It does not authorize a broad
executor, bridge, transport, or multiprocessing redesign.

## Established starting evidence

- The blocking TF fallback is fixed by `e9ff5f3`; its acceptance run eliminated
  mask `UNSET` and raised both depth rows to 90.23% observation coverage.
- That run still recorded `NO_DEPTH_FRAME` on 73/747 detected-box observations.
  The node's lookup diagnostics ended at 771/840 exact hits and 69 misses:
  63 `target_inside`, 6 `target_newer`, 0 `target_older`, 0 empty-buffer, with
  nearest buffered deltas of 32--34 ms.
- Lookup counts are per detection batch; benchmark status counts are per box.
  Do not compare 69 and 73 as if they have the same denominator.
- Exact-stamp matching is deliberate. Both depth estimators consume the same
  prepared depth region, so one missing input stamps both rows
  `NO_DEPTH_FRAME` before estimator-specific mathematics runs.
- A color frame at stamp `T` proves the detector had color input at `T`; it does
  not prove matching depth crossed every source, bridge, DDS, callback and
  buffer boundary.
- The old separate aligned-depth producer was measured and removed. Do not
  reintroduce it.

Canonical gap ownership remains in `docs/BACKLOG.md`. Historical evidence is in
`docs/history/refactor_validation.md` and
`docs/history/aligned_depth_coverage.md`.

## Non-negotiable boundaries

1. Preserve exact integer timestamp matching and honest `NO_DEPTH_FRAME`
   accounting. No nearest-frame substitution, tolerance widening, or status
   relabeling.
2. Treat retry, queue depth, executor choice, process priority, DDS transport,
   bridge behaviour and every other proposed cause as hypotheses.
3. Change one experimental axis at a time. Revert experiments that do not move
   the predicted counter.
4. Do not reintroduce a separate depth producer or cadence/latest-frame stage.
5. Do not use `MultiThreadedExecutor` unless traces first show a slow executor
   callback blocking reception. Heavy inference already uses a worker thread.
6. Do not change estimator recipes, gates, camera geometry, scenario content,
   detector settings, or scoring rules during the A/B.
7. Instrument stamp keys and receipt/lookup monotonic times, not image payloads.
   Keep callbacks O(1), records bounded, and measure observer effect.
8. Preserve unrelated work and processes. Inspect ownership before cleanup;
   use isolated output/log paths and a dedicated ROS domain where practical.
9. Do not commit or push unless separately instructed.

## Phase 0: freeze the comparison contract

Before editing code:

1. Record the HEAD, worktree state, installed-workspace provenance, ROS/RMW
   implementation, DDS/SHM settings, ROS domain, host and GPU load, and
   simulation real-time factor.
2. Resolve one scenario YAML and record its hash. Use identical bytes throughout.
3. Use `mask_gate:=box` and `depth_source:=stereoscopic` to exclude SlimSAM and
   monocular inference from the primary diagnosis.
4. Select exactly
   `estimators:=projective_ranging,euclidean_reconstruction`.
5. Keep repeats, settle/capture times, recording mode, RMW configuration and host
   policy identical. Start with the established examples protocol: 5 scenes / 8
   instances, 3 repeats, 2 s settle and 10 s capture. Resolve and record the
   current equivalent if the scenario contract has changed.
6. Run on an otherwise idle host first. Normal-load evidence is a separate axis,
   never a substitute for the matched idle A/B.

Store each run under a unique `/tmp/dynamo-exact-depth-*` root, with `run.json`,
CSVs, summary, ROS logs, launch command, scenario hash and host snapshot.

## Phase 1: reproduce on current source

Run a clean, uninstrumented pre-benchmark. Do not reuse the historical rate as
the current baseline after the D455 and later changes.

Extract separately for both estimators:

- detected-box observations, `OK`, `NO_DEPTH_FRAME`, other reasons and `UNSET`;
- observation coverage and scored instances;
- MAE, median and P95 error;
- per-trial/per-scene misses, not only aggregates;
- wall duration, RTF, depth/color receive rate, process CPU and host/GPU load.

If the loss does not reproduce, repeat once under previously representative
load and stop before changing product code. Report non-reproduction with exact
conditions; do not manufacture load until an old percentage returns.

## Phase 2: add low-overhead causal tracing

Extend diagnostics behind an explicit debug switch or use an external probe,
whichever can observe each boundary with less disturbance. Correlate:

1. Gazebo color/depth source publication stamp and observation time.
2. ROS-side color/aligned-depth bridge output.
3. Arrival in `color_callback` and `depth_callback`.
4. Detection arrival, pending-slot replacement/dequeue, and worker lookup.
5. Exact lookup result and neighbouring buffered stamps.
6. Whether a missing stamp arrives later, and its delay from failed lookup.
7. Duplicate/out-of-order arrival, insertion/eviction, callback gaps and lock
   wait/hold duration.

Use compact counters and bounded per-miss records. First exhaust read-only topic
discovery, bridge logs/statistics and a lightweight observer before changing an
external dependency. Record observer limitations: `ros2 topic hz` measures its
own subscription and alone cannot prove publisher rate or locate a drop.

Repeat the benchmark with tracing and compare coverage, rates, duration and host
load to the uninstrumented pre-run. Reduce instrumentation if it materially
changes the symptom.

## Phase 3: hypothesis ladder

Build a per-stamp accounting table and follow the first boundary where `T`
disappears. This list is required but not exhaustive; add hypotheses when traces
contradict it.

| Hypothesis | Supporting evidence | Cheapest discriminating experiment |
|---|---|---|
| Gazebo omitted depth `T` | color `T` exists; raw depth `T` does not | compare raw source stamp sets in one window |
| Bridge lost depth `T` | raw depth `T` exists; ROS bridge output `T` does not | correlate raw and bridged sets plus bridge health/load |
| DDS/fragmentation loss | bridge emits `T`; subscribers inconsistently receive it; transport evidence agrees | controlled transport/QoS diagnostics, one axis only |
| Subscription history overwrites unread depth | probe sees `T`; callback does not; greater history predicts fewer losses | bounded depth-only QoS A/B, e.g. 5 versus 15/30 |
| Executor/callback starvation | callback gaps and queue age grow while executor is busy | trace executor callbacks before changing executors |
| Shared-lock contention | depth callback materially waits on `processing_lock`; misses follow waits | measure lock timing, then isolate only that ownership |
| Depth arrives after lookup | missing stamp appears later | record later arrival; test bounded event-driven exact deferral |
| Buffer order/duplicate bug | callback sees `T`, lookup record does not | replay captured stamp order through `StampedMessageBuffer` |
| Detection scheduling shifts lookup | overwrite/dequeue timing predicts misses | trace detection receipt through lookup as one timeline |
| Load/debug/recording causes loss | misses track load/subscriber set and vanish when one load is removed | matched headless/recording/subscriber A/B and restoration |
| DDS SHM/UDP mode matters | transport evidence and miss rate change reproducibly | controlled transport-profile A/B after transport evidence |
| Payload size/profile is causal | losses follow depth payload size, not cadence | controlled profile/size diagnostic outside acceptance A/B |

Do not stop at “more load, more misses.” Accept a root cause only when boundary
traces locate it and a targeted intervention moves the predicted result
reproducibly. Where practical, restore the original condition once (A/B/A).

## Phase 4: implement only the evidenced remedy

Examples, not a forced menu:

- **Late arrival:** bounded event-driven exact-stamp deferral; publish the
  original `NO_DEPTH_FRAME` when its deadline expires.
- **Subscription overwrite:** change only depth subscription history/depth;
  preserve sensor-data reliability/durability unless compatibility evidence
  requires otherwise.
- **Lock contention:** separate buffer ownership or snapshot under a narrower
  lock while retaining coherent diagnostics and detection semantics.
- **Buffer defect:** fix it with the captured order as a regression fixture.
- **Bridge/source/transport:** fix that boundary if repository-owned. If
  external, prove the blocker and name its owner instead of masking it downstream.

If evidence names another mechanism, fix that mechanism. Remove unsuccessful
experiments. Retain diagnostics only if bounded, tested, disabled by default and
useful for recurrence.

## Phase 5: regression proof

Add focused coverage appropriate to the cause, including:

- exact equality and refusal of neighbouring frames;
- captured missing/out-of-order/late sequences;
- deadline and one-result-only behaviour if deferral is used;
- explicit QoS ownership if history changes;
- buffer bounds, diagnostic denominators and clean shutdown;
- identical availability status reaching both depth estimators;
- no new depth subscription/model when no depth estimator is selected.

Run focused synchronization, mask-node and launch tests, then the full package
suite. Rebuild before installed-tree/launch checks. Rebuild Graphify after code
changes and run `git diff --check`.

## Phase 6: matched post-benchmark

Run the post benchmark with the exact Phase 0 contract. Compare against a fresh
pre-run from the baseline revision, not only the historical TF report. Prefer
isolated baseline/fix worktrees and record both SHAs. If variance is visible,
use A/B/A or another paired repetition rather than one favorable run.

Report separately for projective and euclidean:

| Metric | Pre | Post | Delta |
|---|---:|---:|---:|
| Detected-box observations | | | |
| `OK` | | | |
| `NO_DEPTH_FRAME` | | | |
| Other misses / `UNSET` | | | |
| Observation coverage | | | |
| Scored instances | | | |
| MAE / median / P95 | | | |
| First valid result latency, if traced | | | |

Also report per-scene distribution, exact-stamp proof, RTF, wall duration,
CPU/GPU/load, diagnostics/recording state, artifact paths and provenance.

### Acceptance gate

Complete only when:

1. Stamp-level evidence plus a discriminating intervention identifies the
   responsible boundary, or proves an exact external blocker.
2. The current-source pre-run reproduced the symptom.
3. Post shows a reproducible `NO_DEPTH_FRAME` reduction in both rows. Zero is
   preferred but not assumed; trace every residual.
4. Every consumed depth stamp still equals its detection stamp.
5. No `UNSET`/other-miss, scored-instance, material accuracy, RTF or latency
   regression is hidden by the coverage gain.
6. Focused/full tests, rebuilt installed checks and `git diff --check` pass.

If the cause is external and no repository fix is justified, do not fake a post
improvement. Deliver the proof, eliminated options and next owner; keep the gap
open.

## Documentation and final report

On success, remove the item from `docs/BACKLOG.md`; put current behaviour in the
target-localization reference and dated evidence in `docs/history/`; update
`docs/ISSUES.md` only for a reusable operational failure; move rejected options
with evidence to `docs/do_not_try_again/`; then mark or retire this plan.

The final report must state the proven boundary/cause, evidence against serious
alternatives, exact fix and preserved behaviour, test results, pre/post artifact
paths and provenance, the two-estimator comparison, remaining limitations, and
uncommitted/unverified work.

Stop after closing or precisely blocking this assignment. Do not begin
AutoVision/process work, broad worker scheduling, physical-camera validation,
isolation tuning, or exploration redesign.
