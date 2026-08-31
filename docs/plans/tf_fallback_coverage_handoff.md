# Handoff: fix blocking TF fallback in target localization

> **STATUS 2026-08-31: DONE.** The fix is implemented in
> `common/tf_utils.lookup_transform_components` (two passes over the same
> candidates: zero timeout first, the original bounded wait only if nothing was
> buffered), covered by `test/test_tf_utils.py`, and validated on the normal
> namespaced default. Mask `UNSET` went 396 → **0** and both depth rows went
> 45.68% → **90.23%** coverage, with MAE identical to six decimals. Canonical
> write-ups now live in [ISSUES.md](../ISSUES.md) and
> [refactor_validation.md](../history/refactor_validation.md) — read those, not this file.
> The separate `NO_DEPTH_FRAME` loss (73/747, 9.77%) is still open. Everything
> below is the original diagnosis, kept for its measurement detail.

Date: 2026-08-31. Diagnosis complete; product fix not implemented.

## Assignment and boundaries

Fix the shared transform lookup so an already available fallback does not wait
behind a timeout on an unavailable configured frame. Add regression coverage
and verify with the normal benchmark launch, without a frame-name workaround.

This is a confirmed pre-existing performance defect, not a reproduced regression
from the completed ownership/naming refactor. Keep the smaller, unresolved
exact-depth-input loss separate; do not claim that this fix solves both.

- Repository: `/home/stefi/DyNAMO/DyNAMO-Ridgeback`.
- Starting snapshot: branch `g1-distance-benchmarks`, HEAD `f16afd0`.
- Product code was unchanged by the investigation. This handoff is documentation
  only; no fix, commit, new task, or background agent was started with it.
- An unrelated, untracked
  [ROI handoff](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/plans/roi_mask_handoff.md)
  was already present. Preserve it and any concurrent work; recheck Git status.
- All investigation ROS/Gazebo processes were stopped at the end of diagnosis.
  Verify current host state rather than assuming that remains true.
- Read [AI_CONTEXT.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/AI_CONTEXT.md) and
  [GRAPH_REPORT.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/graphify-out/GRAPH_REPORT.md)
  before implementation. Use the Graphify wiki if one now exists.

## Confirmed mechanism

The benchmark defaults to `r100_0001/robot/base_link`. In the tested simulation
TF graph that frame is unavailable, while `base_link` is available. A ROS topic
namespace does not establish what frame IDs exist inside TF messages.

`lookup_transform_components()` tries candidates in configured-frame-first
order, giving each lookup a 0.5-second timeout. It therefore waits on the
nonexistent namespaced frame before successfully looking up `base_link`.
`last_fallback_frame` only suppresses repeated warnings; it does not prevent
that wait on subsequent calls.

The mask node calls this helper for every detected batch. Its pending detections
slot is latest-wins. At approximately 4.2 incoming batches/s, a worker occupied
for at least 0.5 seconds per detected batch overwrites pending work and processes
only approximately 2 batches/s. Benchmark observations without a matching mask
result remain `UNSET`.

A live probe of the existing helper, carrying fallback state across calls,
measured 502.8–512.0 ms per namespaced lookup versus 0.021–0.036 ms when directly
requesting `base_link`. All ten calls returned identical rotation/translation.
The probe used latest TF; the production mask caller uses the detection stamp.
The nonexistent primary-frame delay applies in both cases. Preserve the
production caller's exact timestamp in the fix.

The shared helper is byte-for-byte unchanged from pre-refactor `ebf3a02`.
Algorithm-body comparisons also found the measurement worker, buffers,
estimators, and benchmark accounting unchanged apart from renamed identifiers,
moved constants/helpers, and logging text.

## Relevant code and consumers

Line numbers below are from `f16afd0`; locate by symbol if they move.

| Location | Role |
|---|---|
| [tf_utils.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/common/tf_utils.py:31) | Primary fix: `lookup_transform_components`; timeout at line 8, candidate generation at line 11, timed loop at line 41. |
| [mask_measurement_node.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/mask_measurement_node.py:462) | `detections_callback` overwrites the pending slot; `processing_loop` consumes it at line 487. `camera_extrinsic_for_batch` calls the helper at line 873; `scan_points_for_batch` also calls it at line 900. |
| [pointcloud_measurement_node.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/pointcloud_measurement_node.py:235) | Same helper in the separate pointcloud-processing worker. Its measurement worker can publish from the latest transformed cloud, so its old coverage was already high. |
| [overlay_node.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/overlay_node.py:312) | Same helper for scan projection. Preserve behavior for this consumer too. |
| [visualization_node.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/ridgeback_autonomy/perception/target_localization/visualization_node.py:241) | Separate world-pose lookup, already nonblocking. Do not fold it into a blocking helper or broaden this patch into visualization cleanup. |
| [target_benchmark_config.launch.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/launch/target_benchmark_config.launch.py:287) | Configured namespaced default; normal-launch validation must retain this default. |

## Evidence preserved here

Both depth estimators had identical coverage counters in every run below.
Observations count detected boxes, not frames or ground-truth instances.

| Run | Repeats per scene | Depth OK / observations | Coverage | NO_DEPTH_FRAME | UNSET |
|---|---:|---:|---:|---:|---:|
| Fresh old build `ebf3a02` | 3 | 334 / 757 | 44.12% | 23 | 400 |
| Refactored `f16afd0`, normal default | 3 | 344 / 753 | 45.68% | 13 | 396 |
| `f16afd0`, diagnostic `base_frame:=base_link` | 1 | 228 / 246 | 92.68% | 18 | 0 |

The direct-frame diagnostic also yielded pointcloud 246/246 (100%) and polar
205/246 (83.33%), with 41 `TOO_FEW_RAYS_SELECTED` and no `UNSET` observations.
Mask throughput rose to approximately 4.2 batches/s. This was a launch-parameter
experiment, not a source fix or a recommendation to hardcode the bare frame.

Accuracy was unchanged at six decimals:

| Estimator | MAE (m) | Scored instances per single scenario repeat |
|---|---:|---:|
| Pointcloud | 0.129655 | 6/8 |
| Projective ranging | 0.055221 | 6/8 |
| Euclidean reconstruction | 0.056944 | 6/8 |
| Polar profiling | 0.081697 | 5/8 |

There were two detector misses per repeat for every estimator, plus one
sparse-ray/no-value outcome for polar. No extra detections. All scenes completed;
no trials were skipped. Three-repeat runs therefore scored 18/24 or 15/24.

Controls: identical five-scene examples YAML, 2-second settle, 10-second capture,
all four estimators, box masks, stereoscopic depth, default isolation recipes,
unlimited mask-depth gate, RViz/video enabled, ROS domain 42, serial runs with
no extra image subscribers. A short TF-only probe ran during the current run.
The old and current runs each processed 357 mask box-observations; their OK-count
difference was missing depth availability, not fewer mask executions.

Provenance caveats:

- The old build used its own package/message install in
  `/tmp/dynamo-coverage-baseline-ebf3a02`, sharing external dependencies and the
  existing model venv. Its sole untracked item was a venv symlink. A symlink in
  its installed world directory allowed the renamed but physically unchanged
  world to work with the current Clearpath launch. Tracked old source was clean.
- Current-run provenance reported one untracked file: the unrelated ROI handoff.
- The original one-repeat pre-refactor measurement was 45.02%; earlier new-build
  runs were 40.71% and 40.08%. Original baseline temporary artifacts disappeared.
  The matched reruns did not reproduce a fixed refactor regression; do not turn
  that into a claim of formal performance equivalence for all workloads.
- An exploratory old-build run with extra image subscriptions measured 38.76%.
  Exclude it from the controlled comparison because observation load can perturb
  reception. Do not attach image-rate monitors during the acceptance benchmark.

## Suggested implementation approach

Keep the fix at the shared helper, with a small, tested change:

1. Try the existing candidate frames in their existing priority order without
   waiting. Return immediately when a transform at the requested stamp exists.
   Direct zero-timeout lookups can avoid an unnecessary availability-check-plus-
   lookup pair; handle normal transform exceptions.
2. If neither candidate is immediately available, retain the existing bounded
   waiting/error behavior initially. If changing that policy is necessary,
   explain and test it explicitly; the current 0.5-second constant is per
   candidate, not a single global deadline.
3. Keep the return shape `(rotation, translation, last_fallback_frame)`, numeric
   conversion, and warning suppression compatible. Prefer a valid configured
   frame even if an earlier call used a fallback. Do not permanently cache the
   chosen fallback or a transform matrix; availability and stamped poses change.
4. Add focused tests and register any new test file in
   [CMakeLists.txt](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/CMakeLists.txt:134).
   [test_imports.py](/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/test/test_imports.py:66)
   checks that all Python test files are registered with colcon.

Do not change estimator math, depth matching, buffer sizes, worker scheduling,
QoS, model settings, ROS topic contracts, or process ownership in this patch.
Do not globally replace configured frames with `base_link`. Preserve hardware
namespace support; the physical robot was not tested in this investigation.

### Deterministic regression tests

Use a fake/mock TF buffer that records target/source/stamp/timeout; avoid a
wall-clock performance assertion as the only guard.

- Missing primary plus available fallback: first and repeated calls use no
  positive timeout before returning the fallback. Verify warning suppression.
- Both candidates available: configured primary wins.
- Primary becomes available after an earlier fallback: primary wins again.
- Already-unqualified frame: candidate de-duplication avoids duplicate work.
- Neither available: bounded existing wait/error behavior, no identity transform,
  stale cached transform, or fabricated result. Cover no valid candidates too.
- Every attempted lookup receives the exact caller-supplied source and timestamp;
  test with a nonzero stamp, not only latest TF.
- Rotation/translation and return-state semantics remain compatible.

## Validation and acceptance

The last recorded pre-fix validation was 541 tests passing, including all 36
registered colcon test groups. This is historical evidence, not a substitute
for rerunning after the fix. From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select ridgeback_autonomy --symlink-install
source install/setup.bash
python3 -m pytest -q src/ridgeback_autonomy/test
colcon test --packages-select ridgeback_autonomy --event-handlers console_direct+
colcon test-result --test-result-base build/ridgeback_autonomy --verbose
git diff --check
```

Run focused tests before the full suite. Rebuild after import/module changes so
installed ROS artifacts cannot invalidate the result. Never build or run a test
suite concurrently with a timing benchmark.

Benchmark the fixed helper with the normal configured frame, **without** adding
`base_frame:=base_link`. First ensure the host has no conflicting simulation;
only stop exact processes verified to belong to this task. The investigation
used targeted cleanup because the broad cleanup script can kill unrelated work
and remove shared locks. Do not run it blindly in a shared session.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
tf_fix_output=$(mktemp -d /tmp/dynamo-tf-fix-benchmark-XXXXXX)
tf_fix_ros_logs=$(mktemp -d /tmp/dynamo-tf-fix-ros-XXXXXX)
ROS_DOMAIN_ID=42 ROS_LOG_DIR="$tf_fix_ros_logs" \
ros2 launch ridgeback_autonomy target_distance_benchmark.launch.py \
  world:=target_distance_calibration \
  scenario:=/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/config/benchmark_scenarios_examples.yaml \
  estimators:=all mask_gate:=box depth_source:=stereoscopic \
  repeats:=3 settle_sec:=2.0 capture_sec:=10.0 \
  depth_match_debug:=true shutdown_on_complete:=true \
  output_dir:="$tf_fix_output"
```

Acceptance:

- The normal default no longer imposes the repeated half-second fallback delay
  or approximately 2 Hz ceiling during detected scenes.
- Demonstrate the large coverage improvement seen in the direct-frame probe:
  mask `UNSET` should be near zero, while explicit input misses stay visible.
  Treat 92.68% as measured reference evidence, not an exact golden threshold.
  If coverage remains low, separate UNSET from NO_DEPTH_FRAME before judging why.
- All five scenes complete with unchanged scored-instance outcomes and MAE at
  the displayed precision. Inspect `run.json`, status histograms, and collages,
  not just launch exit status. Report exact values and run provenance.
- Verify exploration still launches and its overlay/HUD/markers behave correctly
  if exercising affected shared consumers. Do not claim non-empty target
  verification from an empty-detection exploration run; do not add truth/error
  UI to exploration.
- Stop the owned runtime and verify host process exit. Gazebo parent shutdown
  sometimes left server/GUI children alive despite launch exiting; inspect their
  parent/start time and `ROS_LOG_DIR` before targeted signals. Never reuse old PIDs.
- After code edits, rebuild the graph using the repository's canonical helper:
  `bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"`.

### Documentation to reconcile after the fix

Record the root cause and validated behavior in
[ISSUES.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/ISSUES.md), the canonical
troubleshooting owner. Update the open coverage item in
[refactor_validation.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/history/refactor_validation.md)
with evidence, separating refactor parity, the TF improvement, and residual loss.

The `base_frame` paragraph in
[AI_CONTEXT.md](/home/stefi/DyNAMO/DyNAMO-Ridgeback/AI_CONTEXT.md:90) claims the
bare frame is unavailable under a namespace. That blanket claim contradicts the
observed benchmark graph. Correct the explanation to distinguish configured
frame IDs from topic namespaces while preserving the shared factory's explicit
`base_frame` contract. Do not replace it with the opposite blanket claim that
every deployment uses `base_link`. Keep detailed runtime evidence in ISSUES.

## Separate unresolved item: exact-depth availability

The direct-frame probe still had 18/246 `NO_DEPTH_FRAME` box-observations (7.32%).
Its cumulative lookup diagnostics had 18 misses: 15 stamps inside the buffered
range, three newer, none older/empty. The normal current run had 32 lookup misses:
31 inside and one newer. These diagnostics include empty detections and time
outside capture windows; their units differ from per-box `run.json` histograms.
`color_rx=0` is expected in box/stereo because this node does not subscribe to RGB.

Earlier silhouette diagnostics found mostly late-arrival misses; that does not
establish the cause of this box/stereo workload. An extra best-effort observer
also missed 40 of 44 queried NO_DEPTH_FRAME stamps, but observer loss/load means
this does not isolate Gazebo generation, image bridging, DDS, or the callback.

If subsequently assigned this remaining gap, correlate generation/publication,
arrival and lookup stamps with low-overhead instrumentation. Only then choose
bounded exact-stamp deferral, reception/queue work, or another evidenced fix.
Never switch to nearest-frame depth matching, widen tolerance, or alter accuracy
and coverage accounting to hide the misses. Do not require solving this separate
loss mechanism before delivering the proven TF fix.

## Temporary supporting artifacts

Key numbers are preserved above so this handoff survives temporary-file cleanup.
These local artifacts existed when the handoff was written; check before using:

- [Old-build controlled run](/tmp/dynamo-coverage-baseline-clean/20260831_125911_examples_box_stereoscopic/run.json)
- [Current controlled run](/tmp/dynamo-coverage-current-clean/20260831_125440_examples_box_stereoscopic/run.json)
- [Direct-frame probe run](/tmp/dynamo-coverage-base-frame-probe/20260831_130343_examples_box_stereoscopic/run.json)
- [Current mask log](/tmp/dynamo-coverage-current-clean-logs/python3_112183_1788173680028.log)
- [Direct-frame mask log](/tmp/dynamo-coverage-base-frame-probe-logs/python3_119831_1788174223255.log)
- [Full investigation notes](/tmp/dynamo-coverage-investigation-20260831.md)
- [TF timing probe](/tmp/dynamo-tf-latency-probe.py)
- [AST comparison script](/tmp/dynamo-compare-hotpaths.py)

Deliver the focused fix, registered tests, updated canonical documentation,
fresh benchmark evidence, and an explicit residual-depth-loss note. Leave
unrelated ROI work and the completed ownership refactor intact.
