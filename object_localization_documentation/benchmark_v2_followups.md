# Benchmark v2 — Follow-Ups and Run Instructions

Status notes for the merged benchmark stack (merge `20086ca`: scenario-spec
multi-instance redesign + per-estimator miss-reason attribution, both on
`g1-distance-benchmarks`). The scene set is
`config/benchmark_scenarios_full.yaml` (88 scenes, 20 of them multi-robot;
sampler seed 20260725, extended 2026-08-23 with the in-box clutter family).
It is now the packaged default: the hand-written 15-pose grid
(`benchmark_scenarios.yaml`) and the older 30-scene `full` set were deleted in
its favour on 2026-08-24, and the `_v2` file was renamed onto `full`. Its
visual reference is
[benchmark_scenarios_v2_gallery.html](benchmark_scenarios_v2_gallery.html).

## Resolved: per-detection miss-reason tallies (2026-08-22)

Statuses are now counted **per detected box**, not per frame:

- Every mask-estimator value on the wire carries a `MissReason` status per
  detection (`*_status` arrays on `G1Measurements`); legacy estimators get a
  coarse OK/UNSET inferred from finiteness, per detection.
- `compute_status_histogram` iterates `event.detections` via
  `alignment.detection_status`, so a 2-robot frame contributes both boxes.

This fixed a defect that made the tallies useless for exactly the scenes v2
added. The histogram previously skipped any frame where `count != 1`, so a
multi-robot scene either contributed **nothing** (the detector reliably saw
both robots, so no frame qualified) or contributed **only** its `count == 1`
frames — the frames where the detector had already lost one robot, and no
others. Single-robot scenes are unchanged: one box per frame means boxes and
frames are the same number.

Reasons also moved to where they can be read per scene: the trial CSVs carry
a `miss_reason` column alongside `outcome`, populated for `no_value` misses
(the case where a `MissReason` exists — `gate_miss` and `detector_miss` are
already fully named by `outcome`). `dominant_miss_reason` in `reduction.py`
is the shared rule: a specific code beats `UNSET`, since `UNSET` means the
frame never reached that node and is usually the bulk of the tally.

### Still open: attribution to a specific instance

`miss_reason` on a trial row is per (trial, estimator), so in a multi-robot
scene both instances show the same dominant reason. Making it truly
per-instance means carrying per-detection statuses through the association in
`scoring.py` — but that only resolves for detections that *matched*, and a
`no_value` instance by definition has none. Per-trial-per-estimator is the
honest granularity until that gap is designed away.

Value if solved: the v2 families are banded occlusion experiments
(`interocc_*`, `objocc_*`, `objpartial_*`, traps). Per-instance reasons turn
"the far robot missed" into "the far robot missed because SEGMENTATION_EMPTY
at visibility 0.03 but TOO_FEW_RAYS at 0.15" — the per-band failure-mode
breakdown the set was designed to expose.

### Resolved: per-estimator usable events (2026-07-26)

The all-estimator common-event gate is gone: each estimator scores on its own
usable events (`usable_events_by_estimator`), and
`missed_instance_count` in the summary is now genuinely per estimator
(detector-level misses + estimator-level misses, reasons in the summary's `reason_histogram`).
Collage panels for a blinded estimator show the dominant miss reason.
Zero-detection trials (e.g. a full occluder) score as all-instances-missed
for every estimator instead of vanishing as skips — skips now mean
infrastructure failure only. First
run after this change is a NEW baseline — medians rest on per-estimator
event sets (supersets of the old intersection), so numbers are not comparable
to earlier runs.

### Other open items

- **Sim depth-coverage gap (known, diagnosed):** in sim expect
  `projective_ranging` / `euclidean_reconstruction` at only ~20 % coverage
  with `NO_DEPTH_FRAME` dominating the summary `reason_histogram`, while `polar_profiling`
  sits near 100 %. This is the CPU-starved `aligned_depth_node` sim artifact
  — NOT a v2 regression and largely absent on real hardware. Diagnosis +
  fix options: [aligned_depth_coverage.md](aligned_depth_coverage.md);
  troubleshooting entry in `ISSUES.md`. Fewer usable events per trial also
  means depth-path trial medians rest on a thinner sample in sim.
- Launch default `estimators` still excludes the mask stack — pass the full
  list explicitly (below) or bump the default.
- `mask_gate` defaults to `box`; silhouette (SlimSAM) is the primary path —
  pass `mask_gate:=silhouette` for mask-estimator runs.
- A/B rule: never regenerate `benchmark_scenarios_full.yaml` between estimator
  comparisons — same YAML, different `estimators:=` lists. Fresh sampler
  seeds are coverage expansion, not comparison baselines.
- Probe scenes (`single_facing_01`, `objpartial_01`, `far_mid_01`) carry
  `repeats_override: 8`; run with `repeats:=1` so probes measure the noise
  floor and everything else spends budget on pose diversity.

## Output reference

### What an "event" is

One event = one aligned measurement **frame**: a unique key of (frame_id,
stamp, detection count, bbox set). The camera, lidar, and mask nodes'
messages for the same frame merge into a single event. Events accumulate
along the **time** axis, not the robot axis:

- Per trial (one spawn + settle + capture window) the pipeline produces
  roughly 20–40 events at the sim's effective detector rate.
- A multi-robot scene does NOT multiply events: one frame is one event
  carrying up to N detections; the association step splits those detections
  across ground-truth instances for scoring.
- `raw_events` in a skip log line counts every captured event, including
  frames with zero detections. The trial CSVs' `frames_captured` is the same
  number, per trial.
- An **observation** is one detected box, so it IS multiplied by the robot
  axis: a 2-robot frame is one event but two observations. Accuracy and miss
  columns count instances; the `observations` group counts boxes.

### `run.json` (the run-level numbers, for machines)

Three top-level objects. `run` carries the provenance:

| Field | Meaning |
|---|---|
| `label` | The run timestamp, matching the folder prefix. |
| `started` | Local wall-clock start time. |
| `commit` / `branch` | The checkout that produced the run; `null` when git is unavailable. |
| `uncommitted_files` | Count of modified files — a **number**, so "runs from a clean tree" is a filter. Non-zero means the commit alone will not reproduce the run. |
| `scenario` | Absolute path of the scenario YAML. |
| `scenes`, `instances`, `trials_included`, `trials_skipped` | Run size. A skipped trial is an infrastructure failure, not a miss. |

`parameters` is every parameter the runner was launched with, read off the node
so it cannot drift as parameters are added.

`estimators` is one entry each:

| Field | Meaning |
|---|---|
| `key` | Stable identifier (`polar_profiling`) — query on this. |
| `display_name` | What the report prints (`box-gated polar profiling`). |
| `trial_count` | Every (instance, trial) pair this estimator was asked about — misses included, since each one emits a row. |
| `scored_count` | Rows that produced a number. The gap from `trial_count` is the miss total. |
| `mean_abs_error_m` / `median_abs_error_m` / `p95_abs_error_m` | Over this estimator's scored rows; P95 is the tail the mean hides. |
| `mean_rel_error` | Mean of `abs_error / true_distance`. |
| `missed.total` | Sum of the three below — derived, never counted separately. |
| `missed.detector` | Instance never matched by any estimator — occluded or undetected, nobody had a chance at it. |
| `missed.gate` | This estimator produced values, but none landed within the association gate of the instance: a gross localization error. Only reachable in multi-robot scenes, where the gate runs. |
| `missed.no_value` | Instance was located by someone, but this estimator produced nothing. The trial CSV's `miss_reason` names why, per scene. |
| `extra_detection_count` | Run-level: peak count of detections in a frame that NO estimator could attribute to a ground-truth instance. A detector concept — a badly-localizing estimator does not inflate it. |
| `observations.total` | Every detected box across every capture window. One 2-robot frame contributes 2; single-robot scenes have one box per frame, so this equals the frame count there. |
| `observations.ok` | Boxes where this estimator delivered a value (`MissReason.OK`). |
| `observations.coverage` | `ok / total` — how often the estimator answers at all, independent of accuracy. |
| `reason_histogram` | `reason → count` naming why the non-OK boxes failed. Codes glossary: `object_localization_pipeline.md` → Glossary → "Miss-reason codes". |

Accuracy and miss fields are per **instance**; the `observations` group is per
**box**. Keeping them as separate objects is why this is JSON: the file it
replaced, `comparison_summary.csv`, put both granularities in one flat row and
embedded the reason histogram as JSON inside a cell. (It in turn had absorbed a
separate `coverage.csv`, which carried nothing the summary could not.)

Reading it together with the trial CSVs: `run.json` answers "how good is this
estimator, how often does it answer, and how many instances did it miss"; the
trial rows answer "which scene, which instance, and why" — every (instance,
estimator) pair gets a row, including misses, carrying `outcome`,
`miss_reason`, `usable_aligned_events` and `frames_captured`. `summary.md`
renders the same numbers for reading, rounded.

## Running the v2 benchmark from zero

From a fresh terminal on the machine (main checkout, branch
`g1-distance-benchmarks`):

```bash
cd /home/stefi/DyNAMO/DyNAMO-Ridgeback

# 1. Kill leftovers from any previous sim/benchmark run
bash cleanup.sh

# 2. Toolchain + workspace build (build is a no-op if already current)
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ridgeback_autonomy --symlink-install
source install/setup.bash

# 3. Full run: all 8 estimators, v2 scene set, silhouette mask gate
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py \
  repeats:=1 \
  estimators:=rgb,sensor_depth,depth_anything,pointcloud,lidar,projective_ranging,euclidean_reconstruction,polar_profiling \
  mask_gate:=silhouette
```

Results land in `benchmark-results/<timestamp>_<scenario>[_<gate>][_<depth_source>]/`
(e.g. `20260726_115556_v2_silhouette_stereoscopic`): **`summary.md`** (start here —
the readable report), per-estimator trial CSVs, `run.json` (the same run-level
numbers for machines, plus provenance and parameters), and per-scene collages
under `images/`.

`summary.md` renders the same numbers in narrow tables —
accuracy, reliability, why boxes went unmeasured, and a per-scene pivot with
one column per estimator. That last table is the one the CSVs cannot give
without a manual join, and it is where a single estimator failing while its
peers score is visible at a glance. The CSVs remain the machine-readable
source for diffing runs.

Notes:

- 89 trials × (settle 2 s + capture 10 s + spawn/despawn) ≈ expect a run in
  the ~30–45 min range; leave the sim undisturbed.
- The sim is system-wide (shared ROS domain / Gazebo) — don't start a second
  exploration or benchmark session in parallel.
- Subset runs for quick checks: `estimators:=lidar,projective_ranging` etc.;
  same `scenario:=` path keeps results comparable.
