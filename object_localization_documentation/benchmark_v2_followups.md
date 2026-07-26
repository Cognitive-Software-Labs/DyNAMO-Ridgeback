# Benchmark v2 — Follow-Ups and Run Instructions

Status notes for the merged benchmark stack (merge `20086ca`: scenario-spec
multi-instance redesign + per-estimator miss-reason attribution, both on
`g1-distance-benchmarks`). The scene set is
`config/benchmark_scenarios_v2.yaml` (68 scenes, 92 ground-truth instances,
89 trials per run, sampler seed 20260725); its visual reference is
[benchmark_scenarios_v2_gallery.html](benchmark_scenarios_v2_gallery.html).

## Open follow-up: per-instance miss-reason attribution

### Current semantics (trial-level)

Miss-reason attribution and the v2 multi-instance scoring were developed on
separate branches and merged without changing either one's semantics:

- Every mask-estimator value on the wire carries a `MissReason` status per
  detection (`*_status` arrays on `G1Measurements`); legacy estimators get a
  coarse OK/UNSET inferred from finiteness.
- The benchmark tallies statuses **per estimator per captured event**, taking
  the **first detection's** status (`extract_estimator_statuses`), into
  `coverage.csv` and the `reason_histogram` summary column.
- Per-instance scoring (`association.py` / `scoring.py`) associates
  per-detection measurements to ground-truth instances — but the reason
  tallies never pass through that association.

Consequence: in a multi-robot scene the histogram describes "the frame's
first detection", not "instance k". A `interocc_neartotal` miss shows up as
a missed instance in the summary, but *why* it missed (empty segmentation vs
too-few-points vs no-depth-match) is not attributable to that specific
robot.

### What the upgrade looks like

Small, contained; all pieces already exist:

1. `alignment.py` — statuses are per-detection arrays on the message; store
   them per detection in the event's detection table (a `status` per
   estimator on each `Detection` row) instead of only the first-detection
   scalar map.
2. `scoring.py` — the assignment already maps detection index → GT instance;
   carry the per-detection statuses through that mapping.
3. `summary.py` — add a per-instance reason histogram, e.g. a
   `coverage_by_instance.csv` with `(scene, instance, estimator, events, ok,
   reason_histogram)`, leaving the existing trial-level `coverage.csv`
   unchanged.
4. Unassigned detections' statuses become attributable to the
   `extra_detection` bucket for free.

Value: the v2 families are banded occlusion experiments
(`interocc_*`, `objocc_*`, `objpartial_*`, traps). Per-instance reasons turn
"the far robot missed" into "the far robot missed because SEGMENTATION_EMPTY
at visibility 0.03 but TOO_FEW_RAYS at 0.15" — the per-band failure-mode
breakdown the set was designed to expose.

### Resolved: per-estimator usable events (2026-07-26)

The all-estimator common-event gate is gone: each estimator scores on its own
usable events (`usable_events_by_estimator`), and
`missed_instance_count` in the summary is now genuinely per estimator
(detector-level misses + estimator-level misses, reasons in `coverage.csv`).
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
  with `NO_DEPTH_FRAME` dominating `coverage.csv`, while `polar_profiling`
  sits near 100 %. This is the CPU-starved `aligned_depth_node` sim artifact
  — NOT a v2 regression and largely absent on real hardware. Diagnosis +
  fix options: [aligned_depth_coverage.md](aligned_depth_coverage.md);
  troubleshooting entry in `ISSUES.md`. Fewer usable events per trial also
  means depth-path trial medians rest on a thinner sample in sim.
- Launch default `estimators` still excludes the mask stack — pass the full
  list explicitly (below) or bump the default.
- `mask_gate` defaults to `box`; silhouette (SlimSAM) is the primary path —
  pass `mask_gate:=silhouette` for mask-estimator runs.
- A/B rule: never regenerate `benchmark_scenarios_v2.yaml` between estimator
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
  frames with zero detections.

### `comparison_summary.csv` (one row per estimator)

| Column | Meaning |
|---|---|
| `estimator` | Display name of the estimator row. |
| `trial_count` | Scored (instance, trial) rows for THIS estimator. Per-estimator since the usable-events change: an estimator blinded in a scene simply has fewer rows. |
| `mean_abs_error_m` | Mean of `abs(estimate − true)` over this estimator's scored rows. |
| `median_abs_error_m` | Median of the same. |
| `p95_abs_error_m` | 95th percentile of the same — the tail the mean hides. |
| `mean_rel_error` | Mean of `abs_error / true_distance`. |
| `missed_instance_count` | Per-estimator: detector-level misses (instance never matched by any detection) + estimator-level misses (instance matched, but this estimator produced no value in any frame — reason in `coverage.csv`). |
| `extra_detection_count` | Scene-level (same value in every row): peak count of detections in a frame that matched no ground-truth instance. A detector concept, not per-estimator. |

Reason histograms live in `coverage.csv` only (single source of truth).

### `coverage.csv` (one row per estimator)

| Column | Meaning |
|---|---|
| `estimator` | Display name. |
| `events` | Denominator: captured events with exactly ONE detection (`count == 1`), summed over every trial's capture window — including trials later skipped. Same value for all rows. No-detection frames and multi-robot frames are excluded (a frame-level status can't describe two robots; per-instance attribution is the open follow-up above). |
| `ok` | Events where this estimator delivered a value (`MissReason.OK`). |
| `coverage` | `ok / events` — how often the estimator answers at all, independent of accuracy. |
| `reason_histogram` | JSON `reason → count` naming why the non-OK events failed. Codes glossary: `object_localization_pipeline.md` → Glossary → "Miss-reason codes". |

Reading the two together: `coverage.csv` answers "how often does this
estimator produce anything"; `comparison_summary.csv` answers "how good is
it when it does, and how many instances did it miss". The per-trial CSVs'
`usable_aligned_events` column is the per-trial, per-estimator slice of the
same coverage idea — the frames that actually fed that trial's median.

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
  scenario:=/home/stefi/DyNAMO/DyNAMO-Ridgeback/src/ridgeback_autonomy/config/benchmark_scenarios_v2.yaml \
  repeats:=1 \
  estimators:=rgb,sensor_depth,depth_anything,pointcloud,lidar,projective_ranging,euclidean_reconstruction,polar_profiling \
  mask_gate:=silhouette
```

Results land in `benchmark-results/<timestamp>/`: per-estimator trial CSVs,
`comparison_summary.csv` (now with missed/extra counts and
`reason_histogram`), `coverage.csv`, and per-scene collages under `images/`.

Notes:

- 89 trials × (settle 2 s + capture 10 s + spawn/despawn) ≈ expect a run in
  the ~30–45 min range; leave the sim undisturbed.
- The sim is system-wide (shared ROS domain / Gazebo) — don't start a second
  exploration or benchmark session in parallel.
- Subset runs for quick checks: `estimators:=lidar,projective_ranging` etc.;
  same `scenario:=` path keeps results comparable.
