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

### Other open items

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
