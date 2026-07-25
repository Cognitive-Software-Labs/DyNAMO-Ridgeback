# G1 Distance Benchmark — Scenario-Spec Redesign

Record of the benchmark redesign carried out on branch **`benchmark-redesign`**:
what it does, why, how it's built, how to run it, and how it was validated.

## 1. Motivation

The original benchmark ([G1_DISTANCE_BENCHMARKING.md](G1_DISTANCE_BENCHMARKING.md))
spawned **one** G1 per trial on a hardcoded `5 forward × 3 lateral` grid, facing
the robot, and scored only single-detection frames. The goal of this work was to
vary the scene along several axes:

- **instance count** — one or many robots in the same frame
- **per-instance pose** — position (x, y) + yaw
- **object occluders** — real furniture (hospital bed, privacy curtain) in the room
- **inter-robot occlusion** — one robot blocking another

**Key discovery that made this cheap:** the perception pipeline (detector → the
three measurement nodes → messages) is *already* fully multi-instance — the
detector emits N detections, every node loops over all of them, and the
`G1Detections`/`G1Measurements` messages carry N-element arrays. The single-target
assumption lived **only in the benchmark scoring layer** (`count==1` gate,
scalar collapse, single ground-truth pose). So no perception code changed.

## 2. Design principles

- **Declarative scenario spec** — a YAML lists scenes; each scene declares its
  robots (`x, y, yaw`) and objects (`model, x, y, yaw`). The runner iterates
  scenes × repeats. Replaces the hardcoded grid.
- **Sensor-only association (load-bearing).** The estimators run on the real
  Ridgeback, which never knows the true poses. So the sim's true poses are used
  for **exactly two** benchmark-only things — computing each robot's
  ground-truth distance, and the final scoring assignment (mapping a
  sensor-produced instance to the true robot it should be graded against). Truth
  never helps *detect, separate, or locate* instances.
- **Single-robot path kept byte-identical** — a one-robot scene reproduces the
  old grid benchmark exactly (regression guard).
- **Metric = distance MAE**, plus a **missed-instance count** so occluded /
  undetected robots are counted, not silently dropped (occlusion cannot flatter
  the MAE).
- **Visualization preserved** — per-trial collage images still save; the overlay
  node still launches.

## 3. What was implemented (by stage)

| Stage | Commit | Content |
|---|---|---|
| 0 | `bbdcde8` | Scenario loader `benchmarking/scenarios.py` + YAMLs; `scenario` launch arg |
| 1 | `bbdcde8` | Scene-driven runner: `spawn_model`/`despawn_model`/`spawn_scene`, `compute_scene_ground_truth` (per-instance GT) |
| 2a | `9d4a9ef` | Sensor-only `benchmarking/association.py` |
| 2b | `2b31cf6` | `benchmarking/scoring.py`, per-detection alignment table, drop `count==1`, summary `missed`/`extra` columns |
| 3 | `9c0c376` | `sim/models/hospital_bed` + `privacy_curtain` occluder models |
| 4 | `38298f2` | Per-instance collage annotations (`#i e<est>/t<true>`, `extra`, `missed: N`) |
| — | `c4d2048`, `cb0ffe9`, `de4c236` | Full 30-scene scenario set + object-yaw variety + partial-occlusion group |
| merge | `ba22687` | Merged `g1-distance-benchmarks` in (see §7) |

### Key modules

- **`benchmarking/scenarios.py`** — pure loader: `RobotSpec`/`ObjectSpec`/`Scene`
  dataclasses, `load_scenarios(path)`, validation (raises `ValueError` with the
  scene id on bad input).
- **`benchmarking/association.py`** — `assign_to_ground_truth(instances, gts,
  max_gate_m=1.5)`. Greedy nearest-estimate match (planar, or 1-D distance
  fallback) with a sanity gate. Unmatched GT → `missed_gt`; unmatched detection
  → `extra_detections`. Pure, no ROS.
- **`benchmarking/scoring.py`** — `score_scene(events, gt_instances, estimators)`:
  runs the association **once per frame** (sensor-only), aggregates the matched
  detection's per-estimator distance to a **median per (instance, estimator)**;
  a GT never matched in any frame is MISSED.
- **Runner** (`benchmarking/g1_distance_benchmark_runner_node.py`) — iterates
  scenes, spawns robots + objects via `ros_gz_sim/create`, computes per-instance
  ground truth, scores. Single-robot uses the scalar-median path (regression
  guard); multi-robot uses `score_scene`.
- **Alignment** (`benchmarking/alignment.py`) — `MeasurementEvent` gained a
  per-detection `detections` table, merged across the camera/lidar/mask messages
  that share a frame's alignment key.
- **Collage** (`benchmarking/rendering.py`) — each box labelled with its instance
  + that estimator's value/true; a red `missed: N` note; single-robot output
  unchanged.

## 4. Scenario files (`config/`)

- **`benchmark_scenarios.yaml`** — **default** (no-arg run). The legacy 5×3 grid
  as 15 single-robot scenes → reproduces the old benchmark (regression guard).
- **`benchmark_scenarios_examples.yaml`** — small showcase (single, inter-robot
  occlusion, bed occluder, curtain occluder).
- **`benchmark_scenarios_full.yaml`** — **30 scenes, 6 groups × 5**, varied poses/yaw:
  - `single_*` — one robot, baseline
  - `multi_*` — 2–3 robots, all visible (laterally separated)
  - `interocc_*` — 2 robots on the same bearing → far one occluded/MISSED
  - `objclear_*` — object present but off to the side (robot clear)
  - `objocc_*` — object fully in front (robot occluded → typically skips)
  - `objpartial_*` — object partially occluding (robot ≥ half visible: low bed
    → upper body visible; laterally offset occluders clip part of the width)

### Schema

```yaml
defaults:
  robot_yaw_rad: 3.141592653589793     # G1 faces back toward the sensor
scenes:
  - id: <name>
    repeats_override: <int|omitted>    # optional per-scene repeat count
    robots:
      - { x: <fwd>, y: <lat>, yaw: <rad> }   # yaw optional (defaults above)
    objects:
      - { model: hospital_bed, x: <fwd>, y: <lat>, yaw: <rad> }  # yaw optional (0)
```

World-frame planar: `x` = forward, `y` = lateral (+left, REP-103), z pinned to
the floor. **Object `yaw` tunes occlusion footprint** — a broadside bed
(`yaw: 1.5708`) blocks far more than one pointing at the camera (`yaw: 0`); a
wide curtain (`yaw: 0`) is near-full, edge-on (`yaw: 1.5708`) blocks almost
nothing.

## 5. Occluder models (`sim/models/`)

Lifted from `sim/worlds/mock_hospital.sdf` box-composite furniture, origin at the
model base so spawning at `z=0` sits them on the floor:

- **`hospital_bed`** — `1.8 × 0.9 × 0.5` box; low → partial occluder (lower body).
- **`privacy_curtain`** — `0.06 × 1.7 × 1.6` panel; tall → near-full occluder.

## 6. How to run

Runs live in the worktree (see §7). Source chain: ROS + the main checkout's
install (clearpath sim packages) + the worktree install (its `ridgeback_autonomy`
wins the overlay).

```bash
cd /home/stefi/DyNAMO/DyNAMO-Ridgeback-redesign
source /opt/ros/jazzy/setup.bash
source /home/stefi/DyNAMO/DyNAMO-Ridgeback/install/setup.bash
source install/setup.bash
bash cleanup.sh

# Full set, mask estimators, silhouette gate (needs perception_venv / SlimSAM):
ros2 launch ridgeback_autonomy g1_distance_benchmark.launch.py \
  scenario:=$PWD/src/ridgeback_autonomy/config/benchmark_scenarios_full.yaml \
  estimators:=projective_ranging,euclidean_reconstruction,polar_profiling \
  mask_gate:=silhouette \
  repeats:=1
```

Launch args: `scenario` (empty = packaged grid), `repeats`, `estimators`,
`mask_gate` (`box`|`silhouette`), `depth_source`, `isolation_2d`/`isolation_3d`,
`overlay`. Output → `benchmark-results/<timestamp>/` (per-estimator CSVs,
`comparison_summary.csv` with `missed_instance_count`/`extra_detection_count`,
`images/` collages). The runner exits when done but the launch stays up — Ctrl-C,
then `bash cleanup.sh`.

## 7. Branch / worktree / merge

- **`benchmark-redesign`** — this redesign, developed in a **git worktree** at
  `/home/stefi/DyNAMO/DyNAMO-Ridgeback-redesign` (separate HEAD + build/install)
  so it doesn't fight the main checkout over the shared branch.
- **`g1-distance-benchmarks`** — the main checkout stays here; **still the old
  grid benchmark**, plus ongoing perception work.
- **Merge `ba22687`** — merged `g1-distance-benchmarks` *into* `benchmark-redesign`
  (config-driven overlay, depth-source robustness, aligned_depth, doc renames).
  Merging into redesign changes only redesign; the old-benchmark branch is
  untouched. Zero conflicts.
- **Worktree gotchas:** `perception_venv` is symlinked from the main checkout
  (the launch derives its path from the package root; without it the OWLv2
  detector can't load). Do **not** symlink the vendored clearpath src into the
  worktree — colcon then treats them as worktree packages and the build fails
  (consequence: 2 `test_launch_layout` tests are worktree-environmental — they
  pass in the main checkout). A live gz sim is system-wide — coordinate runs so
  worktree and main don't collide on gz.

## 8. Validation

- **Unit tests:** 186 pass across the affected suite (new `test_scenarios`,
  `test_association`, `test_scoring`; updated `test_benchmark_runner`; the merged
  perception tests). Only failure = the pre-existing `test_geometry` lidar case
  (696e101), which fails on both branches — not a redesign/merge regression.
- **E2E (sim):**
  - Single-robot scenes reproduce the baselines (mask ~0.06, lidar ~0.065).
  - `inter_robot_occlusion` → far robot **MISSED**, near robot scored.
  - Object occluder — **bed aside**: robot scored clean; **bed in front**: with
    lidar it grabs the bed (1.05 vs 3.25 true) while the mask path stays on the
    robot (3.19); with mask-only, a fully-occluded robot **skips** (honest).
  - Full 30-scene mask run: `projective_ranging`/`euclidean_reconstruction` MAE
    ~0.055, `polar_profiling` ~0.073 — consistent with the box-gate baselines.

  **Headline result the benchmark is built to surface:** an object occluder
  cleanly separates a fragile estimator (lidar locks onto the nearer surface —
  the bed) from a robust one (the mask isolates the robot's silhouette).

## 9. Known limitations

- **Software-GL headless rendering** produces only ~1–3 usable frames per trial;
  because a trial needs *every* selected estimator finite on one common frame,
  many multi-estimator / multi-robot / occluded trials **skip**. On a GPU (more
  frames) coverage is much higher. Not a code issue.
- **gz pose-query segfault** (~once per run) fails that one trial (`exit=-11`) —
  pre-existing, harmless.
- Partial-occlusion visible-fraction is approximate (depends on the robot's
  projected width + render noise); the `objpartial_*` set spans ~half-occluded to
  mostly-visible rather than exact fractions.

Nothing is pushed — all commits are local on `benchmark-redesign`.
