# Occlusion Handling

**Scope:** how the mask-based estimators behave when a foreground object partially occludes the robot
(the occluder is *closer* to the camera than the robot, but part of the robot is still visible), why
they currently fail, and the proposed recovery. Cross-cutting across the three paths
(`docs/localization/projective_ranging.md`, `docs/localization/euclidean_reconstruction.md`, `docs/localization/polar_profiling.md`) and their isolation
recipes (`docs/localization/foreground_isolation_2d.md`, `docs/localization/foreground_isolation_3d.md`).

**Status (2026-07-24):** the **problem** below is current behaviour. The **proposed solution** is a
planned, benchmark-gated change — *not yet implemented*. The benchmark scenes to validate it exist
(`config/benchmark_scenarios_full.yaml`: `objpartial_*`, `objocc_*`, `interocc_*`).

---

## 1. The problem

Under a front-occluder the masked region contains **three depth populations**: the occluder (nearest),
the visible robot parts (middle), and background seen past the robot (farthest).

Every path's foreground-isolation assumes **the target is the nearest coherent surface in the mask** —
which the occluder violates:

- **Projective ranging** — `nearest_mode_histogram` (`docs/localization/foreground_isolation_2d.md`) keeps the **nearest**
  significant depth mode → the occluder.
- **Euclidean reconstruction** — `NearestModeBand` (`docs/localization/foreground_isolation_3d.md`, the default since
  2026-08-28) anchors on the nearest significant range mode → the occluder; the robot, if more than
  the inlier window behind it, is dropped. The `RangeBand` chain it replaced anchors at the 25th
  percentile instead, which fails here **twice**: the occluder takes the anchor *and* pushes the
  robot's share of the point set below the quarter the percentile assumes, so heavy occlusion is
  exactly where the two recipes diverge most.
- **Polar profiling** — `merge_near_band` anchors on the **nearest run** → the occluder, *if* the
  occluder intersects the LiDAR scan plane. When the occluder is off the plane (e.g. a tall object with
  the plane passing under it to the robot's legs), polar can still recover the robot — the one path
  that is sometimes occlusion-robust.

**The damaging part:** this is not a clean miss. Each path returns a **confident wrong-near** value —
the occluder's distance reported as the robot's — which silently poisons the benchmark MAE rather than
dropping the trial. (See the parallax note in `docs/localization/object_localization_pipeline.md` §5 for the related
LiDAR case.)

**Front-end contributions.** Partial occlusion can also make the detector shrink the box to the visible
parts, **split** the robot into two boxes (→ `count != 1`, frame dropped, or a false second robot), or
drop below the detection threshold entirely. On the silhouette gate, a box-prompted SAM
(`docs/localization/segmentation_component.md`) may segment the salient occluder rather than the robot, and its
predicted-IoU can be high on that wrong mask, so the confidence floor does not catch it.

**The one configuration that already helps.** A *correct tight mask that excludes the occluder* fixes
the depth paths outright — if only robot pixels are selected, the occluder's depths never enter, so the
median/centroid land on the robot (range correct; the X,Y centre is biased toward the visible parts).
But the `box` gate cannot do this (the rectangle always drags in the occluder), and box-prompted
SlimSAM is not reliably occlusion-aware.

---

## 2. Why it is measurable now

The benchmark now spawns static occluders (a low `hospital_bed`, a tall thin `privacy_curtain`) in
front of the G1 with per-instance ground truth still equal to the robot:

- `objpartial_*` — robot stays ≥ 50 % visible (the tractable recovery case),
- `objocc_*` — object fully blocks the robot (the honest-miss case),
- `interocc_*` — a near robot occludes a far robot.

Per-estimator miss reasons and coverage are already reported (`run.json` observation
columns + the trial CSVs' `miss_reason`, the L3 status pipeline —
`common/miss_reason.py`), so both recovery (lower MAE, higher correct coverage) and honest misses (an
occlusion reason instead of a wrong number) are directly observable.

---

## 3. Proposed solution — depth-clustering recovery (planned)

Replace the "nearest = object" assumption, in the `rect` branch, with **cluster the masked depth and
pick the robot cluster by geometry, not by nearness.** New recipes are **opt-in** additions to the
pluggable registries (`ISOLATION_2D_RECIPES` / `ISOLATION_3D_RECIPES`); the current defaults are
untouched, so this is a benchmarkable A/B rather than a swap.

Clustering is a cheap **1-D range-gap split** (sort masked points by camera-frame range, break where
the gap exceeds a threshold — the same pattern as `polar_profiling.segment_range_profile`), not RANSAC
or full 3-D connected-components.

**The robot-plausibility selector (the core, shared by the variants below).** Per cluster, compute the
height-above-floor range (camera pose from TF, the `HeightCrop` formula), the **Z-extent** (extent
along the camera forward axis), and nearness. A cluster is robot-plausible iff it is tall enough
(`max height ≥ ~0.8 m` — rejects the hospital bed, top ~0.5 m) **and** deep enough
(`Z-extent ≥ ~0.08 m` — rejects the ~0.06 m-thin privacy curtain; the G1's own Z-extent is ~0.3 m).
Among plausible candidates, select the **nearest**, not the largest. Two deliberate choices, both
load-bearing:

- *Z-extent, not Euclidean range spread.* Range spread conflates lateral extent with thickness — a
  fronto-parallel curtain spanning ±0.85 m at 2.5 m shows ~0.14 m of range spread and would wrongly
  pass a "thickness" gate; its Z-extent is 0.06 m and correctly fails.
- *Nearest-plausible, not largest.* Background-wall pixels seen through the box occupy the box's image
  rows, so their cropped height range looks robot-like, and an angled wall passes Z-extent too; with a
  loose box the wall can hold the most points, so "largest" can confidently select the wall. With
  nearest-plausible the occluder fails the gates, the robot is the nearest passer, and the wall is
  never reached.

**3-D euclidean (primary — this is where recovery is reliable).** A floor-aware `depth_cluster_select`
recipe: floor removal, range-gap clustering, the selector above. Works even on the cheap `box` gate —
the clustering separates the occluder from the robot, so a tight silhouette mask is not required.

**2-D projective (same logic, cheap).** 2-D is not height-blind: per masked pixel the metric height
follows from the intrinsics — `height ≈ camera_height − (v − cy)/fy · Z` — so a 2-D variant runs the
*same* cluster + selector logic without building a point cloud. (A "largest depth mode" variant was
considered and rejected: the largest mode in a loose box at range is often the background wall — the
same failure the nearest-mode fallback fix removed.)

**Polar profiling (best-effort).** A **physical-width gate** on the runs: drop runs wider than a robot
(bearing width × range — walls, curtains and beds are wide; robot legs are ~0.1–0.2 m), then anchor
the existing near-band merge on the nearest *surviving* run. Polar has `(X, Z)` only (no height), so
this stays best-effort. (Selecting the run with the *most rays* was considered and rejected — robot
runs are the narrow ones; the widest runs are exactly the occluders and walls.)

**Tight-mask guard (silhouette gate).** The realistic silhouette failure is the segmenter cleanly
segmenting the **occluder** — a wrong-object tight mask with high predicted IoU, and the tight branch
runs no isolation at all. Guard: run the same plausibility gates on the tight branch's point set;
implausible → `OCCLUDED` miss instead of the occluder's distance.

**Honest miss when recovery is impossible.** When the cluster recipe finds clusters but none are
robot-plausible (full occlusion), the path returns a dedicated occlusion miss-reason
(`MissReason.OCCLUDED`) rather than the occluder's distance — a labeled miss the benchmark aggregates,
instead of a confident wrong number.

**Validation.** First a **baseline characterization** run of the current defaults on the new scenes
(per-scene MAE + the miss-reason histograms): some `objpartial_*` cases may partially survive today (a
box covering only the visible robot part can leave the occluder sliver below the nearest-mode
significance floor), and some failures may be detection-level rather than isolation-level — measure
before building. Then the A/B: on `objpartial_*` the cluster recipes should recover the robot distance
where the baseline reports the occluder; on `objocc_*` they should produce `OCCLUDED` misses instead of
wrong-near values; on `single_*` / `objclear_*` they must match the current recipes (no regression).

**Limits (known, accepted).**
- **`interocc_*` is not solved by construction** — the occluder *is* a robot and passes every
  plausibility gate; nearest-plausible then returns the near robot for the far robot's detection.
  Documented limit; a tie-break (cluster pixel support vs. the prompting box) is future work only if
  the measured rate warrants it.
- **Close-proximity merge** — an occluder within the cluster gap (~0.3 m) of the robot merges into one
  cluster and biases the distance near. Inherent to gap clustering.
- Thresholds assume a **standing** G1 and are tuned to the shipped occluders — a benchmark-validated
  heuristic, not a general occlusion solver. Reliable recovery is the **3-D** path's; the 2-D variant
  is close behind (height via intrinsics); polar is best-effort.

---

## 4. Alternatives considered

- **Class-aware segmentation (SAM 3).** Segment *robot-only* pixels so the occluder never enters any
  path — the most structural fix, and it would also settle the pending SAM 3 adoption (see
  `docs/localization/segmentation_component.md` §7). Deferred as the heaviest option (model wiring + ~3.4 GB VRAM, a
  real-hardware GPU-budget question).
- **Honest-miss only.** Detect the occlusion signature and return `None` + `OCCLUDED` without attempting
  recovery. Cheapest, but yields no distance under occlusion — subsumed by the clustering approach,
  which emits the same reason when it cannot recover.
- **Temporal tracking.** A sudden nearer-jump between frames is an occluder appearing; per-frame code
  cannot see it (no tracker exists — the hierarchy is deliberately per-frame,
  `docs/localization/mask_component.md` §6). A future tracking layer could both flag occlusion onset and carry the
  robot's last position through it; out of scope here.

---

## 5. Implementation plan

The design is §3; this section is the execution order. Phases land independently; the benchmark stays
green after each.

### Phase 0 — baseline characterization (do first, it is cheap)

Run the **current** defaults over the new scenario set and read per-scene MAE plus the per-estimator
miss-reason histograms (`run.json` → `reason_histogram`). This measures the failure
before building the fix: it splits
isolation-level failures (the §1 wrong-near lock) from detection-level ones (box shrink/split, the
single-detection gate), and identifies any `objpartial_*` cases that already survive (occluder sliver
under the nearest-mode significance floor). Output: a per-scene-group baseline table the Phase 3 A/B
is judged against — and possibly a re-scoping of the phases below.

### Phase 1 — core recipes + reason (pure code, pure tests)

- `MissReason.OCCLUDED` in `common/miss_reason.py` (a free code). Mapping, scoped to the new recipes
  and the tight guard: pre-isolation input healthy but no robot-plausible cluster → `OCCLUDED`
  (instead of the generic empty-isolation reason).
- `DepthClusterSelect` in `perception/target_localization/core/isolation_3d.py`: floor removal (the `HeightCrop` formula,
  TF-parameterized like the existing per-frame builder) → range-gap clustering → the §3 plausibility
  gates → nearest-plausible keep-mask. Params (`cluster_gap_m` ~0.30, `min_robot_height_m` ~0.8,
  `min_body_depth_m` ~0.08) as fields with module defaults.
- The 2-D twin in `perception/target_localization/core/isolation_2d.py` using per-pixel metric height (§3); the recipe
  needs intrinsics + camera height, so its signature is extended (existing recipes' signatures stay
  unchanged).
- Polar width-gate in `perception/target_localization/core/polar_profiling.py`, behind an opt-in parameter (default =
  current behaviour).
- Tight-branch plausibility guard in the euclidean (and projective) path.

Pure-test matrix (synthetic point/pixel sets, no ROS):

| Case | Expected |
|---|---|
| occluder near + robot behind | robot cluster selected (3-D and 2-D) |
| low bed-like cluster | rejected — height gate |
| thin curtain-like cluster **with wide lateral extent** | rejected — Z-extent gate (regression test for the range-spread trap) |
| tall wall-like cluster behind a plausible robot cluster | robot selected — nearest-plausible (regression test for the largest-cluster trap) |
| clusters present, none plausible, healthy input | empty keep-mask → path returns `OCCLUDED` |
| occluder within the cluster gap of the robot | merged cluster — documented limit |
| polar: wide curtain/wall run + two narrow leg runs | wide run dropped by the width gate; legs survive and merge |
| tight mask of an implausible object | guard returns `OCCLUDED`; a plausible set is unchanged |

### Phase 2 — wiring

Register the recipes in `ISOLATION_2D_RECIPES` / `ISOLATION_3D_RECIPES`; thread the polar parameter;
extend the launch arguments' documented choices. No message change (the status fields exist), so the
only build requirement is the usual node-script reinstall.

### Phase 3 — benchmark A/B

Current defaults vs the cluster recipes, `mask_gate:=box`, plus one silhouette run for the tight
guard. Judged against the Phase 0 baseline:

- `objpartial_*` — recovery: euclidean (and 2-D) error down, OK-coverage up, where the baseline
  reported the occluder.
- `objocc_*` — `OCCLUDED` misses in the summary's `reason_histogram` instead of confident
  wrong-near values.
- `single_*` / `objclear_*` — **no regression**: must match the current recipes.
- `interocc_*` — record the known-limit failure rate (§3 Limits) without attempting a fix.

### Phase 4 — docs

Rewrite §3 of this document to the as-built design with the A/B numbers, and strike the
planned-status note in the header. Add pointers from `docs/localization/foreground_isolation_2d.md` /
`docs/localization/foreground_isolation_3d.md` and the isolation sections of the path docs.

### Reuse (nothing re-invented)

- `camera_floor_geometry` + the `HeightCrop` height formula (`perception/target_localization/core/isolation_3d.py`); the
  per-frame TF-parameterized isolation builder already threaded through `target_mask_measurement_node`.
- `point_ranges`; the range-gap split pattern of `polar_profiling.segment_range_profile`.
- The miss-reason pipeline end to end: `common/miss_reason.py`, the paths' `(result, reason)` returns,
  the per-estimator status fields, and the benchmark's histogram/coverage aggregation — the benchmark
  side needs **no changes** for `OCCLUDED` to appear in the reports.
- The occlusion scenario set, occluder models, and per-instance ground truth
  (`config/benchmark_scenarios_full.yaml`, `sim/models/hospital_bed`, `sim/models/privacy_curtain`).
