# Euclidean reconstruction — Deproject-Then-Aggregate (3D Point-Domain Route)

**Scope:** the point-domain localization path. It deprojects the aligned depth
frame into 3D points, selects the points under the mask, isolates the
foreground in the point domain, and reduces the surviving points to one
coordinate. This document describes euclidean reconstruction end to end. The mask contract is
`mask_component.md`; the aligned depth frame contract is `aligned_depth.md`;
the 3D foreground-isolation methods are catalogued in
`foreground_isolation_3d.md`; projective ranging, the cheaper image-domain sibling, is
`projective_ranging.md`.

---

## 1. Inputs and output

**Inputs**

- A **mask** from the mask interface: an `H×W` boolean array on the RGB color
  grid, plus its precision tag (`tight` | `rect`). See `mask_component.md`.
- An **aligned depth frame** (canonical input): 1:1 with the RGB pixels, from
  either depth source (`aligned_depth.md`). Euclidean reconstruction builds its own point
  cloud from it — see the provenance note below.

**Output**

- One coordinate `(X, Y, Z)` in the **camera optical frame**, per mask (the
  downstream `base_link` planar conversion is fixed —
  `object_localization_pipeline.md` Section 7). Unlike
  projective ranging, euclidean reconstruction also has the full foreground point set available as a
  by-product (extent, orientation, footprint) if a later consumer wants it.

### Cloud provenance: deprojected only (decided)

Euclidean reconstruction **deprojects the aligned depth frame itself** — that keeps the cloud on
the color grid (where the mask lives), works for both depth sources
(Depth-Anything publishes no cloud), and costs one vectorized inverse pinhole
per frame.

A **published** organized cloud (sim: gz `rgbd_camera` plugin; real: the
driver's pointcloud filter) also exists, but it is **not a euclidean reconstruction input** —
this was tested and decided 2026-07-11 (`pointcloud_provenance_test.md` §6–7):
accuracy bit-identical, masked deprojection ~10× cheaper than parsing the
cloud, wire cost 6× against the published cloud, and on real hardware the
published cloud lives on the depth grid where the mask cannot index it. The
published topic's only remaining role is optional RViz/debug visualization.

---

## 2. The four steps

```
1. DEPROJECT  aligned depth -> organized (H, W, 3) points   # inverse pinhole
2. SELECT     points_masked = points[mask]                  # mask as index
3. ISOLATE    foreground points only                        # forks on mask tag
4. REDUCE     foreground point set -> (X, Y, Z)             # centroid (+ range)
```

Steps 1, 2 and 4 are identical for both mask types. Step 3 is the only place
the `tight` / `rect` tag changes behavior — the same fork position as projective ranging.

### 2.1 Deproject

Run the inverse pinhole projection over the depth frame (intrinsics
`fx, fy, cx, cy`):

```
X(u, v) = (u - cx) / fx * Z(u, v)
Y(u, v) = (v - cy) / fy * Z(u, v)
Z(u, v) = Z(u, v)
```

The result is an **organized** cloud: an `(H, W, 3)` array whose pixel indices
still match the color grid. Organization is what lets the mask index it in
step 2, and it is free — the depth frame is already an `H×W` grid.

Two implementation notes:

- **Ray table.** `(u - cx)/fx` and `(v - cy)/fy` depend only on the intrinsics,
  so they are precomputed once as an `(H, W, 2)` ray table; per frame the
  deprojection is one multiply by `Z`.
- **Masked-only deprojection.** Steps 1 and 2 commute: because the mask lives
  on the same grid, the multiply can be restricted to the masked pixels
  (`~5–20 k` instead of `H×W`), which is the form production code should use.
  The full-frame form exists for debugging and RViz export.

Invalid depth pixels (0 / NaN / inf — see the cleaning rules in
`projective_ranging.md` §2.2, which apply verbatim) must be dropped either before
deprojection or carried as invalid points and dropped in step 3; they must
never reach the reduction.

### 2.2 Select

The mask indexes the organized cloud exactly as it indexes the depth frame in
projective ranging:

```python
points_masked = points[mask]   # (N, 3) — the box frustum's points
```

The same grid precondition applies: mask and cloud must share resolution,
intrinsics, and alignment. With deprojection this holds by construction —
one of the reasons the published cloud was rejected as an input
(`pointcloud_provenance_test.md` §7: on real hardware it lives on the depth
grid, not the color grid).

### 2.3 Isolate — the mask-tag fork

What the selection returns depends on the mask precision:

| Tag    | Selected set                       | Recovery |
|--------|------------------------------------|----------|
| `tight`| object points (± edge bleed)       | statistical outlier removal, then reduce |
| `rect` | object + floor strip + background  | **3D foreground isolation**, then reduce |

**`tight` branch.** The points are nearly all object; a statistical outlier
removal pass is enough before reducing. The shipped rule
(`mad_outlier_removal` in `perception/core/isolation_3d.py`) keeps the points
whose camera-frame range is within median ± 3·MAD of the range distribution —
the stragglers being edge bleed onto the background.

**`rect` branch.** The box frustum contains the G1, the floor strip under it,
and whatever background falls inside the box. Selecting by mask only cuts the
frustum — everything inside it survives, so the foreground must be isolated in
the point domain. The methods — from the extrinsic height crop and the
range-band incumbent up to min-cut and learned segmentation — are catalogued
with pros, cons, and citations in `foreground_isolation_3d.md`. The contract
is the same shape as projective ranging's:

- **Input:** the masked point set (organized where possible, so pixel indices
  remain available).
- **Output:** the **foreground points** — the subset belonging to the object.

Chaining is the norm: a floor-remover (height crop, RANSAC plane removal, or
normal filter) followed by a background-separator (range band or Euclidean
clustering) makes a complete isolator.

**Placement alternative.** Isolation can instead run in 2D *before*
deprojection: apply a `foreground_isolation_2d.md` recipe to the depth frame +
mask, then deproject only the returned foreground pixels. That placement
shares one isolation implementation with projective ranging and deprojects fewer pixels;
the 3D placement exploits geometry the 2D methods cannot express (the floor is
a plane in 3D but a mode-less ramp in the depth histogram). The two placements
are a swap point of the benchmark matrix, and they compose — 2D coarse, 3D
fine. See the cross-route comparison in `foreground_isolation_3d.md`'s
evaluation protocol.

### 2.4 Reduce

Collapse the foreground point set to the single output:

- **Coordinate:** the **centroid** of the foreground points — `(X, Y, Z)` in
  the camera optical frame. This is the only statistic the reduce step emits;
  `EuclideanReconstructionResult` carries the coordinate and the foreground
  point set, and no separate `distance_m` field.
- **Distance (benchmark output):** derived **downstream** from that same
  centroid — its planar projection into the base frame — rather than as an
  independent range statistic over the points. Because the distance rests on
  the coordinate, the two stay self-consistent by construction (the foreground
  set is still available if a consumer ever wants a range directly from it).

---

## 3. Batch / per-mask granularity

Euclidean reconstruction runs **per mask** over shared per-frame work, with the same 1:1:1
hierarchy as projective ranging (`mask_component.md` §6.1): one object → one post-NMS
detection → one mask → one coordinate, masks never compared or merged. The
cost split:

| Work | Frequency |
|------|-----------|
| decode + align the depth frame | once per frame |
| ray table | once per intrinsics change (effectively once) |
| masked deproject, isolate, reduce | once per mask |

With masked-only deprojection the per-mask cost scales with the box area, not
the frame area. Results are packed into the same parallel, index-aligned
arrays as projective ranging's (`mask_component.md` Section 6); results are never merged
across masks.

---

## 4. Where euclidean reconstruction beats projective ranging (and what it costs)

- **Single-pixel fragility gone.** projective ranging's answer rides on one representative
  pixel; euclidean reconstruction's centroid averages the whole foreground set, so one bad pixel
  cannot poison the coordinate.
- **Geometry-aware isolation.** The floor has no separable mode in the depth
  histogram (projective ranging's `rect` weakness) but is a literal plane in 3D — plane
  removal and clustering are tools only euclidean reconstruction can use.
- **Full-geometry by-products.** Extent, orientation, and footprint come free
  from the foreground set.
- **Cost:** deprojection of the masked pixels (sub-millisecond with the ray
  table) plus the isolation method's own runtime (see the speed column in
  `foreground_isolation_3d.md`). The heavier isolation methods, not the
  deprojection, dominate.

Projective ranging stays the default cheap path; euclidean reconstruction is the escalation when projective ranging's
fragility or the `rect` contamination dominates the error budget.

---

## 5. Relationship to the current stack

Euclidean reconstruction is shipped as
`perception/core/euclidean_reconstruction.py`
(`localize_euclidean_reconstruction`). The **legacy** `pointcloud` estimator
(`compute_pointcloud_measurement` in `geometry.py`) is a proto-Path-B with
three differences the shipped path resolves:

| Aspect | Legacy estimator | euclidean reconstruction (shipped) |
|--------|-------------------|---------------|
| Cloud provenance | subscribes the published cloud topic | deprojects the aligned depth (published cloud dropped — `pointcloud_provenance_test.md` §7) |
| Region | focus crop of the bbox | the actual mask, forked on tag |
| Isolation | range band (25th-percentile anchor, −0.10/+0.35 m window) | pluggable strategy (`foreground_isolation_3d.md`; range band = incumbent) |
| Output | nearest-inlier scalar range, vehicle frame, front offset applied | `(X, Y, Z)` centroid in the camera frame |

The legacy estimator is also stereo-only and sim-only in practice (no cloud is
published on hardware today; see `pointcloud_provenance_test.md` §1). Euclidean
reconstruction generalizes it the same way projective ranging generalizes the
depth estimator: mask instead of crop, tag fork, pluggable isolation,
camera-frame coordinate output.

---

## 6. Open items

- ~~**Reduction convention**~~ — resolved 2026-07-23: the reduce step emits
  only the centroid of the foreground points
  (`perception/core/euclidean_reconstruction.py`); there is no separate
  distance statistic, and the published distance is derived downstream from that
  centroid (base-planar projection). Applied identically across isolation
  recipes.
- **Isolation recipe** — the `rect`-branch chain is a pluggable strategy.
  ~~Choose and parameterize it~~ — resolved 2026-07-24: the default is
  `Chain(HeightCrop, RangeBand)` (`ISOLATION_3D_DEFAULT` = `height_crop_range_band`
  in `perception/core/isolation_3d.py`) — a floor-remover then a
  background-separator, wired and running. Benchmarking the recipes against the
  range-band incumbent stays open.
- **Isolation placement** — 2D-before-deprojection vs. 3D-after: benchmark the
  swap point (`foreground_isolation_3d.md`, evaluation protocol step 4).
- ~~**Provenance decision**~~ — resolved 2026-07-11: deprojected only; the
  published cloud is demoted to RViz/debug (`pointcloud_provenance_test.md` §7).
- ~~**Sparse-mask fallback**~~ — resolved 2026-07-21: skip —
  `localize_euclidean_reconstruction` returns `None` when too few valid points
  survive isolation, so the benchmark drops the row (never substituted). Any
  defer-to-projective-ranging / polar-profiling routing is a production-pipeline
  consumer concern, not the estimator's.
- ~~**Coordinate frame**~~ — resolved 2026-07-24, same basis as projective ranging
  (`projective_ranging.md` §7, deferring to `object_localization_pipeline.md` Section 7):
  euclidean reconstruction and projective ranging share `deproject_*` +
  `optical_to_base_planar`, so the frame confirmation is identical for both. Two
  parts: (1) the downstream camera-optical → `base_link` conversion is fixed (full
  live-TF extrinsic, lateral base +Y left-positive REP-103); (2) the raw optical
  axis/handedness (X right, Y down, Z forward) is empirically confirmed in sim —
  mask-path MAE ~0.057 m with correct left-positive signs, so a flip is ruled out —
  with only the real-hardware RealSense-SDK/TF axis cross-check outstanding
  (belt-and-suspenders, not a suspected bug). This matters most for euclidean
  because its `HeightCrop` floor plane treats optical Y as gravity-down, so a wrong
  axis would corrupt the isolation, not just the coordinate.
