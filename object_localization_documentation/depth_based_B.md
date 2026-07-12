# Path B — Deproject-Then-Aggregate (3D Point-Domain Route)

**Scope:** the point-domain localization path. It deprojects the aligned depth
frame into 3D points, selects the points under the mask, isolates the
foreground in the point domain, and reduces the surviving points to one
coordinate. This document describes Path B end to end. The mask contract is
`mask_component.md`; the aligned depth frame contract is `depth_based_path.md`;
the 3D foreground-isolation methods are catalogued in
`foreground_isolation_3d.md`; Path A, the cheaper image-domain sibling, is
`depth_based_A.md`.

---

## 1. Inputs and output

**Inputs**

- A **mask** from the mask interface: an `H×W` boolean array on the RGB color
  grid, plus its precision tag (`tight` | `rect`). See `mask_component.md`.
- An **aligned depth frame** (canonical input): 1:1 with the RGB pixels, from
  either depth source (`depth_based_path.md`). Path B builds its own point
  cloud from it — see the provenance note below.

**Output**

- One coordinate `(X, Y, Z)` in the **camera optical frame**, per mask (frame
  convention pinned in `Object_Localization_Pipeline.md` Section 7). Unlike
  Path A, Path B also has the full foreground point set available as a
  by-product (extent, orientation, footprint) if a later consumer wants it.

### Cloud provenance: deprojected only (decided)

Path B **deprojects the aligned depth frame itself** — that keeps the cloud on
the color grid (where the mask lives), works for both depth sources
(Depth-Anything publishes no cloud), and costs one vectorized inverse pinhole
per frame.

A **published** organized cloud (sim: gz `rgbd_camera` plugin; real: the
driver's pointcloud filter) also exists, but it is **not a Path B input** —
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
the `tight` / `rect` tag changes behavior — the same fork position as Path A.

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
`depth_based_A.md` §2.2, which apply verbatim) must be dropped either before
deprojection or carried as invalid points and dropped in step 3; they must
never reach the reduction.

### 2.2 Select

The mask indexes the organized cloud exactly as it indexes the depth frame in
Path A:

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

**`tight` branch.** The points are nearly all object; a statistical or radius
outlier removal pass (drop points far from their neighbors) is enough before
reducing.

**`rect` branch.** The box frustum contains the G1, the floor strip under it,
and whatever background falls inside the box. Selecting by mask only cuts the
frustum — everything inside it survives, so the foreground must be isolated in
the point domain. The methods — from the extrinsic height crop and the
range-band incumbent up to min-cut and learned segmentation — are catalogued
with pros, cons, and citations in `foreground_isolation_3d.md`. The contract
is the same shape as Path A's:

- **Input:** the masked point set (organized where possible, so pixel indices
  remain available).
- **Output:** the **foreground points** — the subset belonging to the object.

Chaining is the norm: a floor-remover (height crop, RANSAC plane removal, or
normal filter) followed by a background-separator (range band or Euclidean
clustering) makes a complete isolator.

**Placement alternative.** Isolation can instead run in 2D *before*
deprojection: apply a `foreground_isolation_2d.md` recipe to the depth frame +
mask, then deproject only the returned foreground pixels. That placement
shares one isolation implementation with Path A and deprojects fewer pixels;
the 3D placement exploits geometry the 2D methods cannot express (the floor is
a plane in 3D but a mode-less ramp in the depth histogram). The two placements
are a swap point of the benchmark matrix, and they compose — 2D coarse, 3D
fine. See the cross-route comparison in `foreground_isolation_3d.md`'s
evaluation protocol.

### 2.4 Reduce

Collapse the foreground point set to the outputs:

- **Coordinate:** the **centroid** of the foreground points — `(X, Y, Z)` in
  the camera optical frame. Robust variants (median per axis, or the centroid
  of the inlier core) resist residual contamination.
- **Distance (benchmark output):** the median range of the foreground points,
  or the nearest-inlier convention the current estimator uses — whichever the
  benchmark pins, applied identically across paths so the comparison stays
  algorithm-controlled.

Because the centroid and the range read the **same** foreground set, they
agree by construction — the same single-source rule as Path A §2.3.

---

## 3. Batch / per-mask granularity

Path B runs **per mask** over shared per-frame work, with the same 1:1:1
hierarchy as Path A (`mask_component.md` §6.1): one object → one post-NMS
detection → one mask → one coordinate, masks never compared or merged. The
cost split:

| Work | Frequency |
|------|-----------|
| decode + align the depth frame | once per frame |
| ray table | once per intrinsics change (effectively once) |
| masked deproject, isolate, reduce | once per mask |

With masked-only deprojection the per-mask cost scales with the box area, not
the frame area. Results are packed into the same parallel, index-aligned
arrays as Path A's (`mask_component.md` Section 6); results are never merged
across masks.

---

## 4. Where Path B beats Path A (and what it costs)

- **Single-pixel fragility gone.** Path A's answer rides on one representative
  pixel; Path B's centroid averages the whole foreground set, so one bad pixel
  cannot poison the coordinate.
- **Geometry-aware isolation.** The floor has no separable mode in the depth
  histogram (Path A's `rect` weakness) but is a literal plane in 3D — plane
  removal and clustering are tools only Path B can use.
- **Full-geometry by-products.** Extent, orientation, and footprint come free
  from the foreground set.
- **Cost:** deprojection of the masked pixels (sub-millisecond with the ray
  table) plus the isolation method's own runtime (see the speed column in
  `foreground_isolation_3d.md`). The heavier isolation methods, not the
  deprojection, dominate.

Path A stays the default cheap path; Path B is the escalation when Path A's
fragility or the `rect` contamination dominates the error budget.

---

## 5. Relationship to the current stack

The existing `pointcloud` estimator (`compute_pointcloud_measurement` in
`geometry.py`) is a proto-Path-B with three differences:

| Aspect | Current estimator | Path B target |
|--------|-------------------|---------------|
| Cloud provenance | subscribes the published cloud topic | deprojects the aligned depth (published cloud dropped — `pointcloud_provenance_test.md` §7) |
| Region | focus crop of the bbox | the actual mask, forked on tag |
| Isolation | range band (25th-percentile anchor, −0.10/+0.35 m window) | pluggable strategy (`foreground_isolation_3d.md`; range band = incumbent) |
| Output | nearest-inlier scalar range, vehicle frame, front offset applied | `(X, Y, Z)` centroid in the camera frame |

It is also stereo-only and sim-only in practice (no cloud is published on
hardware today; see `pointcloud_provenance_test.md` §1). Path B generalizes it
the same way Path A generalizes the depth estimator: mask instead of crop, tag
fork, pluggable isolation, camera-frame coordinate output.

---

## 6. Open items

- ~~**Reduction convention**~~ — resolved 2026-07-12: coordinate = centroid of
  the foreground points; distance = median camera-frame (Euclidean) range
  (`perception/core/path_b.py`). Applied identically across isolation recipes.
- **Isolation recipe** — choose and parameterize the `rect`-branch chain from
  `foreground_isolation_3d.md` (floor-remover + background-separator), then
  benchmark against the range-band incumbent.
- **Isolation placement** — 2D-before-deprojection vs. 3D-after: benchmark the
  swap point (`foreground_isolation_3d.md`, evaluation protocol step 4).
- ~~**Provenance decision**~~ — resolved 2026-07-11: deprojected only; the
  published cloud is demoted to RViz/debug (`pointcloud_provenance_test.md` §7).
- **Sparse-mask fallback** — behavior when too few valid points survive
  isolation (skip, or defer to Path A / Path C).
- **Coordinate frame** — same confirmation as Path A
  (`Object_Localization_Pipeline.md` Section 7).
