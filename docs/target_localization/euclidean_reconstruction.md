# Euclidean reconstruction — Deproject-Then-Aggregate (3D Point-Domain Route)

**Scope:** the point-domain localization path. It deprojects the aligned depth
frame into 3D points, selects the points under the mask, isolates the
foreground in the point domain, and reduces the surviving points to one
coordinate. This document describes euclidean reconstruction end to end. The mask contract is
`docs/target_localization/mask_representation.md`; the aligned depth frame contract is `docs/target_localization/aligned_depth.md`;
its implemented 3D foreground-isolation methods are documented below;
projective ranging, the cheaper image-domain sibling, is
`docs/target_localization/projective_ranging.md`.

---

## 1. Inputs and output

**Inputs**

- A **mask** from the mask interface: a boolean selector on the RGB color
  grid, plus its precision tag (`tight` | `rect`). See `docs/target_localization/mask_representation.md`.
- An **aligned depth frame** (canonical input): 1:1 with the RGB pixels, from
  either depth source (`docs/target_localization/aligned_depth.md`). Euclidean reconstruction builds its own point
  cloud from it — see the provenance note below.

**Output**

- One coordinate `(X, Y, Z)` in the **camera optical frame**, per mask (the
  downstream `base_link` planar conversion is fixed —
  `docs/target_localization/target_localization_pipeline.md` Section 7). Unlike
  projective ranging, euclidean reconstruction also has the full foreground point set available as a
  by-product (extent, orientation, footprint) if a later consumer wants it.

### Cloud provenance: deprojected aligned depth

The path deprojects selected aligned-depth samples on the color grid. This works
with both depth sources and does not require a published pointcloud. The
separate `pointcloud` estimator still consumes a published organized cloud;
its physical-camera eligibility must be validated, not inferred from driver
defaults. The original source comparison and architectural decision are in
[provenance history](../history/pointcloud_provenance_evaluation.md).

## 2. The four steps

```
1. DEPROJECT  aligned depth -> organized (H, W, 3) points   # inverse pinhole
2. SELECT     points_masked = points[mask]                  # mask as index
3. ISOLATE    foreground points only                        # forks on mask tag
4. REDUCE     foreground point set -> (X, Y, Z)             # centroid only
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

Conceptually, the result retains the depth frame's `(H, W)` organization, so
pixel indices still match the color grid. Production does not materialize that
full cloud: it uses the grid relationship to deproject selected samples only.

Two implementation notes:

- **Ray table.** `(u - cx)/fx` and `(v - cy)/fy` depend only on the intrinsics,
  so they are precomputed once as an `(H, W, 2)` ray table; per frame the
  deprojection is one multiply by `Z`.
- **Masked-only deprojection.** Steps 1 and 2 commute: because the mask lives
  on the same grid, the multiply can be restricted to the masked pixels
  (`~5–20 k` instead of `H×W`), which is the form production uses.
  The full-frame form exists for debugging and RViz export.
- **Where the ROI stops.** The mask *selection* is read off the mask's own
  storage window (`docs/target_localization/mask_representation.md` Section 7), but the deprojection is not:
  the surviving indices are lifted to full-grid coordinates and gathered from
  the original depth frame against the original color intrinsics. There are no
  ROI-adjusted intrinsics. `deproject_masked` gathers the selected depths
  *before* widening them to float64 precisely so it can be handed the whole
  frame this way without casting it.

Invalid depth pixels (0 / NaN / inf — see the cleaning rules in
`docs/target_localization/projective_ranging.md` §2.2, which apply verbatim) must be dropped either before
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
(published-cloud grid compatibility is not assumed; see the hardware plan).

### 2.3 Isolate — the mask-tag fork

What the selection returns depends on the mask precision:

| Tag    | Selected set                       | Recovery |
|--------|------------------------------------|----------|
| `tight`| object points (± edge bleed)       | statistical outlier removal, then reduce |
| `rect` | object + floor strip + background  | **3D foreground isolation**, then reduce |

**`tight` branch.** The points are nearly all object; a statistical outlier
removal pass is enough before reducing. The shipped rule
(`mad_outlier_removal` in `perception/target_localization/core/isolation_3d.py`) keeps the points
whose camera-frame range is within median ± `k`·σ of the range distribution —
the stragglers being edge bleed onto the background.

σ is the MAD rescaled by the consistency constant `1.4826`, so `k = 3` is
Miller (1991)'s "very conservative" threshold as reported by Leys et al.
(2013), rather than three raw median deviations (which is the same cut at
`k ≈ 2.02`). The rescaling is derived under normality and body-surface ranges
are not normal, so it does not make the threshold correct for this data — it
makes `k` mean what the published number means. Neither `k` nor the branch has
been measured on this path: no benchmark configuration has yet produced a
scored `tight` euclidean run.

**`rect` branch.** The box frustum contains the G1, the floor strip under it,
and whatever background falls inside the box. Selecting by mask only cuts the
frustum — everything inside it survives, so the foreground must be isolated in
the point domain.

#### Isolation contract

An isolator accepts camera-optical XYZ points as an `(N, 3)` array in metres
(X right, Y down, Z forward) and returns an `(N,)` boolean keep-selector.
`Chain` runs each step on only the previous step's survivors while preserving
original point ordering. Empty selections are valid; the estimator's
minimum-count guard decides whether a measurement can be emitted.

#### Implemented 3D recipes

| Launch token (`isolation_3d`) | Steps |
|---|---|
| `height_crop` | `HeightCrop` |
| `range_band` | `RangeBand` |
| `nearest_mode_band` | `NearestModeBand` |
| `height_crop_range_band` | `HeightCrop` then `RangeBand` |
| `height_crop_nearest_mode_band` | `HeightCrop` then `NearestModeBand` — current default |

**Height crop.** `HeightCrop` retains points whose estimated height above the
floor exceeds `floor_margin_m` (`0.05 m` by default). Height above the floor is
the camera's height minus the point's projection onto gravity-down, so the crop
is a half-space test that is exact for any mount orientation, roll included.
Both the height and the gravity-down direction come from live TF (plus the
chassis base-above-floor offset) via `camera_floor_geometry`; there is no static
mount and no default pose, so a recipe containing a crop can only be built
through `build_isolation_3d`, and only from a pose the caller actually has.
This removes floor, not walls. Calibration error, ramps, and uneven ground can
remove target points or retain floor points; this is not plane fitting.

**Percentile range band.** `RangeBand` anchors at the 25th percentile of
Euclidean camera-frame range and retains the interval from `0.10 m` ahead to
`0.35 m` behind. Those defaults live in `core/ranging_defaults.py`. The values
come from the original G1 tuning and are not automatically target-generic. If
background dominates the box, the percentile can anchor on it. This recipe
remains selectable but is no longer the default.

**Nearest-mode band.** `NearestModeBand` replaces the percentile anchor with
the shared `nearest_significant_mode`, then applies the same asymmetric window.
It reduces dependence on background proportions but still assumes that the
nearest significant surface is the target. A nearer occluder can therefore win.
The default chain removes floor before applying this step.

Recipe names are launch-selectable; `build_isolation_3d` supplies live
floor-geometry parameters where required. Tests in
[`test_isolation_3d.py`](../../src/ridgeback_autonomy/test/test_isolation_3d.py)
cover registered chains, pose-dependent crops, empty selections, and selection
behavior. Comparative validation is tracked in
[the backlog](../BACKLOG.md#isolation-validation).

RANSAC, normal filters, clustering, min-cut, and learned methods remain
[foreground-isolation candidates](../do_not_try_again/foreground_isolation.md), not
selectable implementations. The
[candidate evaluation protocol](../do_not_try_again/foreground_isolation.md#evaluation-protocol)
describes a possible 2D-before-deprojection comparison. It is not a shipped
placement switch or an automatic fallback to projective ranging.

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
hierarchy as projective ranging (`docs/target_localization/mask_representation.md` §6.1): one object → one post-NMS
detection → one mask → one coordinate, masks never compared or merged. The
cost split:

| Work | Frequency |
|------|-----------|
| decode + align the depth frame | once per frame |
| ray table | once per intrinsics change (effectively once) |
| masked deproject, isolate, reduce | once per mask |

With masked-only deprojection the per-mask cost scales with the box area, not
the frame area. Results are packed into the same parallel, index-aligned
arrays as projective ranging's (`docs/target_localization/mask_representation.md` Section 6); results are never merged
across masks.

---

## 4. Geometric differences from projective ranging

- **Distributed coordinate reduction.** Projective ranging gets bearing from
  one representative UV; euclidean reconstruction averages every surviving XYZ
  point. An individual error influences rather than solely determines the
  coordinate, while systematic contamination can still bias the centroid.
- **Geometry-aware isolation.** The floor has no separable mode in the depth
  histogram (projective ranging's `rect` weakness) but is a literal plane in 3D — plane
  removal and clustering are tools only euclidean reconstruction can use.
- **Full-geometry by-products.** The foreground set is available for later extent, orientation, or footprint
  calculations; those calculations are not emitted by this estimator.
- **Cost:** selected-point deprojection plus the chosen isolation and reduction.
  See [migration measurements](../history/roi_mask_migration.md) for scoped CPU
  evidence; there is no universal sub-millisecond or end-to-end guarantee.

The paths are independent selected estimators. Geometry can motivate comparing
them; no automatic escalation policy is implemented.

---

## 5. Integration and checks

Production uses `localize_prepared_euclidean_reconstruction` with the same
prepared region as projective ranging. The standalone full-grid entry point
remains available. Too few valid points or isolation survivors yields no result.
The node transforms the centroid into the shared base-planar convention before
publishing; no independent median-distance statistic is emitted.

The retained `pointcloud` row differs in source (published cloud), region
(focus crop), and reduction (forward-distance percentile/inlier policy). Both
rows publish a planar position, not merely a scalar. Hardware cloud layout is
unverified, not known absent.

[Algorithm tests](../../src/ridgeback_autonomy/test/test_euclidean_reconstruction.py)
and [ROI history](../history/roi_mask_migration.md) document correctness checks.
[The backlog](../BACKLOG.md) tracks comparative isolation and hardware validation.
