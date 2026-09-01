# Projective ranging — Aggregate-Then-Deproject (2D Depth-Image Route)

**Scope:** the aggregate-then-deproject localization path. It takes a mask plus an
aligned depth frame, reads the depth at the masked pixels, reduces them to a
single distance, and deprojects one representative pixel into a 3D point. This
document describes projective ranging end to end. The mask object it consumes is defined in
`docs/target_localization/mask_representation.md`; how the aligned depth frame is produced (stereo alignment
or monocular estimation) is defined in `docs/target_localization/aligned_depth.md`. This doc assumes
both contracts and does not re-describe them. Euclidean reconstruction, the point-domain sibling,
is `docs/target_localization/euclidean_reconstruction.md`.

---

## 1. Inputs and output

**Inputs**

- A **mask** from the mask interface: a boolean selector on the RGB color
  grid, plus its precision tag (`tight` | `rect`). See `docs/target_localization/mask_representation.md`.
- An **aligned depth frame**: a depth image that is 1:1 with the RGB pixels, so
  that depth pixel `(u, v)` is the same ray as color pixel `(u, v)`. Producing
  this is a separate component (`docs/target_localization/aligned_depth.md`); projective ranging assumes it is
  already aligned.

**Output**

- One coordinate `(X, Y, Z)` in the **camera optical frame**, per mask. (The
  downstream `base_link` planar conversion is fixed —
  `docs/target_localization/target_localization_pipeline.md` Section 7.)

Projective ranging does not build a 3D point set: it collapses masked depths to
one number and deprojects exactly one representative pixel. End-to-end cost
also includes the selected mask and depth producers.

---

## 2. The four steps

```
1. SELECT     depths = aligned_depth[mask]        # mask used as an index
2. CLEAN      drop 0 / NaN / inf / out-of-range
3. AGGREGATE  reduce the depths to one distance    # forks on the mask tag
4. DEPROJECT  representative pixel + Z -> (X, Y, Z) # inverse pinhole
```

Steps 1, 2 and 4 are identical for both mask types. Step 3 is the only place the
`tight` / `rect` tag changes behavior.

### 2.1 Select

The mask is used directly as a boolean index into the aligned depth frame:

```python
depths = aligned_depth[mask]   # 1-D array of Z values inside the object region
```

This is the whole reason the front-end converts a box into a mask: `tight` and
`rect` masks index depth with the *same* line. The select step requires the
precondition that the mask and the depth frame share the grid (same resolution,
same intrinsics, aligned) — if that does not hold, the indices do not refer to
the same rays and the result is meaningless.

### 2.2 Clean

Before aggregation, `valid_depth` rejects non-finite and non-positive samples.
An effective finite ceiling also rejects samples above it. The node combines
the optional `mask_depth_max_meters` gate (0 disables it) with the source's
`usable_max_m`. Stereo has no source ceiling; the metric monocular checkpoint
declares one. A finite positive stereo value is not necessarily accurate.
See [aligned depth](aligned_depth.md) for that contract.

### 2.3 Aggregate — the mask-tag fork

The valid masked depths are a *distribution*, and how it is collapsed depends on
the precision tag, because a `rect` mask carries background contamination that a
`tight` mask does not.

| Tag    | Distribution shape          | Recovery |
|--------|-----------------------------|----------|
| `tight`| one cluster (object only)   | **robust median** directly |
| `rect` | bimodal (object + background) | **isolate foreground, then median** |

**`tight` branch.** The masked pixels are nearly all object, so a robust
statistic is enough. Use the **median**, not the mean: the mean is dragged by the
few edge pixels that bleed onto background, while the median ignores them.

**`rect` branch.** The rectangle also covers floor, wall, and neighbours, so the
depths form (at least) two peaks — a near peak (the object) and a far peak (the
background behind it). A plain median can land between the peaks, or on the
background if it dominates the box. So the foreground must be isolated first.

**The `rect` foreground isolation is a pluggable strategy.** Rather than hard-code
one recipe, the `rect` branch is defined as a single component with a fixed
contract, so multiple recipes can be implemented and benchmarked against each
other on identical input:

- **Input:** the aligned depth frame and the mask. Production runs this on the
  mask's own **storage window** — a depth view, the mask payload and the sliced
  validity image, all the same ROI shape — and lifts the surviving pixels back
  to full-grid coordinates before deprojection (`docs/target_localization/mask_representation.md` Section 7).
  The recipes are shape-agnostic, and a window's valid pixels are the frame's
  valid pixels in the same row-major order, so the histogram sees the identical
  value sequence either way.
- **Output:** the **foreground pixels** — the subset of the masked pixels that
  belong to the object (the G1), with the background pixels dropped. Concretely,
  the pixel coordinates inside the frame that the strategy classifies as object;
  the depth values and the centroid both follow from those coordinates.

The isolation callable returns a boolean selector; the estimator obtains its
foreground coordinates. An empty selection is valid at the recipe boundary;
the estimator applies its minimum-count guard and reports `ISOLATION_EMPTY` when
too few pixels survive. An optional precomputed `valid_masked` selector avoids
repeating the validity pass.

#### Implemented 2D recipes

| Launch token (`isolation_2d`) | Algorithm | Default |
|---|---|---|
| `nearest_mode_histogram` | Nearest significant depth bin followed by a symmetric inlier band | Yes |
| `otsu` | Histogram threshold maximizing between-class variance; retain the near side | No |

**Nearest-mode histogram.** `nearest_significant_mode` chooses the nearest bin
meeting the 5% sample-significance floor. If no bin meets that floor, it chooses
the nearest non-empty bin rather than the global mode. The recipe retains depths
within `band_m` of the anchor. The mode helper lives in `core/depth_common.py`;
shared numeric defaults live in `core/ranging_defaults.py`.

This assumes the target is the nearest coherent surface. A closer occluder can
win, spatial connectivity is not enforced, and bin width, significance, and
band width affect the result.

**Otsu.** `otsu_foreground` selects the threshold with maximum between-class
variance and retains depths on its near side. It still has a bin-width setting
(`0.05 m` by default). A single-bin distribution is retained whole and an input
with no valid depths yields an empty selector. Multiple background layers or a
changing foreground/background ratio can move the threshold away from the
target. The method follows N. Otsu, *A Threshold Selection Method from Gray-Level
Histograms*, IEEE TSMC 9(1), 1979, DOI `10.1109/TSMC.1979.4310076`.

Recipe names are launch-selectable, but not every function keyword is exposed
as a launch parameter. These are deterministic NumPy baselines, not guarantees
of foreground identity or measured zero-cost operations. Tests in
[`test_isolation_2d.py`](../../src/ridgeback_autonomy/test/test_isolation_2d.py)
cover subset, invalid/unimodal, and dispersed-histogram behavior. Comparative
validation is tracked in [the backlog](../BACKLOG.md#isolation-validation), and
unimplemented methods live in
[foreground-isolation candidates](../do_not_try_again/foreground_isolation.md).
SAM is a mask producer, not a depth-isolation recipe.

This foreground set is the single source for both remaining steps: the
**aggregate** takes the median of the foreground pixels' depths, and the
**deproject** (§2.4) takes the centroid of the foreground pixels' coordinates as
the representative pixel. Because both read the *same* set, they agree by
construction.

The `rect` branch adds a histogram-based isolation pass over the selected ROI;
the `tight` branch instead pays for segmentation before this estimator. Their
end-to-end costs depend on ROI, frame, producer, and hardware. Scoped CPU
measurements are recorded in the ROI migration history; this contract does not
claim either complete configuration is always cheaper.

### 2.4 Deproject

Take a **representative pixel** `(u, v)` and the aggregated depth `Z`, and run the
inverse pinhole projection (intrinsics `fx, fy, cx, cy`):

```
X = (u - cx) / fx * Z
Y = (v - cy) / fy * Z
Z = Z
```

The result is `(X, Y, Z)` in the camera optical frame.

**The representative pixel must be the centroid of the *foreground* pixels, not
the raw geometric box center.** On the `rect` branch those are the foreground
pixels the isolation strategy returned (§2.3); on the `tight` branch they are all
valid masked pixels. Projective ranging's entire answer rides on this one pixel. If it lands
on a depth discontinuity (object edge against far background) its `Z` is wrong
even when the aggregated range was correct. Taking the centroid of the foreground
pixels keeps the pixel on the object.

For a **non-convex silhouette** (the `tight` branch on a legged robot) the
centroid can itself fall in a concavity — the gap between the legs — landing on
background rather than the object. That is safe here regardless: the deproject
takes `Z` from the **aggregated (median) foreground depth**, never from the
centroid pixel's own depth, so the range is decoupled from wherever the centroid
lands. `(u, v)` only fixes the `(X, Y)` bearing, which is a body-center estimate;
a centroid sitting in the leg gap still points at the body center. The mitigation
is intrinsic to aggregating depth separately from choosing the representative
pixel, so it needs no extra guard.

---

## 3. Depth-source agnostic

Projective ranging does not care *how* the aligned depth frame was produced. The same four
steps run unchanged on RealSense stereo depth (after alignment) and on
Depth-Anything monocular depth (after metric scaling). Both producers and the
contract they converge to are specified in `docs/target_localization/aligned_depth.md`; the depth
source is a swappable input and both are benchmarked through the identical
path.

---

## 4. Batch / per-mask granularity

Projective ranging runs **per mask**, looped within a frame, over a shared depth frame. The
hierarchy is strictly 1:1:1 (`docs/target_localization/mask_representation.md` §6.1): one visible object →
one post-NMS detection → one mask → one `(X, Y, Z)`. Two G1s in view means two
masks and two independent projective ranging runs — masks are never compared, merged, or
ranked against each other; object *i*'s coordinate is computed as if the other
masks did not exist. Crucially, the heavy per-frame work is done once and the
per-mask work is cheap:

| Work | Frequency |
|------|-----------|
| decode + align the depth frame | once per frame (shared by all masks) |
| `depth[mask_i]` select, clean, aggregate, deproject | once per mask |

```
for i in range(count):
    depths_i   = aligned_depth[mask_i]      # this object's pixels
    (X,Y,Z)_i  = aggregate(depths_i) -> deproject(centroid_i)
    results[i] = (X, Y, Z)
```

Results are never merged across masks — each object gets its own coordinate — and
they are packed into the parallel, index-aligned arrays the pipeline messages
already define.

---

## 5. Two aggregation philosophies (where projective ranging sits)

It is worth being explicit about *order of operations*, because it separates
projective ranging from euclidean reconstruction:

- **Aggregate-then-deproject (projective ranging):** collapse the masked depths to one
  number, then deproject one representative pixel. Cheap, yields a single point,
  but fragile on the choice of that pixel.
- **Deproject-then-aggregate (euclidean reconstruction, `docs/target_localization/euclidean_reconstruction.md`):** deproject *all*
  masked pixels into 3D points first, then reduce the point set (e.g.
  centroid). Heavier, more robust, recovers full geometry.

Projective ranging is deliberately the first. When the single-pixel fragility matters, that
is the signal to spend the extra cost and move to euclidean reconstruction.

---

## 6. Implementation and checks

Production uses `localize_prepared_projective_ranging` on the shared prepared
region; `localize_projective_ranging` preserves the standalone full-grid API.
Both use the same selection and reduction. The representative pixel is the mean
foreground row/column, and Z is the median foreground depth. Too few valid or
isolated pixels yields no result, never substitution by another path. The node
performs the live optical-to-base conversion before publishing.

[Tests](../../src/ridgeback_autonomy/test/test_projective_ranging.py) cover the
algorithm; ROI parity is recorded in [migration history](../history/roi_mask_migration.md).
[Estimator history](../history/estimator_evolution.md) retains the comparison
with the deleted depth estimator. Comparative and hardware work is tracked in
[the backlog](../BACKLOG.md), not as unresolved interface decisions.
