# Path A — Aggregate-Then-Deproject (2D Depth-Image Route)

**Scope:** the cheapest of the three localization paths. It takes a mask plus an
aligned depth frame, reads the depth at the masked pixels, reduces them to a
single distance, and deprojects one representative pixel into a 3D point. This
document describes Path A end to end. The mask object it consumes is defined in
`mask_component.md`; how the aligned depth frame is produced (stereo alignment
or monocular estimation) is defined in `depth_based_path.md`. This doc assumes
both contracts and does not re-describe them. Path B, the point-domain sibling,
is `depth_based_B.md`.

---

## 1. Inputs and output

**Inputs**

- A **mask** from the mask interface: an `H×W` boolean array on the RGB color
  grid, plus its precision tag (`tight` | `rect`). See `mask_component.md`.
- An **aligned depth frame**: a depth image that is 1:1 with the RGB pixels, so
  that depth pixel `(u, v)` is the same ray as color pixel `(u, v)`. Producing
  this is a separate component (`depth_based_path.md`); Path A assumes it is
  already aligned.

**Output**

- One coordinate `(X, Y, Z)` in the **camera optical frame**, per mask. (The
  exact frame convention is proposed in `Object_Localization_Pipeline.md`
  Section 7; confirmation against SDK/TF still open.)

Path A is the cheapest path because it never builds a 3D structure — it collapses
the masked depths to a single number and deprojects exactly one pixel.

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

A depth frame contains invalid pixels even inside a perfect mask, so they must be
filtered **before** aggregating:

- **0** — no depth return (occlusion gaps left by alignment, surfaces too close,
  specular reflections, beyond max range).
- **NaN / inf** — undefined values.
- **out of range** — values past a sane maximum are far-field noise.

```python
valid = np.isfinite(depths) & (depths > 0.0) & (depths <= depth_max)
depths = depths[valid]
```

Skipping this biases the aggregate. Zeros in particular pull the result toward
zero, making the object look closer than it is.

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

- **Input:** the aligned depth frame and the mask (both `H×W` on the color grid).
- **Output:** the **foreground pixels** — the subset of the masked pixels that
  belong to the object (the G1), with the background pixels dropped. Concretely,
  the pixel coordinates inside the frame that the strategy classifies as object;
  the depth values and the centroid both follow from those coordinates.

The candidate recipes — from the nearest-depth-mode histogram baseline up to
box-prompted SAM — are catalogued with pros, cons, and citations in
`foreground_isolation_2d.md`. Each decides which masked pixels are foreground
and returns exactly that set. Choosing a recipe is a config choice, and the
benchmark swaps recipes behind the fixed contract to compare them against
ground truth.

This foreground set is the single source for both remaining steps: the
**aggregate** takes the median of the foreground pixels' depths, and the
**deproject** (§2.4) takes the centroid of the foreground pixels' coordinates as
the representative pixel. Because both read the *same* set, they agree by
construction.

The practical consequence of the fork: choosing the cheap box detector also puts
Path A on the heavier `rect` branch — but here the extra work is small (a
histogram or a weighting), which is exactly why Path A is the path where "the box
stays cheap."

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
valid masked pixels. Path A's entire answer rides on this one pixel. If it lands
on a depth discontinuity (object edge against far background) its `Z` is wrong
even when the aggregated range was correct. Taking the centroid of the foreground
pixels keeps the pixel on the object.

---

## 3. Depth-source agnostic

Path A does not care *how* the aligned depth frame was produced. The same four
steps run unchanged on RealSense stereo depth (after alignment) and on
Depth-Anything monocular depth (after metric scaling). Both producers and the
contract they converge to are specified in `depth_based_path.md`; the depth
source is a swappable input and both are benchmarked through the identical
path.

---

## 4. Batch / per-mask granularity

Path A runs **per mask**, looped within a frame, over a shared depth frame. The
hierarchy is strictly 1:1:1 (`mask_component.md` §6.1): one visible object →
one post-NMS detection → one mask → one `(X, Y, Z)`. Two G1s in view means two
masks and two independent Path A runs — masks are never compared, merged, or
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

## 5. Two aggregation philosophies (where Path A sits)

It is worth being explicit about *order of operations*, because it separates
Path A from Path B:

- **Aggregate-then-deproject (Path A):** collapse the masked depths to one
  number, then deproject one representative pixel. Cheap, yields a single point,
  but fragile on the choice of that pixel.
- **Deproject-then-aggregate (Path B, `depth_based_B.md`):** deproject *all*
  masked pixels into 3D points first, then reduce the point set (e.g.
  centroid). Heavier, more robust, recovers full geometry.

Path A is deliberately the first. When the single-pixel fragility matters, that
is the signal to spend the extra cost and move to Path B.

---

## 6. Relationship to the current stack

The repository today has a depth estimator that is close to Path A but not
identical, and the differences define the work:

| Aspect | Current estimator | Path A target |
|--------|-------------------|---------------|
| Region | inner "focus" crop of the box + Gaussian center weighting | the actual mask, forked on tag |
| `rect` foreground | crop + weight (assumes the object is centered) | pluggable isolation strategy (`foreground_isolation_2d.md`) |
| Order | deprojects every ROI pixel, averages the *distances* | aggregate depth, deproject one pixel |
| Output | a scalar range, in the **vehicle** frame, with a front offset applied | `(X, Y, Z)` in the **camera** frame |

In other words the current estimator is permanently on a `rect`-style branch,
approximates foreground recovery with a fixed crop, and emits a range rather than
a point. Path A generalizes it: replace the crop with the mask, add the tag fork,
and emit a camera-frame coordinate without the vehicle-frame transform (which
belongs to a later consumer/fusion stage, once the frame convention is fixed).

---

## 7. Open items

- ~~**Representative-pixel rule**~~ — resolved 2026-07-12: the representative
  pixel is the mean row/column of the foreground pixel set
  (`perception/core/path_a.py`); a sparse mask falls under the invalid-depth
  fallback below.
- **`rect` foreground recipe** — the `rect` branch is a pluggable strategy
  (input: depth frame + mask; output: the foreground pixel set). Choose among
  the candidates in `foreground_isolation_2d.md` and document the thresholds,
  then benchmark them behind the fixed contract.
- **Coordinate frame** — confirm the camera-frame axes/handedness against the SDK
  and TF tree (`Object_Localization_Pipeline.md` Section 7) before integration.
- ~~**Invalid-depth fallback**~~ — resolved 2026-07-12: skip —
  `localize_path_a` returns `None` when fewer than `min_valid_pixels` valid
  (or foreground) pixels remain.
