# Polar profiling — Project-Then-Segment (2D LiDAR Route)

**Scope:** the LiDAR localization path, end to end — the only path that does not
consume the aligned depth frame. It takes a mask plus a planar 2D LiDAR scan,
transforms the scan into the camera frame, projects the scan points into the
image plane, keeps the points falling inside the mask, segments the surviving
1D range profile to reject parallax contamination, and reduces the near runs to
one planar coordinate. Because LiDAR is a single self-contained source (no
stereo/monocular producer split), this one document covers both the
acquisition contract and the path itself — the roles that
`docs/target_localization/aligned_depth.md` and `docs/target_localization/projective_ranging.md`/`docs/target_localization/euclidean_reconstruction.md` split between
them for the depth routes. The mask contract is `docs/target_localization/mask_representation.md`; the
architecture-level view of polar profiling is `docs/target_localization/target_localization_pipeline.md` §5.

---

## 1. Inputs and output

**Inputs**

- A **mask** from the mask interface: a boolean selector on the RGB color
  grid, plus its precision tag (`tight` | `rect`). See `docs/target_localization/mask_representation.md`.
  Same interface as projective ranging and euclidean reconstruction — polar profiling rejoins the pipeline only here.
- A **LaserScan** from the front Hokuyo UST (`sensors/lidar2d_0/scan`): 270°
  (±135°), 0.25° angular resolution, single horizontal plane at the LiDAR's
  mounting height. Range vs. bearing only — no height information.
- The **LiDAR→camera extrinsics**: the rigid transform from the scan frame to
  the camera optical frame, looked up from TF at the scan/detection timestamp.
  Factory-calibrated nowhere — on real hardware this transform must be
  calibrated (tracked in `docs/BACKLOG.md` under camera–LiDAR calibration); in sim it comes from
  the URDF and is exact.
- The **color-camera intrinsics** (`camera_info` of the grid the mask lives
  on) — needed to project the scan points into the image plane so the mask can
  select them. Same intrinsics caveat as the depth paths
  (`docs/target_localization/aligned_depth.md` §2.3): use the grid's `camera_info`, not the FoV
  constants in `camera_config.json`.

**Output**

- One planar coordinate `(X, Z)` in the **camera optical frame**, per mask
  (camera optical frame; the downstream `base_link` planar conversion is fixed —
  `docs/target_localization/target_localization_pipeline.md` §7).
  **Y (height) is unobservable** from a single-plane LiDAR: the result
  carries no Y at all, and the benchmark's optical→base conversion folds
  `Y = 0`, which is exact at the benchmark's zero camera pitch (Section 7, resolved).
  The scan-plane's own Y in the camera frame is available as a by-product, but
  it is the *plane's* height, not the object center's — do not substitute it.

Polar profiling is the accuracy specialist: within its plane the UST is typically more
accurate and longer-range than RealSense stereo, but it recovers no height and
only returns anything at all when the scan plane physically intersects the
object.

---

## 2. The five steps

```
1. CONVERT    polar (range, bearing) -> Cartesian points, scan frame
2. TRANSFORM  scan frame -> camera optical frame          # extrinsics (TF)
3. PROJECT    points -> (u, v) on the color grid          # pinhole, keep in-FoV
4. SELECT     keep points where mask[v, u]                # mask as index
5. RECOVER    segment 1D range profile -> merge near-band runs -> median
              -> (X, Z)                                   # same both tags
```

Steps 1–3 are batch-local work shared by all masks; steps 4–5 run per mask. The
projection keeps the full `(u, v)` result by original beam index plus compact
rounded in-view pixel coordinates, so every mask indexes one mapping rather
than re-projecting the scan. The
`tight` / `rect` tag does **not** fork the recovery here — both branches run
the same segmentation (see §2.5 for why); the tag only changes how wide the
admitted bearing window effectively is.

### 2.1 Convert

Standard polar→Cartesian over the scan array:

```python
angles = angle_min + np.arange(n) * angle_increment
x = ranges * np.cos(angles)
y = ranges * np.sin(angles)          # z = 0: single plane
```

Invalid returns must be dropped first — the LiDAR analogue of the depth clean
step (`docs/target_localization/projective_ranging.md` §2.2): non-finite ranges, ranges below `range_min`
(mixed-pixel and self-hit artifacts), ranges beyond `range_max`. Both bounds
are the driver's declared ones — the path applies no ceiling of its own, so a
far return is background for the segmentation to reject (§2.5), not something
quietly dropped here. They must never survive into the range profile, where a stray 0.06 m
self-hit would masquerade as the nearest run.

### 2.2 Transform

Apply the LiDAR→camera-optical rigid transform to every point. After this the
points live in the same frame the output is expressed in, so no frame juggling
remains downstream. Two properties worth pinning:

- The transform comes from **TF at the matching timestamp**. On a moving
  platform an unsynchronized lookup smears the projected points against the
  mask. The node already matches the nearest scan within its configured
  tolerance; validating physical sensor clocks remains in
  [camera–LiDAR calibration](../BACKLOG.md#camera-lidar-calibration). Polar profiling is its most sensitive
  consumer because the mask test (step 4) is pixel-exact.
- After the transform the scan is no longer at `z = 0`: in the optical frame
  (X right, Y down, Z forward) the plane sits at a Y determined by the
  mounting geometry (camera at 0.85 m, LiDAR near deck height). That Y is what
  makes full `(u, v)` projection possible in the next step.

### 2.3 Project

Forward pinhole projection through the color intrinsics:

```
u = fx * X / Z + cx
v = fy * Y / Z + cy      # valid only for Z > 0
```

Keep only points with `Z > 0` (in front of the camera) and `(u, v)` inside the
image bounds — this implements the "∩ camera FoV" clip: the LiDAR sees 270°
but the mask can only certify the camera's ~70–87°. Note `v` is *not*
constant: the scan plane is at fixed height, so nearer points project to
lower rows than farther ones. This is why the selection is a genuine 2D
mask test, not just a column/bearing gate.

### 2.4 Select

The mask indexes the projected points exactly as it indexes depth pixels in
projective ranging — the same one-line selection mechanic, in sparse form:

```python
keep = mask.contains_pixels(u_px, v_px)   # per-point membership test
```

This is the whole payoff of the mask interface for polar profiling: `tight` and `rect`
masks select scan points with the *same* line, and the front-end choice never
leaks into this path. The grid precondition applies unchanged: the mask and
the projection must share resolution and intrinsics, or the indices refer to
different rays.

One geometric subtlety unique to polar profiling: with a `tight` mask, the scan plane
may cross the object at a height where the silhouette is narrow (between the
G1's legs) — a perfectly valid mask can then select very few or zero points
even though the object is visible. This is the structural fallback case (§4),
not an error.

### 2.5 Recover — segmentation, not a fork

The selected points form a 1D **range profile** over bearing. Unlike
projective ranging and euclidean reconstruction, *both* mask tags run the same
recovery, because mask membership is not
sufficient here even when the mask is pixel-precise:

> **Parallax contamination.** The mask is defined from the camera's viewpoint;
> the LiDAR samples from a different position. Background points the camera
> cannot see — occluded behind the object — still project inside the mask and
> enter the profile carrying background ranges. Mask membership certifies that
> the *camera's* ray hits the object; it says nothing about a LiDAR point
> further along that ray. So a plain median over the arc is unsafe even on the
> `tight` branch. (`docs/target_localization/target_localization_pipeline.md` §5, polar profiling callout.)
>
> A pixel-precise segmenter does *not* close this hole. It excludes background
> the camera can see (e.g. wall visible between the G1's legs — those pixels
> are False, those beams are rejected at select). But an *occluded* point
> projects inside the silhouette by definition of occlusion: the camera ray
> toward it hits the object first, so its pixel is genuinely object. The
> sensors' vertical offset makes such points reachable: a beam in the ~0.2 m
> scan plane slips between the legs and hits the wall behind, while the camera
> ray from 0.85 m down to that same wall point crosses the robot's range at
> torso height — occluded, so the wall point lands on *torso* pixels, which
> are True in any correct tight mask. "Through the legs" for the LiDAR maps to
> "onto the pelvis" for the camera.

The recovery, identical for both tags:

1. **Segment** the profile into contiguous runs: walk the kept points in
   scan order (beam index order — the scan's native bearing order; no sort,
   and no reordering by camera-frame bearing, which parallax can make
   non-monotonic), split wherever the range jumps by more than a
   discontinuity threshold or the beam index gaps by more than a few
   increments — a gap means intervening beams hit something else or
   returned invalid.
2. **Merge the near band**: take the nearest run, then merge every run whose
   range lies within a small band of it. **Convention (pinned,
   `docs/target_localization/target_localization_pipeline.md` §5):** on a legged object the nearest run
   alone would be one leg — range = that leg's face, laterally offset from the
   body center; merging the band averages both legs in range *and* bearing.
3. **Median** the merged set: per-axis median of the merged points' `(X, Z)`
   — the coordinate the core returns. (No separate distance statistic is
   emitted here; the published distance is derived from this coordinate
   downstream, §7.)

   **Median over mean (pinned):** the mean would center better between the
   legs (per-axis median can snap toward the better-sampled leg when beam
   counts differ — bounded by half the leg spacing, ~0.1 m worst case), but
   the mean is dragged proportionally by anything that survives the band
   merge — a background run sneaking under `range_band_m`, clutter at a
   similar range, a neighbor's edge. Separating robot from background is the
   priority, so the reduction stays robust and the residual-contamination
   burden is not shifted onto it.

The only tag-dependent behavior is implicit: the `rect` mask admits a wider
bearing window, so more neighbor runs enter the profile and the segmentation
has more to reject; the `tight` mask narrows the window but changes nothing
about the algorithm. This is the same "fork sits where behavior genuinely
diverges" rule as projective ranging and euclidean reconstruction (`docs/target_localization/target_localization_pipeline.md` §6) — here the
behaviors do not diverge, so there is no fork.

The merged near-band set is polar profiling's **foreground set**, and it is the single
source for the output — the same single-source rule as projective ranging §2.3 and euclidean reconstruction
§2.4. The core emits only the coordinate: the per-axis median `(X, Z)` of
that set. There is no separate core distance statistic — the published
distance is derived downstream from the same coordinate (base-planar
projection, §7), so coordinate and distance rest on the same point.

---

## 3. Batch / per-mask granularity

Polar profiling runs **per mask** over shared per-scan work, with the same 1:1:1
hierarchy as the depth paths (`docs/target_localization/mask_representation.md` §6.1): one object → one
detection → one mask → one coordinate; masks never compared or merged.

| Work | Frequency |
|------|-----------|
| convert + transform the scan; project to image pixels | zero or once per batch, shared by all usable masks |
| mask select, segment, merge, median | once per mask |

Projection is lazy: a batch with no scan, polar profiling disabled, or only
absent masks does no projection at all. It is never cached across batches,
because the scan, its matching TF transform, and its timestamp can change. The
current scan contains roughly 1080 samples at 270° / 0.25°, while depth paths
work from image-grid selections. That establishes different input sizes, not an
end-to-end performance ranking; transforms, subscriptions, mask producer, and
hardware all contribute to cost.

---

## 4. Failure modes and the fallback contract

Polar profiling has a structural failure mode the depth paths do not: **the scan plane
can miss the object entirely** (plane passes above/below it, or crosses a gap
in it). The path must distinguish and report:

- **No points selected / too few rays** (`< min_valid_rays`): the plane missed
  the object, the object is outside the LiDAR FoV overlap, or the scan was
  invalid → return no result with the corresponding miss reason, mirroring the
  depth paths' sparse failures. No consumer currently substitutes another path.
- **Points selected but all far**: parallax-only content (camera sees the
  object, every LiDAR ray inside the mask flew past it). The zero-point check
  does *not* catch this — points exist, they are just wrong. A depth-consistency
  guard is a possible conditional extension, not current behaviour; its policy
  and placement would need a separate decision.
- **TF unavailable**: no extrinsics, no path — return `None` and warn, as the
  current node already does.

No result is first-class: the benchmark records a no-value outcome and reason;
the pipeline publishes no substitute, zero, or stale value for that estimator.

### 4.1 Seeing which beams were used (implemented)

`localize_polar_profiling` returns `selected_beams` and `merged_beams` alongside
the estimate — indices into the **original scan array**, not the selected subset.
`selected_beams` is the mask ∩ FoV select; `merged_beams` is what survived the
near-band merge and is therefore what the estimate medians over.

`target_mask_measurement_node` turns them into RViz markers on
`visualization/target/polar_rays`, in three namespaces (`polar/used`,
`polar/dropped`, `polar/wedge`) so each toggles independently in RViz. Markers
are built in the scan's own frame, where a beam is
`angle_min + i*angle_increment` at `ranges[i]`, so no extrinsics are re-applied.

While the marker topic has a subscriber, beams are recorded for every detection
but **only the nearest one is drawn** — four estimate rings plus three ray layers
for every robot would be unreadable when a scene holds two. With no subscriber the
estimator still runs, but debug records, wedge selection, and marker construction
are skipped. `nearest_beam_record` ranks the batch with the shared
`nearest_instance_index` and matches the chosen index against
`PolarBeamRecord.detection_index`, never against the list position: a detection
whose segmentation came back empty records no beams, so the two disagree. Each
namespace then holds a single fixed marker id, which is what lets the drawn
instance change between frames without stranding the previous one's rays for a
full `ray_marker_lifetime_sec`. A batch nothing could rank still draws its first
record — beams with no estimate is precisely the failure worth seeing.

When subscribed, selected beams are recorded **even when the path returns no
result**: the failed attempt carries its selected indices and nothing is marked
as used.
That is deliberate — the failure modes above are exactly what a viewer needs to
tell apart, and a frame where the estimator discarded the robot must not look
like a frame where nothing was there.

`beams_in_bbox` drives the wedge. It mirrors `mask._fill_box` exactly: round the
projection to a pixel as `select_beams` does, then test the half-open
`[x1, x2) × [y1, y2)`. Testing the raw float against the raw box instead
disagrees by one beam at each edge, which would make the wedge and the rays
contradict each other under a box gate, where they are the same set by
definition.

**Known divergence.** `perception/target_localization/core/rendering.py:polar_highlight_beams` still
re-runs `segment_range_profile` + `merge_near_band` at *library defaults* to
drive the 2D overlay panel, rather than reading the published indices. The node
calls `localize_polar_profiling` with no kwargs, so the two agree today. The
If the pipeline later exposes and passes non-default knobs, the 2D overlay panel
and RViz rays could disagree. Routing the panel through the same indices belongs
with any such scoped parameterization change.

**Known divergence.** This node ranks the nearest instance from its own three
estimators, because nothing else has filled the batch by the time the rays are
published, while `target_visualization_node` ranks the merged measurement topics and
so starts at `pointcloud`. Two robots at near-equal range can therefore put the rays on
one and the estimate rings on the other for a frame. Both sides walk
`PUBLIC_ESTIMATOR_ORDER` and break ties on the lower index, which bounds the
disagreement to near-ties; removing it would mean publishing the chosen index
and accepting a frame of coupling between the nodes.

---

## 5. Sensor and geometry tradeoffs

- **Independent sensor.** LiDAR has a distinct error model and needs no depth
  alignment or monocular inference. Cross-checking it against a depth path is
  possible, but no fusion or confidence policy is implemented.
- **Sparse 1D input.** The recovery operates on ordered scan runs rather than
  an image grid; actual end-to-end cost is a measurement question.
- **No Y.** Planar only; never a standalone 3D source.
- **Conditional availability.** Only fires when the scan plane intersects the
  object — availability depends on object height and range, unlike projective
  ranging / euclidean reconstruction which fire whenever the mask has valid depth.
- **Extrinsic + sync sensitivity.** A camera–LiDAR miscalibration or timestamp
  skew translates directly into wrong mask membership; the depth paths have no
  analogous inter-sensor coupling (their alignment is factory-calibrated or by
  construction).

---

## 6. Implementation and integration

`core/polar_profiling.py` owns scan projection, per-mask selection, range-profile
segmentation, near-band merging, and reduction. `project_scan_to_image` creates
shared geometry lazily, at most once per batch; `localize_projected_polar_profiling`
selects independently for each mask and returns an attempt with a miss reason.
The standalone `localize_polar_profiling` convenience API remains available.
No selected beams or per-detection foreground sets are shared across masks.

The node matches a scan near the detection stamp within `scan_match_tolerance_s`,
looks up the scan-to-optical transform at that stamp, and converts a successful
XZ result to the base-planar convention using optical Y = 0. This approximation
needs re-evaluation for a pitched-camera deployment. Missing depth does not
prevent the polar path from running.

`polar_profiling` is part of the default `estimators=all` registry and publishes
on `measurements/target/mask`. Its output names include the mask gate but no
depth-source or rect-isolation token. No fallback measurement is substituted.

`range_band_m`, `range_jump_m`, `max_bearing_gap_beams`, and `min_valid_rays`
remain core function arguments: the current pipeline calls their defaults and
does not expose them as launch sweep knobs. [Tests](../../src/ridgeback_autonomy/test/test_polar_profiling.py)
cover synthetic profiles, mask membership, projection, and sparse failures.

## 7. Evidence and validation boundaries

[Estimator history](../history/estimator_evolution.md) preserves the deleted
LiDAR-row comparison and early scores. That baseline is not runnable today,
and those results do not isolate a tuning effect from a surface/center offset.
Any future tuning should compare against ground truth on existing clutter and
occlusion scenes; exploration is only a qualitative check.

[Calibration](../BACKLOG.md#camera-lidar-calibration) and
[occlusion characterization](../BACKLOG.md#occlusion-characterization) remain
active work. Fusion/parallax guards, fallback routing, and rear-scan merging are
conditional extensions described in the [pipeline](target_localization_pipeline.md#9-conditional-extensions-not-current-behaviour),
not missing pieces of the current independent-estimator contract.
