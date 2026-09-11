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
  constants any static config might carry.

**Output**

- One planar coordinate `(X, Z)` in the **camera optical frame**, per mask
  (camera optical frame; the downstream `base_link` planar conversion is fixed —
  `docs/target_localization/target_localization_pipeline.md` §7).
  **Y (height) is unobservable** from a single-plane LiDAR: the result
  carries no Y at all, and the benchmark's optical→base conversion folds
  `Y = 0`, which is exact at the benchmark's zero camera pitch and is flagged in
  §6 as needing re-evaluation for a pitched-camera deployment.
  The scan-plane's own Y in the camera frame is available as a by-product, but
  it is the *plane's* height, not the object center's — do not substitute it.

Within its plane the UST is typically more accurate and longer-ranged than
RealSense stereo as a *sensor*, and this path no longer narrows that range
(§2.1). The path built on it is not automatically the accuracy specialist,
though: measured, its median error is excellent and its tail is not (§7). It
recovers no height, and it returns something only when the scan plane both
intersects the object **and** projects inside the image — the second condition
is the binding one at close range (§7).

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
quietly dropped here. A sub-`range_min` self-hit must never survive into the
range profile, where it would masquerade as the nearest run and anchor the band
onto nothing.

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
but the mask can only certify the colour camera's own, which is 71.6° as the sim
renders it and is read off the driver's `camera_info` on hardware rather than
assumed. Note `v` is *not*
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
   discontinuity threshold. Range structure is the only split signal: a
   beam-index gap is deliberately *not* one. Two objects at the same range
   across a gap merge back in step 2 regardless, which is what that split was
   once supposed to prevent, so all it did was re-cut one continuous surface.
2. **Merge the near band**: anchor on the **nearest point** in the profile, then
   merge every run that *reaches* within `range_band_m` of it — a run is
   represented by its **minimum**, not its median. The representative matters:
   under a median the outcome depended on how points happened to be partitioned,
   because splitting a run changes both pieces' medians and can pull a piece
   *into* a band the whole run sat outside of. Minima cannot do that, so a finer
   partition can only drop points, never add them, and the two thresholds become
   independent — `range_jump_m` decides what is connected, `range_band_m` how
   far a *disconnected* surface may sit.
   Kept runs are kept **whole**, which is the point of segmenting at all:
   connectivity vouches for the far end of a surface whose near end is in the
   band, so an oblique face deeper than `range_band_m` survives instead of being
   truncated at it.
   **Convention (pinned,
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

`localize_projected_polar_profiling` returns both beam sets as indices into the
**original scan array**, not the selected subset. `selected_beams` — the mask ∩
FoV select — sits on the returned `PolarProfilingAttempt`, because a **miss** has
one too; `merged_beams`, what survived the near-band merge and therefore what the
estimate medians over, sits on the `PolarProfilingResult` inside it, which only a
hit has. Splitting them that way is why there is exactly one place to read each
from. The standalone `localize_polar_profiling` wrapper returns only the result,
so it does not report a selection at all.

`target_mask_measurement_node` turns them into RViz markers on
`visualization/target/polar_rays`, in three namespaces (`polar/used`,
`polar/dropped`, `polar/wedge`) so each toggles independently in RViz. Markers
are built in the scan's own frame, where a beam is
`angle_min + i*angle_increment` at `ranges[i]`, so no extrinsics are re-applied.

While either beam consumer has a subscriber, beams are recorded for every
detection but **only the nearest one is drawn as rays** — four estimate rings
plus three ray layers for every robot would be unreadable when a scene holds two.
The 2D panel has no such clutter problem and therefore shows **every** detection;
that difference is intentional, and it is what lets the panel range two robots at
different distances where the old union-of-masks single band kept only the
nearer. With neither subscribed the estimator still runs, but debug records,
wedge selection, and marker construction are skipped. `nearest_beam_record` ranks
the batch with the shared
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

### The 2D panel consumes the same indices

The same records go out verbatim on `debug/target/polar_beams` as a `PolarBeams`
message, and `target_overlay_node` renders its LiDAR panel from that. The panel
computes no selection of its own — no mask, no range profile, no band — so it
cannot depict a different run than the one it is drawn on. The message is the
union of `selected` and of `merged` across the frame's detections; their
difference is the dropped set, which the panel colours exactly as the
`polar/dropped` ray layer does, so the two surfaces read beam for beam.

This replaced a second implementation of the algorithm inside the renderer, which
re-ran `segment_range_profile` + `merge_near_band` at *library defaults* against
its own silhouette-union mask and its own latest scan. That was harmless while
nothing could pass anything else, and stopped being harmless when the three
isolation settings became node parameters (§6): a sweep setting
`polar_range_band_m` moved the estimate and the RViz rays while the panel kept
highlighting the default band. It also collapsed the frame to one band, blanking
a second robot at a different range, and had no ray floor, so it highlighted
merges the estimator rejected with `TOO_FEW_RAYS_MERGED`. Forwarding the three
parameters to the overlay would have fixed only the first of those and left two
copies of the algorithm to keep in sync by hand.

Two stamps travel with the indices, and both are load-bearing. The header carries
the **measurement** stamp, so the overlay keys the message exactly as it keys the
silhouette artifact on `debug/target/mask`. `scan_stamp` plus `scan_frame_id`
name the **scan array the indices index into**; the overlay caches scans by that
key rather than keeping a latest-wins slot, because the scan a measurement was
made on is rarely the newest by the time the panel renders it. `beam_count` is a
hard guard: a cached scan of a different length means the indices would land on
beams the estimator never touched, so the panel drops the highlight and draws the
scan plain. Every failure here — no beams message yet, a scan aged out of the
cache, a length mismatch — degrades to a *missing* highlight, never a misaligned
one.

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
  object *and* projects inside the image — availability depends on object height
  and range, unlike projective ranging / euclidean reconstruction which fire
  whenever the mask has valid depth. The second condition puts a hard floor at
  ≈1.27 m on this mount (§7.1), which no parameter can lift.
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
Two debug artifacts accompany it, both subscriber-gated and both built from the
one list of `PolarBeamRecord`s: the RViz rays on `visualization/target/polar_rays`
and the beam indices on `debug/target/polar_beams` (§4). The topic names live in
`contracts.py`, so the mask node and the overlay node default to the same string
and no launch file has to pass it.

`range_jump_m`, `range_band_m`, and `min_valid_rays` are node parameters and
launch arguments — `polar_range_jump_m`, `polar_range_band_m`,
`polar_min_valid_rays` — declared at exactly their shipped values, so a run that
sets none of them is the run that was always happening. They are the only axes
polar profiling has beyond the mask gate, and none of them reaches a depth row or
the point cloud path. The `polar_` prefix is deliberate: `range_band_m` would sit
one line from `isolation_2d_band_m` in the same node, and the two answer to
different sensors — that one is a depth window around a camera anchor, this one a
merge distance between LiDAR bearing runs.

Neither the jump nor the band is grounded in a measurement, and the 0.30 jump is
known to sit *below* the G1's own 0.4457 m fore/aft extent (§7).
[Tests](../../src/ridgeback_autonomy/test/test_polar_profiling.py)
cover synthetic profiles, mask membership, projection, and sparse failures.

## 7. Evidence and validation boundaries

[Estimator history](../history/estimator_evolution.md) preserves the deleted
LiDAR-row comparison and early scores. That baseline is not runnable today,
and those results do not isolate a tuning effect from a surface/center offset.
Any future tuning should compare against ground truth on existing clutter and
occlusion scenes; exploration is only a qualitative check.

The published-beam-index contract that drives both the 3D markers and 2D LiDAR
panel is implemented and covered by focused tests. No post-2026-09-10 in-sim
visual acceptance run is recorded, so treat visual integration as awaiting a
smoke check even though the data contract is current.

### 7.1 A near-range floor the geometry imposes

The scan plane sits **0.686 m below the camera** (camera optical origin 1.028 m
above `base_link`, `lidar2d_0_laser` 0.342 m), so it projects to
`v = fy·0.686/Z + cy`. At `fy = 443.53`, `cy = 240` on a 480-row frame:

| target range | scan-plane row |
|---|---|
| 0.84 m | 602 — off-frame |
| 1.09 m | 519 — off-frame |
| **1.27 m** | **480 — the floor** |
| 2.45 m | 364 |
| 10.0 m | 270 |

**Below ≈1.27 m the scan plane projects off the bottom of the image**, so the
in-FoV clip (§2.3) removes every beam that could have hit the target and the mask
can only ever select background. This is structural, not a tuning failure: no
value of any parameter in §6 recovers it. It is a stronger condition than "the
scan plane intersects the object", and it is the one that binds up close.

### 7.2 Measured, 2026-09-09

`artifacts/benchmarks/20260909_210559_polar_validation`, polar alone, box gate,
full scenario set, 109 trials:

| | scored | MAE | median | p95 |
|---|---|---|---|---|
| this run | 94 | 0.5774 | 0.0760 | 1.3565 |
| minus 4 wall latches | 90 | 0.1687 | 0.0740 | 0.7333 |

**Four trials carry 72 % of all error**, every one of them reporting the
`wall_east` inner face at ~11.9 m instead of the target: `near_clip_03`
(0.84 → 11.652), `near_clip_01` (1.09 → 11.693), `interfere_samerange_01`
(2.45 → 11.689), `objpartial_04` (3.59 → 12.027). The first two are §7.1; the
other two have an occluder standing in the scan plane (a table at 2.17 m, an IV
pole at 2.37 m). All four are the same shape — the target contributes no beams,
so the wall is the nearest run — and that is the parallax failure §4 records as
**unguarded**.

Read the median, not the MAE, when judging a change to §2.5: the core reduction
is accurate and the distribution is all tail.

Two caveats. The comparison point (`20260828_152011`, MAE 0.1496) predates the
D455 render-pose fix, so it is a different camera setup, not a controlled A/B.
And these numbers are *worse* than that baseline because the 10 m scan clip was
removed (§2.1) — that clip was suppressing the wall by accident of this world's
geometry, not guarding against it, and hiding the failure was judged worse than
showing it.

A mask-height consistency guard would catch all four (a G1 at range `Z` subtends
`fy·1.3228/Z` px, and each latch is 3.4–9.6× too tall for the range it reports),
but it is not implemented: choosing its threshold needs per-trial mask geometry,
which is deliberately outside what this benchmark measures.

[Calibration](../BACKLOG.md#camera-lidar-calibration) and
[occlusion characterization](../BACKLOG.md#occlusion-characterization) remain
active work. Fusion/parallax guards, fallback routing, and rear-scan merging are
conditional extensions described in the [pipeline](target_localization_pipeline.md#9-conditional-extensions-not-current-behaviour),
not missing pieces of the current independent-estimator contract.
