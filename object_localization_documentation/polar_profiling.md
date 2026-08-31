# Polar profiling — Project-Then-Segment (2D LiDAR Route)

**Scope:** the LiDAR localization path, end to end — the only path that does not
consume the aligned depth frame. It takes a mask plus a planar 2D LiDAR scan,
transforms the scan into the camera frame, projects the scan points into the
image plane, keeps the points falling inside the mask, segments the surviving
1D range profile to reject parallax contamination, and reduces the near runs to
one planar coordinate. Because LiDAR is a single self-contained source (no
stereo/monocular producer split), this one document covers both the
acquisition contract and the path itself — the roles that
`aligned_depth.md` and `projective_ranging.md`/`euclidean_reconstruction.md` split between
them for the depth routes. The mask contract is `mask_component.md`; the
architecture-level view of polar profiling is `object_localization_pipeline.md` §5.

---

## 1. Inputs and output

**Inputs**

- A **mask** from the mask interface: a boolean selector on the RGB color
  grid, plus its precision tag (`tight` | `rect`). See `mask_component.md`.
  Same interface as projective ranging and euclidean reconstruction — polar profiling rejoins the pipeline only here.
- A **LaserScan** from the front Hokuyo UST (`sensors/lidar2d_0/scan`): 270°
  (±135°), 0.25° angular resolution, single horizontal plane at the LiDAR's
  mounting height. Range vs. bearing only — no height information.
- The **LiDAR→camera extrinsics**: the rigid transform from the scan frame to
  the camera optical frame, looked up from TF at the scan/detection timestamp.
  Factory-calibrated nowhere — on real hardware this transform must be
  calibrated (`object_localization_pipeline.md` §10.2); in sim it comes from
  the URDF and is exact.
- The **color-camera intrinsics** (`camera_info` of the grid the mask lives
  on) — needed to project the scan points into the image plane so the mask can
  select them. Same intrinsics caveat as the depth paths
  (`aligned_depth.md` §2.3): use the grid's `camera_info`, not the FoV
  constants in `camera_config.json`.

**Output**

- One planar coordinate `(X, Z)` in the **camera optical frame**, per mask
  (camera optical frame; the downstream `base_link` planar conversion is fixed —
  `object_localization_pipeline.md` §7).
  **Y (height) is unobservable** from a single-plane LiDAR: the result
  carries no Y at all, and the benchmark's optical→base conversion folds
  `Y = 0`, which is exact at the benchmark's zero camera pitch (§8, resolved).
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
step (`projective_ranging.md` §2.2): non-finite ranges, ranges below `range_min`
(mixed-pixel and self-hit artifacts), ranges beyond `range_max` / a sane
maximum. They must never survive into the range profile, where a stray 0.06 m
self-hit would masquerade as the nearest run.

### 2.2 Transform

Apply the LiDAR→camera-optical rigid transform to every point. After this the
points live in the same frame the output is expressed in, so no frame juggling
remains downstream. Two properties worth pinning:

- The transform comes from **TF at the matching timestamp**. On a moving
  platform an unsynchronized lookup smears the projected points against the
  mask — time synchronization is a pipeline-level open item
  (`object_localization_pipeline.md` §10.3), and polar profiling is its most sensitive
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
> `tight` branch. (`object_localization_pipeline.md` §5, polar profiling callout.)
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
   `object_localization_pipeline.md` §5):** on a legged object the nearest run
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
diverges" rule as projective ranging and euclidean reconstruction (`object_localization_pipeline.md` §6) — here the
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
hierarchy as the depth paths (`mask_component.md` §6.1): one object → one
detection → one mask → one coordinate; masks never compared or merged.

| Work | Frequency |
|------|-----------|
| convert + transform the scan; project to image pixels | zero or once per batch, shared by all usable masks |
| mask select, segment, merge, median | once per mask |

Projection is lazy: a batch with no scan, polar profiling disabled, or only
absent masks does no projection at all. It is never cached across batches,
because the scan, its matching TF transform, and its timestamp can change. The
per-scan work is tiny by depth-path standards — a 270° / 0.25° scan is ~1080
points against euclidean reconstruction's `H×W` pixels — so polar profiling is
computationally the cheapest path end to end, on top of being sensor-accurate
in its plane.

---

## 4. Failure modes and the fallback contract

Polar profiling has a structural failure mode the depth paths do not: **the scan plane
can miss the object entirely** (plane passes above/below it, or crosses a gap
in it). The path must distinguish and report:

- **No points selected / too few rays** (`< min_valid_rays`): the plane missed
  the object, the object is outside the LiDAR FoV overlap, or the scan was
  invalid → return `None`, mirroring `localize_projective_ranging` / `localize_euclidean_reconstruction`'s
  sparse fallback. The consumer routes to projective ranging / euclidean reconstruction
  (`object_localization_pipeline.md` §10.4).
- **Points selected but all far**: parallax-only content (camera sees the
  object, every LiDAR ray inside the mask flew past it). The zero-point check
  does *not* catch this — points exist, they are just wrong. Heuristic guard:
  if the nearest run's range wildly disagrees with a cheap prior (e.g.
  projective ranging's depth median, when available), flag low confidence
  rather than emit.
  Whether this cross-check belongs in the path or in the fusion stage is an
  open item (§8).
- **TF unavailable**: no extrinsics, no path — return `None` and warn, as the
  current node already does.

`None` is a first-class output: in the benchmark it is a skipped row, in the
pipeline it is the signal to fall back — never a zero or a stale value.

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
but **only the nearest one is drawn** — eight estimate rings and three ray layers
per robot are unreadable the moment a scene holds two. With no subscriber the
estimator still runs, but debug records, wedge selection, and marker construction
are skipped. `nearest_beam_record` ranks the batch with the shared
`nearest_instance_index` and matches the chosen index against
`PolarBeamRecord.detection_index`, never against the list position: a detection
whose segmentation came back empty records no beams, so the two disagree. Each
namespace then holds a single fixed marker id, which is what lets the drawn
instance change between frames without stranding the previous one's rays for a
full `ray_marker_lifetime_sec`. A batch nothing could rank still draws its first
record — beams with no estimate is precisely the failure worth seeing.

When subscribed, the beams are recorded **even when the path returns `None`**:
on a miss,
`selected_beams` is recomputed via `select_beams` and nothing is marked as used.
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
moment the §8 tuning campaign passes non-default knobs, the 2D overlay panel and the
RViz rays will disagree. Routing the panel through the same indices is a
separate change.

**Known divergence.** This node ranks the nearest instance from its own three
estimators, because nothing else has filled the batch by the time the rays are
published, while `target_visualization_node` ranks the merged measurement topics and
so starts at `pointcloud`. Two robots at near-equal range can therefore put the rays on
one and the estimate rings on the other for a frame. Both sides walk
`PUBLIC_ESTIMATOR_ORDER` and break ties on the lower index, which bounds the
disagreement to near-ties; removing it would mean publishing the chosen index
and accepting a frame of coupling between the nodes.

---

## 5. Where polar profiling wins (and what it lacks)

- **In-plane accuracy.** The UST's range accuracy and 0.25° angular resolution
  beat stereo depth at range; no alignment resampling, no occlusion holes, no
  monocular scale softness.
- **Independence.** A genuinely separate sensor and error model — the reason
  it is the natural cross-check on euclidean reconstruction's depth in the fusion stage
  (`object_localization_pipeline.md` §9).
- **Cheapest compute.** ~1k points per scan; segmentation is a 1D pass.
- **No Y.** Planar only; never a standalone 3D source.
- **Conditional availability.** Only fires when the scan plane intersects the
  object — availability depends on object height and range, unlike projective
  ranging / euclidean reconstruction which fire whenever the mask has valid depth.
- **Extrinsic + sync sensitivity.** A camera–LiDAR miscalibration or timestamp
  skew translates directly into wrong mask membership; the depth paths have no
  analogous inter-sensor coupling (their alignment is factory-calibrated or by
  construction).

---

## 6. Relationship to the incumbent lidar estimator

> **Historical.** The estimator this section compares against
> (`compute_lidar_measurement` in `geometry.py`, fed by
> `g1_lidar_measurement_node`) was **deleted** along with the rest of the legacy
> stack; polar profiling is now the only LiDAR-sourced row. The comparison is
> kept because it records why each design choice was made.

That estimator was a proto-polar-profiling with the same generalization gaps
its depth siblings had:

| Aspect | Deleted estimator | polar profiling target |
|--------|-------------------|---------------|
| Selection | bbox → bearing window (`compute_camera_bearing_window`, margin + floor padding); horizontal gate only | project points to `(u, v)`, index the actual mask (2D membership, forked on nothing) |
| Intrinsics | fx synthesized from `camera_config.json` HFoV | `camera_info` of the color grid (`aligned_depth.md` §2.3) |
| Contamination recovery | percentile anchor (30th) + fixed inlier margin (0.20 m) over planar range | segment runs over bearing → merge near-band runs → median (pinned convention, handles the two-legs case explicitly) |
| Output | vehicle-frame `lateral / forward / distance`, front offset applied | `(X, Z)` in the camera optical frame, Y = NaN; vehicle-frame transform belongs to a later consumer |
| Scan preprocessing | `extract_scan_points_base` (validity, base-frame Cartesian) | same responsibilities, retargeted at the camera optical frame |

The percentile-anchor inlier band is the incumbent recovery, exactly as the
range band was for euclidean reconstruction — it survives as a benchmark baseline, not as the
target design. Note the current estimator already transforms points into the
camera frame internally (steps 1–2 of `compute_lidar_measurement`) and then
transforms the answer *back* to the base frame; polar profiling simply stops at the
camera frame.

---

## 7. Implementation plan

Mirrors the projective ranging / euclidean reconstruction pattern: a pure core
module, the node as a thin shell, the estimator as an opt-in benchmark row. The
scan-validity clips started as by-value copies of the legacy estimator's; with
that estimator deleted, `polar_profiling.py` is their sole owner.

**`perception/target_localization/core/polar_profiling.py`** — the pure path, no ROS:

```python
@dataclass(frozen=True)
class PolarProfilingResult:
    xz_optical: np.ndarray        # (2,) X right, Z forward, meters; Y unobserved
    foreground_points: np.ndarray # (M, 2) the merged (X, Z) set, by-product
    ray_count: int                # rays that survived the mask ∩ FoV select

def localize_polar_profiling(
    points_optical: np.ndarray,   # (N, 3) scan in the camera optical frame
    valid: np.ndarray,            # (N,) scan-validity from preprocessing
    mask: Mask,
    intrinsics: CameraIntrinsics,
    *,
    range_jump_m: float = ...,        # run-split discontinuity threshold
    range_band_m: float = ...,        # near-band merge width
    max_bearing_gap_beams: int = ..., # run-split beam-gap threshold
    min_valid_rays: int = ...,
) -> PolarProfilingResult | None: ...
```

Internal structure mirrors the sibling modules: SELECT (project + mask index,
§2.3–2.4) → RECOVER (segment / merge / median, §2.5) → reduce, returning
`None` on the sparse fallback. The run segmentation lives in a separate
testable function (`segment_range_profile(...) -> list[runs]`), the LiDAR
analogue of the isolation recipes — and like them it can grow alternatives
behind the same contract if the benchmark motivates any (the incumbent
percentile band being the obvious first alternative row).

**`perception/target_localization/core/intrinsics.py`** — ~~add the forward projection
(`project_points(points, intrinsics) -> (uv, valid)`: an `(N, 2)` pixel
array plus an in-front-and-in-bounds selector), the inverse of the existing
`deproject_*` helpers; polar profiling is its first consumer.~~ — done
2026-07-23: `project_points` is implemented in `intrinsics.py` and consumed
by `polar_profiling.py`.

**Scan preprocessing** — a `scan_points_optical(...)` helper (validity clean +
polar→Cartesian + extrinsic transform, §2.1–2.2): the retargeted equivalent of
`extract_scan_points_base`, living in the new stack. Per-scan work, cached and
shared across masks per §3.

**Node wiring** (done) — polar profiling runs inside `target_mask_measurement_node`
alongside the two depth paths, keeping all new-stack rows in one node on the
camera-optical frame with `camera_info` intrinsics. The node gained a
`LaserScan` subscription (`scan_topic`) and a TF listener; per frame it looks
up the scan→optical extrinsic at the detection stamp and calls
`scan_points_optical` once (shared across masks), then `localize_polar_profiling`
per mask. The result's `(X, Z)` is converted to the shared base_link planar
convention via the full TF transform with `Y = 0` (exact at the benchmark's
zero camera pitch). Polar
runs independently of the depth frame — a missing scan or unavailable TF simply
leaves its fields NaN. The alternative (extending the legacy lidar node) was
rejected: it worked in the vehicle frame with `camera_config` FoV intrinsics,
not the color-grid `camera_info` this stack uses. That node has since been
deleted outright.

**Benchmark row** (done) — `polar_profiling` estimator key, opt-in via
`estimators:=`, published on `measurements/target/mask`. It is in `MASK_ESTIMATORS`
(so it launches the mask node) but not `DEPTH_PATH_ESTIMATORS`, so its
self-describing names fold only the gate: the CSV file is
`box_gated_polar_profiling.csv` and the summary/log prose is "box-gated polar
profiling" (no depth source, no isolation recipe). Distance convention: the
published scalar is the base_link planar distance of the median `(X, Z)`
coordinate — the node runs it through the same `optical_to_base_planar`
conversion as the depth-path rows (full TF transform with `Y = 0`, front
offset subtracted), so all mask rows, the legacy rows, and the ground truth
share one convention (the legacy rows named here have since been deleted). The core result carries no `distance_m` field; the
distance is derived here from the median coordinate, not from a separate
core statistic. The conversion applies the camera's full mounting pose
(rotation *and* translation) from TF, so the optical center is not assumed to
sit at the base origin. Any residual systematic bias the polar row inherits
from the depth-path rows is the G1's surface-vs-body-center offset (the
median lands on the near surface, ground truth is the model origin), not a
missing camera translation. First sim run (2026-07-13, 14 trials): box-gated
polar profiling MAE 0.093 m vs legacy `lidar` 0.064 m — LiDAR-accurate, the
gap attributable to untuned segmentation params (§8) and the leg-front-face
vs body-center convention offset (§8). Later standing runs have landed in the
~0.086–0.094 m range against legacy `lidar` ~0.064 m; treat 0.093 m as one
early data point and read the current figure from `benchmark-results/`.

**Tests** — synthetic-scan unit tests in `test_polar_profiling.py`: a two-legs
profile (band merge averages the legs), a parallax profile (far points inside
the mask are dropped), a plane-miss profile (returns `None`), a wall-behind
profile (background run rejected). The registry/output-name wiring is covered
in `test_benchmark_runner.py`. (The `test_geometry.py` lidar failure noted here
at 696e101 went away with the file: the legacy stack and its tests are gone.)

---

## 8. Open items

- **Camera–LiDAR extrinsic calibration** on real hardware — define the
  procedure, store the transform (`object_localization_pipeline.md` §10.2).
  Sim is exact via URDF; the sim benchmark will not exercise this error.
- ~~**Time synchronization**~~ — resolved 2026-07-23: the mask node matches the
  scan to the detection (mask) stamp via `StampedMessageBuffer.lookup_nearest`
  within `scan_match_tolerance_s` (and looks up the scan→optical TF at that same
  stamp), instead of pairing latest-scan with latest-detections. The static
  benchmark is unaffected; the win is correctness under motion — polar profiling
  being the most sensitive consumer (§2.2). Accurate sensor clocks on real
  hardware remain a driver concern the software matching relies on.
- **Segmentation parameters (tuning, deferred)** — `range_band_m`,
  `range_jump_m`, `max_bearing_gap_beams`, `min_valid_rays` are the LiDAR
  foreground-isolation knobs (the analogue of the `isolation_2d` /
  `isolation_3d` recipes); they are the source of the measured gap to the
  `lidar` row (0.093 vs 0.064 m) — see §7. **That baseline is no longer
  runnable:** `lidar` was deleted with the rest of the legacy stack, so 0.064 m
  is a recorded figure rather than something a rerun can reproduce. Tune
  against ground truth directly. Plan when picked up:
  1. **Tune on the benchmark, not exploration** — tuning needs ground truth
     (known spawn distance → MAE), which only the benchmark has; exploration
     has no labelled target to measure error against.
  2. **Harden the benchmark scenario first** — the `target_distance_calibration`
     world is a sterile empty room, so it barely stresses these
     background-rejection knobs (risk: over-fit, e.g. a huge `range_band_m`
     still scores well with no clutter to admit). Add a neighbour G1 / clutter /
     a closer back wall (keeping ground truth) so the isolation actually bites.
  3. **Wire the four as launch params first** — they are currently hardcoded
     defaults (the mask node calls `localize_polar_profiling` without passing
     them); expose them like `isolation_2d` / `isolation_3d` so a run can sweep
     them, then sweep and pin the values next to the mirrored constants.
  4. **Validate in exploration** — qualitative only (overlay under real clutter
     and motion); not for optimization.
  Note the systematic part of the gap is a convention offset, not a knob: the
  median of near runs sits on the leg *front face* while ground truth is body
  *center* (~0.1 m deeper). If sweeping does not close it, add a forward-depth
  compensation in the reduce step (the legacy row's `+0.20 m behind-anchor`
  margin is the precedent).
- **Parallax-only guard placement** — cross-check against projective ranging's depth
  inside the path vs. in the fusion stage (§4, second failure mode).
- **Fallback routing** — where the `None` → projective ranging / euclidean reconstruction escalation lives
  (consumer logic, not the path itself); shared with euclidean reconstruction's sparse-mask
  fallback item.
- ~~**Y convention**~~ — resolved 2026-07-13: the benchmark output is the
  shared base_link `lateral / forward / distance` triple (no separate Y
  field), and the optical→base conversion (full TF transform) folds `Y = 0`,
  which is exact at the benchmark's zero camera pitch. A non-zero pitch would
  need the true Y,
  which polar cannot observe — revisit only if the camera is ever pitched.
- **Front + rear merge (360°)** — out of scope; perception uses the front 270°
  (`lidar2d_0`) only. Revisit only if rear coverage ever matters for
  localization.
- ~~**Coordinate frame**~~ — resolved 2026-07-23: the camera-optical → `base_link`
  planar convention is fixed (`object_localization_pipeline.md` §7), same as
  projective ranging and euclidean reconstruction.
