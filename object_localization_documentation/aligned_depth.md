# The Aligned Depth Frame — Depth Acquisition

**Scope:** how the depth that the localization paths consume is produced. Both
projective ranging (`projective_ranging.md`) and euclidean reconstruction (`euclidean_reconstruction.md`) consume one and the
same artifact — the **aligned depth frame** — and neither cares how it was
made. This document defines that contract and the two sources that satisfy
it: the physical depth camera (`camera → aligned depth`) and monocular
estimation (`RGB frame → aligned depth`). Both live in
`perception/core/depth_sources.py` and are pulled by
`g1_mask_measurement_node` at the detection stamp — there is no depth
producer process (`aligned_depth_coverage.md` §6).

```
                 ┌───────────────────────────┐
   D435 depth ──▶│ stereo source:            │
   (sensor)      │ capture → align to color  │──┐
                 └───────────────────────────┘  │   ┌─────────────────────┐
                                                ├──▶│ ALIGNED DEPTH FRAME │──▶ projective ranging / euclidean reconstruction
                 ┌───────────────────────────┐  │   │  (one contract)     │
   RGB frame  ──▶│ monocular source:         │──┘   └─────────────────────┘
   (color)       │ Depth-Anything → metric   │
                 └───────────────────────────┘
```

---

## 1. The contract

An **aligned depth frame** is a depth image that satisfies:

- **Grid:** `H×W` equal to the RGB color image; depth pixel `(u, v)` lies on
  the *same ray* as color pixel `(u, v)`. This is the property the mask
  interface depends on — masks are defined on the color grid
  (`mask_component.md`), and both paths index depth (or points deprojected
  from it) with the mask directly.
- **Units:** metric depth in meters, float. (The RealSense driver natively
  publishes `16UC1` millimeters; conversion to float meters is part of the
  source, not the localization path.)
- **Invalid pixels:** `0`, `NaN`, or `inf` mean "no depth here"; consumers
  filter them (`projective_ranging.md` §2.2) and sources must not encode invalid
  as any other value.
- **Usable ceiling:** every source declares `usable_max_m` — the farthest
  reading it can produce that still means something. This is a property of the
  sensor or the network (see §3), *not* of the scene, and it is deliberately
  distinct from the working depth gate the node applies (`depth_max_meters`).
  The two answer different questions — "can this value be believed" versus "how
  much of the scene do we want to admit" — and the node cleans against the
  tighter of them. They were one 10 m constant until 2026-08-27; the cost of
  that conflation is written up in `foreground_isolation_3d.md` §2.

  Only monocular declares a finite ceiling, and it *derives* it rather than
  naming it. Stereo declares none, as of 2026-08-28. There is no honest number
  to put there: Intel's D400 datasheet (337029-017) gives the D435 as "0.2 m to
  over 3 m (varies with lighting conditions)", while the simulated camera is an
  exact render out to a 100 m far clip
  (`clearpath_sensors_description/urdf/intel_realsense.urdf.xacro`, no noise
  model). The 10 m that used to sit there matched neither and was inherited
  from `POINTCLOUD_MAX_METERS`. Out-of-range readings are already excluded by
  the invalid-pixel rule above, which both sources satisfy — a value past what
  a source can resolve arrives as `0`/`NaN`/`inf`, not as a confident number.
- **Timing:** stamped with the color frame it is aligned to, so mask and depth
  can be matched frame-to-frame. The frame is *made* at that stamp: the mask
  node buffers the source's input stream raw and converts only the frame the
  detections were made on, so "the depth at stamp `T`" is a lookup into a
  buffer this process filled, not an intersection with another process's
  thinning (`aligned_depth_coverage.md`).

Everything downstream — select, clean/isolate, aggregate, deproject — is
identical regardless of which source made the frame. The depth source is a
**strategy**: a config choice behind this one contract, and an axis of the
benchmark matrix (`object_localization_pipeline.md` §8).

---

## 2. Source 1: depth camera → aligned depth (stereo)

### 2.1 Real hardware (D435)

The D435 computes depth by IR stereo matching. That depth image is native to
the **left-IR (depth optical) frame**, not the color camera: different sensor
position, different FoV (depth 87°×58° vs. color 69°×42°), potentially
different resolution. Raw depth pixel `(u, v)` is therefore *not* the same ray
as color pixel `(u, v)` — using it against a color-grid mask is a grid
mismatch, not a small error.

**Alignment** fixes this: reproject each depth pixel into 3D, transform by the
depth→color extrinsics, and re-render onto the color grid. The RealSense
driver does this on demand (`align_depth.enable: true` → the
`aligned_depth_to_color` image topic). The stereo source's `depth_topic` must
point at *that* topic on hardware: the source converts units, it does not
align. Consequences to design around:

- **Occlusion holes.** Alignment is a reprojection from a different viewpoint;
  surfaces visible to the depth sensor but hidden from the color camera leave
  invalid (0) pixels, concentrated at object edges — exactly where masks are.
  This is why the clean step is not optional.
- **Resampling.** Depth values are interpolated/selected onto a new grid;
  fine structure can shift by a pixel.
- **Cost.** The align filter runs in the driver. It is the price of admission
  for every mask-based consumer, so for cost accounting it is a *sunk* cost —
  see `pointcloud_provenance_test.md` §2.
- **Configured but unverified hardware path.** `clearpath/robot.yaml` now sets
  `align_depth.enable: true` and `enable_sync: true`; the repository's
  Clearpath parser test confirms both values survive into generated RealSense
  parameters. The robot still needs validation that the driver publishes
  `sensors/camera_0/aligned_depth_to_color/image_raw` on the color grid with
  sensor-owned, synchronized headers.

### 2.2 Simulation (gz `rgbd_camera`)

The sim camera is a GPU render: depth comes from the Z-buffer of the *same
virtual camera* that renders RGB. Depth and color are **co-registered by
construction** — same origin, same FoV, same grid — so the raw sim depth
topic already satisfies the contract with no align step and no occlusion
holes. Published as `32FC1` meters on `sensors/camera_0/depth/image`.

This convenience is also a trap: code that works on raw sim depth will break
on raw real depth. Routing *real* through the driver's align filter and *sim*
straight through is a launch-wiring choice (`depth_topic`), not a code branch —
`decode_depth_to_meters` accepts `16UC1`/`mono16`/`32FC1`, so both grids reach
the same contract unchanged. Divergences to keep in mind (details in
`object_localization_pipeline.md` §2):

| | Sim | Real |
|---|---|---|
| Depth origin | color camera's Z-buffer | left-IR stereo, reprojected |
| Alignment | by construction | driver filter, must be enabled |
| FoV | 71.6° (`horizontal_fov` 1.25 rad) | 87°×58° depth / 69°×42° color |
| Resolution | 640×480 @ 30 (profile keys in `robot.yaml` are stale — see pipeline doc) | per `depth_module.profile` |
| Encoding | `32FC1` m | `16UC1` mm |
| Noise / holes | none (clean render) | speckle, dropouts, alignment holes |

### 2.3 Intrinsics caveat

Deprojection (projective ranging §2.4, euclidean reconstruction §2.1) needs the intrinsics *of the grid the
frame lives on* — after alignment that is the **color** camera's intrinsics.
The **deleted legacy estimators** derived intrinsics from
`config/camera_config.json` FoV values (87°/58° — the real depth FoV), which was
wrong for the sim render (71.6°) and wrong-in-principle for aligned real depth
(color FoV). No surviving path does this: the mask node
subscribes to the **color** camera's `camera_info` directly and deprojects with
it (resolved 2026-07-21; the republish hop went away with the producer node on
2026-08-26). `grid_mismatch_warning` skips the frame's paths if that grid and
the detection grid ever disagree. The provenance test self-calibrates as a
cross-check (`pointcloud_provenance_test.md` §3).

---

## 3. Source 2: RGB frame → aligned depth (Depth-Anything)

The monocular source needs no depth sensor at all: the RGB frame goes through
**Depth-Anything V2** (Metric Indoor Small checkpoint, via HuggingFace
`transformers`), which predicts a dense metric depth map. This document's
source is `depth_sources.MonocularDepthSource` (selected by the mask node's
`depth_source` param, and run on the color frame at the detection stamp — once
per detection batch, not once per camera frame). It is the only consumer of the
network now; the legacy camera node that also ran it has been deleted.

When the silhouette gate and monocular source are both selected, SlimSAM and
Depth-Anything share one batch-local, contiguous RGB preparation. The mask
node first reuses the source's early exact-stamp color lookup when it exists;
when it misses, the silhouette gate keeps its later exact-stamp lookup and a
hit there can still drive both models. This changes no topics, subscriptions,
or QoS settings, and never falls back to a nearest color frame.

- **Aligned by construction.** The network's input *is* the color image, so
  its output is per-color-pixel — the grid property costs nothing. Resized to
  `H×W` if the network ran at a different internal resolution, it satisfies
  the contract directly.
- **Metric scaling.** The checkpoint predicts metric depth for indoor scenes;
  any residual global scale/bias correction (e.g. against a known reference)
  is part of this producer, not the consumers.
- **Metric is a property of the checkpoint, not of Depth-Anything.** The
  architecture ships in two flavours behind identical plumbing, selected by
  `config.depth_estimation_type`. `relative` checkpoints end in `ReLU` and emit
  affine-invariant **inverse** depth (larger means nearer, no units);
  `metric` ones end in `Sigmoid` scaled by `config.max_depth` and emit meters.
  The HuggingFace pipeline returns both as `predicted_depth`, and `produce`
  reads that as meters — so pointing `depth_anything_model_id` at a relative
  checkpoint would not raise: it would publish unitless disparity as a
  distance, and enough of it would clear the depth gate to look plausible.
  `MonocularDepthSource._resolve_usable_max` therefore reads the config on load
  and **refuses** a non-metric checkpoint rather than let it reach the
  estimators.
- **The ceiling comes from that same config.** `usable_max_m` is
  `config.max_depth × 0.9`. The head cannot emit beyond `max_depth` at all
  (20 m for Metric Indoor, 80 m for Metric Outdoor), and the top of that range
  is where the sigmoid saturates rather than where the scene is — predictions
  crowd toward the ceiling instead of resolving against it — so the last tenth
  is treated as no-return. Nothing here is hardcoded to the Indoor figure;
  swapping checkpoints moves the ceiling with them.
- **Error character is inverted vs. stereo.** Stereo depth has *holes and
  speckle* but locally accurate values; monocular depth is *dense and smooth*
  (no invalid pixels, no occlusion holes) but can be globally off in scale and
  soft at depth discontinuities. The clean step passes almost everything; the
  burden shifts to the aggregate/isolate stage, and the benchmark rows differ
  accordingly (sim run of 2026-07-13: `sensor_depth` MAE 0.586 m vs.
  `depth_anything` MAE 0.931 m; see `benchmark-results/` for the run summaries —
  these figures may be superseded by a later run).
- **No published cloud.** The monocular source produces only a depth image.
  euclidean reconstruction reaches it exclusively through in-code deprojection — the reason the
  deprojected provenance is euclidean reconstruction's canonical input
  (`pointcloud_provenance_test.md`).

---

## 4. The strategy pattern

Two sources, one contract, and every localization path is source-blind:

```
stereo source     = capture -> (real: align) -> float meters -> frame
monocular source  = RGB -> Depth-Anything -> metric scale    -> frame

each also declares usable_max_m; the node cleans against
min(usable_max_m, depth_max_meters). Both default to unbounded except
monocular's derived ceiling, so by default a mask reading is bounded
only by what its own source can resolve.

projective ranging, euclidean reconstruction and the mask overlay never branch
on the source; they receive a frame that already satisfies §1.
```

The mask node does branch, but on **acquisition**, never on depth semantics:
each source declares an `input_kind` (`depth` | `color`), which decides only
which stream the node buffers raw and hands back at the detection stamp.

- Selecting the source is a **config choice**, mirroring the other swap points
  (backend sim/real, mask front-end detect/segment, isolation recipe).
- The benchmark treats the source as a matrix axis: every downstream
  combination (path × mask tag × isolation recipe) runs once per source, so
  stereo-vs-monocular is compared under identical downstream code.
- The deleted legacy stack already did this implicitly:
  `add_depth_source_measurements` in `geometry.py` ran the *same* function over
  the sensor depth image and the Depth-Anything map (the `sensor_depth` and
  `depth_anything` rows). This contract formalized what that function assumed;
  it now stands on its own, both of those rows having been removed.

---

## 5. Open items

- **`align_depth` on hardware** — configuration and parser preservation are
  complete (`align_depth.enable: true`, `enable_sync: true`). Still confirm on
  the robot: effective stream profiles, aligned-depth topic name and
  dimensions, matching color/depth header timing, optical-frame TF to base,
  and organized-pointcloud availability/layout before selecting its estimator.
- ~~**Widen the depth gate**~~ — done 2026-08-28. The mask rows' gate
  (`mask_depth_max_meters`) defaults to `0`, meaning no gate, so the only
  ceiling on a mask measurement is whatever its source declares: monocular's
  derived 18 m, and nothing at all for stereo. Landed together with the
  isolation default moving to `height_crop_nearest_mode_band`, which was the
  ordering constraint — widening first would have been the regression. The
  legacy camera rows kept a finite `depth_max_meters` of 10 m, having no source
  ceiling behind them; those rows and that parameter are both gone.
  **Unmeasured.** No benchmark has run with these defaults; the A/B is now a
  regression check rather than a gate, and the benchmark cannot see the change
  it is checking for, since its scenes top out at 5.8 m in a world whose far
  wall is at 12 m. Exploration (`mock_hospital`, ~22 m) is where it bites.
- ~~**Intrinsics source** — switch deprojection to `camera_info` of the color
  camera instead of the FoV constants in `camera_config.json` (§2.3).~~ —
  resolved 2026-07-21: the mask node subscribes to the color camera's
  `camera_info` and deprojects with it. The legacy estimators that fell back to
  the `camera_config.json` FoV constants have since been deleted, so nothing
  reads those constants any more.
- ~~**Metric-scale validation**~~ — resolved 2026-07-24: measured Depth-Anything's
  scale directly as a pixel-wise `mono / stereo` depth ratio on identical sim
  frames (isolating the scale term from surface warping and noise). On the **G1
  region** — the pixels euclidean reconstruction actually consumes — the ratio is
  ≈**1.0** (overall median 1.02) but **range-dependent**: it over-reads ~10–15% at
  2–3 m (ratio 1.10–1.15) and converges to metric (~1.0) by 4–6 m. On the **whole
  frame** (dominated by floor and walls) it reads ~13% short (median 0.87), a
  separate warping of large flat surfaces. So Depth-Anything is *approximately
  metric on the object* with mild close-range warping — **not** a constant global
  bias correctable by a single factor. Consequence: euclidean's absolute-metric
  isolation (`HeightCrop` floor plane, `RangeBand` ±0.10/0.35 m windows) is
  genuinely soft at close range on the monocular source; treat monocular euclidean
  rows as range-warped, not scale-shiftable. (Consistent with the end-to-end
  monocular MAE ~0.19–0.22 m vs. stereo ~0.07 m — the excess is warping, not a
  fixable offset.)
- ~~**Topic-level contract**~~ — superseded 2026-08-26. It was resolved
  2026-07-12 as a pair of topics (`perception/aligned_depth/image` +
  `.../camera_info`) published by `aligned_depth_node`; that transport is gone.
  The contract is now a **call** at the detection stamp
  (`depth_sources.produce`), and the frame is republished only as the debug
  artifact `debug/g1/mask/aligned_depth` for the overlay panel. The artifact
  itself (§1) is unchanged.
- ~~**Stamp-matching coverage in sim**~~ — resolved 2026-08-26. The exact-stamp
  match between detections and aligned depth (the §1 "Timing" invariant) missed
  on ~80–90% of frames in sim because a separate producer process thinned the
  depth stream independently of the detector, under CPU starvation. Depth
  acquisition moved into `g1_mask_measurement_node`, which now buffers the
  source's input raw and converts at the detection stamp; the second thinning
  stage no longer exists. Diagnosis, profiling and the decision are in
  `aligned_depth_coverage.md`.
