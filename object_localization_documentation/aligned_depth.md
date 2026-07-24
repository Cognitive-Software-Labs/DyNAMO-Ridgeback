# The Aligned Depth Frame — Depth Acquisition

**Scope:** how the depth that the localization paths consume is produced. Both
projective ranging (`projective_ranging.md`) and euclidean reconstruction (`euclidean_reconstruction.md`) consume one and the
same artifact — the **aligned depth frame** — and neither cares how it was
made. This document defines that contract and the two producers that satisfy
it: the physical depth camera (`camera → aligned depth`) and monocular
estimation (`RGB frame → aligned depth`).

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
  producer, not the consumer.)
- **Invalid pixels:** `0`, `NaN`, or `inf` mean "no depth here"; consumers
  filter them (`projective_ranging.md` §2.2) and producers must not encode invalid
  as any other value.
- **Timing:** stamped with the color frame it is aligned to, so mask and depth
  can be matched frame-to-frame.

Everything downstream — select, clean/isolate, aggregate, deproject — is
identical regardless of which producer made the frame. The depth source is a
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
`aligned_depth_to_color` image topic). Consequences to design around:

- **Occlusion holes.** Alignment is a reprojection from a different viewpoint;
  surfaces visible to the depth sensor but hidden from the color camera leave
  invalid (0) pixels, concentrated at object edges — exactly where masks are.
  This is why the clean step is not optional.
- **Resampling.** Depth values are interpolated/selected onto a new grid;
  fine structure can shift by a pixel.
- **Cost.** The align filter runs in the driver. It is the price of admission
  for every mask-based consumer, so for cost accounting it is a *sunk* cost —
  see `pointcloud_provenance_test.md` §2.
- **Config gap.** `align_depth.enable` is not set in `clearpath/robot.yaml`
  today; enabling it (and verifying the key passes through the Clearpath
  generation layer) is a prerequisite for running either path on hardware.

### 2.2 Simulation (gz `rgbd_camera`)

The sim camera is a GPU render: depth comes from the Z-buffer of the *same
virtual camera* that renders RGB. Depth and color are **co-registered by
construction** — same origin, same FoV, same grid — so the raw sim depth
topic already satisfies the contract with no align step and no occlusion
holes. Published as `32FC1` meters on `sensors/camera_0/depth/image`.

This convenience is also a trap: code that works on raw sim depth will break
on raw real depth. The backend abstraction must route *real* through the align
filter and *sim* straight through, while consumers see one topic-level
contract. Divergences to keep in mind (details in
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
The **legacy estimators** still derive intrinsics from
`config/camera_config.json` FoV values (87°/58° — the real depth FoV), which is
wrong for the sim render (71.6°) and wrong-in-principle for aligned real depth
(color FoV). The mask stack no longer has this problem: `aligned_depth_node`
already subscribes and republishes `camera_info` of the grid's camera and the
mask node deprojects with it (resolved 2026-07-21). The provenance test
self-calibrates as a cross-check (`pointcloud_provenance_test.md` §3).

---

## 3. Source 2: RGB frame → aligned depth (Depth-Anything)

The monocular source needs no depth sensor at all: the RGB frame goes through
**Depth-Anything V2** (Metric Indoor Small checkpoint, via HuggingFace
`transformers`), which predicts a dense metric depth map. This document's
producer is `aligned_depth_node.MonocularDepthSource` (selected by the node's
`depth_source` param); the legacy `g1_camera_measurement_node` runs the same
network for its own estimators.

- **Aligned by construction.** The network's input *is* the color image, so
  its output is per-color-pixel — the grid property costs nothing. Resized to
  `H×W` if the network ran at a different internal resolution, it satisfies
  the contract directly.
- **Metric scaling.** The checkpoint predicts metric depth for indoor scenes;
  any residual global scale/bias correction (e.g. against a known reference)
  is part of this producer, not the consumers.
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

Two producers, one contract, and every consumer is source-blind:

```
stereo producer     = capture -> (real: align) -> float meters -> frame
monocular producer  = RGB -> Depth-Anything -> metric scale    -> frame

consumers (projective ranging, euclidean reconstruction, mask overlay) never branch on the source.
```

- Selecting the source is a **config choice**, mirroring the other swap points
  (backend sim/real, mask front-end detect/segment, isolation recipe).
- The benchmark treats the source as a matrix axis: every downstream
  combination (path × mask tag × isolation recipe) runs once per source, so
  stereo-vs-monocular is compared under identical downstream code.
- The current estimators already do this implicitly:
  `add_depth_source_measurements` in `geometry.py` runs the *same* function
  over the sensor depth image and the Depth-Anything map (`sensor_depth` and
  `depth_anything` rows). The contract here formalizes what that function
  already assumes.

---

## 5. Open items

- **`align_depth` on hardware** — add the parameter to `robot.yaml`, verify it
  survives the Clearpath generation layer, and confirm the
  `aligned_depth_to_color` topic grid matches the color image.
- ~~**Intrinsics source** — switch deprojection to `camera_info` of the color
  camera instead of the FoV constants in `camera_config.json` (§2.3).~~ —
  resolved 2026-07-21: for the mask stack, `aligned_depth_node` republishes the
  color camera's `camera_info` alongside each frame and the mask node deprojects
  with it; only the legacy estimators still fall back to the
  `camera_config.json` FoV constants.
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
- ~~**Topic-level contract**~~ — resolved 2026-07-12: both producers publish
  `perception/aligned_depth/image` + `perception/aligned_depth/camera_info`
  (`aligned_depth_node`, switched by its `depth_source` param); consumers
  (`g1_mask_measurement_node`) subscribe those topics and never branch on the
  source.
