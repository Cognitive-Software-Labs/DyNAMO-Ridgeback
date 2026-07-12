# The Aligned Depth Frame — Depth Acquisition

**Scope:** how the depth that the localization paths consume is produced. Both
Path A (`depth_based_A.md`) and Path B (`depth_based_B.md`) consume one and the
same artifact — the **aligned depth frame** — and neither cares how it was
made. This document defines that contract and the two producers that satisfy
it: the physical depth camera (`camera → aligned depth`) and monocular
estimation (`RGB frame → aligned depth`).

```
                 ┌───────────────────────────┐
   D435 depth ──▶│ stereo source:            │
   (sensor)      │ capture → align to color  │──┐
                 └───────────────────────────┘  │   ┌─────────────────────┐
                                                ├──▶│ ALIGNED DEPTH FRAME │──▶ Path A / Path B
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
  filter them (`depth_based_A.md` §2.2) and producers must not encode invalid
  as any other value.
- **Timing:** stamped with the color frame it is aligned to, so mask and depth
  can be matched frame-to-frame.

Everything downstream — select, clean/isolate, aggregate, deproject — is
identical regardless of which producer made the frame. The depth source is a
**strategy**: a config choice behind this one contract, and an axis of the
benchmark matrix (`Object_Localization_Pipeline.md` §8).

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
`Object_Localization_Pipeline.md` §2):

| | Sim | Real |
|---|---|---|
| Depth origin | color camera's Z-buffer | left-IR stereo, reprojected |
| Alignment | by construction | driver filter, must be enabled |
| FoV | 71.6° (`horizontal_fov` 1.25 rad) | 87°×58° depth / 69°×42° color |
| Resolution | 640×480 @ 30 (profile keys in `robot.yaml` are stale — see pipeline doc) | per `depth_module.profile` |
| Encoding | `32FC1` m | `16UC1` mm |
| Noise / holes | none (clean render) | speckle, dropouts, alignment holes |

### 2.3 Intrinsics caveat

Deprojection (Path A §2.4, Path B §2.1) needs the intrinsics *of the grid the
frame lives on* — after alignment that is the **color** camera's intrinsics.
The repo currently derives intrinsics from `config/camera_config.json` FoV
values (87°/58° — the real depth FoV), which is wrong for the sim render
(71.6°) and wrong-in-principle for aligned real depth (color FoV). The robust
fix is subscribing `camera_info` of the grid's camera; the provenance test
self-calibrates as a cross-check (`pointcloud_provenance_test.md` §3).

---

## 3. Source 2: RGB frame → aligned depth (Depth-Anything)

The monocular source needs no depth sensor at all: the RGB frame goes through
**Depth-Anything V2** (Metric Indoor Small checkpoint, via HuggingFace
`transformers`; already integrated in `g1_camera_measurement_node`), which
predicts a dense metric depth map.

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
  accordingly (current sim run: `sensor_depth` MAE 0.586 m vs.
  `depth_anything` MAE 0.931 m).
- **No published cloud.** The monocular source produces only a depth image.
  Path B reaches it exclusively through in-code deprojection — the reason the
  deprojected provenance is Path B's canonical input
  (`pointcloud_provenance_test.md`).

---

## 4. The strategy pattern

Two producers, one contract, and every consumer is source-blind:

```
stereo producer     = capture -> (real: align) -> float meters -> frame
monocular producer  = RGB -> Depth-Anything -> metric scale    -> frame

consumers (Path A, Path B, mask overlay) never branch on the source.
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
- **Intrinsics source** — switch deprojection to `camera_info` of the color
  camera instead of the FoV constants in `camera_config.json` (§2.3).
- **Metric-scale validation** — quantify Depth-Anything's global scale error
  against stereo on identical frames before trusting its Path B rows.
- **Topic-level contract** — pin the topic names/remappings under which both
  producers publish (or are consumed as) the aligned depth frame, so the
  source swap is pure config with no code branch.
