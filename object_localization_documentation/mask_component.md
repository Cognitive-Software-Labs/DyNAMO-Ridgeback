# Mask Component

**Scope:** the front-end stage that turns a detector output into the common
mask representation consumed by every downstream path. This document covers the
mask *object* and how it is produced. It deliberately does **not** describe how
the paths (depth-image, point-cloud, LiDAR) consume the mask — that integration
is documented separately.

---

## 1. Purpose

The object-localization pipeline supports more than one detector front-end, and
each front-end produces a different kind of region. To keep everything
downstream agnostic to which model ran, all front-ends emit into a **single
common representation**: a binary mask plus a precision tag. Downstream code
reads only this representation and never branches on which detector produced it.

A mask is fundamentally a **selector**: an `H×W` boolean array that answers, for
every pixel of the RGB color image, "does this pixel belong to the detected
object?" Pixels marked `True` are the object's region; everything else is `False`.

---

## 2. Two mask types

There are **two** types of mask, both emitted into the same interface:

| Type   | Tag     | Produced by                              | Shape of the `True` region |
|--------|---------|------------------------------------------|----------------------------|
| Tight  | `tight` | instance segmentation (pixel-precise)    | an arbitrary blob          |
| Rect   | `rect`  | object detection → rasterized bounding box | a solid rectangle        |

Both are the same *type* — an `H×W` binary mask — so downstream selection is
written once and works for either. They differ only in how faithfully the `True`
region follows the object's true silhouette, which is recorded by the
**precision tag** (`tight` | `rect`).

The tight mask is described only briefly here (Section 4); it is a future
front-end. The remainder of this document describes the **rectangular mask**
thoroughly, because it is the mask produced from the detector that already
exists in the stack (OWLv2).

---

## 3. The rectangular mask

### 3.1 Source

The rectangular mask is built from the object detector's output. The current
detector is OWLv2 (`google/owlv2-base-patch16-ensemble`), an open-vocabulary
detector that emits, per detection, an axis-aligned bounding box together with a
label and a confidence score. After confidence thresholding and non-maximum
suppression, each surviving detection carries a box in the form `bbox_xyxy`:

```
bbox_xyxy = (x1, y1, x2, y2)
```

where `(x1, y1)` is the top-left corner and `(x2, y2)` is the bottom-right
corner, both in **pixel coordinates of the RGB color image**. The box is already
clamped to the image bounds, with `x2 > x1` and `y2 > y1` guaranteed (a minimum
extent of one pixel), so it is always a valid, non-degenerate rectangle.

The detector model itself is an implementation detail. Nothing in the mask
component depends on which detector produced the box — only on the box.

### 3.2 Rasterization to a binary mask

Converting the box to the mask representation is a single fill operation: every
pixel inside the box becomes `True`, every pixel outside becomes `False`.

```python
mask = np.zeros((H, W), dtype=bool)   # H, W = RGB color image height, width
mask[y1:y2, x1:x2] = True             # solid rectangle of True
```

The result is an `H×W` boolean array at the **resolution of the RGB color
image**. There is no interpolation, no thresholding, and no model inference in
this step — it is a pure, deterministic geometric fill.

### 3.3 Pixel grid

The mask is defined in exactly one coordinate system: the **RGB color image
pixel grid**. Concretely, that grid is fixed by three things:

- **Resolution** — the color image size (currently `1280 × 720`). The mask is
  allocated at this size, so `mask.shape == (H, W)` of the color frame.
- **Intrinsics** — the color camera's projection parameters. They are not used
  to build the mask, but they define what each pixel index *means* as a ray into
  the scene, which matters the moment the mask is used as a selector.
- **Timestamp** — the mask inherits the timestamp of the color frame it was
  detected on. It describes the object's region *at that instant*.

A mask is only meaningful inside its own grid. This is a property of the mask
object and is recorded here so that any consumer treats the mask as bound to a
specific color frame (its resolution, intrinsics, and time), not as a
free-floating region.

### 3.4 Precision tag and background contamination

The rectangular mask is tagged `rect`. This tag is not cosmetic: it flags that
the `True` region is a **lossy over-approximation** of the object.

A bounding box is axis-aligned and solid, so unless the object happens to fill
its box exactly, the rectangle also covers pixels that are *not* the object —
floor, wall, or whatever lies behind and beside the target. The rasterized mask
therefore carries a known **background contamination**: some `True` pixels do
not belong to the object.

This is a deliberate, lossy adapter. The rasterization discards the object's
true shape in order to conform to the common mask interface. The contamination
is an intrinsic property of the `rect` mask and must be marked clearly at the
code boundary so a rectangular mask is never mistaken for a pixel-precise one.
The `rect` tag is exactly that marker.

### 3.5 Summary of properties

A rectangular mask is:

- an `H×W` boolean array at color-image resolution,
- with a solid rectangle of `True` pixels matching the detector's box,
- tagged `rect`,
- bound to the color frame's grid (resolution, intrinsics, timestamp),
- and carrying a known background contamination that the tag advertises.

The boolean array is an **in-process representation, not a wire format**: the
whole perception stack (mask component + Paths A/B/C) runs on one machine, so
consumers receive the mask by in-process handoff, never over the network. If a
mask is ever published as a topic, that topic is debug/visualization only and
uses an encoded form — see the wire-cost note in Section 5.3.

---

## 4. Tight mask (deferred)

The tight mask is the second front-end and is **not** detailed here. In short,
it will come from an instance-segmentation model and produce a pixel-precise
`True` region (an arbitrary blob, not a rectangle), tagged `tight`, with
negligible background contamination. It emits into the identical mask interface,
so adding it requires no change to anything that consumes a mask. A full
description belongs in its own document when that front-end is built.

---

## 5. Visualization

It is useful to **see** the mask alongside the other camera views while the
pipeline runs, both to sanity-check detection and to make the `rect` mask's
background contamination (Section 3.4) visually obvious.

### 5.1 Placement

The live perception overlay already shows a single horizontal row of
color-image-sized panels:

```
[ RGB Detection | Sensor Depth | Depth-Anything ]
```

The mask view is added as one more panel appended to the right of this row:

```
[ RGB Detection | Sensor Depth | Depth-Anything | Mask ]
```

All panels share the color image resolution, so the mask panel is the same
`1280 × 720` size as the others.

### 5.2 Panel style: masked RGB

The mask panel renders as **masked RGB**: pixels **inside** the mask show the
real RGB content; everything **outside** the mask is black.

```
panel = zeros (all black)
panel[mask] = rgb[mask]      # copy RGB only where mask is True
```

For a rectangular mask this is exactly the RGB rectangle of the detection box
sitting on a black background. The chosen style is deliberate: showing the real
RGB pixels inside the region (rather than a flat white blob) makes the
contamination directly visible — floor, wall, and neighbouring objects caught
inside the box appear right next to the target. The panel therefore doubles as a
visual measure of "how much background is this box dragging in."

When the tight front-end exists, the **same** panel renders its silhouette for
free: black outside, RGB inside the true object outline. Side by side, the two
mask types make the precision difference immediately obvious — a clean cut-out
(`tight`) versus a rectangle full of background (`rect`).

### 5.3 Data source (current vs. target)

There is an important distinction in **where the displayed mask comes from**:

- **Current (derive at render time):** the binary mask is not yet produced or
  published anywhere in the pipeline — only the bounding box is. So the panel
  would *recompute* the rectangular mask from the detection box(es) at draw
  time (union of boxes, rasterized). This is cheap and needs no new topic or
  node, but it means the overlay is displaying a *locally reconstructed* mask,
  not the actual artifact any path consumes. Note the union is **display-only**:
  localization always uses one mask per detection (Section 6.1) and never
  consumes the union — merging masks would destroy per-object coordinates.
- **Target (display the real artifact):** once the mask component publishes a
  real mask (the `H×W` boolean array plus its `tight | rect` tag), the panel
  should consume that instead. Then the view shows exactly what downstream
  receives, and it renders `tight` and `rect` masks identically with no
  panel-side changes.

**Wire cost of the target.** A naive published mask (1 byte per pixel) is
`1280 × 720 ≈ 0.9 MB` — at 30 fps that is ~28 MB/s for a single mask, times the
detection count. The convention that keeps this harmless:

- **Consumers never take the mask off the wire.** The whole perception stack
  runs on one machine (robot PC or workstation — never split across both), so
  Paths A/B/C receive the boolean array by in-process handoff at zero wire
  cost.
- **The published topic is debug/visualization only**, and goes out encoded:
  `mono8` Image with compressed transport (PNG). Binary masks compress to a few
  kB — two to three orders of magnitude under the naive figure — at negligible
  CPU cost.

### 5.4 Cost

Negligible: one boolean fill plus one masked copy per rendered frame. No model
inference, and — if derived at render time — no new topic or node. The only
practical caveat is window width: a fourth panel makes the row `4 × 1280 =
5120 px` wide, which the overlay window scales down to fit the screen.

---

## 6. Batch semantics — one or more masks per frame

The mask interface is **batch-native**: a single RGB frame yields *zero or more*
detections, and each detection produces exactly **one** mask. So per frame the
interface carries a *list* of masks, not a single mask.

- `count = 0` — nothing detected, no masks.
- `count = 1` — one mask (the common single-target case, e.g. the benchmark).
- `count = N` — N independent masks carried together.

### 6.1 The hierarchy: one object → one detection → one mask

The relationship is strictly **1:1:1** per frame:

```
frame
 └─ count detections        (post-NMS: one box per visible object)
     └─ 1 mask each          (rasterized from that one detection's box)
         └─ 1 result each    (one coordinate per mask, per path)
```

Two G1s in view means `count = 2`, two masks, two coordinates. There is **no**
"N detections per object" layer: the raw detector head does propose many
candidate boxes per object, but confidence thresholding and non-maximum
suppression collapse them *inside the detector front-end*, before anything is
published. Everything downstream of the detector — the messages, the masks,
the paths — sees only post-NMS detections.

Consequences worth pinning:

- A mask is "the pixel set of one detection's box," **not** "all pixels
  belonging to one object across several detections." No consolidation step
  exists or is needed.
- If NMS ever under-suppresses (two surviving boxes on the same G1), the
  pipeline honestly reports two objects with two coordinates. De-duplicating
  that is an association problem for the fusion stage, not for the mask
  component or the paths.
- The hierarchy is **per frame** — nothing persists across frames. The same G1
  in consecutive frames yields a fresh detection, mask, and coordinate each
  time; temporal association (tracking) is a separate, later concern.

Two properties follow, and both matter downstream:

- **Masks are independent and never merged.** Each mask selects one object's
  pixels and yields one result for *that* object. Two different objects always
  get two separate coordinates; there is no "distance for the whole batch."
  Masks may even partially overlap (the detector's non-maximum suppression
  allows boxes up to 0.5 IoU), and each is still selected on its own.
- **The source frame is shared; only selection is per-mask.** The RGB frame —
  and anything derived from it once per frame (e.g. an aligned depth frame) — is
  produced a single time and then indexed once per mask. The expensive
  per-*frame* work is not repeated per object; the per-*mask* work is just the
  cheap selection. This keeps N masks roughly as cheap as one plus N light
  selections.

This batch shape is already fixed by the pipeline messages, which carry it as
**parallel, index-aligned arrays**: a `count`, a flat `bbox_xyxy` of `4 · count`
values, and one length-`count` array per measurement field. A consumer unpacks
by detection index `i` to recover each object's mask region and its result.

---

## 7. Open items

- **Tight-mask front-end** — choose and document the segmentation model and its
  output handling.
- **Interface signature** — pin the in-code contract for the mask object (the
  `H×W` boolean array plus the `tight | rect` tag) so the separation between
  front-ends and consumers is enforced, not just described.
- **Visualization data source** — decide between deriving the displayed mask
  from the box at render time (quick, throwaway) and consuming a real published
  mask (matches the interface, reusable by other consumers such as the benchmark
  collage). See Section 5.3.
