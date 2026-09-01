# Mask representation

**Scope:** the mask representation, geometric constructors, storage boundaries,
and visualization semantics shared by target-localization paths. Learned mask
production has its own [segmentation contract](segmentation.md). It deliberately does **not** describe how
the paths (depth-image, point-cloud, LiDAR) consume the mask — that integration
is documented separately.

---

## 1. Purpose

The object-localization pipeline supports more than one detector front-end, and
each front-end produces a different kind of region. To keep everything
downstream agnostic to which model ran, all front-ends emit into a **single
common representation**: a binary mask plus a precision tag. Downstream code
reads only this representation and never branches on which detector produced it.

A mask is fundamentally a **selector**: it answers, for every pixel of the RGB
color image, "does this pixel belong to the detected object?" Pixels marked
`True` are the object's region; everything else is `False`.

That question is about the whole color grid; how the answer is *stored* is a
separate matter, covered in Section 7. Production stores it as a `MaskRegion` —
a boolean payload over a rectangular window of the grid, with everything outside
the window implicitly `False`. Sections 2–6 describe the selector and hold for
either storage form.

---

## 2. Two mask types

There are **two** types of mask, both emitted into the same interface:

| Type   | Tag     | Produced by                              | Shape of the `True` region |
|--------|---------|------------------------------------------|----------------------------|
| Tight  | `tight` | instance segmentation (pixel-precise)    | an arbitrary blob          |
| Rect   | `rect`  | object detection → rasterized bounding box | a solid rectangle        |

Both are the same *type* — a boolean selector plus a tag — so downstream
selection is written once and works for either. They differ only in how faithfully the `True`
region follows the object's true silhouette, which is recorded by the
**precision tag** (`tight` | `rect`).

The tight mask is described only briefly here (Section 4); its producer has
its own document, `docs/target_localization/segmentation.md`. The remainder of this document
describes the **rectangular mask** thoroughly, because it is the default mask
produced from the detector (OWLv2).

---

## 3. The rectangular mask

### 3.1 Source

The rectangular mask starts from the accepted-box contract owned by
[detection](detection.md). Each surviving detection supplies one box in the
form `bbox_xyxy`:

```
bbox_xyxy = (x1, y1, x2, y2)
```

where `(x1, y1)` is inclusive and `(x2, y2)` is exclusive in **RGB colour-image
pixel coordinates**. Detection guarantees a clamped, non-degenerate box. Mask
representation neither knows nor depends on which model produced it.

### 3.2 Rasterization to a binary mask

Converting the box to the mask representation selects every pixel inside the
box and nothing outside it. Because the selected set *is* the box, production
stores the box as its own window and the payload needs no `False` at all:

```python
x1, y1, x2, y2 = clamp_box(bbox_xyxy, H, W)   # H, W = color image height, width
region = MaskRegion(
    data=np.ones((y2 - y1, x2 - x1), dtype=bool),
    origin_u=x1, origin_v=y1,
    image_width=W, image_height=H,
    precision=MaskPrecision.RECT,
)
```

The full-grid form of the same selector is what `rasterize_bbox` returns, for
whole-frame artifacts:

```python
mask = np.zeros((H, W), dtype=bool)
mask[y1:y2, x1:x2] = True             # solid rectangle of True
```

Both go through the same `clamp_box`, so they cover identical pixels. There is
no interpolation, no thresholding, and no model inference in either — it is a
pure, deterministic geometric fill.

### 3.3 Pixel grid

The mask is defined in exactly one coordinate system: the **RGB color image
pixel grid**. Concretely, that grid is fixed by three things:

- **Resolution** — the color grid of the active backend (the current Clearpath
  default is `640 × 480`). Every mask records this grid, whether or not it
  allocates the whole of it: a `Mask` is `(H, W)`, a `MaskRegion` carries
  `image_shape == (H, W)` alongside its smaller payload.
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

- a boolean selector over the color-image grid (stored as the box's own
  window — Section 7),
- with a solid rectangle of `True` pixels matching the detector's box,
- tagged `rect`,
- bound to the color frame's grid (resolution, intrinsics, timestamp),
- and carrying a known background contamination that the tag advertises.

The boolean array is an **in-process representation, not a wire format**: the
whole perception stack (mask component + projective ranging, euclidean reconstruction, and polar profiling) runs on one machine, so
consumers receive the mask by in-process handoff, never over the network. If a
mask is ever published as a topic, that topic is debug/visualization only and
uses an encoded form — see the wire-cost note in Section 5.3.

---

## 4. Tight mask

The tight mask is the second front-end and is **not** detailed here — see
`docs/target_localization/segmentation.md`. In short: a box-promptable segmentation model
(SlimSAM by default), prompted with the detector's boxes, produces a
pixel-precise `True` region (an arbitrary blob, not a rectangle), tagged
`tight`, with negligible background contamination. It emits into the identical
mask interface via `region_from_blob(blob, MaskPrecision.TIGHT)`, so nothing
that consumes a mask changed when it was added. The front-end is selected per
run by the `mask_gate` parameter (`box` | `silhouette`).

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

The mask views form a **second row below** the camera panels — one panel per
front-end, so both mask types are visible at once and directly comparable:

```
[ RGB Detection | Sensor Depth | Depth-Anything ]
[ Box Mask      | Silhouette Mask |             ]
```

All panels share the active backend's color-image resolution, so the mask
panels are the same size as the others (the shorter mask row is black-padded
to the grid width).

### 5.2 Panel style: masked RGB

Both mask panels render as **masked RGB**: pixels **inside** the mask show the
real RGB content; everything **outside** the mask is black.

```
panel = zeros (all black)
panel[mask] = rgb[mask]      # copy RGB only where mask is True
```

For the rectangular mask this is exactly the RGB rectangle of the detection box
sitting on a black background. The chosen style is deliberate: showing the real
RGB pixels inside the region (rather than a flat white blob) makes the
contamination directly visible — floor, wall, and neighbouring objects caught
inside the box appear right next to the target. The panel therefore doubles as a
visual measure of "how much background is this box dragging in."

The silhouette panel renders the same style off the tight mask: black outside,
RGB inside the true object outline. Side by side, the two panels make the
precision difference immediately obvious — a clean cut-out (`tight`) versus a
rectangle full of background (`rect`).

### 5.3 Data source (per panel)

The two panels have different sources, because only one of the two masks is
reconstructible at the consumer:

- **Box Mask panel (derive at render time):** the rect mask is fully
  determined by the detection boxes already on the wire, so the panel
  *recomputes* it at draw time (union of boxes, rasterized). Always shown.
  Note the union is **display-only**: localization always uses one mask per
  detection (Section 6.1) and never consumes the union — merging masks would
  destroy per-object coordinates.
- **Silhouette Mask panel (display the real artifact):** a tight mask is *not*
  reconstructible downstream, so the mask node publishes the union of the
  frame's consumed masks as a debug-only `mono8` Image on `debug/target/mask`
  (`docs/target_localization/segmentation.md` §6) — silhouette gate only. The panel never
  shows substitute content: it renders the artifact whose stamp matches the
  rendered frame exactly (what downstream received), holding the most recent
  artifact when the current frame's mask has not landed yet (it lags the
  measurements by the segmentation latency). Only a run that has produced no
  artifact at all (box gate, startup) shows the "No silhouette mask"
  placeholder.

**Wire cost of the artifact.** A raw published mask (1 byte per pixel) scales
with the active backend's color grid. At the current `640 × 480` default it is
`≈ 0.3 MB` per mask; any future higher-resolution profile scales directly with
pixel count. The convention that keeps it harmless:

- **Measurement consumers never take the mask off the wire.** Projective
  ranging, euclidean reconstruction, and polar profiling receive it by
  in-process handoff, so this debug publication is not part of measurement dataflow.
- **The published topic is debug/visualization only** and uses a normal
  `sensor_msgs/Image` publisher with `mono8` encoding. Compressed image transport
  is not part of this component's contract; do not assume PNG size or CPU cost.

### 5.4 Cost

Panel composition performs boolean fills/copies and image rendering per output
frame; no model inference occurs in the panel code itself. Its cost depends on
grid size, detection count, enabled panels, and subscriber-driven artifacts.
The composite is published for the RViz Image display.

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

Two G1s in view means `count = 2`, two masks, two coordinates. Everything
downstream starts from the post-filtering boxes published by
[detection](detection.md); raw model proposals are outside the mask contract.

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
- **Oversized-box exception to the 1:1 mapping.** A detection whose box covers
  more than `MAX_BOX_FRAME_FRACTION` (0.60) of the frame — a likely detector
  failure that would otherwise mask the whole scene — is gated to a `None` mask
  *before* rasterization or segmentation, so it produces no mask and no result.
  The mapping is therefore "one *accepted* detection → one mask"; an oversized
  detection is skipped rather than turned into a full-frame mask.

Two properties follow, and both matter downstream:

- **Masks are independent and never merged.** Each mask selects one object's
  pixels and yields one result for *that* object. Two different objects always
  get two separate coordinates; there is no "distance for the whole batch."
  Masks may even partially overlap. Detection owns duplicate suppression and
  retains legitimate surviving overlap; each published box is still selected
  independently here.
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

## 7. Storage: a local window, global meaning

Sections 1–6 describe what a mask *means*: a selector over the color grid. This
section describes how one is *stored*, which is a different thing and has been
since the ROI migration.

### 7.1 Two representations

| Type | Indexing | Where it is used |
|------|----------|------------------|
| `MaskRegion` | `data[local_row, local_col]`, offset by `(origin_u, origin_v)` | every per-detection measurement stage — the canonical production form |
| `Mask` | `data[v, u]` over the whole image | whole-frame artifacts, masks arriving off the wire, the standalone helper signatures |

A `MaskRegion` carries a boolean payload sized to a **rectangular storage
window** of the color grid, plus the window's origin, the full grid's
dimensions, and the same `MaskPrecision` tag. Outside the window, membership is
implicitly `False`. A region and the `Mask` it materializes to therefore select
exactly the same pixels — Sections 1–6 hold unchanged for both.

Both forms carry the precision tag explicitly. Precision is never inferred from
the shape of the storage: a `tight` silhouette that happens to fill its window
completely is still `tight`, and still gets the tight foreground policy.

### 7.2 What each producer stores

- **`rect`** (`region_from_bbox`): the clamped detector box *is* the window, so
  the payload is all-`True` and carries no `False` at all. `clamp_box` is shared
  with the full-grid rasterizers, so both forms cover identical pixels.
- **`tight`** (`region_from_blob`): the segmenter returns a blob on the full
  color grid — that is the model boundary and it does not move. The producer
  crops it to the blob's **own nonzero extent**, not to the detector box, and
  copies. Cropping to the box would delete real silhouette pixels: the prompt is
  padded (`PROMPT_PADDING_REL_DEFAULT`), so the returned silhouette can extend
  past the box that prompted it. The copy is what lets the frame-sized model
  output expire immediately; the region does not keep it alive as a numpy base.

A degenerate or fully clamped-away box yields the **canonical empty region** — a
`(0, 0)` payload that still names its grid and precision. That is a valid
selector with zero pixels, and it is *not* `None`. The distinction from Section
6.1's oversized-box `None` is load-bearing: `None` means "no mask for this
detection, skip it", while an empty region is measured normally and fails
through the ordinary minimum-count guards with the ordinary miss reasons.

### 7.3 Local storage, global coordinates

Everything a mask hands to a consumer that is defined against the camera is in
**full-grid** coordinates, because the intrinsics describe the color grid and
nothing else. The rule downstream is therefore:

- select on window-local arrays (depth window, mask payload, sliced validity),
- convert to global with `MaskRegion.global_pixels` exactly once,
- deproject against the original frame and the original intrinsics.

There are no ROI-adjusted intrinsics, no resampling, and no per-region cache.
Membership queries (`contains_pixels`) test the window bounds *before* indexing,
so a coordinate outside the window reads as "not selected" instead of wrapping
around to a pixel on the far edge of the payload.

### 7.4 Why the window rather than the frame

Storage was previously one full-grid array per detection regardless of how small
the detection was. For a production-sized box (~2% of frame) the region is
roughly 50× smaller, at either 640×480 or 1280×720. The measured consequences,
and the two cases that got slower, are recorded in `docs/history/roi_mask_migration.md`.

## 8. Contract checks

`Mask` and `MaskRegion` validate boolean payloads and dimensions; the region also
validates bounds and retains its explicit `MaskPrecision`. `None` is not an
empty selector. Constructors, origin-aware membership, copy/ownership rules,
and whole-grid compatibility are covered by
[region tests](../../src/ridgeback_autonomy/test/test_mask_region.py).
Migration evidence and limitations live in [history](../history/roi_mask_migration.md).
