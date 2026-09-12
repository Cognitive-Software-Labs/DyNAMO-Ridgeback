# Segmentation

**Scope:** the *tight* (silhouette) mask front-end — the instance-segmentation
producer that `docs/target_localization/mask_representation.md` §4 defers to. This document covers the model,
how it is prompted, and the semantics of its output. How the resulting mask is
consumed is unchanged and documented with the paths; the mask *object* is
documented in `docs/target_localization/mask_representation.md`.

**Code:** `perception/target_localization/core/segmentation.py` (`SamBoxSegmenter`), wired into
`target_mask_measurement_node` behind the `mask_gate` parameter
(`box` | `silhouette`).

## 1. Current model and boundary

`SamBoxSegmenter` supports SAM and SAM2 checkpoints, dispatching from the
checkpoint's `model_type`. The default is `Zigeng/SlimSAM-uniform-50`;
`segmentation_model` selects the checkpoint. The default was chosen for leg-gap
fidelity and resource use in a recorded simulation comparison, not on verified
D455 imagery. See [experimental evidence](../history/segmentation_experiments.md).

The current stack is [OWLv2 detection](detection.md) followed, in silhouette mode, by a
box-prompted segmenter. The segmenter is class-blind: class information comes
from the detector's box, and a clean wrong-object mask can score highly.
Consumers depend on the mask contract, not the checkpoint. Alternative
single-stage producers remain [candidates](../do_not_try_again/segmentation.md).

## 2. Prompting

- **Prompt = padded detection box.** OWLv2 boxes can clip the object, and a
  SAM box prompt truncates hard at its boundary, so the prompt box is inflated
  by a small relative padding (`PROMPT_PADDING_REL_DEFAULT = 0.05`, i.e. 5% of
  each extent, clamped to the grid) before prompting. Padding widens only the
  *prompt* — the segmenter still returns the object silhouette inside it, so
  the mask does not inherit the padding.
- **One forward per frame.** All of a frame's boxes go through a single model
  forward: the image encoder runs once per frame, the prompt decoder once per
  box (`docs/target_localization/mask_representation.md` §6 cost split). N masks cost roughly one
  frame-encode plus N light decodes.
- **Multimask output → highest predicted IoU.** SAM returns several mask
  options per prompt with predicted-IoU scores; the highest-scoring option is
  binarized to the `H×W` boolean blob on the color grid. That argmax option is
  then held to a **predicted-IoU floor**: if its score is below the floor it is
  dropped to `None` (→ that detection skips, no-fallback — §3), rather than
  passing a low-confidence mask downstream. The floor is a node parameter,
  `segmentation_min_iou` (default 0.5). Predicted IoU rates mask *boundary*
  quality, not whether the mask is the robot, so the floor drops
  low-quality/uncertain masks but cannot catch a crisp wrong-object mask.

## 3. Output semantics

- One blob per box, index-aligned with the detections, on the **full color
  grid**. That is this module's boundary and it does not move.
- The consuming node crops each blob to its own extent with
  `region_from_blob(blob, MaskPrecision.TIGHT)` — the module returns plain
  arrays and knows nothing about producers or consumers, mirroring
  `region_from_bbox` on the rect side. The crop is to the blob's **nonzero
  extent, not to the prompt box**: the prompt is padded (Section 2), so a
  returned silhouette can legitimately extend past the box that produced it,
  and cropping to the box would delete those pixels. The crop is copied, so the
  frame-sized blob expires at that point rather than being kept alive as the
  region's backing store. See `docs/target_localization/mask_representation.md` Section 7.
- **Empty or below-floor segmentation → `None` → no measurement for that detection.** A winning mask
  that is empty **or** whose predicted IoU falls below the floor
  (`segmentation_min_iou`, default 0.5 — §2) yields `None` for that detection;
  the node leaves the detection's path fields NaN. No fallback to a rect mask —
  silently rasterizing would mislabel the benchmark row (the run *is* the gate
  axis).

## 4. Execution host: inside `target_mask_measurement_node`

`docs/target_localization/mask_representation.md` §5.3 pins the wire convention: **consumers never take the
mask off the wire**; a published mask topic is debug-only. A tight mask cannot
be reconstructed from the detections message the way a rect mask can, so the
only convention-respecting host is the consuming node itself — the segmenter
runs in `target_mask_measurement_node`'s process and hands the boolean array over
in-process. This mirrors the rect precedent: rasterization also executes in
the consuming node while belonging to the front-end's contract.

Rejected alternatives (both would push masks across a process boundary):
inside `target_detector_node` (couples the two model loops, full-rate mask
serialization even for `box` runs) and a separate segmentation node (same wire
violation plus a third model-hosting process to sequence in bringup).

Consequences the wiring honors:

- **`box` mode does not load a segmentation model.** The segmentation module import
  is torch-free; the model libraries load only when `mask_gate:=silhouette`.
  Other enabled models (detection or monocular depth) have their own dependencies.
  Without the venv the node fails at startup with the same actionable
  `RuntimeError` pattern as `OwlV2Detector.load()` (caught in `main`, fatal
  log, clean exit).
- **Stamp-matched RGB, no gate fallback.** Segmentation needs the exact color
  frame the detections were made on. The detections message inherits the color
  frame's header, so the node (silhouette mode only) buffers recent color
  frames in a stamp-keyed deque (`color_buffer_depth`, default 15 ≈ 0.5 s at
  30 fps) and looks the frame up by exact stamp. A miss skips the frame's
  paths — same convention as a missing depth frame
  or scan, never a downgrade to rect.
- **The worker survives segmenter exceptions** per frame (the
  histogram-crash lesson): a failing frame logs and drops, it does not starve
  trials.

## 5. Benchmark axis

The gate is a run-level axis threaded exactly like `depth_source`: the
`mask_gate` launch argument (default `box`) forwards to both the mask node and
the runner, and folds into every mask-row output name.

Silhouette rows **drop the isolation token** — the tight branches never run an
isolation recipe, so folding `isolation_2d/3d` into the name would describe
code that did not execute:

- `box_gated_stereoscopic_projective_ranging_nearest_mode_histogram`
- `silhouette_gated_stereoscopic_projective_ranging`
- `silhouette_gated_polar_profiling`

Display prose follows the same rule: `silhouette-gated stereoscopic projective
ranging`.

## 6. Debug artifact

In silhouette mode the node publishes the union of the frame's tight masks as
a `mono8` Image on `debug/target/mask` (debug-only; see the mask representation reference, Section 5.3). The overlay mask panel consumes
the published artifact when its stamp matches the rendered measurements
exactly, so the panel shows exactly what downstream consumed; otherwise it
keeps deriving the rect union at render time. This is visualization only; measurement consumers use in-process masks,
not this published image.
