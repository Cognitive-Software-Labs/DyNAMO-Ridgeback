# Segmentation Component — Implementation Plan

**Goal:** add the *tight* (silhouette) mask front-end promised by
`mask_component.md` §4: an instance-segmentation producer that emits
pixel-precise masks into the existing mask interface, wired as a second value
of the benchmark's mask-gate axis (`box` → `silhouette`).

**Status:** implemented (2026-07-16), all phases. The component doc proper is
`segmentation_component.md`; this document is kept as the executed plan.
Spike outcome: `Zigeng/SlimSAM-uniform-50` pinned, prompt padding 0.05.

**Direction revision (2026-07-17), closed same day:** Florence-2 was briefly
adopted as the main path (crowded scenes + multi-G1 motivation), then its
spike failed three of four acceptance gates — leg-gap fidelity, latency, and
multi-instance separation — so the main path **reverted to the SlimSAM
producer this plan delivered**. The failure record and the remaining
fallback options (SAM 3, sim-trained segmenter) are in
`segmentation_component.md` §7. This plan is unedited below — it records what
was executed.

---

## 1. Current state — what exists, what is missing

Already in place (no changes needed):

- **Mask interface** (`perception/core/mask.py`): `Mask` (H×W bool + precision
  tag), `mask_from_array(blob, MaskPrecision.TIGHT)` is the documented
  integration hook for exactly this front-end.
- **Tight consumption branches** — implemented *and* unit-tested in the pure
  suites, but never exercised by a real producer:
  - projective ranging: `tight` → direct robust median, isolation recipe never
    called (`core/projective_ranging.py`);
  - euclidean reconstruction: `tight` → median ± k·MAD outlier removal instead
    of the 3D isolation recipe (`core/euclidean_reconstruction.py`);
  - polar profiling: deliberately **no** tight/rect fork — parallax survives a
    tight mask, so run-segmentation + near-band merge stays identical
    (`core/polar_profiling.py`, rationale in `lidar_based_path.md` §2.5).
- **Gate axis stub**: `MASK_GATE = 'box'` constant in
  `benchmarking/estimators.py`, with the `silhouette` token reserved in its
  comment and folded into every mask-row output name.
- **Prompt source**: OWLv2 detections (post-NMS, clamped, non-degenerate
  `bbox_xyxy` on the color grid) flow to `g1_mask_measurement_node`, which
  rasterizes per detection — the 1:1:1 frame→detection→mask hierarchy
  (`mask_component.md` §6) is exactly the prompt structure a box-promptable
  segmenter needs.
- **Dependencies**: `perception_venv` already carries transformers 5.10.2 +
  CUDA torch 2.12.0 — the SAM family loads through the same transformers API
  the detector and Depth-Anything use. No new dependency expected.

Missing:

1. the segmentation model wrapper (the producer),
2. node wiring that turns detections + the RGB frame into tight masks,
3. the benchmark gate axis as a run-level parameter instead of a constant,
4. (optional) the real-artifact mask debug topic for the overlay panel,
5. docs.

---

## 2. Design decisions

### 2.1 Model: box-prompted SAM family (not standalone instance segmentation)

The target class ("humanoid robot") is open-vocabulary — closed-vocabulary
instance segmenters (COCO-trained Mask2Former, YOLO-seg) cannot produce it
zero-shot. A **promptable segmenter prompted by the OWLv2 boxes** keeps the
open-vocab property, reuses the existing detector unchanged, and preserves the
1:1:1 hierarchy: one box prompt → one mask. This is also the extension the
pipeline doc anticipates ("a promptable segmenter like SAM",
`Object_Localization_Pipeline.md` §3).

Candidates (all box-promptable via transformers, pinned at the spike):

| Model | HF id | Expectation |
|---|---|---|
| SAM ViT-B | `facebook/sam-vit-base` | reference quality, mature API |
| SlimSAM | `Zigeng/SlimSAM-uniform-50` | SAM-compatible API, much lighter |
| SAM 2.1 tiny/small | `facebook/sam2.1-hiera-tiny` | better + faster, newer API surface |

Selection criterion is **silhouette fidelity on the G1's legs**: the leg gap
must come out `False` — that gap is precisely what the tight mask buys polar
profiling (rect admits through-the-gap background rays; see
`Object_Localization_Pipeline.md` §5 parallax note). Latency and VRAM are
secondary on the benchmark machine (Blackwell GPU) but recorded for the real
robot decision later.

### 2.2 Execution host: inside `g1_mask_measurement_node` (in-process)

`mask_component.md` §5.3 pins the convention: **consumers never take the mask
off the wire**; a published mask topic is debug-only. A tight mask cannot be
reconstructed from the detections message the way a rect mask can, so the only
convention-respecting host is the consuming node itself — the segmenter runs
in `g1_mask_measurement_node`'s process and hands the boolean array over
in-process. This mirrors the existing precedent: rasterization also executes
in the consuming node while belonging to the front-end component's contract
(`Object_Localization_Pipeline.md` §3).

Rejected alternatives:

- *Inside `g1_detector_node`*: masks would have to cross a process boundary to
  reach the measurement node — violates the wire convention, couples the two
  model loops, and adds full-rate mask serialization for a benchmark that may
  run gate `box`.
- *Separate `g1_segmentation_node`*: same wire violation, plus a third
  model-hosting process to sequence in bringup.

Consequences the wiring must honor:

- **`box` mode must keep zero model dependencies.** The node currently imports
  no torch and runs outside the venv. The segmenter import/load happens lazily
  and only when `mask_gate:=silhouette`; without the venv it fails cleanly
  with the same actionable message pattern as `OwlV2Detector.load()`.
- **Per-frame vs per-mask cost split** (`mask_component.md` §6): run the SAM
  image encoder **once per frame**, then one cheap prompt-decode per
  detection. N masks stay roughly one frame-encode plus N light decodes.

### 2.3 RGB frame access: stamp-matched buffer, no gate fallback

Segmentation needs the exact color frame the detections were made on. The
detections message inherits the color frame's header
(`g1_detector_node.process_color_image` publishes with `color_msg.header`), so
the stamp is an exact match key. The measurement node (silhouette mode only)
subscribes to the color topic and keeps a short deque of recent frames
(~15 at 30 fps ≈ 0.5 s — comfortably above detector latency at 5 FPS; depth a
node parameter); on each detections message it looks up the frame by exact
stamp.

**Miss policy: skip, never downgrade.** If the frame has aged out, the frame's
path fields stay NaN with a rate-limited warning — same convention as a
missing depth frame or scan. Silently rasterizing a rect mask instead would
mislabel the benchmark row (the run *is* the gate axis) and violate the pinned
no-fallback rule (`None` = trial dropped).

Same policy per detection: an empty or degenerate mask from the segmenter
(below the paths' minimum-pixel floors) yields `None` from the path, and the
trial drops — honestly reported, not patched over.

### 2.4 Benchmark axis: `MASK_GATE` constant becomes the `mask_gate` parameter

Threaded exactly like `depth_source`:

- launch arg `mask_gate:=box|silhouette` (default `box`) on
  `g1_distance_benchmark.launch.py`, forwarded to both the mask node and the
  runner (params kept in sync via launch, existing convention);
- `benchmark_output_name` / `benchmark_display_name` take the gate as an
  argument instead of reading the constant.

**Naming subtlety — silhouette rows drop the isolation token.** The tight
branches never run an isolation recipe, so folding `isolation_2d/3d` into a
silhouette row's name would describe code that didn't execute:

- `box_gated_stereoscopic_projective_ranging_nearest_mode_histogram` (as today)
- `silhouette_gated_stereoscopic_projective_ranging` (no isolation token)
- `silhouette_gated_polar_profiling`

Display prose follows the same rule: `silhouette-gated stereoscopic projective
ranging` (isolation was never in the prose form anyway).

### 2.5 Box-prompt padding

OWLv2 boxes can clip the object; a SAM box prompt truncates hard at its
boundary. Expose a small relative inflation of the prompt box (e.g. 5–10%,
clamped to the grid) as a constant pinned at the spike. The *mask* stays
whatever the segmenter returns — padding widens only the prompt, and SAM
segments the object inside it, so the silhouette does not inherit the padding.

---

## 3. Implementation phases

Each phase lands independently; the benchmark stays green after every phase.

### Phase 0 — model spike (offline, no repo code changes)

- Capture a handful of sim frames with G1 detections (office world; reuse
  benchmark image dumps if available) — plus real D435 frames if any exist.
- Script in scratchpad: load each candidate, prompt with the recorded OWLv2
  boxes, save silhouette overlays; record per-frame encoder + per-box decode
  latency and VRAM alongside OWLv2 + Depth-Anything resident.
- Verify the transformers 5.10.2 API surface for SAM 2.1 specifically (SAM 1 /
  SlimSAM are long-stable; SAM 2 support is newer).
- **Exit criterion:** pinned `SEGMENTATION_MODEL_DEFAULT` + prompt padding
  value, leg-gap check passed, latency compatible with the 5 FPS detector
  cadence.

### Phase 1 — core module: `perception/core/segmentation.py`

- `SEGMENTATION_MODEL_DEFAULT`, `SamBoxSegmenter` mirroring `OwlV2Detector`'s
  shape: `__init__(model_name, logger)`, lazy `load()` raising the actionable
  venv `RuntimeError`, `resolve_torch_device()` reuse.
- `segment_boxes(frame, boxes_xyxy) -> list[np.ndarray | None]` — encoder once,
  decode per box, multimask output resolved to the highest-scoring mask,
  binarized to `H×W` bool on the color grid; `None` for an empty result.
- Pure helpers split out for torch-free tests: prompt padding/clamping, best-
  mask selection from (masks, scores), binarization.
- Call-site wrapping stays `mask_from_array(blob, MaskPrecision.TIGHT)` — the
  module returns arrays, the node builds `Mask` objects (keeps the core module
  free of producer/consumer coupling, same as `rasterize_*`).
- Tests `test_segmentation.py`: pure helpers + wrapper contract via a fake
  model (shape/dtype, 1:1 box→mask alignment, empty handling). Register in
  `CMakeLists.txt` (install-PROGRAMS gotcha: rebuild before ROS-side runs).

### Phase 2 — node wiring: `g1_mask_measurement_node`

- New params: `mask_gate` (`box` default), `color_topic`, buffer depth.
- Silhouette mode adds: color subscription + stamp-keyed deque; lazy segmenter
  construction on first use; per-frame encode, per-detection decode →
  `mask_from_array(..., TIGHT)`; miss/empty policies per §2.3.
- `box` mode: byte-for-byte current behavior, still zero torch imports.
- Worker keeps the survive-any-frame guard (segmenter exceptions must not
  starve trials — the histogram-crash lesson).
- Tests: gate param validation, stamp-match hit/miss behavior, and a fake-
  segmenter end-to-end pass through `fill_path_measurements` confirming the
  tight branches run (first real-producer exercise of those branches).

### Phase 3 — benchmark axis: `benchmarking/estimators.py` + launch + runner

- `MASK_GATE` constant → validated `mask_gate` run parameter (`box` |
  `silhouette`); naming rules per §2.4.
- `g1_distance_benchmark.launch.py`: `mask_gate` arg forwarded to mask node +
  runner; README public-params table entry.
- Tests: output/display names for both gates (incl. silhouette dropping the
  isolation token), runner↔node param sync.

### Phase 4 — visualization (optional, deferrable)

- When silhouette is active, publish the debug-only encoded mask:
  `mono8` Image on `debug/g1/mask` (compressed transport gives PNG — a few kB
  per frame, per the wire-cost note in `mask_component.md` §5.3).
- Overlay mask panel consumes the published artifact when present; otherwise
  keeps deriving the rect union at render time. Resolves the §7 open item
  "visualization data source" — the panel then shows exactly what downstream
  consumed, and renders `tight` vs `rect` with zero panel changes.
- Not on the benchmark critical path; can land after Phase 5.

### Phase 5 — validation

- Colcon rebuild, full pure suites, ROS suites (msg untouched, but node
  scripts are `install(PROGRAMS)` copies).
- **Sim A/B run:** identical config except the gate —
  `depth_source:=stereoscopic`, default isolation recipes, default polar
  params, all three mask rows, full 1.5–5.5 m grid:
  - `mask_gate:=box` (reproduces current baselines: depth paths ≈ 0.068 m MAE,
    polar 0.093 m),
  - `mask_gate:=silhouette`.
- Expectations to test, not assume:
  - depth paths: modest change — the 2D/3D isolation recipes already isolate
    well in sim; the silhouette's win there is removing the recipe dependence
    (one fewer tuning axis), not necessarily MAE;
  - polar profiling: the likeliest MAE gain — silhouette gating drops
    leg-gap background rays before run-segmentation. Keep polar params frozen
    across both gates so the gate effect is not confounded with the separate
    polar-param-tuning open item.
- Record per-frame segmentation latency from the node logs; sanity-check the
  5 FPS detector cadence still holds end-to-end.
- Real-hardware pass stays parked with the other real-robot open items
  (`align_depth.enable`, TF confirmation).

### Phase 6 — docs

- **New `segmentation_component.md`**: the component doc proper (model,
  prompting, padding, multimask selection, empty-mask semantics, execution
  host rationale) — the doc `mask_component.md` §4 defers to.
- `mask_component.md`: §2/§4 tight front-end no longer "future"; §7 open items
  (tight front-end ✓, visualization source ✓ if Phase 4 lands).
- `Object_Localization_Pipeline.md`: §3 segmentation component now
  implemented; flowchart tag unchanged (already drawn).
- `README.md`: `mask_gate` public param + benchmark usage line.
- `ISSUES.md`: only if the spike surfaces an operational gotcha (e.g. SAM2 API
  pin).
- Graphify rebuild after code phases (`tools/rebuild_graphify`; currently not
  installed on this machine — run where available).

---

## 4. Risks and open questions

- **SAM 2 API in transformers 5.10.2** — verify at the spike; SAM 1 / SlimSAM
  are the safe fallback with identical prompting semantics.
- **VRAM with three resident models** (OWLv2 + Depth-Anything + SAM): fine on
  the benchmark GPU; the real robot's compute is undecided — model-size
  parameter (`segmentation_model`) stays public for that reason.
- **Prompt quality ceiling**: the silhouette can only be as good as the OWLv2
  box that prompts it — a badly clipped box truncates the mask even with
  padding. If the spike shows this dominating, that is detector-threshold /
  padding tuning, not a segmentation defect.
- **Isaac Sim port (1280×720)**: no new risk — the mask is built on the
  detection grid by construction, and the existing `grid_mismatch_warning`
  guard already covers detection-vs-camera_info divergence.
- **Sim visual gap**: SAM segments rendered Gazebo frames differently than
  real imagery; sim A/B results guide the gate axis but the model pin should
  be revisited on real D435 data.
