# Segmentation Component

**Scope:** the *tight* (silhouette) mask front-end — the instance-segmentation
producer that `mask_component.md` §4 defers to. This document covers the model,
how it is prompted, and the semantics of its output. How the resulting mask is
consumed is unchanged and documented with the paths; the mask *object* is
documented in `mask_component.md`.

**Code:** `perception/target_localization/core/segmentation.py` (`SamBoxSegmenter`), wired into
`target_mask_measurement_node` behind the `mask_gate` parameter
(`box` | `silhouette`).

**History note (2026-07-17):** Florence-2 was evaluated as a candidate
replacement main path (single model for detection + segmentation, motivated by
crowded scenes and multiple G1s per frame). Its spike **failed three of four
acceptance gates** — see §7.1 — so the box-prompted SAM family below remains
the main path. The failure record and the other evaluated alternatives are
kept in §7.

---

## 1. Model: box-prompted SAM family

The target class ("humanoid robot") is open-vocabulary, so a closed-vocabulary
instance segmenter (COCO-trained Mask2Former, YOLO-seg) cannot produce it
zero-shot. Instead a **promptable segmenter is prompted with the OWLv2
detection boxes**: one box prompt → one pixel-precise mask. This keeps the
detector's open-vocabulary property, reuses the existing detector unchanged,
and preserves the 1:1:1 frame → detection → mask hierarchy
(`mask_component.md` §6).

**Pinned default:** `Zigeng/SlimSAM-uniform-50` (SlimSAM, a pruned SAM with
the identical prompting API), selected by the Phase 0 spike on raw sim frames
across the full 1.5–5.5 m benchmark grid against `facebook/sam-vit-base` and
`facebook/sam2.1-hiera-tiny`:

| Candidate | Predicted IoU (1.5–5.5 m) | Full forward | Resident VRAM |
|---|---|---|---|
| SlimSAM-uniform-50 (**pinned**) | 0.94–0.99, stable | ~29 ms | ~116 MB |
| sam-vit-base | 0.95–0.98, stable | ~41 ms | ~380 MB |
| sam2.1-hiera-tiny | 0.82–0.95, dips mid-range | ~19 ms | ~129 MB |

The selection criterion was **silhouette fidelity on the G1's legs**: the leg
gap must come out `False` — that gap is precisely what the tight mask buys
polar profiling (a rect mask admits through-the-gap background rays,
`object_localization_pipeline.md` §5). All three candidates passed the leg-gap
check visually; SlimSAM won on stable quality at the lowest footprint. Latency
is well inside the 5 FPS detector cadence. The model stays a public parameter
(`segmentation_model`) because the real robot's compute budget is undecided;
both SAM 1 and SAM 2 checkpoints load (the family is dispatched on the
checkpoint's `model_type`).

The specific detector-plus-segmenter stack (OWLv2 + SlimSAM) is an
**implementation detail of this component, not an architectural fixture** —
`object_localization_pipeline.md` §2 shows only the abstract segmentation
front-end, and nothing downstream of the mask interface depends on the choice.
It is a **swappable, benchmarkable axis**: multiple segmentation implementations
are planned to be compared on the same three metrics used above —
**accuracy** (silhouette / leg-gap fidelity, and MAE parity once wired into the
distance benchmark), **processing time**, and **memory usage** (resident VRAM).
The spikes recorded in §7 are early, one-off entries in that comparison, not the
systematic run.

The spike ran on **rendered Gazebo frames**; SAM segments real imagery
differently, so the pin should be revisited on real D435 data before the
real-hardware pass.

**Known limit (accepted):** SAM-family models are class-blind — the box prompt
carries all class knowledge, so crowding robustness and the same-class overlap
case (two adjacent G1s in one ambiguous box) are inherited from the detector
and cannot improve inside the segmenter. The alternatives in §7 were evaluated
against exactly this limit; none has displaced the main path so far, though
the SAM 3 spike passed its gates and the adoption question is open (§7.2).

## 2. Prompting

- **Prompt = padded detection box.** OWLv2 boxes can clip the object, and a
  SAM box prompt truncates hard at its boundary, so the prompt box is inflated
  by a small relative padding (`PROMPT_PADDING_REL_DEFAULT = 0.05`, i.e. 5% of
  each extent, clamped to the grid) before prompting. Padding widens only the
  *prompt* — the segmenter still returns the object silhouette inside it, so
  the mask does not inherit the padding. The spike showed 5% recovering
  box-clipped feet at 3.5 m with no background bleed; 10% added nothing.
- **One forward per frame.** All of a frame's boxes go through a single model
  forward: the image encoder runs once per frame, the prompt decoder once per
  box (`mask_component.md` §6 cost split). N masks cost roughly one
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

- One blob per box, index-aligned with the detections.
- The consuming node wraps each blob with
  `mask_from_array(blob, MaskPrecision.TIGHT)` — the module returns plain
  arrays and knows nothing about producers or consumers, mirroring
  `rasterize_*` on the rect side.
- **Empty or below-floor segmentation → `None` → trial drops.** A winning mask
  that is empty **or** whose predicted IoU falls below the floor
  (`segmentation_min_iou`, default 0.5 — §2) yields `None` for that detection;
  the node leaves the detection's path fields NaN. No fallback to a rect mask —
  silently rasterizing would mislabel the benchmark row (the run *is* the gate
  axis).

## 4. Execution host: inside `target_mask_measurement_node`

`mask_component.md` §5.3 pins the wire convention: **consumers never take the
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

- **`box` mode keeps zero model dependencies.** The segmentation module import
  is torch-free; the model libraries load only when `mask_gate:=silhouette`.
  Without the venv the node fails at startup with the same actionable
  `RuntimeError` pattern as `OwlV2Detector.load()` (caught in `main`, fatal
  log, clean exit).
- **Stamp-matched RGB, no gate fallback.** Segmentation needs the exact color
  frame the detections were made on. The detections message inherits the color
  frame's header, so the node (silhouette mode only) buffers recent color
  frames in a stamp-keyed deque (`color_buffer_depth`, default 15 ≈ 0.5 s at
  30 fps) and looks the frame up by exact stamp. A miss skips the frame's
  paths with a rate-limited warning — same convention as a missing depth frame
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
a `mono8` Image on `debug/target/mask` (debug-only, per the §5.3 wire convention;
compressed transport gives PNG at a few kB). The overlay mask panel consumes
the published artifact when its stamp matches the rendered measurements
exactly, so the panel shows exactly what downstream consumed; otherwise it
keeps deriving the rect union at render time. This resolves the
"visualization data source" open item of `mask_component.md` §7.

## 7. Other options evaluated

Alternatives assessed against crowded scenes and multiple G1s per frame — the
two requirements the main path's class-blindness limit (§1) motivates. Two
have been spiked: Florence-2 failed (§7.1); SAM 3 passed and its adoption is
an open decision (§7.2). YOLO-seg remains an unspiked fallback (§7.3).

### 7.0 Spike protocol (shared by §7.1 and §7.2)

Every candidate is evaluated **offline, against acceptance gates fixed before
the run, with no wiring into the pipeline**. A one-session throwaway script
loads the candidate in the perception venv, runs the fixed frame set, and
archives the evidence (a results JSON plus one overlay image per frame) under
a per-candidate directory in `~/tmp/` — the evidence outlives the script; the
script is not maintained. A candidate that fails any gate is recorded and
dropped; a candidate that passes earns a *decision*, not adoption.

**Frame set.** Eight 640×480 Gazebo renders captured from the benchmark
scene:

- Six single-G1 frames sampling the benchmark grid — one per forward distance
  (1.5 m twice, then 2.5, 3.5, 4.5, 5.5 m) across lateral offsets −0.75, 0.0
  and +0.75 m. Each carries the OWLv2 detection box recorded for it during
  the SlimSAM spike, used as the reference box.
- Two two-G1 frames covering the crowding requirement: one with the robots
  overlapping in depth (their boxes overlap in the image) and one with them
  side by side.

**Measurement process.** One warm-up pass runs first so model compilation and
cache effects stay out of the numbers. Each frame is then processed once,
end-to-end (pre-processing, forward pass, post-processing to final masks),
with GPU work synchronized before and after timing. GPU memory resident after
load and the peak across the run are recorded alongside.

**The gates and their parameters:**

1. **Leg-gap fidelity.** The through-the-legs gap is what the tight mask buys
   polar profiling (§1), so it is the first thing a candidate must not lose.
   Metric: within the bottom 35% of each instance's box (the leg region), the
   fraction of mask rows that show at least two separate filled runs — i.e. a
   visible gap between the legs. Bar: the SlimSAM reference achieves 0.89+
   on every frame of the grid; a candidate must match that shape, not just
   approach it.
2. **Latency.** Budget: **200 ms per frame, all instances included**,
   matching the 5 FPS detector cadence the front-end runs at. Measured as the
   synchronized end-to-end time above, after warm-up.
3. **Multi-instance separation.** On the two-G1 frames the candidate must
   produce exactly one confident instance per robot, with pairwise-disjoint
   masks (mask overlap ≈ 0). Spurious extra instances are tolerable only if
   the model emits per-instance confidence scores that cleanly separate them
   from the real robots — without scores there is no principled way to
   suppress duplicates (the Florence-2 lesson, §7.1).
4. **Benchmark MAE parity.** Distance-estimate parity against the SlimSAM
   silhouette benchmark rows. Not measurable in a spike — it requires wiring
   the candidate in as a producer — so it is recorded as untested and becomes
   the first post-adoption check. A failure of any earlier gate makes it moot.

**Also recorded, outside the gates:** the candidate's own instance boxes are
compared against the recorded OWLv2 reference boxes (box overlap per frame)
to judge whether the candidate could absorb the detector stage as well; where
the model emits per-instance confidence scores, the spike keeps them with a
detection threshold of 0.5 so that borderline instances stay visible in the
evidence rather than being filtered before inspection.

### 7.1 Florence-2 — evaluated, spike FAILED (2026-07-17)

**How it works:** a small vision-language model (0.23B base / 0.77B large,
transformers-native) exposing multiple vision tasks as task-prompt tokens on
one set of weights. Two tasks map onto this component: **phrase grounding**
(text "humanoid robot" → boxes for all matching instances) and
**region-to-segmentation** (one box region → a polygon outline). Chained, one
model could replace both OWLv2 and the SAM-family segmenter, moving class
discrimination into the segmentation stage itself.

**Why it was attractive:** one small checkpoint for both front-end stages;
clutter rejected by the text prompt rather than post-hoc heuristics; one box
per instance from a single grounding pass; smallest footprint of the
all-in-one open-vocab options.

**Spike outcome — failed three of four acceptance gates.** Run per the §7.0
protocol against `florence-community/Florence-2-base` and `-large` (the two-G1
frames were freshly captured for this spike and joined the protocol's frame
set). Evidence archived in `~/tmp/florence_spike/`.

| Gate | base | large | Verdict |
|---|---|---|---|
| Leg-gap fidelity | fails from 3.5 m (gap rows 0.0; half the body missed) | holds to 4.5 m, legs fuse into one column at 5.5 m (0.11) | **FAIL** — the filled polygon bridges the legs (SlimSAM: 0.89+ everywhere) |
| Latency | 1.9–6.5 s per region | 0.9–4.5 s per region | **FAIL** — region-to-segmentation is a generation pass emitting polygon tokens; 5–30× over the 200 ms cadence budget (`num_beams=1` buys ~3×, nowhere near). Grounding itself is 60–190 ms |
| Multi-instance separation | 4 boxes for 2 overlapping G1s (duplicates, box-IoU 0.88) | 4 boxes + a hallucinated floor-shadow instance | **FAIL** on the overlap case (side-by-side is clean); grounding emits no scores, so NMS cannot resolve the duplicates |
| Benchmark parity | — | — | untestable given the latency gate |

The one clean positive: grounding recall on single-G1 frames was 100% across
the grid on both sizes.

Environment pin for any retry: the `microsoft/Florence-2-*` checkpoints do
**not** load through the native transformers 5.10.2 classes
(`RobertaTokenizer has no attribute image_token`) — the converted checkpoints
live under `florence-community/`.

**Consequence:** the Florence-2 producer was not implemented; the polygon
output format is a structural mismatch with the leg-gap requirement, so a
retry would need a fundamentally better polygon fidelity, not just tuning.

### 7.2 SAM 3 (concept segmentation) — spike PASSED (2026-07-17), adoption open

**How it works:** one model, concept prompt ("humanoid robot") → detects,
segments, and instance-separates **all** matches in a single pass. The
detector is absorbed, but with SAM-grade per-pixel masks instead of polygons.

**Why it is attractive:** instance-native in crowds (built exactly for the
many-instances-of-one-concept case); per-pixel mask quality — no polygon
coarseness risk; replaces both front-end stages at once; per-instance
confidence scores (which Florence-2 lacked, §7.1).

**Spike outcome — passed all three testable gates.** Run per the §7.0
protocol against `facebook/sam3` (license-gated on Hugging Face; access must
be requested and granted per account). Evidence archived in
`~/tmp/sam3_spike/`.

| Gate | Result | Verdict |
|---|---|---|
| Leg-gap fidelity | gap rows 0.924–1.0 on every frame at every range | **PASS** — matches SlimSAM's 0.89+ bar (Florence-2: 0.0–0.11) |
| Latency | 139–144 ms per frame, one forward covering *all* instances | **PASS** — inside the 200 ms cadence budget; no per-box decode economy needed |
| Multi-instance separation | side-by-side: exactly 2 instances (scores 0.961/0.958), mask IoU 0.0. Overlapping: both G1s cleanly separated (mask IoU 0.0, leg-gap 1.0) despite overlapping boxes | **PASS** — one extra floor-shadow instance appeared at score 0.641 vs 0.935+ for real robots; unlike Florence-2 the scores exist, so a threshold (~0.75 instead of the spike's 0.5) removes it |
| Benchmark parity | — | untestable in a spike — needs the producer wired into the benchmark (MAE vs the SlimSAM silhouette rows) |

**Detector absorption signal:** SAM 3's instance boxes scored IoU 0.72–0.98
against the recorded OWLv2 boxes; the low end is dominated by OWLv2's known
foot-clipping (the reason prompt padding exists, §2), with the SAM 3 boxes
visually the more correct of the two. Combined with per-instance scores, this
supports absorbing OWLv2 entirely rather than only replacing the segmenter.

**Measured cost:** ~3.4 GB resident / ~4.1 GB peak VRAM (vs SlimSAM's
~116 MB + OWLv2), ~140 ms per frame all-inclusive (vs SlimSAM's ~29 ms
excluding the detector stage). The real-robot GPU budget remains the main
argument against adoption.

**Environment notes:** `Sam3Model`/`Sam3Processor` are native in the pinned
transformers 5.10.2 — the spike script ran unmodified on first attempt.
Access is granted per Hugging Face account (a normal read token is used;
login via `hf auth login` — the `huggingface-cli` entry point is deprecated
and non-functional in the pinned hub version).

**Consequence:** per the decision recorded at spike time, a pass re-opens the
main-path question. The choice — adopt SAM 3 single-stage (replacing
OWLv2 + SlimSAM) or keep the current two-stage path — is open; no producer
has been implemented. If adopted, the remaining gate is benchmark MAE parity
plus a VRAM budget check on the target robot GPU.

### 7.3 Sim-trained dedicated segmenter (YOLO-seg class) — viable fallback

**How it works:** generate pixel-perfect per-instance labels for free from the
simulator (Gazebo and Isaac Sim both offer instance-segmentation ground-truth
sensors), fine-tune a small instance-segmentation network (YOLO-seg family) on
them, run it standalone — no detector, no prompting.

**For:** fastest inference of every option (real-robot CPU plausible);
detector-independent by construction; multi-instance native; crowding handled
by putting clutter in the training scenes — which sim generates for free;
zero annotation cost.

**Against:** closed vocabulary — the network knows the G1 and nothing else; a
new target class means regenerating data and retraining; sim-to-real transfer
on real D435 imagery is the classic failure mode and must be validated before
any real-hardware reliance; a training pipeline becomes repo infrastructure to
own and maintain.
