# Segmentation candidate experiments — July 2026

Recorded dates: 2026-07-17

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Scope

The Phase 0 SAM comparison had no separately recorded date. Florence-2 and
SAM3 spikes were dated July 17. These runs preceded the August 31 D455 geometry
correction. Temporary artifact availability has not been reverified. Passing
an offline gate did not establish benchmark parity or authorize integration.

## 1. Phase 0: recorded SAM comparison and selection

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
polar profiling (a rectangular mask admits through-the-gap background rays). All three candidates passed the leg-gap
check visually; SlimSAM won on stable quality at the lowest footprint. Latency
was well inside the then-recorded 5 FPS detector cadence. That experiment did not settle the target hardware compute budget.

The recorded padding check found that 5% recovered box-clipped feet at
3.5 m without observed background bleed; 10% added no improvement in that check.
This is a result on those frames, not a guarantee for physical-camera imagery.

## Recorded candidate protocol

Eight 640×480 simulation frames covered six single-target views at 1.5–5.5 m
and two crowded views (overlapping and side-by-side). Recorded OWLv2 boxes
served as references. A warmup preceded one synchronized GPU-timed pass per
frame; resident and peak VRAM were recorded.

The gates required leg-gap fidelity matching the reference's 0.89+ split-row
fraction in the bottom 35% of each box, at most 200 ms/frame including all
instances, and separate disjoint masks with usable confidence scores in the
crowded views. Benchmark MAE parity could not be tested without integration.

## 3. Florence-2 — evaluated, spike FAILED (2026-07-17)

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

**Spike outcome — failed three of four acceptance gates.** Run per the Section 2
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

## 4. SAM 3 (concept segmentation) — spike PASSED (2026-07-17), adoption open

**How it works:** one model, concept prompt ("humanoid robot") → detects,
segments, and instance-separates **all** matches in a single pass. The
detector is absorbed, but with SAM-grade per-pixel masks instead of polygons.

**Why it is attractive:** instance-native in crowds (built exactly for the
many-instances-of-one-concept case); per-pixel mask quality — no polygon
coarseness risk; replaces both front-end stages at once; per-instance
confidence scores (which Florence-2 lacked, Section 3).

**Spike outcome — passed all three testable gates.** Run per the Section 2
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
foot-clipping (the reason prompt padding exists; see the segmentation reference), with the SAM 3 boxes
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
OWLv2 + SlimSAM) or keep the then-recorded two-stage path — was left open on July 17. The spike did not establish benchmark MAE
parity or fit within the target robot's VRAM budget.
