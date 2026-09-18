# ROI-mask representation measurements — August 31, 2026

Recorded dates: 2026-08-31

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Scope

The migration replaced full-image mask storage with owned ROI-sized data while
preserving global pixel coordinates, precision policies, misses, and output
contracts. This record retains parity evidence and measured tradeoffs; it does
not establish lower whole-system latency or resolve depth-delivery losses.

## Correctness evidence

Reference results were captured from the **pre-migration** implementation and
checked in as `test/mask_region_pre_migration_reference.json`; the parity tests
compare against that file, not against a second wrapper of at the recorded date's code. Twenty
reference cases cover both precisions, both 2D recipes and all three 3D recipes,
finite and unlimited depth gates, float32 and float64 depth, a scene carrying
zero / NaN / inf / negative returns, both starvation reasons, and polar beam
selection under both mask types.

Both entry points pass the reference comparisons: exact checks for miss reasons,
foreground pixel/point counts and original beam indices, plus numerical checks
for recorded depth, representative global UV and optical coordinates using
`pytest.approx(..., abs=1e-9)`. Euclidean point sets are checked through their
count, first/last points and coordinate sums (the sums use `rel=1e-12`), not an
element-by-element comparison of the entire ordered set. These are fixture-level
parity checks, not a claim of bitwise equality for every runtime output.

Allocation assertions use `tracemalloc`'s peak counter, which covers numpy data
buffers including temporaries freed before the call returns. They are bounds
relative to one full-frame float64 array — the thing being ruled out — not
timing assertions.

## Measured storage and CPU

Same deterministic inputs before and after; "before" is the full-grid entry
points, which are behaviour-identical to the pre-migration production body.
Warm medians over 60–200 repeats, single process, host under concurrent load
from other sessions. The higher-resolution case is synthetic test data only; no
camera configuration was changed.

### Storage per detection

| Frame | Case | Full-grid mask | Region | Ratio |
|---|---|---:|---:|---:|
| 640×480 | box / tight, ROI ≈ 2% of frame | 307 200 B | 6 084 B | 50.5× |
| 640×480 | box / tight, ROI ≈ 50% of frame | 307 200 B | ≈153 000 B | 2.0× |
| 1280×720 | box / tight, ROI ≈ 2% of frame | 921 600 B | ≈18 300 B | 50.4× |
| 1280×720 | box / tight, ROI ≈ 50% of frame | 921 600 B | ≈460 000 B | 2.0× |

Every region reports `data.base is None`: no region retains the segmenter's
frame-sized output as backing storage.

### Both depth estimators, one shared preparation (640×480)

| Detections × ROI | Before med/p95 ms | After med/p95 ms | Before peak B | After peak B |
|---|---|---|---:|---:|
| 1 × small | 0.87 / 0.92 | 0.28 / 0.29 | 1 025 084 | 800 744 |
| 4 × small | 3.38 / 3.55 | 0.98 / 1.00 | 1 025 216 | 800 928 |
| 12 × small | 10.10 / 10.29 | 2.98 / 3.18 | 1 026 900 | 802 981 |
| 1 × large | 3.77 / 4.18 | 3.55 / 4.16 | 6 400 852 | 7 342 945 |
| 4 × large | 13.27 / 13.98 | 11.74 / 12.39 | 5 358 788 | 6 076 200 |
| 12 × large | 16.78 / 19.19 | 9.91 / 10.31 | 2 142 976 | 2 166 203 |

At 1280×720 the same shape holds: 12 × small goes 29.6 → 7.7 ms, 12 × large
53.5 → 29.4 ms, 1 × large 11.8 → 11.4 ms.

### Other stages (640×480)

| Stage | Before | After |
|---|---|---|
| bbox construction | 0.0073 ms | 0.0029 ms |
| tight construction | 0.0006 ms | 0.0161 ms |
| debug union, 12 detections | 0.29 ms | 0.04 ms |
| `deproject_masked` | 0.067 ms, 2 498 200 B peak | 0.024 ms, 229 920 B peak |
| full-image mask, standalone API | 1.73 / 1.87 ms | 1.77 / 1.85 ms |

### Cases that got slower, or did not improve

- **Tight-mask construction is ~27× slower** (0.0006 → 0.0161 ms at 640×480;
  0.0006 → 0.0327 ms at 1280×720). The old producer wrapped the model's array in
  O(1); the new one scans for the nonzero extent and copies. This is the price of
  not pinning a frame-sized model output per detection, and it sits next to a
  segmentation forward pass costing tens of milliseconds.
- **Large-ROI peak allocation is higher** (1 × large at 640×480: 6.40 → 7.34 MB).
  The extra is the two global index arrays `global_pixels` produces for a
  non-zero origin. It is proportional to the selection rather than to the frame,
  which is the intended trade; wall time for the same case still improved
  slightly. A window at the origin returns its inputs instead, which is what
  keeps the full-image standalone case at parity.
- **1 × large is essentially unchanged in time** (3.77 → 3.55 ms). A mask
  covering half the frame has little ROI to save.

Smaller mask storage and these CPU microbenchmarks do not establish lower
end-to-end latency or resolve dropped frames. The simulator runs below check
integration and aggregate accuracy; they are not a controlled end-to-end latency
comparison.

## Simulator smoke, both mask gates

`examples` scenario, 5 scenes / 8 instances, 1 repeat, 2 s settle, 10 s capture,
all four estimators, stereoscopic depth, default recipes, unlimited mask-depth
gate, headless. Artifacts (temporary, not checked in):
`/tmp/dynamo-roi-smoke/box` and `/tmp/dynamo-roi-smoke/silhouette`.

**Box-gate aggregate MAE matches the recorded baseline to six decimal places,
with the same scored-instance counts.** The baseline column is the pre-refactor
run recorded in `archive/engineering/refactor_validation.md`. This does not establish
per-frame or bitwise equality:

| Estimator | MAE at that stage (m) | Pre-migration MAE (m) | Δ | Scored |
|---|---:|---:|---:|---|
| Point cloud | 0.129655 | 0.129655 | 0.000000 | 6/8 |
| Projective ranging | 0.055221 | 0.055221 | 0.000000 | 6/8 |
| Euclidean reconstruction | 0.056944 | 0.056944 | 0.000000 | 6/8 |
| Polar profiling | 0.081697 | 0.081697 | 0.000000 | 5/8 |

Miss reasons are the expected ones: `NO_DEPTH_FRAME` ×13 of 246 on both depth
rows — the residual exact-stamp gap, untouched by this change — and
`TOO_FEW_RAYS_SELECTED` ×41 for polar. Four scene collages rendered.

**Silhouette gate exercises `region_from_blob` in production.** There is no
pre-migration silhouette baseline to diff against, so this is an integration
check rather than a before/after parity check. Aggregate distance errors alone
cannot verify every crop origin or rule out coordinate bugs. Crop placement and
tight-path numerical parity are checked by the unit and reference tests above:

| Estimator | Box MAE (m) | Silhouette MAE (m) | Scored (both) |
|---|---:|---:|---|
| Point cloud | 0.129655 | 0.129655 | 6/8 |
| Projective ranging | 0.055221 | 0.052476 | 6/8 |
| Euclidean reconstruction | 0.056944 | 0.060593 | 6/8 |
| Polar profiling | 0.081697 | 0.072268 | 5/8 |

`pointcloud` consumes no mask, and its aggregate MAE matches to six decimals
across both runs. This is a useful consistency check, not proof of identical
input frames or a deterministic harness; it does not isolate all mask-row
differences from run-to-run variation.

Two mask rows have lower aggregate MAE in this comparison; euclidean has higher
MAE. Tighter masks do not guarantee improved estimates. The existing `tight` MAD
versus `rect` 3D isolation fork is a possible contributor, but these two runs do
not establish causality or independently exclude a geometry fault. The unit and
reference tests provide the geometry and parity evidence.

SlimSAM ran 11 segmentation batches and the mask rows produced
230 / 230 / 203 OK observations. `NO_COLOR_FRAME` ×3
appears only under this gate, which is correct — it is the one gate that needs
the exact-stamp color frame. Both runs recorded one `UNSET` batch.

## Not performed

- **No exploration run.** Exploration shares the same node factories and is
  covered by `test_launch_layout`, but nothing here drove it live.
- **No hardware validation**, and no sweep-scale benchmarking.
- The silhouette gate has no pre-migration numeric baseline, so its parity
  evidence is the unit and reference tests rather than the benchmark run.
