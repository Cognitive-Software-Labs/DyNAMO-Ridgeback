# Preliminary lossless replay-storage experiment

Recorded dates: 2026-09-10

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Conclusion

On September 10, 2026, a preliminary storage spike selected compressed NPZ for
exact RGB/depth payloads. Its synthetic repetition made the compression ratio
unsuitable as a full-capture capacity estimate.

## Preliminary lossless storage spike

The plan required a storage choice before fixing the full sensor payload. No
full typed capture existed yet, so the preliminary spike used a real benchmark
RGB crop and tiled float32 values from a real legacy depth ROI at 640 x 480.
Every candidate was decoded and checked byte/NaN-equivalent for 20 iterations.

| Layout | Bytes/event | Raw/encoded | Median encode | Median decode |
|---|---:|---:|---:|---:|
| separate `.npy` RGB + depth | 2,150,656 | 1.00x | 0.242 ms | 0.207 ms |
| lossless PNG RGB + `.npy` depth | 1,282,045 | 1.68x | 4.625 ms | 1.232 ms |
| one compressed NPZ | 66,689 | 32.25x | 6.458 ms | 1.207 ms |

The unusually high NPZ ratio is **not** a capacity estimate: tiling a small ROI
repeats depth structure and compresses much better than a real full frame. The
useful result is narrower: compressed NPZ preserved exact uint8/float32
semantics, kept the existing one-trial locality/atomicity contract, required no
new dependency, and decoded in the same measured time as PNG plus NumPy. NPZ was selected at that point. Representative full-capture size,
encode/decode, RSS, and scaling were not measured by this spike.

Sources used by the spike:

- RGB: `artifacts/benchmarks/20260907_144140_seat_validation/`
  `r2_d_silhouette_monocular/images/single_forward_2p5_rep1.png`
- depth values: `artifacts/benchmarks/replay_full_validation_20260909T1900/`
  `dataset/trials/single_back_02.npz`

## Validation boundaries

Offline tests exercised exact RGB/depth round-trips, mask shapes and empty
outcomes, corruption, lineage, deterministic evaluation, and atomic reports.
Injected SlimSAM behavior did not constitute a real-checkpoint validation.
No live/offline parity, representative full-capture storage, model-runtime,
or scaling result was established by this record. The source worktree revision
was not recorded. Open work is maintained separately from this dated evidence.

## Additional undated capture observation

The pre-migration replay reference reported `sensor_capture_full_benchmark`
with 109 trials and 545 events: 526 events (96.5%) had a scan and extrinsics,
versus 86.4% exact RGB and 71.9% exact depth. That reference supplied neither
a capture date nor its tested revision. The comparison describes that capture
only; it does not establish a general reliability ordering between channels.
