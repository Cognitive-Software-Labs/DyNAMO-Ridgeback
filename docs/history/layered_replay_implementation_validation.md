# Layered Replay Implementation Validation

Validated 2026-09-10 in the repository checkout. This note proves the offline
contracts and execution mechanics; it does **not** claim the remaining live
parity, model-runtime, full-capture storage, or scaling gates have run.

## Implemented contract

- Four profiles (`measurement`, `mask-output`, `mask-model`, `live-system`) and
  their question mappings, stage ownership, axes, types/ranges, compatibility,
  claims, and limitations come from one ROS-free registry.
- Canonical jobs validate required inputs, baselines, measurement/model worker
  bounds, profile-owned axes, and materialization variants before execution.
- Typed sensor captures and mask caches use the `dynamo-replay` manifest v2
  envelope, per-payload SHA-256, content-derived IDs, producer signatures, and
  exact parent ID plus parent-manifest hash.
- Sensor payloads round-trip exact RGB bytes and float32 depth semantics,
  including explicit missing evidence and empty raw batches.
- Packed mask caches round-trip rectangular, tight, empty, holed,
  disconnected, outside-prompt, and `None` outcomes without changing the
  canonical `MaskRegion` meaning.
- Box and injected-SlimSAM materializers stream one sensor parent into separate
  immutable children. Changing prompt padding changes both producer signature
  and artifact identity.
- Projective and Euclidean variants reuse the live measurement kernel and the
  same prepared depth region. Multiple caches and variants evaluate in stable
  cache/variant/trial order; one- and two-worker results match exactly.
- The generalized job executor publishes its output directory only after the
  complete report succeeds. Incomplete typed captures and hidden partial output
  directories are not accepted as benchmark evidence.
- Legacy `schema_version: 1` datasets and `target_offline_replay_benchmark`
  remain compatible.

## Automated evidence

The focused layered replay suite covers profile errors, manifest dispatch,
exact RGB/depth, lineage, corruption, incomplete state, mask round-trips,
producer signatures, box/SlimSAM cache evaluation, parallel determinism,
canonical jobs, atomic reports, and import boundaries. The repository suite
also exercises the unchanged legacy replay, sweep, launch, runner, segmentation,
mask, scoring, and report contracts. After rebuilding the ROS package, the
complete non-GUI repository suite passed with **752 tests**.
After the parallel configurator consumed the canonical contract, the combined
installed-package suite passed with **758 tests**.

The import guard blocks ROS message packages, `rclpy`, TF, visualization
messages, Torch, and Transformers while importing the generalized executor.
This proves measurement and frozen-mask evaluation do not load ROS or a model
stack. SlimSAM imports its heavy dependencies only when materialization starts.

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
new dependency, and decoded in the same measured time as PNG plus NumPy. It is
therefore the implemented layout. A fresh full sensor capture must replace the
ratio with representative size, encode/decode, RSS, and worker-scaling evidence.

Sources used by the spike:

- RGB: `artifacts/benchmarks/20260907_144140_seat_validation/`
  `r2_d_silhouette_monocular/images/single_forward_2p5_rep1.png`
- depth values: `artifacts/benchmarks/replay_full_validation_20260909T1900/`
  `dataset/trials/single_back_02.npz`

## Gates still open

- Capture the small edge-case corpus through the live runner, including missing
  exact evidence and oversized/rejected detections.
- Materialize the actual current SlimSAM checkpoint and at least one changed
  checkpoint or model setting from the same sensor parent.
- Prove live/offline outcome, miss-reason, association, and numeric parity for
  box and SlimSAM projective/Euclidean rows.
- Record sequential/parallel byte identity across representative worker counts,
  real full-capture/cache sizes, peak RSS/VRAM, materialization time, replay
  scaling, and the worker knee.

Until those gates close, use layered replay for implementation verification and
controlled offline experiments only, not for a production-default decision or
whole-system efficiency claim.
