# Aligned Depth — Stamp-Matching Coverage in Sim

**Scope:** why the two depth-based paths (`projective_ranging.md`,
`euclidean_reconstruction.md`) report *no estimate* on most frames in
simulation, even though depth is nominally available. This is a **timing /
coverage** failure mode of the aligned-depth contract (`aligned_depth.md` §1,
the "Timing" bullet), not an accuracy problem. The measurements below were
taken 2026-07-24 on the `g1_distance_calibration` world.

The symptom surfaced through the benchmark's per-estimator miss-reason
attribution (`object_localization_pipeline.md`): a box-gate run tallies the
depth rows almost entirely as `NO_DEPTH_FRAME`.

> **Resolved 2026-08-26 by option 1 (§6).** Depth acquisition now happens
> inside `g1_mask_measurement_node`, at the detection stamp; there is no
> depth producer process and no depth topic. Sections 1–5 describe the
> topology that caused the loss and are kept as the diagnosis that motivated
> the fix. §6 records the decision.

---

## 1. The symptom

A box-gate smoke run (all three mask estimators, 1 repeat = 15 trials,
`capture_sec=4`) produced this coverage tally over the captured single-object
events:

| estimator | OK | miss | coverage | reason |
|---|---|---|---|---|
| projective ranging | 22 | 89 | 19.8 % | `NO_DEPTH_FRAME` ×89 |
| euclidean reconstruction | 22 | 89 | 19.8 % | `NO_DEPTH_FRAME` ×89 |
| polar profiling | 111 | 0 | 100 % | — |

Two things stand out:

- The two depth paths share the **identical** histogram — expected, since they
  consume the same aligned depth frame and fail together whenever it is absent.
- Polar profiling (LiDAR) is at **100 %**. Same trials, same alignment window —
  so this is not "the sim dropped the robot from view." It is specific to how
  depth is matched to the detection.

`NO_DEPTH_FRAME` is stamped in `fill_path_measurements`
(`g1_mask_measurement_node.py`) when the depth lookup for a detection returns
`None` — i.e. no aligned depth frame carried that detection's exact stamp.

---

## 2. The matching contract

The mask node matches depth to a detection by **exact stamp, no tolerance**
(`g1_mask_measurement_node.py`, `StampedMessageBuffer.lookup`):

```
depth_msg = self.depth_buffer.lookup(detections_msg.header.stamp)   # exact key, or None
scan_msg  = self.scan_buffer.lookup_nearest(stamp, tolerance_s)     # nearest within 50 ms
```

This asymmetry is deliberate (the code comments it as audit C3):

- **Depth is pixel-aligned to the mask.** The mask is rasterized on the color
  grid at stamp `T`. Pairing a depth frame from `T±Δ` would index the depth of
  wherever the target *was* at `T±Δ` with a mask drawn where it was at `T`.
  Under motion those disagree, and the path silently reads background pixels.
  So depth demands an **exact** match — a miss is preferred over a smear.
- **The scan is angle-selected, not pixel-aligned**, and free-runs at ~40 Hz on
  its own clock, so it matches to the nearest stamp within
  `SCAN_MATCH_TOLERANCE_S` (0.05 s). That tolerance is why polar is at 100 %.

The contract itself is sound. The problem is that in sim the set of stamps that
*have* an exact depth match is small.

---

## 3. Why the exact-stamp set is small in sim

The RealSense is a single `rgbd_camera` sensor
(`intel_realsense.urdf.xacro`), so color and depth are rendered together and
**share a stamp** — at the source, a depth frame with the detection's exact
stamp always exists. The loss happened because **two independent processes each
sub-sampled the stream**, and they kept *different* frames:

```
   rgbd_camera (color+depth, shared stamps, ~30 Hz nominal)
        │
        ├──▶ detector          latest-wins slot + detector_fps=5 cap  ──▶ detections at stamps {D}
        │        (g1_detector_node.py: grab latest color, set None)
        │
        └──▶ aligned_depth_node                                       ──▶ aligned depth at stamps {A}
                 thinned mostly at the rmw/executor layer (§4), not by
                 its latest-wins slot, which is rarely contended

   mask node needs  T ∈ {D} ∩ {A}   (exact)   ──▶  coverage = |{D} ∩ {A}| / |{D}|
```

`{D}` and `{A}` are two independent thinnings of the same stamp grid. Their
exact-stamp overlap is small, so most detections land on a stamp for which
`aligned_depth_node` never received the depth frame → `NO_DEPTH_FRAME`.

The deeper point is that the two thinnings cannot be reconciled: `T` is decided
downstream, in a process `aligned_depth_node` never talks to. A producer
optimizing for **freshness** (keep the newest frame, drop the backlog) cannot
serve a consumer that needs **specificity** (the one frame the detector used,
already ~200 ms old by the time detections arrive).

### Measured rates (same run, `ros2 topic hz`)

The sim is render-bound, so nothing runs at the nominal 30 Hz:

| topic | rate |
|---|---|
| color / raw depth (source) | ~3.0 / ~2.7 Hz |
| **aligned depth (`{A}` producer)** | **~1.0 Hz ← bottleneck** |

`aligned_depth_node` emits only ~1 of every ~3 source frames. A detection's
stamp matches only when the producer kept that same frame — roughly the
producer's duty cycle — which lines up with the observed ~20 % coverage.

---

## 4. Profiling: the producer is starved, not slow

The obvious guess — the stereo "alignment" is expensive — is wrong. The sim
stereo source does no reprojection; it decodes the co-registered gz depth to
metric float and passes the header through (`StereoDepthSource.produce` →
`decode_depth_to_meters` → return `msg.header`). Profiled:

| measurement | value | reading |
|---|---|---|
| decode + encode, offline microbench | **0.056 ms/frame** (~18 k fps ceiling) | compute is free |
| produce + publish, live under sim load | **~2 ms/frame** | still trivial |
| worker `wake ≈ produced` every sample | yes | not dropping internally; the latest-wins slot is rarely contended |
| worker **wake rate** | **0.16–4.48/s, avg ~1.4/s** | it is only *notified* of ~1.4 frames/s |

The worker produces every frame it wakes for, in ~2 ms, but only **wakes
~1.4×/s** against ~2.7 Hz of published depth. Frames are therefore dropped at
the **rmw/executor reception layer**, because the `aligned_depth_node` process
is **CPU-starved** by the saturated sim host (gz render + physics + the YOLO
detector + the mask worker + bridges). The wide swing (0.16/s in a starved
slice, 4.48/s in a catch-up burst) is the scheduling-contention signature.

Consequences for any fix:

- There is **no decode/reproject hotspot to optimize** — it is already ~2 ms.
- A deeper subscription queue will not help: the node consumes (~1.4/s)
  persistently *slower* than depth is published (~2.7/s); a queue only smooths
  bursts, not a standing rate deficit.

---

## 5. Real hardware: the problem largely evaporates

On a real robot this failure mode is mostly a sim artifact:

- A RealSense publishes hardware-synchronized color + `aligned_depth_to_color`
  at a true 30 fps with **shared stamps**, on dedicated compute that is not
  contending with a renderer and a physics engine.
- Alignment happens in the driver, so the stereo source is again a trivial
  unit-convert-and-restamp.

With the producer keeping up, `{A}` ≈ the full stamp grid, `{D} ⊆ {A}`, and
exact-stamp coverage approaches 100 %. **Exact-stamp matching is the correct
choice there** — it is only pathological when a starved producer thins `{A}`.

(The monocular Depth-Anything source, `aligned_depth.md` §3, *is* a genuine
per-frame NN cost and would thin `{A}` on any host; it is a separate axis from
this sim-contention finding.)

---

## 6. Options considered — option 1 chosen (2026-08-26)

1. **Fold depth acquisition into the consumer.** ← **chosen and implemented.**
   Because the decode is ~2 ms, the mask node buffers the *raw* (or
   driver-aligned) depth by stamp and converts on demand at the exact detection
   stamp, inside the worker it already runs per detection. That removes the
   second independent sub-sampling stage entirely, so the depth for whatever
   frame the detector chose is essentially always present.
2. **Raise the `aligned_depth_node` process priority** (`nice`/`chrt` in the
   launch) so its executor drains the subscription under contention. Small and
   sim-only; partial, and does not address the two-sub-sampler divergence.
3. **Accept and document as a sim artifact** (this file). Zero code; justified
   by §5.

Profiling (§4) is what justified option 1: a separate always-on process buys
nothing for the stereo path but the sub-sampling penalty. Two pieces of
in-repo evidence said it was safe — the mask node already does exact-stamp
lookup on the *color* topic for the silhouette gate, in the same process on
the same saturated host, missing 1 frame in 135; and the legacy
`depth_anything` estimator already computes depth inside its consumer, with a
full NN forward pass, and scores 130/135.

### What was built

- `perception/core/depth_sources.py` holds both sources unchanged
  (`StereoDepthSource`, `MonocularDepthSource`) plus a `build_depth_source`
  factory. Monocular moved in with stereo rather than staying a producer: the
  same specificity argument applies to it, and running the NN once per
  detection batch (5 Hz) is not obviously worse than once per camera frame.
- `g1_mask_measurement_node` subscribes to the source's *input* stream
  (`input_kind`), buffers it raw in the `StampedMessageBuffer` it already
  owned, and converts only the frame at the detection stamp. Monocular reads
  the same color buffer the silhouette gate uses — one buffer, two readers,
  one stamp.
- `aligned_depth_node` is deleted, with the `max_fps` cadence cap (a
  demand-driven path has no cadence) and the producer-side
  `camera_info_matches_depth` check (the consumer's `grid_mismatch_warning`
  covers it). The aligned-depth **artifact** contract is unchanged; only its
  transport is, from a topic to a call at stamp `T`. It is still published on
  `debug/g1/mask/aligned_depth` for the overlay panel, encoded only when
  something is subscribed.

### Measured effect

| | before (`20260825_203145`, silhouette / stereoscopic) | after |
|---|---|---|
| projective ranging | 12 / 135 (8.9 %), `NO_DEPTH_FRAME` ×85 | not yet re-run |
| euclidean reconstruction | 12 / 135 (8.9 %), `NO_DEPTH_FRAME` ×85 | not yet re-run |
| polar profiling | unaffected (never depended on depth) | — |

Expected after: roughly 97/135 for the two depth rows. The residual `UNSET`
×37 in that run is a **different** defect — the mask node's own latest-wins
detections slot drops backlog — and should be unchanged by this work.

---

## 7. Cross-references

- `aligned_depth.md` — the aligned-depth contract and its two sources; §1
  "Timing" bullet is the invariant this document stresses.
- `projective_ranging.md`, `euclidean_reconstruction.md` — the two consumers
  that report `NO_DEPTH_FRAME` together.
- `polar_profiling.md` — the LiDAR path, at 100 % here because it matches the
  scan within a tolerance window rather than on an exact stamp.
- `object_localization_pipeline.md` — the benchmark and its per-estimator
  miss-reason attribution, which surfaced this.
