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
stamp always exists. The loss happens because **two independent workers each
sub-sample the stream with latest-wins frame-drop**, and they keep *different*
frames:

```
   rgbd_camera (color+depth, shared stamps, ~30 Hz nominal)
        │
        ├──▶ detector          latest-wins, capped detector_fps=5  ──▶ detections at stamps {D}
        │        (g1_detector_node.py: grab latest color, set None)
        │
        └──▶ aligned_depth_node latest-wins, uncapped              ──▶ aligned depth at stamps {A}
                 (aligned_depth_node.py: grab latest depth, set None)

   mask node needs  T ∈ {D} ∩ {A}   (exact)   ──▶  coverage = |{D} ∩ {A}| / |{D}|
```

`{D}` and `{A}` are two independent thinnings of the same stamp grid. Their
exact-stamp overlap is small, so most detections land on a stamp for which
`aligned_depth_node` happened to drop the depth frame → `NO_DEPTH_FRAME`.

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
- Alignment happens in the driver, so `aligned_depth_node`'s stereo source is
  again a trivial unit-convert-and-restamp.

With the producer keeping up, `{A}` ≈ the full stamp grid, `{D} ⊆ {A}`, and
exact-stamp coverage approaches 100 %. **Exact-stamp matching is the correct
choice there** — it is only pathological when a starved producer thins `{A}`.

(The monocular Depth-Anything source, `aligned_depth.md` §3, *is* a genuine
per-frame NN cost and would thin `{A}` on any host; it is a separate axis from
this sim-contention finding.)

---

## 6. Options considered (no decision made)

Recorded for a later call; **nothing is implemented**.

1. **Fold the stereo/driver-aligned decode into the consumer.** Because the
   decode is ~2 ms, the mask node could buffer the *raw* (or driver-aligned)
   depth by stamp and decode on-demand at the exact detection stamp, inside the
   worker it already runs per detection. That removes the second independent
   sub-sampling stage entirely, so the depth for whatever frame the detector
   chose is essentially always present. Monocular stays a producer (real NN
   bottleneck). Faithful on real hardware. Cost: a real refactor touching the
   mask depth path, the `depth_source` branch, launch wiring, and tests.
2. **Raise the `aligned_depth_node` process priority** (`nice`/`chrt` in the
   launch) so its executor drains the subscription under contention. Small and
   sim-only; partial, and does not address the two-sub-sampler divergence.
3. **Accept and document as a sim artifact** (this file). Zero code; justified
   by §5.

Profiling (§4) is what justifies option 1 — a separate always-on process buys
nothing for the stereo path but the sub-sampling penalty — but the trade
against refactor cost is left open.

---

## 7. Cross-references

- `aligned_depth.md` — the aligned-depth contract and its two producers; §1
  "Timing" bullet is the invariant this document stresses.
- `projective_ranging.md`, `euclidean_reconstruction.md` — the two consumers
  that report `NO_DEPTH_FRAME` together.
- `polar_profiling.md` — the LiDAR path, at 100 % here because it matches the
  scan within a tolerance window rather than on an exact stamp.
- `object_localization_pipeline.md` — the benchmark and its per-estimator
  miss-reason attribution, which surfaced this.
