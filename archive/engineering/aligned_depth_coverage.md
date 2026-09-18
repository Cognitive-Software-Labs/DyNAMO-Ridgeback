# Aligned-depth stamp coverage investigation

Recorded dates: 2026-07-24, 2026-08-26

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Conclusion

The July 2026 simulation investigation found independent detector and depth
producer sampling selected different stamps. The August 26 decision moved raw
depth buffering and on-demand conversion into the mask consumer. This removed
one sampling stage; it did not establish lossless ROS delivery.

## July 24 observations

The box-gate smoke used one repeat, 15 trials and 4 seconds of capture per
trial in `target_distance_calibration`.

| estimator | OK | miss | coverage | reason |
|---|---|---|---|---|
| projective ranging | 22 | 89 | 19.8 % | `NO_DEPTH_FRAME` ×89 |
| euclidean reconstruction | 22 | 89 | 19.8 % | `NO_DEPTH_FRAME` ×89 |
| polar profiling | 111 | 0 | 100 % | — |

Source color/depth rates were about 3.0/2.7 Hz; the separate aligned-depth
producer published about 1.0 Hz. Offline decode/encode took 0.056 ms/frame;
live produce/publish took about 2 ms. Worker wake rates ranged 0.16–4.48/s,
averaging about 1.4/s. The worker generally processed each wake, suggesting
loss before its compute stage. Those counters alone did not distinguish all
transport and scheduling causes.

```text
RGBD source: paired color/depth stamps
  ├─ detector sampling       → requested stamps D
  └─ separate depth producer → available stamps A
Exact-match observations required membership in D ∩ A.
```

Depth was indexed on the mask's color grid at the detection stamp. Substituting
another frame could move foreground pixels onto background under motion. The
investigation therefore kept exact depth matching; the independently clocked
scan used a 50 ms nearest-stamp tolerance.

## August 26 decision and limits

Decode was cheap enough to perform on demand. Consumer-side raw buffering
avoided predicting which frame the detector would later request. Raising the
producer priority addressed contention only; tolerating the low coverage left
the independent-sampling mechanism in place.

The August 25 silhouette/stereo baseline had 12/135 valid observations per
depth estimator and 85 `NO_DEPTH_FRAME` observations. No immediate matched
post-change run was recorded. The subsequent
[TF investigation](refactor_validation.md#resolution-of-the-coverage-item-2026-08-31)
and [DDS comparison](exact_stamp_depth_availability.md) addressed separate losses.
Their results must not be attributed solely to removing the producer.

## Recorded monocular metric-scale check

On July 24, pixel-wise monocular/stereo depth ratios on identical simulation
frames had median 1.02 over the G1 region and 0.87 over the whole frame.
The object ratio was 1.10–1.15 at 2–3 m and approached 1.0 at 4–6 m.
This supported range-dependent warping rather than a single scale correction.
The original record supplied no exact run identifier. These observations
preceded the D455 geometry correction and did not validate hardware depth.

## Evidence limitations

The original July trace and the monocular check have no recorded tested commit.
Rates and costs describe that host and workload; hardware behavior was untested.
