# Published-cloud versus deprojected-depth experiment

Recorded dates: 2026-07-11

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Conclusion

On July 11, 2026, a controlled simulation comparison produced identical
estimates from published points and depth deprojected with matched intrinsics.
The decision was to use masked deprojection for Euclidean reconstruction.
The independent pointcloud estimator retained its published-cloud input.

## Method and results

The experiment held the reduction algorithm constant and changed only point
provenance: 100 synchronized frames at each of 1.5, 3.5 and 5.5 m, lateral
offset zero, target yaw π, 640×480 in `target_distance_calibration`.
It compared full-cloud parsing, full-frame deprojection, and masked-only
deprojection with a cached ray table.

| Metric | Published | Deprojected |
|---|---|---|
| median abs error vs truth (m) | 0.090 / 0.168 / 0.166 (per position) | identical — paired diff 0.000 on all 300 frames |
| median cloud diff RMS (m) | ~1×10⁻⁷ (shared metric; max 4.2×10⁻⁷) | float32 noise; x-axis diff exactly 0 |
| median consumer cost (ms) | 0.72 (parse) | 0.87 (full) / 0.076 (masked + ray table) |
| median msg size (bytes/frame) | 7 372 800 (24 B/point padded XYZ) | 1 228 800 (32FC1) — 6.0× smaller |
| topic bandwidth (MB/s, observed) | 13–21 | 1.4–3.4 |
| topic delay, stamp→arrival (ms, sim time) | 18 | 14 |
| producer CPU delta (%) | ≈ 0 — gz server flat at ~175 % with the points bridge alive or killed; rendering dominates, cloud assembly not measurable | n/a in sim (no align step) |

The fitted intrinsics were fx=fy=443.53, cx=319.50, cy=239.50, compatible with
the simulated sensor and bridged CameraInfo. An earlier concern about using
the hand-written 87° camera configuration did not describe the experiment's
actual deprojection inputs.

The sampled cloud used base-like XYZ despite its frame label; its raw-cloud
fallback was used on all 300 frames. That was an observation of the July
pipeline, not a contract for later clouds. The repeated accuracy errors arose
from the shared percentile reduction against model-origin truth.

Sim-time stamp-to-arrival deltas were measured inside the probe; the ordinary
wall-clock topic-delay command was unsuitable. Killing the point bridge did
not reveal a measurable producer-CPU benefit under renderer load.

## Decision and limitations

Masked deprojection avoided parsing the larger cloud, supported monocular
depth, and selected points directly on the color mask grid. The proposed
hardware published-versus-deprojected comparison was cancelled in favor of
validating aligned depth itself. No hardware equivalence was established.

The probe `~/tmp/cloud_provenance_test.py` was never committed. Its recorded
CSV paths were `~/tmp/cloud_provenance_d{1p5,3p5,5p5}.csv`; availability has not
been re-established. The tested revision was not recorded. Static targets,
noise-free rendered depth, and geometry predating the August 31 D455 correction
limit the conclusions. The measured costs are not present-day performance claims.
