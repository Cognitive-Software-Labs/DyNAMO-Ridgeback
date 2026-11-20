# Full-matrix Isaac repeatability pair

Recorded dates: 2026-11-06
Tested revisions: `cc86eef5a5c708be2a31aac64db29a91225f4023`
Provenance: partial

Correction added 2026-11-20, based on the investigation recorded 2026-11-11 in
`docs/plans/handoffs/BENCHMARKING_isaac_leftover_g1_findings.md`, section 5:
that investigation attributes this pair's between-session mask-estimator
differences to stale Fabric G1 geometry and its shadows. The tables below
remain observations of a contaminated pair, not measurements of intrinsic
renderer or monocular-model variation. The original "trigger unidentified"
wording records the state of knowledge on 2026-11-06; this correction does not
claim a newly reproduced diagnosis. Polar differences were attributed
separately to RTX LiDAR noise.

## Question and result

The [two-scene pair](2026-11-05-monocular-readiness-and-isaac-repeatability.md)
matched to 4.732 mm but did not cover the full scene and mask matrix. This
experiment asked whether two independent fresh Isaac sessions produce
approximately matching final outputs across all four conditions.

Execution succeeded and collection was complete. Agreement depended on the
depth source:

- Box/stereo projective and Euclidean estimates were bit-identical in all 115
  paired rows.
- Silhouette/stereo projective and Euclidean estimates differed by at most
  4.3 mm.
- Polar estimates differed by a median of about 2 mm and at most 28.5 mm in
  every condition.
- Monocular mask estimators had a median difference of 1 to 3 mm, but a heavy
  tail reaching 1.376 m. The two-scene result does not generalize to them.

No scientific acceptance margin was agreed, so this record declares no
scientific pass or failure. The 20 mm and 1% figures below are operational
review triggers.

## Conditions and provenance

Each session ran box/stereo, silhouette/stereo, box/monocular and
silhouette/monocular, in that order, inside one persistent simulator. Each
condition used 88 scenes expanded to 109 trials and 135 target rows, a 2-second
settle, fixed 10-second windows and five stored raw batches per window. Both
used 640×480, the furnished lab with props, OWLv2 at threshold 0.30 with a
10 FPS ceiling, deterministic PathTracing with AA 0, ideal depth for stereo,
Depth Anything V2 Metric Indoor Small for monocular, and GUI, video, overlays
and diagnostics enabled.

Workstation-local evidence is under
`artifacts/benchmarks/isaac_repeatability_prepared_20261105T234856Z/`: source
snapshots, `workspace.patch`, `frozen.json`, model references, sweeps, stage
logs, typed captures, CSVs, `comparisons.json`, `REPORT.md`,
`RECOVERY_LEDGER.json` and, in `analysis/`, the scripts behind this record.
The tested state was the revision above plus that patch, frozen for the whole
pair. Provenance is partial because the complete binary and dependency
environment was not locked.

An earlier bundle, `isaac_repeatability_prepared_20261105T210614Z`, stopped in
readiness when a 240-second stage cap refused the third condition. No full run
began. Its 136.5 seconds were charged to this campaign. The tested
implementation raises the readiness cap to 900 seconds with a 120-second
admission allowance. It also uses 16,200-second blocks and 45/55-minute Isaac
admission and timeout limits. Measured stages show the larger block limits
were not needed.

## Execution and completeness

| Stage | Exit | Wall (s) | Owned leftovers | External CUDA samples | Minimum available RAM (GiB) |
| --- | ---: | ---: | --- | ---: | ---: |
| Readiness | 0 | 236.6 | none | 0/23 | 76.2 |
| Session r1 | 0 | 6512.9 | none | 0/646 | 65.0 |
| Session r2 | 0 | 6524.2 | none | 0/647 | 65.3 |

The live phase took 13,277 seconds; 29,359 of the 42,772 budgeted seconds
remained. All three readiness conditions passed, and both monocular gates
opened. Neither session log contains a traceback.

All 872 full windows were complete, lasting 10.0006 to 10.0045 seconds with 77
to 98 events. Every condition stored 545 batches, 4,360 in total, with no
sampled event lacking a live record and no incomplete or under-sampled trial.
Per-condition events were 10,094 to 10,241. Paired captures differed by a mean
of 0.7 to 1.0 frames and at most 4. Raw batch and event counts differed by one
in 272 windows.

Ten scenes had no usable events in every one of the eight runs, including
`objocc_heavy_01` to `04`, `objpartial_02`, `objpartial_03` and the
`edge640_*` scenes. Those sampled were detector misses, so this is a stable
scene property. Misses are retained in the row counts below.

## Output differences

Pairs match condition, estimator, `trial_id` and `instance_index` in the
full-window CSVs. Recomputing from those CSVs reproduced `comparisons.json`
exactly.

| Condition | Estimator | Paired | Bit-equal | Median (mm) | P95 (mm) | Max (mm) | Trigger rows | Changed outcomes |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| box/stereo | projective | 115 | 115 | 0 | 0 | 0 | 0 | 0 |
| box/stereo | Euclidean | 115 | 115 | 0 | 0 | 0 | 0 | 0 |
| box/stereo | polar | 103 | 0 | 2.05 | 7.68 | 16.7 | 0 | 0 |
| silhouette/stereo | projective | 117 | 38 | 0.012 | 0.27 | 1.1 | 0 | 0 |
| silhouette/stereo | Euclidean | 117 | 53 | 0.005 | 0.41 | 4.3 | 0 | 0 |
| silhouette/stereo | polar | 101 | 0 | 1.71 | 7.50 | 28.5 | 1 | 0 |
| box/monocular | projective | 109 | 3 | 2.17 | 475.2 | 1376.3 | 19 | 0 |
| box/monocular | Euclidean | 104 | 4 | 1.17 | 123.7 | 478.8 | 14 | 2 |
| box/monocular | polar | 102 | 0 | 1.88 | 7.48 | 18.1 | 0 | 1 |
| silhouette/monocular | projective | 100 | 4 | 2.50 | 120.6 | 247.6 | 11 | 0 |
| silhouette/monocular | Euclidean | 99 | 4 | 2.74 | 123.2 | 245.0 | 14 | 0 |
| silhouette/monocular | polar | 101 | 0 | 2.13 | 8.11 | 12.2 | 0 | 0 |

The monitor alert named only the changed outcomes for box/monocular Euclidean,
because it reports one line per estimator. The 14 trigger rows above come from
this recomputation.

Three rows changed outcome, all in box/monocular:

- `multi_04` Euclidean scored one target in each session, but a different one:
  instance 0 in r1 (4.694 m, truth 3.98 m) and instance 1 in r2 (4.407 m,
  truth 3.32 m). The other instance was a gate miss. Boxes matched.
- `interfere_infront_04` polar scored 2.678 m in r1 and returned
  `TOO_FEW_RAYS_MERGED` in r2. The box top edge moved by one pixel.

Across the monocular mask estimators, 58 of 412 pairs exceeded 20 mm, 32
exceeded 100 mm and 2 exceeded 500 mm. The largest differences were:

| Scene | Estimator | r1 (m) | r2 (m) | Truth (m) | Delta (mm) |
| --- | --- | ---: | ---: | ---: | ---: |
| `trap_lidar_table` | box projective | 7.114 | 5.738 | 4.10 | 1376 |
| `far_mid_02` | box projective | 9.770 | 10.281 | 7.40 | 510 |
| `single_facing_03` | box Euclidean | 3.877 | 3.398 | 2.65 | 479 |
| `far_mid_01` (8 repeats) | box projective | 10.643 | 10.168 | 7.96 | 475 |
| `interfere_infront_02` | box Euclidean | 2.696 | 2.443 | 2.66 | 253 |
| `objpartial_06` | box Euclidean | 6.453 | 6.201 | 4.07 | 252 |

Repeated scenes count once per repeat: the 19 box/monocular projective trigger
rows cover 12 distinct scenes. Trigger rows spread across 11 to 14 scenes in
the other three monocular mask pairs. Mean absolute error moved by at most
45 mm between sessions, from 1.489 to 1.444 m for box/monocular projective.

## What differed between the sessions

RGB was compared by hashing the first stored frame of each window. Only the
first five batches of a window are stored, about 0.17 s of simulated time, so
later frames are unobserved. Those starts still represent the mask results:
full-window and first-five estimates agree within 6.2 mm on every mask row
except `single_facing_03` box/monocular Euclidean. Its 479 mm difference is
absent from the first five batches, so it arose later in a window and the
stored frames cannot explain it.

| Condition | RGB byte-identical between sessions | Same detector boxes (first event) | Depth byte-identical where stored |
| --- | ---: | ---: | ---: |
| box/stereo | 109/109 | 109/109 | 99/99 |
| silhouette/stereo | 22/109 | 96/109 | 89/99 |
| box/monocular | 3/109 | 95/109 | 3/99 |
| silhouette/monocular | 3/109 | 95/109 | 3/99 |

Where RGB differed, the difference was tiny and discrete, not continuous noise.
The median was 0.0044 of 255 levels with 0.26% of pixels changed, and the
largest trial mean was 0.143 levels. Each trial showed only 1 to 3 distinct
images across all eight runs (3, 87 and 19 trials respectively).

The change followed session history. The first condition of both sessions
matched on every trial. In r2, the last three conditions were byte-identical
to each other on all 109 trials and matched box/stereo on 91. The 18 trials
that differed were the same ones each time, all among the first 19 in run order.
In r1, the last two conditions matched each other on 108 trials, but only 3 or
4 trials of any later condition matched box/stereo. The trigger for this state
change was not identified. On the three trials where monocular RGB matched
between sessions, monocular mask estimates were bit-equal.

Within a session, the first frames of the eight `far_mid_01` and
`single_facing_01` monocular repeats were byte-identical, and all mask
estimators had zero spread across every repeated scene in all four conditions.
The same `far_mid_01` difference therefore appears eight times.

Polar is a separate mechanism. LiDAR scans differed in all 109 box/stereo
trials, by a median of about 44 mm per beam, and changed inside every window
checked, while RGB and boxes matched exactly. Polar's within-session repeat
spread (4 to 19 mm) exceeds its between-session difference. Its roughly 2 mm
median is sensor noise surviving aggregation, not session state.

## Why small differences became large

Box/monocular projective selects a near surface with
`nearest_significant_mode`: the nearest 50 mm bin holding at least 5% of the
box's depth samples. A small depth change can move a bin across that floor and
jump the estimate to another surface. Rerunning the function on stored depth
reproduced this:

- `far_mid_01`: same box, mean depth change 2.6 mm. The 9.97 m bin held 4.9% in
  r1 and 5.06% in r2, so the selected peak moved from 10.42 to 9.97 m.
- `trap_lidar_table`: same box. A near bin around 5.5 m held 4.9% in r1 and
  5.2% in r2, moving the peak from 6.75 to 5.47 m.
- `far_mid_02`: no bin reached 5% (the largest held about 2%), so the
  nearest-nonempty fallback applied, and the box bottom moved 5 pixels.

Over all 19 box/monocular projective trigger rows, the first-event peak moved
by at least one bin in 17, and 14 had the same first-event box. Detector box
changes therefore do not explain most of them. The monocular model turns
sub-level RGB differences into depth differences, and this threshold can then
amplify those into surface jumps. Stereo depth does not take RGB as input, which
is consistent with its near-zero differences despite the same RGB states.

## Limits

- Two sessions give one between-session difference per target, not a variance.
  Larger differences may exist.
- Within-session repeats do not measure this variation. They saw identical
  inputs and reported zero spread for scenes that differed between sessions.
- Condition order was fixed, so condition type is confounded with position in
  the session. Box/stereo always ran first and monocular never did, so the
  matrix cannot separate the two effects.
- The peak analysis covers box/monocular projective on the first stored event,
  with a reimplemented valid-pixel filter. Silhouette masks were not stored
  and Euclidean selection was not rerun, so the mechanism is inferred, not
  tested, for those rows.
- RGB beyond the first five batches of a window is unobserved.
- This measures repeatability, not accuracy: monocular mask MAE is about 1.4 to
  1.7 m, against 0.07 to 0.24 m for the stereo and polar rows.
- The offline replay phase did not run, so `REPORT.md` has no replay parity
  rows. No estimator or parameter was changed in response to these results.
