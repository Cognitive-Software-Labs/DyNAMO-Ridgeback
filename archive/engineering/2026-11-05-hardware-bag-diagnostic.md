# Saved hardware-bag diagnostic and review

Recorded dates: 2026-11-04, 2026-11-05
Tested revisions: `1a96754918cf3d6c7920a0c9994be64b4907c8db`
Provenance: partial

## Conditions and provenance

The tested checkout included the uncommitted hardware-bag bridge in addition
to the named base revision. That base commit alone does not reproduce this run.
The frozen job, tracked patch and source snapshot below identify the executed
implementation; this record's later documentation commit is not a tested run.

The raw session was
`/home/stefi/ScenesForDetectionTesting/20261029T141724Z-distance-benchmark-raw`.
Closeout selected 48 capture-complete arrangements from 50 attempts, excluding
`01_P01-03_attempt1` and `25_P01-31_attempt1`. All source attempts were retained.
Aliases were not additional physical trials; P04/P06 shared one near-range bag.

Processing used CPU/float32, eight OMP/MKL threads, OWLv2 base ensemble with
prompt `humanoid robot`, threshold 0.30, and deterministic nominal 10 Hz source
sampling. Each bag supplied 200 selected frames. The same detections were
shared across the ten paths below. Pointcloud was excluded. All derivatives
explicitly used hardware `front_offset_m = 0.465` m. The job retained pinned
OWL, SlimSAM and metric indoor Depth Anything snapshots and resolved settings.
Monocular depth retained the implementation's 18 m usable ceiling; its 0.9
range fraction was not validated by this experiment.

Execution finished with exit 0 at 2026-11-05 07:54:17 UTC, about eight hours on
CPU. This establishes no live throughput, autonomy or final accuracy result.

Local evidence under `artifacts/`:

- `benchmark-jobs/hardware_bridge_20261105_ready_cpu.json`, job identity
  `591cef3af2d3e1cd64d0cccc016daf26eef3d782ad1855055d089977ef0fcd18`;
- `benchmarks/hardware_bridge_20261105_ready_cpu/`: source/inference caches,
  event results, overlays, per-bag summaries and completion hashes;
- `benchmarks/hardware_bridge_20261105_ready_cpu_control/`: launch settings,
  `exit-2.json`, log, `tracked.patch`, Git status and `sources/` snapshot;
- `benchmarks/hardware_bridge_20261105_review/`: `review.md`, `analysis.json`,
  configuration/detection/per-bag CSVs, sampled overlays, cached-depth samples
  and `integrity.json`.

These are host-local evidence paths, not bundled datasets or guarantees of
future availability. All 48 derivative receipt/hash trees passed revalidation.
Review accounted for every selected event and all ten path rows, checked
timestamp matching and coordinate consistency, and sampled middle-frame masks
for all bags plus first/middle/last RGB for eight detector-failure cases. It did
not exhaustively annotate target identity on all frames.

## Input and detector outcomes

Of 9,600 selected frames, 9,598 had exact stereo-depth matches. Missing depth
occurred at bag 08 event 144 (two boxes) and bag 39 event 68 (one box), producing
three `NO_DEPTH_FRAME` candidate outcomes in each stereo path. No selected
missing scan, calibration or transform context was reported. The missing-depth
events remained in denominators and unaffected paths continued.

| Detections per frame | Frames | Percentage |
|---|---:|---:|
| None | 1,501 | 15.64% |
| One | 7,451 | 77.61% |
| Two | 648 | 6.75% |

There were 8,747 boxes on 8,099 frames. This is detection presence, not
independently established G1 recall or precision. Six bags had no detections
throughout: 19 (foreground stands), 24 and 32 (heavy screens), 40 (clear distant
side profile), 44 (right sliver) and 48 (left partial). Bag 45 had 195 empty
frames, and bag 31 had 81. Sampled RGB was intact and consistent with those
difficult recorded conditions.

Sampled masks also covered false stand-hook detections in bags 08/16 and
separate G1 fragments around the table in bag 29. A numerical estimate for any
candidate does not prove successful localization of G1. Target association must
not select the candidate closest to the nominal distance reference.

## Numerical-return coverage

The frame denominator is all 9,600 selected frames. A positive frame has an
estimate for any candidate, including possibly a wrong object. Candidate counts
use the separate denominator of 8,747 boxes. These are not accuracy scores.

| Estimator | Mask | Input | Frames with any estimate | Candidate estimates |
|---|---|---|---:|---:|
| Projective | Box | Stereo | 8,097 | 8,744 |
| Projective | SlimSAM | Stereo | 8,097 | 8,744 |
| Projective | Box | Monocular | 8,099 | 8,747 |
| Projective | SlimSAM | Monocular | 8,099 | 8,747 |
| Euclidean | Box | Stereo | 8,097 | 8,744 |
| Euclidean | SlimSAM | Stereo | 8,097 | 8,744 |
| Euclidean | Box | Monocular | 8,099 | 8,747 |
| Euclidean | SlimSAM | Monocular | 8,099 | 8,747 |
| Polar | Box | Front scan | 7,620 | 7,707 |
| Polar | SlimSAM | Front scan | 7,701 | 7,788 |

Polar candidate misses comprised 843 `TOO_FEW_RAYS_SELECTED` in each mask mode,
plus 197/116 `TOO_FEW_RAYS_MERGED` for box/SlimSAM. Some belonged to extra
candidates rather than G1. No accepted candidate had an explicit mask rejection
or empty segmentation, which does not establish that its mask was correct.

## Distances and selection limitations

All comparisons below use provisional supplied references, not independently
qualified accuracy truth. No labels or geometry were fitted to outputs.

For clear baseline bag 02, nominal radial range was 2.866 m. Median estimates
across its 200 single-detection frames were:

| Estimator | Stereo box / SlimSAM | Monocular box / SlimSAM | Scan box / SlimSAM |
|---|---:|---:|---:|
| Projective | 3.009 / 3.001 m | 4.151 / 4.242 m | — |
| Euclidean | 3.005 / 2.993 m | 4.188 / 4.357 m | — |
| Polar | — | — | 2.869 / 2.850 m |

Cached middle-frame target-mask optical-depth medians were 3.182 m stereo and
4.395 m monocular. The disagreement was present before estimator reduction;
the causes of all reference/sensor/geometry differences were not established.

- Bag 28: box/stereo returned about 1.82 m versus silhouette/stereo 2.98 m,
  nominal 2.87 m, consistent with foreground cabinet selection. Polar returned
  values on 198/200 box frames but only 90/200 silhouette frames.
- Bag 26: both polar modes failed all 200 detected frames. The cabinet hid the
  lower target and scan returns did not provide sufficient selected target rays.
- Bag 36: polar box returned on 35/192 detected frames, silhouette on 192/192.
  A tighter mask could reject an obstructing pole, but was not uniformly better
  across other scenes.
- Bag 38: polar box median was 4.044 m versus silhouette 3.578 m, nominal about
  3.528 m; return selection mattered despite an `OK` status.
- Bag 50: stereo medians were about 1.00–1.01 m, polar 1.62–1.63 m and
  monocular 1.68–1.72 m, versus nominal about 1.04 m. Other surfaces/support
  might contribute to scan estimates; exact beam attribution remained open.

## Decision and remaining limits

Retain all 48 recordings. Review identified no new need for re-recording,
repeated physical measurements or the operator beside the robot. Algorithm
failures are observations; recordings must not be changed until a method works.

References remained provisional, including eight assumed-unchanged post-turn
positions. The main ten-reading reference supported 30 bags. Its observed
repeatability envelope was not a bound on total systematic uncertainty. A
review of those existing measurements adds no independent physical information
and cannot establish whether they need repeating. Common reference uncertainty
does not shrink with the number of bags or adjacent frames. The marked pelvis
reference and estimator surface-selection semantics still require an explicit
evaluation definition before final error/ranking claims.

Remaining work is offline target annotation/association, a frozen metric and
comparison protocol, cached selection/depth investigation, then the agreed
tuning/ablation/final reporting stages. The maintained backlog owns that work.
Live hardware performance qualification is separate.
