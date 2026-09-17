# Isaac 6.1 LiDAR and SLAM qualification — 2026-09-17–18

Status: **PASS for the bounded LiDAR and closed-loop SLAM qualification.**
All twelve uncontended trials, the final fresh-boot capture, isolated
compensation check, and affected-package tests passed. Exploration/P5 and
shared-load timing qualification remain outside this result.

![Raw scan coverage and merged output](assets/2026-09-17-lidar/scan-coverage.png)

Regenerate this figure with `python3 tools/isaac/lidar_qualification.py plot
artifacts/isaac-lidar-qualification/20260918_final_capture_rounding/capture
--out docs/history/assets/2026-09-17-lidar/scan-coverage.png`.

The current interface is owned by the [LiDAR reference](../isaac/lidar-pipeline.md).
This record covers LiDAR/SLAM evidence only, not autonomous exploration,
Gazebo comparison, or the P5 coverage gate. The public default remains
`front_only`.

## Rig and protocol

Seating/map-height work landed independently at `ca215ccc`. The regenerated
analytic `mock_hospital` grid is 465×344 cells at 0.05 m resolution, sliced at
world Z=0.30257 m. Trials use clean Isaac Sim 6.1.0.0 boots, deterministic
timing, `rtf:=1.0`, camera/target localization/RViz/coverage overlay/frontier
motion disabled. LiDAR diagnosis used ROS domain 77, the isolated skew check
78, and the SLAM matrix 79 (initial) / 81 (resumed). Artifacts include dirty-tree provenance because
this qualification was measured before its deliverable commit.

## Coverage diagnosis and fix

The historical two-half-cloud configuration declared 270° scans but left
three 45° sectors empty in each raw scan during confirmed robot rotation.
A 3× internal rotary/firing-rate candidate did not change that failure.
Twenty consecutive clouds confirmed the narrow windows; runtime logging
confirmed the candidate attributes were actually loaded.

Positive market-frame bounds were also tested and produced no valid returns.
An unclipped 360° rotary control produced full-circle returns. The measured
failure is therefore in the configured clipped rotary path on this stack,
not a shortage of firing rate or a missing ROS topic. No claim is made here
that all NVIDIA clipped rotary configurations fail.

The retained implementation uses one full rotary cloud per sensor at the
original 40 Hz / 57600 firings/s and clips to ±135° in the ROS assembler.
No old scan bins or synthetic interpolation are used. The obsolete second
prim and half-cloud synchronization were removed.

## Live LiDAR evidence

The single-cloud run collected 500 matched triples during a 3.7237 rad
ground-truth yaw change. Every message passed its geometry/frame checks;
all three topics measured 40 Hz in simulation time.

| Gate | Observed result |
|---|---|
| Raw scans | 1081 bins, ±135°, 0.25°, 0.06–10 m |
| Merged scan | 1440 bins, -180° through +179.75°, `base_link`, maximum 10.3922 m |
| All six 45° sectors per raw sensor | Populated during controlled rotation |
| Unique attributable front / rear matches | 234165 / 230786 |
| Nearest-return collision matches | 153820 |
| Independent merge mismatches | 0, including finite masks |
| Finite ranges outside declared limits | 0 |
| Exact front/rear deltas | 500/500 zero nanoseconds |
| Cumulative paired / equal-stamp count | 975 / 975 |
| Compensated / front-only / rear-drop / TF-miss counts | 0 / 0 / 0 / 0 |

The observed Isaac pairs bypassed motion compensation because their stamps
were equal. This is a measured property of these runs, not a reason to remove
the skew-compensation path.

The final scan-versus-GT check measured 0.016 m median static error over 259 sampled
rays; the scan had 1033 finite bins at that pose. Rotation alignment ratios
were `[1.075, 1.003, 0.974, 1.061, 1.104, 1.119]`, consistent with content
rotating with the robot rather than the old angular-label stretching error.

The first provenance-rich capture (`20260918_final_capture/`) failed only
the recorder's 1 ns cadence tolerance: 479 intervals were exactly 25 ms,
ten were 25 ms minus 2 ns, and ten were 25 ms plus 2 ns, identically across
all topics, with no missing periods. The recorder now allows 10 ns rounding
(0.00004% of one period), with a regression rejecting 11 ns and lost periods.
Exact deltas remain recorded. A fresh boot in
`20260918_final_capture_rounding/` passed every gate; no sensor, merger, or
SLAM behavior was changed for this recorder fix.

## Independent namespaced-TF skew evidence

The test published only namespaced TF, with a 10 ms sensor skew and 0.01 rad
odom rotation. The rear ray survived at bin 1438, range 3.3922 m. A real 10 m
front sensor ray survived at bin 720, range 10.39220047 m, equal to the
float32 declared maximum. Counters showed one compensated pair, zero
equal-stamp pairs, zero front-only publishes, zero rear drops, and zero TF
misses. No global `/tf` publisher was present.

Startup delivery was checked separately: the merger now measures its rear
callback deadline from local receipt rather than the capture timestamp,
preventing already-queued startup scans from expiring immediately.

## Artifact index

Paths below are relative to the repository root; generated raw data remains
ignored and is not part of the commit.

- `artifacts/isaac-lidar-qualification/20260917T203037Z_live/retry3/`: original
  clipped-scan diagnostic captures.
- `artifacts/isaac-lidar-qualification/20260917T203037Z_live/fov_fix/` and
  `multitick/`: rejected 3×-rate candidate and consecutive-cloud diagnostic.
- `artifacts/isaac-lidar-qualification/20260917T203037Z_live/positive_bounds/`:
  rejected positive clipped-bound experiment.
- `artifacts/isaac-lidar-qualification/20260917T203037Z_live/unclipped/`:
  full-circle control.
- `artifacts/isaac-lidar-qualification/20260917T203037Z_live/single_cloud/`:
  passing 500-triple capture, raw NPZ, metadata, matched stamps, GT, logs,
  checksums, coverage plot, and scan-versus-GT check.
- `artifacts/isaac-lidar-qualification/20260917T211227Z_skew/`: passing actual
  maximum-ray namespaced-TF integration evidence.
- `artifacts/isaac-lidar-qualification/20260917_final_gt/`: regenerated GT.
- `artifacts/isaac-lidar-qualification/20260917_matrix/`: fresh-boot SLAM
  trials, overlays, logs, manifests, and host/GPU samples.
- `artifacts/isaac-lidar-qualification/20260918_matrix_resume/`: ten accepted
  remaining trials, preserving zero-before-configured-noise order.
- `artifacts/isaac-lidar-qualification/20260918_summary/`: complete 12-run
  metric summary and seed-paired deltas.
- `artifacts/isaac-lidar-qualification/20260918_final_capture_rounding/`:
  final 500-pair capture, per-message/raw evidence, launch log/arguments,
  GT, host samples, checksums, and geometry check.

## SLAM comparison

All four conditions completed 3/3 trials at seeds 0, 1, and 2, with finite
pose, loop, map, yaw-decomposition, correction, and duration metrics. No
invalidated July number or quality threshold is used as the baseline.

![Paired front-only versus merged results for all twelve trials](assets/2026-09-17-lidar/slam-comparison.png)

Each line pairs one seed, not an average. Regenerate with
`python3 tools/isaac/lidar_qualification.py summary-plot
artifacts/isaac-lidar-qualification/20260918_summary/slam_matrix_summary.json
--out docs/history/assets/2026-09-17-lidar/slam-comparison.png`.

Cells below are **median [minimum, maximum]** across three seeds. Deltas
are computed per seed as merged minus front, then summarized; they are not
the difference between the two condition medians. IoU/precision/recall are
fractions, duration is simulation time, and yaw values are RMS components.

### Zero odometry noise

| Metric | Front only | Merged | Paired delta |
|---|---|---|---|
| Pose RMSE (m) | 0.1735 [0.1563, 0.1864] | 0.0928 [0.0867, 0.1330] | -0.0635 [-0.0868, -0.0534] |
| Pose maximum (m) | 0.2827 [0.2689, 0.3178] | 0.1697 [0.1606, 0.2162] | -0.1016 [-0.1221, -0.0992] |
| Mean loop error (m) | 0.0417 [0.0327, 0.0489] | 0.0064 [0.0055, 0.0129] | -0.0288 [-0.0434, -0.0263] |
| Maximum loop error (m) | 0.0606 [0.0518, 0.0790] | 0.0151 [0.0150, 0.0229] | -0.0377 [-0.0639, -0.0368] |
| Map IoU | 0.5564 [0.4827, 0.5772] | 0.6672 [0.6443, 0.6995] | 0.0900 [0.0879, 0.2168] |
| Wall precision | 0.6486 [0.5789, 0.6576] | 0.7408 [0.7141, 0.7593] | 0.0832 [0.0655, 0.1804] |
| Wall recall | 0.8699 [0.8113, 0.8711] | 0.9493 [0.9177, 0.9508] | 0.0794 [0.0466, 0.1395] |
| EKF yaw RMS (deg) | 0.26 [0.26, 0.26] | 0.26 [0.26, 0.26] | 0 [0, 0] |
| SLAM yaw RMS (deg) | 0.73 [0.59, 1.05] | 0.36 [0.35, 0.41] | -0.32 [-0.70, -0.23] |
| TF correction events | 111 [109, 115] | 67 [58, 68] | -43 [-57, -42] |
| Maximum TF jump (m) | 0.2049 [0.1285, 0.2144] | 0.1377 [0.1355, 0.1459] | -0.0685 [-0.0694, 0.0092] |
| Drive duration (s) | 145.2 [145.2, 145.2] | 145.2 [145.2, 145.2] | 0 [0, 0] |

### Configured odometry noise (1.0)

| Metric | Front only | Merged | Paired delta |
|---|---|---|---|
| Pose RMSE (m) | 0.1834 [0.1306, 0.1845] | 0.1350 [0.1067, 0.1471] | -0.0495 [-0.0767, 0.0165] |
| Pose maximum (m) | 0.2931 [0.2403, 0.3047] | 0.2309 [0.1937, 0.2428] | -0.0622 [-0.1110, 0.0025] |
| Mean loop error (m) | 0.0286 [0.0271, 0.0545] | 0.0075 [0.0067, 0.0107] | -0.0211 [-0.0478, -0.0164] |
| Maximum loop error (m) | 0.0524 [0.0454, 0.0731] | 0.0144 [0.0136, 0.0192] | -0.0380 [-0.0595, -0.0262] |
| Map IoU | 0.5525 [0.5286, 0.6626] | 0.5940 [0.5804, 0.6330] | 0.0415 [-0.0822, 0.1044] |
| Wall precision | 0.6487 [0.6213, 0.7339] | 0.6815 [0.6548, 0.7078] | 0.0328 [-0.0791, 0.0865] |
| Wall recall | 0.8397 [0.8254, 0.9215] | 0.9226 [0.9070, 0.9374] | 0.0816 [0.0011, 0.0977] |
| EKF yaw RMS (deg) | 1.31 [0.91, 2.34] | 2.35 [0.72, 4.14] | 1.04 [-0.19, 1.80] |
| SLAM yaw RMS (deg) | 1.16 [1.15, 1.98] | 2.21 [0.89, 3.87] | 1.06 [-0.27, 1.89] |
| TF correction events | 171 [153, 176] | 155 [126, 157] | -19 [-27, -16] |
| Maximum TF jump (m) | 0.2600 [0.2233, 0.2914] | 0.1369 [0.1352, 0.1683] | -0.1231 [-0.1248, -0.0864] |
| Drive duration (s) | 145.2 [145.1, 145.2] | 145.2 [145.2, 145.2] | 0 [0, 0.1] |

### Interpretation

The LiDAR reference embeds the configured-noise seed-0 map/trajectory
overlays, copied without modification from
`20260918_matrix_resume/noise1_seed0_front_only/noise1_seed0_front_only_overlay.png`
and `20260918_matrix_resume/noise1_seed0_merged/noise1_seed0_merged_overlay.png`
under the artifact root. All twelve overlays remain available there; this
fixed pair illustrates the exception described below, not a best-run sample.

Merged improves pose RMSE, loop error, IoU, wall precision, and wall recall
in all three zero-noise seed pairs. With configured noise it reduces loop
error and increases wall recall in all three pairs, but seed 0 has worse
pose RMSE and IoU. The noisy EKF and SLAM yaw components are larger in two
pairs; their RMS values are not additive and do not alone measure total yaw
error. This is a small, closed-loop baseline, not proof of autonomous
exploration performance or universal merged superiority. Seeds control noise
streams, not bitwise determinism of the asynchronous ROS/SLAM stack.
Keep the public `front_only` default. A later default-change proposal should
include exploration/P5 evidence and investigate the noisy yaw tradeoff.

### Rejected attempts and provenance

### Half-speed timing pilot

An independent zero-noise, seed-0, merged-source pilot uses deterministic
steps with `rtf:=0.5`. `lidar_qualification.py timing-pilot --load quiet|shared`
labels its manifests `timing_pilot`; the qualification summarizer rejects
these runs. The normal matrix retains its idle-GPU and contention gates.
The shared case requires an already-observed competing GPU process, whose
presence throughout the run must be checked in the saved host samples.
A quiet/shared pair is a screening experiment, not statistical proof of
equivalent SLAM quality. The original two accepted `rtf:=1.0` trials are not
mixed with this pilot. A protocol change would require a consistent new matrix.

The quiet pilot completed successfully under
`artifacts/isaac-lidar-qualification/timing-pilot-quiet-run1/`:
achieved RTF 0.50, pose RMSE 0.1200 m, maximum pose error 0.2025 m,
map IoU 0.6599. Its final periodic merger diagnostic accounted for 5733
equal-stamp pairs, zero motion-compensated pairs, and zero front-only
publishes, rear drops, or TF misses. All saved GPU samples were available
and showed no competing compute process. The simulator exited cleanly.
The shared-load case remains unavailable: the collider task finished before
this pilot started, and no competing workload was present afterward.
These single-run results do not establish equivalence under contention.
Pilot harness regression checks: 23 passed, one ROS integration test
deselected; the required graph rebuild completed. An initial local test
invocation failed because ROS logging targeted the read-only home directory;
rerunning with an explicit writable `ROS_LOG_DIR` passed.

The zero-noise seed-0 front/merged trials completed without a second GPU
process in their saved host samples. A second Isaac process appeared during
seed-1 front-only (17 sampled intervals, first at 21:25:31 UTC) and seed-1
merged (30 sampled intervals, first at 21:27:03 UTC). The latter process was
identified as another task's `validate_chassis_contacts.py`. Both seed-1
attempts were rejected; this task's matrix was interrupted and its processes
stopped without touching the other task. Raw attempts and rejection reasons
remain in `20260917_matrix/acceptance-review.json`. The remaining ten runs
completed in `20260918_matrix_resume/`, with zero noise completed before
configured noise. All retained host samples have available GPU process
evidence and no competing compute process. The two retained older runs have
the same measured simulation behavior: the later `ros_io.py` difference is
comment-only, and qualification-tool changes affect evidence collection and
plotting rather than the SLAM controller or sensor behavior. Seating and
collider work landed separately and are not swept into this deliverable.

## Final verification

- `tools/check_dependencies`: all declared checkouts and recorded patches pass.
- Affected package builds pass; the installed copied merger matches source.
- Package-level `colcon test`: 58/58 test targets pass; scoped `colcon test-result`
  reports 1030 tests, zero errors, zero failures, zero skips. This includes the
  namespaced-TF/max-ray integration regression. Logs are under
  `artifacts/isaac-lidar-qualification/20260918_verification/`.
  The Isaac adapter package has no separate tests; its behavior is covered
  by the autonomy package's Isaac suites. A workspace-wide result query also
  found 22 old dependency XML-schema failures from September 12; these were
  not produced by the current affected-package test run.
- Public launch argument inspection retains `slam_source=front_only` and
  `noise_seed=0` defaults. Tests exercise generated source-dependent range
  YAML, Isaac-only seed forwarding, seed streams, geometry, range filtering,
  merger branches/startup pairing, and invalid-summary rejection.
- Required repository graph rebuild completed: 4124 nodes, 8737 edges,
  218 communities. Documentation image/link validation passes offline.
- Concurrent documentation cleanup landed separately in `6abbfdde`, including
  the preliminary LiDAR lifecycle-backlog cleanup and removal of the old port
  plan. Its shared-checkout changes are preserved; no history was rewritten.
- The visual-ownership convention from `701725cd` is followed: the contract
  diagram lives with the LiDAR reference, dated plots live with this record,
  and both are embedded in the LiDAR page. Raw logs/JSON/overlays remain in
  ignored artifacts. The completed active plan is removed; only exploration/
  P5 remains in the recertification backlog.
