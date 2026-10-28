# UST-10LX Gazebo LiDAR parity and Isaac scan-noise measurement

Recorded dates: 2026-10-25
Tested revisions: `5e65a0bd3c904a7676ee5042b4a3138eeec11964`, `33e4b311dd9a2dcaa8e8d262ae75fab2c53e560b`, `39fb8c1a5ba812367dc50bc025cd51596fcac54f`
Provenance: complete

## Configuration and evidence boundary

The root revision above is the base. `33e4b311` is the pinned
`clearpath_common`. The Isaac noise sessions ran on the clean base tree
(`git status --porcelain` empty) with only the existing
`clearpath_realsense_sim_frames.patch` applied. Every Gazebo session ran on
the base tree plus the uncommitted change this record accompanies. The UST-10LX
dependency patch `clearpath_ust10lx_sim_lidar.patch`, SHA-256
`a614b5174dcf2016faf141f500114a5ab6bd6859cdadc0f9d6941531e3247fb3`, was stacked
on the realsense patch, together with the root checker, contract and test edits.
The Gazebo runtime inputs (generator, macro, contract constant) did not change
between those sessions and the commit. Later edits touched only documentation
and test registration.

Host: RTX PRO 6000 Blackwell, NVIDIA GLX through `tools/gpu-run`, ROS 2 Jazzy,
CycloneDDS with the packaged profile, ROS domain 53, a unique `GZ_PARTITION`
per Gazebo session, Isaac Sim 6.1 headless, gz-sensors 8.2.2. Every launch was
driven on a PTY and stopped with Ctrl-C. The evidence root is
`artifacts/benchmarks/ust10lx_20261025_base_5e65a0bd/`. It holds the drivers,
recorders, analysis scripts, raw scans (`*_scans.npz`, `*_short_range.npz`),
the rendered descriptions before and after the patch, and the dependency
proofs and lifecycle summaries.

## Why the simulators disagreed

C5's matched full runs on the base revision used one robot declaration
(`model: hokuyo_ust` twice), but the two simulators produced different scans:

| | Gazebo before | Isaac and hardware UST-10LX |
|---|---|---|
| bins | 540 | 1081 |
| step | 0.50093° | 0.25° |
| range | 0.05–25 m | 0.06–10 m |
| rate | 40 Hz | 40 Hz |
| range noise | none | `rangeAccuracyM` 0.04 |

Two causes, both in the [pinned Clearpath generator](https://github.com/clearpathrobotics/clearpath_common/blob/33e4b311dd9a2dcaa8e8d262ae75fab2c53e560b/clearpath_generator_common/clearpath_generator_common/description/sensors.py)
and its [hokuyo macro](https://github.com/clearpathrobotics/clearpath_common/blob/33e4b311dd9a2dcaa8e8d262ae75fab2c53e560b/clearpath_sensors_description/urdf/hokuyo_ust.urdf.xacro):

- `Lidar2dDescription` hardcodes 0.5°, 0.05–25 m and 40 Hz for every 2D model.
- The macro's `samples` omitted the end ray, so even 0.25° would have given 1080.

In C5, Gazebo's 25 m range let polar profiling latch `wall_east` at x = 12 m,
where Isaac reported `no_value`. Those latches carried 83% of Gazebo's polar
error.

## Isaac noise measurement

The benchmark environment ran with no config layer, so the robot stood at the
origin and nothing spawned. GetEntitiesStates gave |x|, |y| < 3e-9 m. Two
sessions recorded 1000 consecutive scans per LiDAR at 40.00 Hz of sim time with
no gaps. A third spawned a catalog cabinet, a flat 0.9 m face, square to the
front LiDAR at 1, 2, 3 and 4 m, with 400 scans each.

- **Shape.** Per bin the noise is Gaussian: median skew 0.000, median excess
  kurtosis -0.02, with spreads matching sampling error for n = 1000. At every
  cabinet distance, 0.682/0.954/0.997 of samples fall within 1/2/3 sigma.
- **Bias.** None. On near-perpendicular wall bins the median of mean minus
  geometry is 0.01–0.05 mm, and its p05..p95 of ±2.5 mm is the standard error.
  On the cabinet face the bias is at most 0.7 mm.
- **Independence.** Correlation between neighbouring bins is below 0.001, and
  between consecutive scans below 0.002.
- **Magnitude.** The stddev depends on range, not incidence: 12.9 mm at 1 m,
  22.0 at 2 m, 30.8 at 3 m, 38.4 at 4 m, 45.2 at 4.9 m, 49.9 at 5.5 m and about
  69 mm at 8–9 m. Fit over 4158 bins: sigma = 0.01262 m * r^0.803, residual RMS
  1.1 mm. With range and incidence fitted together, the incidence exponent is
  0.006. `rangeAccuracyM` 0.04 equals sigma only near 4.2 m. The schema calls
  it a "range error scale" and states no distribution.
- **Azimuth noise** (`azimuthErrorStd` 0.01°). Its range-equivalent stddev on
  the walls has a median of 0.4 mm and a p95 of 1.7 mm, under 1% of the range
  variance.
- **Quantisation.** About 0.2% of published ranges lie on a 1 mm grid, which is
  chance. `rangeResolutionM` therefore does not quantise the ROS output.

Gazebo's `gpu_lidar` noise is one constant Gaussian stddev per sensor and
cannot follow range. The operator chose 0.036 m, Isaac's sigma at the
benchmark's median target distance of 3.7 m; true distances span p10 2.3 m to
p90 5.2 m across C5's 135 instances. The rejected alternatives were the
spec's 0.040 m, a gz-sensors source overlay for range-dependent noise, and no
noise at all.

## Gazebo after the patch

Two checks were made: the rendered SDF, and a live environment-only session
with the same recorder and cabinet procedure.

- 1081 ranges at -135..+135°, increment 0.25°, `range_min` 0.06,
  `range_max` 10.0, 40.00 Hz of sim time.
- The 235 front bins facing the far east wall are `+inf` in every scan, the
  same count as Isaac. No NaN, `-inf` or out-of-range value appears.
- Per-bin stddev 0.0360 m everywhere (p05..p95 0.0347..0.0374): 0.0359–0.0364 m
  on the cabinet at 1–4 m and 0.0359–0.0362 m on the walls at 4.9–9.5 m.
  Unbiased, Gaussian (0.682/0.955/0.997) and unquantised.
- 344 of the 846,000 finite front readings were exactly 10.0 m. gz-sensors clamps a
  noisy finite return into [min, max]. Isaac drops such a return, so the bin is
  `+inf`.
- The bridge leaves `scan_time` 0; Isaac sets 0.025 s.
- The generated `~/clearpath/robot.urdf.xacro` carried `ang_res="0.25"`,
  `min_range="0.06"`, `max_range="10.0"` and `noise_stddev="0.036"`.
- The hardware expansion (`is_sim:=false`) is canonically identical to the base
  revision's except inside the two LiDAR `<gazebo>` blocks.
- A SICK LMS1xx in Clearpath's all-sensors sample keeps its 548 samples, 0.05–25 m
  and no noise.

The new `test_lidar_description.py` failed 6 of 8 tests with the patch reversed
(`540 == 1081`, missing noise element, resolution 0.01) and passed 8 of 8 with
it applied. The checker gained ordered patch lists for this second
`clearpath_common` patch. On fresh clones at the core pins, `--apply` applied
both patches in order and reproduced the live checkout's 275-line diff byte
for byte. It applied only the missing later patch when the second was removed,
was idempotent on rerun, and rejected changes beyond the patches.

## Determinism

- **Gazebo.** Two identical examples runs (all estimators, one repeat) agreed
  bit for bit on all projective, euclidean and pointcloud estimates. Polar
  profiling, the only LiDAR consumer, matched 0 of 5: median |difference|
  1.8 mm, maximum 3.0 mm, no outcome change. Noise draws from an unseeded
  `gz::math::Rand`; ros_gz_sim's `gzserver` exposes no seed. The operator
  accepted this: LiDAR-dependent A/Bs need repeats, and seeding was not
  pursued.
- **Isaac.** Aligned by stamp, the two sessions carry the same noise at the same
  sim time. The median |difference| is 0.3 mm, against 70 mm for independent
  draws, and 77–82% of bins agree within 1 mm. Yet only 18 of 897 rear scans
  and no front scan were bit-identical, and isolated bins differed by a full
  redraw. Isaac's benchmark-level determinism was not measured.

All Isaac and Gazebo Ctrl-C stops exited in 0.9–4.1 s with no escalation and
left no process. The Gazebo environment session's done pattern never matched,
because the environment-only launch has no readiness gate. Its driver
therefore recorded after its 900 s timeout; the data are unaffected.

## Limits

- The noise model was measured only on flat, painted, static surfaces in one
  world, at 1–9.5 m. Below 1 m and on other materials it is unmeasured.
- Polar coverage and accuracy on the full set after the patch are not part of
  this record. The C5 full reruns on the new commit own those numbers.

## Independent review, 2026-10-25

The implementation at `39fb8c1` was reviewed and then uncommitted locally,
with a backup branch retained, to correct its rollback instructions and a
pre-existing test-domain collision. The reviewed sensor implementation and
dependency patches were retained unchanged. The review's root diff, commands,
raw data and logs are preserved under
`artifacts/benchmarks/ust10lx_review_20261025_base_39fb8c1/`; the corrected
shutdown test uses domains 91-97 instead of 161-167. At domain 167, the DDS
discovery port 49150 was occupied by another worktree's ROS daemon, and the
failure reproduced before the test-only correction. No unrelated process was
stopped.

A new disposable-checkout proof reproduced the live dependency diff, failed
the LiDAR regression suite before applying the UST patch, and verified the
hardware URDF was unchanged outside the two LiDAR Gazebo blocks. Fresh
stationary Gazebo and Isaac sessions each captured 1000 scans per LiDAR and
confirmed the geometry/range/rate contract and the documented noise difference.
These measurements do not establish full benchmark determinism or general
camera-path determinism; the earlier examples comparison supports only its
observed rows.

The original sessions' clean-stop result above did not reproduce for Isaac:
two review sessions logged SIGTERM escalation, and the second also logged
SIGKILL, despite launcher exit 0. Both reached `RUNNER EXIT` and
`Simulation App Shutting Down` before escalation. The installed SimulationApp
source matched its wheel's RECORD hash; neither it nor the Isaac runner was
modified by the LiDAR commit or this review. The cause of delayed native
shutdown remains unproven. Scan parity is therefore supported, while clean
Isaac teardown remains an open qualification issue.
