# Target-distance benchmark on Gazebo and Isaac: two-backend qualification

Recorded dates: 2026-10-25, 2026-10-26
Tested revisions: `5e65a0bd3c904a7676ee5042b4a3138eeec11964`, `854c34c32cdf075b0c60fc45bacdd4c6f53ee9da`
Provenance: complete

## Configuration and evidence boundary

This record closes the plan that let the target-distance benchmark run on
Gazebo or Isaac Sim through the standard ROS `simulation_interfaces` contract
(`backend:=gz|isaac`). It covers two populations:
- **`5e65a0b`: the first matched pair.** It exposed that the two simulators
  modelled different LiDARs.
- **`854c34c`: the accepted pair, after the UST-10LX Gazebo LiDAR patch.**
  The patch's own measurement is in
  [UST-10LX Gazebo LiDAR parity](2026-10-25-gazebo-ust10lx-lidar.md). On
  `854c34c` this record also covers six interrupted lifecycle cycles and the
  installed-launch smokes.

The automated gate (build, all repository test suites, dependency check,
doc links) was run by the post-patch review on candidate
`7e6cedf2169942c859248b7cfcbf7a386fe7d5f0`. That tree differs from `854c34c`
only in regenerated graphify output.

All runs share these conditions:
- **Tree and dependencies.** Clean tree (`uncommitted_files` 0 in every
  `run.json`), with `tools/check_dependencies` passing.
- **Host.** RTX PRO 6000 Blackwell, driver 595.99.02, NVIDIA GLX through
  `tools/gpu-run`, display `:12.0`.
- **ROS.** Jazzy, CycloneDDS with the packaged profile, ROS domain 53, a
  unique `GZ_PARTITION` per Gazebo launch.
- **Isaac Sim 6.1.** Run from `isaac_venv`, with the Gazebo adapter removed
  from the ament index during Isaac launches.
- **Benchmark settings.** The packaged `benchmark_scenarios_full.yaml`
  (SHA-256 `618c6e52f070cdfb1735ebe9717b16d6ad4f8c40afbed18f622e1f64c67c0fc9`,
  88 scenes, 135 instances), all four estimators, `repeats:=1` (the three
  probe scenes keep `repeats_override: 8`), `camera_profile:=640x480`,
  `depth_source:=stereoscopic`, box mask gate, and default isolation recipes.
- **Isaac defaults.** `depth_fidelity:=ideal` (renderer depth, as in
  Gazebo), `rtf:=1.0`, `sim_mode:=realtime`.
- **Catalogs.** Gazebo SHA-256 `18c8ddb6…`, Isaac `6c92ddf9…`.
- **Viewers on throughout, at the operator's request:** `gz_gui:=true`,
  `headless:=false`, `exploration_rviz:=true`.
- **Drive and stop.** Every launch ran on a PTY and was stopped with
  terminal Ctrl-C.

Evidence roots:
- `artifacts/benchmarks/c5_20261025_base_5e65a0bd/`, which holds the
  `5e65a0b` pair and `analysis_followup.txt`;
- `artifacts/benchmarks/c5rerun_ust10lx_20261025_base_854c34c3/`, which holds
  the `854c34c` pair, `lifecycle/`, `smokes/` and the comparison JSONs;
- `artifacts/benchmarks/ust10lx_review_20261025_base_39fb8c1/`, which holds
  the automated gate.

Each has a `README.txt` that indexes its files.

## First matched pair, `5e65a0b`

Both runs completed 109/109 trials with no infrastructure failure and no
leftover entity or process. Their stops were clean in 1.12 s (Gazebo) and
2.46 s (Isaac).

Polar profiling differed sharply: Gazebo MAE 0.609 m and p95 3.40 m, against
Isaac 0.183 m and 0.62 m. The cause was not the estimator. Clearpath's
generator gave every simulated 2D LiDAR 0.5° steps out to 25 m, while Isaac
modelled the UST-10LX at 0.25° over 0.06–10 m. Five Gazebo instances latched
the east wall at 10.9–12.1 m, and those latches carried 83% of Gazebo's polar
error. The pair was therefore not accepted. The LiDAR patch followed, and the
pair was rerun on `854c34c`.

## Accepted pair, `854c34c`

| | Gazebo | Isaac |
|---|---|---|
| completion | 1355 s | 1384 s, RTF 0.43 |
| trials included / skipped | 109 / 0 | 109 / 0 |
| CSVs, collages | 4 × 135 rows, 96 | 4 × 135 rows, 101 |
| clock advancing → first spawn | 0.014 s | 0.071 s |
| Ctrl-C to exit after completion | 1.25 s, all 36 children clean | 2.53 s, all 14 children clean |
| entities left | world and robot only | robot only |

| estimator | sim | scored | detector / gate / no-value misses | coverage | MAE | median | p95 [m] |
|---|---|---|---|---|---|---|---|
| pointcloud | Gazebo | 87 | 24 / 24 / 0 | 1.000 | 0.364 | 0.359 | 0.905 |
| pointcloud | Isaac | 114 | 20 / 0 / 1 | 0.991 | 0.202 | 0.141 | 0.627 |
| projective | Gazebo | 109 | 24 / 2 / 0 | 1.000 | 0.136 | 0.060 | 0.623 |
| projective | Isaac | 114 | 20 / 1 / 0 | 0.999 | 0.147 | 0.058 | 0.623 |
| euclidean | Gazebo | 109 | 24 / 2 / 0 | 1.000 | 0.136 | 0.058 | 0.592 |
| euclidean | Isaac | 114 | 20 / 1 / 0 | 0.999 | 0.134 | 0.058 | 0.583 |
| polar | Gazebo | 89 | 24 / 3 / 19 | 0.808 | 0.113 | 0.055 | 0.620 |
| polar | Isaac | 92 | 20 / 2 / 21 | 0.796 | 0.183 | 0.059 | 0.621 |

**Structural parity.** Counts, run and parameter keys, estimator schema, CSV
files, headers and row keys are equal. Spawn x, y and yaw are identical, and
derived truth agrees within 2.2e-7 m.

**Detector.** 18 detector misses are shared. Gazebo alone misses chaos_03,
far_beyond_01, objpartial_06, trap_contact_02, trap_layered_01 and
trap_layered_02. Isaac alone misses chaos_04 instances 0 and 1. Every
estimator inherits this difference, so per-estimator scored counts are not
comparable across simulators without restricting to a common set.

**Common-set ranking.** On the 68 instances that all four estimators scored in
both simulators, the order agrees: polar first (MAE 0.116 Gazebo / 0.107
Isaac), pointcloud last (0.415 / 0.207). Projective and euclidean swap places
within 3 mm (Gazebo 0.165 / 0.167, Isaac 0.147 / 0.144), which is inside
Isaac's run-to-run spread. Ranking each estimator on its own scored set gives a
different order and must not be reported as a simulator effect.

**Polar after the LiDAR patch.** On the 88 instances both simulators scored,
MAE is 0.113 m (Gazebo) and 0.108 m (Isaac), and the median |Gazebo − Isaac|
estimate difference is 9 mm. Isaac's higher headline MAE is one instance:
objpartial_05 (true 3.35 m) latches the wall at 9.94 m, inside Isaac's 10 m
window, while Gazebo returns `no_value`. No Gazebo polar estimate exceeds
10 m.

**Pointcloud.** Gazebo's 24 gate misses fall in 16 multi-target scenes. 22 of
them score on Isaac, while Gazebo's single-target pointcloud distances are only
about 0.36 m short. This points to a planar-position problem in the Gazebo
point cloud, first seen at C4 and not investigated here.

## Reproducibility

**Gazebo.** The full runs at `5e65a0b` and `854c34c` agree bit for bit on every
projective, euclidean and pointcloud estimate (109, 109, 87), so the patch left
the camera path untouched and that path repeated exactly. Polar changed as the
patch predicts:
- exactly the 5 wall latches became `no_value`;
- the remaining estimates moved by a median of 8.8 mm (max 87 mm).

With LiDAR noise on, polar is no longer bit-reproducible between runs.

**Isaac.** Isaac's runtime is identical between the two revisions, yet the runs
differ:
- **Detector.** chaos_04 instance 0 became a detector miss, and
  interfere_beside_01 was newly detected and scored.
- **Projective.** 55 of 113 estimates were bit-identical; the rest moved at
  most 9 mm.
- **Pointcloud.** 94 of 113 identical, at most 68 mm apart.
- **Euclidean.** 97 of 113 identical, but interfere_pair_03 moved from 2.10 to
  2.60 m.
- **Polar.** 0 of 91 identical; median difference 2 mm, max 15 mm.

Isaac benchmark A/Bs therefore need repeats for every estimator.

## Lifecycle and cleanup

Six cycles ran through the installed public launch with the full-run settings.
Each was stopped by Ctrl-C on the PTY mid-trial: about 4 s after trial 1,
during warm-up, and about 9 s after trial 3. A benchmark G1 was spawned at every
interruption.

| | Ctrl-C to exit | escalation | processes left | `bench_*` entities left |
|---|---|---|---|---|
| Gazebo, 3 cycles | 3.12–3.15 s | none | 0 | none (server gone) |
| Isaac, 3 cycles | 3.17–3.68 s | none | 0 | none (runner gone) |

In every cycle the runner logged "Benchmark interrupted." and exited 130. Its
rollback delete failed because the simulator had received the same SIGINT and
was already stopping, so the G1 ended with the simulator process.

Further non-clean exits:
- **Gazebo `gzserver`**, 3 of 3 cycles: `std::future_error: Broken promise`
  (exit -6), 0.15–0.18 s after Ctrl-C. This is the gz-sim `UserCommands`
  blocking-handler defect recorded in the
  [ros_gz bridge record](2026-10-22-ros-gz-bridge-shutdown.md).
- **Gazebo `target_overlay_node`**, once: exit 1.
- **RViz**, once: SIGSEGV on exit.
- **Isaac:** `isaac_runner` reached `RUNNER EXIT` and finished cleanly in all
  3 cycles. The environment-only escalation that the post-patch review
  recorded did not reproduce.

Repeated cleanup leaves nothing behind. Every cycle started with an empty
process table, and every completed run ended with an empty `^bench_.*$` query.

## Installed-launch smokes

All six public launch files in `install/` are symlinks into tracked source and
load with `--show-args` (exit 0). The installed sweep resolves both backends
in `--dry-run`, Isaac with the Gazebo adapter hidden. It does so without
launching anything or creating an output directory.

The following are refused before any process starts, each with its intended
message:

| request | exit |
|---|---|
| sweep `backend: hardware` | 2 |
| Isaac sweep carrying `gz_gui` | 2 |
| per-config `backend` | 2 |
| `target_distance_benchmark.launch.py backend:=hardware` | 1 |
| `target_benchmark_env.launch.py backend:=webots` | 1 |
| `ridgeback_exploration.launch.py backend:=webots` | 1 |

## Automated gate

Review candidate `7e6cedf`:
- **Build.** The three-package set built.
- **colcon.** `colcon test-result` reported 0 errors and 0 failures.
  ridgeback_autonomy 1287, ridgeback_autonomy_gz 21, ridgeback_autonomy_isaac
  79 plus 22 skipped (the pxr cases; they pass under the Isaac interpreter).
  Over the whole build tree, 1885 records.
- **pytest.** Full core suite 1287 passed. The gz, Isaac (system and Isaac
  interpreter), camera-support and mecanum suites passed.
- **Checks.** `tools/check_dependencies` and `tools/check_doc_links.py` passed.

The run of the same gate on the content of `39fb8c1` had one failure,
`test_localization_node_shutdown`. Its test domain's discovery port was held by
an unrelated daemon. The candidate's fix moved those test domains below the
ephemeral range.

## Limits

- **One run per simulator per revision.** Cross-simulator differences smaller
  than Isaac's run-to-run spread are not resolvable.
- **One configuration.** Only the box gate, stereoscopic depth, default
  recipes and the 640x480 profile were run. Isaac ran ideal depth, so its
  D455 depth mode was not exercised.
- **Not qualified for tuning.** The execution-environment qualification gate
  had not been completed for either simulator. These runs are acceptance
  evidence, not qualified tuning results.
- **Configurator.** It has no backend control: its authored sweeps run on
  Gazebo.
