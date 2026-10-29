# BENCHMARKING — Geometry reruns

Status: **planned, not started; blocked on dependencies.** The
[benchmarking backlog](../target_distance_benchmarking/BACKLOG.md#camera-geometry-benchmark-recertification)
owns the open item; this plan owns the rerun order and preconditions. Every
benchmark result recorded before the scene recertification predates the
current camera model, the floor-standing assets and the `edge640_*` scenes, so
none of it may be quoted as current.

## Preconditions

| Precondition | Why | State when planned |
|---|---|---|
| `tools/check_dependencies` passes | Without `clearpath_ust10lx_sim_lidar.patch` Gazebo models the wrong LiDAR, and pre-patch polar runs are not a baseline ([semantics](../target_distance_benchmarking/semantics.md)) | fails: patch not applied; `src/gz-sim`, `src/ros_gz`, `src/urg_node` missing; `src/clearpath_simulator` beyond its patch |
| Clean committed tree, install rebuilt from it | Sweeps record uncommitted files; the concurrency sweep requires a clean checkout | shared checkout often dirty; use a quiet window or an isolated worktree with its own build |
| [Qualified execution environment](../target_distance_benchmarking/running_benchmarks.md#qualified-execution-environments) | Only that environment's results may be quoted | physical seat `:0` or xrdp through `tools/gpu-run`, Gazebo GUI and RViz open |
| Quiet host | Model timing in the concurrency sweep | needed for step 2 only; the others run on simulation time |

Restoring dependencies changes the shared install. Coordinate with any session
working on the dependency checkout before running `--apply` and rebuilding.

## Before any run

Label the numbers current pages still quote as predating the recertification,
so they are not reused by accident:

- `docs/target_localization/polar_profiling.md` §7.2 (2026-09-09 polar run)
- `docs/do_not_try_again/monocular_depth_error.md` (2026-09-03 concurrency sweep)
- `docs/target_localization/aligned_depth.md`, the 2026-07-13 sensor-depth
  versus Depth Anything comparison
- the measured values in the `benchmark_sweep_projective_parameters.yaml` header

`exact_stamp_depth_delivery.md` describes transport behaviour and is not
geometry-dependent; leave it.

## Reruns, in order

Run `cleanup.sh` once before each sweep, never between its configurations.
Estimates use the sweep tool's 15.4 s per trial; the last measured full run took
12.4 s per trial, so they are upper-end.

| Step | Sweep | Size | Estimated wall time | Unblocks |
|---|---|---|---|---|
| 1 | `benchmark_sweep_baseline.yaml` | 4 configs × 109 trials | about 2 h | current reference for all four estimators; replaces the polar §7.2 numbers; occlusion-family data |
| 2 | `benchmark_sweep_model_concurrency.yaml` | 14 configs × 15 trials | about 1 h, quiet host | [combined-model throughput](BENCHMARKING_model_throughput.md), the monocular-depth note, the aligned-depth comparison |
| 3 | `benchmark_sweep_isolation.yaml` | 12 configs × 109 trials | about 5.6 h, run as two resumable halves | isolation-validation backlog item |
| 4 | analysis of steps 1 and 3 | no new sweep | — | occlusion-characterization backlog item (`interocc_*`, `objocc_*`, `objpartial_*`) |

Step 3 runs in halves because a long sweep with the GUI on has lost Gazebo
before ([troubleshooting](../troubleshooting.md#long-sweep-loses-gazebo)).
After each step, replace the labelled numbers it covers with the new results
in their owning pages, and record the run in a dated archive record.

## Deferred

- `benchmark_sweep_projective_parameters.yaml` (15 configs, about 7 h): rerun
  only when projective ranging is tuned again.
- A fresh live capture for layered replay: a separate capture workflow, after
  step 1.
- The Isaac four-cell matrix that `monocular_depth_error.md` lists as its reopen
  step: Isaac is not a qualified execution environment yet.

## Completion

Steps 1–4 have run under the preconditions, the labelled numbers are replaced
or removed, and the backlog item's completion criteria hold. Then retire this
plan.
