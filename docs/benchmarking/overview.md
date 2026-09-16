# Target-distance benchmarking

What the benchmark is, what it compares, and what it may not be used to claim.
To start a run, see [running benchmarks](running_benchmarks.md).

## Purpose and scope

The benchmark compares independent target-localization estimators on fixed
spawned target instances in `target_distance_calibration`. Unitree G1 is the
current scenario asset; reusable perception and benchmark APIs use target names.
It measures simulator ground-truth accuracy, misses/coverage, and run provenance.
It is not hardware validation and never substitutes one estimator for another.

## Supported estimators and axes

Public estimator keys, in canonical order:

- `pointcloud` — organized `PointCloud2` input;
- `projective_ranging` — mask plus aligned depth, image-domain reduction;
- `euclidean_reconstruction` — mask plus aligned depth, point-domain reduction;
- `polar_profiling` — mask plus projected planar scan.

`estimators:=all` is the default. A subset launches only required measurement
nodes and disables unselected branches. Mask estimators expose `mask_gate`
(`box` default, or `silhouette`). Depth paths additionally expose `depth_source`
and registered `isolation_2d`/`isolation_3d` recipes; silhouette depth paths
bypass rect isolation. Polar is independent of depth source; pointcloud has its
own source. Output names include only axes that actually executed.

Which of these axes a given run may vary depends on the profile, and which
estimators it may select depends on the profile and on the evidence named — see
[profiles](profiles.md).

The root README owns complete build/run instructions. Main launch:
[`target_distance_benchmark.launch.py`](../../src/ridgeback_autonomy/launch/target_distance_benchmark.launch.py).

## Not this: `tools/benchmark/`

`tools/benchmark/` is the instrumented **exploration** harness — a probe and a
camera-less robot profile for measuring frontier exploration. It has nothing to
do with target-distance benchmarking and shares no code, scenarios, or outputs
with it. Its runbook is
[the exploration benchmark runbook](../exploration/benchmarking.md).

## Where to go next

- [running benchmarks](running_benchmarks.md) — which question, which profile,
  and which of the four ways to start it
- [semantics](semantics.md) — scenarios, events, observations, instances, miss
  reasons, and what may be compared with what
- [outputs](outputs.md) — `run.json`, `sweep.json`, `summary.md`, CSVs,
  collages, provenance, rename rules, and module ownership
- [profiles](profiles.md) — the legacy measurement dataset and the four-profile
  layered replay contract
- [configurator](configurator.md) — the local browser UI, including its run
  substrate
- [scenario gallery](benchmark_scenarios_v2_gallery.html) — visual reference for
  the packaged scene set
