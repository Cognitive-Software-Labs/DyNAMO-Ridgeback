# Running benchmarks

This page routes a question to a profile, a profile to a way of starting it, and
names the constraint that decides when more than one way exists. It routes; it
does not explain. For what the benchmark is, what it compares, and what it may
not claim, read [the overview](overview.md) first. The root README owns the
live commands; [launch arguments](launch_arguments.md), [sweeps](sweeps.md)
and [profiles](profiles.md) own the arguments, the sweep format and the replay
workflows.

## Qualified execution environments

Gazebo rendering through `tools/gpu-run`, with the Gazebo GUI and RViz open,
is qualified on the physical seat (`:0`) and in an xrdp session: matched light (box, stereo) and heavy
(silhouette, monocular) runs gave identical error and scored counts on both,
and observation coverage varied as much between replications on one display
as between displays. Results from either display may be compared, tuned on and
reported, and replay tuning may use captures from either. The evidence predates
the current camera model and covers the example scenes; the owner accepted it
as sufficient.

Not qualified: Isaac, headless EGL, `1280x720`, video recording, and runs
without the Gazebo GUI or RViz. Qualify those before quoting them.
Confirm the GL path with `tools/gpu-run glxinfo -B` from the run's own
session. Software rendering is a failure, not an environment; see
[troubleshooting](../troubleshooting.md#camera-rate-collapses-under-software-rendering).
The CLI does not enforce any of this.

## Which question, which profile

`benchmarking/replay_profiles.py` is the authority; `target_replay_describe
--json` prints the same contract without importing ROS or a model.

| Question | Profile | Mutable from | Reads |
|---|---|---|---|
| Tune measurement | `measurement` | measurement | a legacy measurement dataset |
| Compare box with current SlimSAM | `mask-output` | measurement | a sensor capture plus its mask caches |
| Try another mask model or settings | `mask-model` | mask | a sensor capture |
| Measure latency, throughput, or integration | `live-system` | the whole system | nothing — it runs the robot |

Every offline profile evaluates the three mask estimators. Only `live-system`
reads the organized point cloud, and only `live-system` may claim latency,
throughput, integration, GPU contention, or simulator real-time factor. An
individual artifact narrows the offline set further — see
[profiles](profiles.md).

## Which way do I start it

| Way to start | Profiles | Needs | Survives closing the tab |
|---|---|---|---|
| configurator **Start** | the three offline | nothing special | yes — re-attaches from disk |
| `target_replay_benchmark <job>` | the three offline | nothing special | it is your terminal |
| `target_offline_replay_benchmark <dataset> <sweep>` | `measurement` only | nothing special | it is your terminal |
| [`target_benchmark_sweep <yaml>`](sweeps.md) | `live-system` | GPU-backed X session, hours | terminal only |

Plus `target_distance_benchmark.launch.py` for a single live run — and for
*every* capture. Both replay formats are written by a live benchmark run, not by
a separate capture tool, so the first offline experiment always begins with a
live one.

Live sweeps stay terminal-only until the page can supervise a live run's
conflicts, progress and cancellation; the
[live-sweep plan](../plans/BENCHMARKING_live_configurator_sweeps.md) owns that
work. Preflight cleanup no longer kills the configurator
([troubleshooting](../troubleshooting.md#what-cleanupsh-leaves-alone)). The configurator can still author,
validate, and render a live sweep command — it just cannot press Start on one.

## Which simulator

`backend` selects Gazebo (`gz`, the default) or Isaac Sim (`isaac`) for a live run
or for a sweep's persistent environment. Everything downstream of the simulator
is the same on both: scenario file, runner, estimators, scoring, and reports.
- **One simulator per run.** A sweep may set `backend` and its backend-only
  settings only under `defaults`. Setting the other backend's options (for
  example `gz_gui` on Isaac) is refused, and so is `hardware`, which has no
  scene control and no ground truth.
- **What each run records.** Every run records which simulator, which native
  world, and which asset catalog produced it.
- **Offline replay.** It reads captures and does not care which simulator made
  them. The capture manifest itself does not name the simulator, though: only
  the `run.json` of the live run that wrote it does, so keep the two together.
- **Comparing the two simulators:** see [semantics](semantics.md#scenario-and-comparison-discipline).

`camera_profile` selects `640x480` (default) or `1280x720`, and is recorded in
every run. A sweep may set it only under top-level `defaults`; it cannot vary
per configuration, because changing the render grid requires restarting the
simulator. `sweep.py` enforces this. Two profiles are separate benchmark
conditions and must not be pooled as repetitions — for what the profile changes
downstream, see [the camera depth pipeline](../isaac/camera-depth.md).

### Two commands reach `measurement`

`target_offline_replay_benchmark` takes a dataset directory and a sweep YAML
positionally. `target_replay_benchmark` takes one canonical replay job that
names the same inputs, and is what the configurator renders. They validate the
same sweep variants and call the same evaluation and reporting code, so the
numbers agree; they differ in interface and in what lands on disk:

| | `target_offline_replay_benchmark` | `target_replay_benchmark` |
|---|---|---|
| Inputs | `<dataset> <sweep> --output-dir --workers` | one job file; workers come from the job |
| Profiles | `measurement` only | all three offline |
| Output layout | results at the output root | `results/` plus `job.json` |
| Partial runs | writes into the output directory | stages, then renames atomically |

Use `target_replay_benchmark` when the job came from the configurator, when the
run should be reproducible from a single committed file, or when an interrupted
run must not leave a directory that looks like a result. Use
`target_offline_replay_benchmark` for a quick sweep over a dataset you already
have — it is the form the [legacy smoke workflow](profiles.md#legacy-smoke-workflow) uses.

## After the run

[semantics](semantics.md) defines what a scored row is and why an estimator
missed; [outputs](outputs.md) describes what landed on disk.
[The overview](overview.md) maps the rest.

## Archived evidence

- [Remote versus physical-seat validation](../../archive/engineering/2026-09-07-seat-validation.md)
