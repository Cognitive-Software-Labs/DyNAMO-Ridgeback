# Running benchmarks

This page routes a question to a profile, a profile to a way of starting it, and
names the constraint that decides when more than one way exists. It routes; it
does not explain. For what the benchmark is, what it compares, and what it may
not claim, read [the overview](overview.md) first. The root README owns the
commands.

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
| `target_benchmark_sweep <yaml>` | `live-system` | GPU-backed X session, hours | terminal only |

Plus `target_distance_benchmark.launch.py` for a single live run — and for
*every* capture. Both replay formats are written by a live benchmark run, not by
a separate capture tool, so the first offline experiment always begins with a
live one.

Live sweeps stay terminal-only because
`target_benchmark_sweep` runs `cleanup.sh` before Gazebo starts, whose catch-all
would kill the configurator and any run it started; see
[troubleshooting](../troubleshooting.md). The configurator can still author,
validate, and render a live sweep command — it just cannot press Start on one.

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
have — it is the form the README's legacy smoke workflow uses.

## After the run

[semantics](semantics.md) defines what a scored row is and why an estimator
missed; [outputs](outputs.md) describes what landed on disk.
[The overview](overview.md) maps the rest.
