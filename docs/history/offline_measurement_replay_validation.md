# Offline Measurement Replay V1 Validation

Validated 2026-09-09 on branch `g1-distance-benchmarks`, commit `b5a22c15`,
with a clean tracked worktree. Replay V1 passes its stop condition for
box-gated stereoscopic `projective_ranging`: the default replay result preserves
the live scoring contract and the current 15-variant projective sweep is about
50x faster end to end than its prior live-sweep estimate.

This validates a measurement-accuracy benchmark over frozen evidence. It does
not make replay authoritative for detector behavior, ROS delivery, latency,
throughput, model changes, or final integration.

## Decision

- Use **five raw detection batches per trial** as the V1 capture default.
- Keep the CLI worker count explicit. On this 32-logical-CPU host, **16 workers**
  was the measured knee; 32 added overhead without reducing wall time.
- Treat Replay V1 as validated for projective recipe/parameter selection.
- Do not change a production measurement default on replay evidence alone.
- Defer Euclidean, model-output, scan, and point-cloud replay until each path has
  its own live/offline parity fixture. The next sensible extension is Euclidean
  reconstruction because the V1 payload already contains depth ROIs,
  intrinsics, and extrinsics, but this run does not validate that code path.

This was the V1 decision at the date of the run. The later
[layered replay implementation](layered_replay_implementation_validation.md)
added typed sensor/mask artifacts and projective-plus-Euclidean execution; its
remaining live-parity and scaling gates supersede this extension note.

## Batch-count selection

The selection artifact is
`artifacts/benchmarks/replay_batch_selection_20260909T1845/`. It captured the
five-scene example set with 10 raw batches per trial: 5/5 trials, 8 instances,
zero skips, 18.387 seconds, and 1.7 MB on disk. It includes single-target,
multi-target, foreground-occlusion, and intentional detector-miss cases.

Each trial was replayed with the first 1, 3, and 5 events and compared with all
10. Every prefix preserved all eight instance outcomes and miss reasons.

| Prefix | Outcome changes | Miss-reason changes | Numeric pairs | Max estimate delta vs 10 | Mean delta |
|---:|---:|---:|---:|---:|---:|
| 1 | 0 | 0 | 6 | 0.024087 mm | 0.004015 mm |
| 3 | 0 | 0 | 6 | 0.024087 mm | 0.004015 mm |
| 5 | 0 | 0 | 6 | 0.001669 mm | 0.000278 mm |

One event happened to be sufficient in this compact run, but five was selected
as the conservative default: it retains repeated observations and trial-median
behavior while remaining numerically indistinguishable from 10 at the scale of
the measurement.

The matching 10-batch live/offline baseline comparison had zero outcome or
miss-reason mismatches and a maximum estimate delta of `1.12e-7 m`.

## Full capture and provenance

The full artifact is
`artifacts/benchmarks/replay_full_validation_20260909T1900/`.

| Field | Value |
|---|---|
| Dataset ID | `20260909_184740` |
| Dataset schema | 1 |
| Source | `b5a22c15`, clean tree |
| Scenario | `config/benchmark_scenarios_full.yaml` |
| Scenario SHA-256 | `ce05ce4f278570aa8b02f47f92b37f9af5a924c05836c50875f55069b476fe67` |
| Sweep SHA-256 | `6744d5bfe84ee4bc80ac3201bff511d76b3ab698cdd2d7ce4f9142a1e87aaf44` |
| Dataset content hash | `b55181388e00b31d7a92d7909c8f6015fb94d06bc3800d783b80cc8433f758c6` |
| Scenes / trials / instances | 88 / 109 / 135 |
| Included / skipped trials | 109 / 0 |
| Raw batches | 545 total, exactly 5 per trial |
| Capture wall time | 501.118 s (8m21.118s; 4.597 s/trial) |
| Dataset size | 11 MB by `du`; 11,054,144 payload bytes |
| Uncompressed NPZ entries | 32,178,404 bytes |
| Payload compression ratio | 2.91:1 |

The live baseline scored 112 instances and retained 23 detector misses. Its 558
projective measurement observations were all `OK`; intentional empty detector
batches remain valid detector-miss evidence rather than capture failures.

## Live/offline parity

The full offline baseline was joined to the matching live CSV by
`(trial_id, instance_index)`:

| Check | Result |
|---|---:|
| Rows on each side | 135 |
| Outcome mismatches | 0 |
| Miss-reason mismatches | 0 |
| Paired numeric estimates | 112 |
| Maximum estimate delta | `4.04e-7 m` |

The joined rows cover multi-instance scenes, so identical per-instance outcomes
and estimates also prove association parity over this capture. The residual
numeric delta is below one micrometre and is consistent with float32 transport
through the live ROS message; offline evaluation retains the underlying NumPy
precision.

Sequential and parallel determinism was checked at 1, 2, 4, 8, 16, and 32
workers. Every one of the 15 `projective_ranging.csv` files was byte-identical
to its one-worker counterpart.

## Performance

The installed CLI evaluated all 15 variants over 109 trials. `Evaluator median`
is five repetitions of the evaluator, including process-pool startup and trial
payload loading. `CLI wall` is one complete command including report writes.
GNU `time -v` reported 42.3--43.5 MiB maximum RSS across the runs; that metric is
the maximum reported for the process tree, not the sum of simultaneously live
workers.

| Workers | Evaluator median | Min--max | CLI wall | Reported max RSS |
|---:|---:|---:|---:|---:|
| 1 | 1.974 s | 1.961--2.016 s | 2.13 s | 43.5 MiB |
| 2 | 1.174 s | 1.154--1.185 s | 1.34 s | 43.3 MiB |
| 4 | 0.674 s | 0.661--0.690 s | 0.88 s | 42.8 MiB |
| 8 | 0.486 s | 0.468--0.502 s | 0.68 s | 42.8 MiB |
| 16 | **0.331 s** | 0.326--0.349 s | **0.53 s** | 42.5 MiB |
| 32 | 0.338 s | 0.329--0.350 s | 0.55 s | 42.3 MiB |

At 16 workers, keeping the 109-trial dataset fixed and increasing the variant
count produced:

| Variants | Evaluator median | Min--max |
|---:|---:|---:|
| 1 | 0.113 s | 0.111--0.116 s |
| 2 | 0.131 s | 0.129--0.132 s |
| 5 | 0.178 s | 0.177--0.178 s |
| 10 | 0.255 s | 0.250--0.261 s |
| 15 | 0.329 s | 0.327--0.330 s |

The 15-variant evaluator processes 1,635 trial-variant pairs in 0.331 seconds,
about 0.203 ms per pair after the frozen capture exists. More variants add
roughly linear measurement work without repeating Gazebo or OWLv2.

The prior live supervisor estimate was 6.99 hours for 15 configurations. The
new end-to-end path is the one-time 501.118-second capture plus the 0.53-second
16-worker CLI run: 501.648 seconds, or about 8m21.6s. That is a **50.2x
speedup**, clearing the 5x stop condition by roughly an order of magnitude.

## Limits of the result

- The capture is one clean simulation run on one host. Five batches is grounded
  for this V1 scenario and detector cadence, not a universal sensor statistic.
- Worker scaling is host-specific; memory bandwidth, storage, and CPU topology
  can move the knee.
- Replay deliberately freezes detector outputs and delivery outcomes. It cannot
  compare detector/model parameters or establish live throughput and latency.
- Replay candidates still require the plan's final live confirmation before a
  production default changes.
