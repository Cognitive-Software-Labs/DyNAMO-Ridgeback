# Plan: does a remote session measure the same as the physical seat?

Status: **READY TO RUN.** Prepared 2026-09-05 against `262e8a1`.

## Decision to make

With `tools/gpu-run` applied, does a benchmark run on the **remote X session**
produce the same measurements as one on the **physical seat (`:0`)**? If not,
name every surviving difference and say which of them invalidate a result
rather than merely slowing it.

This is not "is remote fast enough". It is "can a number taken remotely be
quoted as if it came from the seat".

## Why it is open

The physical seat has NVIDIA GLX natively. The remote session does not, so
`tools/gpu-run` routes GLX to NVIDIA
and NVIDIA reads every frame back over PCIe at roughly 290 MB/s. That readback
is a real, measured cost that the seat does not pay:

| surface | pixels | readback ceiling |
|---|---|---|
| simulated camera, 640x480 | small | ~235 FPS |
| RViz window, 1920x1080 | large | ~35 FPS |

So the *sensor* path should be unaffected (28 Hz sits far under 235) while the
*display* path is heavily capped. Whether that display cost leaks into sensor
timing through CPU/GPU contention is exactly what is unmeasured.

## What is already established — do not re-derive

- **Software GL is the failure mode, not remoteness per se.** `llvmpipe` gave
  3.80 Hz camera, forced NVIDIA gave 28.07 Hz, same configuration. See
  [`docs/troubleshooting.md`](../troubleshooting.md#camera-rate-collapses-under-software-rendering).
- Refuted causes of that collapse: subscriber drops, QoS, the ffmpeg screen
  recorder, the `detector_debug` probe, detector rate, orphaned processes, CUDA
  availability. All tested. The evidence is in
  [operational incident history](../history/operational_incidents.md#camera-software-rendering-collapse--measured-2026-09-05).
- **This plan tests Gazebo's OpenGL/GLX path.** It makes no claim about other
  rendering APIs or simulators.
- **`tools/gpu-run` reproduces the manual export**: 25.61 Hz vs 26.51 Hz camera on the
  same config.
- Exploration needed the CycloneDDS participant ceiling raised (`262e8a1`);
  that is orthogonal to seat and already fixed for both.

## The methodological trap this plan exists to avoid

Two runs of an **identical** cell D configuration, same seat, same GL, gave:

| run | worker p95 | slot replacements |
|---|---|---|
| A | 71.9 ms | 9 |
| B | 140.2 ms | 18 |

A single run per seat would therefore "find" a seat difference that is really
run-to-run variance. **Within-seat variance must be measured before any
across-seat claim.** Three replicates minimum per seat, and the comparison is
between distributions, not between single values.

## Preconditions

- Tree clean at a known commit; `git status --porcelain` empty.
- Nothing else running: `ps` clear of `gz sim`, `rviz2`, `ffmpeg`, `target_*`.
- Record the remote session's active renderer and Xorg start time. Host display
  configuration changes apply only to newly started remote sessions, so a
  pre-existing session is not evidence that the current configuration works.
- On `:0`, confirm no one is using the seat. Do not take over an active session.

## Phase 0 — freeze provenance

For each of the two runs record, before starting:

```bash
echo "$DISPLAY"; glxinfo -B | grep -E "OpenGL (vendor|renderer)"
ps -o lstart= -p $(pgrep -f 'Xorg' | head -1)
git rev-parse --short HEAD; git status --porcelain | wc -l
nvidia-smi --query-gpu=name,memory.used,clocks.sm,temperature.gpu --format=csv
uptime; nproc; free -g | head -2
echo "${CYCLONEDDS_URI:-<unset>}"; echo "${RMW_IMPLEMENTATION:-<unset>}"
```

The sweep already records the GL renderer in `sweep.json` (`32efac7`), so the
per-run artifact is self-describing — but capture the above anyway, because the
seat, the X server start time and the host load are not in it.

## Phase 1 and 2 — the same sweep, twice

Use one checked-in sweep YAML as the contract so neither seat can silently
differ in settings. Cells: **A** (`box`/`stereoscopic`, no models) and **D**
(`silhouette`/`monocular`, both models). A is the floor; D is the only cell whose
margin is thin enough for a seat difference to show.

```yaml
defaults:
  scenario: <installed>/benchmark_scenarios_examples.yaml
  repeats: 3
  estimators: projective_ranging,euclidean_reconstruction
  detector_fps: 10.0
  detector_debug: true
configs:   # rotated, three replicates each
  r1_a, r1_d, r2_d, r2_a, r3_a, r3_d
```

Run identically on both seats, through the wrapper in both cases so the command
is the same (`tools/gpu-run` is a no-op on `:0`):

```bash
tools/gpu-run ros2 run ridgeback_autonomy target_benchmark_sweep <yaml>
```

Keep `record_video` at one setting for both, and state which. Video off is the
better default here: the screen grab is a large-surface readback, which is
precisely the cost that differs between seats, so leaving it on confounds the
comparison. Run one separate visual smoke with it on.

## Phase 3 — compare

Report per seat, per cell, across the three replicates: median **and spread**.

| quantity | source | why |
|---|---|---|
| camera + depth rate | mask node `rx color=` / `depth=` deltas | the thing that collapsed before |
| detector achieved rate, `gate_wait`, `inference` | `detector_debug` | separates schedule from model |
| worker p50/p95/p99, `pending_replaced`, `completion_rate`, dequeue age | mask node | where the margin is thin |
| MAE / median / P95 error, scored count, miss reasons | `summary.md` | must be seat-invariant |
| RTF, wall time | `sweep.json` | catches a slow simulator |
| GL renderer | `sweep.json` `gl` | proves which path each run took |

Decision rules:

- **Accuracy must be identical.** MAE and scored counts are deterministic given
  the same scenario; any difference means something other than rendering speed
  changed, and that is a bigger finding than the seat question.
- **A difference counts only if it exceeds within-seat spread.** Compute the
  remote-vs-remote spread first; a remote-vs-seat gap smaller than that is not a
  result.
- **Separate "slower" from "wrong".** A lower RViz frame rate remotely is
  expected and harmless. A lower *sensor* rate, replacements, or lost scored
  instances are not.

Outcomes:

1. **Equivalent** — remote numbers may be quoted as seat numbers. Record the
   result in history so the comparison is not re-litigated.
2. **Equivalent for sensors, worse for display** — the expected result. Record
   which metrics travel and which do not.
3. **Sensor-path difference survives** — remote is not a valid measurement
   environment; benchmarks move to `:0` and the reason gets documented.

## Phase 4 — hypotheses to check afterwards, most actionable first

### H1. The cell D margin is thinner than one run suggested

Worker p95 was 71.9 ms then 140.2 ms in a 100 ms budget, replacements 9 then 18,
`completion_rate` 1.000 both. Nothing failed, but "75% duty" came from n=1.
**This plan's replicates settle it** — read the answer out of Phase 3 rather
than running anything extra. If p95 routinely exceeds 100 ms, 10 Hz is at the
edge and the concurrency question reopens earlier than 16 Hz.

### H2. Gazebo cannot open the render node selected for remote X

One exploration launch logged:

```
libEGL warning: failed to open /dev/dri/renderD129: Permission denied
libEGL warning: failed to open /dev/dri/renderD128: Permission denied
```

If the intended render-node permission does not hold for the current user, the
remote session stays on `llvmpipe`. Check `ls -l /dev/dri/render*`, group
membership, and the active renderer in a newly started remote session.

### H3. Ground truth is measured to the wrong surface

Every estimator is biased ~0.06 m near, because truth is the target's base
origin while any depth sensor sees its near face — the G1 torso puts that face
0.0836 m in front of the origin. Confirmed arithmetic, not a hypothesis
(`docs/do_not_try_again/monocular_depth_error.md`). The open question is
whether the benchmark's truth *should* be surface-relative. It affects every
number ever reported, so it is a definition decision, not a bug fix. Decide
deliberately; do not quietly change it.

### H4. Six instances are missed by the detector in every single cell

Every configuration of the 2026-09-03 sweep scored 18/24 with
`missed: detector 6`, identical across cells and replicates. That is systematic,
not noise, and it is independent of depth source and mask gate. Nobody has
looked at *which* six. If they are the deliberately occluded instances the
scenario intends to be missed, this is correct behaviour and should be
documented as such. If not, detector recall is silently capping every result.
Cheap to check from the per-trial CSVs.

### H5. The CUDA-sync barrier may be measuring nothing

`cuda_sync` warm mean is ~0.075 ms immediately after a 40 ms inference, which is
the cost of an already-satisfied barrier. That suggests the model call already
synchronizes internally (HF pipelines move outputs to CPU), so the explicit
barrier is redundant rather than wrong. Harmless, but if true the stage timings
are honest for a reason other than the one documented. Verify before relying on
the barrier for a future GPU-overlap experiment.

### H6. Detector rate above 10 Hz

20 Hz is ruled out on measurement: the detector reaches 19.99 Hz but cell D
needs more than the 50 ms budget. 15 Hz is undecided — 62-75 ms p95 in a 66.7 ms
budget is genuinely marginal and cannot be settled on paper. Only worth running
if a faster cadence is actually wanted.

## Boundaries

- One commit for both seats. A rebuild between them invalidates the comparison.
- Do not change estimator recipes, scenario bytes, detector settings or the
  transport for this experiment.
- Do not run the two seats concurrently — they share the GPU and the host.
- Do not take over an occupied physical session, and do not modify the display
  manager or seat configuration as part of this measurement.
- Preserve other users' processes. Inspect ownership before terminating
  anything; an interrupted sweep can orphan a benchmark runner that `cleanup.sh`
  does not catch.

## Traps

- **`ros2 topic hz` reports nothing** on the camera: the stream is best-effort
  and the CLI subscribes reliable. Use a `qos_profile_sensor_data` subscriber, or
  the mask node's `rx color=` counter across two log lines.
- **RTF does not reveal a software-GL run.** It stayed ~0.89 while the camera was
  at 3.80 Hz. Trust the recorded GL renderer, not RTF.
- **Remote X configuration applies at session start.** A session that predates
  a host change keeps the old configuration, so verify the X server's start
  time rather than assuming a reset affected it.
- **Cold model load is not steady state.** First-use slot replacements are
  expected in every run; only replacements after the load window count.
