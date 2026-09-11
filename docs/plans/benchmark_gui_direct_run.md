# Plan: start a live sweep from the configurator GUI

Status: **PROPOSED — BLOCKED.** Phases 0, 1 and 3 of this plan shipped on
2026-09-11; what remains is Phase 2, starting the `live-system` profile from the
page. Delivered work, its evidence, and the reversals it required are recorded in
[`../history/benchmark_gui_direct_run.md`](../history/benchmark_gui_direct_run.md).

Prepared 2026-09-11 against `18eacb4`, branch `feat/benchmark-gui`.

## What already exists

The run substrate is built and exercised by the three offline profiles:
server-side job write, one path anchor, on-disk run records with
cmdline-verified liveness, graded SIGINT → SIGTERM → SIGKILL cancellation of the
run's process group, server-side single flight, and a browser tab that
re-attaches from disk instead of owning the run. Phase 2 reuses all of it and
adds only what is genuinely live-system-specific.

The UI already shows a disabled Start for `live-system` with the reason stated,
rather than hiding the button.

## The blocker

**`cleanup.sh` kills the configurator.** The catch-all

```bash
# Every node here is installed under lib/ridgeback_autonomy, so match that.
kill_matches "lib/ridgeback_autonomy/"
```

([`cleanup.sh:95`](../../cleanup.sh:95)) is a `pgrep -f` substring match, and
`kill_matches` is `kill -9` excluding only `$$` and `$PPID`. The configurator's
installed copy lives at
`install/ridgeback_autonomy/lib/ridgeback_autonomy/target_benchmark_configurator`,
so it matches.

`target_benchmark_sweep` calls `run_preflight_cleanup` before Gazebo starts
([`target_benchmark_sweep.py:628`](../../src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/target_benchmark_sweep.py:628)),
so a GUI-launched sweep would `kill -9` the GUI mid-click. Do **not** work around
it with `--skip-preflight-cleanup`; that flag produces the two-`gz sim` failure
where every trial dies with "Target pose ... not present". The catch-all must be
narrowed to actual nodes first. Full description in `docs/ISSUES.md`.

## Why live is riskier than the three profiles already running

| Profile | Executor | Spawn risk | Why |
|---|---|---|---|
| `measurement` | `target_replay_benchmark` | Low | No `rclpy`, no Gazebo, no X. Pure CPU subprocess. |
| `mask-output` | `target_replay_benchmark` | Low | Same. |
| `mask-model` | `target_replay_benchmark` | Low | Same, plus segmentation model load. |
| `live-system` | `target_benchmark_sweep` | High | Gazebo + RViz + ffmpeg screen capture, hours long, `kill -9` preflight, needs a GPU-backed X session |

## Phase 2 — Start for `live-system`

### 2.1 Progress is already on disk

The sweep writes `sweep.json` and regenerates `summary.md` after **every
config** ([`target_benchmark_sweep.py:726`](../../src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/target_benchmark_sweep.py:726)),
carrying per-config `status`, `error`, `wall_time_sec` and `real_time_factor`.
The GUI polls that file. No log-streaming protocol to invent — this is the
single biggest reason Phase 2 is smaller than it looks.

Note the filename is `sweep.json`, not `manifest.json`.

### 2.2 Locating the sweep directory

`sweep_dir = os.path.join(output_root, f'{_timestamp()}_{spec.name}')`
([`target_benchmark_sweep.py:601`](../../src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/target_benchmark_sweep.py:601)),
or an existing directory when `_find_resumable_sweep` matches. Parse it from the
log lines `Created sweep directory <path>` / `Resuming incomplete sweep <path>`
rather than recomputing the timestamp, which would race. The run log the
supervisor already writes is the place to read them from.

### 2.3 GL preflight

`software_gl_warning` fires immediately before Gazebo starts. An llvmpipe
fallback does not fail the run — it silently reduces every rendered sensor,
taking the camera from ~28 Hz to ~4 Hz, and RTF does not reveal it. The GUI
inherits whatever `DISPLAY`/GL environment its own shell had, which for an SSH
or reconnected-xrdp session may not be the GPU one.

Surface the renderer **in the UI before Start**, not in a log nobody opens. An
operator should not discover a software rasterizer six hours in.

### 2.4 Concurrency, harder

Server-side single flight already refuses a second GUI-owned run. The live guard
must additionally detect a sweep **nobody started from the GUI**
(`pgrep -af "gz sim"`), and say what it found rather than refusing blankly — two
simultaneous sweeps is the documented catastrophic case, where the second
`gz sim` server makes every trial fail with "Target pose not present".

### 2.5 Detach and re-attach

Already provided by the Phase 0 substrate: records live on disk, liveness is
cmdline-verified, and the tab re-attaches on load. Phase 2 needs only to add the
`sweep.json` view on top. Verify tab close, browser restart and GUI restart
against a real multi-hour sweep before calling this done.

### 2.6 Dry run

The GUI still punts the live estimate:

```python
'capture_duration': 'Live sweep; use its --dry-run estimate.'
```

`target_benchmark_sweep --dry-run` validates and estimates without launching.
Wire it to a Dry run button and show the real number before the operator commits
the machine for an afternoon. Cheap, and it is the natural safety gate in front
of a Start button this expensive. `--dry-run` does not run preflight cleanup, so
this part is **not** blocked and could ship first.

## Non-goals

- Reimplementing any validation in `app.js`. The browser owns presentation and
  draft state; the server canonicalises and validates through the same parser
  the CLI uses. Adding an axis must still require no front-end change.
- An operator toggle for `--skip-preflight-cleanup`. It is the flag that produces
  the two-`gz sim` failure. The one legitimate use — keeping another session
  alive — stays a CLI-only escape hatch.
- Editing or deleting results beyond the rename that already shipped.
- Changing the profile contract, the job format, or `replay_profiles.py`.

## Testing

- dry run returns an estimate without launching anything
- the sweep directory is read from the log lines, not recomputed
- a software-GL session is reported before Start, not after
- an externally started `gz sim` blocks Start and is named in the refusal
- a sweep survives GUI restart and re-attaches with its per-config progress

Run pytest from the repo root or `msg` imports fail; source
`/opt/ros/jazzy/setup.bash` then `install/setup.bash`.

## Build and edit notes for the implementer

`--symlink-install` behaves three different ways in this package; the rule is in
`AI_CONTEXT.md`. The short version: `configurator.py` is an
`install(PROGRAMS ... RENAME)` **copy**, so it needs `colcon build` and must be
verified by grepping
`install/ridgeback_autonomy/lib/ridgeback_autonomy/target_benchmark_configurator`
— not `site-packages`, which is a symlink and shows the edit either way.

Several Claude sessions share this working tree. Re-check `git status` before
blaming a failure on your own diff.
