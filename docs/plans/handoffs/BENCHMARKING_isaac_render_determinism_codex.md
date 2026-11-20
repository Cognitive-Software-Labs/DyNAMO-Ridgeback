# BENCHMARKING — Isaac render determinism follow-up handoff (Codex)

Written: 2026-11-05, Europe/Berlin. This is an investigation handoff, not a
chosen fix or a qualification verdict.

## Start here

The user wants support fixing Isaac image differences between independent
simulator sessions, which can amplify into different monocular distance
estimates. Two further experiments are complete. **Strict RGB determinism
still fails, including in an isolated frozen-scene rendering probe.**

The most actionable new finding is that **requesting AA off did not keep it
off in the isolated probe**: Isaac restored global AA to `3` and camera AA
to `dlss`. Disabling the additional `limitedOps` restriction allowed us to
verify AA off. Whether this also affected earlier full-pipeline runs remains
untested. Check effective settings in that pipeline before assuming it did.

Read repository instructions in `docs/project/PROJECT_CONTEXT.md` and
`docs/project/conventions.md` independently; documentation ownership is in
`docs/project/documentation.md`. Read the graph report before broad architecture
analysis. Preserve the existing dirty tree and other users' processes.

Then read the [original Claude handoff](BENCHMARKING_isaac_render_determinism_claude.md).
It owns the earlier campaign background, renderer comparisons, proposed
approaches, and instructions for preparing a later campaign rerun. This file
adds the subsequent evidence and qualifies what we can conclude from it.

## User intent and decisions

The user authorized running the original two-session test, then authorized a
rendering-only experiment after this explanation: freeze scene and camera,
clear/control accumulation, request fixed rendering work, wait for completion,
save RGB, restart Isaac, and compare raw images. Both experiments were executed.
The current request is to create this handoff for another agent to help resolve
the problem. No production fix was selected in this chat.

Carry forward the original decisions:

- Keep Isaac LiDAR noise enabled; differing scans are expected.
- Keep GUI on for the qualification campaign; these probes also used the GUI.
- After a render choice, rerun all eight Isaac conditions and retain Gazebo
  results. Prepare the campaign only; the user starts it.
- Campaign budget and external GPU policy require the user's decision; the
  original handoff's carried budget is historical, not a fresh allocation.
- Discuss estimator changes with the user before committing them. Do not
  silently change shared ranging defaults to hide image variation.
- No push. Follow the session attribution convention if committing later.

## Experiment 1: repeat of the full two-session test

Recorded: 2026-11-04 CET. Tested HEAD:
`b66f6d11a5e02746a7cc14a2a811fd34166fdad5` plus the dirty workspace preserved
with the run. This used the renderer options already present in the workspace;
no candidate production fix was added.

Evidence root:
`artifacts/benchmarks/isaac_determinism_test_20261104_b66f6d1/`.
Fresh sessions: `pt_verify_20261104T222250Z_a` and
`pt_verify_20261104T222250Z_b`.

- [Report](../../../artifacts/benchmarks/isaac_determinism_test_20261104_b66f6d1/pair_20261104T222250Z/REPORT.md)
- [Summary, including every estimator/instance](../../../artifacts/benchmarks/isaac_determinism_test_20261104_b66f6d1/pair_20261104T222250Z/summary.json)
- [Raw matched-frame comparison](../../../artifacts/benchmarks/isaac_determinism_test_20261104_b66f6d1/comparison_pt_verify_20261104T222250Z_a_pt_verify_20261104T222250Z_b.json)
- The pair folder also contains `provenance.json` and `workspace.patch`.

Conditions: `interfere_control_01`, `chaos_03`; box + stereo and box + monocular;
640×480, deterministic simulation, PathTracing, requested AA 0, denoiser off,
GUI on, campaign-style OWLv2 and Depth Anything pipeline. Compare by simulation
timestamp, never batch position.

| Check | Stereo | Monocular |
|---|---:|---:|
| Timestamp-matched frames | 100 | 80 |
| RGB identical | 0/100 | 0/80 |
| Depth identical | 98/100 | 0/77 |
| Maximum depth difference | 4.76837158203125e-7 m | 0.27983665466308594 m |
| Median mean RGB channel difference, 0–255 | 0.04696 | 0.08412 |
| Maximum RGB channel difference | 9 | 10 |

Stereo depth here is renderer depth; monocular depth is Depth Anything output.
Three monocular pairs had no depth on either side. Camera translations matched
for every compared frame. Both sessions completed all four scene/condition
trials with no skips and exited successfully. All per-instance estimator
outcomes matched; maximum estimate difference was **18.858 mm**. This pair did
not reproduce the much larger campaign outliers described in the original
handoff, but it failed the original bit-identity acceptance criteria.

Important comparison-tool pitfalls:

- Existing `compare.py` keeps instance identity while reading CSVs but omits it
  in its final trial-output keys, overwriting `chaos_03` instance 0 with another
  instance. Use `summary.json`'s `estimate_comparison` for complete estimates.
- Its RGB `share_changed` counts channel elements, not whole RGB pixels.
- This run recorded the **requested** AA setting, not a verified effective AA
  value after renderer initialization. The later AA discovery does not establish
  what was effective in this earlier pair.

## Experiment 2: rendering only, frozen scene

Recorded: 2026-11-05 CET; artifact timestamp is UTC on 2026-11-04.
Tested HEAD: `1a96754918cf3d6c7920a0c9994be64b4907c8db` plus the preserved dirty
workspace. Installed Isaac `VERSION` reported
`6.1.0-rc.26+release.49347.2d230af4.gl`; consult the manifests for exact runtime
identity rather than assuming documentation version labels establish it.

Evidence root: `artifacts/benchmarks/isaac_render_only_20261104T231224Z/`.

- [Report](../../../artifacts/benchmarks/isaac_render_only_20261104T231224Z/REPORT.md)
- [Comparison](../../../artifacts/benchmarks/isaac_render_only_20261104T231224Z/comparison.json)
- [Final probe source](../../../artifacts/benchmarks/isaac_render_only_20261104T231224Z/probe_fixed_aa.py)
- [Comparison source](../../../artifacts/benchmarks/isaac_render_only_20261104T231224Z/compare.py)
- Valid sessions: `fixed_a`, `fixed_b`; each contains ten `frame_*.npz`, a
  `manifest.json`, `preview.png`, and `frozen_scene.usda`.
- `provenance.json`, `workspace.patch`, and `probe_sha256.json` preserve the
  starting revision/diff and final script hashes. Evidence is workstation-local
  and ignored by git; a handoff commit alone will not transport it.

Method:

1. Start a fresh GUI-enabled `SimulationApp`; load the repository's
   `g1_distance_calibration.usda` and a static G1 at 4 m. Use an explicitly
   aimed 640×480 camera. This is a simpler room, not a reconstruction of either
   furnished qualification scene.
2. Keep timeline paused at 0. No physics playback, ROS graph, scene-control
   service, detector, monocular network, or ranging pipeline.
3. PathTracing; `spp=32`, `totalSpp=32`; denoiser, adaptive sampling and motion
   blur disabled. Configure accumulation reset on external frame-counter change.
4. Ten warmup captures, followed by ten saved captures. Each requests:
   `rep.orchestrator.step(rt_subframes=1, delta_time=0., pause_timeline=True,
   wait_for_render=True)`.
5. Verify nonblank outputs, frozen time, global AA 0 and render-product AA
   `none`. Save raw RGB and geometric depth, hashes, counters and effective
   settings. Close Isaac; repeat in a second fresh process.

| Check | Final valid pair |
|---|---|
| RGB bit-identical pairs | 0/10 |
| Geometric depth bit-identical pairs | 10/10 |
| Pixels with any changed RGB channel | 0.285–0.314% |
| Maximum RGB channel difference | 1 on a 0–255 scale |
| Mean absolute channel difference | 0.001432–0.001572 |
| Simulation times | 0 throughout |
| Render counters | Matched across sessions for every compared capture |
| Within-session hashes | 10 distinct RGB hashes; one depth hash per session |

Both final units exited status 0. The composed scenes match after content-based
normalization of USD flattened prototype numbering/order and exclusion of the
viewport-only `viewPickingId`. Raw flattened file hashes differ; that alone
does not indicate different geometry. The comparison source implements this
normalization and records the canonical hashes.

**Conclusion:** small RGB variation remains in the isolated rendering path even
with confirmed AA off and this fixed-budget capture procedure. This does not
identify its source. Matching counters and configured sample budgets are not
independent GPU sample-count instrumentation, and no fixed renderer RNG seed
was established. Stochastic sampling and post-processing remain hypotheses.
Do not claim a measured improvement factor over the full pipeline: the scene
and workload differ. No test yet shows the effect on monocular estimates.

## AA finding: reproduce and verify before changing the pipeline

During setup, both global AA 0 and a per-render-product `none` override reverted
to global 3 / `dlss`. Reapplying them after warmup alone did not solve it.
The final probe disabled the restriction:

```text
startup argument: --/rtx-transient/post/aa/limitedOps=false
Carb setting:    /rtx-transient/post/aa/limitedOps = False
render product: omni:rtx:post:aa:limitedOps = False
render product: omni:rtx:post:aa:op = "none"
```

The final probe reapplies the requested controls before saved captures and
verifies the effective values afterwards. Inspect its source for exact ordering.
The installed vendor implementation is useful primary evidence:

- `isaac_venv/lib/python3.12/site-packages/isaacsim/extscache/omni.replicator.core-1.13.36+110.3.0.lx64.r.cp312/omni/replicator/core/scripts/settings.py`:
  `set_render_rtx_realtime()` clears `limitedOps` for non-DLSS AA choices.
- The adjacent `scripts/orchestrator.py` documents `step()`, render completion,
  sample settings and external-frame-counter accumulation control.
- `isaacsim/exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py`
  under the same site-packages root documents `reset_render_settings()` after
  stage changes and `close(exit_code=...)`.

This is a concrete setup issue in the probe, not proof of the production
pipeline's effective AA state or proof that clearing it fixes determinism.

## Recommended next investigation — not yet implemented

1. Inspect the current
   [runner](../../../src/ridgeback_autonomy_isaac/sim/isaac/isaac_runner.py) and
   [camera setup](../../../src/ridgeback_autonomy_isaac/sim/isaac/sensors.py).
   Instrument effective global and camera render-product AA, `limitedOps`,
   renderer, sample/denoiser/adaptive settings **after initialization and during
   capture**. Requested launch arguments alone are insufficient evidence.
2. If AA is being overridden there, try the narrow Isaac-only correction and
   rerun a fresh two-session stereo/monocular pair. Preserve the control run;
   compare one change at a time. Verify image shape, timestamps and detection
   sanity as well as raw pixel differences.
3. Only if needed, prototype explicit render completion / accumulation control
   in benchmark mode. Current deterministic mode manually fixes the `main`
   loop; its comment explicitly says `rendering_0` and GUI presentation remain
   wall-tracked. That is a hypothesis route, not established root cause.
4. Camera helpers currently use `OnPlaybackTick` → `ROS2CameraHelper`. Extra
   render subframes must not advance physics twice, publish intermediate frames,
   break camera cadence, or mis-stamp completed images. Keep the GUI usable.
5. Investigate the remaining one-level RGB differences in the isolated probe
   only as useful to the user's repeatability objective. Possibilities include
   light-sampling state or post-processing; neither was verified. Do not relax
   bit-identity acceptance or alter estimators without discussing the tradeoff.

Original full-pipeline acceptance: RGB identical in every timestamp-matched
frame in both conditions; renderer depth identical; at least 50 matched frames
per condition; plausible detections; LiDAR differences expected. Separate
monocular network repeatability from renderer repeatability when interpreting
depth differences.

## Running safely and preserving evidence

Workspace: `/home/stefi/DyNAMO/DyNAMO-Ridgeback`.

The existing artifact launch scripts are **single-use examples with hardcoded
paths and unit names**. Do not simply rerun them against existing directories:
the probe calls `mkdir()` without `exist_ok`, while other wrappers can overwrite
logs. Copy/adapt them into a fresh experiment folder, preserving old scripts,
inputs, outputs and comparison results. Update both session names in the
comparison script. The full-pipeline pair needs new sweep `name`, `output_dir`
and `sensor_capture_dir` values for each session.

At the last experiment check, all of this chat's test units/processes had
exited. GUI was the user's active xrdp session at `DISPLAY=:12.0`, UID 1009,
`XDG_RUNTIME_DIR=/run/user/1009`, user bus `unix:path=/run/user/1009/bus`.
**Recheck these and GPU/process load before launching.** Shared-GPU jobs belong
to other users; do not terminate them. These probes launched as separate
transient user-systemd units with bounded runtimes.

Excluded setup attempts are retained in the render-only folder:
`initial_a/b` stopped on blank output before warmup; `a` captured under unwanted
DLSS; `aa_reset_attempt_a` and `aa_reapply_attempt_a` rejected restored DLSS.
The `a` attempt's non-fast extension teardown was stopped and exited 139; this
was not a successful render comparison. Final `fixed_a/b` exited normally.
Use `app.close(exit_code=code)` and verify output manifests: default fast
shutdown can otherwise obscure a probe exception with process status 0.

No production source/default changes or commits were made by these experiments.
Probe code and evidence live under ignored `artifacts/`; graphify was rebuilt.
At handoff writing, HEAD is `1a96754918cf3d6c7920a0c9994be64b4907c8db` with
pre-existing renderer edits plus concurrent hardware-bag validation work and
other dirty files. Reinspect `git status` and preserve unrelated edits. Do not
mistake the current tree for either tested state; use each run's provenance.

Build only when needed: follow the installed-copy versus symlink rules,
run `tools/check_dependencies` before builds, verify installed executables,
and rebuild graphify after production code changes. No rebuild is required
merely to run an independent artifact probe.

When a fix is selected, follow the original handoff for campaign preparation
and final durable documentation. Do not start the campaign from this handoff.
