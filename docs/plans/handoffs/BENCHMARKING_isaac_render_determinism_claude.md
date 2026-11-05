# BENCHMARKING — Isaac render determinism handoff (Claude)

**Author:** Claude Code session on the workstation (user `stefi`)
**Written:** 2026-11-04, about 23:00 CET · **Repository HEAD:** `b66f6d11a5e02746a7cc14a2a811fd34166fdad5`
on `dev`, **plus an uncommitted diff of 10 files** (section 4). Nothing is pushed.
**Update 2026-11-05:** the section 4 diff is now commit
`d8eef45629b7530de5445dfd9ba80f27477e8482`; the rest of this record stays as written.
**Governing plan:** none yet. This work follows from the unattended Gazebo/Isaac
qualification campaign; see
[Unattended qualification](../../target_distance_benchmarking/unattended_qualification.md).
**Related:** [Isaac camera and depth](../../isaac/camera-depth.md),
[Isaac LiDAR pipeline](../../isaac/lidar-pipeline.md),
[Target-distance benchmarking](../../target_distance_benchmarking/overview.md)

Evidence folders under `artifacts/benchmarks/` are on this workstation and are
not tracked by git.

## 0. Goal of the next chat

Isaac Sim results are not repeatable between two independent simulator sessions,
while Gazebo results are bit-identical. The next chat should:

1. run the two-session render test (section 5) on new candidate fixes;
2. support the user in choosing and implementing a fix (section 6);
3. once a fix is chosen, make it the benchmark setting and prepare, **but not
   start**, a rerun of the 8 Isaac conditions (section 6.3);
4. document the process and the decision at the end (section 8).

The user decided: **keep the Isaac LiDAR noise model**; GUI stays on for the
qualification campaign; all 8 Isaac conditions are rerun once a render setting is
chosen.

## 1. State at handoff

| Item | State |
|---|---|
| Qualification campaign | **Complete.** Bundle `artifacts/benchmarks/qualification_20261104_b66f6d1_restart11`, status `execution-complete-review-required`. Live 16/16 conditions, offline replay of the 8 first-repetition datasets passed (max per-event difference < 1e-6 m, exact repeat). `REPORT.md`, `comparisons.json`, `campaign.json`, `offline_inputs.json` hold the evidence. 11 464 s of its 12 h allocation remain. |
| Processes | Nothing of this work is running. Test units `ridgeback-isaac-determinism-*` are inactive. |
| Code | HEAD `b66f6d1` plus the uncommitted renderer/anti-aliasing options (section 4). Defaults are unchanged, so behaviour equals HEAD unless the new settings are passed. |
| Display | GUI runs need the user's xrdp session (`DISPLAY=:12.0` at handoff; check `loginctl list-sessions`). Two campaign stops came from the xrdp session ending, so verify the session before each run. |
| Shared GPU | Other users (digit, pratham, samer, deivid) run GPU jobs intermittently. They slow runs but do not invalidate timestamp-matched comparisons. |

## 2. Background: what the campaign showed

### 2.1 Repeatability between the two sessions (r1 vs r2)

Source: `restart11/comparisons.json` and per-trial CSVs of the runs listed in
`restart11/offline_inputs.json`.

| | Gazebo | Isaac |
|---|---|---|
| Stereo projective/euclidean | identical (0 m, 0 outcome changes) | median Δ ≈ 0, 1–2 outcome changes, max 0.67 m |
| Monocular projective/euclidean | identical | median Δ 0.09–0.12 m, 2–6 outcome changes, max 6.2 m |
| Polar | median ≈ 2 mm (LiDAR noise) | median 2–3 mm, a few large outliers |

Pairs with both repetitions scored: Gazebo 0 of 1197 differ by more than 0.8 m.
Isaac 19 of 1308 differ by more than 0.8 m and 18 by more than 1.0 m: 13 in box +
monocular, 5 in silhouette + monocular, 1 polar in box + stereo (`chaos_02`,
4.13 vs 2.93 m).

### 2.2 Mechanism

- **Isaac renders different RGB in each session.** Same trial, same camera pose
  (Δ 0.0000 m): `interfere_control_01` mean pixel difference 3.4 grey levels, 16.5 %
  of pixels > 10; `chaos_03` 1.3 / 1.6 %. Gazebo frames are bit-identical.
- **Renderer depth (Isaac "ideal" stereo) is unaffected**, so stereo only flips in
  borderline scenes.
- **The detector mostly works.** In the largest monocular differences both runs
  detected the target in ~96–100 of ~99 frames with nearly the same box. Detector
  flips are limited to three heavily occluded or partial scenes:
  `objpartial_03`, `interocc_moderate_02`, `interocc_neartotal_02`.
- **Monocular amplifies small image changes.** Depth Anything V2 Metric Indoor
  Small, compared with Isaac renderer depth inside the detector box: target bias
  +1.3 m at ~4 m, +5.5 m at 9 m; target depth spread (IQR) 0.45–2.1 m against
  0.03–0.07 m true; 53–61 % of box pixels smeared between target and background.
- **Isolation knife-edge.** `nearest_significant_mode`
  (`src/ridgeback_localization/ridgeback_localization/core/depth_common.py:118`) takes
  the nearest 5 cm bin holding ≥ 5 % of the box pixels, else the nearest non-empty
  bin. With monocular depth no target bin reaches 5 % (3.4–4.0 %) and wall bins sit
  at 3.0–3.6 %. When a wall bin crosses 5 % the wall is isolated. Example
  `interfere_control_01` box + monocular: sampled batches agree (5.16 vs 5.18 m),
  full-window estimates 11.41 vs 5.25 m (truth 4.14 m). The flip frames were not in
  the 5 stored batches, so this mechanism is inferred, not directly observed.

### 2.3 Why Isaac ran in realtime mode

Not a deliberate choice. `sim_mode` defaults to `realtime` in
`benchmarking/backends.py` (since `e33b7bd`), the 2026-10-26 qualification listed it
under "Isaac defaults", and this campaign copied that. The runner documents
`deterministic` as the mode for reproducible A/B benchmarks.

## 3. Experiments done on 2026-11-04 (tested revision `b66f6d1` + section 4 diff)

All under `artifacts/benchmarks/isaac_determinism_test_20261104_b66f6d1/`. Scenes
`interfere_control_01` and `chaos_03`; conditions box + stereo and box + monocular;
settings otherwise identical to the campaign's `isaac_r1` (640×480, OWLv2 0.30 at
10 FPS, lab background, ideal depth, GUI on, rtf 1.0), `sim_mode: deterministic`.

### 3.1 Deterministic mode, 5 stored batches (`session_a`, `session_b`)

Renderer depth identical 10/10. RGB identical 0/20 (mean 0.34–0.83 grey levels,
27–46 % of pixels). Even at equal simulation stamps (`chaos_03` box + stereo) RGB
differed on ~27 % of pixels. About 5× smaller than realtime. LiDAR scans differed in
every batch (noise model, kept by user decision).

Lesson: with wall-clock capture windows the 5 stored batches are often from
different simulation times between sessions (`rtl_a_5batch`/`rtl_b_5batch` had 0
equal stamps), so later tests store every batch.

### 3.2 Full-window, timestamp-matched comparison (`capture_batches: 100`)

| Pair | Renderer | Condition | Matched frames | RGB identical | Median mean Δ | Median pixels changed | Renderer depth | Depth Anything depth |
|---|---|---|---:|---:|---:|---:|---|---|
| `def_a/b` | RealTimePathTracing + DLSS (Isaac default) | stereo | 136 | 0 | 0.77 | 43 % | max Δ 1e-5 m | — |
| `def_a/b` | same | monocular | 0 | — | — | — | — | — (no matching stamps; not measured) |
| `rtl_a/b` | RaytracedLighting, AA 0 | stereo | 64 | 0 | 0.30 | 22 % | identical 64/64 | — |
| `rtl_a/b` | same | monocular | 103 | 0 | 0.73 | 36 % | — | max Δ 2.58 m, mean 0.40 m |
| `pt_a/b` | PathTracing, AA 0, denoiser off | stereo | 112 | 0 | **0.08** | **8 %** (max 9 levels) | identical 110/112 | — |
| `pt_a/b` | same | monocular | 114 | 0 | 0.44 | 34 % | — | max Δ 0.75 m |

Two-trial wall times (box + stereo): default 58 s, RaytracedLighting 57 s,
PathTracing 92–98 s. Deterministic-mode RTF 0.25–0.47.
`pt_a_interrupted` is an aborted run (user pause); ignore it.

### 3.3 Working hypothesis

Deterministic mode fixes only the simulation loop (`omni.kit.loop` manual mode with
`name="main"`, `isaac_runner.py:321-336`). The comment there records that the
`rendering_0` and GUI `present` loops stay wall-clock-tracked. Every tested renderer
builds an image partly from earlier frames (temporal anti-aliasing, path-tracing
accumulation, denoiser history), so an image depends on how many render passes
happened before it, which depends on GPU load. This fits the data: the monocular
condition, which runs Depth Anything on the same GPU, differs most, and other users'
GPU load changed per-window batch counts (67–100). Not yet proven.

## 4. Uncommitted code (10 files, +69/−8)

Adds two Isaac backend settings, `renderer` (`RealTimePathTracing` default,
`RaytracedLighting`, `PathTracing`) and `anti_aliasing` (`3` default; `0` off,
`1` TAA, `2` FXAA, `3` DLSS, `4` RTXAA), passed from a sweep YAML to `SimulationApp`.

| File | Change |
|---|---|
| `src/ridgeback_autonomy/ridgeback_autonomy/benchmarking/backends.py` | two `BackendSetting`s on the Isaac backend |
| `src/ridgeback_autonomy/ridgeback_autonomy/localization_launch.py` | names added to `CONFIG_LAUNCH_ARGUMENT_NAMES` and `ENV_LAYER_CONFIG_KEYS` |
| `src/ridgeback_autonomy/launch/target_benchmark_env.launch.py`, `target_benchmark_config.launch.py` | `DeclareLaunchArgument`s |
| `src/ridgeback_autonomy/launch/includes/simulation.launch.py` | forwarded to the Isaac adapter |
| `src/ridgeback_autonomy_isaac/launch/backend.launch.py` | declared and passed as `--renderer`, `--anti-aliasing` |
| `src/ridgeback_autonomy_isaac/sim/isaac/isaac_runner.py` | argparse options; `app_cfg` gets `renderer`, `anti_aliasing`; `denoiser: False` for PathTracing (`:151`) |
| `src/ridgeback_autonomy/test/test_benchmark_backends.py`, `test_launch_layout.py`, `test_sweep.py` | expected setting sets/defaults |

Verified: `test_benchmark_backends`, `test_launch_layout`, `test_sweep`,
`test_benchmark_scene_control` pass except
`test_dependency_profiles_keep_gazebo_out_of_the_common_closure`, a known host-only
failure on the untouched tree; `src/ridgeback_autonomy_isaac/test` 71 passed, 49
skipped. The launch files and `isaac_runner.py` are symlinked into `install/`, and
`backends.py`/`localization_launch.py` are modules, so no rebuild was needed. Not
committed: commit it (and rebuild graphify) before any campaign run, because the
campaign requires a clean tree.

## 5. The two-session render test

### 5.1 Files (in the test folder)

- `run_session.py SESSION_DIR`: one fresh Isaac session; runs `SESSION_DIR/sweep.yaml`
  through the installed `target_benchmark_sweep` with the campaign environment
  (`campaign.environment(..., 'isaac')`: ROS domain 127, CycloneDDS profile, offline
  model hubs, Gazebo adapter removed from the ament path) wrapped in `tools/gpu-run`.
- `session.sh SESSION_DIR`: sources ROS and the workspace, runs `run_session.py`,
  writes `SESSION_DIR/session.log`.
- `run_pair.sh A B`: runs session A, then B, each as a transient user unit
  `ridgeback-isaac-determinism-<name>` (with `reset-failed` first), then
  `compare.py A B`. It captures `DISPLAY`, `XDG_RUNTIME_DIR` and
  `DBUS_SESSION_BUS_ADDRESS` **at the time the script was written**. Regenerate it, or
  edit those values, if the user's session changes.
- `compare.py A B`: pairs frames of the same trial by simulation stamp, compares RGB,
  renderer/Depth Anything depth, detections, scans and per-trial CSV estimates; writes
  `comparison_A_B.json`.
- `<name>/sweep.yaml`: one per session. Copy `pt_a/sweep.yaml`, change `name`,
  `output_dir`, `sensor_capture_dir` and the settings under test.

### 5.2 Run

```bash
cd /home/stefi/DyNAMO/DyNAMO-Ridgeback
```

```bash
bash artifacts/benchmarks/isaac_determinism_test_20261104_b66f6d1/run_pair.sh pt_a pt_b
```

A pair takes about 6 minutes. Run it in the background and wait for the shell to
exit. A unit can be stopped cleanly with
`systemctl --user stop ridgeback-isaac-determinism-<name>.service`.

### 5.3 Pass criteria

- RGB bit-identical for every timestamp-matched frame in **both** conditions
  (monocular is the stricter case: it shares the GPU with Depth Anything).
- Renderer depth identical.
- Enough matched frames (≥ 50 per condition). If a pair has none, the sessions
  ran at different speeds; rerun or investigate stamp alignment.
- Scans will differ (LiDAR noise, kept); detections and estimates should follow RGB.
- Sanity: detection rate on these scenes should remain plausible against the
  campaign (`interfere_control_01` detected in ~all frames in Isaac).

### 5.4 Gotchas

- The first monocular trial of a configuration often has no stored depth (Depth
  Anything warm-up; the mask worker keeps only its newest pending batch).
- Wall-clock windows: stored frames differ in simulation time when GPU load differs;
  always compare by stamp.
- A test unit stopped with `systemctl stop` stays loaded as failed; `run_pair.sh`
  now clears it.
- Run pytest with `ROS_DOMAIN_ID=77`; Isaac needs `ROS_DOMAIN_ID` ≤ 232.

## 6. Next steps

### 6.1 Option 1 — lock rendering to simulation steps (likely real fix, needs investigation)

Make each published camera image depend only on the scene, not on render timing:

- `isaac_runner.py:321-336` sets manual mode only for `name="main"`. Investigate
  driving the render loop (`rendering_0`) in lockstep too, or rendering a fixed
  number of subframes per simulation step before the camera publishes (Replicator
  `rt_subframes`/orchestrator step).
- The camera publishes through an OmniGraph `OnPlaybackTick` →
  `ROS2CameraHelper` graph (`sensors.py:284`, `:335-350`) on render products from
  `_render_product` (`sensors.py:219`). A locked render must keep that publication
  working and stamped by simulation time.
- For PathTracing, check whether accumulation can be reset each frame with a fixed
  sample count, so a frame never inherits samples from earlier frames.
- Use the two-session test (section 5) as the acceptance check.

### 6.2 Option 2 — PathTracing plus statistics (pragmatic fallback)

Benchmark Isaac with `sim_mode: deterministic`, `renderer: PathTracing`,
`anti_aliasing: 0`: about 9× less RGB noise with stereo, renderer depth practically
identical. Treat the remaining variation as sensor noise (≥ 3 Isaac repetitions) and
make monocular isolation robust to spread depth, for example a floor relative to the
fullest bin, range-scaled bins, histogram smoothing or temporal consistency. Estimator
changes are user-gated: discuss each one with the user before committing, and do not
edit the shared ranging defaults on your own.

### 6.3 After a fix is chosen (user instruction: prepare, do not start)

1. Make the chosen settings explicit in
   `tools/benchmark/qualification/sweeps/isaac_r1.yaml` and `isaac_r2.yaml`
   (`sim_mode` and the render settings). Decide with the user whether the benchmark
   backend defaults in `backends.py` should change; exploration keeps `realtime`.
2. Commit, run `bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"`,
   commit the graph.
3. `campaign.py prepare` a new bundle, then `campaign.py import` all 8 Gazebo
   conditions from their source bundles (identical settings, so the import check
   accepts them):
   - `gz_r1/box_stereo`, `gz_r1/silhouette_stereo`: `qualification_20261103_4e22a20`
   - `gz_r1/box_monocular`: `qualification_20261103_6d7817c_restart4`
   - `gz_r1/silhouette_monocular`, `gz_r2/box_stereo`: `qualification_20261103_6d7817c_restart6`
   - `gz_r2/silhouette_stereo`, `gz_r2/box_monocular`, `gz_r2/silhouette_monocular`:
     `qualification_20261104_8321456_restart8`

   `check_gz` is then skipped automatically; `check_isaac`, `isaac_r1` and `isaac_r2`
   run. Run `campaign.py check`.
4. **Budget is a user decision.** The carried allocation has 11 464 s left, less than
   8 Isaac conditions need (~24 min each with the default renderer; PathTracing is
   slower). A rerun with new render settings is a new experiment; ask whether it gets
   its own allocation.
5. Do not run `campaign.py start`. The external GPU policy was `record` in the
   final campaign; ask whether to keep it.

## 7. Constraints and decisions to respect

- Keep the Isaac LiDAR noise model (user, 2026-11-04).
- All 8 Isaac conditions are rerun after a render choice; Gazebo results stay.
- Prepare only; the user starts campaigns.
- Never edit `campaign.py` or other frozen-set files while a campaign runs.
- Installed `RENAME` executables (node scripts, `target_benchmark_sweep`, etc.) are
  copies: run `colcon build --symlink-install --base-paths src --packages-select ...`
  after editing them, and verify the installed copy. `campaign.py check_installed()`
  checks the benchmark set.
- Run `tools/check_dependencies` before builds; keep the `COLCON_IGNORE` markers.
- No push. Commit messages end with the session's attribution line.

## 8. Documentation at the end

- **Archive record** for the investigation and decision, under
  `archive/engineering/` (for example `2026-11-04-isaac-render-determinism.md`),
  following [documentation policy](../../project/documentation.md#archived-evidence):
  title, then `Recorded dates`, `Tested revisions` (full hashes; the 2026-11-04 tests
  ran on `b66f6d1` plus the section 4 diff, so preserve that patch or commit first),
  `Provenance`. Keep method, conditions, the tables in section 3, unfavourable
  results and limits.
- **Current references:** add `sim_mode` and the render settings to the "Choosing a
  mode" table in [Isaac camera and depth](../../isaac/camera-depth.md); record the
  chosen benchmark settings and why in the target-distance benchmarking docs and
  [Unattended qualification](../../target_distance_benchmarking/unattended_qualification.md).
- **Backlog:** monocular isolation robustness and monocular scale bias if not fixed;
  link from [target-distance BACKLOG](../../target_distance_benchmarking/BACKLOG.md).
- **Troubleshooting:** "Isaac results differ between sessions" with the render-history
  diagnosis, if it stays relevant.
- Rebuild graphify after code changes only, not documentation-only edits.
- Move resolved findings out of this handoff into those references; this file stays a
  dated snapshot.
