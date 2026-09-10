# Continue the Isaac Sim 6.0 port — validate the post-P7 batch, then P8

You are picking up an in-flight port of a Ridgeback autonomy stack from
Gazebo Harmonic to **Isaac Sim 6.0 GA**. Work happens ONLY in the worktree
`/home/deivid/dev/DyNAMO-Ridgeback/.claude/worktrees/isaac` on branch
`feat/isaac-sim-6-port` — the main checkout stays on its own branch.

## Read these first, in order

1. Your memory file `isaac-sim-6-port` (the 6.0.1 landmine list + the SLAM
   root-cause paragraph — trust it, do not rediscover). Also skim
   `nav-tuning-in-flight` and `shared-dev-box-contention`.
2. `tools/isaac/PORT_PLAN.md` — the authoritative 9-phase plan. **P0–P4 + P7
   done; P5 plumbing done, A/B sign-off open; a post-P7 batch landed
   2026-09-02 unvalidated; P6 deferred; P8 next.** Phase checkboxes are kept
   current; keep them current.
3. `tools/isaac/SLAM_QUALITY_REPORT.md` — source of truth for how the RTX
   lidar reaches ROS. The old P4 "270° ROI honored" claim was WRONG; the
   assembler synthesizes the 270° contract scan from two OmniLidar prims.
4. `AI_CONTEXT.md` (repo conventions, doc ownership: README/AI_CONTEXT/ISSUES)
   and `ISSUES.md` (troubleshooting + the lidar entry). Note: AI_CONTEXT.md
   still has **zero** Isaac content — that rewrite is P8 scope, not drift.

## State as of this handoff (2026-09-10)

- Branch tip `1a4079b9`, clean tree, in sync with `origin`, contains all of
  `master` (74 commits ahead, 0 behind).
- **Post-P7 batch (2026-09-02) is landed but NOT re-validated** — see the
  dedicated PORT_PLAN section. Three commits: `warehouse_full` GT map,
  `LidarScanAssembler` stale-bin fix + wheel `JointState`, and the new
  `scan_merger_node` (`slam_source:=merged`, default under `sim:=isaac`).
  They were measured in a since-discarded working tree; nothing has been run
  from this checkout against a fresh Isaac boot.
- **P5 A/B remains unsigned.** Not a code bug — a measurement problem.
  Coverage variance is 51–83% run-to-run; `odom_noise=1.0` costs ~10 pts and
  2× aborts; residual drift is scan-match rotation. Firm numbers need 3–5
  seeds per condition on a single-tenant box, which this one never is.
- **P6 untouched.** The G1 benchmark still runs the gz spawn/remove/pose
  plumbing.

## Your objective

1. **Validate the post-P7 batch, `odom_noise:=0` first.** Perfect odom is the
   right starting condition: the merger motion-compensates the rear scan
   through the odom→base_link chain, so zero-noise odom isolates the merge
   geometry from the drift lever P5 already characterized. Only then repeat
   with realistic noise. Check the merger's own log line
   (`front=/paired=/front_only=/tf_misses=/delta_abs_*`) before trusting any
   downstream map — a high `front_only` or `tf_misses` count means the merge
   is silently degrading to front-only.
2. **Then P8** (Gazebo removal + docs + graphify) — scope in the plan.

## How to run the stack (learned the hard way)

```bash
cd <worktree root>
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
# sim (headless):
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/simulation_isaac.launch.py world:=mock_hospital
#   + odom_noise:=0 to isolate odom drift; headless:=false for windowed
# full exploration:
bash start_exploration.sh warehouse_full sim:=isaac sim_mode:=deterministic \
  camera:=false g1_perception_enabled:=false
```

**Benchmark hygiene is mandatory** (it was wrong for months): `camera:=false`
(the runner used to render the D455 unconditionally, RTF 0.33→0.65),
`g1_perception_enabled:=false`, an isolated `ROS_DOMAIN_ID` (a co-tenant ran
a `/r100_0001` stack on domain 42 whose `hud_node` publishes the same
`hud/coverage` the probe reads), and `setup_path:=/tmp/bench-clearpath/`. Use
`sim_mode:=deterministic` for anything A/B — realtime mode couples frame_dt to
render-wall duration, so contention smears the lidar sweep.

Sim warm-boots in ~1 s if kit is cached; cold boot up to ~4 min. Measure all
rates in **sim time**, not wall-clock (RTF ≈ 0.5–0.8 under load makes 40 Hz
look like 20 Hz on the wall).

## Landmines that will cost you an hour each if you forget

- **`pgrep -f` / `pkill -f` in a wait loop matches its own bash wrapper** →
  reports "process down" while it is still alive → two sims silently overlap
  and poison every measurement. Use `ps aux | grep X | grep -v grep | wc -l`
  in loops; kill by explicit PID; put kill calls in a separate tool call from
  launches.
- **A stale `ros2 daemon` hides namespaced topics from the CLI** (`ros2
  daemon stop`, or `--no-daemon`). rclpy probes are unaffected but still need
  the CycloneDDS env above.
- **The user's VNC display number rotates per session** (was `:0`, then `:3`).
  Before any windowed launch or rviz, detect it from a live session process's
  env: find a `xfce4-session`/terminal PID owned by `deivid`, read `DISPLAY`
  and `XAUTHORITY` from `/proc/<pid>/environ`. Never assume `:0`.
- **Shared box.** Check the GPU seat ACL (`getfacl /dev/dri/renderD128`)
  before any long GPU session, then co-tenant CPU/GPU load. One kit boot
  crashed spontaneously mid-investigation (breakpad); a plain relaunch fixed
  it — treat a single boot crash as flaky, don't over-diagnose.
- **The full 6.0.1 landmine list** is in your memory file (bridge-after-
  open-stage segfault, ArticulationRoot rules, timeline end-time, importer
  drops meshes/collisions, remote-anchor `AddReference` needing a `file://`
  URI, etc.). Read it before touching the runner or USD.

## When you finish a deliverable

Commit in repo style — **no AI-attribution trailers** (no `Co-Authored-By`,
no "Generated with"; the user is the author). Then rebuild graphify
(`GRAPHIFY_PYTHON=/home/deivid/dev/DyNAMO-Ridgeback/perception_venv/bin/python3
bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"`, or the
post-commit hook with that env var set), update the affected PORT_PLAN
section, and update the `isaac-sim-6-port` memory.

## Remaining phases (for context)

- **P6** — G1 distance benchmark: rewrite gz spawn/remove/pose plumbing onto
  `simulation_interfaces` Control services; recalibrate estimators for D455.
- **P8** — remove Gazebo entirely; fresh-clone drill; final docs.

The user works terse and expects you to act autonomously on reversible work,
stopping only for destructive actions or genuine scope decisions. They caught
a real bug by eyeballing rviz — show them results they can eyeball, and verify
your own claims by measurement, not by reading the config.
