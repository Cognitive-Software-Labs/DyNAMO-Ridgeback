# Continue the Isaac Sim 6.0 port — P5 (E2E exploration + A/B sign-off)

You are picking up an in-flight port of a Ridgeback autonomy stack from
Gazebo Harmonic to **Isaac Sim 6.0 GA**. Work happens ONLY in the worktree
`/home/deivid/dev/DyNAMO-Ridgeback/.claude/worktrees/jolly-borg-f48cab` on
branch `feat/isaac-sim-6-port` — the main checkout stays on `dev`.

## Read these first, in order

1. Your memory file `isaac-sim-6-port` (the 6.0.1 landmine list + the SLAM
   root-cause paragraph — trust it, do not rediscover). Also skim
   `nav-tuning-in-flight` and `shared-dev-box-contention`.
2. `tools/isaac/PORT_PLAN.md` — the authoritative 9-phase plan. **P0–P4 done,
   P5 is next.** Phase checkboxes are kept current; keep them current.
3. `tools/isaac/SLAM_QUALITY_REPORT.md` — the sensor pipeline was just
   overhauled (3 bugs fixed). This is now the source of truth for how the
   RTX lidar reaches ROS; the old P4 "270° ROI honored" claim was WRONG.
4. `AI_CONTEXT.md` (repo conventions, doc ownership: README/AI_CONTEXT/ISSUES),
   and `ISSUES.md` (troubleshooting + the new lidar entry).

## State as of this handoff (2026-07-12)

- **Just landed** (branch tip): SLAM-quality investigation. The rotation-
  smeared maps were NOT slam params — they were three Isaac 6.0.1 sensor
  bugs (bridge laser_scan writer hardcodes 360° for ROTARY; rotary model
  fires only 180°/tick so one prim is half-blind; runner starved ROS
  callbacks). Fixed in `66493670` + `e1862498`. Lidar now publishes point
  clouds from TWO OmniLidar prims per frame; `ros_io.LidarScanAssembler`
  bins them into the true 270° contract `LaserScan`. Verified: 40 Hz
  sim-time, full FOV, closed-loop pose RMSE 0.19–0.23 m, loop error 7–9 cm,
  map IoU ~0.55.
- Committed slam config change: `link_match_minimum_response_fine 0.1 → 0.8`
  in `slam_toolbox_params.yaml` (only surviving delta; travel-gating measured
  strictly worse, kept at 0.0/0.0).
- New reusable harness under `tools/isaac/`: `gt_occupancy.py` (analytic GT
  grid from SDF), `slam_quality_probe.py` (GT-feedback closed loop + metrics
  + overlay), `scan_geometry_check.py` (sensor regression check). These feed
  the P5 gate directly.
- Working tree is clean. There may be a windowed sim + rviz still running on
  the user's VNC display from the demo — check `ps aux | grep isaac_runner`;
  kill by PID if you need a clean box (`ps aux | grep X | grep -v grep`, NOT
  `pgrep -f` in a loop — it self-matches, see gotcha below).

## Your objective: P5 — E2E exploration + sign-off

Exact scope and acceptance are in `PORT_PLAN.md` §P5. Summary:

1. **`ridgeback_exploration.launch.py`**: forward a `sim:=gz|isaac` arg down
   the include chain; set `sim_ready_timeout` (45 s gz / 300 s isaac) on the
   first readiness gate. Add the HUD localization-error panel (GT pose vs
   SLAM `map→base_link`) — the TF-vs-GT computation already exists in
   `slam_quality_probe.py`, reuse it.
2. **`tools/isaac/ab_compare.py`** (new): gz-vs-isaac results table —
   coverage complete/accuracy, time-to-complete, achieved RTF, aborts,
   success, isaac localization error.
3. **Run the A/B matrix** on mock_hospital via the real exploration
   entrypoint (`bash start_exploration.sh mock_hospital sim:=isaac`), NOT the
   diagnostic probe — the probe was a SLAM-quality tool; P5 exercises the
   actual nav2 + frontier_explorer stack.

**Acceptance (from the plan):** 3/3 isaac runs complete; coverage-complete
≥ gz mean − 10 pts (loose — sensors differ by design); genuine aborts ≤ gz
max; achieved RTF ≥ 0.8 throttled; one `--rtf 0` run finishes faster
wall-clock. gz baseline numbers are in `tools/isaac/baseline/`.

Confirm scope with the user before you start building `ab_compare.py` — the
gz baseline capture and the exact comparison columns are the kind of thing
worth a 30-second check.

## How to run the stack (learned the hard way)

```bash
cd <worktree root>
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
# sim (headless):
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/simulation_isaac.launch.py world:=mock_hospital
#   + odom_noise:=0 to isolate odom drift; headless:=false for windowed
# slam standalone:
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/slam.launch.py use_sim_time:=true setup_path:=$PWD/clearpath/
```

Sim warm-boots in ~1 s if kit is cached; cold boot up to ~4 min. Measure all
rates in **sim time**, not wall-clock (RTF ≈ 0.5–0.8 under load makes 40 Hz
look like 20 Hz on the wall).

## Landmines that will cost you an hour each if you forget

- **`pgrep -f` / `pkill -f` in a wait loop matches its own bash wrapper** →
  reports "process down" while it is still alive → two sims silently overlap
  and poison every measurement. Use `ps aux | grep X | grep -v grep | wc -l`
  in loops; kill by explicit PID; put kill calls in a separate tool call from
  launches.
- **The user's VNC display number rotates per session** (was `:0`, then `:3`).
  Before any windowed launch or rviz, detect it from a live session process's
  env: find a `xfce4-session`/terminal PID owned by `deivid`, read `DISPLAY`
  and `XAUTHORITY` (`~/.Xauthority`) from `/proc/<pid>/environ`. Never assume
  `:0`.
- **Shared box.** Run the `box-health` agent before any long GPU session
  (GPU seat ACL `getfacl /dev/dri/renderD128` first, then co-tenant CPU/GPU).
  One kit boot crashed spontaneously mid-investigation (breakpad); a plain
  relaunch fixed it — treat a single boot crash as flaky, don't over-diagnose.
- **The full 6.0.1 landmine list** is in your memory file (bridge-after-
  open-stage segfault, ArticulationRoot rules, timeline end-time, importer
  drops meshes/collisions, etc.). Read it before touching the runner or USD.

## When you finish P5

Commit in repo style (`Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`
trailer), rebuild graphify
(`GRAPHIFY_PYTHON=/home/deivid/dev/DyNAMO-Ridgeback/perception_venv/bin/python3
bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"`, or the
post-commit hook with that env var set), tick the P5 checkbox in
`PORT_PLAN.md`, and update the `isaac-sim-6-port` memory. Then P6 (G1
distance benchmark port) is next.

## Remaining phases after P5 (for context, don't start them yet)

- **P6** — G1 distance benchmark: rewrite gz spawn/remove/pose plumbing onto
  `simulation_interfaces` Control services; recalibrate estimators for D455.
- **P7** — stock envs (warehouse/hospital/office) + `generate_gt_map.py` USD
  ground-truth maps + repeat harness.
- **P8** — remove Gazebo entirely; fresh-clone drill; final docs.

The user works terse and expects you to act autonomously on reversible work,
stopping only for destructive actions or genuine scope decisions. They caught
a real bug by eyeballing rviz last session — show them results they can
eyeball, and verify your own claims by measurement, not by reading the config.
