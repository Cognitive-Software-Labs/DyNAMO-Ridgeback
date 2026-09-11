# Continue the Isaac Sim 6.0 port — navigation works, produce the first baseline

You are picking up an in-flight port of a Ridgeback autonomy stack from
Gazebo Harmonic to **Isaac Sim 6.0 GA**. Work happens ONLY in the worktree
`/home/deivid/dev/DyNAMO-Ridgeback/.claude/worktrees/isaac` on branch
`feat/isaac-sim-6-port` — the main checkout stays on its own branch. If that
worktree is gone, recreate it per "Recreating the worktree" below; the branch
is on `origin`. **Check `git rev-parse --abbrev-ref HEAD` before committing** —
the worktree was once checked out on `feat/isaac-vendor-chassis` and two
commits landed on the wrong branch.

## Read these first, in order

1. **`docs/isaac/open-issues.md`** — the live register of what is broken.
   Start here. It records what has been ruled out *by measurement* for each
   open bug, which is most of the value: several plausible theories are
   already dead and re-testing them wastes a day.
2. Your memory file `isaac-sim-6-port` (the 6.0.1 landmine list — trust it,
   do not rediscover). Also skim `nav-tuning-in-flight` and
   `shared-dev-box-contention`.
3. `docs/isaac/port-plan.md` — the 9-phase plan and its history. Its
   performance numbers are **stale by design**: geometry changed four times on
   2026-09-10, the lidars were reparented on 2026-09-11, and nothing has been
   rerun since.
4. `docs/isaac/lidar-pipeline.md` — how the RTX lidar reaches ROS. The
   assembler synthesizes the 270° contract scan from two OmniLidar prims.
5. `docs/isaac/assets/robot-geometry.svg` + `robot_render.png` — the sensor mounting,
   dimensioned. Worth 5 minutes before touching anything geometric.
6. `AI_CONTEXT.md`. Note: it still has **zero** Isaac content — that rewrite is
   P8 scope, not drift.

## Where things actually stand

**Navigation runs, as of 2026-09-11.** The long-standing stall is fixed. The
2D lidars had been parented to `base_link`, which is a bare Xform with **no
joint into the articulation**, so they were orphan rigid bodies: PhysX turned
`chassis_link` and left the sensors behind. The chassis then swept underneath
a stationary emitter and its own diamond-notch edge came into range, which is
what produced the "phantom" returns that tripped `collision_monitor`'s
`min_points: 6` and pinned `cmd_vel` at zero. One-line fix in
`clearpath/robot.yaml` (`parent: chassis_link`) plus a USD regen.
`open-issues.md` §1 has the measurements and the full reasoning trail.

After the fix, in `warehouse_full`: `cmd_vel` 799 msgs / 40 s (was 0), odom
displacement 1.15 m (was 0.000), zero `away from collision` log lines (was
continuous), and frontier goals reaching `Goal succeeded`.

**The robot still floats 49.8 mm** on every stock world, which puts the scan
plane 5 cm high (`open-issues.md` §2). Independent of the stall, and still
open.

**No baseline has ever been rerun.** That is the original request, it is still
outstanding, and it is now actually achievable. Every coverage and SLAM number
on record predates the fix and was measured with a sensor that did not rotate
with the robot — they are void, not comparison points.

## Recreating the worktree

It was set up on 2026-09-10 and may have been deleted since. Nothing is lost if
it was — everything tracked is pushed to `origin/feat/isaac-sim-6-port` — but
these facts cost an hour to rediscover, and the five dependency checkouts are
**gitignored**, so they come back only from the hashes below:

- **`isaac_venv` is a symlink** to the main checkout's venv, not a copy. Saves
  a 30–50 GB reinstall. It shows as untracked because `.gitignore` has
  `isaac_venv/` with a trailing slash, which does not match a symlink. Leave
  it. Verify with
  `OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 -c "import isaacsim"`.
- **`colcon build`** covers 24 packages and is clean. The only stderr is
  `tl_expected` deprecation noise.
- **Dependencies are pinned by hand** to the main checkout's commits:
  `clearpath_common 9960354`, `clearpath_config 5c92caa`,
  `clearpath_msgs 5d04171`, `clearpath_simulator 69e6833`,
  `slam_toolbox 22450ca`. Verified against the on-disk checkouts on
  2026-09-11 — all five matched. `src/clearpath_*` and `src/slam_toolbox` are
  **gitignored with zero tracked files**, so `.repos` (exact commits, pinned
  2026-09-11) plus `patches/` are the only record of them. Restore or verify
  with `bash tools/setup_deps.sh [--check]` — it imports the pins, applies the
  patches idempotently, and fails on drift. Both patches were verified
  byte-identical to the live working diffs, with no untracked files in any
  dependency.

## What landed 2026-09-10

Four commits, all verified live unless noted:

- **`2ed1674a` — scan merger was dropping every rear scan.** It was launched
  without the `/tf` remap; `tf2_ros.TransformListener` subscribes to the
  *absolute* `/tf`, so the node's namespace did not move it and its buffer
  stayed permanently empty. Every rear scan whose stamp differed from the
  front's was silently discarded — `slam_source:=merged` was feeding
  slam_toolbox a front-only scan wearing a merged scan's name. Also fixed
  `range_max` (copied from the sensor frame into a `base_link` message, so
  slam_toolbox dropped the outermost returns) and split `rear_dropped` out of
  `paired` so the health log stops lying. Regression test:
  `test_scan_merger_node_remaps_tf_into_the_namespace`.
- **`1969a707` — both lidars on one plane.** The rear unit's 5 cm z offset was
  a Gazebo workaround (gz's idealized hokuyo is 360°, so at equal height the
  two saw each other). Isaac honors a real 270° emitter window, so it was
  unnecessary. This deliberately breaks the gz lidars; accepted, gz dies in P8.
- **`a33111c2` — lidars were mounted 11.6 cm too high.** They were parented to
  `default_mount` (the top deck) when physically they sit recessed in a
  diamond notch under the top plate. `xyz: [±0.3922, 0.0, 0.179]` → laser
  plane 0.252 m off the floor. Robot USD regenerated, not hand-patched. Full
  derivation in PORT_PLAN improvement 2. **The height was right, but the
  `parent: base_link` it also introduced detached the sensors from the
  articulation — see 2026-09-11 below.**
- `686ecc0b` — graphify rebuild.

## What landed 2026-09-11

- **The lidars were orphan rigid bodies.** `parent: base_link` →
  `parent: chassis_link` in `clearpath/robot.yaml` + USD regen. `chassis_link`
  is coincident with `base_link` (identity local transform), so `xyz` and
  published TF are unchanged. This is what unblocked navigation.
- **A 10° scan edge mask was added, then removed.** It masked the symptom;
  once the root cause landed it measured unnecessary and cost 7.4% of each
  arc. `test_lidar_scan_assembler.py` fails if one is reintroduced.
- **New diagnostics.** `diag_rig.py --spin-transforms` is the permanent guard
  for this class of bug — it spins in place and asserts the sensor's world yaw
  tracks the chassis (expect `lidar-chassis` ≈ 0 and a constant `notch_r`).
  Also `stall_probe.py` (footprint + scans + raw clouds + the `cmd_vel` chain
  in one attach), `self_occlusion_check.py` (self-occlusion sliced from the
  robot USD) and `world_probe.py` (world geometry at the scan plane, with prim
  attribution).
- **`cleanup.sh` matches this workspace's nodes by install path now.** The
  hand-maintained name list had missed `scan_merger_node` since `2ed1674a`, so
  an orphaned merger survived every cleanup for 16 h and double-published into
  a benchmark run.

## Verified live, so you don't have to re-prove it

On `mock_hospital`, `odom_noise:=0`, deterministic, camera off, isolated domain:

- TF after regen: front `[0.392, 0, 0.226]`, rear `[-0.392, 0, 0.226]` yaw 180°
- both scans publishing, `angle_min` -2.356 rad, 40 Hz sim time
- merger over 374 scans: `paired=374 front_only=0 rear_dropped=0 tf_misses=0`
- merged scan `range_max` 10.3922, slam_toolbox subscribed, map at 0.05 m
- 326 finite returns in the rear-only sector (|θ|>135°) the front cannot see

⚠️ **All measured before the 2026-09-11 reparent**, i.e. with a sensor that did
not rotate with the robot. The static facts (TF offsets, rates, `range_max`,
merger pairing) are unaffected and still hold — the reparent changes no
offsets. Anything that depends on the robot *moving* — map quality, coverage,
the rear-sector count while driving — must be re-measured.

## Your objective

**Out of scope: the G1 distance/detection benchmark (P6).** Do not port it,
do not run it, do not touch `g1_distance_benchmark.launch.py`, the G1
perception stack or `G1_DISTANCE_BENCHMARKING.md`. Every exploration run uses
`g1_perception_enabled:=false` anyway. P6 stays deferred.

1. **Seat the robot** — `open-issues.md` §2. It floats 49.8 mm on every stock
   world because `--spawn-z 0.076` is tuned for `mock_hospital`'s 0.05 floor.
   The scan plane rides up with it, so the lidars sample 5 cm high everywhere
   except `mock_hospital`. Fix shape: `spawn_z = floor_z + 0.02617`, and
   `LIDAR_PLANE_Z` becomes `floor_z + 0.25257` rather than one constant.
   (Those are the **measured** wheel-mesh values; the 0.0259/0.2523 quoted
   before came from axle − radius using a 0.0759 radius, and the mesh bottoms
   at −0.02617 with radius 0.07617. The 0.3 mm is below map resolution, but
   use the measured pair so the docs stop disagreeing.)

2. **Regenerate the ground-truth maps once**, after the seat fix — it moves
   the slice plane for stock worlds, so doing it first wastes the work.
   `open-issues.md` §4.

3. **Validate the hull collider** — `open-issues.md` §5. Drive into a wall,
   confirm the robot stops where the geometry says it should.

4. **Then the exploration baselines** — `open-issues.md` §3, the original
   request and still unmet, now unblocked. Canonical recipe and the hygiene
   table are in `../../tools/benchmark/README.md`; every flag there earns its
   place. Re-measure SLAM quality too (`slam_quality_probe.py`) — the old
   RMSE 0.23 describes a different rig.

5. **Then the P5 A/B** — 3–5 seeds per condition. Coverage variance ran
   51–83%, so a single run proves nothing and this box is never genuinely
   single-tenant.

6. **Then Isaac 6.1** (`port-plan.md` §P9), and only then P8. The 6.1 move is
   sequenced after a baseline deliberately: it is the only way to tell whether
   the upgrade helped, and it may let several 6.0.1 workarounds be deleted
   outright. Do not migrate first.

## How to run the stack

```bash
cd <worktree root>
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
export OMNI_KIT_ACCEPT_EULA=YES
export ROS_DOMAIN_ID=77          # isolate; 77 was free on 2026-09-10

# sim only (runner + EKF + robot_state_publisher):
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/simulation_isaac.launch.py \
  world:=mock_hospital camera:=false odom_noise:=0.0 sim_mode:=deterministic setup_path:=$PWD/clearpath/

# slam on top, with the front+rear merge feeding it:
ros2 launch install/ridgeback_autonomy/share/ridgeback_autonomy/launch/includes/slam.launch.py \
  use_sim_time:=true setup_path:=$PWD/clearpath/ slam_source:=merged

# full exploration:
bash start_exploration.sh warehouse_full sim:=isaac sim_mode:=deterministic \
  camera:=false g1_perception_enabled:=false
```

Runner reaches `RUNNER READY` in ~20 s with a warm shader cache; cold boot up
to ~4 min. Robot USD regen (only when `clearpath/robot.yaml` changes):
`OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 tools/isaac/import_ridgeback_urdf.py`
— takes ~165 s and rewrites all eight USD layers.

**Benchmark hygiene is mandatory** (it was wrong for months): `camera:=false`
(the runner used to render the D455 unconditionally, RTF 0.33→0.65),
`g1_perception_enabled:=false`, an isolated `ROS_DOMAIN_ID` (a co-tenant ran a
`/r100_0001` stack on domain 42 whose `hud_node` publishes the same
`hud/coverage` the probe reads), and `setup_path:=/tmp/bench-clearpath/`. Use
`sim_mode:=deterministic` for anything A/B — realtime couples frame_dt to
render-wall duration, so contention smears the lidar sweep.

Measure all rates in **sim time**, not wall-clock (RTF ≈0.67 makes 40 Hz look
like 27 Hz on the wall — that is expected, not a bug).

## Landmines that will cost you an hour each

- **~~`.repos` pins branches, not commits~~ — FIXED 2026-09-11.** It pinned
  `version: jazzy` for all five deps, so `vcs import` pulled whatever upstream
  HEAD was that day; on 2026-09-10 all five came down different (
  `clearpath_simulator` by five months) and
  `clearpath_gz_customizations.patch` stopped applying entirely. `.repos` now
  pins exact commits. Restore or verify with `bash tools/setup_deps.sh
  [--check]`, which imports the pins, applies `patches/` idempotently, and
  fails on any drift beyond them. See `patches/README.md`.
- **`.repos` paths already start with `src/`.** Run `vcs import < .repos` from
  the repo root, NOT `vcs import src < .repos` — the latter creates
  `src/src/...`.
- **`pgrep -f` / `pkill -f` in a wait loop matches its own bash wrapper**, and
  `kill $!` after `nohup ... &` in a compound command kills the *wrapper*, not
  the python child. Hit for real on 2026-09-10: a merger survived its kill and
  two instances published to the same topic, silently corrupting a probe run.
  Always `ps aux | grep X | grep -v grep`, kill by explicit PID, verify the
  count is 0 afterwards, and keep kills in a separate tool call from launches.
- **Isaac stamps the front and rear lidar identically** (same tick), so the
  merger always takes its `abs(delta) < 1e-6` static-extrinsic fast path and
  the motion-compensation/TF branch is effectively dead code in sim. Do not
  conclude from a green sim run that that branch works — it is only exercised
  on hardware, where the two units have independent clocks.
- **A stale `ros2 daemon` hides namespaced topics from the CLI** (`ros2 daemon
  stop`, or `--no-daemon`). rclpy probes are unaffected but still need the
  CycloneDDS env above.
- **The Bash tool's working directory persists between calls — and can reset
  without warning.** A `cd` in one command silently changes the cwd for every
  later one, *and* the shell is periodically re-initialized from the user's
  profile, which snaps the cwd back to the primary checkout
  (`/home/deivid/dev/DyNAMO-Ridgeback`) mid-session. In a worktree session that
  means a relative path can silently act on the **wrong checkout**. Hit on
  2026-09-11: a `bash -n cleanup.sh` and a `pgrep` dry-run both ran against the
  main checkout and reported a false negative. Use absolute paths for anything
  that reads or writes, and `git -C <worktree>` for git.
- **mock_hospital logs `CreateJoint - cannot create a joint between static
  bodies`** for the G1 model prims at startup. Pre-existing and harmless; do
  not chase it.
- **The user's VNC display number rotates per session.** Before any windowed
  launch or rviz, read `DISPLAY`/`XAUTHORITY` from a live session process's
  `/proc/<pid>/environ`. Never assume `:0`.
- **Shared box.** Check `getfacl /dev/dri/renderD128` before any long GPU
  session, then co-tenant load. A single spontaneous kit boot crash is flaky —
  relaunch, don't over-diagnose.
- **The full 6.0.1 landmine list** is in your memory file (bridge-after-open-
  stage segfault, ArticulationRoot rules, importer drops meshes/collisions,
  remote-anchor `AddReference` needing a `file://` URI, etc.).

## Working agreements that bit during this session

- Verify geometry against the **mesh**, not against box primitives in the
  xacro. A claim that the lidars overhung the riser plate by 0.146 m was wrong
  because it measured `riser_link`'s 0.493 m collision box instead of the
  0.932 × 0.790 m `top.stl` deck. Slice or ray-cast the actual STL.
- Commit in repo style — **no AI-attribution trailers** (no `Co-Authored-By`,
  no "Generated with"; the user is the author). Note `ca903276` has one; it
  predates the rule and should not be copied.
- Rebuild graphify after code changes:
  `GRAPHIFY_PYTHON=/home/deivid/dev/DyNAMO-Ridgeback/perception_venv/bin/python3
  bash "$(git rev-parse --show-toplevel)/tools/rebuild_graphify"`. The
  post-commit hook fails without that env var set.

The user works terse and expects you to act autonomously on reversible work,
stopping only for destructive actions or genuine scope decisions. They will
push back on wrong numbers — check your claims by measurement, and when they
contradict you, re-measure rather than concede or dig in.
