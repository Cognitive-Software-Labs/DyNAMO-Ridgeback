# Continue the Isaac Sim 6.0 port — rerun baselines on corrected geometry, then P8

You are picking up an in-flight port of a Ridgeback autonomy stack from
Gazebo Harmonic to **Isaac Sim 6.0 GA**. Work happens ONLY in the worktree
`/home/deivid/dev/DyNAMO-Ridgeback/.claude/worktrees/isaac` on branch
`feat/isaac-sim-6-port` — the main checkout stays on its own branch.

## Read these first, in order

1. Your memory file `isaac-sim-6-port` (the 6.0.1 landmine list + the SLAM
   root-cause paragraph — trust it, do not rediscover). Also skim
   `nav-tuning-in-flight` and `shared-dev-box-contention`.
2. `tools/isaac/PORT_PLAN.md` — the authoritative 9-phase plan. **P0–P4 + P7
   done; P5 plumbing done, A/B sign-off open; P6 deferred; P8 next.** The
   post-P7 section and improvement 2 carry the 2026-09-10 sensor work.
3. `tools/isaac/SLAM_QUALITY_REPORT.md` — source of truth for how the RTX
   lidar reaches ROS. The old P4 "270° ROI honored" claim was WRONG; the
   assembler synthesizes the 270° contract scan from two OmniLidar prims.
4. `AI_CONTEXT.md` and `ISSUES.md`. Note: AI_CONTEXT.md still has **zero**
   Isaac content — that rewrite is P8 scope, not drift.

## The worktree is already built — do not redo this

Set up 2026-09-10, and it costs an hour to rediscover:

- **`isaac_venv` is a symlink** to the main checkout's venv, not a copy. Saves
  a 30–50 GB reinstall. It shows as untracked because `.gitignore` has
  `isaac_venv/` with a trailing slash, which does not match a symlink. Leave
  it. Verify with
  `OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 -c "import isaacsim"`.
- **`colcon build` is done** — 24 packages, clean. The only stderr is
  `tl_expected` deprecation noise.
- **Dependencies are pinned by hand** to the main checkout's commits:
  `clearpath_common 9960354`, `clearpath_config 5c92caa`,
  `clearpath_msgs 5d04171`, `clearpath_simulator 69e6833`,
  `slam_toolbox 22450ca`. Both patches in `patches/` are applied.
  **Do not `vcs import` over this** without re-pinning — see the landmine
  below.

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
  diamond notch under the top plate. Now `parent: base_link`,
  `xyz: [±0.3922, 0.0, 0.179]` → laser plane 0.252 m off the floor. Robot USD
  regenerated, not hand-patched. Full derivation in PORT_PLAN improvement 2.
- `686ecc0b` — graphify rebuild.

## Verified live, so you don't have to re-prove it

On `mock_hospital`, `odom_noise:=0`, deterministic, camera off, isolated domain:

- TF after regen: front `[0.392, 0, 0.226]`, rear `[-0.392, 0, 0.226]` yaw 180°
- both scans publishing, `angle_min` -2.356 rad, 40 Hz sim time
- merger over 374 scans: `paired=374 front_only=0 rear_dropped=0 tf_misses=0`
- merged scan `range_max` 10.3922, slam_toolbox subscribed, map at 0.05 m
- 326 finite returns in the rear-only sector (|θ|>135°) the front cannot see

## Your objective

1. **Rerun the Isaac baselines.** Two sensor-geometry changes are stacked
   (coplanar + the 11.6 cm drop) and *every* Isaac coverage number in the plan
   predates both. Nothing has been rerun. Treat all existing Isaac numbers as
   void, not as a baseline to compare against.
2. **Then the P5 A/B**, which needs 3–5 seeds per condition on a genuinely
   single-tenant box — which this one never is (pratham/vilmos/stefi/digit
   rotate). Coverage variance run-to-run is 51–83%, so single runs prove
   nothing. Do not quote a number from one run.
3. **Then P8** (Gazebo removal + docs + graphify).

Still unconfirmed and worth one question to the owner: whether the units are
UST-**10**LX or UST-**20**LX. Both carry the same "Smart-URG" casing branding
and are physically identical; only the side label or the device distinguishes
them. If it is a 20LX then `farRangeM: 10.0` in `ust10lx_2d.json` and
`max_laser_range: 10` in the slam params are both wrong. Cheapest check when
the robot is powered: `ros2 topic echo /scan --field range_max` off `urg_node`.

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

- **`.repos` pins branches (`version: jazzy`), not commits.** A fresh
  `vcs import` pulls whatever upstream HEAD is that day. On 2026-09-10 all
  five deps came down different from the main checkout,
  `clearpath_simulator` by five months, and
  `clearpath_gz_customizations.patch` stopped applying entirely. A background
  task is filed to pin them. Until then, re-pin by hand after any import.
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
- **The Bash tool's working directory persists between calls.** A `cd` in one
  command silently changes the cwd for every later one. Use absolute paths.
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
