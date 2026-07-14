# Port plan: Gazebo Harmonic → NVIDIA Isaac Sim 6.0

Status: **in progress — P0–P4 done, P5 next.** Kept current per phase; mark phases done as they land.
Branch: `feat/isaac-sim-6-port` (cut from `dev`). Base includes `4f8b72d8` (explore_lite removed — single in-repo `frontier_explorer_node`).

## Context

The simulation layer runs on Gazebo Harmonic via the Clearpath stack (`clearpath_simulator` in `.repos` + `patches/clearpath_gz_customizations.patch`). This port moves it to **Isaac Sim 6.0 (GA)**, **fully replacing Gazebo** — both public entrypoints (`ridgeback_exploration.launch.py`, `g1_distance_benchmark.launch.py`) end up on Isaac. **Not a bug-for-bug copy**: where Isaac-native approaches beat the gz-era design, we adopt them (approved list below).

Decisions:
- Replace Gazebo outright; gz stays runnable only during the port for sign-off, removed in the final phase.
- Full scope: exploration + G1 distance benchmark.
- Kinematic holonomic drive (planar virtual joints); true mecanum = documented upgrade path.
- G1 model from `unitree_sim_isaaclab` (Unitree official USD); STL→USD converter fallback.
- Follow-up addons (designed-for, not in this port): procedural warehouse generation (headless Warehouse Creator), animated humans (behavior-tree pedestrians), Replicator SDG datasets.

## Verified facts (researched 2026-07-10 against live docs; Isaac 6 is post-LLM-cutoff — do not trust model memory, verify via Isaac Sim MCP)

**Box**: RTX PRO 6000 Blackwell 96GB (Isaac "ideal" spec), driver 580.159.03 ≥ 580.95.05, Ubuntu 24.04.4, Python 3.12.3, 91GB RAM, 32 cores, 1.1TB free. Shared box (GPU seat ACL history) → headless-first.

**Isaac Sim 6.0**:
- GA (6.0.0/6.0.1). Pip `isaacsim[all,extscache]==6.0.1 --extra-index-url https://pypi.nvidia.com`, **Python 3.12** = Jazzy's — one venv sources ROS + imports isaacsim.
- ROS 2 bridge: OmniGraph nodes; Jazzy supported; **bundles rmw_cyclonedds_cpp** (source system ROS first → bridge uses system DDS; repo default CycloneDDS works). Clock/TF/Odometry/IMU/RTX-lidar→LaserScan/camera publishers; Twist(Stamped) subscriber.
- **Simulation Control ROS 2 services** new in 6.0 (`simulation_interfaces`: SpawnEntities, GetSpawnables, GetEntityBounds, entity state/reset family) → replaces gz spawn/remove/pose CLI plumbing; also enables in-session benchmark reset.
- **No Ridgeback USD in the 6.0 catalog** (only Jackal/Dingo; old Ridgeback+arm assets gone and were visual-only) → **URDF import is the primary robot source**.
- **No Unitree G1 in the 6.0 catalog** (only Z1/Dex) → vendor from `github.com/unitreerobotics/unitree_sim_isaaclab` (git-lfs; 4.5/5.x-era but static-visual USD fine in 6.0; license check + attribution).
- RealSense **D455** USD asset: `/Isaac/Sensors/RealSense/D455/rsd455.usd` (RGB + depth + IMU). No D435 asset.
- Stock USD environments: Warehouse, Hospital, Office (NVIDIA assets browser).
- Occupancy-map generator extension (`isaacsim.asset.gen.omap`-family) → analytic GT occupancy grids from the USD stage.
- Headless `SimulationApp(headless=True)` (Vulkan, no X) + WebRTC livestream client. Breaking vs 5.x: `omni.isaac.*` → `isaacsim.*`.
- **Isaac Sim MCP server** (official, `NVIDIA-Omniverse/kit-usd-agents` → `source/mcp/isaacsim_mcp`): standalone RAG over Isaac extensions/APIs/examples/settings; Docker, streamable HTTP `:9904/mcp`; needs NVIDIA API key (build.nvidia.com) + git-lfs; no running Isaac required. Claude Code config: `{"type":"http","url":"http://localhost:9904/mcp"}`. Separate **Isaac Sim Skills** allow live-instance control from Claude Code.

**Current stack facts**:
- Topic contract (names/types/QoS/TF frames FROZEN; sensor specs intentionally diverge — see improvements): `/r100_0001/sensors/lidar2d_{0,1}/scan` (frames `lidar2d_{0,1}_laser`), `.../camera_0/{color/image,depth/image,points}` + camera_info (frame `camera_0_color_optical_frame`), `platform/odom/filtered`, `cmd_vel` (**TwistStamped** — `enable_stamped_cmd_vel: true`), `/clock`, namespaced tf/tf_static. Publish RELIABLE/VOLATILE/keep-10 (ros_gz-compatible with best-effort subscribers).
- gz camera actually renders 640×480 hfov 1.25 despite robot.yaml's 1280×720@30 — latent mismatch, fixed by the D455 migration.
- gz "hokuyo" lidar is idealized 360°/25m/zero-noise — not a real UST; replaced by real spec.
- Bringup event-driven (`launch_wait` gates scan+odom → map → costmap → explorer; no timers). **Single explorer**: in-repo `frontier_explorer_node` (`explore_lite` removed in `4f8b72d8`; no `explorer` launch arg; `start_exploration.sh` takes only `world` positionally; `explore/status` is `std_msgs/String` with `exploration_started`/`exploration_complete`).
- `mock_hospital.sdf` = 98 boxes + 2 spheres + 1 include. GT maps exist for mock_hospital/office/warehouse (gz-captured). `tools/benchmark/explore_probe.py` is pure ROS, reused unchanged.
- Perception `base_frame` default `r100_0001/robot/base_link` (identity shim needed).
- `.repos` after port: **remove** `clearpath_simulator`; **stay**: `clearpath_common` (description xacros + `generate_description`; `clearpath_control` EKF configs reused), `clearpath_config`, `clearpath_msgs` (build dep), `slam_toolbox`. (`m-explore-ros2` already removed on dev.)
- Remaining patches: `clearpath_gz_customizations.patch` (dies in P8), `slam_toolbox_tf_namespace.patch` (stays).

## Deliberate improvements over the gz version (approved)

1. **D455-native camera**: mount `rsd455.usd`, publish 1280×720@30 with true D455 intrinsics. Update `config/camera_config.json` (single source of intrinsics); perception estimators recalibrated. Fixes gz's silent 640×480 render. D455 IMU available.
2. **Real UST-10LX lidars**: 270° FOV, 0.05–10 m, realistic range noise, 40 Hz, custom RTX profile JSON. Front+rear pair still covers 360° combined. Config deltas (the only ROS-side YAML touches, physically motivated): `slam_toolbox_params.yaml` `max_laser_range` 20→10; review costmap `obstacle_max_range`/`raytrace_max_range` vs 10 m sensor. Historical continuity intentionally broken → fresh Isaac baselines.
3. **Honest odometry pipeline**: runner publishes **noise-injected raw wheel odom** (configurable per-meter translational + yaw drift; σ=0 debug knob) + **IMU** → ROS-side `robot_localization` EKF (config derived from `clearpath_control`) → `platform/odom/filtered`. SLAM earns its keep; closest to the real robot.
4. **USD-derived GT maps**: analytic occupancy grids from the stage (occupancy-map generator) → `sim/ground_truth_maps/` for every Isaac world. Replaces manual capture drives; `capture_ground_truth.sh` retired to historical.
5. **GT pose + localization-error metric**: runner publishes `ground_truth/pose`; probe + HUD gain live SLAM drift (map→odom error). New HUD panel via the existing aggregator pattern.
6. **In-session reset**: `simulation_interfaces` reset/entity services → `--repeat N` exploration runs without sim relaunch.
7. **Headless + WebRTC livestream** as the only GUI story; `gz_gui` concept dies; seat-ACL pain class gone.
8. **Unthrottled mode** `--rtf 0` for benchmarks (all-sim-time stack), RTF 1.0 default interactive.
9. **Repo-local setup_path default**: use in-repo `clearpath/` directly; `~/clearpath/` mirror requirement dropped.
10. **Isaac stock environments** (Warehouse/Hospital/Office USD) as NEW worlds with USD-derived GT maps — replaces the dying gz office/warehouse; old GT maps marked historical.

## Architecture

**Runner**: standalone `SimulationApp` process under `src/ridgeback_autonomy/sim/isaac/`, launched via `ExecuteProcess` with `<workspace>/isaac_venv/bin/python3` (perception_venv precedent), ROS Jazzy + CycloneDDS env inherited.

- `isaac_runner.py` — CLI: `--world <name|path.usd> --namespace r100_0001 --headless --livestream --physics-hz 120 --rtf {1.0,0} --odom-noise <profile|0> --robot-usd <override>`. Lifecycle, main loop, clean SIGINT.
- `worlds.py` — name → `share/.../sim/isaac/usd/worlds/{name}.usda` resolver + registry for stock-env worlds. `start_exploration.sh mock_hospital` works verbatim.
- `robot_rig.py` — robot USD load, planar drive rig, per-step velocity targets, raw-odom integration + noise model.
- `sensors.py` + `ust10lx_2d.json` — 2× RTX lidar (UST-10LX datasheet spec) on `lidar2d_{0,1}_laser` prims; D455 rig on camera mount (USD −Z-forward → ROS optical rotation), color+depth+points+camera_info, IMU.
- `ros_io.py` — in-process rclpy (`isaac_sim_runner`, ns `r100_0001`, tf remaps): TwistStamped sub on `cmd_vel` (+ tolerant Twist sub), raw odom pub, IMU pub, `ground_truth/pose` pub, `/clock` pub, `sim/spawn_g1` Trigger service, reset glue. TF `odom→base_link` ownership: EKF (runner TF off) — matches real robot.
- OmniGraph only for GPU sensor paths: `isaacsim.ros2.bridge.ROS2RtxLidarHelper` (laser_scan, namespaced), `ROS2CameraHelper` (rgb/depth/depth_pcl) + `ROS2CameraInfoHelper`, `ROS2PublishImu`. Exact 6.0 ids resolved via Isaac Sim MCP / `og.get_registered_nodes()`. Fallback if rclpy-in-process misbehaves: all-OmniGraph I/O.

**Drive rig** (added at import): `world → prismatic-X → prismatic-Y → revolute-Z → base_link`, velocity drives (stiffness 0, high damping). Per step: read θ, rotate body twist to world, set 3 velocity targets; accel-limited (1.0 m/s² platform match); 0.5 s cmd timeout. Wheels free/visual; URDF colliders keep PhysX contacts real (no tunneling).

**Timing**: physics 120 Hz (divides 40 Hz lidar, 30 Hz camera under 6.0 multitick), render decoupled, RTF throttle per flag.

## Phases

Bootstrap: cut `feat/isaac-sim-6-port` from `dev`; workspace already built in the main checkout (`vcs import` only if clones missing; rosdep adds `robot_localization`). `bash tools/rebuild_graphify` after every code phase.

### P0 — Contract freeze + gz reference capture [S] ✅
- This file committed; `tools/isaac/capture_contract.sh` → committed `tools/isaac/baseline/contract/` (topic list/types/QoS, camera_info, view_frames, hz for all contract topics).
- 3× gz probe runs (`explore_probe.py gz_base{1..3}`) on `bash start_exploration.sh mock_hospital` → `tools/isaac/baseline/gz_mock_hospital/` (reference only — sensors diverge by design).
- ✓ Baseline dirs complete; probe JSONs show expected coverage (mock_hospital ≈93% reference from `4f8b72d8` validation).

### P1 — isaac_venv + installer + headless smoke + MCP dev tooling [M] ✅
- `requirements-isaac.txt` (pinned 6.0.1), `tools/install_isaac_venv.sh` (system-site-packages venv + ros2.pth like perception_venv; EULA env; `--warmup` shader-cache bake; nvidia-smi driver/VRAM preflight), `tools/isaac/smoke_test.py` (headless + bridge + clock publish, 5 s). `.gitignore` += `isaac_venv/`. README install subsection.
- **Isaac Sim MCP setup**: clone `kit-usd-agents`, `./build-docker.sh` in `source/mcp/isaacsim_mcp`, run on `:9904`, add http entry to project `.mcp.json`. Needs NVIDIA API key (build.nvidia.com) — if unavailable, skip; fall back to 5.1-era names + `og.get_registered_nodes()`. Used through P2–P7 to resolve exact 6.0 APIs before writing code. Evaluate Isaac Sim Skills during P3–P4 as a live-verification aid.
- ✓ Smoke test exits 0 while a plain-terminal `ros2 topic hz /clock` (CycloneDDS) ticks. MCP: one successful extension-search query (if key available).

### P2 — Assets: worlds + G1 [L] ✅
- `tools/isaac/sdf2usd.py`: xml parse layer (unit-testable, no pxr) + pxr emit layer (Cube/Sphere, colliders exactly where SDF `<collision>`, UsdPreviewSurface, lights, PhysicsScene, `model://g1` → USD reference). `.usda` text output. `--check` re-opens USD, asserts per-model AABB/pose vs SDF.
- Convert `mock_hospital`, `g1_distance_calibration` → `src/ridgeback_autonomy/sim/isaac/usd/worlds/`.
- G1: vendor USD from `unitree_sim_isaaclab` (git-lfs; license check + attribution) → `sim/isaac/usd/models/g1/`; fallback `tools/isaac/convert_g1_model.py` (STL→USD).
- Pytest `test_sdf2usd_parse.py` (98 boxes/2 spheres/1 include, exact named-wall poses) in CMake BUILD_TESTING.
- ✓ `--check` passes both worlds; `colcon test` green; headless screenshot eyeball.

### P3 — Robot: URDF import + rig + raw odom/IMU/TF/clock [L] ✅
- `tools/isaac/import_ridgeback_urdf.py`: `clearpath_generator_common generate_description` (repo-local setup_path) → xacro → URDF → URDF importer (`merge_fixed_joints=False`, no drives) → append planar rig → committed `sim/isaac/usd/robots/ridgeback_r100.usda`. Regen only when robot.yaml changes.
- `isaac_runner.py`, `worlds.py`, `robot_rig.py` (+ odom noise model), `ros_io.py`.
- ✓ Manual: runner headless in mock_hospital; TwistStamped teleop → raw odom ≈50 Hz arcs, /clock ticks, stop ≤0.5 s, strafe proves holonomic; `--odom-noise 0` vs default shows drift in odom while `ground_truth/pose` stays exact.
- **DONE** — motion verified two ways: `tools/isaac/diag_rig.py --battery` 10/10 (stillness at spawn, zero wheel contacts, sim-time advance, set_planar_pose teleport, fwd/strafe/spin/combined tracking exact, stop drift 0.00 cm, cmd-timeout stop) + live-runner ROS checks (/clock advances at RTF≈1, odom arcs under teleop, strafe-at-yaw holonomic, stop twist ~0, noisy odom vs exact GT split). Deviation: raw odom publishes at the render rate (~35 Hz under co-tenant load), not 50 Hz — publish-per-physics-step decoupling is a P4 option if the EKF wants it.

### P4 — Sensors + EKF + launch include: full contract [L] ✅
- `sensors.py`, `ust10lx_2d.json`, D455 rig + `camera_config.json` update (real D455 intrinsics), IMU publisher.
- `launch/includes/simulation_isaac.launch.py`: event chain generate_description → OnProcessExit → robot_state_publisher + `robot_localization` ekf_node (→ `platform/odom/filtered` + TF) + runner ExecuteProcess. Same arg surface as the gz include.
- `includes/simulation.launch.py`: transient `sim:=gz|isaac` dispatch (default gz until P8).
- `cleanup.sh` += isaac kill patterns (user-scoped `isaac_runner.py`, `omni.kit`); extend `test_launch_layout.py`.
- Config deltas land here: slam `max_laser_range` 10, costmap range review (documented in commit).
- ✓ Isaac include alone: all contract topics live, types/QoS match baseline, camera_info = D455 1280×720 (intentional divergence, matches new camera_config.json), EKF publishes filtered odom + TF, view_frames complete, standalone slam_toolbox maps.
- **DONE** — include-alone acceptance: 14/14 contract topics live, QoS RELIABLE/VOLATILE matches baseline, camera_info 1280×720 fx=631 frame=color-optical, EKF filtered odom within ~1 mm of GT while raw drifts (IMU+odom fusion), TF tree 22 edges incl. odom→base_link from EKF, slam_toolbox maps standalone (1758 occ / 17k free cells after one arc). Deviations, all documented in code: 6.0 replaced lidar JSON profiles with OmniLidar prims (`ust10lx_2d.json` is now our spec file that sensors.py authors onto the prim; needs `omni:sensor:tickRate` 40 + `accumulateOutputs`); **[CORRECTED 2026-07-12]** the "LaserScan is a 360° frame with the rear 90° inf, ROI honored, effective 270°" claim here was WRONG — both the mechanism it describes and the state it asserted. The 6.0.1 bridge laser_scan writer hardcodes 360° for ROTARY lidars (ignores the ROI) and only fires 180°/tick, so the P4 scan was actually half-blind (135°, right side only) and rotation-warped — never a clean 270°. The fix (commits `66493670` + `e1862498`) makes the 270° outcome real but NOT via the mechanism above: two OmniLidar prims per laser frame publish point clouds, and `ros_io.LidarScanAssembler` emits a **native 270° message** (`angle_min -135°`, 1081 bins, `angle_max +135°`) — there is no 360° frame and no rear-inf sector anymore. So the FOV number now matches by outcome, but the "360° frame with rear inf, ROI honored" description was never how this worked. Full writeup: `tools/isaac/SLAM_QUALITY_REPORT.md`. Scan now 40 Hz sim-time; odom ~30 Hz — render-frame-locked under co-tenant load (gz baseline 37/46; slam fine, revisit at P5 RTF gate); camera = plain USD camera with true D455 720p intrinsics at the d435 mount pose instead of referencing the cloud `rsd455.usd` asset (self-contained repo beats a boot-time network fetch; mesh is cosmetic); UST-10LX min range 0.06 m per datasheet (plan said 0.05).

### P5 — E2E exploration + sign-off [M] (transient A/B window opens) — plumbing ✅, A/B ⛔ blocked
- ✅ `ridgeback_exploration.launch.py`: forwards `sim` + `rtf`/`headless`/`livestream` down the include chain (simulation.launch.py already dispatched on `sim`); `sim_ready_timeout` (45 gz / 300 isaac, PythonExpression default) on the first gate. HUD localization-error panel (GT pose vs SLAM `map->base_link`) via new `localization_overlay_node` (Isaac-only — gz has no `ground_truth/pose`), wired into the HUD aggregator. `explore_probe` now logs GT-drift + achieved RTF (from `/clock`) + coverage accuracy (additive; gz schema unchanged). (9e0f3c02)
- ✅ `tools/isaac/ab_compare.py`: pure-stdlib gz-vs-isaac table + §P5 gate checker (coverage complete/accuracy, time-to-complete, achieved RTF, aborts, success, isaac localization error). Verified vs the gz baseline + synthetic pass/fail sets. (9e0f3c02)
- ⛔ Acceptance NOT met — deferred. Live `sim:=isaac` validated (gates pass, all HUD panels publish, probe captures every field), but exploration **stalls ~40% coverage**: the SLAM map yaws ~14° in feature-poor rooms → phantom costmap wall the explorer can't pass (surfaced by the new localization panel: yaw err −14°, trans 1.14 m). The A/B green run (3/3 complete; coverage ≥ gz mean − 10; genuine aborts ≤ gz max; RTF ≥0.8 throttled headless; one `--rtf 0` faster wall-clock) waits on the drift fix below.
- 🔬 SLAM-drift investigation (2026-07-13, 858ee678) — diagnosed, not closed. Extended `slam_quality_probe` to split the yaw error into `phi_ekf` (odom/EKF heading = `yaw(odom→base) − yaw(GT)`) vs `theta_mo` (SLAM `map→odom`), with a `--repro` high-wz drive into the feature-poor NE room. Measured facts:
  - **Cause is SLAM scan-match rotation** (`theta_mo` 5–9°), NOT the EKF (`phi_ekf` stable ~2.5°) and NOT rotation-rate smear (wz 0.8 measured *worse* than 1.8 → nav2 wz-cap **ruled out**). `theta_mo` is noise-bound run-to-run, so single-run SLAM param A/B is unreliable — angle-penalty + rotation-search-bound tweaks were inconclusive and **reverted**.
  - **`odom_noise=1.0` is a major upstream inflator**: perfect odom (`odom_noise=0`) cut drift ~14°→~6°, but did NOT clear the plateau (~45%, abort-churn). 1.0 is likely pessimistic for wheel+IMU.
  - **Sensor is NOT the cap**: `tools/isaac/coverage_ceiling.py` → 90.8% observable @10 m vs 91.4% @25 m (gz). So the 83% gate is reachable; the plateau is drift-induced map inconsistency, not range.
  - Landed durable: probe yaw-decomposition + `--wz-max`/`--repro`; `headless`/`livestream`/`odom_noise` threaded through the launch chain; `frontier_explorer` costmap-clear-on-stall (kept — the aggressive 30 s / blacklist-2 timeouts abort-churned and were reverted to 60 s/3); `coverage_ceiling.py`. Reaching 83% needs a focused multi-factor effort: realistic `odom_noise`, the residual scan-match degeneracy, and explorer robustness.
- 🔎 **Noise + hygiene A/B (2026-07-13, follow-up) — the ~40% "plateau" does NOT reproduce.** With benchmark hygiene corrected (below), clean `mock_hospital sim:=isaac` runs land **51–83% coverage**, never ~40%.
  - **`odom_noise` is a real but PARTIAL lever, not the cap.** Clean paired A/B (camera-off, domain-isolated): `odom_noise=0` → 61.9% / 16 aborts; `odom_noise=1.0` → 51.3% / 35 aborts — ~10 pts + 2× aborts, not a slam to 40%.
  - **Prime suspect for the historical ~40% = the OLD aggressive explorer timeouts** (30 s / blacklist-2, already reverted to 60 s/3). The prior "~45% noise-off" run's abort-churn matches those timeouts, not SLAM/sensor/odom.
  - **H2 (assembler flap) is dead:** 0 flap events across ~110k scans (all runs), incl. under GPU contention. **H3 (RTF/timing) does not cap coverage.** Residual drift = H1 scan-match rotation (`--repro`, noise off: `theta_mo` 1.47° rms > `phi_ekf` 0.71° rms — scan-match, not EKF; on full scans), transient + loop-closure-recoverable.
  - **Benchmark hygiene (mandatory; was wrong before):** the runner rendered the D455 **unconditionally** (RTF 0.33–0.45). Added `camera:=false` (commit b9c46c77 → RTF 0.55–0.65) + `g1_perception_enabled:=false` + isolate `ROS_DOMAIN_ID` — co-tenant `stefi` ran a `/r100_0001` stack on domain 42 whose `hud_node` publishes the same `hud/coverage` topic the probe reads. Canonical: `camera:=false g1_perception_enabled:=false ROS_DOMAIN_ID=<isolated> setup_path:=/tmp/bench-clearpath/`.
  - **CAVEAT:** coverage variance is large run-to-run (62–83% noise-off; loc-err 0.9–9.4 m) — stochastic scan-match excursions + intermittent co-tenant GPU bursts dominate. Firm numbers need **multi-seed (3–5/condition) on a genuinely single-tenant box** — this box (pratham/vilmos/stefi/digit rotating) never is. New read-only tool: `scan_pipeline_probe` (per-scan finite-bin/flap/pair telemetry vs RTF).

### P6 — G1 distance benchmark port [L]
- Rewrite gz plumbing in place in `g1_distance_benchmark_runner_node.py`: spawn/remove/pose → Simulation Control services (`simulation_interfaces`; exact names via `ros2 service list` with the sim-control extension enabled). Fallback: custom rclpy srvs in `ros_io.py` on the USD stage.
- `ros_io.py` += `sim/spawn_g1` Trigger (G1 2 m ahead — SpawnG1-GUI replacement). Forward `sim` in `g1_distance_benchmark.launch.py`. Update `test_benchmark_runner.py` mocks (subprocess → service clients). Recalibrate estimators for D455 intrinsics via `g1_distance_calibration`.
- ✓ Full run `sim:=isaac estimators:=rgb,sensor_depth,lidar repeats:=1`: 15 poses, comparison CSV + collages, lidar MAE sane (≤2× gz history band), repeat clean (no residual prims), manual spawn service works.

### P7 — Stock environments + USD GT maps + repeat harness [M]
- `worlds.py` registry += `warehouse`, `hospital`, `office` (Isaac stock envs; assets cached; registry/internet need documented).
- `tools/isaac/generate_gt_map.py` (occupancy-map generator) → fresh `sim/ground_truth_maps/` for all Isaac worlds incl. mock_hospital (analytic beats driven capture); old gz-captured maps marked historical in `sim/ground_truth_maps/README.md`.
- In-session reset: runner reset service + probe/harness `--repeat N`.
- ✓ `bash start_exploration.sh warehouse sim:=isaac` explores the stock warehouse with a valid live coverage HUD; `--repeat 3` yields 3 summaries without relaunch.

### P8 — Gazebo removal + docs + graphify [M]
- `.repos`: remove `clearpath_simulator` (stay-list above). Delete `patches/clearpath_gz_customizations.patch`, `sim/gz_plugins/` (SpawnG1.*). CMakeLists drops gz/Qt5 + SpawnG1; package.xml drops `clearpath_gz`/gz vendors/Qt5, adds `robot_localization`.
- `simulation.launch.py` becomes the Isaac include (dispatch/`sim`/`gz_gui` die; `headless_rendering` semantics = Isaac headless + `livestream` arg; `sim_ready_timeout` settles 300).
- Keep `sim/worlds/*.sdf` as converter source-of-truth (documented). Retire `capture_ground_truth.sh` to historical.
- Update `diag.sh` (Isaac section), `cleanup.sh` (drop gz), README (isaac_venv install, livestream, spawn_g1, repeat mode), AI_CONTEXT.md (runner architecture, asset layout, regen workflows), ISSUES.md (retire gz-EGL RTF recipe; add Isaac shader-cache/VRAM/rclpy-DDS sections; keep camera-optical-TF), `.claude/agents/{sim-runner,box-health,log-triage}.md`, `tools/benchmark/README.md`, `start_exploration.sh` comments (positional contract: `world` only, unchanged). This file marked completed. `bash tools/rebuild_graphify`.
- ✓ Fresh-clone drill: `vcs import` → `colcon build` → `install_isaac_venv.sh` → `bash start_exploration.sh` completes Isaac mock_hospital exploration. `grep -ri "clearpath_gz\|ros_gz\|gz sim"` → only intentional historical mentions. `colcon test` green.

## Key risks

| Risk | Mitigation |
|---|---|
| First-start shader compile trips gates | installer `--warmup`; `sim_ready_timeout` 300; publisher-based gates just wait |
| RTF below gz; MPPI timing sensitivity | 120 Hz physics, decoupled render, headless; gate RTF ≥0.8; `--rtf 0` mode |
| rclpy inside SimulationApp vs bundled bridge DDS | source system ROS first (documented pattern); fallback all-OmniGraph I/O |
| 6.0 API churn (post-cutoff APIs, node ids) | pin 6.0.1; Isaac Sim MCP semantic search resolves exact names during impl; 5.1 fallbacks + `og.get_registered_nodes()` without it; smoke test asserts registration |
| Sim Control services incomplete/renamed | fallback custom rclpy srvs on the USD stage (designed in) |
| 270°/10m lidar changes exploration behavior | intentional (approved); slam/costmap ranges adjusted; fresh baselines; loose A/B gate |
| EKF tuning rabbit hole | start from clearpath_control config; `--odom-noise 0` isolates EKF vs sensor issues |
| unitree G1 USD license/format | license check step; STL→USD fallback |
| Stock envs need asset download | only P7 depends on it; core worlds are repo-local USDs |
| VRAM contention on the shared box | nvidia-smi preflight in installer + runner (<16 GB warn); box-health agent updated |
| TwistStamped/Twist drift in future Nav2 | runner subscribes both, logs active type |

## Effort

P0 S · P1 M · P2 L · P3 L · P4 L · P5 M · P6 L · P7 M · P8 M. Long poles: P2 assets, P3 rig, P4 contract+EKF. Each phase = separate commit(s); gz functional until P8.

## Verification (port-level)

P5 A/B on mock_hospital via unchanged `explore_probe.py` (3/3 completion, coverage ≥ gz−10 pts, aborts ≤ gz max, RTF ≥0.8) + localization-error metric. P6 benchmark CSV sanity vs gz history. P7 stock-warehouse exploration with valid GT coverage. P8 fresh-clone drill.
