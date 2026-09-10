# Port plan: Gazebo Harmonic → NVIDIA Isaac Sim 6.0

Status: **in progress — P0–P4 + P7 done; P5 plumbing done but A/B sign-off open; a post-P7 sensor/SLAM-input batch landed 2026-09-02 and is NOT re-validated; P6 (G1 benchmark) deferred; P8 (gz removal) next.** Kept current per phase; mark phases done as they land.
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
2. **Real UST-10LX lidars**: 270° FOV, 0.05–10 m, realistic range noise, 40 Hz, custom RTX profile JSON. Front+rear pair still covers 360° combined. **Coplanar (2026-09-10):** the rear lidar used to sit 5 cm above the front purely to dodge a gz artifact — gz's idealized "hokuyo" is 360°, so at equal height the two lidars saw *each other*. Isaac's RTX lidar honors a real 270° emitter window (`validStartAzimuthDeg -135` / `validEndAzimuthDeg +135` in `ust10lx_2d.json`), and each lidar sits at bearing 180° in the other's frame — 45° outside the window, against a ~±3.7° angular width for the body at 0.784 m separation. So the workaround is unnecessary here and both lidars are now coplanar: the merged scan is one honest plane instead of two 5 cm apart. This deliberately breaks the gz lidars (they will see each other) — accepted, gz dies in P8, and the gz baselines in `tools/isaac/baseline/` are already captured and frozen.

  **Mounting corrected the same day (2026-09-10), and this one was a real modelling error, not a workaround.** The config mounted both units on `default_mount`, which is the **top deck surface** (0.295 m above `base_link`) — but on the physical robot the lidars sit recessed in a diamond notch cut into the body's end faces, *under* the top plate, so the whole sensor was **11.6 cm too high** and every Isaac coverage number to date was taken scanning above obstacles it should have been hitting. Corrected to `parent: base_link`, `xyz: [±0.3922, 0.0, 0.179]` → laser plane **0.2264 m above `base_link` = 0.252 m off the floor** (`base_link` is 0.0259 m up: axle at 0.050, mecanum wheel radius 0.0759). Confirmed live in the TF tree after regen: front `[0.392, 0, 0.226]`, rear `[-0.392, 0, 0.226]` yaw 180°.

  Evidence trail, since the numbers were contested twice before settling: `hokuyo_ust.stl` spans z 0.0000–0.0700 so its **origin is the unit's base**, which is what a tape measures to; owner measured base at 20/21 cm, glass band 23.5–26 cm; base 0.205 + the URDF's 0.0474 laser offset = 0.2524, landing ~65% up the glass against the mesh's 67.7% up the casing. The `x = ±0.3922` was right all along — a vertical ray cast through `body.stl` + covers at both lidar spots returns **no** surfaces while the centre returns four, i.e. the notch is real geometry and `±0.3922` is its centre. Notch clearance re-checked at the new height: **0 of 271 bearings** blocked within 0.35 m across the full ±135°, so lowering the sensor does not bury it in its own hull. (An earlier claim in review that the lidars overhung the riser plate by 0.146 m was **wrong** — it measured against `riser_link`'s 0.493 m box primitive, which is internal structure, not the 0.932 × 0.790 m `top.stl` deck.)

  **Marks all Isaac sensor-geometry baselines for rerun** — both the coplanar change and the 11.6 cm drop. Config deltas (the only ROS-side YAML touches, physically motivated): `slam_toolbox_params.yaml` `max_laser_range` 20→10; review costmap `obstacle_max_range`/`raytrace_max_range` vs 10 m sensor. Historical continuity intentionally broken → fresh Isaac baselines.
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

### P7 — Stock environments + USD GT maps + repeat harness [M] ✅
- ✅ **Stock worlds load.** `worlds.get_assets_root()` (isaacsim.storage.native) feeds `resolve_world`, so `warehouse`/`office`/`hospital` stream from the NVIDIA S3 asset root (`STOCK_WORLDS`). **Robot-compose fix (a 6.0.1 landmine):** referencing the local robot USD onto a stage whose root layer is a remote S3 URL 404s — `AddReference` URL-joins a bare local path against the remote anchor. Fixed by an absolute `file://` URI (`Path(robot_usd).as_uri()`). PhysicsScene fallback authors `/physicsScene` if a stock USD ships without one. Verified live: warehouse streams from S3, `articulation root /ridgeback/drive_rig/world_fix`, sensors attach, RUNNER READY.
- ✅ **`tools/isaac/generate_gt_map.py`** — analytic GT from the world USD. NOT the omap extension (needs `timeline.play()`, which trips an `omni.graph.core` crash on these stages) and NOT bbox rasterization (a consolidated wall mesh → whole-floor AABB): a pure-pxr **triangle slice at the lidar plane** (no physics, contention-immune), handling Mesh + native instance proxies + PointInstancer (proto-root-relative compose). A **dominant-cluster spatial crop** drops the far skybox/backdrop geometry stock envs ship (a count-based percentile can't — a heavily-tessellated far prop survives it). Fresh maps: warehouse 440×680, office 760×2040, hospital 1560×880 (`.pgm`/`.yaml`/`.png`/`.npz`), each visually verified as a real floor plan. Old gz-captured warehouse/office + mock_hospital maps → `sim/ground_truth_maps/historical/`; README rewritten (analytic canonical; gz manual capture = historical). mock_hospital dropped from the GT set (Cube/Sphere-based → the Mesh slicer skips it; it has the exact SDF path via `gt_occupancy.py`, and the world is being retired).
- ✅ **In-session reset.** `ros_io` `sim/reset` (std_srvs/Trigger) → runner teleports to spawn + re-zeros odom on the main loop (physics ops off the callback thread). `explore_probe.py --repeat N`: observe → per-run summary → reset (runner `sim/reset` + `slam_toolbox/reset` + both costmap clears) → settle → repeat; combined summary at the end (repeat=1 keeps the unchanged single-run schema, ab_compare-compatible).
- ✅ **Nav2 fix for stock worlds.** The global costmap was static/map-synced (no `rolling_window`); a stock-env slam_toolbox seeds `/map` with an origin offset from the (0,0) spawn, so every plan aborted "start outside bounds" and the robot never moved. Made the global costmap rolling (60 m window; ⊇ the small worlds' maps → their planning is unchanged, no A/B regression).
- ✅ **Live acceptance** (`start_exploration.sh warehouse sim:=isaac sim_mode:=deterministic camera:=false g1_perception_enabled:=false`, isolated `ROS_DOMAIN_ID`): RUNNER READY, robot spawns+drives, SLAM maps, coverage HUD live — **34% complete / 93% accuracy** on the near-empty stock `warehouse.usd` (its own frontier ceiling, not a gate). `--repeat 2`: run 1 = **89/89 goals, 0 aborts, 4 cm mean loc-err**, summary written; reset all-ok (runner teleport confirmed in-log, `runner/slam/global/local:ok`); run 2 re-explored (94 goals). Achieved RTF 0.54 under a co-tenant GPU job (deterministic mode kept SLAM correct). **Deviation:** coverage carries across a reset — `slam_toolbox/reset` clears the pose graph but not the already-published occupancy grid, so run 2 is a real run but not a blank-map run (documentable refinement; `--repeat` still yields N summaries without relaunch). office + hospital: runner smoke green — both stream from S3, robot composes (`world_fix` root), **and both fire the PhysicsScene fallback** (they ship without one, unlike warehouse — the fallback was genuinely needed), RUNNER READY; full exploration is world-agnostic (same rolling-costmap fix). Bench hygiene needs the CycloneDDS env + `ros2 daemon stop` (a stale daemon hides the namespaced topics from the CLI; rclpy probes are unaffected).

### Post-P7 batch (2026-09-02) — landed on the branch tip, ⚠️ NOT re-validated

Three commits that sit outside the phase numbering (they refine P4/P7 deliverables rather than opening a new phase). All three were developed and measured in a since-discarded working tree; **none has been re-run against a fresh Isaac boot from this clean checkout**, so treat every performance claim below as "claimed, unconfirmed".

- **`ca903276` — `warehouse_full` GT map.** The stock `full_warehouse.usd` (7 racking rows + open staging) replaces the near-empty `warehouse.usd` as the real exploration workout — the latter has its own ~40% frontier ceiling that makes coverage gates meaningless. Analytic map, 34×58 m. Gotcha: dense geometry lands `generate_gt_map.py`'s auto flood-seed on a shelf (free=277 cells), so it needs `--origin=-10,5` on open floor; documented in the maps README table.
- **`e647ac5a` — lidar + joint-state runtime correctness.** (a) `LidarScanAssembler` no longer carries a bin forward across ticks: a published bin comes only from the current completed tick, unrefreshed bins go `+inf`. Claimed to cut rotation-induced map smear. (b) Runner publishes a static all-zero wheel `JointState` — the planar drive rig never exposes the URDF wheel joints as articulation DOFs, so `robot_state_publisher` had no source for the four wheel-link transforms.
- **`1a4079b9` — front+rear scan merge for SLAM input only.** New `common/scan_merger_node.py`: pairs front/rear `LaserScan` within a stamp tolerance, motion-compensates rear into front's capture time via the **odom→base_link** chain (never slam_toolbox's own `map→base_link` — that would be circular), publishes a 360° virtual scan on `sensors/scan_slam_merged`. No stale ranges, no interpolation; unobserved bins stay `+inf`; a genuine bin collision keeps the nearer return. Raw `lidar2d_{0,1}/scan` keep publishing unchanged to every other consumer (Nav2 costmaps, collision_monitor). `slam.launch.py` gains `slam_source` (`front_only` default / `merged`) switching only slam_toolbox's `scan_topic`; the merger node launches only under `merged`. `ridgeback_exploration.launch.py` defaults it to `merged` under `sim:=isaac`, `front_only` otherwise — so gz behavior is untouched.

**Merger audit (2026-09-10) — two bugs found and fixed before any live run.** Proven offline with synthetic scans on an isolated domain (no Isaac needed; harness pattern: rear lidar sees one wall 3.0 m straight behind, front sees nothing, so the merged bin at `π` must read 3.3922 m — `+inf` means the rear scan was dropped):

- **The merger was launched without the `/tf` remap.** `tf2_ros.TransformListener` subscribes to the *absolute* `/tf`, so a node namespace does not move it, and the whole stack publishes into `<ns>/tf`. Its buffer was therefore permanently empty, every `odom→base_link` lookup failed, and **every rear scan whose stamp differed from the front's was silently discarded** — `merged` mode degraded to front-only. Measured: stamps 10 ms apart with TF on `<ns>/tf` only → rear bin `+inf`; add a global `/tf` publisher → 3.3922 m. Fixed by adding `remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]`, with a regression test (`test_scan_merger_node_remaps_tf_into_the_namespace`) so it cannot silently come back. **This was latent in sim** — both Isaac lidars stamp off the same tick, hitting the `abs(delta) < 1e-6` static-extrinsic fast path that never consults TF — and would have bitten on real hardware, where the two lidars have independent clocks.
- **`range_max` was copied from the sensor frame into a `base_link` message.** The lidars sit ±0.3922 m off `base_link`, so a return at the front sensor's 10.00 m range_max lands at 10.39 m from `base_link` — above the declared `range_max`, so slam_toolbox discarded it. Merged mode was losing the outermost shell of returns that `front_only` keeps. Fixed by padding `range_max` with the largest lidar offset.
- **The health log overstated itself:** a TF-miss merge still incremented `paired`, so the counters read `paired=4 tf_misses=1` for four publishes of which one carried no rear data at all. Split out `rear_dropped(no odom TF)`; the same probe now reads `paired=3 rear_dropped=1`.
- Not bugs, checked: the hardcoded planar extrinsics match the robot USD chain (base_link → riser_link `(0,0,0.22)` → default_mount `(0,0,0.075)` → lidar link), identity rotation throughout, so front `(0.3922, 0, 0)` / rear `(-0.3922, 0, π)` are right; bin math (1440 bins, `angle_min=-π`, nearest-wins) is right; the correction composes to `T_front←rear` off the odom chain only, never `map→base_link`. The 5 cm rear-lidar height offset the audit flagged (a gz-only workaround) was removed the same day — see improvement 2 above; both lasers are now coplanar at 0.3424 m.

Validation still owed: fresh-boot run with `odom_noise:=0` first (perfect odom isolates the merge geometry from the drift lever P5 already measured), then the noisy case. Confirm there whether Isaac really does stamp front and rear identically — if it does, the motion-compensation path is dead code in sim and only the real robot exercises it.

### Vendor chassis graft (2026-09-10, branch `feat/isaac-vendor-chassis`)

The URDF importer produced a coarse body — open gaps under the deck, **no rear end panel at all**, flat untextured materials. NVIDIA ships an authored Ridgeback in the Isaac asset catalog (`/Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd`, BSD-3-Clause, Clearpath Robotics, from `ridgeback_manipulation`) whose hull matches ours to a few mm but is closed, chamfered and textured. `tools/isaac/extract_vendor_chassis.py` pulls the chassis out; `import_ridgeback_urdf.py:graft_vendor_chassis` references it and hides the 12 imported prims it replaces.

- **Geometry needed no re-referencing.** Both models bottom out at z = −0.0262 (wheel contact plane) and agree in y to 1.4 mm, so the vendor root *is* our `base_link` and the tape-measured 0.179 lidar mount carries over untouched. Verified after regen: laser frames at `(±0.3922, 0, 0.2264)`.
- **Wheels stay ours** (articulated, they spin); the vendor drives its base as one rigid body with static wheels, so taking theirs would double them. The UR5, its mount plate, the dummy joint chain and the vendor `physicsScene` are dropped — a second articulation root would fight ours.
- **Collider: `convexHull`, decided 2026-09-10.** Measured on the chassis collision mesh: true volume 0.15968 m³, `convexHull` 0.17064 (**+6.9 %**), the AABB `Cube` it replaces 0.20018 (**+25.4 %**). The hull keeps 119 verts / 234 facets against the source's 972 / 324, so it tracks the real form; what it over-claims is the underside cavity between the wheels, which the wheel cylinders already occupy and nothing else reaches. `convexDecomposition` would chase that last 6.9 % at per-step contact cost — rejected. Compare them with `tools/isaac/inspect_robot.py --compare-colliders`.
- **Self-occlusion cleared, and an earlier theory refuted.** In `sim/isaac/usd/worlds/empty.usda` (added for this — floor slab, nothing else, so any finite return is necessarily the robot seeing itself) both lidars read **0/1081 finite bins** across the full ±135°, before *and* after the graft. The "grazing the side cover at 0.52 m" explanation for the Nav2 stall was **wrong**; that geometry never occluded the lidars.

Two traps this uncovered, both silent:

- `import_ridgeback_urdf.py` `rmtree`s the entire committed robot directory during its flatten step, which also deletes the vendored chassis and its licence. They are now stashed and restored across the rebuild.
- Kit runs with `--/app/fastShutdown=True`, so `app.close()` **hard-exits the process** — a post-processing step appended after `import_urdf_to_usd()` returns never runs, while the script still prints `IMPORT OK` and exits 0. Anything post-import must sit inside, before the close.

Known defect, not addressed: the **D435 floats at z = 1.145–1.170 m** with no supporting structure — `robot.yaml` parents it to `default_mount` (the 0.295 deck) with a further `xyz: [0.3, 0, 0.85]`. Owner confirms the real robot has a mast, so the height is right and the *mast geometry* is what the model lacks. Same shape of error as the lidar mount, opposite resolution. Camera plays no part in SLAM, collision monitoring or any benchmark (every run uses `camera:=false`).

⚠️ **Rerun debt grows again:** the graft changes rendered geometry the RTX lidar raytraces, so the GT slice wants re-checking and every coverage number is void until it is.

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
