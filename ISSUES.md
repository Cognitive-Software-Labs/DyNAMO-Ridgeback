# Issues and Troubleshooting

## Troubleshooting

| Problem | Check |
|---------|-------|
| Robot doesn't move | `ros2 topic echo /r100_0001/cmd_vel` - if empty, Nav2 may not be active |
| No map in RViz | `ros2 topic hz /r100_0001/map` - if 0, check `slam_toolbox` logs and the scan topic |
| Detection overlay does not appear | Make sure `g1_perception_enabled:=true`, remember the overlay is a separate OpenCV window, and check `/r100_0001/sensors/camera_0/color/image` |
| `explore_lite` not finding frontiers | Verify `track_unknown_space: true` in the global costmap config |
| TF errors | Ensure all nodes use `use_sim_time: true` |
| Startup hangs / a stage never comes up | Bringup is event-driven (readiness gates) — find the `gate_*` process log `[launch_wait]: waiting for …`; the `unmet:` list on timeout names the exact missing topic/service. See "Event-Driven Startup" below. Do **not** re-add `TimerAction` delays |
| Stale processes from previous runs | Run `bash cleanup.sh` before each launch |
| Diagnostics | Run `bash diag.sh /tmp/logfile.log hospital` or `bash diag.sh /tmp/logfile.log warehouse` |

## Event-Driven Startup (Readiness Gates)

Bringup is sequenced by **readiness gates**, not fixed timers. Each stage waits
for its real precondition to exist, then the next stage starts immediately
(`RegisterEventHandler(OnProcessExit(...))`). This cut explorer-ready from a
fixed ~105 s to roughly real-ready (~10–15 s on a fast machine) and let us delete
the blind 92 s Nav2 lifecycle re-startup that used to paper over an
`odom→base_link` TF race at activation.

The gate helper is `ros2 run ridgeback_autonomy launch_wait`
(`ridgeback_autonomy/common/launch_wait.py`): it blocks until all `--topic`
publishers / `--tf` lookups / `--service` names appear, then exits 0. The
per-gate `--timeout` (30–60 s) is a **safety fallback** — on timeout it logs a
warning and proceeds anyway, so a missing input slows startup but never deadlocks
it. Topic checks use `count_publishers` (a publisher existing), so they're
QoS-agnostic (scan is best-effort, map is transient-local).

Chain in `ridgeback_exploration.launch.py`:

| Gate | Waits for | Then starts |
|------|-----------|-------------|
| `gate_slam` | `…/sensors/lidar2d_0/scan` + `…/platform/odom/filtered` publishers | SLAM (`slam.launch.py`) |
| `gate_slam_configure` | `…/slam_toolbox/change_state` service | SLAM `CONFIGURE`; `ACTIVATE` then fires on the configured-state event (`OnStateTransition`) |
| `gate_nav2` | `…/map` publisher | Nav2 (`nav2.launch.py`) |
| `gate_explorer` | `…/global_costmap/costmap` publisher | explorer (`explore.launch.py`) |

`manual_mapping.launch.py` uses the same pattern (SLAM gated on scan+odom; the
drive-controller activation gated on `…/controller_manager/switch_controller`).

**Why `…/map` gates Nav2 instead of a TF check:** the robot's TF is published on
the namespaced `/<ns>/tf`, so a plain TF listener (on `/tf`) sees nothing. A
`…/map` publisher existing implies SLAM is active and publishing `map→odom`,
i.e. the whole `map→odom→base_link` TF chain is alive — so Nav2 only activates
once TF is ready, which is what removed the activation race.

**If a stage stalls:** read its `gate_*` log line `[launch_wait]: waiting for …`;
the `unmet:` entries it prints on timeout are the missing topic/service. Fix that
input, or — if the machine is just slow and the input does eventually appear —
raise that gate's `--timeout`. Don't reintroduce fixed `TimerAction` delays.

## Platform Command Timestamp Warnings

During exploration runs, Gazebo may occasionally log `platform_velocity_controller` messages like `Received message has timestamp ... older by ... than allowed timeout (0.1000)`.

This is usually ROS/Gazebo timing jitter around Clearpath's generated `reference_timeout: 0.1` in `/home/deivid/clearpath/platform/config/control.yaml`. Do not change the generated Clearpath controller config as a first response. First check whether Nav2 retarget churn or collision-monitor approach flicker is causing irregular command timing, then instrument `/cmd_vel`, `/cmd_vel_smoothed`, `/platform/cmd_vel`, sim-time rate, and controller update timing if the warnings remain frequent after startup.

## Historical: slam_toolbox TF Namespace Issue

This project requires `patches/slam_toolbox_tf_namespace.patch` because `slam_toolbox` otherwise fails in a namespaced Clearpath setup.

### Problem

Clearpath robots require a non-empty ROS 2 namespace such as `/r100_0001/`. All topics, including TF, live under that namespace: `/r100_0001/tf` and `/r100_0001/tf_static`. The normal ROS 2 pattern is:

```python
remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
```

That works for Nav2, `explore_lite`, and the other nodes in this stack, but not for the stock `slam_toolbox` build.

### Root Cause

In `slam_toolbox_common.cpp`, `slam_toolbox` creates its `TransformListener` like this:

```cpp
tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
```

That constructor creates a hidden internal node which subscribes to the absolute topics `/tf` and `/tf_static`. Because that internal node is separate from `slam_toolbox`'s own node, it ignores the launch remappings. The result is that `slam_toolbox` listens to the global `/tf` instead of `/r100_0001/tf`, its laser `MessageFilter` drops scans, and no map is produced.

### Fix

The local patch changes that one line to:

```cpp
tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_, shared_from_this());
```

Passing `shared_from_this()` makes the listener use `slam_toolbox`'s own node interface, so it respects the `/tf` -> `tf` remapping and subscribes to the namespaced TF topics correctly.

### What We Tried Before the Patch

1. `tf_relay` nodes forwarding `/r100_0001/tf` to `/tf`
2. process-level remapping through `ExecuteProcess`
3. larger `scan_queue_size` values

None of those resolved the underlying subscription problem. The 1-line source patch was the change that made the namespaced setup work reliably.

## DDS Middleware (CycloneDDS default; FastDDS fallback)

`start_exploration.sh` now defaults to **CycloneDDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=cyclonedds.xml`). It proved more robust on this multi-NIC host than FastDDS shared memory. The Cyclone config pins the **loopback** interface (multiple UP NICs otherwise cause `failed to create domain`), raises **`MaxAutoParticipantIndex`** (the 40+ node stack exhausts the default → `Failed to find a free participant index`), and requests **large socket buffers** for camera/costmap/point-cloud bursts. Those buffers need a raised kernel ceiling — run `tools/setup_dds.sh` once (sudo; installs `/etc/sysctl.d/60-ros-dds.conf` for `net.core.rmem_max` etc., persisted across reboots). ref: https://www.stereolabs.com/docs/ros2/dds-and-network-tuning

To fall back to FastDDS, set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`; the script then defaults `FASTRTPS_NO_SHM=true` to use the UDP-only profile described below.

### FastDDS Shared-Memory Workaround (fallback path)

The UDP-only FastDDS profile (`fastrtps_no_shm.xml`) sidesteps the SHM failures listed below, which historically broke discovery on this setup after Gazebo/ROS crashes:

- `RTPS_TRANSPORT_SHM`
- `open_and_lock_file`
- `No unicast locators`

Related cleanup the script and tooling still do:

- `cleanup.sh` removes `/dev/shm/fastrtps_*` and `/dev/shm/sem.fastrtps_*`
- `diag.sh` looks for SHM-related FastDDS errors

### Using the FastDDS fallback

CycloneDDS is the default. To switch back to FastDDS, set `RMW_IMPLEMENTATION`:

```bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp bash start_exploration.sh office
```

On that path the script defaults `FASTRTPS_NO_SHM=true`, exporting `FASTRTPS_DEFAULT_PROFILES_FILE` (the UDP-only profile) and `RMW_FASTRTPS_USE_QOS_FROM_XML`. Set `FASTRTPS_NO_SHM=false` alongside it to run FastDDS with the system default (shared memory) instead. The public launches (`ridgeback_exploration`, `g1_distance_benchmark`) set the same RMW default but respect an explicit override, so `ros2 launch` matches the script.

### A/B History

On April 12, 2026, the stack was A/B tested in the `office` world with and without the UDP-only FastDDS profile; neither run showed SHM-specific errors on that machine. The UDP-only profile had previously fixed sim-bringup failures on a different machine. The project later moved its default off FastDDS entirely to **CycloneDDS** (more robust on this multi-NIC host); the FastDDS UDP-only profile remains the opt-in fallback.

## Simulation RTF Collapse: GPU Rendering Lost to Seat ACL

**Symptom**: Gazebo real-time factor drops from ~0.9 to 0.02–0.08. Exploration
crawls, `platform_velocity_controller` floods `Received message has timestamp …
older by …` errors (commands get dropped as stale), Nav2 recovery churn.
Launch log shows `libEGL warning: failed to open /dev/dri/renderD128:
Permission denied` and `nvidia-smi` shows 0% GPU while a `gz sim server`
thread pins one CPU core — all sensor rendering (GPU lidars, RGBD camera)
falls back to Mesa llvmpipe software rendering.

**Root Cause**: `/dev/dri/*` is `root:render`/`root:video` with an ACL that
systemd-logind grants only to the *active desktop seat* user (check with
`getfacl /dev/dri/renderD128`). On this shared box the seat can belong to a
display manager or another user, so shell/SSH sessions of other users get no
GPU device access. Runs are fast only while your user holds the seat ACL.
Verified 2026-07-09: historical logs show RTF ≈ 0.91 (June runs, seat owned)
vs 0.04 same stack, same config (seat lost).

**Fixes**:
- Immediate (per boot): `sudo setfacl -m u:$USER:rw /dev/dri/renderD128 /dev/dri/card*`
- Durable: `sudo usermod -aG render,video $USER`, then re-login.
- **Even with `/dev/dri` access**, Mesa cannot drive the NVIDIA card for GLX
  direct rendering (`libEGL warning: pci id … 10de:…, driver (null)`), so the
  default X11/GLX sensor-rendering path stays on llvmpipe. The working
  GPU path for SSH/non-seat sessions is NVIDIA EGL headless
  (`/dev/nvidia*` is world-accessible, no `/dev/dri` needed):

  ```bash
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  bash start_exploration.sh mock_hospital headless_rendering:=true
  ```

  `headless_rendering:=true` (new launch arg, plumbed through the patched
  `clearpath_gz` launches as `gz sim --headless-rendering`) makes the server
  render sensors via EGL instead of GLX/X11. Verified 2026-07-09: RTF
  0.04 → ~1.0 with this combination. The Gazebo GUI window still opens and
  renders on llvmpipe; that part is cosmetic.

**Triage order when the sim is slow** (see also the shared-box note): check
`getfacl /dev/dri/renderD128` and `grep libEGL <launch log>` first, then
co-tenant CPU load (`uptime`, `ps -eo pcpu,user,comm --sort=-pcpu | head`).

## Phantom Coverage "Collapse" — Stale HUD Nodes Surviving cleanup.sh

**Symptom**: The HUD `complete` metric appears to collapse instantly (e.g.
~80% → ~39%) and then "flap" between two values every second, while `accuracy`
stays ~97%. Recorded coverage curves look like a mid-run SLAM catastrophe.

**Root cause (found 2026-07-10)**: not SLAM at all. `cleanup.sh`'s kill list
was missing `coverage_overlay_node` and `hud_node`, so every launch leaked one
of each. After N runs, N coverage nodes (some configured for *other worlds'*
ground-truth maps) all subscribe to the **same** `/r100_0001/map` (topic names
are identical across runs) and all publish onto the **same** `hud/coverage`
topic. Subscribers see interleaved values from every generation: nodes with
the current world's GT agree (true value), stale nodes configured for a
different world score the map against the wrong GT (bogus value) → 1 Hz
flapping. Verified with `ros2 topic info -v /r100_0001/map` (15 subscriptions
named `coverage_overlay_node`) and 14 `hud/coverage` messages per tick.

**Fix**: `coverage_overlay_node` and `hud_node` added to `cleanup.sh` PATTERNS
and to its verification greps.

**Diagnosis recipe when a HUD metric looks impossible**: check for multiple
publishers first — `ros2 topic info -v /r100_0001/hud/coverage` (or the
metric's topic). More than one publisher/subscription with the same node name
= stale processes from previous launches; extend `cleanup.sh` accordingly.

**Benchmarking note**: coverage CSVs recorded before this fix (2026-07-09/10
warehouse A/B runs) contain interleaved multi-node values; the *upper
envelope* of `complete` is the live node's true value.

## Exploration Quits Early — explore_lite Frontier Blacklist Exhaustion

**Symptom**: `explore_lite` logs "All frontiers traversed/tried out, stopping."
minutes into a run with roughly half the map unexplored (warehouse: quits at
280–450 s with ~45–48% coverage). Quit time varies wildly between
identical-config runs.

**Root causes (found 2026-07-10, instrumented warehouse runs C–G)** — three
stacked failure modes, each ending in the same blacklist-exhaustion quit:

1. **Preempted goals blacklisted as failures.** explore_lite re-targets the
   best frontier every planner tick; nav2 reports each preempted goal as
   `ABORTED`, and upstream explore_lite blacklists every aborted goal. Under
   normal goal churn (~85 preemptions per warehouse run) the blacklist slowly
   consumes the frontier list. Fixed in
   `patches/m_explore_customizations.patch`: only count an abort when the
   goal is still the one being pursued (genuine navigation failure).
2. **Wall-flush frontier centroids unplannable at tolerance 0.75.** Frontier
   centroids often sit inside the inscribed-lethal band of the 0.75 m
   inflation; NavFn finds no endpoint within its 0.75 m tolerance, the goal
   aborts, the frontier gets blacklisted. Fixed in `nav2_params.yaml`:
   `GridBased.tolerance: 1.5` (goals succeeded went 0 → 11 in the first
   validation run).
3. **Transient TF/controller outages cascade.** Under co-tenant CPU load
   (see the shared-box note below), the controller loop drops from 20 Hz to
   2–8 Hz and `map→odom` lags by up to ~1.5 s; every active goal aborts with
   "Unable to transform goal pose into costmap frame" within 1–2 s of being
   sent. One such burst blacklisted 8 frontiers in 18 s and ended run F. Fixed
   in the same m-explore patch: a frontier is only blacklisted after
   `abort_blacklist_threshold` (default 3) genuine failures, and retries wait
   for the next planner tick, so a seconds-long outage can't burn every
   attempt.

**Validation (warehouse, fixed HUD)**: baseline quit 448 s / 45.2% coverage;
with all three fixes the run survived 720 s under ~6× worse TF-error
conditions (box load 40+/32 from co-tenant training). Coverage stayed ~47%
in that run because the controller couldn't complete goals at 5–8 Hz — a
load ceiling, not explorer logic; re-benchmark coverage on a quiet box.

**Diagnosis recipe for early exploration quits**: count
`grep -c "Received goal preemption request"` vs
`grep -c "Blacklisting unreachable"` in the launch log. Preemptions >> real
failures + a growing blacklist = failure mode 1. Repeated
`Failed to create plan with tolerance` at the same coordinates = mode 2.
`Exception in transformPose … extrapolation into the future` bursts +
`Control loop missed its desired rate` = mode 3 (check co-tenant load
first: `uptime`).

## SLAM Drift in Featureless Environments (Office World)

**Symptom**: After launching in the office world, the robot appears to jump/move randomly in RViz (map→odom transform drifts) while the robot remains physically stationary in Gazebo.

**Root Cause**: `minimum_travel_distance: 0.0` and `minimum_travel_heading: 0.0` in `slam_toolbox_params.yaml` cause slam_toolbox to process *every* incoming scan even when the robot has not moved. In the warehouse world the shelving provides many distinctive scan features, so small mismatches stay bounded. The office world has large open areas with uniform walls; each scan-to-scan mismatch is small but uncorrected, and the accumulated drift eventually makes the map→odom TF spin the robot around in RViz.

**Fix**: Set `minimum_travel_distance: 0.05` and `minimum_travel_heading: 0.05` in `slam_toolbox_params.yaml`. This gates scan processing to moments when the robot has moved ≥ 5 cm or rotated ≥ 3°, eliminating spurious updates while stationary.

## Camera Optical Frame TF in Simulation

### Problem

The lidar estimator requires a TF transform from `camera_0_color_optical_frame` to `base_link` to project scan points into camera space and gate them against the detection bounding box. Without it, the node logs an error and produces no lidar measurements.

### Root Cause

On real hardware, the `realsense2_camera` driver reads factory-calibrated extrinsics from the camera firmware and publishes them as TF at runtime — including the `camera_0_link` → `camera_0_color_optical_frame` chain. Because the driver owns these frames, the D435 URDF (`d435.urdf.xacro`) gates the same joints behind `use_nominal_extrinsics=false` (the default) to avoid a conflict. In simulation there is no driver — Gazebo stamps image messages with `camera_0_color_optical_frame` (via `<optical_frame_id>` in `intel_realsense.urdf.xacro`) but never publishes the corresponding TF, leaving the tree incomplete.

### Fix

Both launch files include a `static_transform_publisher` node behind `IfCondition(use_sim_time)` that publishes the nominal version of the transform:

```
camera_0_link → camera_0_color_optical_frame
  xyz = 0  0.015  0          (colour lens offset from d435.urdf.xacro)
  rpy = -π/2  0  -π/2       (standard ROS optical frame rotation)
```

This node does **not** start on a real robot (`use_sim_time:=false`), where the RealSense driver takes over.

### Symptom If Missing

The lidar node logs:
```
[ERROR] Lidar measurement skipped: TF lookup from "camera_0_color_optical_frame" to
"r100_0001/robot/base_link" failed: ... In simulation, ensure the
camera_0_color_optical_tf static_transform_publisher is running (requires use_sim_time:=true).
```

## Namespace Gotchas

If you add new nodes to this project, always:

1. Set `namespace=namespace` on the node
2. Add `remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]`
3. Use `/**/node_name:` as the YAML root key in parameter files so namespaced nodes still match their params
4. Set `use_sim_time: true` in simulation
