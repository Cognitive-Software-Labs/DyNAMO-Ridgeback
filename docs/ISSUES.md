# Issues and Troubleshooting

## Troubleshooting

| Problem | Check |
|---------|-------|
| Robot doesn't move | `ros2 topic echo /r100_0001/cmd_vel` - if empty, Nav2 may not be active |
| No map in RViz | `ros2 topic hz /r100_0001/map` - if 0, check `slam_toolbox` logs and the scan topic |
| Detection overlay does not appear | Make sure `target_localization_enabled:=true`, enable the `Perception overlay` Image display in RViz, and check `/r100_0001/debug/target/overlay` plus `/r100_0001/sensors/camera_0/color/image` |
| Perception overlay is a sliver in the narrow left dock | Expected until `exploration.rviz`'s `QMainWindow State` blob is regenerated — `addPane()` hardcodes the left dock area. Drag the pane to the bottom dock, stretch it full width, File → Save Config. If the *whole* layout reverted to defaults instead, `restoreState` rejected the blob and restored nothing |
| Fewer than four rings, or a missing HUD column | `ros2 node list` should show `target_pointcloud_measurement`, `target_mask_measurement`, `target_visualization` and `hud_target_node`; then `ros2 topic hz /r100_0001/measurements/target/mask`. A column reading `--` means that estimator ran and reported nothing; a column missing entirely means `estimators` did not select it |
| All three mask rows blank while `pointcloud` still reads | Suspect `base_frame`. Grep the startup log for `Configured base frame ... is unavailable`; a wrong frame skips the segmenter for the whole batch. See "A namespaced `base_frame` is not automatically a real TF frame" below |
| `explore_lite` not finding frontiers | Verify `track_unknown_space: true` in the global costmap config |
| TF errors | Ensure all nodes use `use_sim_time: true` |
| Startup hangs / a stage never comes up | Bringup is event-driven (readiness gates) — find the `gate_*` process log `[launch_wait]: waiting for …`; the `unmet:` list on timeout names the exact missing topic/service. See "Event-Driven Startup" below. Do **not** re-add `TimerAction` delays |
| Stale processes from previous runs | Run `bash cleanup.sh` before each launch |
| Gazebo/RViz vanished during a benchmark sweep | Do not run `cleanup.sh` between sweep configurations; it kills the persistent environment. Run it once before starting the supervisor |
| Diagnostics | Run `bash diag.sh /tmp/logfile.log hospital` or `bash diag.sh /tmp/logfile.log warehouse` |

## Benchmark Sweep Cleanup Trap

`cleanup.sh` is intentionally aggressive: it kills `gz sim`, `ruby.*gz`,
`gz-sim`, `rviz2`, ROS launch processes, and every package node it can find.
That is correct before a normal launch, but wrong between configurations in a
benchmark sweep. The point of `target_benchmark_sweep` is that Gazebo, RViz, the
Ridgeback, camera TF, detector, and clock survive for the whole comparison.

Run cleanup once before starting the supervisor. During the sweep, let the
supervisor stop only the per-config process group. It also checks Gazebo's pose
topic before each config and removes leftover `bench_*` models from a crashed
runner. If the supervisor itself is interrupted, rerun the same command to
resume completed `run.json` outputs; do not manually clean between its configs.

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

## FastDDS Shared-Memory Workaround

`start_exploration.sh` exports a UDP-only FastDDS profile (`fastrtps_no_shm.xml`) by default. This sidesteps the SHM failures listed below, which historically broke discovery on this setup after Gazebo/ROS crashes:

- `RTPS_TRANSPORT_SHM`
- `open_and_lock_file`
- `No unicast locators`

Related cleanup the script and tooling still do:

- `cleanup.sh` removes `/dev/shm/fastrtps_*` and `/dev/shm/sem.fastrtps_*`
- `diag.sh` looks for SHM-related FastDDS errors

### Toggling It On

The script defaults to the system RMW (shared memory on). If you hit stale
shared-memory lock symptoms, force the UDP-only profile with:

```bash
FASTRTPS_NO_SHM=true bash start_exploration.sh office
```

That exports `RMW_IMPLEMENTATION`, `FASTRTPS_DEFAULT_PROFILES_FILE`, and `RMW_FASTRTPS_USE_QOS_FROM_XML`. With the default (`false`) the script leaves them untouched, and the launch file (`ridgeback_exploration.launch.py`) does not set these on its own, so `ros2 launch` invocations honor whatever is in your shell env.

### A/B History

On April 12, 2026, the stack was A/B tested in the `office` world with and without the UDP-only profile. Both runs brought up Gazebo, `/clock`, SLAM, and the target-localization nodes; no SHM-specific FastDDS errors appeared on that machine in either run. The profile had previously fixed sim-bringup failures on a different machine, but since there was no observed downside to plain shared memory it is now off by default and kept available as an opt-in toggle.

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

### A namespaced `base_frame` is not automatically a real TF frame

Every launch caller must pass `base_frame` explicitly to the mask node. Its own
default is the bare string `base_link`
(`target_mask_measurement_node.BASE_FRAME_DEFAULT`), while the pointcloud and viz
nodes derive `<namespace>/robot/base_link` from `get_namespace()` at
construction, and `target_benchmark_config.launch.py` declares the same
namespaced default.

**A topic namespace does not establish what `frame_id` strings exist inside
`/tf`.** These are two independent naming systems, and this repository's
simulated graph disagrees with the assumption on both sides: the topics are
namespaced, but the frames are not. In the `target_distance_calibration`
benchmark, `r100_0001/robot/base_link` does not resolve and bare `base_link`
does — the opposite of what the configured default expects. Both the pointcloud
and mask nodes log the fallback once at startup:

```text
Configured base frame "r100_0001/robot/base_link" is unavailable; using "base_link" for transforms.
```

Do not read that as "every deployment uses `base_link`" either. The frame that
exists is whatever the robot's TF publishers emit; hardware has not been tested.
Check it directly rather than deriving it from the namespace:

```bash
ros2 run tf2_tools view_frames        # writes frames.gv/frames.pdf listing every frame
ros2 topic echo /r100_0001/tf --once  # read header.frame_id / child_frame_id directly
```

The failure is **silent in the row**, which is the trap. `base_frame` feeds the
mask node's `camera_extrinsic_for_batch`, which gates all three mask rows: a miss
stamps the whole frame `TF_MISS_EXTRINSIC` and skips the segmenter entirely, so
the HUD shows three estimators that ran and found nothing rather than one bad
frame. `launch_common.mask_measurement_node()` makes `base_frame` a required
keyword argument, so a new caller cannot inherit the bare default by omission.

### The fallback used to cost half a second per lookup

`common/tf_utils.lookup_transform_components()` tries the configured frame, then
its last path segment. Until 2026-08-31 it gave **each** candidate the full
`TF_LOOKUP_TIMEOUT_SEC` (0.5 s). A configured frame nothing publishes never
resolves, so every call spent that timeout before reaching the fallback that was
already sitting in the buffer. `last_fallback_frame` suppressed the repeat
*warning* but not the repeat *wait* — the cost was invisible in the log and paid
on every call.

Measured on the live benchmark graph: 502.8–512.0 ms per namespaced lookup
versus 0.021–0.036 ms when asking for `base_link` directly, with identical
rotation and translation returned.

That is a throughput bug, not just latency. The mask node looks the extrinsic up
once per detected batch, and its pending-detections slot is latest-wins. At
~4.2 incoming batches/s, a worker blocked ≥0.5 s per batch overwrites pending
work and processes only ~2 batches/s; every overwritten batch scores `UNSET`.

**Fix:** the helper now runs the same candidates in the same order twice — first
with a zero timeout, then, only if nothing was buffered, with the original
bounded per-candidate wait. An available fallback returns immediately; a
transform that genuinely has not arrived yet still gets its wait. Nothing is
cached between calls, so a configured frame that starts publishing later wins
again on the next call. `test_tf_utils` covers this with a recording buffer
rather than a wall-clock assertion.

If mask coverage is low, split `UNSET` from `NO_DEPTH_FRAME` in `run.json`
before diagnosing: `UNSET` means no mask result was produced for that
observation at all (the failure above), while `NO_DEPTH_FRAME` means the batch
ran but its exact-stamp depth input was missing — a separate, still-open loss.
