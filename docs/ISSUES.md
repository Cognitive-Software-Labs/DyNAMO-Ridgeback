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

## SLAM Drift in Featureless Environments (Office World)

**Symptom**: After launching in the office world, the robot appears to jump/move randomly in RViz (map→odom transform drifts) while the robot remains physically stationary in Gazebo.

**Root Cause**: `minimum_travel_distance: 0.0` and `minimum_travel_heading: 0.0` in `slam_toolbox_params.yaml` cause slam_toolbox to process *every* incoming scan even when the robot has not moved. In the warehouse world the shelving provides many distinctive scan features, so small mismatches stay bounded. The office world has large open areas with uniform walls; each scan-to-scan mismatch is small but uncorrected, and the accumulated drift eventually makes the map→odom TF spin the robot around in RViz.

**Fix**: Set `minimum_travel_distance: 0.05` and `minimum_travel_heading: 0.05` in `slam_toolbox_params.yaml`. This gates scan processing to moments when the robot has moved ≥ 5 cm or rotated ≥ 3°, eliminating spurious updates while stationary.

## Camera Optical Frame TF Ownership

### Problem

Every mask estimator needs a TF transform from `camera_0_color_optical_frame` to the base frame to project between camera space and the robot. Without it the node stamps the whole frame `TF_MISS_EXTRINSIC` and produces no measurements.

### Who owns that transform

Exactly one publisher, in either deployment:

| Deployment | Owner | How |
|---|---|---|
| Hardware (`use_sim_time:=false`) | `realsense2_camera` driver | Reads factory-calibrated extrinsics off the camera firmware and publishes them at runtime |
| Simulation (`use_sim_time:=true`) | `robot_state_publisher` | The camera macro passes `use_nominal_extrinsics="$(arg is_sim)"`, so the selected model's URDF emits the nominal chain as fixed joints |

The description must never publish a competing copy on hardware, which is why the Intel macros default `use_nominal_extrinsics` to false and why the sim value is derived from `is_sim` rather than from a parameter of its own. `test_camera_description.py` asserts both halves, plus that no launch file publishes a rival `camera_0_color_optical_frame` transform.

The same `is_sim` flag also gates the Gazebo render sensor, which hangs off `camera_0_color_frame` so the rendered viewpoint *is* the pose of the `camera_0_color_optical_frame` its images and cloud are labelled with.

### Historical: the D435 static publisher (removed 2026-08-31)

Before this, the sim chain was supplied by a `static_transform_publisher` included by both launch files behind `IfCondition(use_sim_time)`:

```
camera_0_link → camera_0_color_optical_frame
  xyz = 0  0.015  0          (colour lens offset copied from d435.urdf.xacro)
  rpy = -π/2  0  -π/2       (standard ROS optical frame rotation)
```

Two defects, both fixed by moving to the model's own frames:

1. The offset was a hand-copied D435 constant living in a second file, so it did not follow `device_type`. The robot is a D455, whose nominal depth-to-colour offset is `-0.059`, not `+0.015`.
2. The render sensor sat on `camera_0_link` while its products were labelled `camera_0_color_optical_frame`, so the rendered viewpoint and the advertised frame disagreed by the colour offset no matter which camera was configured.

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
