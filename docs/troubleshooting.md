# Troubleshooting

This page owns current failure signatures and recovery steps. Dated root-cause
investigations and measurements live in
[operational incidents](history/operational_incidents.md).

## Quick checks

| Symptom | Check or action |
|---|---|
| Robot does not move | Inspect `/r100_0001/cmd_vel`; if it is empty, Nav2 may not be active |
| No map in RViz | Check `/r100_0001/map`, `slam_toolbox`, and the scan topic |
| Target overlay missing | Confirm `target_localization_enabled:=true`, enable RViz's `Perception overlay`, and inspect the color and `debug/target/overlay` topics |
| Overlay is a sliver in the left dock | Drag the pane to the bottom, stretch it, and save the RViz config; pane placement is stored in `QMainWindow State` |
| Estimator ring or HUD column missing | Verify the estimator was selected and the corresponding measurement node/topic exists; `--` means the estimator ran but returned no value |
| All mask rows blank, pointcloud works | Inspect startup logs for an unavailable configured `base_frame` |
| Explorer finds no frontiers | Confirm the global costmap has `track_unknown_space: true` |
| Startup stage stalls | Read the matching `gate_*` process's `unmet:` list; do not add a fixed timer |
| Stale processes | Run `bash cleanup.sh` before a normal launch or once before a sweep |
| Need a log summary | Run `bash tools/diag.sh <console.log> hospital` or `warehouse` |

## Benchmark sweep cleanup

`cleanup.sh` intentionally kills Gazebo, RViz, ROS launches, and package nodes.
Run it once before `target_benchmark_sweep`, never between configurations: the
sweep deliberately keeps Gazebo, RViz, the robot, camera TF, detector, and clock
alive while it restarts only the configuration layer.

Two `gz sim server` processes indicate that a stale single-run environment was
not cleaned. Gazebo transport may then spawn a target in one server and query
its pose from the other, producing a misleading `Target pose ... not present`
failure. Clean once and restart the sweep. A valid `run.json` lets the supervisor
resume completed configurations.

## Long sweep loses Gazebo

A long GUI-backed sweep can fail in Gazebo's render thread while creating a
new visual. The sequence is usually:

- configuration wall times rise steadily although short-window RTF remains healthy;
- pose lookup times out;
- later spawns time out;
- `pgrep -f "gz sim"` finds no simulator while the supervisor keeps running.

Stop the supervisor, run `bash cleanup.sh`, then rerun the identical sweep
command so valid configuration outputs resume. Split runs expected to exceed
roughly five hours. RTF alone does not detect this GUI-resource degradation.

## Sweep YAML changed during a run

Installed configuration files are symlinked to the source tree. Editing a
`benchmark_sweep_*.yaml` file therefore changes the source a running sweep will
hash. Resume requires the same source SHA-256 and selected configuration set;
even a comment change or a different `--only` selection creates a new sweep.
Compare the source hash with `resume_signature.sweep_source_sha256` in
`sweep.json` and restore the original bytes before resuming.

## Event-driven startup

Bringup uses `ros2 run ridgeback_autonomy launch_wait` and launch events, not
fixed delays. Exploration waits in this order:

| Gate | Requires | Starts |
|---|---|---|
| `gate_slam` | scan and filtered odometry publishers | SLAM |
| `gate_slam_configure` | `slam_toolbox/change_state` service | SLAM configure/activate |
| `gate_nav2` | map publisher | Nav2 |
| `gate_explorer` | global costmap publisher | frontier explorer |

Timeouts are bounded safety fallbacks: they report missing prerequisites and
permit launch to continue. If a stage stalls, fix the named input or raise the
timeout for a genuinely slow host. `manual_mapping.launch.py` uses the same
pattern for SLAM and controller activation.

## Platform command timestamp warnings

Occasional `platform_velocity_controller` warnings about a command older than
the generated `reference_timeout: 0.1` can be simulation jitter. Do not edit the
generated Clearpath controller configuration first. Check Nav2 retarget churn,
collision-monitor approach flicker, the command chain (`cmd_vel`,
`cmd_vel_smoothed`, `platform/cmd_vel`), simulation rate, and controller timing.
The generated file is under the active Clearpath `setup_path`, normally
`~/clearpath/platform/config/control.yaml`.

## CycloneDDS participant ceiling

If Gazebo, RViz, and detection start but SLAM/Nav2 disappear with
`Failed to find a free participant index`, verify `CYCLONEDDS_URI`. The packaged
`config/cyclonedds.xml` raises `MaxAutoParticipantIndex`, and public entrypoints
set it only when the caller has not supplied a value. An unrelated environment
override can silently restore the too-small ceiling:

```bash
echo "${CYCLONEDDS_URI:-<unset>}"
```

## Camera rate collapses under software rendering

If the simulated camera falls from about 28 Hz to 4–6 Hz, inspect the renderer:

```bash
glxinfo | grep "OpenGL renderer"
```

`llvmpipe` means Gazebo is rasterizing on the CPU. Launch through
`tools/gpu-run` for the measured NVIDIA GLX path. For a server-only session,
select the NVIDIA EGL vendor and pass `headless_rendering:=true`. Disabling the
Gazebo GUI alone does not turn software sensor rendering into GPU rendering,
and healthy RTF does not prove the camera rate is healthy.

The camera stream is best-effort, so a default reliable `ros2 topic hz`
subscription may not match. Use a sensor-data QoS subscriber or the mask node's
depth-match receive counters.

## SLAM TF lag aborts valid Nav2 plans

If MPPI reports `Unable to transform goal pose into costmap frame` with a small
future-extrapolation gap, keep these settings:

- `restamp_tf: true`, so `slam_toolbox` owns a current `map -> odom` stamp;
- `scan_queue_size: 1`, so asynchronous mapping keeps only the latest scan.

Increasing MPPI's transform wait does not fix stale producer timestamps and can
cause missed controller cycles. The measured investigation is preserved in
[operational incidents](history/operational_incidents.md#slam-tf-lag-2026-09-10).

## SLAM drift while stationary

Featureless worlds can drift if `slam_toolbox` processes every stationary scan.
Keep `minimum_travel_distance: 0.05` and `minimum_travel_heading: 0.05` in
`slam_toolbox_params.yaml`; these gates require about 5 cm or 3 degrees of
motion before another scan update.

## Camera optical-frame ownership

Mask estimators require `camera_0_color_optical_frame` to the configured base
frame. Exactly one component publishes the camera chain:

| Deployment | Owner |
|---|---|
| Hardware | `realsense2_camera`, using calibrated device extrinsics |
| Simulation | `robot_state_publisher`, using the selected camera model's fixed joints |

The description must not publish a competing nominal chain on hardware. In
simulation the render sensor hangs from the model's color frame, so image pose
and advertised optical frame agree.

## Namespace and base-frame checks

New namespaced nodes should set their namespace, remap absolute TF topics to
relative `tf`/`tf_static`, use a wildcard node key in parameter YAML, and enable
simulation time. A topic namespace still does not determine TF frame IDs.

Every mask-node launch caller passes `base_frame` explicitly. When that frame
does not exist, the mask node can skip the entire batch with
`TF_MISS_EXTRINSIC`, making all mask rows look like ordinary misses. Inspect the
actual graph instead of deriving the frame from the namespace:

```bash
ros2 run tf2_tools view_frames
ros2 topic echo /r100_0001/tf --once
```

The shared TF helper tries the configured frame and its last path segment. It
checks both without waiting before applying the bounded lookup timeout, so an
already-buffered fallback does not cost half a second per batch. If mask
coverage is low, distinguish `UNSET` (no mask result produced) from
`NO_DEPTH_FRAME` (the batch ran but exact-stamp depth was absent) in `run.json`.

## Required local patches

The stack currently requires the patches listed in the root README. In
particular, `patches/slam_toolbox_tf_namespace.patch` makes the TF listener use
the namespaced node interface. Apply each patch only after `git apply --check`;
the upstream dependency pins and patches are one compatibility unit.
