# Issues and Troubleshooting

## Troubleshooting

| Problem | Check |
|---------|-------|
| Robot doesn't move | `ros2 topic echo /r100_0001/cmd_vel` - if empty, Nav2 may not be active |
| No map in RViz | `ros2 topic hz /r100_0001/map` - if 0, check `slam_toolbox` logs and the scan topic |
| Detection overlay does not appear | Make sure `g1_perception_enabled:=true`, remember the overlay is a separate OpenCV window, and check `/r100_0001/sensors/camera_0/color/image` |
| `explore_lite` not finding frontiers | Verify `track_unknown_space: true` in the global costmap config |
| TF errors | Ensure all nodes use `use_sim_time: true` |
| Gazebo slow to start | Increase the `TimerAction` delays in `ridgeback_exploration.launch.py` |
| Stale processes from previous runs | Run `bash cleanup.sh` before each launch |
| Diagnostics | Run `bash diag.sh /tmp/logfile.log hospital` or `bash diag.sh /tmp/logfile.log warehouse` |

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

### Toggling It Off

If you want to launch with the system-default RMW (no UDP-only override), set:

```bash
FASTRTPS_NO_SHM=false bash start_exploration.sh office
```

The script then leaves `RMW_IMPLEMENTATION`, `FASTRTPS_DEFAULT_PROFILES_FILE`, and `RMW_FASTRTPS_USE_QOS_FROM_XML` untouched. The launch file (`ridgeback_exploration.launch.py`) does not set these on its own, so `ros2 launch` invocations also honor whatever is in your shell env.

### A/B History

On April 12, 2026, the stack was A/B tested in the `office` world with and without the UDP-only profile. Both runs brought up Gazebo, `/clock`, SLAM, and the G1 perception nodes; no SHM-specific FastDDS errors appeared on that machine in either run. The profile was kept as the script default because it had previously fixed sim-bringup failures on a different machine and there was no observed downside to leaving it on.

## SLAM Drift in Featureless Environments (Office World)

**Symptom**: After launching in the office world, the robot appears to jump/move randomly in RViz (map→odom transform drifts) while the robot remains physically stationary in Gazebo.

**Root Cause**: `minimum_travel_distance: 0.0` and `minimum_travel_heading: 0.0` in `slam_toolbox_params.yaml` cause slam_toolbox to process *every* incoming scan even when the robot has not moved. In the warehouse world the shelving provides many distinctive scan features, so small mismatches stay bounded. The office world has large open areas with uniform walls; each scan-to-scan mismatch is small but uncorrected, and the accumulated drift eventually makes the map→odom TF spin the robot around in RViz.

**Fix**: Set `minimum_travel_distance: 0.05` and `minimum_travel_heading: 0.05` in `slam_toolbox_params.yaml`. This gates scan processing to moments when the robot has moved ≥ 5 cm or rotated ≥ 3°, eliminating spurious updates while stationary.

## Namespace Gotchas

If you add new nodes to this project, always:

1. Set `namespace=namespace` on the node
2. Add `remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]`
3. Use `/**/node_name:` as the YAML root key in parameter files so namespaced nodes still match their params
4. Set `use_sim_time: true` in simulation
