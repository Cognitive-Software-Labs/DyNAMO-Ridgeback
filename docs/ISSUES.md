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
| Frontier explorer not finding frontiers | Verify `track_unknown_space: true` in the global costmap config |
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

**"Run cleanup once before" is not optional, and skipping it fails in a way
that does not name its cause.** A preceding single-run
`target_distance_benchmark.launch.py` leaves `gz sim server` and `gz sim gui`
alive after its launch service exits — the launch log even reports the
`ruby ... gz sim` wrapper escalating to `SIGTERM`, but those two children
outlive it. Start a sweep on top of them and there are **two servers bound to
the same world name**, so gz-transport service calls are answered by whichever
one replies first: the target spawns into one world and the pose query is
served by the other. Every trial then fails with

```
single_facing_01_rep1 failed: Target pose
"bench_<stamp>_single_facing_01_rep1_target_0" not present on
/world/target_distance_calibration/pose/info
```

which reads as a spawn or naming fault and is neither. Confirm with
`pgrep -af "gz sim"`: **two** `gz sim server` entries (and two GUIs) means this,
not the pre-existing single-trial `pose/info` segfault that drops one trial per
run. `bash cleanup.sh` clears it; then restart the sweep. The partial sweep
folder is safe to discard — a sweep that failed this way writes no `run.json`
for any configuration.

## A long sweep dies in the Gazebo GUI's render thread

Observed 2026-09-08 on a 15-configuration sweep: at configuration 9, roughly
6.4 hours in, `gz sim` exited with

```
Segmentation fault (Address not mapped to object [(nil)])
  gz::gui::plugins::RenderThread  ->  GzSceneManager::Update
  ->  RenderUtil::Update()  ->  SceneManager::CreateVisual
  ->  LoadGeometry  ->  libgz-rendering-ogre2
```

That is the **GUI** render path (`libMinimalScene.so`, `libGzSceneManager.so`)
building a visual for a freshly spawned model — not the server's sensor
rendering, which is what the measurements depend on.

**The early warning is per-configuration wall time, not RTF.** Configurations
lengthened monotonically for hours first — 0.43, 0.51, 0.55, 0.67, 0.67, 0.80,
0.84, 0.93 h — while the pre-run real-time factor held at 0.97+, host RAM stayed
at ~70 GB free and GPU memory flat. RTF is measured over a short settled window
and says nothing about this, so a steadily growing config time on a fixed trial
count is the signal to act on.

When it dies, trials fail in a recognisable order: first
`Timed out waiting for Gazebo pose of "bench_..."`, then
`Failed to spawn "bench_...": Command timed out after 10.0s`. The supervisor and
every per-config node keep running against a dead simulator, so the run does not
stop on its own — confirm with `pgrep -f "gz sim"` returning nothing.

Recovery: stop the supervisor, `bash cleanup.sh`, and rerun the **same** command.
Configurations with a valid `run.json` are skipped and the sweep continues in the
same directory. Sweeps expected to run past ~5 h are worth splitting.

## Editing a sweep YAML mid-run breaks its own resume

`config/` is installed by **symlink**, so editing
`src/ridgeback_autonomy/config/benchmark_sweep_*.yaml` edits the file a running
sweep is reading. `target_benchmark_sweep` records
`resume_signature.sweep_source_sha256` in `sweep.json` and `_find_resumable_sweep`
demands an exact match, so changing even a comment makes the next invocation
create a **new** sweep directory and orphan every completed configuration
instead of resuming.

Before resuming, compare the installed YAML's SHA-256 against
`resume_signature.sweep_source_sha256` in `<sweep_dir>/sweep.json`; restore the
original bytes if they differ. For the same reason `--only` with a different
configuration set forks a new directory — `selected_configs` must match exactly
— so it cannot be used to batch a resume that should land in the existing folder.

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

## Nav2 and slam_toolbox abort on CycloneDDS's participant-index ceiling

### Symptom

`ridgeback_exploration.launch.py` comes up with Gazebo, the detector and RViz
alive while navigation is simply absent. The first sign is not a death but a
node-creation failure:

```
Failed to find a free participant index for domain 106
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain, error Error
```

`slam_toolbox`, every Nav2 server and `explore_lite` then abort with SIGABRT
(`exit code -6`), and the readiness gates die with them. Measured on a default
configuration: **51 processes started, 15 failed to create a node.**

### Root cause

Each ROS 2 process claims a CycloneDDS participant index, and Cyclone only
probes up to `MaxAutoParticipantIndex` before failing. The default ceiling sits
below what exploration needs. The benchmark entrypoint starts far fewer
processes and stays under it, which is why this survived the switch to
CycloneDDS unnoticed — exploration had not been run in simulation since.

### Fix

`config/cyclonedds.xml` raises the ceiling, and all three entrypoints apply it
through `cyclonedds_actions()`. Each entrypoint sets it for itself because the
sweep supervisor starts the environment and per-config layers as separate
processes; neither inherits from the other.

It defers to an operator: if `CYCLONEDDS_URI` is already set, the launch files
leave it alone rather than discard a configuration someone chose. To override,
export your own before launching.

Raising the ceiling only widens the range of ports probed when claiming an
index. It changes no wire behaviour.

### If it comes back

Check the variable actually reached the nodes — an entrypoint launched by hand
from a shell that exports an unrelated `CYCLONEDDS_URI` will use that instead:

```bash
echo "${CYCLONEDDS_URI:-<unset>}"
```

## Camera rate collapses when X falls back to software rendering

### Check this first

```bash
glxinfo | grep "OpenGL renderer"
```

`llvmpipe` means Mesa is rasterizing on the CPU and the GPU is doing no
graphics at all. Gazebo then renders every camera frame in software, and the
sensor rate collapses as soon as anything else wants CPU. Fix by forcing the
NVIDIA GLX vendor for the simulator:

```bash
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __NV_PRIME_RENDER_OFFLOAD=1
```

Measured on cell D, identical config, only this variable changed:

| GL renderer | camera |
|---|---|
| `llvmpipe (LLVM 20.1.2)` | 3.80 Hz |
| `NVIDIA RTX PRO 6000 Blackwell` | **28.07 Hz** |

The session defaults to `llvmpipe` under XRDP even though the hardware path is
available; it can also change across reboots, so a run that was healthy
yesterday is not evidence that today's is.

### Symptom

The simulated camera publishes at ~4–6 Hz instead of ~28 Hz. Every subscriber
sees the identical reduced count, so it looks like a subscriber or QoS fault and
is not one. Downstream this caps the detector at the camera rate no matter what
`detector_fps` says, which silently invalidates any cadence measurement.

### Root cause

Gazebo rasterizes every camera frame on the CPU, because the X session
resolves GL to `llvmpipe`. The GPU does no graphics at all -- which is why it
reads as nearly idle (25% utilization, all of it CUDA from the perception
models) while `gz sim gui` burns three cores. The camera sensor then holds its
rate only while spare CPU exists, and collapses as soon as the per-config layer
adds the mask node.

Sim time keeps advancing normally, so **RTF does not reveal this** -- it stayed
~0.89 throughout. Only the sensor rate drops.

### Do not re-derive these

Each was tested against the collapse and refuted. The camera rate is identical
at every subscriber in every case, so it is never a subscriber-side fault.

| suspected cause | test | result |
|---|---|---|
| detector dropping frames at its subscription | compare detector `rx` to mask node `rx color` over identical windows | identical counts, always |
| QoS mismatch | both nodes use `qos_profile_sensor_data` | same profile |
| `ffmpeg` screen-grab | rerun with `record_video:=false` | 4.19 Hz, no change |
| the default-off timing probe | rerun with `detector_debug:=false` | 3.80 Hz, no change |
| detector rate / GPU load | rerun at `detector_fps:=4.13`, matching the last healthy run | 3.55 Hz, no change |
| orphaned processes | `ps` after a clean shutdown | none |
| CUDA unavailable | `torch.cuda.is_available()` in `perception_venv` | True, models on GPU |

Turning off the Gazebo GUI alone only reduces a competing rasterizer client;
the camera remains software-rendered. The primary measured fix remains
`tools/gpu-run`, which selects NVIDIA GLX while retaining the GUI. As an
additional server-only option, launch with `headless_rendering:=true` and an
NVIDIA EGL vendor selection when the run does not need Gazebo's GUI. Headless
rendering is default-off and is not a replacement for the measured GLX path.

### Measured, same config and scenario

| run | GL renderer | camera |
|---|---|---|
| 2026-09-03 `r1_d_silhouette_monocular` | hardware (implied) | 27.87 Hz |
| 2026-09-04 `d_silhouette_monocular` | `llvmpipe` | 4.95 Hz |
| 2026-09-05, video off | `llvmpipe` | 4.19 Hz |
| 2026-09-05, video off + probe off | `llvmpipe` | 3.80 Hz |
| 2026-09-05, probe off + forced NVIDIA GLX | **NVIDIA RTX PRO 6000** | **28.07 Hz** |

### How to measure the camera rate

`ros2 topic hz` reports nothing on this stream: it is best-effort and the CLI
subscribes reliable by default, so the subscription never matches. Use a
subscriber with `qos_profile_sensor_data`, or read the mask node's
`depth-match: rx color=` counter across two log lines and divide by the
timestamp delta.

## Historical: the detector throttle overshot its own setpoint (fixed 2026-09-04)

### Problem

`detector_fps` was documented and read as a rate, but the achieved rate sat
consistently under it. The 2026-09-03 model-concurrency sweep measured a
detection interarrival of **242.0 ms warm p50** across all twelve matrix
configurations (spread 0.56 ms) from a `detector_fps: 5.0` setting whose period
is 200 ms — 4.13 Hz delivered against a 5.0 Hz setpoint, 17% short.

### Root cause

`processing_loop` advanced its clock *after* the detection step:

```python
next_allowed_time = self.last_detection_time + self.detector_period
...sleep...
self.run_detection_step(color_msg)
self.last_detection_time = time.monotonic()   # after the work
```

That makes the period a **gap between a step ending and the next beginning**,
so the cycle is `configured_period + inference`, not `configured_period`. The
overshoot is exactly one inference: 242 − 200 = ~42 ms.

### Why it stayed hidden

Nothing measured the detector from inside. Its cost could only be inferred from
the gap seen by a downstream consumer, and that gap conflates inference with the
deliberate throttle wait — so a period longer than the setpoint read as "the
model is slow" rather than "the schedule is wrong". The two are now separate
stages (`gate_wait` versus `detector_step`) under `detector_debug`.

### Fix

Advance the clock before the step, making the period a floor on step *starts*.
The achieved period becomes `max(configured_period, inference)`, which both
hits the setpoint when there is headroom and degrades gracefully without a
hot-loop when there is not. Measured with a 40 ms stub inference:

| configured | achieved period p50 | achieved rate |
|---|---|---|
| 5 Hz (200 ms) | 200.09 ms | 5.00 Hz |
| 10 Hz (100 ms) | 100.05 ms | 9.99 Hz |
| 40 Hz (25 ms) | 40.41 ms | 24.74 Hz (inference-bound, as expected) |

Advancing before the step also preserves the older guarantee it was written for:
a frame that always raises still waits a full period and cannot hot-loop.

### Reading the diagnostic

`detector_debug:=true` logs achieved cadence and per-stage percentiles every
`detector_debug_period_s`. `gate_wait` near zero means the throttle is no longer
the limiter and the detector has become inference-bound; `superseded` counts
camera frames dropped by the latest-wins slot, which is by design whenever the
camera outruns the detector.

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
