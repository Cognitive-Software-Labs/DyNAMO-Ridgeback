# Exploration launch arguments

`ridgeback_exploration.launch.py` selects one I/O backend and starts the shared
SLAM, Nav2, frontier and target-localization stack. This page lists its worlds
and arguments. The root [README](../../README.md) owns example commands, and
[the architecture](architecture.md#bringup-sequence) owns the bringup sequence
these arguments feed.

The generated list is the final authority. Print it from a sourced workspace:

```bash
ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py --show-args
```

A default written *sim / hardware* depends on `backend`; Gazebo and Isaac share
the simulation value.

## Worlds

| World | Backend | Source | Notes |
|---|---|---|---|
| `initial_test_world` | Gazebo and Isaac | Repository SDF and converted USD | Default shared multi-room test geometry |
| `depot` | Gazebo | Clearpath `warehouse.sdf`, translated by the adapter | Warehouse/depot geometry |
| `coworking_space` | Gazebo | Clearpath `office.sdf`, translated by the adapter | Multi-room workplace geometry |
| `warehouse` | Isaac | NVIDIA Simple Warehouse | Cropped warehouse environment |
| `warehouse_full` | Isaac | NVIDIA Simple Warehouse | Full warehouse environment |
| `office` | Isaac | NVIDIA Office | Office environment |
| `hospital` | Isaac | NVIDIA Hospital | Hospital environment |

The Gazebo adapter maps `depot` and `coworking_space` to Clearpath's internal
filenames. Direct Gazebo requests for `warehouse` or `office` are rejected, so
distinct geometry never reuses an Isaac world name. Stock Isaac worlds load
from NVIDIA's asset root; `initial_test_world` is repository-local.

## Backend and robot identity

| Argument | Default | Meaning |
|---|---|---|
| `world` | `initial_test_world` | World to load; see [worlds](#worlds) |
| `backend` | value of `sim` (`gz`) | I/O provider: `gz`, `isaac`, or `hardware` |
| `sim` | `gz` | Deprecated compatibility alias for `backend`; accepts `gz` or `isaac` |
| `namespace` | empty | Empty selects `r100_0001` in simulation and reads `<setup_path>/robot.yaml` on hardware; an explicit value overrides either |
| `use_sim_time` | `true` / `false` | Simulation clock in simulation, wall time on hardware |
| `setup_path` | `~/clearpath/` / `/etc/clearpath/` | Directory holding `robot.yaml` and the generated Clearpath files |
| `start_hardware_platform` | `false` | Hardware only: `false` attaches to existing Clearpath services; `true` explicitly includes platform bringup |
| `autonomous_motion_enabled` | `true` / `false` | Allow frontier goals to command motion; hardware needs an explicit operator opt-in ([hardware safety boundary](architecture.md#hardware-safety-boundary)) |
| `sim_ready_timeout` | `45`; `300` on Isaac | Seconds the first readiness gate waits for scan and filtered odometry; the local camera gate uses the same bound |

## Sensor inputs

| Argument | Default | Meaning |
|---|---|---|
| `color_topic` | `sensors/camera_0/color/image` | Shared camera contract's colour image; the hardware camera service publishes the same name |
| `camera_info_topic` | `sensors/camera_0/color/camera_info` | Compatibility override for colour-grid camera intrinsics |
| `depth_topic` | `sensors/camera_0/depth/image` | Colour-aligned depth; the same name on every backend |
| `pointcloud_topic` | `sensors/camera_0/points` / empty | Simulation organized points; empty on hardware until its layout is validated |
| `scan_topic` | `sensors/lidar2d_0/scan` | LaserScan for polar profiling and localization health |
| `base_frame` | `<namespace>/robot/base_link` / empty | Vehicle frame for measurements and rings. Hardware has no default: pass the observed TF frame whenever target localization is enabled, or the launch fails |

The [camera stack](../target_localization/camera_stack.md#ros-topics) owns what
each backend publishes on these names.

## Target localization

| Argument | Default | Meaning |
|---|---|---|
| `target_localization_enabled` | `true` | Launch detection, the selected measurement rows, rings, distance HUD and camera overlay; local mode needs the [perception environment](../target_localization/distributed_deployment.md#workstation-inference-environment) |
| `localization_mode` | `local` | `local` runs the detector and selected workers on this host; `external` starts only result consumers for a detector elsewhere and rejects local detector overrides |
| `localization_required` | `true` when external, otherwise `false` | Start the mission paused and pause on health loss ([mission pause](../target_localization/distributed_deployment.md#mission-pause-and-cancellation)) |
| `estimators` | `all` / `projective_ranging,euclidean_reconstruction` | Comma-separated subset of `pointcloud`, `projective_ranging`, `euclidean_reconstruction`, `polar_profiling`; hardware selects the aligned-depth pair |
| `estimate_viz` | `true` | Publish the estimator rings and the wide distance HUD |
| `detector` | `owlv2` | Startup-only implementation selector; only `owlv2` is supported |
| `detection_model` | `google/owlv2-base-patch16-ensemble` | Checkpoint identifier for the selected detector |
| `target_labels` | `humanoid robot` | Comma-separated detection queries |
| `detection_threshold` | `0.55` | Confidence floor in `(0, 1]` |
| `detector_fps` | `10.0` | Upper bound on detection step starts; every selected measurement row inherits this cadence ([cadence defaults](../target_localization/detection.md#detector-cadence-defaults)) |
| `detector_debug` | `false` | Log detector cadence, superseded frames, and bounded stage timings |
| `depth_source` | `stereoscopic` | Aligned depth source for the mask depth rows: `stereoscopic` or `monocular` (Depth-Anything V2; its checkpoint must be cached or downloads enabled) |
| `mask_gate` | `box` | Mask front-end: `box` (no segmentation model) or `silhouette` (SlimSAM) |

The [distributed contract](../target_localization/distributed_deployment.md#launch-roles-and-configuration)
owns what `local` and `external` start and how detector settings stay
startup-only.

## Localization health budgets

These six arguments set the localization-health and mission-supervision budgets,
in seconds. Their meaning is owned by the
[health settings table](../target_localization/distributed_deployment.md#topics-types-and-acquisition-semantics).

| Argument | Default |
|---|---|
| `startup_timeout` | `120.0` |
| `input_timeout` | `3.0` |
| `progress_timeout` | `15.0` |
| `source_age` | `15.0` |
| `health_timeout` | `3.0` |
| `cancellation_timeout` | `5.0` |

The frontier's watchdog receives `progress_timeout` as its
`localization_progress_timeout`.

## Display and navigation

| Argument | Default | Meaning |
|---|---|---|
| `exploration_rviz` | `true` | Launch the exploration RViz config; hardware gets a derived layout ([Intel RViz view](../target_localization/distributed_deployment.md#intel-rviz-view)) |
| `mppi_visualize` | `false` | Publish MPPI trajectory visualization topics; RViz's namespaced `MPPI Optimal` and `MPPI Samples` displays already subscribe to the relative `optimal_trajectory` and `trajectories` topics |
| `coverage_overlay_enabled` | `true` / `false` | Live coverage HUD panel; hardware has no packaged truth map |
| `slam_source` | `front_only` | Identical default on every backend. `merged` makes `slam_toolbox` read the SLAM-only front+rear merge; Nav2 and the collision monitor keep the raw scans either way |

## Simulator settings

Each simulator ignores the other's settings, and hardware ignores both.

| Argument | Default | Meaning |
|---|---|---|
| `gz_gui` | `true` | Gazebo: launch the Gazebo GUI window |
| `headless_rendering` | `false` | Gazebo: optional server-only EGL sensor rendering for SSH or non-seat sessions; the `tools/gpu-run` NVIDIA GLX workflow remains the default GUI-capable path ([troubleshooting](../troubleshooting.md#camera-rate-collapses-under-software-rendering)) |
| `camera_profile` | `640x480` | Shared nominal D455 render profile for Gazebo and Isaac; `1280x720` is available and requires a simulator restart. Hardware uses its externally managed profile |
| `depth_fidelity` | `ideal` | Isaac depth path: clean colour-render depth (`ideal`) or nominal D455 stereo artifacts plus explicit colour alignment (`d455`) |
| `headless` | `true` | Isaac: run without the viewport window |
| `livestream` | `false` | Isaac: WebRTC livestream |
| `rtf` | `1.0` | Isaac real-time factor; `0` is unthrottled |
| `sim_mode` | `realtime` | Isaac timing: `realtime` or fixed-step `deterministic`; use `deterministic` for comparisons |
| `odom_noise` | `1.0` | Isaac odometry-drift scale; `0` gives exact debug odometry |
| `noise_seed` | `0` | Isaac repeatability: odometry uses seed N and IMU noise N+1; seed 0 preserves the original streams |
| `camera` | `true` | Attach the Isaac D455 renderer; `false` for lidar-only performance runs |
| `sensor_hz` | `40.0` | Isaac deterministic step and lidar rate; camera cadence stays at its authored 30 Hz |

## Declared but fixed by this launch

`clearpath_rviz` (`false`) and `localization_progress_timeout` (`15.0`) appear
in `--show-args` because included launches declare them. The exploration launch
sets both itself: Clearpath's own RViz stays off, and the frontier receives
`progress_timeout`. Overriding either on the command line has no effect.
