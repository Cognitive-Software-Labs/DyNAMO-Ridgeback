# Exploration architecture

The exploration workflow composes one I/O backend with the same state
estimation, SLAM, Nav2, target-localization, and diagnostic application stack.
The root [README](../../README.md) owns commands and launch arguments; this page
owns the high-level component and dependency boundary.

```text
 ridgeback_autonomy_gz     ridgeback_autonomy_isaac     ridgeback_autonomy_hardware
 Gazebo + Clearpath        Isaac runner + USD           attach/start Clearpath
           \                       |                       /
            +---- normalized ROS contract ---------------+
                  scan | odom | TF | camera | /clock*
                                   |
                         ridgeback_autonomy
                 SLAM -> Nav2 -> frontier explorer
                    \-> target localization + HUD

 * `/clock` and `use_sim_time=true` exist only in simulation.
```

## Package and dependency ownership

| Package | Owns | Must not own |
|---|---|---|
| `ridgeback_autonomy` | Common nodes, messages, configuration, readiness gates, and public workflows | Gazebo/Qt build dependencies, Isaac runtime, or a hardware driver |
| `ridgeback_autonomy_gz` | Gazebo launch adapter, worlds, models, and `SpawnG1` plugin | SLAM/Nav2/perception behavior |
| `ridgeback_autonomy_isaac` | Isaac launch adapter, runner, sensor specs, and USD assets | Common autonomy policy |
| `ridgeback_autonomy_hardware` | Hardware-safe Clearpath adapter | Simulation packages or autonomous-motion policy |

`dependencies/core.repos` is the common source closure.
`dependencies/gz.repos` adds only `clearpath_simulator`. Package lookup in the
dispatcher is lazy, so an Isaac or hardware install does not need the Gazebo
adapter merely to load the common launch.

The public selector is `backend:=gz|isaac|hardware`. `sim:=gz|isaac` remains a
deprecated compatibility alias. The adapters normalize deployment differences;
common application defaults remain identical unless a physical constraint
requires otherwise. Current intentional differences are simulation time,
camera topic/profile selection, hardware-safe estimator selection, truth-map
overlays, and motion/platform opt-ins.

Bringup follows actual readiness rather than elapsed time:

1. scan and filtered odometry publishers permit SLAM startup;
2. the SLAM lifecycle service permits configuration and activation;
3. the map publisher permits Nav2 startup;
4. the global costmap publisher permits the frontier explorer to start.

Gate timeouts are bounded safety fallbacks. A timeout reports the missing
topic, service, or transform and then permits launch to continue; it is not a
replacement for the readiness condition.

## Hardware safety boundary

`ridgeback_autonomy_hardware` defaults to attaching to externally managed
Clearpath platform and sensor services. `start_hardware_platform:=true` is the
only path that includes platform bringup. The application also defaults
`autonomous_motion_enabled:=false`, which prevents the frontier explorer from
sending goals while still allowing sensor, SLAM, Nav2, perception, and TF
validation. The quick-start wrapper skips broad simulator cleanup and does not
invent a ROS domain for hardware.

These defaults reduce accidental actuation; they do not certify a deployment.
The live velocity message type, controller configuration, e-stop, footprint,
sensor topics, TF ownership, and camera behavior must pass their physical
validation gates before motion is enabled.

## Exploration and diagnostics

`frontier_explorer_node` reads the Nav2 global costmap, chooses frontiers, and
sends `NavigateToPose` goals. It is the only supported explorer.

The HUD is an aggregator. Independent producers publish text panels such as
velocity and coverage, and `hud_node` combines them for one RViz overlay.
In simulation, coverage compares the live SLAM map with the selected world's
packaged ground-truth map. It is disabled by default on hardware. Map
generation and interpretation are documented beside the
[ground-truth maps](../../src/ridgeback_autonomy/sim/ground_truth_maps/README.md).

Target localization is an optional consumer of the same sensor stack. Its
detector, measurement paths, visualization, and benchmark semantics are owned
by the [target-localization pipeline](../target_localization/target_localization_pipeline.md),
not duplicated here.

## Validation

The [exploration evaluation runbook](evaluation.md) records the
instrumented probe workflow. Current failure signatures and recovery steps live
in [troubleshooting](../troubleshooting.md); dated tuning and incident evidence belongs
in the engineering archive.
