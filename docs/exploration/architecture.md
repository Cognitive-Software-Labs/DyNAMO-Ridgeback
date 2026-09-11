# Exploration architecture

The exploration workflow composes simulation or robot sensors, state
estimation, SLAM, Nav2, the in-repository frontier explorer, and diagnostic
overlays. The root [README](../../README.md) owns commands and launch arguments;
this page owns the high-level component boundary.

```text
sensors + filtered odometry
          |
          v
     slam_toolbox ----> map
          |              |
          v              v
        Nav2 <---- global/local costmaps
          |
          v
 frontier_explorer_node
          |
          v
    NavigateToPose
```

Bringup follows actual readiness rather than elapsed time:

1. scan and filtered odometry publishers permit SLAM startup;
2. the SLAM lifecycle service permits configuration and activation;
3. the map publisher permits Nav2 startup;
4. the global costmap publisher permits the frontier explorer to start.

Gate timeouts are bounded safety fallbacks. A timeout reports the missing
topic, service, or transform and then permits launch to continue; it is not a
replacement for the readiness condition.

## Exploration and diagnostics

`frontier_explorer_node` reads the Nav2 global costmap, chooses frontiers, and
sends `NavigateToPose` goals. It is the only supported explorer.

The HUD is an aggregator. Independent producers publish text panels such as
velocity and coverage, and `hud_node` combines them for one RViz overlay.
Coverage compares the live SLAM map with the selected world's packaged
ground-truth map. Map generation and interpretation are documented beside the
[ground-truth maps](../../src/ridgeback_autonomy/sim/ground_truth_maps/README.md).

Target localization is an optional consumer of the same sensor stack. Its
detector, measurement paths, visualization, and benchmark semantics are owned
by the [target-localization pipeline](../target_localization/target_localization_pipeline.md),
not duplicated here.

## Validation

The [exploration benchmark runbook](benchmarking.md) records the
instrumented probe workflow. Current failure signatures and recovery steps live
in [troubleshooting](../troubleshooting.md); dated tuning and incident evidence belongs
in history.
