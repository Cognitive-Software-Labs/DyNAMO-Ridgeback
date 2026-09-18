# Documentation

The root [README](../README.md) owns installation, public launch commands, and
operator workflows. This index routes deeper project, architecture, benchmark,
troubleshooting, planning, and historical documentation.

## Project

- [Project context](project/PROJECT_CONTEXT.md) — short repository orientation and
  pointers to the canonical technical references
- [Documentation ownership](project/documentation.md) — where facts belong,
  precedence, and update rules
- [Project conventions](project/conventions.md) — namespaces, generated state,
  tooling, dependencies, and entrypoint conventions
- [External dependency management](project/dependencies.md) — import, patch,
  refresh, verification, live migration, and rollback procedure
- [Project engineering gaps](BACKLOG.md)
- [Troubleshooting](troubleshooting.md)

## Robot

- [Geometry and mounting](robot/geometry.md) — configured mounts, model dimensions, and provenance
- [Collision model and navigation footprint](robot/collision_model.md) — physical, simulator, and navigation envelopes

## Exploration

- [Exploration architecture](exploration/architecture.md)
- [Exploration evaluation](exploration/evaluation.md)
- [Ground-truth maps](../src/ridgeback_autonomy/sim/ground_truth_maps/README.md)

## Isaac Sim

- [Isaac backend overview](isaac/overview.md)
- [Isaac import, articulation, and collision geometry](isaac/robot-model.md)
- [RTX lidar pipeline](isaac/lidar-pipeline.md)
- [Active Isaac engineering gaps](BACKLOG.md#isaac-exploration-recertification--p5)
- [Isaac rollback](isaac/rollback.md)

## Target localization

- [Camera stack](target_localization/camera_stack.md) — mount and frames,
  Gazebo/Isaac/hardware optics, topics, launch ownership, and consumers
- [Pipeline](target_localization/target_localization_pipeline.md)
- [Detection](target_localization/detection.md)
- [Aligned depth](target_localization/aligned_depth.md)
- [Mask representation](target_localization/mask_representation.md)
- [Segmentation](target_localization/segmentation.md)
- [Projective ranging](target_localization/projective_ranging.md)
- [Euclidean reconstruction](target_localization/euclidean_reconstruction.md)
- [Polar profiling](target_localization/polar_profiling.md)

## Target-distance benchmarking

- [Open benchmarking work](target_distance_benchmarking/BACKLOG.md) — validation gates, scenario recertification, and tooling gaps
- [Target-distance benchmarking](target_distance_benchmarking/overview.md) — start here: what
  the benchmark is, the estimators and axes, and what it may not claim
- [Running benchmarks](target_distance_benchmarking/running_benchmarks.md) — which question,
  which profile, which of the four ways to start it
- [Benchmark semantics](target_distance_benchmarking/semantics.md) — scenarios, events,
  observations, instances, and miss reasons
- [Benchmark outputs](target_distance_benchmarking/outputs.md) — run and sweep artifacts,
  provenance, rename rules, and module ownership
- [Replay profiles](target_distance_benchmarking/profiles.md) — the legacy measurement dataset
  and the four-profile layered replay contract
- [Benchmark configurator](target_distance_benchmarking/configurator.md) — the local browser UI
  and its run substrate
- [Scenario gallery](target_distance_benchmarking/benchmark_scenarios_v2_gallery.html)

## Active plans

- [Live sweep from the configurator GUI](plans/benchmark_gui_direct_run.md) —
  blocked on live process ownership and cleanup; follows environment qualification
- [D455 hardware validation](plans/camera_hardware_validation.md)
- [Combined-model throughput measurement](plans/model_concurrency_evidence.md) — run the existing sweep and determine whether concurrency is needed
- [Occlusion characterization](plans/occlusion_handling.md)
- [Benchmark environment qualification](plans/remote_vs_physical_seat_validation.md) — prerequisite for tuning and benchmark claims

Only genuinely open work belongs here. Completed implementation plans are
retired once their durable contract and evidence have canonical homes.

## Rejected or deferred approaches

These pages preserve negative results and the evidence required before a retry.
They are not supported implementations or automatic backlog commitments.

- [Segmentation candidates](do_not_try_again/segmentation.md)
- [Foreground-isolation candidates](do_not_try_again/foreground_isolation.md)
- [Exact-stamp depth-delivery experiments](do_not_try_again/exact_stamp_depth_delivery.md)
- [Monocular-depth error](do_not_try_again/monocular_depth_error.md)

## Archive

[Engineering archive](../archive/engineering/README.md) — dated experiments,
decisions, and validation limits. Current behavior is owned by the references
above; historical records require deliberate consultation.
