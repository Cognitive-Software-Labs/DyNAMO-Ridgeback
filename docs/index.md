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
- [Active engineering gaps](BACKLOG.md)
- [Troubleshooting](troubleshooting.md)

## Exploration

- [Exploration architecture](exploration/architecture.md)
- [Exploration benchmark runbook](exploration/benchmarking.md)
- [Ground-truth maps](../src/ridgeback_autonomy/sim/ground_truth_maps/README.md)

## Isaac Sim port

- [Port status and document map](isaac/README.md)
- [Robot, sensor, and generated-USD model](isaac/robot-model.md)
- [RTX lidar pipeline](isaac/lidar-pipeline.md)
- [Active Isaac engineering gaps](BACKLOG.md#isaac-lidar-and-exploration-recertification)
- [Isaac rollback](isaac/rollback.md)
- [Isaac 6.1 migration evidence](history/2026-09-13-isaac-6.1-migration.md)
- [Investigation history](isaac/port-history.md)

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

## Benchmarking

- [Target-distance benchmarking](benchmarking/overview.md) — start here: what
  the benchmark is, the estimators and axes, and what it may not claim
- [Running benchmarks](benchmarking/running_benchmarks.md) — which question,
  which profile, which of the four ways to start it
- [Benchmark semantics](benchmarking/semantics.md) — scenarios, events,
  observations, instances, and miss reasons
- [Benchmark outputs](benchmarking/outputs.md) — run and sweep artifacts,
  provenance, rename rules, and module ownership
- [Replay profiles](benchmarking/profiles.md) — the legacy measurement dataset
  and the four-profile layered replay contract
- [Benchmark configurator](benchmarking/configurator.md) — the local browser UI
  and its run substrate
- [Scenario gallery](benchmarking/benchmark_scenarios_v2_gallery.html)

## Active plans

- [Layered replay profiles](plans/layered_replay_profiles.md) — implementation
  landed; live parity, real-model, storage, and scaling gates remain open
- [Live sweep from the configurator GUI](plans/benchmark_gui_direct_run.md) —
  the three offline profiles run from the page; the `live-system` phase is
  blocked on the `cleanup.sh` catch-all
- [D455 hardware validation](plans/camera_hardware_validation.md)
- [Model-concurrency evidence](plans/model_concurrency_evidence.md)
- [Occlusion characterization](plans/occlusion_handling.md)
- [Remote versus physical-seat validation](plans/remote_vs_physical_seat_validation.md)

Only genuinely open work belongs here. Completed implementation plans are
retired once their durable contract and evidence have canonical homes.

## Rejected or deferred approaches

These pages preserve negative results and the evidence required before a retry.
They are not supported implementations or automatic backlog commitments.

- [Segmentation candidates](do_not_try_again/segmentation.md)
- [Foreground-isolation candidates](do_not_try_again/foreground_isolation.md)
- [Exact-stamp depth-delivery experiments](do_not_try_again/exact_stamp_depth_delivery.md)
- [Monocular-depth error](do_not_try_again/monocular_depth_error.md)

## History and validation

- [Upstream dependency refresh](history/dependency_refresh.md)
- [Operational incidents](history/operational_incidents.md)
- [Benchmark evolution](history/benchmark_evolution.md)
- [Segmentation experiments](history/segmentation_experiments.md)
- [Estimator evolution](history/estimator_evolution.md)
- [Aligned-depth coverage](history/aligned_depth_coverage.md)
- [Point-cloud provenance evaluation](history/pointcloud_provenance_evaluation.md)
- [Target-localization refactor validation](history/refactor_validation.md)
- [ROI mask migration](history/roi_mask_migration.md)
- [Exact-stamp depth availability](history/exact_stamp_depth_availability.md)
- [Projective-parameter sensitivity](history/projective_parameter_sensitivity.md)
- [Chassis attitude and floor crop](history/chassis_attitude_and_floor_crop.md)
- [Offline measurement replay validation](history/offline_measurement_replay_validation.md)
- [Layered replay implementation validation](history/layered_replay_implementation_validation.md)
- [Running benchmarks from the configurator GUI](history/benchmark_gui_direct_run.md)
- [Isaac world seating and map regeneration](history/isaac_world_seating.md)
- [Isaac hull-collider live validation](history/isaac_hull_collider_validation.md)

History preserves dated evidence and limitations. Read the corresponding
technical reference for current behavior.
