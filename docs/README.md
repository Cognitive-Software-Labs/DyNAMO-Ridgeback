# Documentation

Setup and public launch usage remain in the [root README](../README.md).
Agent conventions remain in [AI_CONTEXT.md](../AI_CONTEXT.md).
Troubleshooting and operational root causes live in [ISSUES.md](ISSUES.md).
The maintained engineering gaps are in the [backlog](BACKLOG.md).

## Target localization

- [Target-localization pipeline](target_localization/target_localization_pipeline.md)
- [Detection](target_localization/detection.md)
- [Aligned depth](target_localization/aligned_depth.md)
- [Mask representation](target_localization/mask_representation.md)
- [Segmentation](target_localization/segmentation.md)
- [Projective ranging](target_localization/projective_ranging.md)
- [Euclidean reconstruction](target_localization/euclidean_reconstruction.md)
- [Polar profiling](target_localization/polar_profiling.md)

## Benchmarking

- [Target-distance benchmarking](benchmarking/target_distance_benchmarking.md)
- [Scenario gallery](benchmarking/benchmark_scenarios_v2_gallery.html)

## Do not try again

These documents preserve rejected, deferred, or unselected approaches and the
evidence required before reconsidering them. They are not supported runtime
implementations or committed backlog work.

- [Segmentation](do_not_try_again/segmentation.md)
- [Foreground isolation](do_not_try_again/foreground_isolation.md)
- [Exact-stamp depth delivery](do_not_try_again/exact_stamp_depth_delivery.md)
- [Monocular depth error](do_not_try_again/monocular_depth_error.md) — the ~9x
  gap against `stereoscopic` is **not a bug**: that baseline is a noise-free
  ground-truth render. Reopen only on the Isaac Sim migration

## Active plans

- [Remote vs physical seat validation](plans/remote_vs_physical_seat_validation.md) — can a benchmark taken over xrdp be quoted as a physical-seat number
- [D455 hardware validation](plans/camera_hardware_validation.md)
- [Model-concurrency evidence](plans/model_concurrency_evidence.md)
- [Occlusion-handling proposal](plans/occlusion_handling.md)

## History and validation

- [Benchmark evolution](history/benchmark_evolution.md)
- [Segmentation experiments](history/segmentation_experiments.md)
- [Estimator evolution](history/estimator_evolution.md)
- [Aligned-depth coverage](history/aligned_depth_coverage.md)
- [Point-cloud provenance evaluation](history/pointcloud_provenance_evaluation.md)
- [Target-localization refactor validation](history/refactor_validation.md)
- [ROI mask migration](history/roi_mask_migration.md)
- [Exact-stamp depth availability](history/exact_stamp_depth_availability.md)
- [Exact-stamp investigation handoff](plans/exact_stamp_depth_availability_handoff.md)

History preserves dated evidence and limitations. Read the corresponding
localization or benchmark reference for current behaviour.
