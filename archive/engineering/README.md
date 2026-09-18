# Engineering archive

Dated experiments and decisions. Select a record for a specific historical
question; its results describe only its recorded configuration. The headers
separate known tested revisions from partial or unknown provenance. These files
are excluded from ordinary repository search and the generated knowledge graph.

## Localization and perception

- [Package split simulator runtime qualification](2026-09-18-package-split-runtime.md)

- [Depth producer sampling and monocular scale](aligned_depth_coverage.md)
- [Exact-stamp DDS delivery comparison](exact_stamp_depth_availability.md)
- [Published versus deprojected point geometry](pointcloud_provenance_evaluation.md)
- [Projective parameter sensitivity](projective_parameter_sensitivity.md)
- [Floor-crop attitude measurements](chassis_attitude_and_floor_crop.md)
- [Segmentation candidate experiments](segmentation_experiments.md)
- [Estimator result-comparability transitions](estimator_evolution.md)
- [Refactor coverage and TF investigation](refactor_validation.md)
- [ROI-mask parity, storage and CPU measurements](roi_mask_migration.md)

## Benchmarking and execution

- [Benchmark population and scoring transitions](benchmark_evolution.md)
- [Offline measurement replay parity and scaling](offline_measurement_replay_validation.md)
- [Preliminary layered-replay storage experiment](layered_replay_implementation_validation.md)
- [GUI execution and cancellation decisions](benchmark_gui_direct_run.md)

## Simulation and operations

- [Static collision-envelope audit](2026-09-18-collision-envelope-audit.md) — description/USD/Nav2 comparison with top-down and side views.

- [Physical mecanum drive investigation](2026-09-18-mecanum-drive-investigation.md)

- [Isaac 6.0 first-drive snapshot](2026-07-11-isaac-first-drive.md)
- [Invalidated July SLAM investigation](2026-07-12-isaac-slam-investigation.md)
- [Sensor attachment and chassis investigations](port-history.md)
- [Isaac 6.1 migration controls](2026-09-13-isaac-6.1-migration.md)
- [Lidar and closed-loop SLAM qualification](2026-09-17-isaac-lidar-qualification.md)
- [Gazebo lidar consumer integration](2026-09-18-gazebo-lidar-integration.md)
- [World seating and map regeneration](isaac_world_seating.md)
- [Hull-collider contact qualification](isaac_hull_collider_validation.md)
- [Dependency refresh compatibility](dependency_refresh.md)
- [Operational incident evidence](operational_incidents.md)

## Curation record — September 18, 2026

The archive boundary moved dated material out of current documentation and
removed routine verification logs and repeated implementation descriptions.
All 22 source records retained some unique evidence; the duplicated July port
chronology was consolidated into its investigation. Mechanism explanations and
operational guidance were transferred to their maintained owners. No experiments
were rerun, missing snapshots were not reconstructed, and temporary evidence
availability was not newly certified. Original prose remains in Git history.

A follow-up naming cleanup moved the July 11 first-drive presentation out of
`docs/demos/` into this archive. Its embedded images and HTML were preserved
unchanged; the record scopes its claims and links the old instructions through
an immutable documentation snapshot.
