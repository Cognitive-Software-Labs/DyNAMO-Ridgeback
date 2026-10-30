# Engineering archive

Dated experiments and decisions. Select a record for a specific historical
question; its results describe only its recorded configuration. The headers
separate known tested revisions from partial or unknown provenance. These files
are excluded from ordinary repository search and the generated knowledge graph.

## Localization and perception

- [Intel–Thor perception frame timing and depth-one deployment](2026-10-22-perception-frame-timing.md) — warm detector cost, matched-frame delivery, population limits, and queue-change verification.
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

- [Remote versus physical-seat validation](2026-09-07-seat-validation.md) — identical error on both displays; accepted as the execution-environment qualification.
- [Scene recertification for the current camera](2026-10-29-scene-recertification.md) — scene_audit method, legacy cross-check, relabel/re-place decisions and asset seating.
- [Gazebo and Isaac backend qualification](2026-10-26-benchmark-gazebo-isaac-qualification.md) — the first and accepted matched full runs, cross-simulator detector and ranking differences, reproducibility of each simulator, interrupted-stop cycles and installed-launch smokes.
- [Benchmark shutdown ordering and RViz node ownership](2026-10-23-benchmark-shutdown-ownership.md) — client-before-provider rollback, retained view-controller services, native DDS unload evidence, and distinct qualification populations.
- [Benchmark population and scoring transitions](benchmark_evolution.md)
- [Offline measurement replay parity and scaling](offline_measurement_replay_validation.md)
- [Preliminary layered-replay storage experiment](layered_replay_implementation_validation.md)
- [GUI execution and cancellation decisions](benchmark_gui_direct_run.md)

## Simulation and operations

- [Gazebo Sensors shutdown races](2026-10-18-gazebo-sensors-shutdown.md) — waiter wakeup, observed same-instance concurrent joins, and separate diagnostic/qualification populations.
- [ros_gz bridge ownership cycles and image_bridge teardown](2026-10-22-ros-gz-bridge-shutdown.md) — two bridge-node ownership cycles, the image_bridge in-flight callback race, and stock/partial/final crash populations.
- [RViz Image display teardown abort](2026-10-22-rviz-image-display-teardown.md) — stale message-filter connection after unsubscribe, stock/patched signal populations, and the separate process-exit SIGSEGV still open.
- [UST-10LX Gazebo LiDAR parity and Isaac noise](2026-10-25-gazebo-ust10lx-lidar.md) — the Clearpath hardcode and end-ray off-by-one, Isaac's range-dependent Gaussian noise, the constant-stddev choice, and both simulators' determinism.

- [r100_0160 front LiDAR stationary field screening](2026-10-23-r100-0160-front-lidar-field-screen.md) — 0.5–2 m target screens, material detection, front-to-merged contribution, five-minute continuity, and explicit accuracy/reliability limits.
- [Intel DDS local-reader screening](2026-10-05-intel-dds-screening.md) — scan interruption after a local color reader under the original profile, brief discovery-only multicast comparison, and verified restoration.
- [Static collision-envelope audit](2026-09-18-collision-envelope-audit.md) — description/USD/Nav2 comparison with top-down and side views.
- [Navigation footprint and simulator-contact qualification](2026-09-22-navigation-footprint-qualification.md) — effective Nav2 containment plus three-boot Gazebo and Isaac wall-contact matrices.
- [HCC arena doorway runs and the mesh-shaped robot outline](2026-10-30-hcc-arena-doorway-runs.md) — four Gazebo runs through 1.05 m doorways, collision-monitor and Smac Lattice failure modes, and the outline the mesh shape replaced.

- [Intel–Thor DDS transport diagnosis](2026-09-18-intel-thor-dds-transport.md) — Wi-Fi duplication, receive-buffer drops, and Ethernet-only profile measurements.
- [Intel–Thor middleware benchmark](2026-09-18-intel-thor-rmw-benchmark.md) — Fast DDS vs CycloneDDS vs mixed across the link, and the reader-side interface rule.
- [Intel–Thor transport handoff](2026-09-18-intel-thor-transport-handoff.md) — agent 2's state snapshot: CycloneDDS switch, LiDAR lockouts under the services configuration, manual rollback.
- [D455 camera runs handoff](2026-09-18-d455-camera-runs-handoff.md) — agent 1's state snapshot: Fast DDS camera passes, CycloneDDS failures.

- [Physical mecanum drive investigation](2026-09-18-mecanum-drive-investigation.md)
- [r100_0160 Intel clock gate, sources and boot-step qualification](2026-10-13-r100-0160-intel-clock-qualification.md) — first reboot check, source expansion and poll bound, the October 14 startup delay and the S6 boots.
- [r100_0160 commissioning guard installation and first low-speed trials](2026-10-06-driving-guard-and-deadman-trials.md) — guard install, capped direction pulses and deadman-release timing.
- [Puma diagnostic build, first rate capture and startup-transient aborts](2026-10-06-puma-build-and-startup-transients.md) — repair build, first CAN rate capture and the mixed-wheel startup aborts.
- [r100_0160 e-stop QoS repair and stationary disconnect trials](2026-10-09-estop-qos-and-disconnect-trials.md) — incompatible e-stop QoS, the reliable relay and controller power-off trials.
- [r100_0160 test-permit interlock, bounded-permit and moving-disconnect trials](2026-10-12-interlock-and-moving-disconnect-trials.md) — permit timeline, interlock installation and the interlocked moving disconnect.
- [r100_0160 controller input loss, recovery and reconnect cycles](2026-10-12-controller-input-loss-and-reconnect-cycles.md) — connected-but-no-input failures, manual recovery and the autoconnect helper.
- [Puma repair driver, CAN rates and offline freshness models](2026-10-12-puma-repair-rates-and-freshness-replays.md) — installed repair rates, superseded 16.5 Hz/200 ms models and replays.
- [Puma feedback-consumer repair, installation and stationary interruptions](2026-10-13-consumer-plugin-and-feedback-outage.md) — consumer candidates, plugin install, feedback and CAN-ingress interruptions.
- [r100_0160 bounded direction trials and D2 deadman and command-loss runs](2026-10-14-direction-and-deadman-trials.md) — D1 six-direction floor trials and the first D2 runs.
- [r100_0160 D2 physical-stop capture attempts](2026-10-16-physical-stop-captures.md) — physical e-stop capture attempts and their deviations.
- [Single-bound field-age candidate and stationary field captures](2026-10-16-field-age-captures.md) — the superseded 200 ms bound and stationary field captures.
- [r100_0160 original-manual guard profile and D4 handover trials](2026-10-21-original-manual-and-d4-trials.md) — original-manual install and the live handover pass.
- [Field-age evidence from short moving captures, replays and queue-timing builds](2026-10-21-d5-motion-captures-and-queue.md) — moving captures, replays and the queue-timing layer.
- [Split field-age bounds, field-driver installation and stationary acceptance](2026-10-22-d5-acceptance.md) — 150/500 ms bounds, install and the revised 75 ms queue criterion.
- [r100_0160 D-lane baseline, D2 preparation and the reconnected deadman run](2026-10-22-d2-preparation-and-run.md) — baseline snapshot, finalize workflow and the deadman run.
- [r100_0160 field readings and LiDAR box checks](2026-09-18-r100-0160-field-measurements.md) — tape mount readings, LiDAR range/side checks, and the rear-offset finding.
- [r100_0160 Thor perception environment and first live localization](2026-10-12-r100-0160-thor-localization-smoke.md) — Thor GPU environment, model timings at 120W and MAXN, the person-threshold and static-TF findings, and the stationary distance smoke.
- [r100_0160 Intel RViz view of Thor localization](2026-10-13-r100-0160-intel-rviz-observer.md) — SLAM map, scans and Thor's results drawn on Intel in RViz; no image crosses with the box gate; overlay draws on the newest colour frame; CLI probe pitfalls.
- [r100_0160 D455 factory calibration against the nominal simulator profiles](2026-10-13-r100-0160-d455-calibration-vs-nominal.md) — live VGA and recorded HD intrinsics against the nominal profiles, factory extrinsics, and the cost of the pipeline ignoring distortion; nominal profiles kept.
- [r100_0160 Nav2 command chain observed live](2026-10-13-r100-0160-nav2-command-chain.md) — first live Nav2-to-controller trace: TwistStamped throughout, mux remap in force; controller_server reads an unpublished `odom` topic.
- [urg_node reconnect lockout: leaked socket on a failed open](2026-10-13-urg-node-reconnect-leak.md) — source-level cause of the Hokuyo lockout, offline single-client reproduction, and the one-line close patch; not deployed.
- [r100_0160 IGMP querier discovery stall](2026-10-13-r100-0160-igmp-querier-discovery-stall.md) — the 313–381 s discovery outage, causes ruled out, and the 0.2 s result with the querier on.
- [r100_0160 remote desktop boot and recovery](2026-10-06-r100-0160-remote-desktop-boot-and-recovery.md) — the October 6 cold-boot failures, the six-file repair, the boot inspection, the update incident and the S7 recovery check.
- [r100_0160 front LiDAR session reset (S3)](2026-10-14-r100-0160-front-lidar-session-reset.md) — same-process reconnect after 2.25 s with the patched driver.
- [r100_0160 local camera-reader screens](2026-10-13-r100-0160-local-camera-reader-screens.md) — contract screen, port comparisons, S1 quiet repeat, S2 VGA matrix, S4 soak and the `check_link --camera-reader` pass.
- [r100_0160 Thor image copies and HD trials](2026-10-16-r100-0160-thor-image-copies-and-hd-trials.md) — the health image-copy reduction, the HD trial and diagnosis, and the decision to keep Thor at VGA.
- [r100_0160 Intel–Thor camera transport candidates](2026-10-06-r100-0160-thor-transport-candidates.md) — raw baseline against the larger send buffer (O1) and compression (O4), with the decoder findings.
- [r100_0160 D455 camera service qualification](2026-10-06-r100-0160-camera-service.md) — contract checks at both profiles, after a cold boot and after a rollback and reinstall, with the platform-restart and serial findings.
- [r100_0160 rear LiDAR side-target recheck](2026-09-24-r100-0160-rear-lidar-recheck.md) — independent placements after the correction, with a before/after corner overlay.

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
