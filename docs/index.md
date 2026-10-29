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
  - [Maintained dependency patches](project/dependency_patches.md) — defect,
    change, build and loading proof, and rollback for each local patch
- [Project engineering gaps](BACKLOG.md)
- [Troubleshooting](troubleshooting.md)

## Robot

- [Geometry and mounting](robot/geometry.md) — configured mounts, model dimensions, and provenance
- [Collision model and navigation footprint](robot/collision_model.md) — physical, simulator, and navigation envelopes

## Physical deployment

- [Driving readiness](physical/driving_readiness.md) — router for the driving
  stack: installed state and links to its mechanism pages
  - [Driving command path](physical/driving_command_path.md) — topology, mux
    priorities, stamps, frames and speed scales from Nav2 and teleop to the base
  - [E-stop and stop semantics](physical/estop_stop_semantics.md) — QoS relay,
    deadman release, source loss and disconnection
  - [Commissioning guard](physical/commissioning_guard.md) — speed modes,
    admission and watchdog, test-permit interlock, install and rollback
  - [Controller input](physical/controller_input.md) — joystick chain, neutral
    input, autoconnect helper and connected-but-no-input recovery
  - [Driving check tools](physical/driving_check_tools.md) — what each stationary
    check, observer and recorder measures
  - [Driving qualification procedure](physical/driving_qualification_procedure.md)
    — supervised procedure, verdicts and the stopping-distance method
- [Wheel feedback and diagnostics](physical/drive_feedback_diagnostics.md) —
  installed feedback stack, fault flags and the three-rate model
  - [Per-wheel field age](physical/wheel_feedback_field_age.md) — field
    eligibility, age and queue bounds, installation and acceptance
  - [Feedback consumer](physical/wheel_feedback_consumer.md) — missing-feedback
    policy, lifecycle and recovery
  - [Motor-link fault injection](physical/motor_link_fault_injection.md) — D6
    feedback and D7 command-link fault paths and verdicts
- Operator runbooks (commands, preconditions, rollback):
  - [Driving operator runbook](physical/driving_operator_runbook.md) — audits,
    guard install and rollback, observer and pulse trials, controller checks
  - [Puma feedback runbook](physical/puma_feedback_runbook.md) — repair driver,
    feedback plugin and field-age driver install, rollback and checks
  - [Stationary repairs runbook](physical/stationary_repairs_runbook.md) —
    desktop and controller setup repair, auxiliary repairs, health batch, clock
- [Ridgeback host setup and SSD recovery](physical/ridgeback_recovery.md) —
  reconstruction order, deployed scripts/configs, network and service ownership,
  credentials recreated locally, and unverified recovery steps
- [Recovery without the vendor dashboard](physical/ridgeback_public_recovery.md) —
  input verification, pinned public source, inert host candidates and fresh-host build
- [Intel–Thor DDS transport](physical/intel_thor_transport.md) — CycloneDDS on
  both hosts, Ethernet-only configurations, the Intel service switch, and the
  pre-deploy link check

## Exploration

- [Exploration architecture](exploration/architecture.md)
  - [Exploration launch arguments](exploration/launch_arguments.md) — worlds and
    every `ridgeback_exploration.launch.py` argument
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
- [Benchmark launch arguments](target_distance_benchmarking/launch_arguments.md)
  — launch layers, backends and every benchmark argument
- [Benchmark sweeps](target_distance_benchmarking/sweeps.md) — sweep mechanism,
  file format, resume and the shipped sweeps
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

- [BENCHMARKING — Execution-environment qualification](plans/BENCHMARKING_environment_qualification.md) — prerequisite for tuning and benchmark claims
- [BENCHMARKING — Combined-model throughput](plans/BENCHMARKING_model_throughput.md) — run the existing sweep and determine whether concurrency is needed
- [BENCHMARKING — Stationary hardware scene recording](plans/BENCHMARKING_hardware_recording.md) — raw sensor bags and measured G1 positions for later estimator comparison; recording not started
- [BENCHMARKING — Shutdown lifecycle and crash qualification](plans/BENCHMARKING_shutdown_lifecycle.md) — remaining shutdown qualification and dependency investigations
- [BENCHMARKING — Live configurator sweeps](plans/BENCHMARKING_live_configurator_sweeps.md) — blocked on live process ownership and cleanup; follows environment qualification
- [Distributed localization contract](target_localization/distributed_deployment.md) — packages, host roles, health, and mission pause/resume
- [PHYSICAL — Intel–Thor deployment and transport qualification](plans/PHYSICAL_intel_thor_deployment.md) — robot-side installation, GPU offload, Ethernet measurements, and failure handling
  - [Intel–Thor sensor latency tools](physical/intel_thor_sensor_latency.md) — report components, clock bounds and interleaved qualification
- [PHYSICAL — Robot measurements and stationary sensor validation](plans/PHYSICAL_robot_measurements_and_validation.md) — quick mount readings/photos, D455 and LiDAR checks, stationary alignment, and G1/person tests
  - [PHYSICAL — Camera and alignment robot session](plans/PHYSICAL_camera_alignment_session.md) — ordered blocks, commands and pre-declared limits for the alignment overlay, off-centre person checks, and estimator modes
- [PHYSICAL — Real-hardware LiDAR validation](plans/PHYSICAL_lidar_validation.md) — front/rear range and coverage, merged geometry, loaded continuity, recovery and conditional motion tests
- [PHYSICAL — Remaining qualification and time estimates](plans/PHYSICAL_remaining_qualification.md) — canonical status of the S, P and D qualification items, remaining driving trials and estimates

Dated handoff snapshots live in [plans/handoffs/](plans/handoffs/README.md).

Only genuinely open work belongs here. Completed implementation plans are
retired once their durable contract and evidence have canonical homes.

## Rejected or deferred approaches

These pages preserve negative results, unselected proposals, and the evidence
required before an approach is selected or retried.
They are not supported implementations or automatic backlog commitments.

- [Segmentation candidates](do_not_try_again/segmentation.md)
- [Foreground-isolation candidates](do_not_try_again/foreground_isolation.md)
- [Occlusion-recovery candidates](do_not_try_again/occlusion_recovery.md) — deferred until baseline characterization
- [Exact-stamp depth-delivery experiments](do_not_try_again/exact_stamp_depth_delivery.md)
- [Monocular-depth error](do_not_try_again/monocular_depth_error.md)
- [Intel–Thor camera transport candidates](do_not_try_again/intel_thor_camera_transport.md)

## Archive

[Engineering archive](../archive/engineering/README.md) — dated experiments,
decisions, and validation limits. Current behavior is owned by the references
above; historical records require deliberate consultation.
