# Project context

DyNAMO-Ridgeback is a ROS 2 Jazzy workspace for autonomous Ridgeback
exploration and target localization across Gazebo, Isaac Sim, and physical
hardware, plus repeatable simulator benchmarking.
This page is intentionally short: it provides orientation and routes detailed
facts to their canonical documents.

## Conventions

@conventions.md

The cross-cutting conventions above are imported rather than linked because
every session needs them, including the graphify rules. They remain owned by
`docs/project/conventions.md`; edit that file, not this one. Every other
document below stays a link so it loads only when a task needs it.

## Public workflows

- `ridgeback_exploration.launch.py` selects `backend:=gz|isaac|hardware`, then
  runs the shared SLAM, Nav2, diagnostics, and optional target localization;
  frontier goals default off on hardware.
- `target_distance_benchmark.launch.py` compares the registered localization
  estimators on declarative simulator scenarios.
- `manual_mapping.launch.py` supports map generation and validation.
- `start_exploration.sh` is the normal exploration quick-start;
  `build_and_start_expl.sh` rebuilds first.

Complete setup and commands live in the [root README](../../README.md).
Lower-level launches in `src/ridgeback_autonomy/launch/includes/` are internal
building blocks unless a technical reference says otherwise.

## Repository map

- `src/ridgeback_autonomy/` — backend-neutral nodes, messages, configuration,
  tests, readiness gates, and public launch workflows
- `src/ridgeback_autonomy_gz/` — Gazebo adapter, worlds, models, and GUI plugin
- `src/ridgeback_autonomy_isaac/` — Isaac adapter, runner, sensor specs, and USD
- `src/ridgeback_autonomy_hardware/` — attach-first Clearpath hardware adapter
- `dependencies/` — split `vcstool` manifests: common source dependencies and
  the optional Gazebo source dependency
- `clearpath/robot.yaml` — canonical robot and sensor declaration
- `patches/` — maintained changes to dependencies imported through the
  manifests under `dependencies/`;
  these are standalone ignored repositories governed by the
  [dependency runbook](dependencies.md)
- `tools/` — diagnostics, benchmark helpers, and development utilities
- `artifacts/` — generated run evidence and logs
- `build/` and `install/` — normal top-level Colcon outputs
- `docs/` — project knowledge, organized by ownership in the
  [documentation guide](documentation.md)

## Architecture pointers

- [Exploration architecture](../exploration/architecture.md)
- [Camera stack](../target_localization/camera_stack.md)
- [Target-localization pipeline](../target_localization/target_localization_pipeline.md)
- [Target-distance benchmarking](../target_distance_benchmarking/overview.md)
- [External dependency management](dependencies.md)
- [Troubleshooting](../troubleshooting.md)
- [Active engineering gaps](../BACKLOG.md)
- [Isaac Sim port](../isaac/overview.md)

The graph report required by the imported graphify conventions identifies
high-connectivity nodes and module communities. It does not replace the
maintained references above.
