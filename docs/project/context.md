# Project context

DyNAMO-Ridgeback is a ROS 2 Jazzy workspace for autonomous Ridgeback
exploration, target localization, and repeatable simulator benchmarking.
This page is intentionally short: it provides orientation and routes detailed
facts to their canonical documents.

## Public workflows

- `ridgeback_exploration.launch.py` runs simulation, SLAM, Nav2, the in-repo
  frontier explorer, diagnostics, and optional target localization.
- `target_distance_benchmark.launch.py` compares the registered localization
  estimators on declarative simulator scenarios.
- `manual_mapping.launch.py` supports map generation and validation.
- `start_exploration.sh` is the normal exploration quick-start;
  `build_and_start_expl.sh` rebuilds first.

Complete setup and commands live in the [root README](../../README.md).
Lower-level launches in `src/ridgeback_autonomy/launch/includes/` are internal
building blocks unless a technical reference says otherwise.

## Repository map

- `src/ridgeback_autonomy/` — the ROS package, launch/configuration files,
  Python package, tests, messages, and simulation assets
- `clearpath/robot.yaml` — canonical robot and sensor declaration
- `patches/` — maintained changes to dependencies imported through `.repos`
- `tools/` — diagnostics, benchmark helpers, and development utilities
- `artifacts/` — generated run evidence and logs
- `build/` and `install/` — normal top-level Colcon outputs
- `docs/` — project knowledge, organized by ownership in the
  [documentation guide](documentation.md)

## Architecture pointers

- [Exploration architecture](../exploration/architecture.md)
- [Target-localization pipeline](../target_localization/target_localization_pipeline.md)
- [Target-distance benchmarking](../benchmarking/target_distance_benchmarking.md)
- [Troubleshooting](../ISSUES.md)
- [Active engineering gaps](../BACKLOG.md)

For codebase structure, also inspect `graphify-out/GRAPH_REPORT.md` before
making architecture claims. It identifies high-connectivity nodes and module
communities but does not replace the maintained references above.
