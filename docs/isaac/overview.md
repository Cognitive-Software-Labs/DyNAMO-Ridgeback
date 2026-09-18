# Isaac Sim backend overview

Isaac Sim 6.1 is one of the shared autonomy stack's three backends, alongside
Gazebo and physical hardware. The bounded migration compatibility gate passed;
statistical exploration qualification remains open. The
[global backlog](../BACKLOG.md) owns remaining work, including deferred benchmark
support. Historical controls are not current performance baselines.

## Current references

| Document | Owns |
|---|---|
| [Shared robot geometry](../robot/geometry.md) | Mounts, dimensions and provenance |
| [Collision model](../robot/collision_model.md) | Physical envelope, Nav2 footprint and backend distinctions |
| [Isaac robot model](robot-model.md) | USD import, articulation and PhysX colliders |
| [LiDAR pipeline](lidar-pipeline.md) | Scan assembly, ROS contract and qualification procedure |
| [Camera depth](camera-depth.md) | D455 modes, ROS contract, validation and troubleshooting |
| [Exploration evaluation](../exploration/evaluation.md) | Coverage, navigation outcomes, and run procedure |
| [Ground-truth maps](../../src/ridgeback_autonomy/sim/ground_truth_maps/README.md) | Map generation and provenance |
| [Rollback](rollback.md) | Retained environments, shared-driver recovery and verification |

## Archived evidence

- [Isaac 6.0 first-drive snapshot](../../archive/engineering/2026-07-11-isaac-first-drive.md)
- [6.1 migration](../../archive/engineering/2026-09-13-isaac-6.1-migration.md)
- [LiDAR qualification](../../archive/engineering/2026-09-17-isaac-lidar-qualification.md)
- [Gazebo lidar integration](../../archive/engineering/2026-09-18-gazebo-lidar-integration.md)
- [World seating](../../archive/engineering/isaac_world_seating.md)
- [Investigation history](../../archive/engineering/port-history.md)
