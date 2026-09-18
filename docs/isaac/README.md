# Isaac Sim

Isaac Sim 6.1 is one of the shared autonomy stack's three backends, alongside
Gazebo and physical hardware. The bounded migration compatibility gate passed;
statistical exploration qualification remains open. The
[global backlog](../BACKLOG.md) owns remaining work, including deferred benchmark
support. Historical controls are not current performance baselines.

## Current references

| Document | Owns |
|---|---|
| [Robot model](robot-model.md) | Geometry, importer, sensors and colliders |
| [LiDAR pipeline](lidar-pipeline.md) | Scan assembly, ROS contract and qualification procedure |
| [Camera depth](camera-depth.md) | D455 modes, ROS contract, validation and troubleshooting |
| [Exploration benchmarking](../exploration/benchmarking.md) | Run procedure and benchmark hygiene |
| [Ground-truth maps](../../src/ridgeback_autonomy/sim/ground_truth_maps/README.md) | Map generation and provenance |
| [Rollback](rollback.md) | Retained environments, shared-driver recovery and verification |

## Archived evidence

- [6.1 migration](../../archive/engineering/2026-09-13-isaac-6.1-migration.md)
- [LiDAR qualification](../../archive/engineering/2026-09-17-isaac-lidar-qualification.md)
- [Gazebo lidar integration](../../archive/engineering/2026-09-18-gazebo-lidar-integration.md)
- [World seating](../../archive/engineering/isaac_world_seating.md)
- [Investigation history](../../archive/engineering/port-history.md)
