# EXPLORATION - Parking mission migration

Status: **Phase 1 geometry foundation implemented; mission manager migration
pending.** This plan covers adapting the existing Ridgeback-G1 parking mission
to the current package-split architecture without duplicating localization or
changing Nav2 safety configuration.

## Phase 1 foundation

The pure parking math now lives in
`ridgeback_autonomy.mission.parking_geometry`. Its task-point transforms,
inverse base-pose calculation, angle normalization, rotated footprint math,
pre-parking offset, path length, and inclusive reach-boundary behavior are
unchanged from the original mission. The geometry tests are independent of ROS
nodes and are registered with the package's existing ament pytest suite.

The original `test_mission_geometry.py` also contains tests for occupancy-grid
sampling, footprint and full-path rejection, candidate ranking, marker state,
mission commitment, start-pose capture, and localization fallback. Those are
mission-manager integration tests rather than parking-geometry tests. They are
deferred to Phase 2 and must be migrated with their assertions intact once the
new manager exists.

## Compatibility matrix

| Area | Original mission contract | Current dev contract | Phase 2 adaptation |
|---|---|---|---|
| Detection | `G1Detections` on `detections/g1/raw` | `ridgeback_interfaces/TargetDetections` on `detections/target/raw` | Consume the shared type and keep target-found confirmation behind a small mission input boundary so detection is not hard-wired to G1 naming. |
| Measurements | Monolithic `G1Measurements` on `measurements/g1/camera` | `TargetMeasurements` split across `measurements/target/mask` and optional `measurements/target/pointcloud` | Select one qualified localization source or fuse explicit valid fields. Do not recreate RGB-D processing inside the mission manager. |
| Measurement semantics | Legacy RGB, sensor-depth, mono-depth, pointcloud, and LiDAR arrays | Pointcloud, projective-ranging, euclidean-reconstruction, and polar-profiling arrays with per-estimator status | Map only valid estimator fields. Legacy `sensor_depth_distance_m`, `rgb_*`, `mono_depth_*`, and `lidar_*` have no direct schema equivalent. |
| Planar reference | Legacy mission reconstructed camera/base positions itself | Shared estimators publish lateral as base +Y left-positive and forward from the robot-front reference | Call `remove_vehicle_front_offset()` before map projection, then transform the base-origin-relative point with the measurement-time `map -> base` transform. |
| Time/synchronization | Mission cached raw RGB/depth/cloud and used legacy matching helpers | Localization workers own sensor matching and publish source-stamped batches; shared stamp/buffer helpers replace old utilities | Treat the localization output stamp as authoritative. Do not add a second raw-sensor synchronization path in autonomy. |
| TF | Mission assumed `map` and bare `base_link` under namespaced TF topics | Topic namespace and TF frame IDs are deliberately independent; simulation localization currently receives `r100_0001/robot/base_link`, while Nav2 uses `base_link` | Make map and observed base frame explicit launch parameters and verify the live TF chain. Never derive frame IDs from the ROS namespace. |
| Exploration ownership | `explore/set_enabled` (`SetBool`) plus `explore/active` (`Bool`) | No equivalent stop/active pair. `MissionGuard` owns the explorer goal, exposes latched JSON on `mission/status`, and `mission/resume` only resumes a healthy, settled owner | Add an explicit, terminally acknowledged mission pause/stop contract to the existing guard or a narrowly scoped owner API. Reuse its handle identity and cancellation-settling rules; do not add sleeps or a second Nav2 owner. |
| Exploration status | Active Boolean represented an in-flight goal | `explore/status` reports `exploration_started` or `exploration_complete`; `mission/status` reports paused, health, cancellation, and goal-pending state | Use `mission/status` for ownership settlement. `explore/status` alone does not prove an action goal is terminal. |
| Nav2 actions | Relative `compute_path_to_pose` and `navigate_to_pose`, planner loaded from YAML | Same namespaced actions and `GridBased` planner remain | Preserve action code, error-code checks, non-empty-path requirement, and sequential ownership after explorer settlement. |
| Footprint | Mission loaded the global-costmap footprint from Nav2 YAML | YAML now contains the authoritative 16-point Ridgeback envelope with padding | Continue loading current configuration; do not copy the old rectangular test footprint into runtime code. Retest complete-footprint and path sampling against the new polygon. |
| Costmaps/map | Occupancy map plus mission-side unknown/occupied/path checks | SLAM/Nav2 still publish map and costmaps, with global planning allowed through unknown for exploration | Preserve stricter mission candidate/path rejection of unknown space. Do not change Nav2 parameters. |
| Shared helpers | TF, pointcloud, image, and synchronization helpers lived under autonomy | Helpers moved to `ridgeback_common` and `ridgeback_localization.core`; some legacy helpers were removed | Import shared TF/image/pointcloud utilities from their owning packages. Replace removed bbox/time helpers with current structured APIs. |
| RViz markers | Mission-owned reachability, candidate, selected-pose, and thesis marker topics | Target estimate visualization is generic; current exploration RViz has no parking displays | Retain mission-owned parking markers on relative namespaced topics and add only the final clean displays to the current RViz layout. Do not reuse target-estimate namespaces as parking state. |
| Launch/readiness | Monolithic mission launch brought up simulator, perception, SLAM, Nav2, explorer, manager, and RViz | `ridgeback_exploration.launch.py` owns backend-neutral bringup, readiness gates, localization mode, and autonomous-motion gating | Add the manager through the existing event-driven launch boundary after localization/Nav2/explorer readiness. Preserve `backend`, `target_localization_enabled`, `autonomous_motion_enabled`, namespace, and simulated-time arguments. |

## Code that can be preserved

- The migrated parking geometry module.
- Start-pose capture and immutable return target, after frame parameters are adapted.
- The committed-target state sequence and three-of-five confirmation policy.
- Reach-annulus sampling and occupancy classification.
- Task-point-to-base conversion and pre-parking geometry.
- Complete rotated-footprint, clearance, and sampled-path checks.
- `ComputePathToPose` validation, deterministic candidate ranking, sequential
  pre-parking/final navigation, parking wait, and return-to-start behavior.
- Mission-owned reachability and selected-pose marker construction, after topic
  names and the current footprint are wired in.

## Code that requires adaptation

- All imports of generated `G1*` messages and old autonomy-owned perception
  helpers.
- Legacy measurement field selection and raw depth/cloud synchronization.
- Map projection of front-referenced target measurements.
- The exploration stop/settled handshake; the old service and active topic do
  not exist in the new explorer.
- Frame defaults and TF lookup assumptions.
- Marker input logic that depended on `g1_estimates/rgb`; current estimator
  marker namespaces are generic and are visualization output, not a stable
  mission data contract.
- Launch integration, readiness, node installation, and RViz displays.
- Manager-coupled tests from the original geometry test module.

## Phase 2 implementation order

1. **Define the target input boundary.** Add a small adapter from
   `TargetDetections` to a detector-independent target-found event and preserve
   the current three-of-five confirmation behavior.
2. **Choose and test one localization contract.** Consume qualified
   `TargetMeasurements`, honor estimator status, undo the robot-front offset,
   and transform each sample at its source stamp. If the existing estimator
   outputs cannot supply the required stable planar target center, extend
   `ridgeback_localization` once rather than rebuilding perception in the
   mission manager.
3. **Add an explicit explorer ownership handoff.** Extend `MissionGuard` with a
   deliberate pause/stop request whose completion is reported only after the
   owned Nav2 goal reaches a terminal result. Keep health-fault pause and
   operator resume semantics intact.
4. **Port the mission skeleton.** Restore start-pose capture, search,
   commitment, stop settlement, localization, safe-failure, return, and terminal
   states with state-transition unit tests before parking actions are enabled.
5. **Port candidate generation and map safety.** Load the current Nav2
   footprint/planner configuration, then migrate occupancy, rotated-footprint,
   clearance, and sampled-path checks. Restore the deferred safety assertions.
6. **Port Nav2 feasibility and ranking.** Preserve all-candidate evaluation,
   action result/error checks, path length, and deterministic ordering. Do not
   introduce the supervisor's proposed donut changes in this migration.
7. **Port execution.** Add pre-parking, final parking, the existing wait, and
   return-to-start actions only after explorer ownership is proven settled.
8. **Port markers and launch integration.** Add relative marker topics and
   clean RViz displays, then integrate the manager into the current readiness
   chain without changing backend packages or Nav2 safety parameters.
9. **Restore integration coverage.** Migrate the remaining manager tests,
   exercise message serialization/frame conversions, fake action ownership and
   cancellation, then run bounded Gazebo startup and end-to-end mission tests.

## Phase 2 acceptance gates

- No legacy `G1Detections`/`G1Measurements` dependency and no duplicate
  perception pipeline.
- Every stabilized target sample is map-frame data transformed at its source
  timestamp with the robot-front offset handled exactly once.
- Frontier and mission navigation goals never overlap; handoff waits for a
  terminal action result.
- The current authoritative footprint and planner are loaded from configuration,
  with unknown/out-of-map/path-footprint rejection retained.
- A failed target localization or empty feasible set follows the documented
  mission outcome without changing SLAM, localization workers, frontier
  selection, or Nav2 safety parameters.
- Geometry and migrated manager tests pass before the mission manager is added
  to a public launch.
