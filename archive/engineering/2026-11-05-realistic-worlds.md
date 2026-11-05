# Realistic world and pedestrian profile validation

Recorded dates: 2026-11-05
Tested revisions: `566fbd8cd58dbd61a5d90adc4eb1f4ac4a06cd49`
Provenance: partial

The tested continuation includes dirty-tree changes beyond the listed checkpoint.
The [source/receipt snapshot](assets/2026-11-05-realistic-worlds/source-and-receipts.tar.gz)
and [provenance receipt](assets/2026-11-05-realistic-worlds/provenance.json)
preserve source, hashes and conversion inputs. Binary assets are retained through
repository Git LFS rather than embedded in this record; raw logs, generated
worlds, setup files and per-run source snapshots remain under
`artifacts/leader-following/realistic-development/`. This record does not claim
that the checkpoint alone reproduces the final render or startup behaviour.

## Population and visual design

Three additional worlds use shared static layouts: an office lobby, a warehouse
and a courtyard. The props include fourteen Poly Haven asset types, textured
surfaces, daylight HDRI, original wayfinding panels and constructed architecture.
The source characters are Innerscene/MakeHuman Sam, Maya, Harper and Jordan.
Repeated clothing/stature variants fill crowds; deliberate look-alikes copy the
leader's model, scale and clothing. This is four source identities, not nine
independently modelled people. All routes are scripted and the robot is fixed.

Original F1/F2/F3 corpora remain a separate result population. Their detector
ranking, calibration and identity scores do not qualify the new cast, lighting,
obstacles or image backgrounds. The realistic alone route adds straight sections
and turns; the other five fixture paths are retained.

The first visual review exposed roof occlusion in the camera placement,
vertically oriented wall textures, dark faces and matching shirt/trouser colours.
The continuation corrected UV orientation, moved views inside the usable space,
added enclosed ceilings/skylights and practical lights, and retained neutral
casual trousers. Office clutter, bins and signs provide additional visual context.

## Asset conversion and collision cost

Static export normalizes world-space geometry to a floor-centred origin and
verifies the exported pose against its vertex cache. The final conversion
receipts report at most approximately 6e-8 m deviation across the fourteen props.
Duplicate source material names are normalized during import. Human Collada
exports retain the existing cache agreement and single-root skeleton checks.

The tree source contains 2,062,487 triangles. A simulation LOD reduces that to
180,000 triangles. The first Gazebo courtyard startup timed out before sensor
readiness. A retry using approximate cylindrical trunk collision instead of the
foliage triangle mesh completed; the same representation passed in Isaac.
The collision proxy does not reproduce branch or foliage contact. Rendered
foliage remains in the visual/lidar geometry.

## Clock startup failure

A later parallel Gazebo warehouse run also timed out, with thousands of valid
RGB, depth and scan messages but no dynamic TF/odometry readiness. Its controller
manager repeatedly reported that no clock had been received. This distinguishes
that failure from a missing camera or failed mesh load.

The short `/clock` Gazebo topic is conditional at startup. The local upstream
`ros_gz_bridge` README documents that Gazebo can publish only the world-qualified
clock when another publisher is detected. The fixture launch now bridges
`/world/leader_follow_scene/clock` unidirectionally to ROS `/clock`. Three fresh,
simultaneous Gazebo starts then completed. Earlier failed evidence is retained;
the result is not obtained by extending readiness timeouts or bypassing checks.

## Verification boundary

The [startup summary](assets/2026-11-05-realistic-worlds/startup-summary.json)
contains one complete sensor/observed-ground-truth run per world/backend pair:

| Backend | World | Scenario family | Commanded duration |
|---|---|---|---:|
| Gazebo | Office | Crossing | 1 s |
| Gazebo | Warehouse | Stop in group | 1 s |
| Gazebo | Courtyard | Crowd, eight bystanders | 1 s |
| Isaac | Office | Look-alike parallel | 1 s |
| Isaac | Warehouse | Swap/look-alike crossing | 1 s |
| Isaac | Courtyard | Alone | 1 s |

These checks establish startup and basic sensor/truth delivery, not completion
of the full twenty-second scenarios or recognition performance. Static tests
sample every pedestrian path across all six families and three worlds for
obstacle clearance, and check deliberate twins, cast variety, actor scale,
texture paths, UV orientation and preservation of commanded paths.

Review PNGs are derived from the actual Isaac renderer, with the Ridgeback and
nine-person crowd. They are visual evidence rather than inference scores.
The simulators close after checks and rendering. Full leader-recognition
qualification, moving-robot control, hardware timing and sim-to-real performance
remain outside this validation.
