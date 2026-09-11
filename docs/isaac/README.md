# Isaac Sim port

Porting the Ridgeback autonomy stack from Gazebo Harmonic to **Isaac Sim 6.0
GA**. In progress on `feat/isaac-sim-6-port`.

## Start here

| doc | what it owns |
|---|---|
| [**open-issues.md**](open-issues.md) | **what is currently broken** — read first |
| [port-plan.md](port-plan.md) | the phase plan, P0–P9, and acceptance criteria |
| [robot-model.md](robot-model.md) | URDF → USD → meshes → sensors → colliders |
| [lidar-pipeline.md](lidar-pipeline.md) | how the RTX lidar reaches ROS, and SLAM quality |
| [port-history.md](port-history.md) | superseded investigation narratives |
| [handoff.md](handoff.md) | session handoff for the next agent |

Assets: [`assets/robot-geometry.svg`](assets/robot-geometry.svg) is the
dimensioned mounting drawing; `assets/robot-render.png` is the same geometry in
Isaac.

## Current state, in one paragraph

Navigation **runs** as of 2026-09-11. The long-standing stall was the 2D
lidars being parented to `base_link`, which has no joint into the
articulation, so PhysX turned `chassis_link` and left the sensors behind; the
chassis sweeping under a stationary emitter is what produced the "phantom"
returns that pinned `cmd_vel` at zero. Reparenting them to `chassis_link`
fixed it. **No Isaac baseline has ever been rerun** — that is the next
deliverable — and every coverage and SLAM number on record is void, having
been measured either before the 2026-09-10 geometry changes or with a sensor
that did not rotate with the robot. The robot still floats 49.8 mm on stock
worlds (§2). P0–P4 and P7 are done; P5 plumbing is done with sign-off pending
a baseline; P6 is deferred.

## Where facts live

Each fact has one home. Cross-reference rather than copy:

- **broken now** → `open-issues.md`
- **plan, phases, acceptance** → `port-plan.md`
- **robot geometry, the importer, colliders** → `robot-model.md`
- **lidar → ROS, scan assembly, SLAM quality** → `lidar-pipeline.md`
- **concluded investigations** → `port-history.md`
- **solved problems worth keeping** → `../../ISSUES.md` (repo-wide archive)
- **benchmark recipe and hygiene** → `../../tools/benchmark/README.md`
- **ground-truth maps** → `../../src/ridgeback_autonomy/sim/ground_truth_maps/README.md`

Numbers in `port-plan.md` and `port-history.md` are **claims at the time they
were written**, not current — treat them as history, not as a baseline.
