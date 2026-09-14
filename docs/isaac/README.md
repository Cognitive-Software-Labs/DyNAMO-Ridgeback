# Isaac Sim port

Adding **Isaac Sim 6.1 GA** as a first-class backend for the shared Ridgeback
autonomy stack. Gazebo and hardware remain separate adapters. In progress on
`feat/isaac-sim-6-port`.

## Start here

| doc | what it owns |
|---|---|
| [**open-issues.md**](open-issues.md) | **what is currently broken** — read first |
| [port-plan.md](port-plan.md) | the phase plan, P0–P9, and acceptance criteria |
| [**migration-6.1.md**](migration-6.1.md) | **approved 6.1 migration, driver rollout, 5.1 safety gate, and depth decision** |
| [**camera-depth.md**](camera-depth.md) | **Isaac D455 colour/depth implementation, modes, ROS contract, validation, and troubleshooting** |
| [robot-model.md](robot-model.md) | URDF → USD → meshes → sensors → colliders |
| [lidar-pipeline.md](lidar-pipeline.md) | how the RTX lidar reaches ROS, and SLAM quality |
| [port-history.md](port-history.md) | superseded investigation narratives |

Assets: [`assets/robot-geometry.svg`](assets/robot-geometry.svg) is the
dimensioned mounting drawing; `assets/robot-render.png` is the same geometry in
Isaac.

## Current state, in one paragraph

Navigation **runs** on Isaac Sim 6.1. The long-standing stall was the 2D
lidars being parented to `base_link`, which has no joint into the
articulation, so PhysX turned `chassis_link` and left the sensors behind; the
chassis sweeping under a stationary emitter is what produced the "phantom"
returns that pinned `cmd_vel` at zero. Reparenting them to `chassis_link`
fixed it. The 6.1 migration passed its bounded compatibility gate on
2026-09-13 at RTF 0.868 with 17 successful goals and no aborts. This is not a
statistical baseline, and every older coverage or SLAM number measured before
the 2026-09-10 geometry correction remains void. The robot still floats
49.8 mm on stock worlds (§2). P0–P4, P7, and the P9 runtime migration are
done; P5 still needs a multi-seed baseline and P6 is deferred. See
[`migration-6.1.md`](migration-6.1.md) for the execution record.

## Where facts live

Each fact has one home. Cross-reference rather than copy:

- **broken now** → `open-issues.md`
- **plan, phases, acceptance** → `port-plan.md`
- **6.1 migration and shared-driver procedure** → `migration-6.1.md`
- **Isaac D455 image/depth implementation and runbook** → `camera-depth.md`
- **robot geometry, the importer, colliders** → `robot-model.md`
- **lidar → ROS, scan assembly, SLAM quality** → `lidar-pipeline.md`
- **concluded investigations** → `port-history.md`
- **cross-backend recovery procedures** → `../troubleshooting.md`
- **completed root causes and evidence** → `../history/`
- **benchmark recipe and hygiene** → `../exploration/benchmarking.md`
- **ground-truth maps** → `../../src/ridgeback_autonomy/sim/ground_truth_maps/README.md`

Numbers in `port-plan.md` and `port-history.md` are **claims at the time they
were written**, not current — treat them as history, not as a baseline.
