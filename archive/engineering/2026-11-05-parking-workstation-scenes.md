# Parking workstation scenes, route checks near the spawn, and the presentation pages

Recorded dates: 2026-11-05, 2026-11-08
Tested revisions: `bb8aa94cc6c96051269214266fbb08c5f76199f1`, `b4f855bf3dff110623df3a68a356925c32b63115`
Provenance: partial

## Question and conditions

Can the unchanged mission find G1 standing at a worktable out of the robot's sight,
plan beside it and park, in both simulators; and can the scenes and runs be shown on
trustworthy pages? Every run used `tools/parking_scenarios/run_scenario.sh`
(loopback DDS domain 83, one simulator at a time, SIGTERM to its own session) on
HEAD `bb8aa94` plus the working tree committed as `b4f855b`, and the unrelated
uncommitted `--parallel` change to `start_parking_demo.sh`. Each run folder keeps
the `scenarios.yaml` it ran. Evidence is in the untracked
`artifacts/exploration/workstation_2026-11-05/`, `workstation_2026-11-08/`,
`scenes_2026-11-05/` and `scenes_2026-11-08/`. Gazebo reach, pose error and clearance
are ground truth, never fed to the mission; Isaac rows are estimated state.

## Scenes

Schema `ridgeback_parking_scenarios/2` places G1 at a workstation, 0.75 m from a
worktable and facing it, at one pose per world; each scenario places its objects in
the workstation frame, so a scenario has the same layout around G1 in both worlds.
The robot keeps both simulators' default spawn (origin, facing +x).

Three Gazebo workstations were tried:

| Workstation | Runs | Outcome |
|---|---|---|
| (2.5, -6.0), aisle behind the long shelf south of the start | `gz_standard_aisle`, `gz_no_feasible_aisle` (11-05) | the explorer never entered the aisle in 20 min; one run saw G1 through a gap from 9.3 m and then failed planning on a delayed odometry stamp (backlog) |
| (0.5, 5.0), floor north of the start | `gz_*_north` (11-05, 11-08) | G1 found within about 30 s, heading 82 deg (truth 90); every route rejected `path_footprint_unknown` (below) |
| (-5.5, 0.5), edge of the west lane | `gz_standard_west` | found from the front-left at 2.8 m; the static view scored 0.42-0.45, below the 0.50 first-localization floor |
| (-4.5, -0.6), in the west lane, facing west | `gz_standard`, later runs | kept |

## Route checks near the spawn

With G1 north of the start, all 21 valid poses were rejected `path_footprint_unknown`.
Replaying the mission's own `path_rejection` on the recorded map (received 0.4 s
before evaluation, the version RViz showed) and Nav2's recorded plans found two causes:

1. A defect. NavFn starts a path at the robot's grid-snapped cell and, here, put a
   second pose in that cell's neighbourhood, 6 cm behind the robot. `driven_poses`
   dropped only the first planner pose, so the check swept the footprint 6 cm
   backwards onto two unseen cells at (-1.718, 0.98) and (-1.718, 1.03), beside the
   robot. The robot does not drive there. All leading placeholder poses within the
   0.10 m start tolerance are now skipped; a real heading there is still swept as an
   in-place rotation, and anything farther is swept as driven.
2. Genuinely unknown space. With that fixed, the 11-08 north run was still rejected:
   after leaving its spawn by 0.6 m and turning in place, the robot had a wedge of
   never-scanned floor beside its flank (both LiDARs are blocked by the chassis there),
   and every route began by moving the flank into it. The checker is right; G1-centred
   exploration only maps the parking area, not the route. A workstation works when the
   robot has driven into G1's area before it sees G1, which scans the floor beside it.

## Runs

| Run | Workstation | Outcome | Detail |
|---|---|---|---|
| `workstation_2026-11-05/gz_standard_aisle` | aisle | `PARKING_PLANNING_FAILED`, stopped | G1 seen through a gap from 9.3 m after 62 s; g1_left selected, then the plan was invalidated by a delayed odometry stamp |
| `workstation_2026-11-05/gz_no_feasible_aisle` | aisle | runner limit (1200 s) | the explorer never entered the aisle; G1 never closer than 6.5 m |
| `workstation_2026-11-05/gz_standard` | north | `MISSION_FAILED`, returned | all routes `path_footprint_unknown` (start-cell back-step), before the fix |
| `workstation_2026-11-08/gz_standard_north` | north | `MISSION_FAILED`, returned | after the fix: still all routes rejected, now by the unscanned flank wedge |
| `workstation_2026-11-08/gz_left_blocked_north` | north | `MISSION_FAILED`, returned | as above |
| `workstation_2026-11-08/gz_no_feasible_north` | north | `MISSION_FAILED`, stopped | localization timeout inside the pen |
| `workstation_2026-11-08/gz_standard_west` | west edge | `MISSION_FAILED`, stopped | confirmed at 2.8 m from the front-left; static view 0.42-0.45 on 52/52 frames |
| `workstation_2026-11-08/gz_standard` | lane | `MISSION_FAILED`, returned | found after 422 s, heading 175 deg (truth 180); g1_right, g1_left, then the approach side, each vetoed by the LiDAR monitor (obstacle on the route; in the stopping path 0.15 m, 0.44 m from G1); G1 relocalized before both retries; back 0.04 m from the start |
| `workstation_2026-11-08/gz_standard_run2` | lane | `MISSION_FAILED`, stopped | found after 946 s, then first-localization timeout |
| `workstation_2026-11-08/gz_no_feasible` | lane | `MISSION_COMPLETE`, but parked 1.72 m from G1 | G1 localized 1.25 m behind itself, beyond its table; parked safely (0.30 m from the table) beside the wrong estimate |
| `workstation_2026-11-08/gz_left_blocked` | lane | `MISSION_FAILED`, stopped | G1 localized 1.09 m off toward the panel (heading right); relocalization timeout |
| `workstation_2026-11-08/gz_heading_90` | lane | `MISSION_FAILED`, stopped | confirmed at 0.500, first-localization timeout |
| `workstation_2026-11-08/isaac_standard` | south room | `MISSION_FAILED`, stopped | confirmed at 0.749 while moving; after the stop G1 was out of view (no detection on 53 frames), first-localization timeout |
| `workstation_2026-11-08/isaac_no_feasible` | south room | runner limit (1200 s) | G1 never confirmed inside the pen |

No workstation run parked beside G1. The pipeline ran end to end in
`gz_standard` (lane): search, detection, localization with the right heading, parking
planning, three attempts with G1 relocalized before each retry, and a safe return;
the LiDAR monitor vetoed every approach near G1's legs and the table. The other runs
stopped at G1's first localization: a static view below the 0.50 floor, G1 out of view
after the stop, or a depth estimate pulled 1.1-1.25 m off G1 by the table or a panel
inside its detection box (one of which then parked safely beside the wrong estimate).
These are perception limits in cluttered, out-of-sight scenes, recorded in the G1
detection backlog item; the parking planner, retries and safety checks were unchanged
except for the route-start fix.

## Presentation

An audit of the P7 gallery images found Gazebo GUI views dominated by the standing
person beside the start, views of other areas, Isaac viewports without G1, and RViz
crops where the exploration HUD covered the state machine. They were withdrawn from
the pages and stay in the run folders. New scene views are rendered by the simulators
(temporary Gazebo camera sensors; the Isaac runner's opt-in view cameras), checked for
blank frames and listed in manifests; results-page images are labelled with the
mission state active when they were captured.

RViz ignored the layout's `Hide Left Dock` when restoring the saved window state
(reproduced with a standalone `rviz2` on 2026-11-08); the launcher now clears each left
dock's visibility in that state. The overlays need a 3D view about 620 px tall; the
remote session used here (1280 x 598) is smaller, and there they overlap.

## Limitations

- One or two runs per scene; search time depends on the explorer's frontier order.
- The explorer sends goals with a fixed heading on this branch, so the holonomic base
  often travels sideways or backwards; the group's `dev` fixes it in `ede9bd1`, which
  also widens the shared goal checker and is left for the next `dev` merge.
- Side poses come close to G1's legs and the table; the LiDAR monitor vetoes such
  approaches.
