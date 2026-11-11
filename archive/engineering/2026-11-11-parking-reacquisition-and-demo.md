# Parking reacquisition near G1, target identity, timing and the recorded demonstration

Recorded dates: 2026-11-11
Tested revisions: `a67d3c6cec53df99e77b53084887afc5718ff1d2`, `640e8ec1111dbad5f27a124523d652d37822bb6b`
Provenance: partial

The recording attempts ran `a67d3c6` plus states of the changes committed as
`640e8ec` (each attempt's `run/code.txt`); attempt 3 ran the committed code.

## Question and conditions

Why did returns to the G1 observation pose stall or time out; can a parking mission be
recorded end to end; how long does each mission state take; what does the group's
latest `dev` change; and can the mission keep one G1 when two are in view? Evidence is
the 2026-11-05 to 2026-11-09 run folders and the untracked
`artifacts/exploration/demo_2026-11-11/` (one folder per recording attempt, each with
`run/` from `run_scenario.sh`, `frames/`, `frames.csv` and `demo.mp4`). Isaac runs
record no ground truth.

## Observation return

`gz_standard_run6` (2026-11-09) is the case that failed: after the LiDAR check vetoed
the `g1_left` attempt the robot stood 1.2-1.3 m from G1, below the 1.7 m minimum for
turning in place, so it drove toward the observation pose 4.3 m from G1. It reached
(-1.62, -3.43) after 34 s and then stood still about 1 m short of the goal: the
controller reported `Failed to make progress` at +55 s, Nav2 retried, and the 90 s
deadline ended the mission `MISSION_FAILED` in place (a goal Nav2 may still own
blocks the return home). `gz_standard_run3` had driven toward a first observation
pose 10 m away for the same reason; `3becd27` already moves the observation pose to
the latest verified localization. The `Parameter general_goal_checker... not found`
warnings beside those goals are harmless: `imu_filter_madgwick` logs every parameter
event, and the controller's goal-checker plugin applies the value (the view checks
measured 0.02 m / 0.03 rad, inside the 0.03 m / 0.10 rad checker only).

The 1.7 m minimum existed because G1's feet leave the image below about 1.65 m; the
position check uses the head and torso, and G1 was localized from 1.35 m on 2026-11-09
(`gz_no_feasible_run2`) and not detected at 1.01 m on 2026-10-30. The minimum is now
1.3 m. Closer, the robot steps straight back, less than 1 m, to put the camera 1.5 m
from G1 when that footprint is free in the map, and only then falls back once to the
observation pose. The same step back (up to 1.5 m) runs once before planning when G1
is localized with the robot inside G1's keep-out plus the planner's clearance: in
recording attempt 2 the search ended with the camera 0.4 m from G1 and every route was
rejected `path_g1_keepout`. Retries stay at three; cancellation, LiDAR and freshness
checks are unchanged.

## Parking near G1's legs

Recording attempt 3 parked beside G1 (`g1_right`) in Isaac with 0.28 m to the
nearest LiDAR return on the final leg; in Gazebo the 2026-11-09 record's analysis
stands (the reach band leaves
0.12-0.15 m between the rear edge and G1's legs, less than the monitor's margin plus
the map-pose error), and no further correction was made. The parking-clearance
backlog item owns it.

## Target identity with several humanoids

The detector gives humanoids no identity, and the mission took the highest-scoring
one in every frame; a relocalization gated only weak views by the known position, so
a confident second G1 could have become the target and passed the depth check. Now,
in a frame with several humanoids, the one nearest the chosen G1's expected image
column is taken, and once G1 has a position every sample must lie within 0.75 m of it
(`detection_away_from_known_target`). Unit-tested only; no two-G1 scene was run (the
demo startup spawns exactly one G1). Backlog: more than one G1.

## Mission state timing

`tools/parking_scenarios/state_timing.py` measured every state from the transition
lines of 53 runs (46 Gazebo, 7 Isaac); the table is in the maintained
states-and-timing reference (`docs/exploration/parking_mission_states.md`, `640e8ec`) reference.

## RViz on a small screen

On the 1280 x 598 remote session the general HUD and the state machine overlapped.
`rviz_layout:=compact` opens a 1280 x 598 window without the camera-image dock and
with the general HUD off; in the recordings below the state machine, the parking
legend and the target distances do not overlap at the normal text size.

## Recorded demonstration

`tools/parking_scenarios/record_demo_run.sh isaac standard OUT` runs one mission
(Isaac headless, compact RViz) and records twice a second the Isaac `task` and
`isometric` view cameras, the RViz window and the mission state into one frame;
`demo.mp4` plays them at 8x. The first try was abandoned within two minutes because
the Isaac window covered RViz; Isaac now runs headless and the recorder raises and
crops the RViz window, and resets the X idle timer every minute after the screen saver
blanked RViz in the last 58 s of attempt 1.

| Attempt | Code | Outcome | Detail |
|---|---|---|---|
| 1 (`isaac_standard_run1`) | before the identity guard and the step back before planning | `MISSION_FAILED`, returned | G1 confirmed and verified from 12.3 m after 317 s of search; exploring around G1 found no reachable frontier; relocalized from 5.2 m, then every pose `footprint_occupied` (188) or `footprint_unknown` (13), `no_valid_candidate`; 375 s; video 50 s, RViz blank in its last 58 s |
| 2 (`isaac_standard_run2`) | before the step back before planning | `MISSION_FAILED`, returned 0.06 m from the start | the search ended with the camera 0.4 m from G1 (G1 0.07 m from its configured pose); every route `path_g1_keepout`, `bounded_search_exhausted`; 706 s; video 91 s |
| 3 (`isaac_standard_run3`) | `640e8ec` | `MISSION_COMPLETE`, returned 0.016 m from the start | search 484 s, ended with G1 0.25 m from the footprint; stepped back 1.03 m, relocalized from 1.45 m (G1 0.057 m from its configured pose); `g1_right` on the first attempt, planned reach 0.509 m, map clearance 0.101 m, arrival error 0.023 m, final-leg LiDAR minimum 0.28 m; 5 s wait; 628 s; video 81.5 s, 1.2 MB |

## `dev` on 2026-11-11

`origin/dev` is 47 commits ahead of the merge base `6d81e67`: benchmark qualification
and placed-scene tooling, Isaac render settings and a Kit CPU-pin shutdown fix, HUD and
overlay shutdown-signal fixes, a 0.30 default detector threshold (the parking launch
sets 0.40 explicitly), the full `ede9bd1` (goal checker at any heading), the hardware
front datum, and Nav2 changes that matter here: a mesh-shaped robot outline (rear edge
at -0.476 m instead of -0.497 m) and 0.05 m costmaps. The parking planner loads its
footprint from that file, so a merge changes every parking pose and clearance and
needs the workstation scenes rerun. A trial merge (`git merge-tree`) conflicts in
`README.md`, `docs/exploration/architecture.md`,
`frontier_explorer/frontier_explorer_node.py`, `src/ridgeback_autonomy_isaac/sim/isaac/worlds.py`
and the generated graph. Nothing from `dev` was integrated.

## Gazebo world

`initial_test_world`, the repository's plain multi-room geometry, is the world Isaac's
parking scene is layered on, with the same coordinates. Using it for Gazebo parking
needs a workstation keyed by world as well as backend in the scenario file and the
world passed through `start_parking_demo.sh`; not done.
