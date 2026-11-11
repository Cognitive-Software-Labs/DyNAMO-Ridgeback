# G1 position check, first localization and the workstation reruns

Recorded dates: 2026-11-09
Tested revisions: `8dd2a1e3d3d22fe6461d237e0cb9cdd21fe11243`, `3becd2773c9e97082a00d172ffe72b94313c38dd`
Provenance: partial

Runs 1-5 of `gz_standard` ran `8dd2a1e` plus earlier states of the changes committed
as `3becd27`, each before one later fix (named in each run's `code.txt`); the other
runs ran the code committed as `3becd27`.

## Question and conditions

Why did the 2026-11-08 Gazebo `no_feasible` workstation run park 1.72 m from G1 and
report `MISSION_COMPLETE`; can G1's position be made reliable without simulator
input; and does the workstation scene then park? Runs used
`tools/parking_scenarios/run_scenario.sh` (loopback DDS domain 83, one simulator at a
time, SIGTERM to its own session) and, for the camera views,
`tools/parking_scenarios/probe_g1_views.sh` with `capture_g1_views.py`, which moves the
robot in a `--plan-only` Gazebo session and records depth, the camera pose and the
detector's boxes. Gazebo ground truth was used only to measure errors. Evidence is in
the untracked `artifacts/exploration/g1_views_2026-11-09/` and
`artifacts/exploration/workstation_2026-11-09/`; the runs now also record both 2D scans.

## Cause of the 1.25 m error

At the 2026-11-08 `no_feasible` localization the robot stood at (-10.99, -3.72),
7.2 m from G1, looking along a diagonal over the worktable. The probe reproduced that
view to the millimetre. Inside G1's detection box (31 x 69 px) the depth held three
layers: the worktable's near edge and legs at 5.6-6.2 m (heights 0.16-0.76 m), G1 at
6.4-7.0 m (up to the box top) and the bed behind it at 7.6-8.6 m. All four shared range
estimators reported about 5.8 m, the table edge: they anchor on the nearest surface
in the box. The estimate therefore lay on G1's bearing but 1.2 m short, at the
table's corner. The body-axis step then found the table's own legs within 0.30 m
of that seed (G1's were 1.2 m further), and the heading step matched the tabletop
edge (-12 deg against 180). TF and timestamps were consistent: the robot was still
and the bearing was right. With a panel beside G1 (`left_blocked`) the panel's end
inside the box gave the same failure, about 1 m off.

## Changes

- Position check (`target_orientation.py`, `_observe_target_body`): the top quarter
  of the box across its middle 40% is G1's head and upper torso; its nearest fifth of
  ranges is G1's range. A surface estimate more than 0.50 m in front of it or 0.30 m
  behind it is another object, and the position is re-seeded on the head before the
  legs refine it. A position is used only with at least 20 depth points at 0.85-1.40 m
  within 0.25 m of it in 3 frames; otherwise `target_position_unverified`, and the
  localization deadline ends the attempt. On the probe views the checked positions
  were 0.018-0.034 m from ground truth, including both recorded failures, and the
  table edge the run parked beside had no torso points.
- First localization: a turn in place toward the confirmed detection's camera ray
  when the stop left it more than 10 deg off the camera axis, turning back once if G1
  is then not localized; weak views from 0.40 accepted within 0.75 m of that ray.
- While the robot stands still for planning, new views of G1 within 0.75 m of its
  estimate renew the estimate's stamp; the observation pose follows the latest
  verified localization; an odometry stamp ahead of the node's clock restarts the
  stationary interval instead of discarding the plan.
- Planner: valid poses keep the LiDAR check's 0.05 m margin plus the 0.03 m final
  tolerance to occupied cell centres (`obstacle_clearance`).
- Explorer goals face along their approach (the heading part of `dev` `ede9bd1`);
  the shared goal checker keeps 0.40 rad.

## Runs

| Run | Code | Outcome | G1 error (truth) | Detail |
|---|---|---|---|---|
| `gz_standard` | before ray hysteresis | `MISSION_FAILED`, stopped | not localized | confirmed at 0.556 from about 9 m; the centred static view scored 0.44 on every frame; first-localization timeout |
| `gz_standard_run2` | before target refresh | `PARKING_PLANNING_FAILED` (`target_stale`), stopped | 0.118 m | turned +34 deg toward the detection, localized and verified from 6.0 m; the settle wait, path checks and stationary wait outlasted the 5 s target age before driving |
| `gz_standard_run3` | before the observation pose followed | `MISSION_FAILED`, stopped | 0.045, then 0.009 m | verified from 10.1 m, explored around G1, relocalized from 1.7 m (heading 169.5 deg, error 10.5 deg); the return to the first observation pose 10 m away ran past its deadline |
| `gz_standard_run4` | before the turn-back fallback | `MISSION_FAILED`, stopped | not localized | after the +34 deg turn the centred view of G1 7 m away produced no detection at 0.40; the views before the turn had scored 0.42-0.52 |
| `gz_standard_run5` | before the odometry-stamp restart | `PARKING_PLANNING_FAILED` (`motion_evidence_lost: odometry_timestamp`), stopped | 0.055 m | turned back after a failed localization; the surface estimate (-5.559, -1.186) was the worktable edge and was re-seeded on G1's head (heading 175.1 deg, error 4.9 deg); `g1_left` selected on an 8.1 m route |
| `gz_standard_run6` | `3becd27` | `MISSION_FAILED`, stopped | 0.173, then 0.130 m | `g1_left` attempt vetoed (`lidar_obstacle_on_route`, a return in the final pose's rear corner, free in the SLAM map, 0.2 m from G1's mapped legs); the return to the observation pose 4.3 m from G1 took the full 90 s, slowed by the collision monitor |
| `gz_no_feasible` | `3becd27` | runner limit (1800 s) | not localized | G1 never seen; the explorer searched the far south-west |
| `gz_no_feasible_run2` | `3becd27` | `MISSION_FAILED`, returned 0.17 m from the start | 0.068 m | verified from 1.35 m; `no_valid_candidate` (every footprint occupied), no attempt; ground-truth clearance 0.027 m base-to-worktable during the search (the tabletop is above the base) |
| `isaac_standard` | `3becd27` | `MISSION_COMPLETE`, returned | no Isaac truth | verified from 8.1 m (0.22 m from the configured G1 pose, SLAM drift included), relocalized from 2.4 and 2.2 m (heading -87.0 deg, configured -90); `g1_right` vetoed (`lidar_obstacle_on_route`), then parked at `g1_rear`: planned reach 0.539 m, map clearance 0.118 m, arrival error 0.027 m, final-leg LiDAR minimum 0.277 m |

## Parking near G1

The 2026-11-08 lane `standard` run chose poses 0.106, 0.079 and 0.017 m (cell-edge
clearance) from G1's mapped legs, and the LiDAR monitor vetoed all three approaches.
The reach band (task point 0.30-0.60 m from G1's axis, planned to 0.54 m), the bin
0.217 m inside the rear edge and the 0.29 m generic keep-out put the rear edge
0.29-0.32 m from G1's axis. Depth puts G1's lower legs within 0.17 m of its axis, so
0.12-0.15 m remains for the monitor's 0.05 m margin, the stopping path and any
relative error between the camera-based G1 estimate and the robot's live map pose.
The camera estimate sat 0.08-0.09 m from the centre of G1's mapped legs, and in
`gz_standard_run6` the vetoing return fell in free space 0.2 m from G1's mapped legs,
at the final pose's rear corner, while the robot was driving: the map pose of the
robot and of G1 disagreed by more than the budget. The monitor was right each time.
The planner now refuses poses within the monitor's margin of mapped obstacles; the
remaining budget is a design question (backlog).

## Limitations

- One run per configuration after each change; search time depends on the explorer.
- The position check was qualified offline on Gazebo views; Isaac ran it live once
  (`isaac_standard`), without ground truth.
- Isaac's completed park is behind G1 after the right side was vetoed; no workstation
  run has parked beside G1.
- The ground-truth track is sampled every 1-2.5 s with a lag, too coarse to place
  scans of a moving robot; the SLAM map was used instead.
