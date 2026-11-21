# Parking mission states and timing

The mission manager (`mission_manager_node.py`) drives the parking mission as one
state machine; [exploration architecture](architecture.md#parking-mission-coordination)
explains its components. This page lists every state with its purpose, how it is
left, the configured limit that bounds it, and how long it took in recorded runs.

## How the times were measured

`tools/parking_scenarios/state_timing.py RUN_DIR...` reads each run's console log:
every `Mission state transition: A -> B` line carries the ROS logger's wall-clock
stamp, and a state lasts from the transition into it to the transition out of it. A
state still active when a run stopped is not counted. Gazebo figures come from the 20
workstation-scene runs of 2026-11-05 to 2026-11-09 (G1 out of the robot's sight);
Isaac figures from all 7 Isaac runs of 2026-11-05 to 2026-11-09, because only 3 used
the workstation scene. Times are medians with the observed range, in seconds; one or
two visits are reported as such. Configured limits are parameter defaults of the
mission manager unless noted.

## States

| State | Purpose | Left when | Configured limit | Gazebo, measured | Isaac, measured |
|---|---|---|---|---|---|
| `WAITING_FOR_START_POSE` | Save the start pose before anything moves | a valid map-to-base transform | startup gates, `startup_timeout_s` 300 s each (launch) | not logged per state | not logged per state |
| `SEARCH_G1` | The frontier explorer searches, turning the camera at frontier stops; detections are voted | G1 confirmed (3 of 5 frames at 0.50), or the search ends without G1 (below) | `search_exhausted_confirm_s` 10 s of `exploration_complete`; the experiment runner stops a run at 900-1800 s | 155 (8-946), 18 visits | 524 (149-898), 2 visits |
| `STOPPING_EXPLORATION` | Take Nav2 ownership from the explorer | the explorer's pause is confirmed | `handoff_timeout_s` 10 s | 0.0 | 0.0 |
| `RETURNING_TO_OBSERVATION` | Turn toward G1, step back (also once before planning when the robot stands inside G1's keep-out or G1's heading is unknown from closer than 1.5 m; that step is LiDAR-checked like the parking legs), or return to the observation pose before a (re)localization | goal reached, then motion settles; for the step before planning also a veto, abort or deadline, then a confirmed stop | `parking_navigation_timeout_s` 90 s; `g1_reobservation_timeout_s` 60 s for the step before planning | 4.0 (0.1-93.8), 17 visits | 5.5 (5.4-8.1), 3 visits |
| `SETTLING_MOTION` | Wait for 2 s of stationary odometry | the robot is still | `motion_settlement_timeout_s` 10 s | 3.1 (2.7-4.0) | 5.3 (5.2-5.5) |
| `LOCALIZE_G1` | Five consistent G1 samples, then the depth check of head and torso | a verified position | `target_localization_timeout_s` 5 s | 0.6 (0.5-5.2) | 1.0 (0.7-5.0) |
| `G1_LOCALIZED` | Plan parking from the map, or decide to map more | candidates, exploration or failure | `parking_input_timeout_s` 10 s for missing inputs | 0.2 (0.1-0.3) | 0.2 (0.1-0.2) |
| `EXPLORING_AROUND_G1` | Map the unknown floor around G1 that decides parking poses | positions decided, a checkpoint or the limit | `g1_exploration_timeout_s` 240 s in total | 10.9 (1.1-23.6), 7 visits | 32.2, 1 visit |
| `PARKING_CANDIDATES_READY` | Wait for fresh stationary evidence after planning | the robot is still | `motion_settlement_timeout_s` 10 s | 0.0 (0.0-2.6) | 0.0 |
| `EVALUATING_PARKING_PATHS` | Nav2 route checks of the ranked poses, one at a time | a safe route, or none | `parking_path_timeout_s` 3 s per route, 10 s per batch | 2.3 (0.8-3.2) | 1.1 (0.8-1.4) |
| `PARKING_PATH_FEASIBLE` | Revalidate the plan and wait for stationary evidence before driving | driving starts, or the plan is invalid | target age 5 s (renewed by views of G1), stationary evidence within 10 s | 2.4 (0.0-3.1) | 0.0 |
| `NAVIGATING_TO_PREPARKING` | Drive to the pose 0.5 m before the parking pose, LiDAR checked | arrival, veto or failure | `parking_navigation_timeout_s` 90 s, `parking_progress_timeout_s` 20 s | 14.0 (2.6-18.2), 5 visits | 9.8 (2.2-17.3), 2 visits |
| `NAVIGATING_TO_PARKING` | The final, tight-tolerance leg, LiDAR checked | arrival, veto or failure | as above | 2.7 (2.4-3.0), 2 visits | 6.2, 1 visit |
| `PARKED` | Confirm the stop, then wait; then check the departure | the departure starts, or `MISSION_FAILED` (`departure_not_safe`) with the robot stopped | `parking_wait_s` 5 s after the stop is confirmed; fresh stop evidence then, within `motion_settlement_timeout_s` | 8.6, 1 visit | 17.3, 1 visit |
| `RETURNING_TO_START` | Depart (after parking or a failed final leg), then drive back to the saved start pose | arrival or failure | departure `parking_navigation_timeout_s` 90 s; return `return_navigation_timeout_s` 300 s | 17.8 (3.0-77.3), 6 visits | 51.2, 1 visit |
| `MISSION_COMPLETE` | Parked, waited and returned | terminal | - | - | - |
| `MISSION_FAILED`, `PARKING_PLANNING_FAILED`, `PARKING_EXECUTION_FAILED`, `MOTION_SETTLEMENT_FAILED`, `MISSION_CANCELED` | Terminal failure or cancellation, with a reason | terminal | - | - | - |

A failed or vetoed parking leg goes to `RETURNING_TO_OBSERVATION` for the next of at
most 3 attempts; a failure that cannot be retried, or the last attempt, goes to
`RETURNING_TO_START` and ends `MISSION_FAILED` with the reason. A localization that
times out after a turn falls back once to the observation pose. A failure while Nav2
may still own a goal stops the robot in place (`MISSION_FAILED`) instead of starting
another goal.

### Stop evidence

Every wait for a stop needs 2 s of continuous odometry below 0.005 m/s and 0.005 rad/s,
counted in both the odometry stamps (simulated time in a simulator) and wall time. One
sample over a limit, an odometry stamp more than 0.1 s ahead of the manager's clock, a
stale sample, or a gap of more than 0.5 s restarts the interval. Motion, an invalid
frame or value, or a stamp that does not advance also counts as a motion fault. After
planning, a fault discards the plan; other restarts may recur only until
`motion_settlement_timeout_s` (10 s) has passed without a completed interval. Such a
lost plan says which: `motion_evidence_lost: motion fault ...` or `motion_evidence_lost:
restart allowance of 10.0 s expired without motion; ...`. Diagnostics do not affect
these decisions. `motion_evidence` in the mission state counts this wait's restarts by
cause (`callback_gap` and `odometry_gap` included), and gives the latest restart and
fault with the odometry stamp, the manager's clock and their difference, and the range
of that difference. While the mission waits for a stop, the log names the latest
restart at most once every 2 s.

### Incomplete route search

A route search cut short, by the 10 s batch deadline or a 3 s request deadline
(`planner_timeout`, reported as `planning_timeout`) or by the 16-check route budget
with poses still untested (`route_budget_exhausted`), says nothing about whether a
parking pose exists, so neither reads as `no_valid_candidate`, and each keeps its own
code. A search that checked every valid pose and found no safe route stays
`bounded_search_exhausted` and, like `no_valid_candidate` and `no_free_donut_cells`,
may map around G1 first; since 2026-11-21 all three then take the same guarded return
below under their own code (a completed return formerly reported `parking_failed`).
Missing post-planning motion evidence stays `motion_settlement_timeout`. A spent budget may
still map around G1 first, as before; a timeout does not. With driving on, the
outstanding check is cancelled and the mission enters `RETURNING_TO_START` without a
goal. Once the stop is confirmed, it starts the shared-checker return to the start
only when the scans and odometry are fresh with nothing within the 0.05 m stopping
margin, both Nav2 action servers are ready, the footprint is outside G1's protected
region (keep-out plus `g1_exploration_protected_margin_m`; otherwise
`g1_protected_region`, since only the checked departure may leave it), and Nav2's
planner returns a route home within 3 s. That route only gates the start: Nav2 replans the return on its live
costmaps and every command passes its collision monitor. Otherwise nothing moves: the
mission ends `MISSION_FAILED` under the search's code with the reason, for example
`stop not confirmed`, `lidar_stale` or `home_route_unavailable`, and is not retried. A
completed return also ends `MISSION_FAILED` under that code, and a failed one
`return_navigation_failed` naming it; `search_recovery` in the mission state records
the code and outcome. With driving off, nothing moves and the failure stays
`PARKING_PLANNING_FAILED`.

### Camera observation at frontier stops

Frontiers follow LiDAR coverage, which says nothing about where the 80 degree camera
has looked. For the parking mission only (`camera_observation_enabled`, set by
`ridgeback_mission.launch.py`; off for every other explorer user), the explorer
keeps a coverage grid on the SLAM map (`frontier_explorer/camera_coverage.py`). A
detector frame (the detector publishes one per processed image) marks the map-known
cells within 30 degrees of the camera axis and 1.3-3.5 m away with an unbroken line of
sight; a ray stops at the first occupied cell, which is seen, and at unknown cells.
Frames taken while the camera turns faster than 0.35 rad/s, or without a camera pose
at their stamp, mark nothing.

After a successful frontier goal (not while exploring around a localized G1), the
explorer confirms a 0.5 s stop on the mission's odometry, then picks at most three
headings, each adding at least 1 m^2 of unseen visible cells, and turns to them in one
signed sweep or one crossing (`frontier_explorer/camera_observation.py`). Before each
turn the swept footprint, basket included, must keep 0.05 m from lethal cells of the
global costmap and stay on known cells. The turn is a Nav2 `Spin` (15 s allowance),
which checks the local costmap itself and whose command passes the collision monitor;
MissionGuard owns it exactly like a frontier goal, so the mission's pause on a G1
confirmation cancels it. At each heading the explorer waits up to 3 s for five frames
taken there after the turn. A refused, rejected, aborted, cancelled or overdue turn
ends that stop's observations, and a view without those frames is not counted and not
retried from within 1 m. All observations share a 180 s budget. While a stop is
observed, no frontier is chosen and `exploration_complete` is not reported.

### Search without G1

The explorer repeats `exploration_complete` every tick while it has no frontier, and
never while a stop's observation turns are pending. Once that report has held for
`search_exhausted_confirm_s` (10 s, gaps of at most 2 s) and no detection vote is
pending, the mission pauses the explorer through the usual handoff
(`STOPPING_EXPLORATION`), then fails as `g1_not_found` through the same guarded return:
home only once stopped with fresh sensors and a route home, otherwise stopped where it
is. It never reads as `MISSION_COMPLETE`. With driving off it ends `MISSION_FAILED`
stopped.

## Whole missions

Search dominates. Where G1 stood in view of the start (2026-11-05), the robot
confirmed it within 3-7 s and completed missions took 78-269 s in both simulators.
With G1 at a workstation out of sight, search took 8-946 s in Gazebo (median 155 s)
and 149 and 898 s in Isaac; the completed Isaac workstation mission on 2026-11-09
took 1065 s, 909 s of it before G1 was localized. Once G1 is verified, planning takes
2-6 s, and parking, the wait and the return take about 1-1.5 min.

Two states vary widely for a reason recorded in the runs: `RETURNING_TO_OBSERVATION`
reached 94 s when the controller stalled on a 4.3 m return to a distant observation
pose (2026-11-09; the robot now turns or steps back near G1, see
[observation return](architecture.md#observation-return)), and `RETURNING_TO_START`
is a long drive whenever G1 is far from the start.
