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
| `SEARCH_G1` | The frontier explorer searches; detections are voted | G1 confirmed (3 of 5 frames at 0.50) | none in the mission; the experiment runner stops a run at 900-1800 s | 155 (8-946), 18 visits | 524 (149-898), 2 visits |
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

### Planning timeout

A route search that outlives the 10 s batch deadline or a 3 s request deadline
(`planner_timeout`) says nothing about whether a parking pose exists, so it neither
explores around G1 nor reads as `no_valid_candidate`; a spent route-check budget and a
search with no safe route stay `bounded_search_exhausted`, and missing post-planning
motion evidence stays `motion_settlement_timeout`. With driving on, the outstanding
check is cancelled and the mission enters `RETURNING_TO_START` without a goal. Once the
stop is confirmed, it starts the shared-checker return to the start only when the scans
and odometry are fresh with nothing within the 0.05 m stopping margin, both Nav2 action
servers are ready, and Nav2's planner returns a route home within 3 s; the return is
then watched by Nav2 and its collision monitor like any other. Otherwise nothing moves:
the mission ends `MISSION_FAILED` (`planning_timeout`) with the reason, for example
`stop not confirmed`, `lidar_stale` or `home_route_unavailable`, and is not retried. A
completed return also ends `MISSION_FAILED` (`planning_timeout`), and a failed one
`return_navigation_failed` naming the timeout; `timeout_recovery` in the mission state
records the outcome. With driving off, nothing moves and the failure stays
`PARKING_PLANNING_FAILED` (`planner_timeout`).

After the wait, the robot leaves G1 before the return: straight out to the plan's
pre-parking pose, 0.5 m directly away from G1, with the parking legs' 0.08 m / 0.12
rad goal checker. It starts only on current stop evidence, with the swept planning
footprint (basket included) free in the current map and never nearer G1, and with
fresh scans and odometry showing nothing in the stopping path or on the route. The
parking legs' LiDAR check then runs on every scan and tick until the pose is reached
(`departure` in the clearance legs). Nav2's default behavior tree replans at 1 Hz and
may spin or back up, so the robot must also stay within the goal checker's 0.08 m and
0.12 rad of the checked straight route (`route_deviation`). If the departure cannot
start, the mission ends `MISSION_FAILED` (`departure_not_safe`) with the robot
stopped. A veto, leaving the route, a stale sensor, a Nav2 failure or the deadline
cancels it; once Nav2 releases the goal and the stop is confirmed, the mission ends
`MISSION_FAILED` (`departure_failed`) without another goal near G1
(`stop_not_confirmed` if no stop is confirmed). Reaching the pose starts the unchanged
return. A failed or vetoed final leg leaves the robot near G1, so once the stop is
confirmed the same departure runs first, in `RETURNING_TO_OBSERVATION` before a retry
or in `RETURNING_TO_START` before the return; a failure on the pre-parking leg goes
straight to the retry or return.

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
