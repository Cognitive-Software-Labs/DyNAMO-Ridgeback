# Parking in the hospital ward: both simulators on one world

Recorded dates: 2026-11-17, 2026-11-18
Tested revisions: `9400c10036f10b660f26ed451d3b4b3df0473c15`, `5b5133cff7a2152d04d28f2526cf2b0d53fb0df2`, `f5687890a4cb91aeff836f241d49e2b73418ae17`, `f6ade2f7e8b58e844eef6b05b8c053135120d112`, `b519ab01214b447f0df6f1eef315aa522a872ce5`, `544ead1998dae8dd7ee786718afa96e79cdb78a5`, `0890e137b0b78dc6c553e0bde47d0f4b6fbaa90c`
Provenance: partial

## Question and conditions

Can both simulators run the parking demonstration in the same hospital ward
(`parking_demo`: `initial_test_world` without its authored distant G1) while the
Gazebo warehouse stays available with `world:=depot`? The 2026-11-17 Gazebo trial
in the ward (the hospital-ward record) parked in `standard` but, in `no_feasible`,
localized G1 0.57 m off and attempted a pose that only the live LiDAR check stopped.
This record covers that cause, a navigation stall the safety scene caused, and the
runs on the corrected code.

Commit `c44dac4` keyed the workstations by world (schema 3) and made the ward the
default in both simulators; the Gazebo SDF and Isaac's USD have identical collision
boxes (largest difference below 1e-7 m). Mission, planning, navigation and safety
parameters were not changed: reach band 0.30-0.60 m, side priorities, the
basket-aware outline, Nav2 and the LiDAR check, retry limits, cancellation,
MissionGuard and the return.

Runs used `tools/parking_scenarios/run_scenario.sh` (loopback DDS domain 83, one
simulator on the host, no other user's simulator running, `rviz_layout:=compact`).
Evidence is in the untracked `artifacts/exploration/hospital_2026-11-18/`, one folder
per run with `commit.txt`, the console, a small bag (states, map, plans, detections,
both scans; no depth), saved camera frames and screenshots. The folder name says
2026-11-18, but every run in it except `gz_standard_final` and `gz_no_feasible_final`
was recorded on 2026-11-17 by the host clock (CET). `gz_no_feasible_final` ran while
another user's single headless Isaac job ran (load average about 10-31); its
uncommitted changes were page tools and documentation only. The tree's uncommitted documentation and its
`--parallel` option in `start_parking_demo.sh` were present and unused. Gazebo
errors are against ground truth sampled every 1-2.5 s and used only offline;
Isaac runs have no ground-truth track.

Provenance is partial: the first Gazebo `standard` run used a stale installed
mission node (below), and depth frames were not recorded, so the views below were
reproduced at the recorded robot poses rather than replayed.

## Why G1 was localized 0.57 m off

The head check of the 2026-11-09 G1 position-check record takes the 20th-percentile
range in the top 25 % and middle 40 % of the detection box as G1's head and
re-seeds the position there when the surface estimate disagrees. At the trial's
robot pose (`g1_views/nf_recorded`, reproduced in Gazebo at the logged pose), the
near panel hid most of G1 and the detector boxed only its visible part
(`[310, 233, 344, 329]`). The head patch was entirely the far panel, 4.09-4.22 m
away; the re-seed moved G1 onto that panel, and the torso check passed on 1648 panel
points at torso height. Clear views of the pen from in front and behind
(`nf_front`, `nf_behind`) localized G1 correctly.

`9400c10` refuses a position with 20 or more depth points at 1.45-2.20 m in three
frames, since G1 stands at most 1.33 m: the far panel showed 443-505 such points,
and no recorded view showed any on G1. That alone turned the bad estimate into a
refusal, but it also refused good views. In Gazebo `standard` (`gz_standard_run2`)
and Isaac `standard` (`isaac_standard`), G1 seen close and obliquely with its arms
forward had its head off the box's middle columns; the head region measured the
wall 4.7-10.3 m behind G1, and the re-seed onto the wall was refused. In Isaac this
repeated five times until the localization deadline.

`b519ab0` therefore keeps the surface estimate when the re-seed position shows
something taller than G1, and refuses the view when G1's legs place its axis more
than 0.25 m from that estimate. On the reproduced views the half-hidden G1's legs
lay 0.32 m from the estimate on its hands (refused), and the close Gazebo view
(`g1_views_standard/std_close`, legs outside the image) was localized 0.07 m from
G1. Both views are test fixtures (`g1_views_2026-11-17.npz`).

## The robot wedged beside the no_feasible bed

`gz_no_feasible` (`f568789`, the rebuilt mission node) never saw G1. The pen's bed,
then 1.25 m behind G1, left a 1.10 m strip to the corridor wall. The navigation
footprint is 0.80 m wide but 1.05 m across its diagonal; the robot reached the
strip's west end at (-0.44, -2.23) m turned 58 degrees from it, with 0.03 m to spare
on each side (below the 0.05 m cell) and 0.005 m to turn in place. Explorer goals in
the north-east room were routed through the strip; each was blacklisted after three
60 s stalls, and the next took the same route. The run was stopped by the operator
at the same pose. `f6ade2f` moved the bed to 0.95 m behind G1: the strip is 1.40 m,
and the gaps from the bed to the ward's exam table and chair (0.60 and 0.78 m) are
too narrow to enter. Rear parking stays blocked. Navigation was not changed; the
backlog item on orientation-aware navigation through narrow passages records the
gap.

## Runs

| Run | Mission code | Outcome | Detail |
|---|---|---|---|
| `gz_standard` | 9400c10 world; the installed mission node was 5b5133c's (no taller-than-G1 check) | `MISSION_FAILED`: PARKED deadline expired, robot parked | G1 0.074-0.104 m off, heading error 6.0 deg; `g1_right` and `g1_left` cancelled by the LiDAR check in the stopping path; `g1_rear` parked on attempt 3, reach 0.487 m, basket 0.097 m from G1's lower body. Three Gazebo view cameras created on arrival stalled the server (real-time factor 0.33-0.52 at load 47-58) and the motion settle missed its 10 s limit; `f568789` takes the parked views after the mission confirms the stop. Returned 0.24 m from the start |
| `gz_no_feasible` | f568789 | stopped by the operator | wedged at the bed strip, above |
| `gz_standard_run2` | f6ade2f | `MISSION_COMPLETE` in 178 s | first localization refused (head region on the wall 5.25 m away, 1419 points above G1); relocalized from 0.68 m; G1 0.053 m off; heading estimate -155.1 deg, error 65.1 deg; attempt 1 `g1_right`, ground-truth side `g1_right` at -119 deg from G1's heading; reach 0.607 m, 7 mm over the band; basket 0.217 m from G1's lower body, nearest edge 0.446 m from G1's axis; base 0.08 m from G1's lower body during the approach; returned 0.24 m from the start |
| `gz_no_feasible_run2` | f6ade2f | `MISSION_FAILED`, `no_valid_candidate`, as intended | G1 0.125 m off, heading error 2.2 deg; 0 of 255 poses valid (`footprint_occupied`), no attempt; nearest approach 0.60 m (base to worktable); returned 0.51 m from the start; 943 s, two 60 s explorer stalls at the east doorway, none at the bed |
| `isaac_standard` | f6ade2f | `MISSION_FAILED`: localization timeout | the head region measured the south wall 10.25 m away from about 6.2 m; the wall re-seed was refused five times |
| `isaac_standard_run2` | b519ab0 | `MISSION_COMPLETE` in 745 s | the head region at 10.65 m showed 242 points above G1: surface estimate kept, legs confirmed it (0.612, -4.553); relocalized at (0.605, -4.543), heading -86.8 deg; attempt 1 `g1_right`, arrival 0.020 m, LiDAR minimum 0.288 m; parked, five-second wait, returned. The estimates are 0.11-0.12 m from G1's configured pose, which includes the map's drift from the world frame |
| `gz_standard_final` (2026-11-18) | 544ead1 (b519ab0's mission code; installed executables compared with the source before the run) | `MISSION_COMPLETE` in 696 s | search 553 s, no explorer stall; three localizations (from 4.4, 2.3 and 1.4 m), each accepted on its first try: G1 0.073-0.084 m off, headings -92.1, -96.0 and -83.8 deg (errors 2.1-6.2 deg); attempt 1 `g1_right` cancelled by the LiDAR check in the stopping path (nearest return 0.151 m, 0.416 m from G1); attempt 2 `g1_left` parked: reach 0.509 m, basket 0.118 m from G1's lower body, base 0.37 m from the worktable, LiDAR minimum 0.109 m; five-second wait completed; returned 0.22 m from the start |
| `gz_no_feasible_final` (2026-11-18) | 0890e13 (b519ab0's mission code) | `MISSION_FAILED`, `no_valid_candidate`, as intended | search 635 s; G1 0.152 m off, heading error 10.0 deg; the detector's surface estimate lay 0.93 m beyond G1's head, on the worktable, and the head check moved the position onto G1's head, where its legs confirmed the axis (3320 torso points, none above G1's height); 0 of 258 poses valid (`footprint_occupied`), no attempt; nearest approach 0.67 m (base to worktable); returned 0.35 m from the start |

Screenshots per run are in each `shots/` folder: RViz at the mission stages and
server-rendered views (top-down workstation, close-up, angled) taken while parked
after the stop was confirmed, and at the end. `gz_standard`'s parked views come from
the failed run and are not a success.

## Heading and reach in Gazebo `standard`

The planner ranks a pose by the bearing of its rear-bin task point (0.28 m behind
the base) from G1, relative to G1's estimated heading: within 45 deg of G1's left or
right first, then within 30 deg of behind, then in front, then the rest. Bearings
below are from the planned map pose and G1's estimate, and from ground truth.

| Run, attempt | Estimated heading | Bearing planned | Bearing, ground truth | Reach planned / parked (ground truth) |
|---|---|---|---|---|
| `gz_standard_run2`, 1 | -155.1 deg | -55 deg (`g1_right`) | -118 deg (`g1_right`) | 0.517 / 0.607 m |
| `gz_standard_final`, 1 (vetoed) | -96.0 deg | -129 deg (`g1_right`) | -136 deg | 0.511 m / not parked |
| `gz_standard_final`, 2 | -83.8 deg | 134 deg (`g1_left`) | 137 deg | 0.530 / 0.509 m |

In `gz_standard_run2` the 65 deg heading error did not change the side label, but
it moved the chosen pose: the planner placed the bin at G1's front right, where its
forearms work, and it stood behind G1's right shoulder, its nearest edge 0.45 m from
G1's axis. The parked reach of 0.607 m is outside the 0.30-0.60 m band: the plan
measured 0.517 m from G1's estimate, and the 0.053 m localization error and the
robot's 0.048 m map-pose error added the rest. A mission end of
`MISSION_COMPLETE` does not show the band was met; this run does not satisfy it.

That heading came from a view 0.68 m from G1's head (14589 depth points on the
forearm band, against 41-506 in every other view). Two views reproduced near that
robot pose on 2026-11-18 (`g1_views_heading/`, plan-only Gazebo, which overlapped
another user's headless Isaac job; the robot settled 0.1 m and 12 deg from the
requested pose) gave no heading at all, so the estimate was not reproduced
and no change was made. In `gz_standard_final` no view was closer than 1.4 m and all
three headings were within 6.2 deg. Attempt 2 parked with a ground-truth bearing of
137 deg, 2 deg past the 45 deg left sector, behind G1's left hip; the planned bearing
was 134 deg. Sectors only order the checks, so this is a ranking difference, not a
safety one.

## Outcome

Both simulators load the same ward, and on the final code (`b519ab0`'s mission code)
each completed `standard` once: Isaac parked at G1's right on attempt 1, Gazebo at
G1's left-rear on attempt 2, reach 0.509 m, G1 0.073-0.084 m and 6 deg off. Gazebo
`no_feasible` refused correctly on `f6ade2f` and again on the final code
(`gz_no_feasible_final`, G1 0.152 m off): the pen leaves about 0.48 m beside G1's
keep-out at each panel and 0.21 m behind it, against a 0.80 m wide footprint, so
that error cannot make a pose fit. The
earlier Gazebo `standard` completion on `f6ade2f` exceeded the reach band (0.607 m)
after a 65 deg heading error. One run per scene shows behaviour, not a rate.

Remaining limits: the head region misses G1's head in oblique, close and partly
hidden views, and the surface fallback then depends on the legs agreeing; the
65 deg heading from a 0.68 m view is unexplained; the LiDAR check still vetoes many
Gazebo approaches beside G1; Gazebo's localization error in the ward (0.05-0.15 m)
is larger than in the warehouse (0.015-0.041 m on 2026-11-14); the ward's own exam
table and chair stand 0.83 m from G1 in every scene; and narrow passages between
0.80 and 1.05 m can still wedge the robot. The mission's "Target position
stabilized" log line prints the raw stabilized sample, not the position the depth
check verified and planning uses (in `gz_no_feasible_final` the line shows the
surface estimate on the worktable); `g1_estimates` in the metrics and the G1 marker
in RViz show the verified position.
