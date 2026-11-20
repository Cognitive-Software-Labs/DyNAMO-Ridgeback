# Departure from G1: first live Isaac run

Recorded dates: 2026-11-20
Tested revisions: `e54007651bf9a01b0aa03bd0e41d9f20b6e2281b`
Provenance: partial

## Conditions

One Isaac hospital-ward `standard` run (`parking_demo`, G1 at (0.5, -4.5) facing
-90 deg) on `e540076`: the LiDAR-checked departure after the parked wait, the 0.08 m /
0.12 rad route corridor, and the Nav2 outline enclosing the provisional basket by
5.1 mm. It ran through `tools/parking_scenarios/run_scenario.sh` (headless, compact
RViz, camera frames once a second) with a passive recorder of TF, filtered odometry and
`cmd_vel`; no other simulator ran on the host. The tree's uncommitted
`start_parking_demo.sh` option and untracked plan documents were present and unused.
Evidence is in the untracked `artifacts/exploration/departure_2026-11-20/isaac_standard/`.
Provenance is partial: Isaac has no ground-truth track, the route deviation is measured
from the robot's own localization, and the scene had no moving obstacle.

## Result

- `MISSION_COMPLETE`. Parked on the first attempt on `g1_left` (valid poses: left 2,
  rear 8, other 3; both left routes safe), 0.031 m and 0.002 rad from the selected pose.
- The stop was confirmed 4.6 s after arrival and the 5 s wait completed. Fresh stop
  evidence for the departure came 6 s after the wait, inside its 10 s allowance: the
  filtered odometry of the stationary robot reported 0.005-0.009 rad/s, just above the
  0.005 rad/s settlement limit, and each such sample restarted the 2 s interval. No
  velocity command was sent meanwhile.
- The departure drove 0.50 m straight to the pre-parking pose in 4.8 s (up to 0.30 m/s
  forward, at most 0.018 m/s sideways, never reversing) and ended 0.078 m short, within
  the 0.08 m goal tolerance.
- The mission's LiDAR check ran 181 times during it with no veto; it vetoes on a stale
  scan from either LiDAR, so both stayed fresh. Nearest return 0.289 m from the
  footprint (basket included); G1 gap at least 0.32 m.
- Largest deviation from the checked straight route: 0.014 m and 0.010 rad, against the
  corridor's 0.08 m and 0.12 rad.
- Nav2 ran no recovery (spin, back-up, wait or costmap clearing) during the mission;
  the collision monitor's approach stops occurred only during the G1 search.
- The return ended 0.40 m from the start, accepted by the shared goal checker.

## Limits

One run. It did not exercise the departure after a failed final approach or a corridor
cancellation; automated tests alone cover them. The odometry fluctuations above can use
up the 10 s allowance, which ends the mission `departure_not_safe` with the robot
stopped beside G1. Moving obstacles and the real robot were not tested.
