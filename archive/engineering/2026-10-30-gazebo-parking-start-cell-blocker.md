# Gazebo parking mission after the goal-ownership fix: start-cell blocker

Recorded dates: 2026-10-30
Tested revisions: `710c9d8fee2fe96cdea97a99a38425572eb65f3d`
Provenance: partial

## Purpose and configuration

One controlled full-mission run of `bash start_parking_demo.sh gz` after the
mission manager stopped releasing a `NavigateToPose` goal before its terminal
result. It was meant to confirm that the fix leaves the working mission intact
from detection through `MISSION_COMPLETE`.

The run used the unchanged launcher and scene: Gazebo `depot` (native world
`warehouse`), one spawned G1 at `(3, 0)`, the table at `(3, 1.05)`, CycloneDDS on
loopback domain 83, and the existing perception environment selected with
`RIDGEBACK_PERCEPTION_VENV`. It ran from the working tree of `40691c2` with the
fix uncommitted. The fix was then committed unchanged as the tested revision:
the installed `mission_manager_node.py`, the run's source and that commit's
file share sha256 `1938827a4000ccfb6d540b631e963032d3b854bd749c08679103b7ce91189110`.
The only other uncommitted differences were tests and documentation.

Provenance is partial. The SLAM map was inspected live and not saved, the ROS
logs remain under the untracked `artifacts/exploration/parking_2026-10-30_08-27-19_gz.Zogju4/`,
and no Gazebo ground-truth poses were recorded, because no parking pose was
ever reached.

## Result

The mission ran correctly up to path feasibility and then failed before
driving to any parking pose:

1. Start pose saved at `(0, 0)` in `map`, then
   `SEARCH_G1 -> STOPPING_EXPLORATION -> SETTLING_MOTION -> LOCALIZE_G1`.
2. Generic target estimate `(2.943, 0.003)` from `euclidean_reconstruction`,
   then `G1_LOCALIZED`.
3. `EXPLORING_AROUND_G1` with 266 of 341 donut cells unknown. It completed
   after about 10 s with every donut cell known (`donut_mapped`).
4. The explorer handed off, and the manager's own `NavigateToPose` goal to the
   observation pose `(0, 0, 0)` returned a terminal result after about 31 s.
   That goal was accepted, tracked, released and settled under the new
   ownership lifecycle; no goal was cancelled in this run.
5. Fresh localization gave `(2.954, 0.014)`. The planner produced 338
   donut-free candidates: 32 footprint-free, 167 `footprint_occupied`,
   87 `g1_keepout`, 47 `reach_budget` and 5 `footprint_unknown`.
6. All 18 bounded `ComputePathToPose` checks were rejected with
   `path_footprint_occupied`, each within about 20 ms. The run ended in
   `PARKING_PLANNING_FAILED` (`bounded_search_exhausted`).

Pre-parking, final parking, the wait, return to start, `MISSION_COMPLETE`, reach
and final parking error were therefore not exercised or measured.

## Diagnosis

With the stack still running, a read-only probe read the latched SLAM map
(414 x 346 cells at 0.05 m) and `map -> base_link`. The robot stood at
`(0.312, 0.043)`, yaw -0.315 rad, after the observation return's 0.40 m shared
tolerance. Exactly one occupied cell (value 100) lay at `(0.017, -0.001)`, the
map origin where the robot spawned. All eight neighbouring cells were free.
That cell touches the Nav2 footprint both at the saved start pose and at the
robot's current pose.

`path_rejection()` sweeps every route from the robot's actual pose and also
checks the saved start footprint. `_start_footprint_known()` deliberately
treats only *unknown* cells beneath the start footprint as free, never occupied
ones. Every route was therefore rejected at its first sample, regardless of
candidate. The fast, uniform rejections match this.

This run does not establish where the occupied cell comes from: it could be a
SLAM artifact at the origin or a return the map attributes to the spawn
position. (Later the same day, recorded runs attributed such cells to
Gazebo's corrupted first LiDAR ray; see
[the first-ray record](2026-10-30-gazebo-lidar-first-ray-artifact.md). This
run's own cell remains inferred, not observed.) The run deliberately did not change mapping, footprint, geometry or
feasibility rules to obtain a pass.

## Limits

- One run; recurrence across launches is unknown.
- The cancellation paths themselves (veto, deadline, timeout, reset) were
  verified only by unit tests with fake action handles, not in this
  simulator run.
- The probe script and console log are in the untracked run directory above,
  not in the repository.
