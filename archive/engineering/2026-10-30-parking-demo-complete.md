# Parking demonstration completes on Gazebo and Isaac

Recorded dates: 2026-10-30
Tested revisions: `aa90d45d138fa6ca6788ca67e3bd00ac74de7325`
Provenance: partial

## Conditions

All runs used `start_parking_demo.sh` with its loopback DDS domain 83, the
demo G1 at `(3, 0)` and the table at `(3, 1.05)`, merged SLAM, and the
existing perception environment. They ran from the working tree before
`aa90d45` was committed. The Gazebo run and the first Isaac run predate only
the change of Isaac's `slam_source` default, so the first Isaac run passed
`slam_source:=merged` explicitly; the second used the exact command with no
arguments. Logs, bags and ground truth are in the untracked
`artifacts/exploration/` folders `parking_2026-10-30_14-32-36_gz.in8dDQ`,
`14-35-12_isaac.mdG6H3` and `14-39-42_isaac.khqrgE`, so provenance is partial.

## Diagnosis before the fix

With bin 360 already invalidated, a Gazebo run still vetoed pre-parking. That
was 8.6 s into the leg, with the robot within 0.033 m of the checked route the
whole way. A replay with the mission's code showed the offending pose 0.67 m
behind the robot, on a straight sweep back toward the route's start, and the
return was a real person the route had passed. The mission passed
`driven_poses(...)`, which begins with the robot's own pose, to
`remaining_route`, which then always found the robot "nearest" and treated the
whole route, from the start, as remaining. The isolated Isaac failure before
this change (`front_only`) rejected its routes as `path_footprint_unknown` (9)
and `path_footprint_occupied` (5), the pattern merged SLAM had removed on
Gazebo.

## Results

| Run | Outcome | Pre-parking | Final parking | Return to start |
|---|---|---|---|---|
| Gazebo | `MISSION_COMPLETE`, 96.6 s wall | 0.021 m / 0.109 rad | 0.015 m / 0.004 rad | 0.044 m / 0.383 rad |
| Isaac (explicit merged) | `MISSION_COMPLETE` | 0.045 m / 0.102 rad | 0.028 m / 0.004 rad | 0.027 m / 0.383 rad |
| Isaac (exact command) | `MISSION_COMPLETE` | 0.066 m / 0.118 rad | 0.034 m / 0.005 rad | 0.026 m / 0.391 rad |

Errors are the mission's own map-frame arrival errors. Gazebo ground truth
for that run:

- Parked base `(2.107, -0.074, yaw -3.055)` against the selected
  `(2.159, -0.074, -3.051)`: 0.052 m and 0.004 rad. The SLAM map was offset
  0.054 m from the world.
- Task-point reach to the G1 model origin: 0.616 m, just above the 0.30–0.60 m
  band. The plan targets the localized generic G1 estimate, not the model
  origin.
- Minimum footprint clearance to the G1 origin over the mission: 0.398 m.
- Return to start: 0.104 m from the spawn pose.

Isaac ground truth was not recorded. Both Isaac runs went through every state,
including G1-centred exploration and a verified observation return
(0.016 m / 0.059 rad), and Ctrl+C (SIGINT) shut every process down cleanly.
