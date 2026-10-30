# Parking donut display validation and a final-approach veto on G1

Recorded dates: 2026-10-30
Tested revisions: `0e5cf4280fa9d0757307f772e6b0944be0a119f7`
Provenance: partial

## Conditions

One `start_parking_demo.sh gz` run (loopback DDS domain 83, Gazebo partition
`ridgeback-parking-1010-83`), started after a teammate's separate simulation on
the same workstation had ended. It ran from the working tree before `0e5cf42`
was committed; only the plan's open-issue note changed afterwards. The
screenshots, bag (raw and guarded scans, merged scan, TF, map, markers,
legend, mission state) and console are in the untracked
`artifacts/exploration/parking_2026-10-30_15-45-59_gz.KHiGFH`.

## Display

Before the change, a recorded Gazebo plan's rays were 97% grey and 3% green,
with no red or orange, while 343 candidate dots carried the rejections. In
live RViz the donut now shows red where poses are blocked by the table legs
or the G1's mapped legs, orange where footprints reach unmapped floor,
grey for keep-out and reach failures, and green for valid poses. No dots are
drawn, the legend has no dot entries, the selected pose, footprint and
validated route are drawn in blue, and the P6 state machine updated
throughout. A re-plan from an earlier recorded map gave 112 green, 326 red,
115 orange and 522 grey ray segments.

## Mission

Detection, both localizations, G1-centred exploration, planning (selected
`(2.418, -0.571, yaw -2.336)`) and pre-parking (0.031 m) succeeded. During the
final reverse toward G1, the live monitor vetoed `lidar_obstacle_in_stopping_path`
(nearest return 0.099 m from the footprint, G1 gap 0.334 m). The return came
from rear bin 462 at `(2.87, -0.22)`, 0.25 m from the G1 estimate, and its
neighbours read a consistent 0.22–0.36 m surface: the G1's real leg. The
robot returned to the start (0.023 m) and the mission ended `MISSION_FAILED`.
The veto is correct. The keep-out is measured from the generic G1 estimate,
so a pose that passes it can still bring the footprint within the 0.05 m
stopping margin of G1's body.

## LiDAR

Both raw `scan_gz` streams, both guarded scans and the merged scan delivered
3840–3842 messages at 23.8 Hz (wall time), with no simulation-time gap above
0.027 s and no delivery gap above 0.10 s after startup. Bins 0 and 360 were
NaN in every guarded scan and never in the raw scans.
