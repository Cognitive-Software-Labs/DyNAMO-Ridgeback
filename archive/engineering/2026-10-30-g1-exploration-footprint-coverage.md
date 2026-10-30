# G1-centred exploration: footprint coverage before parking planning

Recorded dates: 2026-10-30
Tested revisions: `0cfb38f548bd4f3b271f6dc7daf7bf56c49e6980`, `dbea5636380e0272271c8d486c45267ee463e8a8`
Provenance: partial

## Question and conditions

Gazebo plans still showed orange rays after G1-centred exploration had
completed. Were otherwise valid poses left undecided because exploration
stopped once the donut task cells were known?

Two consecutive `start_parking_demo.sh gz` runs used loopback DDS domain 83
and Gazebo partition `ridgeback-parking-1010-83`; no other simulator was
running. The first ran the old installed mission manager (`0cfb38f`). The
second ran the rebuilt working tree, which was then committed as `dbea563`
without code changes. Both recorded `/map`, the mission state and the parking
markers. The bags, screenshots and console captures are in the untracked
`artifacts/exploration/parking_2026-10-30_16-15-06_gz.y9B4lB` (old) and
`parking_2026-10-30_16-18-35_gz.Trpwks` (new), so provenance is partial.

## Findings

- The old criterion ended exploration when every donut task cell was known;
  the wider footprint area was only reported. Replaying the
  [display run's](2026-10-30-parking-donut-display.md) recorded maps at that
  stop gave 12–27 positions that keep-out and reach allowed but whose parked
  footprints touched unknown cells, and 78–168 orange rays.
- The new count matched the same count derived from full `plan_parking` runs on
  41 recorded maps. Keep-out and reach depend only on the reach, and that
  reach-only check agreed with every planner keep-out and reach-budget
  decision. It took at most 27 ms per map.

| Run | Exploration end | Duration | Undecided at plan | `footprint_unknown` | Rays green / red / orange / grey | Outcome |
|---|---|---|---|---|---|---|
| Old (`0cfb38f`) | `donut_mapped` | 53.1 s | 7 | 30 | 119 / 128 / 40 / 73 | `MISSION_FAILED`, final-approach `lidar_obstacle_in_stopping_path` (0.137 m) |
| New (`dbea563`) | `positions_decided` (44 undecided at start) | 54.4 s | 0 | 0 | 192 / 102 / 0 / 66 | `MISSION_COMPLETE` |

In the new run, one Nav2 route was rejected (`path_footprint_occupied`), and
the selected pose `(2.560, -0.705, yaw -2.044)` had a 3.346 m path. Parking
and the return completed without a LiDAR veto. One run does not show that the
separate final-approach clearance issue at G1's legs is resolved.
