# Parking donut display: one colour per complete ray

Recorded dates: 2026-10-30
Tested revisions: `52c175ba6711d841a0482f6f5a04cfbad4581b10`
Provenance: partial

## Conditions

The [earlier display](2026-10-30-parking-donut-display.md) split each 1° ray
into segments by the cell it crossed. Each ray is now one class over its
complete 0.30–0.60 m length. Code and tests were final when this run started.
Only the commit followed. The run was one `start_parking_demo.sh gz` run (loopback
DDS domain 83, Gazebo partition `ridgeback-parking-1010-83`). No other
simulator was running on the workstation. Screenshots and the console capture
are in the untracked
`artifacts/exploration/parking_2026-10-30_16-01-04_gz.rzidzB`. No bag was
recorded, so provenance is partial.

## Findings

- A re-plan from the [earlier run's](2026-10-30-parking-donut-display.md)
  recorded map and G1 estimate drew exactly 360 segments, each from 0.30 to
  0.60 m. They were 93 green, 107 red, 88 orange and 72 grey.
- Grey sectors are real planner results. With the live 0.06 m reach
  allowance, keep-out and reach leave task points only about 0.51–0.54 m from
  the generic G1 estimate, narrower than a 0.05 m cell. Some directions cross
  no allowed cell, for example east of G1, where the cells lie at 0.497 m
  (keep-out) and 0.546 m (reach).
- Live RViz showed solid red, grey, green and orange sectors, no dots, the
  whole-ray legend, and the blue selected pose, footprint and route.
- Mission: the plan selected `(2.473, -0.591, yaw -2.243)` with a 3.093 m
  path. Pre-parking was reached. On the final reverse, the live clearance
  monitor vetoed `lidar_obstacle_in_stopping_path`: the nearest return was
  0.131 m from the footprint, and the G1 gap was 0.365 m. The robot returned
  to the start, and the mission ended `MISSION_FAILED`. This matches the
  earlier veto on G1's leg, but without a bag the return was not attributed.
