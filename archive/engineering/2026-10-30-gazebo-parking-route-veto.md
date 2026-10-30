# Gazebo pre-parking veto: a LiDAR seam phantom, not a route mismatch

Recorded dates: 2026-10-30
Tested revisions: `31d0d1af169a9c4c5ee327fa84e8dc9fa0a75e86`
Provenance: partial

## Question and conditions

In the complete mission of the
[G1 reacquisition record](2026-10-30-gazebo-g1-reacquisition.md), the route to
the pre-parking pose passed the map-based path check. About 3 s into the leg,
the live parking clearance monitor vetoed it (`lidar_obstacle_on_route`). This
record establishes why the two checks disagreed.

The earlier run's bag did not contain the checked route or Nav2's executed
plans. One reproduction of the same mission (`start_parking_demo.sh gz`,
`depot`, G1 at `(3, 0)`, table at `(3, 1.05)`, merged SLAM, observation return)
therefore also recorded the mission's `parking/visualization` markers,
including the selected route, Nav2's `plan`, MPPI's `optimal_trajectory`, both
costmaps and `cmd_vel_nav`, alongside the map, both scans, TF, odometry and the
mission state. It vetoed in the same way: nearest return 0.435 m from the
footprint, G1 gap 1.448 m, selected pose `(2.394, -0.587, yaw -2.285)`. The
replay used the mission's own `driven_poses`, `remaining_route`, footprint and
`scan_points` code on the recorded data. Bags, ground truth and scripts are in
the untracked `artifacts/exploration/parking_2026-10-30_10-47-21_gz.DHBEV0`.
Provenance is partial because that folder is untracked.

## Findings

1. **Route returned by Nav2 at planning:** the selected-route marker had 85
   points, from the robot to the final pose.
2. **Route accepted by the footprint validator:** swept at the planning
   heading, it kept the footprint 0.155 m from the nearest occupied map cell
   (rejection at about 0.035 m). Against the LiDAR returns at planning time
   it kept 0.198 m, at the G1 near the final pose. It was not grazing a real
   obstacle.
3. **Route Nav2 executed:** four `plan` messages during the leg, deviating at
   most 0.009, 0.021 and 0.031 m from the checked route. There was no
   replanning onto a different route, and no recovery behaviour ran.
4. **Driven trajectory:** within 0.024 m of the checked route. The heading
   drifted steadily, from -0.045 rad at the start to -0.19 rad after about
   1 m, whereas both checks assume the robot keeps its heading until the
   pre-parking rotation.
5. **Obstacle representation:** the planning-time map and the depot ground
   truth are both free around the offending return at `(2.12, -1.22)`. The
   world's static "Person 1 - Standing" at `(1, -1)`, the obstacle near the
   start in earlier maps, is in both, and the route cleared it.
6. **LiDAR returns behind the veto:** the offending point came from front bin
   360 (-45°), the cube-map face seam. It read 1.38–1.48 m in consecutive
   scans while its neighbours read 6.1–6.3 m. In the checking scan it lay
   inside the footprint of remaining route pose #84 `(1.945, -0.785)` at the
   current heading of -0.16 rad. At the planning heading the same pose missed
   it by 0.009–0.019 m.
7. **Margins:** both route checks have zero clearance margin. The validator
   rejects a footprint that touches an occupied cell, and the live monitor
   rejects any return inside a remaining route footprint. The live monitor's
   0.05 m margin applies only to the stopping path, which did not trigger. No
   margin is applied twice.

Bin 360 is a systematic phantom source on both Gazebo LiDARs. Counting a
finite reading more than 0.3 m shorter than two neighbours that agree within
0.1 m:

| Run | Front bin 360 | Rear bin 360 | Any other bin, mean (max) |
|---|---|---|---|
| Reproduction | 374 / 2581 (14.5%) | 383 / 2580 (14.8%) | 0.08 (11) / 0.44 (265) |
| Previous mission (`ceb8d9a`) | 430 / 2994 (14.4%) | 768 / 2993 (25.7%) | 0.08 (14) / 0.21 (139) |
| Earlier mission (`aa5eb2f`) | 440 / 4246 (10.4%) | 613 / 4246 (14.4%) | 0.07 (10) / 0.07 (10) |

Bin 720, the +45° seam, reads `+inf` often but showed no such short phantoms.
The rear LiDAR's bins 738 and 27 had a few hundred such readings in one run
each; they were not examined.

In the previous mission, in the 0.3 s before its veto, bin 360 read 2.1 m on
the front LiDAR (neighbours 6.3 m) and 2.5–2.7 m on the rear (neighbours
`+inf`). That run recorded no route, so its veto is consistent with the same
phantom, not proven.

## Conclusion

The checked route was safe against every real obstacle, and Nav2 followed it.
The live monitor correctly treats every return as real, and vetoed because a
Gazebo seam phantom fell inside the remaining route. The smallest supported
correction would invalidate bin 360 in the existing Gazebo-only first-ray
guard, as it does bin 0. On 2026-10-30 that was decided against, because LiDAR
preprocessing was outside the change's scope. No code was changed, and the
Gazebo parking mission remains blocked by this artifact.

The heading drift in finding 4 is a real difference between the validators'
held-heading model and MPPI's execution. It did not cause this veto: a
0.15 rad drift moves the footprint's corners about 0.1 m, less than the
0.155 m clearance to the nearest real obstacle.
