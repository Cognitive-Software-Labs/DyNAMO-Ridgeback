# Gazebo G1 reacquisition after the observation return

Recorded dates: 2026-10-30
Tested revisions: `a7c84a055c084db6346230e3c55622da1e40fcc9`, `ceb8d9a681143e5c06cafff52d2fd29bee71f1cc`
Provenance: partial

## Question and conditions

Two of five earlier full Gazebo runs failed the second G1 localization after
the observation return. In those runs the detector published no G1 for the
whole 5 s window
([merged-SLAM record](2026-10-30-gazebo-parking-merged-slam.md)). The robot
had stopped about 18° off the observation yaw, inside the shared 0.40 rad goal
checker. This record establishes why detection failed and validates the
correction.

All runs used `start_parking_demo.sh gz` on `depot` (native `warehouse`), with
the demo G1 at `(3, 0)` and the table at `(3, 1.05)`, merged SLAM, and the
unchanged detector: OWLv2 `google/owlv2-base-patch16-ensemble`, prompt
`humanoid robot`, threshold 0.55. The colour camera is 640x480 with a 384 px
focal length (79.6° horizontal field of view).

| Run | Code | Method |
|---|---|---|
| View capture | `a7c84a0` | `--plan-only`; the robot teleported with Gazebo `set_pose` to fixed poses; 2–3 s of live detections and one frame per pose |
| Complete mission | `ceb8d9a` | the plain one-command demonstration |

The mission ran from the working tree before `ceb8d9a` was committed; its code
and tests were final then, and only documentation changed before the commit.
Frames, sweeps, bags, ground truth and scripts are in the untracked
`artifacts/exploration/parking_2026-10-30_10-17-56_gz.fzUoDP` (capture) and
`parking_2026-10-30_10-36-00_gz.2oEC5u` (mission). Provenance is partial
because those artifacts are untracked; the earlier failing runs recorded no
images.

## Why detection failed

Captured frames at the recorded return poses show the G1 fully in view,
complete, and not occluded. The table is on the opposite side of the image. A
person model of the stock warehouse enters the right of the frame when the
robot turns right, but scores only 0.13–0.18 as `humanoid robot`. Offline
OWLv2 on the same frames, with the same model and prompt and threshold 0.05,
always ranks the G1 first, and reproduces the live scores exactly:

| Pose (map, robot yaw) | Live detection | G1 score |
|---|---|---|
| Observation pose `(0, 0)`, 0° | 30/30 frames | 0.626 |
| Recorded failure C `(-0.012, -0.037)`, -17.4° | 0/31 | 0.506 |
| Recorded failure A1 `(0.112, 0.008)`, -19.2° | 30/30 | 0.550 |
| Recorded successes `(0.168, 0.016)` -18.2°, `(0.159, 0.029)` -19.2° | 30/30, 31/31 | 0.582, 0.580 |
| `(0, 0)` at -5° / -10° / -15° / -20° / -25° / -30° | pass / **fail** / pass / pass / **fail** / pass | 0.599 / 0.540 / 0.600 / 0.609 / 0.522 / 0.554 |

The G1 was visible but not recognized: away from the observation pose its score
moves within roughly 0.50–0.61 around the threshold, and does not decrease
monotonically with heading. The heading error was the cause only in that it
left the one view known to work. Position matters as much. Within ±0.10 m and
±6° of the observation pose, 8 of 45 poses failed, all with a lateral offset or
x = -0.10 m. Within ±0.05 m and ±6°, 4 of 45 failed, each combining a 0.05 m
lateral offset with a 6° turn. Every pose with a lateral offset of at most
0.025 m passed at any heading in ±6°.

## Correction and validation

The observation return applies a temporary 0.03 m / 0.10 rad goal checker,
restores the shared checker when Nav2 releases the goal, and verifies the
stopped pose and the camera view before relocalizing.

Complete mission on `ceb8d9a`:

- Initial detection, localization and G1-centred exploration (0/334 donut cells
  unknown) succeeded.
- The tighter checker was applied before the single observation goal, and the
  shared checker was restored once Nav2 released it.
- The verified stop was 0.008 m / 0.070 rad from the observation pose in the
  map. Ground truth: 0.024 m / 0.072 rad (4.1°) from the spawn pose, with the
  G1 4.9° off the camera axis.
- The second localization stabilized immediately, and the plan selected
  `(2.562, -0.703, yaw -2.043)`, a 3.325 m path, giving `PARKING_PATH_FEASIBLE`.
- About 3 s into the pre-parking leg the live LiDAR clearance vetoed
  `lidar_obstacle_on_route`: the nearest return was 0.454 m from the footprint,
  1.89 m from the G1. All 42–44 returns within 1.2 m of the robot at
  `(0.54, -0.01)` lay on a real ground-truth obstacle at about `(0.9, -0.95)`.
  The leg was cancelled and released, the robot stopped and returned to within
  0.387 m of the start, and the mission ended `MISSION_FAILED`
  (`parking_failed`). No parking pose was reached, so reach and parking error
  were not measured.

The veto is a correct reaction to a real obstacle inside the remaining route as
the clearance monitor sweeps it. That route had passed the map-based path
check, so the two checks disagree near the start (the separate
checked-route-versus-executed-route issue), which this change does not
address.

Correction, 2026-10-30: the paragraph above inferred a real obstacle from the
returns within 1.2 m of the robot only. A recorded reproduction found that
the route cleared real obstacles, and that the offending return was a Gazebo
LiDAR seam phantom (bin 360), which also appeared in this run's scans just
before its veto; see the [route-veto record](2026-10-30-gazebo-parking-route-veto.md).
