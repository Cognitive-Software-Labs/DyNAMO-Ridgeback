# Gazebo parking with merged SLAM: unknown start area and relocalization

Recorded dates: 2026-10-30
Tested revisions: `edbfb9a9642a950463e4fd7b7ba5fb1b6d92c5ee`, `aa5eb2fd0a18678ba91303b2ae28c16fa77bc2ab`, `5afb8bce607bab88bd72e109d2853f19f4c31cd6`
Provenance: partial

## Question and conditions

With the first-ray guard in place
([first-ray record](2026-10-30-gazebo-lidar-first-ray-artifact.md)), every
Gazebo parking route was rejected as `path_footprint_unknown` because the floor
behind the spawn pose stayed unknown. This record compares SLAM input
`front_only` with the existing `slam_source=merged` and records the resulting
complete mission.

All runs used `start_parking_demo.sh gz` on `depot` (native `warehouse`) with
the demo G1 at `(3, 0)` and the table at `(3, 1.05)`, the same spawn pose
`(0, 0, 0)`, loopback DDS domain 83, and recorded `/map`, both scans, the
merged scan where present, TF, odometry, the mission state and Gazebo ground
truth. Runs were sequential.

| Run | Code | Command | SLAM input |
|---|---|---|---|
| A1 | `edbfb9a` (no first-ray guard) | `--planning-only` | front_only |
| A2 | `aa5eb2f` | full mission | front_only |
| B | `aa5eb2f` (later commits were documentation only) | `--planning-only slam_source:=merged` | merged |
| C | `5afb8bc` | full mission, no extra arguments | merged by default |

Run C ran from the working tree before `5afb8bc` was committed; its launch
default and test were final then, and only documentation changed before the
commit. The
bags, ground truth and analysis scripts are in the untracked
`artifacts/exploration/` run folders `parking_2026-10-30_09-13-24_gz.NEqjk9`
(A1), `09-31-10_gz.tUGu23` (A2), `09-47-58_gz.gNS2xC` (B) and
`09-52-39_gz.IGp6ul` (C). Provenance is partial because those artifacts are
untracked.

## Merger checks (run B)

- Inputs are the guarded `sensors/lidar2d_{0,1}/scan`, so the first-ray NaN
  is applied before merging and dropped by the merger's finite-range filter.
  Bin 0 was NaN in all 3453 scans of each LiDAR.
- Static extrinsics `(0.3922, 0, 0)` and `(-0.3922, 0, pi)` match the
  repository declaration.
- Every front scan paired with an equal-stamp rear scan (2593 of 2593 at the
  last log): no motion compensation, TF misses or front-only publishes.
- The output was `base_link`, 1440 bins from -180°, range_max 10.3922 m.
  Coverage behind the robot rose from 34–62% to 89.5%, which the front LiDAR
  cannot see, so slam_toolbox consumed the merge.
- Neither LiDAR showed self-hits: minimum ranges over A2 were 0.439 m (front)
  and 0.794 m (rear), and no bin was short in most scans.

## Mapping comparison (final maps)

False occupied: a SLAM-occupied cell with no ground-truth obstacle within
0.20 m, outside 1.0 m around the spawned G1 and table, against the packaged
`depot` truth map. That map is sliced at z = 0.25 m against the 0.179 m scan
plane, so some disagreement with a real obstacle is expected. False free: a
SLAM-free cell at least 0.15 m inside a truth obstacle.

| Metric | A1 | A2 | B | C |
|---|---|---|---|---|
| Unknown cells touching the start footprint | 76 | 131 | 0 | 0 |
| Unknown within 1 m of start | 260 | 376 | 23 | 0 |
| Occupied within 1 m of start | 10 | 0 | 0 | 0 |
| Known behind the start (x < -0.5, within 1.5 m) | 62.0% | 34.2% | 89.5% | 98.9% |
| Occupied cells within 0.20 m of truth | 94.0% | 94.6% | 94.2% | 94.3% |
| False occupied within 2 m of start | 11 | 0 | 0 | 0 |
| False occupied more than 1 m from any truth obstacle | 6 | 6 | 0 | 1 |
| False free | 0 | 0 | 0 | 0 |

A1's 10 occupied and 11 false cells near the start are the bin-0 phantoms of
the unguarded run. Merged SLAM added no false obstacles near the start and no
false free space.

## Path feasibility

A2 (front_only): every bounded `ComputePathToPose` route was rejected as
`path_footprint_unknown`; an in-place rotation at the live pose still touched
unknown cells at four headings.

B (merged): G1-centred exploration mapped the whole donut (0/344 unknown). Of
342 donut candidates, 24 were geometrically valid (51 reach budget, 78 G1
keep-out, 174 footprint occupied, 15 footprint unknown). All 6 bounded paths
were accepted, and the pose `(2.403, -0.598, yaw -2.263)` was selected with a
2.965 m path: `PARKING_PATH_FEASIBLE`. Those checks are the unchanged
validator: the route from the actual pose, the swept footprint, the G1
keep-out and the straight final approach. About 5 s later the
`--planning-only` snapshot expired (`target_stale`) as designed.

## Complete mission (run C)

Automatic G1 and table spawning, the merger without extra arguments,
exploration, detection, the first localization, G1-centred exploration
(0/332 donut unknown) and the observation return all succeeded. The second
localization then failed (`LOCALIZE_G1 -> MISSION_FAILED`,
`target_localization_timeout`) before planning, so no parking leg ran and no
reach, parking error, clearance or return error was measured.

During that 5 s window the detector published `detected=false` for every
frame; a live probe saw 81 of 81. The measurement node still published a
message per frame, and the manager rejected all 52 as
`measurement_without_detection_association`. The G1 never moved or tilted in
any run. The same signature ended A1.

| Run | Ground-truth pose at the second localization | G1 off the camera axis | Range | Outcome |
|---|---|---|---|---|
| A1 | `(0.112, 0.008)`, yaw -19.2° | +20.9° | 2.64 m | no detection |
| A2 | `(0.168, 0.016)`, yaw -18.2° | +19.8° | 2.58 m | localized |
| B | `(0.159, 0.029)`, yaw -19.2° | +20.5° | 2.59 m | localized |
| C | `(-0.012, -0.037)`, yaw -17.4° | +19.9° | 2.76 m | no detection |

The earlier blocker run of the same date also localized twice. The
observation return ends within the shared 0.40 rad yaw tolerance, about 18°
from the observation yaw, so the G1 sits about 20° off-axis where detection is
marginal. Failure is independent of the SLAM input (one front-only and one
merged run failed). Detector scores are not logged, so the margin could not be
quantified.
