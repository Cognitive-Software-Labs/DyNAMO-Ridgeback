# Gazebo LiDAR first-ray artifact and the occupied start cell

Recorded dates: 2026-10-30
Tested revisions: `edbfb9a9642a950463e4fd7b7ba5fb1b6d92c5ee`, `aa5eb2fd0a18678ba91303b2ae28c16fa77bc2ab`
Provenance: partial

## Question and conditions

An earlier run on the same date (see
[the start-cell blocker](2026-10-30-gazebo-parking-start-cell-blocker.md))
failed parking because one occupied SLAM cell at `(0.017, -0.001)` lay
under the robot's own start footprint. This record establishes where such cells
come from and verifies the correction.

All runs used `start_parking_demo.sh gz` on Gazebo `depot` (native
`warehouse`) with the demo G1 at `(3, 0)` and the table at `(3, 1.05)`, the
repository `clearpath/robot.yaml` (both UST-10LX on `chassis_link` at
z = 0.179 m, ±135°, 1081 bins), and the default `slam_source=front_only`:
slam_toolbox maps only `sensors/lidar2d_0/scan`. Each run recorded a rosbag of
`/map`, both scans, TF, filtered odometry, `/clock` and the mission state,
plus Gazebo `/world/warehouse/pose/info` ground truth at 10 Hz.

| Run | Code | Mode |
|---|---|---|
| Diagnostic | `edbfb9a` | `--planning-only` |
| Validation | `aa5eb2f` | full mission |

The validation ran from the working tree before `aa5eb2f` was committed. The
guard node, launch change and tests were final by then and reached the run
through the symlink install; only documentation changed before the commit. Bags, ground truth and analysis scripts are in the untracked
`artifacts/exploration/parking_2026-10-30_09-13-24_gz.NEqjk9/` (diagnostic)
and `artifacts/exploration/parking_2026-10-30_09-31-10_gz.tUGu23/`
(validation). Provenance is partial because those artifacts are not tracked and
the first failing run recorded no scans.

## Diagnosis (diagnostic run)

The start cell was not a genuine obstacle. Ground truth contained only the
robot, the demo G1 and table, and a world table at `(-12.7, 6.5)`. During the
run 9206 front and 23701 rear beams passed through the origin cell. The only
endpoints that landed in it came from three consecutive front scans at sim
time 64.8 s, all from bin 0 (-135°) at 1.09–1.10 m. The robot was then at map
`(0.85, 0.37)`; projected with the ground-truth pose, the endpoints were still
at world `(-0.04, -0.03)`. In this run SLAM left the cell free.

Bin 0 is wrong in general. Where bins 0 and 1 were both finite, they differed
by more than 0.3 m in 1507 of 1507 front scans (median difference -8.4 m) and
1378 of 1535 rear scans. Bins 1–3 and 1078–1080 disagreed with their
neighbours in 0.2–1.0% of scans. Bin 0 best matched bins 359–361 (median
absolute difference 0.04 m), the rays near -45°. Bins 360 and 720, at the ±45°
cube-map face seams, read `+inf` in 1710 of 3665 front scans, against about
600 for bins 359 and 361.

The artifact did reach the map. Of 2714 occupied cells in the final map, 26
were supported only by front bin 0 endpoints and by no other front ray. The
largest cluster lay just behind the start footprint, for example
`(-0.701, -0.060)` with 23 bin-0 hits. The first failing run's start cell fits
the same mechanism, but that run recorded no scans, so its attribution is
inferred rather than observed.

Other explanations were checked. The camera mast (x = 0.1955, bottom at
z = 0.295) is above the 0.179 m scan plane. The cell is directly behind the
front LiDAR, outside its ±135° field of view, so no correctly labelled front
ray can end there. The robot-declaration change (old `~/clearpath` mounts on
`default_mount`, z = 0 and 0.05, now `chassis_link` at 0.179) does not explain
an angle mislabel confined to bin 0 and was left unchanged.

## Correction

The gz-rendering library is a binary dependency here, so the scan is corrected
where Gazebo data enters ROS. `gazebo.launch.py` remaps each generated bridge
(`lidar2d_N_gz_bridge`) to `sensors/lidar2d_N/scan_gz` with a node-targeted
rule. `lidar_first_ray_guard` republishes every scan on the normal topic with
`ranges[0]` set to NaN. Path feasibility, unknown-cell handling, thresholds,
SLAM, Nav2 and the parking geometry were not changed.

## Validation run

- Topic wiring: only the generated bridges published `scan_gz`, and the guard
  was the only publisher of `scan`. SLAM, the collision monitor, localization,
  the startup gate and RViz subscribed to `scan`.
- Bin 0 was NaN in 4246 of 4246 guarded scans on each LiDAR and never NaN in
  the raw `scan_gz`. Bins 1–1080, metadata and intensities were identical to
  the raw scan in all 4246 stamp-paired front scans.
- The final map had no occupied cell within 1 m of the start (10 in the
  diagnostic run) and no occupied cell supported only by bin 0 (26 before).
  The origin cell went from unknown to free.
- The mission ran through detection, localization, G1-centred exploration
  (donut fully mapped), the observation return and candidate generation, then
  stopped in `PARKING_PLANNING_FAILED`. All bounded paths were rejected as
  `path_footprint_unknown`, not occupied.

## Remaining blocker

Front-only SLAM never observes the floor behind the spawn pose. Both final maps
had a large unknown wedge there: 260 unknown cells within 1 m of the start in
the diagnostic run and 376 in the validation run. Bin 0's false rays had
painted part of that wedge as free. With the start and current footprints
exempted, a footprint rotating in place at the live robot pose
`(0.205, 0.044, -0.324)` still touched unknown cells at -90°, -15°, +90° and
+165°. Before the guard, the occupied cell masked this, because footprint
rejection reports occupied before unknown. Unknown space is still rejected
conservatively; resolving it means observing that floor (for example with
the existing `slam_source=merged`), which changes shared SLAM input and was
not done here.

The diagnostic run also failed its second localization after the observation
return (`target_localization_timeout`); that failure did not recur in the
validation run and was not investigated.
