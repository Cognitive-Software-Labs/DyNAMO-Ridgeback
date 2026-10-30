# Progressive parking planning around G1

Recorded dates: 2026-10-30
Tested revisions: `260069c24cbe100595c0a554c429a7393825b08c`, `c089f86f8285bd6c995b780bea6b04ffa40beb2b`
Provenance: partial

## Question and conditions

Does planning first, checkpointing newly valid poses while exploring, and
relocalizing G1 in place shorten the mission without bypassing a safety check?

There were two consecutive Gazebo runs of the standard scene (`depot`, demo G1
at `(3, 0)`, table at `(3, 1.05)`, merged SLAM), on loopback DDS domain 83 and
Gazebo partition `ridgeback-parking-1010-83`. A teammate's separate simulation
ran meanwhile on domain 43 with its own partition. Our runs used a scratch copy
of `start_parking_demo.sh gz` without its any-simulator refusal; the lock,
environment and launch arguments were the same.

- **Baseline:** installed `mission_manager_node.py` from `260069c`, the only
  file this change touches.
- **Optimized:** the working tree before `c089f86`, which differed only in
  `g1_reacquire_min_range_m`, 1.0 m instead of 1.7 m.

Bags (state, map, markers, TF, odometry), console captures, screenshots,
`metrics.json` and the scripts are in the untracked
`artifacts/exploration/parking_2026-10-30_16-41-29_gz.eompVY` (baseline) and
`parking_2026-10-30_16-45-01_gz.ItOUMy` (optimized), so provenance is partial.

## Results

| Metric | Baseline | Optimized |
|---|---|---|
| G1 exploration | 1 round, 60.1 s, 4.71 m | 1 round, 10.9 s, 2.50 m (checkpoint: 2 newly valid poses) |
| Relocalization | observation return, 46.8 s driving, 4.05 m | in-place turn at 1.01 m failed, then the observation-pose fallback; 33.8 s driving, 3.05 m |
| Planning attempts (candidates, valid) | 1 (342, 25) | 2: plan-first (74, 0), then (339, 14) |
| Nav2 path checks | 6 | 7 (1 rejected, `path_footprint_unknown`) |
| First G1 localization to pre-parking leg | 117.6 s | 63.7 s |
| First G1 localization to parked | not parked | 79.9 s |
| Mission (start to end state) | 159.7 s, `MISSION_FAILED` | 108.9 s, `MISSION_COMPLETE` |
| Final reach to G1 ground truth | n/a | 0.598 m (map frame, pose `(2.122, 0.028, 3.121)`) |
| LiDAR minimum, pre-parking leg | 0.182 m, G1 gap 0.656 m | 0.330 m, G1 gap 0.672 m |
| LiDAR minimum, final leg | vetoed at 0.105 m, G1 gap 0.354 m | 0.210 m, G1 gap 0.330 m |

The baseline's final-leg veto is the separate G1-leg clearance issue recorded
in the [display record](2026-10-30-parking-donut-display.md). The optimized
run selected a pose on G1's west side instead; one run does not show that the
issue is avoided.

On the earlier `260069c`-equivalent run's maps
(`parking_2026-10-30_16-18-35_gz.Trpwks`), the first geometrically valid pose
appeared 14.3 s into a 54.4 s exploration.

The in-place relocalization at 1.01 m passed its view check (G1 0.27 rad off
the axis), but the detector reported no G1 for the whole 5 s window. That
attempt cost about 14 s. The camera optical frame is 1.03 m above `base_link`,
with a 32° vertical half view, so G1's feet leave the image below about
1.65 m. `c089f86` therefore raised the minimum in-place range to 1.7 m, so this
run would have gone straight to the observation pose. A successful in-place
relocalization was not observed live.
