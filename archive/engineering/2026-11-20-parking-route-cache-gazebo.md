# Route revalidation cache: controlled Gazebo runs

Recorded dates: 2026-11-20
Tested revisions: `804c1f76fa71da359a006463e4b41475867cf1ed`, `9f7120e4501ee94cbb25372b0106be4a5941a27d`
Provenance: partial

## Conditions

Two controlled diagnostic runs, not standard autonomous-search results. Both used the
Gazebo hospital ward (`parking_demo`, `standard` scenario, G1 at (0.5, -4.5) facing
-90 deg) and the real detection, localization, route planning, stop check, Nav2 and
LiDAR check. Only the robot's spawn differed from the scenario: `x:=3.04 y:=-3.58
yaw:=-2.7925`, 2.7 m east of G1 and facing it. A `standard` run on `804c1f7` with the
normal spawn had timed out in the search: G1 was never in the camera's view, so no
stop check ran. Each run used `tools/parking_scenarios/run_scenario.sh` with a 420 s
limit. Each also used a temporary, environment-gated trace hook in the mission
manager's `main()`, never committed and since reverted. The hook timed callbacks and
recorded the manager's clock at each odometry callback, plus, in the validation run,
the cache's reuse. A passive recorder captured `/clock`, and TF too in the validation
run. The tree's uncommitted `start_parking_demo.sh` option was present.

Evidence is untracked under `artifacts/exploration/`. The diagnosis run is in
`motion_evidence_2026-11-20/gz_controlled_east/` and the validation run in
`cache_validation_2026-11-20/gz_controlled_east/`. Each folder holds the scenario bag
(mission state, map, Nav2 plans, scans, raw and filtered odometry), the trace, the
hook patch and a label file. Provenance is partial: the trace hook was present, the
spawn was not the scenario's, and the scene had no moving obstacle.

## Diagnosis (`804c1f7`)

`MISSION_COMPLETE` on the third attempt. Attempts 1 and 2 were LiDAR vetoes beside G1.
While the first plan (a 5.0 m route) waited to drive, every tick re-swept the route:
0.26-0.30 s each, back to back. The cache keyed the whole map, which SLAM rewrites
every 0.5 s even while the robot stands still. Odometry callbacks fell to 6.4/s, and
gaps of 0.58-0.69 s restarted the stationary interval (`odometry_stale`) three times.
The wait took 7.2 s, and the restart allowance ran 9.47 s of its 10 s. The manager's
odometry stamp minus its clock stayed within -200 to +74 ms, so no `odometry_timestamp`
occurred.

## Validation (`9f7120e`)

| | Diagnosis run | Validation run |
|---|---|---|
| Outcome | `MISSION_COMPLETE` | `MISSION_COMPLETE` |
| Routes of the three plans | 5.0, 2.6, 1.0 m | 2.78, 2.55, 1.24 m |
| Revalidations reused while waiting | about 2 of 25 (5.0 m wait; from tick timing) | 24 of 32 |
| Odometry callbacks during the waits | 6.4/s (5.0 m wait) | 36-40/s |
| Largest odometry gap during the waits | 0.69 s | 0.41 s |
| Stationary-evidence restarts during the waits | 3 (5.0 m wait) | none |
| Waits before driving | 7.2, 2.2, 1.4 s | 1.45, 1.94, 1.91 s |
| Restart allowance used, at most | 9.47 s | 3.16 s |

G1 was confirmed at once (score 0.80). Its heading was estimated 5-6 deg from the
true heading, except at the two close views, which correctly gave none. As before, attempts 1 and 2
were LiDAR vetoes beside G1 (0.10 and 0.12 m). Attempt 3 parked, waited and departed
0.49 m. The robot then returned to its start.

The 8 re-sweeps had these causes:

- 4 robot-pose moves beyond 0.5 mm or 0.5 mrad: SLAM corrections of up to 8 mm and
  0.87 mrad, mostly right after a stop.
- 3 window shifts that followed such moves.
- 1 first check.

None came from map cells. A reuse cost 0.15-0.42 ms against 0.08-0.17 s for a sweep.
Each attempt still had one `odometry_stale` restart during its route search. The
planning callback took 0.36-0.39 s and the first route-check tick followed it. The
stop check then completed within the allowance. On the run's 354 recorded maps,
cached and uncached decisions agreed for all three routes. That includes the one map
where each route was blocked by unknown cells, which the cache swept again and
rejected.

## Limits

The validation run's routes were at most 2.78 m. The original 5.0 m route was
validated only offline, by replaying the diagnosis run's recorded maps and route.
Cached and uncached decisions agreed on all 326 maps, and wait 1's 25 ticks fell from
15 sweeps to 4. No moving or newly placed obstacle was tested live. Reusing a result
while the robot stays within 0.5 mm and 0.5 mrad of the swept pose is a bounded
approximation, not a proof of identical decisions; it is tighter than the former
1 mm / 1 mrad rounding. Each run is a single controlled run, Isaac was not run, and
the real robot was not tested.
