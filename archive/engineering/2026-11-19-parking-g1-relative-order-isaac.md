# G1-relative parking order: first live Isaac run

Recorded dates: 2026-11-19
Tested revisions: `b2b84b42fe0688afdd4654d2ffcd4727af97cc84`
Provenance: partial

## Conditions

One Isaac hospital-ward `standard` run (`parking_demo`, G1 at (0.5, -4.5) facing
-90 deg) on `b2b84b4`: the strict order right, left, front, rear, other, with the
route-check budget of 16 and the guard that keeps a found route from the batch
timeout. It ran through `tools/parking_scenarios/run_scenario.sh` (headless, compact
RViz, camera frames once a second) with a passive observer saving the localization
depth frames; no other simulator ran on the host. The tree's uncommitted
`start_parking_demo.sh` option and untracked plan documents were present and unused.
Evidence is in the untracked
`artifacts/exploration/priorities_2026-11-19/isaac_standard/`. Provenance is partial:
Isaac has no ground-truth track, so headings are compared with the configured yaw.

## Decision

- G1 was confirmed (score 0.644) with the camera 6.2 m away. The three localizations
  gave headings of -86.3, -86.9 and -87.4 deg (status `estimated`) with the camera
  6.2, 2.6 and 1.8 m from G1; no step back was needed (`g1_reobservation` empty).
- The first plan had no valid pose (footprints occupied or unknown); two rounds of
  mapping around G1 followed, the second because a better-ranked class was still
  unmapped.
- The final plan, with G1 at (0.613, -4.556), had valid poses only in `g1_right` (5),
  `g1_rear` (10) and `g1_other` (7). Replaying the planner on this run's map gives the
  same classes: G1's left had none (72 footprints on mapped obstacles, the ward chair
  and the worktable's corner on that side; 3 too close to obstacles; 7 over the reach
  budget; 2 in the keep-out), and its front none (the worktable: 37 occupied, 19
  unknown).
- Route checks: the five right poses only, all safe, in 1.4 s; the budget was not
  reached and no lower class was checked. The log and `route_search` state:
  `g1_right feasible (5/5 checked, 5 feasible); g1_rear unchecked (0/10);
  g1_other unchecked (0/7)`.
- Selected: `g1_right`, task point 0.534 m from G1's estimate, about -132 deg from its
  heading (right-rear), clearance score 0.142 m, route 2.99 m. The parked scene view
  shows the robot behind G1's right shoulder.

## Execution

- Pre-parking leg: 681 LiDAR checks, nearest return 0.557 m from the footprint,
  G1 gap at least 0.717 m. Final leg: 246 checks, 0.301 m and 0.362 m. No LiDAR
  cancellation; one attempt.
- Arrival error 0.029 m, 0.001 rad, so the reach lay within 0.505-0.563 m.
- Returned to the start 0.400 m and 2.801 rad off, accepted by the shared goal
  checker (0.40 m; any heading). `MISSION_COMPLETE`.

## Limits

One run. Only one side had valid poses, so right was chosen over left by geometry
rather than by order; the left-over-right decision, the batch continuation and the
route-check budget were not exercised live. Gazebo, where checks are slower, was not
run.
