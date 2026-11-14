# Parking after merging the group's dev: integration, geometry and regression runs

Recorded dates: 2026-11-14
Tested revisions: `64b317638c63d0fd10691b433b44d7a5f5fdc21c`, `c1435d9815822ca98222aca3b576832a67afc589`, `99387ae858ed38a26b986247999977b0506fa66e`, `5b5133cff7a2152d04d28f2526cf2b0d53fb0df2`
Provenance: partial

## Question and conditions

Does the parking mission keep working after merging `origin/dev` at
`cc86eef5a5c708be2a31aac64db29a91225f4023` into the integration branch, and what
did the merge change for parking? Pre-merge baseline: `993c339` (local backup branch
`backup/parking-pre-p8-993c339`). Runs used `tools/parking_scenarios/run_scenario.sh`
and `record_demo_run.sh` (loopback DDS domain 83, one simulator at a time, no other
user's simulator running); evidence is in the untracked
`artifacts/exploration/p8_2026-11-14/`, each run with `commit.txt` and `code.txt`.
Gazebo errors are against ground truth sampled every 1-2.5 s; Isaac rows are
estimated state.

## Merge

`dev` was 47 commits ahead of the merge base `6d81e67`. Conflicts and resolutions:

| File | Resolution |
|---|---|
| `frontier_explorer/frontier_explorer_node.py` | same code on both sides (goals face their approach); comments merged |
| `src/ridgeback_autonomy_isaac/sim/isaac/worlds.py` | both world entries kept (`parking_demo`, `robocup_hcc_arena`) |
| `README.md` | both links kept; the parking section moved to its reference to keep the README within its 600-line budget |
| `docs/exploration/architecture.md` | dev's explorer paragraph merged with the parking sections; the goal-checker statement corrected |
| `graphify-out/` | regenerated |

The new `ros_gz_scene_state.patch` (Gazebo scene state survives entity additions,
which the parking startup makes) was applied by the runbook: the empty untracked
`ros_gz_sim/COLCON_IGNORE` removed, `tools/check_dependencies --apply --profile all`,
and `ros_gz_sim` built from source.

## What the merge changed for parking

- **Goal checker.** The shared checker now accepts any heading (3.15 rad). Every
  parking leg, observation turn and step back sets its own checker (0.08 m / 0.12 rad
  and 0.03 m / 0.10 rad); only exploration goals and the return to the start use the
  shared one, so the return now ends anywhere within 0.40 m of the start, any heading.
  The parking profile qualification and four tests pinned the shared checker at
  0.40 rad; they now compare it with `nav2_params.yaml`.
- **Robot outline.** Nav2's outline became the mesh-shaped 24-point body (rear edge
  -0.476 m instead of -0.497 m, angled rear ends). The provisional basket's rear
  corners (-0.48, ±0.19) now lie up to 0.035 m outside it; the parking planner,
  route checks and parking LiDAR check use the hull of both (`parking_footprint`),
  Nav2 keeps the group's outline. Before the merge the hull equals Nav2's outline.
- **Clearance, measured in open space** (G1 at the origin, generic keep-out 0.29 m,
  0.08 m obstacle floor, legs taken 0.17 m from G1's axis as depth shows them):

  | | Valid poses | Reach | Footprint to G1's axis | Gap to G1's legs |
  |---|---|---|---|---|
  | before (`993c339`) | 44 | 0.526-0.540 m | 0.308-0.323 m | 0.138-0.153 m |
  | after the merge | 76 | 0.491-0.540 m | 0.291-0.340 m | 0.121-0.170 m |

  The band is wider in both directions: the best case gains 0.017 m, and the
  closest poses are 0.017 m nearer. Reach, keep-out and floor are unchanged.
- **Costmaps** at 0.05 m and the explorer's rescaled cell counts; the SLAM map was
  already 0.05 m.

## Tests

The full suite on the uncommitted merge showed 16 failures and 210 errors. 213 were
`Context.init() must only be called once`: `test_lidar_qualification_contract` runs
`git diff HEAD` and decodes it as UTF-8, and during the uncommitted merge that diff
was the whole dev delta (30 MB, with non-UTF-8 bytes), so the test failed with the
ROS context open and every later fixture failed; committed, the diff is small. The
remaining failures were the 0.40 rad pins. After the merge commit: 2295 passed, 1
skipped, 3 environment-only failures (`ffmpeg_image_transport_msgs` not installed,
the untracked `docs/experiments/` in the doc-link test, and a hardware-bag test
reading another user's unreadable directory).

## Runs

| Run | Code | Outcome | Detail |
|---|---|---|---|
| `isaac_standard` (recorded) | `64b3176` | `MISSION_COMPLETE`, returned | search 397 s; G1 verified from 6.3 m, then 1.4 m (heading -88.2 deg, configured -90); `g1_right` vetoed on the route (a return 0.24 m from G1's estimate at the planned pose's rear corner); `g1_left` parked on attempt 2, arrival 0.020 m; return ended 0.397 m from the start, any heading (the shared checker); video 78 s, 1.3 MB, `isaac_standard/demo.mp4` |
| `gz_standard` | `64b3176` plus documentation | `PARKING_PLANNING_FAILED` (`target_stale`), stopped | G1 re-seeded on its head 1.0 m off the worktable edge, 0.011 m from ground truth, heading error 8.7 deg; `g1_left` selected; the refresh compared samples with the moved position (fixed in `c1435d9`) |
| `gz_standard_run2` | `c1435d9` | `PARKING_PLANNING_FAILED` (`motion_evidence_lost: collecting_stationary_interval`), stopped | G1 0.131 m from ground truth; `g1_left` on a 9.1 m route; revalidating it every tick starved the odometry callbacks (fixed in `99387ae`) |
| `gz_standard_run3` | `99387ae` | `MISSION_FAILED`, stopped | search 1055 s; no detection on the static view, the turn back did not help; first-localization timeout (perception) |
| `gz_standard_run4` | `99387ae` | `MISSION_FAILED`, returned 0.23 m from the start | G1 0.019 m from ground truth; plan passed its gate; the first leg ended `lidar_stale` on the rear LiDAR although both published at 29.5 Hz (largest gap 0.15 s): the 14 ms check on every scan held the node back (fixed in `5b5133c`) |
| `gz_no_feasible` | `5b5133c` | `MISSION_FAILED`, returned 0.36 m from the start | G1 0.025 m then 0.053 m from ground truth, heading error 7.8 deg; `no_valid_candidate`, no attempt |
| `gz_standard_run5` | `5b5133c` | `MISSION_COMPLETE`, returned 0.60 m from the start (ground truth) | search 511 s; G1 0.015-0.041 m from ground truth, heading error 6.2 deg; `g1_left` vetoed on the route, `g1_right` in the stopping path, `g1_rear` parked on attempt 3: reach 0.479 m, basket 0.089 m from G1's lower body, final-leg LiDAR minimum 0.104 m (all ground truth except the LiDAR) |

## Two regressions the runs found

- **Planning refresh after a re-seed** (`c1435d9`). The head check moved G1 1.0 m off
  the worktable edge the detector measured (position 0.011 m from ground truth), but
  the planning refresh compared new samples of that edge with the moved position, so
  a verified plan ended `target_stale` with G1 in view. Samples are now compared with
  the surface estimate of the same stationary view.
- **Route revalidation load** (`99387ae`). With the 24-vertex outline and 0.05 m
  paths, one sweep of a 9 m route took 0.27 s (0.18 s before the merge, synthetic
  map); repeated every tick it delayed odometry past the 0.5 s stationary window and
  the plan expired (`motion_evidence_lost: collecting_stationary_interval`). An
  unchanged revalidation (same grid contents, robot pose to 1 mm, route, target and
  start footprint) is now reused.
- **Parking LiDAR check load** (`5b5133c`). The check took 14 ms on every scan of
  both LiDARs after the merge (7.7 ms before, synthetic scene); the node fell behind
  and judged fresh scans stale. Only returns within the footprint's circumscribed
  radius (plus the margin) of a pose are measured exactly: 2.2 ms, identical outcomes
  and vetoing returns on 1,500 random scenes.

## Outcome

After the three fixes the merged branch completed one mission in each simulator
(Isaac at G1's left, Gazebo behind G1 after both sides were vetoed) and refused the
`no_feasible` scene correctly. Gazebo side parking is still vetoed near G1's legs;
the wider reach band did not change that in this run.
