# G1 heading quality gate and the step back before planning

Recorded dates: 2026-11-19
Tested revisions: `a162527a698a28bb675c63ded3bf15d9f7a6e530`
Provenance: partial

## Question and conditions

Why did the final Isaac demonstration of 2026-11-19 get no G1 heading, and does a
heading need a view that shows G1's lower legs? Commit `a162527` added a quality gate to
the heading estimate and one LiDAR-checked step back before planning when the heading is
unknown from too close. Parking priorities, sectors, reach, Nav2 and safety thresholds
were not changed.

The analysis below ran the estimator offline on recorded depth views; ground truth was
used only to score. One live Isaac run of hospital `standard` followed
(`tools/parking_scenarios/run_scenario.sh`, headless, compact RViz, robot camera frames
saved once a second, no other simulator on the host), evidence in the untracked
`artifacts/exploration/reobservation_2026-11-19/isaac_standard/`; two more live runs
on the same code followed ([below](#two-later-live-runs)). The tree's uncommitted
`--parallel` option in `start_parking_demo.sh` and untracked plan documents were present
and unused. Provenance is partial: Isaac depth frames were not recorded in either run,
and Isaac has no ground-truth track.

## Why the final Isaac demonstration had no heading

The final-code Isaac demonstration (`artifacts/exploration/final_demo_2026-11-19/isaac_standard`,
mission code `b519ab0`) confirmed G1 from about (-0.33, -4.07) and localized it after a
34 deg turn in place, with the camera 0.76-0.86 m from G1, behind its right shoulder
(about -114 deg from G1's facing direction). The log reports no heading and a body axis
0.085 m toward G1's left of the midpoint of G1's two leg cells in the map; ranking fell back
to the approach side and the robot parked behind G1. The run recorded no depth or colour
frames.

The robot camera is level, 1.0345 m above base_link, with a 32 deg vertical half view
(640x480, 384 px focal length). At 0.86 m it sees nothing lower than about 0.5-0.6 m at
G1's legs; the forearm fit is centred on leg points between 0.10 and 0.55 m. Gazebo
captures at the same approach (`hospital_2026-11-18/g1_views_heading/standard/final_a2_obs`,
`g1_views_standard/std_close`, the fixture view `ward_close`, 0.69-0.70 m, -117 to -119 deg)
show the forearms (15523 depth points at forearm height) and no leg point: the lowest point
within 0.35 m of G1 is 0.56 m high, and the estimator stopped before fitting the forearms.
The Isaac view, slightly farther, probably saw only the top of the far (left) leg, which
would explain both the missing heading and the axis bias toward G1's left. Without the
frames that cannot be confirmed.

The same approach recurs in this scene: the 2026-11-17 Gazebo run `gz_standard_run2` and
the 2026-11-18 captures confirmed G1 from about (-0.32, -4.04). In the console logs of all
recorded runs, Isaac gave no heading from 0.41, 0.47 and 0.86 m and from about 12 m, and a
heading within about 6 deg of the configured yaw from every localization at 1.5-8.2 m (20
with a known distance). The one wrong heading on record (`gz_standard_run2`, -155.1 deg for
a G1 facing -90 deg) came from a 0.68 m view and had 14589 supporting points and 0.968
inliers. Distances in this paragraph are from the last observation goal plus the camera's
0.28 m offset, so they are approximate.

## Quality gate

`target_orientation.py` now accepts a heading only from a view that shows G1's lower legs:

- the camera's lowest image row reaches 0.30 m above the floor at G1's nearest leg surface
  (0.15 m from its axis), from the camera pose and intrinsics: about 1.3 m for the level
  camera;
- at least 50 leg points (the body axis's minimum; the fit previously ran on one), whose
  2nd-percentile height is at most 0.35 m.

Each localization reports a status: `estimated`, `view_too_close`, `insufficient_lower_body`,
`insufficient_heading_evidence`, `inconsistent_observations` or `view_geometry_unavailable`.
Support counts are not used as confidence.

The limits come from every recorded depth view of G1 (offline, ground truth only to score):

| Views | Range | Lowest visible height at G1's nearest leg | Leg points | Lowest leg points (2nd percentile) | Heading before | Gate |
|---|---|---|---|---|---|---|
| 72-view Gazebo sweep, 2026-11-01 (warehouse, 24 headings x 3 robot poses) | 1.62-2.72 m | at most 0.12 m | at least 937 | at most 0.13 m | 72/72, max error 12.9 deg | 72/72 kept, also with the seed 0.1 m off G1's axis |
| Ward views `ward_front`, `ward_behind`, `ward_occluded` (2026-11-17) | 2.1-3.3 m | at most -0.16 m | 258-1881 | 0.12-0.28 m | errors 0.2, 0.0, 10.0 deg | all kept |
| Workstation views (2026-11-09) | 2.3-6.9 m | below 0 m | 219-3241 | 0.11-0.12 m | 3 headings, 1 none | unchanged |
| Close ward views `ward_close`, `std_close`, `final_a2_obs`, `run2_close` | 0.63-0.70 m | 0.72-0.76 m | 0 | none | none | `view_too_close` |
| Ward views with the legs hidden (`nf_back_left`, corridor views) | 2.9-5.9 m | below 0 m | 0 | none | none | `insufficient_lower_body` |

The `side_view_close` workstation view (2.32 m) gives -97 deg; its two forearm bars both
point that way. The fixture stores G1's position but not its yaw, so it is left out of
the error figures.

The sweep is kept as a test fixture (`src/ridgeback_autonomy/test/data/g1_heading_sweep_2026-11-01.npz`,
depth within 0.7 m of G1, cut by `tools/parking_scenarios/make_heading_sweep_fixture.py`).
The leg-cell axis in the SLAM map was considered as an independent check: in the
demonstration's map the two leg cells (x 0.50 and 0.735 m, y -4.50 m) lie across G1's
facing direction, but they give the axis without its direction, merge into one cell from a
side view and sit next to table legs. It was not implemented.

## Live run on `a162527`

Isaac hospital `standard`, one run, `MISSION_COMPLETE`:

- G1 was confirmed about 7 m away this time. All three localizations gave a heading:
  -86.8 deg from about 7 m, -88.0 deg from 2.4 m and -88.8 deg from 1.5 m. The
  configured yaw is -90 deg; Isaac has no ground-truth track.
- The gate kept the 1.5 m view's heading (status `estimated`, 5/5 frames).
- The step back was never triggered: every view had a heading, and the camera was never
  closer than 1.5 m. `g1_reobservation` stayed empty.
- After two rounds of exploring around G1, the planner had 20 valid poses and chose
  `g1_right`. The bin was at -132 deg from G1's heading, 0.516 m from G1's body-axis
  estimate.
- LiDAR clearance:
  - pre-parking: 664 checks, minimum return 0.583 m;
  - final leg: 250 checks, minimum return 0.302 m, minimum G1 gap 0.343 m.
- Arrival error 0.026 m. The robot waited, then returned: 0.375 m and 2.79 rad from the
  start, within the shared goal checker.

So this run shows the gate accepting good views live and the unchanged parking path. It
does not show the step back in a simulator: that sequence is covered only by
`test_g1_reobservation.py`, which uses fake Nav2, scans and odometry. A live check needs
a run in which G1 is confirmed from closer than 1.5 m, as in the final demonstration;
the search order decides that.

## Two later live runs

Both ran the same day at `58c4077`, whose code is `a162527` (it adds only this record),
through `run_scenario.sh` with the same uncommitted, unused launcher option. A passive
observer saved the depth frames of each localization; evidence in the untracked
`artifacts/exploration/reobservation_2026-11-19/isaac_close_start/` and `gz_standard/`.
Neither triggered the step back.

- **Isaac `standard`, started at the final demonstration's confirmation pose**
  (-0.327, -4.072), through an `ISAAC_PYTHON` wrapper that adds the runner's `--spawn`.
  From there the detector scored G1 0.40-0.44, below the 0.50 search floor, so the
  robot searched for about 10 minutes and confirmed G1 about 4.4 m away (score 0.684).
  Heading -86.4 deg from about 4.1 m (status `estimated`, 746 leg points). It parked at
  `g1_left` (reach 0.53 m, arrival 0.028 m) and returned 0.39 m and 2.68 rad from the
  start: the return accepts any final heading (shared goal checker, 3.15 rad).
  `MISSION_COMPLETE`. The final leg's smallest LiDAR clearance, 0.13 m, is the distance
  from a return to the planning footprint, not a range: the rear LiDAR measured 0.40 m to
  a return behind the robot's rear-right corner, at about (1.10, -4.78) m in the map,
  which matches the ward chair `chair_s1_a` beside the worktable, 0.66 m from G1's axis.
  It appeared only at the parking pose, so it was not the robot itself, and no return in
  the 780 final-leg and parked scans came within the 0.05 m stopping-path margin. The
  check sees the 0.18 m scan plane only.
- **Gazebo `standard`, normal start.** G1 was confirmed (score 0.563) with the camera
  about 4.2 m away, unlike the 0.65-0.70 m confirmations of earlier Gazebo runs; the
  search order decides that. The gate accepted the heading: -94.5 deg (5/5 frames, 162
  points; ground truth -90 deg, error 4.4 deg). Offline, the saved frames had 69 leg points
  with a 2nd-percentile height of 0.31 m, close to the 50-point and 0.35 m limits. G1's
  estimate was 0.157 m from ground truth. A `g1_right` pose (reach 0.519 m, 8.08 m route)
  was selected, but 7.4 s later the plan was dropped:
  `PARKING_PLANNING_FAILED`, `motion_evidence_lost: collecting_stationary_interval`.
  Odometry was stale when the plan was selected after about 3 s of path checks, and two
  stamps ahead of the manager's clock restarted the stationary interval before a counted
  motion fault ended it. The robot did not drive. It is the plan-invalidation failure of
  the earlier Gazebo runs (backlog: delayed odometry stamp); the step-back code was not
  active. The two passive recorders added load, which may have contributed.

Over the three live runs the gate kept every heading, 1.2-4.5 deg from the configured
(Isaac) or true (Gazebo) yaw, from 1.5 to about 7 m. The step back has still not been
triggered in a simulator, and the 65 deg close-view fit remains unexplained.
