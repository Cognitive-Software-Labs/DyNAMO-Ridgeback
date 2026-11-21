# Autonomous parking beside a G1: supervisor handover

A Clearpath Ridgeback searches a hospital ward for a Unitree G1 standing at a worktable,
localizes it, parks so that the basket on its deck is within the G1's reach, waits five
seconds and drives back to its start. If no safe parking position exists, it refuses and
returns. Everything runs in simulation: Gazebo Sim and NVIDIA Isaac Sim, with the same
autonomy code. This page is the entry point; the linked references hold the details.

## System architecture

One ROS 2 Jazzy workspace drives either simulator through backend packages
(`ridgeback_autonomy_gz`, `ridgeback_autonomy_isaac`). Application behaviour lives in
`ridgeback_autonomy`; detection and depth-based localization in `ridgeback_localization`;
shared messages in `ridgeback_interfaces`. The robot and its sensors (two planar
Hokuyo UST LiDARs, a RealSense D455 on a mast) are declared once in
`clearpath/robot.yaml`. [Exploration architecture](architecture.md) and the
[repository map](../project/PROJECT_CONTEXT.md#repository-map) give the layout.

| Node | Responsibility |
|---|---|
| `mission_manager_node.py` | The parking mission: state machine, G1 localization and depth checks, parking planning, Nav2 goals, the live LiDAR check, wait and return |
| `frontier_explorer_node.py` | Frontier exploration; its MissionGuard, a latched safety and ownership interlock around its Nav2 client, lets the mission pause it and take over navigation |
| `target_detector_node` | OWLv2 open-vocabulary detection of the G1 in the camera image |
| `async_slam_toolbox_node` | 2D SLAM; the parking demo feeds it both LiDARs merged by `scan_merger_node` (`slam_source:=merged`) |
| Nav2 (`planner_server`, `controller_server` with MPPI, `bt_navigator`, `behavior_server`, `smoother_server`, `velocity_smoother`, `collision_monitor`) | Route planning and execution with the robot's outline; the collision monitor stays in front of the wheels |
| `stationary_mission_guard` | Stand-in for the explorer in the non-driving `--plan-only` mode; not MissionGuard |
| `parking_simulation_startup` | Spawns the scenario's G1 and objects through the simulator's scene services and gates startup on them |
| `hud_node`, overlay nodes, RViz | Operator display: state machine, parking donut, selected pose |

## Mission state machine

The mission moves through `SEARCH_G1`, `STOPPING_EXPLORATION`,
`RETURNING_TO_OBSERVATION`, `SETTLING_MOTION`, `LOCALIZE_G1`, `G1_LOCALIZED`,
`EXPLORING_AROUND_G1` (only when unmapped floor could change the choice),
`PARKING_CANDIDATES_READY`, `EVALUATING_PARKING_PATHS`, `PARKING_PATH_FEASIBLE`,
`NAVIGATING_TO_PREPARKING`, `NAVIGATING_TO_PARKING`, `PARKED`, `RETURNING_TO_START`
and `MISSION_COMPLETE`. A rejected or failed parking leg returns to the observation
pose for the next of at most three attempts; a refusal or an unrecoverable failure
returns home and ends `MISSION_FAILED` with its reason. Every state, its exit
condition, its configured limit and its measured duration are in
[states and timing](parking_mission_states.md).

## Perception and localization

The detector confirms G1 when three of five frames score at least 0.50. The robot
turns toward the detection, waits for stationary odometry, and takes five consistent
position samples from depth in the detection box. A position is accepted only when
depth shows G1 there: at least 20 points at torso and head height within 0.25 m in
three frames, and fewer than 20 points at 1.45-2.20 m (G1 stands at most 1.33 m).
The body axis comes from G1's legs; the head region of the box re-seeds the position
when the surface estimate lies on something else, and is itself ignored when it shows
an object taller than G1. G1's heading is estimated from its forearms and only orders
the parking poses. Ground truth is never an input. Details:
[G1 position check](architecture.md#g1-position-check) and
[G1 heading](architecture.md#g1-heading).

## Parking candidates, priority and safety

- **Candidates.** A 360-ray donut around G1: each pose puts the rear-bin task point,
  0.28 m behind the robot's centre, 0.30-0.60 m from G1's axis.
- **Geometry.** Poses are checked with the hull of Nav2's robot outline and the
  provisional basket, a 0.29 m keep-out around G1, 0.08 m from mapped obstacles;
  unknown cells count as unsafe; tabletops the LiDARs pass under are marked from depth.
- **Priority.** G1's right, then its left, then in front, then behind, then diagonal
  or ambiguous poses (sectors of 45, 30 and 30 degrees around G1's estimated heading);
  a lower class is route-checked only after every pose of the higher ones. Sectors
  only order the checks; they never make a pose valid. Without a heading, poses are
  ranked by the approach direction and never called G1's right or left.
- **Route.** Each ranked pose is checked with a Nav2 route before driving; the robot
  drives to a pre-parking pose 0.5 m before the parking pose, then a tight final leg.
- **Live LiDAR check.** During both legs, both raw scans are checked against the
  stopping path and the route ahead with a 0.05 m margin; a return there cancels the
  leg, and the mission tries the next direction (at most three attempts).
- **Goal ownership, wait and return.** The mission owns at most one Nav2 goal and
  releases it only on a terminal result; it waits five seconds after the stop is
  confirmed, departs straight back to the pre-parking pose under the same LiDAR check
  and within 0.08 m / 0.12 rad of that line (a blocked or failed departure stops the
  robot and fails the mission), then drives back within 0.40 m of the start (any
  heading). A failed final leg departs the same way before the retry or return.

The [technical review](parking_review.md) summarizes the design for review.

## Simulation environments

Both simulators load the same hospital ward (`parking_demo`), `initial_test_world`
without its own G1: Gazebo through `parking_demo.sdf`, Isaac through a USD layer, with
identical collision boxes. The robot starts in the corridor at the world origin; G1
stands at (0.5, -4.5) m in the south room, facing its worktable, out of the camera's
sight. Eight scenarios change what stands around G1 and which way it faces
([scenarios](parking_scenarios.md)). Gazebo's earlier world, Clearpath's warehouse,
stays available with `world:=depot` to reproduce the runs recorded there.

## Installation and launch

Install ROS 2 Jazzy, import the dependencies, build, and create the perception
environment as in the [root README](../../README.md#installation); Isaac Sim 6.1 is
optional (README section 4). In short, from the checkout:

```bash
vcs import < dependencies/core.repos
vcs import < dependencies/gz.repos        # Gazebo hosts
vcs import < dependencies/viz.repos       # hosts that run RViz
tools/check_dependencies --apply --profile all
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --base-paths src
source install/setup.bash
```

Run one simulator at a time. Each command runs the full mission in the ward:

```bash
export RIDGEBACK_PERCEPTION_VENV=/path/to/perception_venv
bash start_parking_demo.sh gz rviz_layout:=compact                            # standard scene
bash start_parking_demo.sh gz rviz_layout:=compact demo_scenario:=no_feasible # safety scene
bash start_parking_demo.sh isaac rviz_layout:=compact                         # Isaac Sim
```

`--plan-only` and `--planning-only` stop before driving
([modes and startup gates](parking_demo.md)).

## Reproducing and evaluating a scenario

`run_scenario.sh` runs one scenario in its own session with a time limit, records a
small bag, samples Gazebo ground truth, and saves screenshots; `scenario_metrics.py`
computes the measurements:

```bash
export RIDGEBACK_PERCEPTION_VENV=/path/to/perception_venv DISPLAY=:0
DEMO_ARGS="rviz_layout:=compact" PARKED_VIEWS=workstation,closeup,angled \
  END_VIEWS=overview,workstation,closeup,angled FRAMES=1 FRAMES_PERIOD=2 LIMIT=1800 \
  tools/parking_scenarios/run_scenario.sh gz standard artifacts/exploration/OUT_standard
python3 tools/parking_scenarios/scenario_metrics.py artifacts/exploration/OUT_standard
# The same for no_feasible, or isaac instead of gz (Isaac has no ground truth).
```

The [parking scenarios reference](parking_scenarios.md#evaluating-a-run) explains the
measurements and how the two pages are rebuilt.

## The two galleries

```bash
firefox docs/exploration/autonomous_parking_scenarios.html
firefox docs/exploration/autonomous_parking_results.html
```

The scenarios page shows all eight scenes in both simulators. The results page leads
with the three final-code demonstrations, keeps the development runs in a collapsed
history with their causes, and lists every one of the 61 recorded runs with its code
version and measurements. Both pages show hospital-ward images only.

## Final results

Four runs on the final mission code (`b519ab0`; the later commits `544ead1`, `0890e13`
and `2c6c5db` changed only comments, tools and documentation), on 2026-11-17 to
2026-11-19. Gazebo values are ground truth, used only for evaluation.

| Run | Outcome | Measurements |
|---|---|---|
| Gazebo `standard` (`gz_standard_final`) | `MISSION_COMPLETE` | attempt 1 at G1's right cancelled by the LiDAR check; parked on attempt 2 at G1's left (by ground truth 137 deg from G1's heading, behind its left hip); reach 0.509 m; G1 0.073-0.084 m and 2.1-6.2 deg off; basket 0.118 m from G1's lower body; waited 5 s; back 0.22 m from the start |
| Isaac `standard` (`isaac_standard_run2`) | `MISSION_COMPLETE` | parked at G1's right on attempt 1, arrival error 0.020 m (mission estimate), waited 5 s, returned; Isaac records no ground truth |
| Isaac `standard`, recorded on video (`final_demo_2026-11-19/isaac_standard`) | `MISSION_COMPLETE` | G1's heading was not estimated from its view, so poses were ranked by the approach side and the robot parked behind G1 on attempt 1 (bin about 0.46 m from G1's configured position by the mission's map pose; the scene expects G1's left or right); arrival error 0.028 m; waited 5 s; returned |
| Gazebo `no_feasible` (`gz_no_feasible_final`) | `MISSION_FAILED`, `no_valid_candidate` (intended) | G1 0.152 m and 10.0 deg off; 0 of 258 poses valid; no attempt; nearest approach 0.67 m; back 0.35 m from the start |

These are demonstrations, not reliability statistics. Five earlier ward runs on
earlier code exposed the problems addressed during validation, and one of them, the
heading error below, is still open; the results page's development history and the
dated record under archived evidence hold them.

## Demonstration video

One continuous Isaac `standard` run on the final code, recorded on 2026-11-19, in
`artifacts/exploration/final_demo_2026-11-19/isaac_standard/` (its `README.md` describes it):

- `presentation.mp4` (2:59): for the supervisor. Search plays 4x faster and the return
  2x faster; detection, localization, planning, the drive, parking and the five-second
  wait play at the recorded speed. Every banner states its speed, and title cards
  separate the stages.
- `complete.mp4` (4:50): the whole recording at the recorded speed.

Each frame shows two Isaac Sim cameras over the ward, the RViz window (map, state
machine, parking donut, selected pose) and the mission time and state. Isaac Sim ran at
about 0.39x real time, so the five simulated seconds of the wait take about 12 s of the
recording. In this run G1's heading was not estimated, and the robot parked behind G1
(see the results above).

```bash
xdg-open artifacts/exploration/final_demo_2026-11-19/isaac_standard/presentation.mp4
```

To record another run the same way (Isaac headless, compact RViz, one simulator on the
host), then build both videos from it:

```bash
export RIDGEBACK_PERCEPTION_VENV=/path/to/perception_venv DISPLAY=:0
bash tools/parking_scenarios/record_demo_run.sh isaac standard artifacts/exploration/NEW_DEMO
python3 tools/parking_scenarios/make_demo_videos.py artifacts/exploration/NEW_DEMO
```

A stills storyboard of the final-code Gazebo runs, including the attempt the LiDAR
check cancelled and the `no_feasible` refusal, is in `artifacts/exploration/final_demo_2026-11-19/`
(`storyboard.mp4`, 10 stills of 4 s each, not real time;
`tools/parking_scenarios/make_demo_storyboard.py`). Earlier complete videos ran older
mission code: `artifacts/exploration/p8_2026-11-14/isaac_standard/demo.mp4` and
`artifacts/exploration/demo_2026-11-11/isaac_standard_run3/demo.mp4`. Evidence and
recordings are kept untracked under `artifacts/`, not in Git.

## Known limitations

- **Few final-code runs.** Four runs on the final code; no success rate can be stated,
  and search time depends on the explorer's frontier order.
- **Parking without G1's heading.** When G1's forearms are not seen reliably, the heading
  is unknown and poses are ranked by the approach side instead of G1's sides; the
  recorded Isaac run then parked behind G1, a valid pose but not the side the scene
  prefers. That run saw G1 from 0.76-0.86 m, where its lower legs are out of the
  image; since 2026-11-19 such a view triggers one LiDAR-checked step back to about
  1.6 m before planning ([observation return](architecture.md#observation-return)).
  If the new view still gives no heading, the ranking stays undirected. The step back
  is covered by automated tests only: no simulator run has triggered it yet.
- **Departure from G1.** One Isaac hospital run demonstrated the checked departure, with
  the LiDAR check active throughout (record under archived evidence below);
  the departure after a failed final approach is covered by automated tests only. The
  stop confirmation it needs took 6 s of the 10 s allowed in that run, and the cause is
  unknown: the mission reads raw `platform/odom` in Isaac, but only the filtered odometry
  was recorded. Past the allowance the robot stays stopped beside G1. Moving obstacles
  were not tested.
- **Waiting to drive in Gazebo.** Re-sweeping a verified route against SLAM's
  constantly rewritten map once delayed odometry until the plan was discarded
  (`motion_evidence_lost`). The route check is now reused while the map cells along the
  route and the robot pose stay unchanged. One controlled Gazebo run confirmed it, with
  routes of at most 2.78 m (record under archived evidence below); the original 5 m case
  was checked only by replaying recorded maps. Reuse within 0.5 mm and 0.5 mrad of the
  checked pose is a bounded approximation, and no moving obstacle was tested.
- **Return after an incomplete route search.** After a planning timeout
  (`planning_timeout`) or a spent route-check budget (`route_budget_exhausted`), the
  robot returns to the start only once its stop is confirmed, its sensors are fresh and
  Nav2 plans a route home; otherwise it stays stopped
  ([incomplete route search](parking_mission_states.md#incomplete-route-search)). Both
  recoveries passed automated tests only; neither has been validated in a simulator or
  on the robot. Since 2026-11-21 `bounded_search_exhausted`, `no_valid_candidate` and
  `no_free_donut_cells` take the same guarded return under their own codes, and the
  return is also refused inside G1's keep-out. A failed last attempt and a retry that
  cannot proceed take it as well (the last final leg departs first, then stops again);
  automated tests only.
- **Camera search and `g1_not_found`.** The explorer turns the camera toward unseen
  directions at frontier stops, and a search without frontiers or a G1 confirmation
  ends as `g1_not_found` through the guarded return
  ([camera observation](parking_mission_states.md#camera-observation-at-frontier-stops),
  [search without G1](parking_mission_states.md#search-without-g1)). Replaying the
  2026-11-20 hospital run, the selection would have turned toward G1 (2.7-3.5 m,
  15-22 deg off-axis) at two stops; that is not a detection guarantee. One live
  `standard` run on 2026-11-21 performed 17 turns (all completed; 86 s); none faced G1,
  which was confirmed while driving, 4.3 m away. In a second (on `c6a1c7b`) the turn
  at the second stop brought G1, 2.7 m away and 38 deg off the heading, into view:
  confirmed 0.9 s into it, then parked (rear, after right and left LiDAR vetoes) and
  returned, `MISSION_COMPLETE`. Exploration order differs between runs, so one success
  is not a rate. `g1_not_found` was not reached live. Since then, camera frontiers
  follow exhausted LiDAR frontiers (at most 8 viewpoints in 300 s).
- **Relocalization from a distant first view.** In that run G1 was first localized from
  4.3 m, past the 3.5 m reacquisition range; its depth check passed on the torso while
  the head region showed the background. After mapping around G1, the in-place
  reacquisition 2.4 m away stopped 0.043 m from its pose (limit 0.040 m): its goal had
  been taken from a robot still moving at 0.5 m/s. The robot returned to the 4.3 m pose,
  where every head-depth check failed (`target_localization_timeout`). Since then the
  view is chosen from a confirmed stop, an in-place view is judged by the camera's view
  of G1, and only a reliable view (head verified, 1.3-3.5 m) is ever driven back to;
  otherwise `g1_relocalization_failed`, stopped
  ([observation return](architecture.md#observation-return)). In the second run both
  retries' in-place turns left the camera 0.94-0.96 m from G1 (the turn brings the
  forward camera nearer), so each returned to the reliable 2.4 m view instead. The
  camera range after the turn is now predicted first, so such a turn is skipped.
- **Reach violation on earlier code.** A Gazebo `standard` run (`gz_standard_run2`,
  `f6ade2f`) completed with a reach of 0.607 m, outside 0.30-0.60 m: the localization
  and map-pose errors added to a plan of 0.517 m.
- **Heading error.** That run estimated G1's heading 65 degrees off from a 0.68 m view.
  It was not reproduced or explained; the final Gazebo runs' headings were within
  10 degrees. Since 2026-11-19 a view that close is refused by the heading quality
  gate ([G1 heading](architecture.md#g1-heading)), which bounds this case but does not
  explain the wrong fit.
- **Viewpoint-dependent localization.** Gazebo's G1 error in the ward was 0.05-0.15 m
  (0.015-0.041 m in the warehouse); the head region can measure what stands behind a
  partly hidden or obliquely seen G1, and the fallback then depends on the legs.
- **LiDAR vetoes beside G1.** The reach band leaves 0.12-0.17 m between the robot and
  G1's legs, so the live LiDAR check cancels many Gazebo approaches beside G1 and the
  mission moves to another side ([backlog](../BACKLOG.md#parking-clearance-beside-g1)).
- **Narrow passages.** Nothing chooses the heading at which the robot enters a passage
  between its width (0.80 m) and its diagonal (1.05 m); orientation-aware navigation is
  not implemented ([backlog](../BACKLOG.md#orientation-aware-navigation-through-narrow-passages)).
- **No physical validation.** Parking, the basket's dimensions and G1's reach to the
  basket have not been tested on the real robot or the real G1; the
  [technical review](parking_review.md#before-a-real-robot) lists what must come first.

Future improvements are tracked in the [backlog](../BACKLOG.md#g1-detection-in-parking-scenes)
and its neighbouring parking items.

## Archived evidence

- [Parking in the hospital ward](../../archive/engineering/2026-11-18-parking-hospital-validation.md) — every ward validation run, the localization and scenario problems they exposed, and the final-code runs.
- [Departure from G1, first Isaac run](../../archive/engineering/2026-11-20-parking-departure-isaac.md) — the checked departure live, its LiDAR checks and route deviation, and the delayed stop confirmation.
- [Route revalidation cache, controlled Gazebo runs](../../archive/engineering/2026-11-20-parking-route-cache-gazebo.md) — the cache misses that starved odometry while a verified plan waited, and one controlled run of the route-local cache.
