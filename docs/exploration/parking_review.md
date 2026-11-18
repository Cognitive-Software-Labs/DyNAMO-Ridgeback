# Autonomous parking: technical review

A summary for review of the simulated autonomous parking mission: what it does, how
it stays safe, what has been shown and what is not. Details live in the linked
references; dated evidence lives in the archive records.

## What the mission does

Ridgeback starts away from a Unitree G1 standing at a workstation, out of the
camera's sight, and:

1. explores with the shared frontier explorer until the detector confirms G1
   (three of five frames at 0.50);
2. stops, turns its camera toward the confirmed detection if needed, and localizes
   G1 from five consistent samples, accepting a position only when depth shows G1's
   head and torso there ([G1 position check](architecture.md#g1-position-check));
3. estimates G1's heading from its forearms ([G1 heading](architecture.md#g1-heading));
4. plans parking poses whose rear-bin task point is 0.30-0.60 m from G1's axis,
   ranked G1-left and G1-right, then behind, then in front, then other; maps the
   floor around G1 first when undecided poses could change the choice
   ([G1-centred exploration](architecture.md#g1-centred-exploration));
5. checks each pose's route with Nav2, drives to a pre-parking pose and the parking
   pose with a tight goal checker, waits five seconds and returns to its start;
6. after a failed attempt, tries again from another direction, making at most
   three parking attempts in total, and reacquires G1 by turning in place or
   stepping back
   ([observation return](architecture.md#observation-return)).

Every state, its limit and its measured duration are in
[states and timing](parking_mission_states.md); RViz shows the state machine, the
360-ray parking donut and the selected pose ([parking display](parking_demo.md#parking-display)).

## Safety design

- **Nav2 ownership.** The mission owns at most one goal and releases it only on a
  terminal action result; cancellation, retries and the return wait for that and for
  stationary odometry ([goal ownership](architecture.md#mission-navigation-goal-ownership)).
- **Live LiDAR check.** During both parking legs the two raw scans are checked
  against the stopping path and the route still ahead, with a 0.05 m margin; a veto
  cancels the leg. The shared Nav2 collision monitor stays in front of the wheels.
- **Geometry.** Parking plans with the hull of Nav2's robot outline and the
  provisional basket, keeps a 0.29 m keep-out around G1 and 0.08 m from mapped
  obstacles, treats unknown cells as unsafe, and marks tabletops the LiDARs pass
  under from depth ([parking scenarios](parking_scenarios.md#robot-height-and-the-basket)).
- **Perception.** No position is planned against unless depth shows G1 there; a
  second humanoid cannot replace the chosen G1; ground truth is never an input.
- **Honest outcomes.** A refusal, a perception failure or a vetoed approach ends
  `MISSION_FAILED` with its reason, after a return home when Nav2 owns no goal.

## Validation

Both simulators run one shared autonomy stack in the same hospital ward
(`parking_demo`); Gazebo's warehouse remains available with `world:=depot`. The
[results page](autonomous_parking_results.html) features the ward runs below and lists
every recorded run with its code version and measurements; the
[scenarios page](autonomous_parking_scenarios.html) shows the eight scenes in the ward.
Both pages show hospital-ward images only; warehouse runs keep their measurements in
the results table and their screenshots in their evidence folders.

Latest ward runs (Gazebo errors against ground truth, used only offline; Isaac
estimated):

| Scenario | Simulator | Outcome |
|---|---|---|
| `standard` | Gazebo | `MISSION_COMPLETE` on the final code: G1 0.073-0.084 m and 2-6 deg off; G1's right vetoed by the LiDAR check; parked behind G1's left hip on attempt 2 (reach 0.509 m, basket 0.118 m from G1's lower body), waited, returned |
| `standard` | Isaac | `MISSION_COMPLETE` on the final code: parked at G1's right on attempt 1, waited, returned |
| `no_feasible` | Gazebo | `MISSION_FAILED` as intended on the final code: G1 0.152 m and 10 deg off, none of 258 poses fit, no attempt, returned (also on the code before the surface fallback: G1 0.125 m off, none of 255) |

An earlier Gazebo `standard` completion in the ward, before the surface fallback,
estimated G1's heading 65 degrees off from a 0.68 m view and parked with a reach of
0.607 m, outside the 0.30-0.60 m band; it is not counted as meeting the reach
requirement. Earlier warehouse runs (2026-11-14) completed `standard` in both
simulators with G1 0.015-0.041 m off and refused `no_feasible`; perception
timeouts, LiDAR vetoes beside G1 and a historical `no_feasible` false completion
(parked 1.72 m from G1) are kept in the archive records.

The reference demonstration is a video of one complete Isaac run (parked at G1's
right, five-second wait, return): `artifacts/exploration/demo_2026-11-11/isaac_standard_run3/demo.mp4`,
kept outside the repository; `artifacts/exploration/p8_2026-11-14/isaac_standard/demo.mp4`
records the merged code's Isaac run (a vetoed attempt, then G1's left).
`tools/parking_scenarios/record_demo_run.sh isaac standard OUT_DIR` records another.

## Known limitations

- **Gazebo side parking.** The reach band leaves 0.12-0.17 m between the robot's
  rear edge and G1's legs; with the camera estimate and the robot's map pose a few
  centimetres apart, the LiDAR check vetoes most Gazebo approaches beside G1
  ([backlog](../BACKLOG.md#parking-clearance-beside-g1)).
- **Search time** depends on the explorer's frontier order: from under a minute to
  about 18 min in the workstation scenes, and completed missions took 3-18 min.
- **Narrow passages.** Nothing chooses the heading at which the robot enters a
  passage between its width (0.80 m) and its diagonal (1.05 m); the ward's
  `no_feasible` bed was moved after one wedged it
  ([backlog](../BACKLOG.md#orientation-aware-navigation-through-narrow-passages)).
- **Perception.** Detection scores of a static view vary with the exact view; the
  head and torso check is qualified in simulation only, the head region can
  measure what stands behind G1, and one close view gave a heading 65 degrees off
  ([backlog](../BACKLOG.md#g1-detection-in-parking-scenes)); several G1s are kept
  apart by geometry only ([backlog](../BACKLOG.md#more-than-one-g1-in-a-scene)).
- **Basket and reach.** The basket is provisional and unmeasured, and nothing
  verifies that G1's hands reach it.
- **Return heading.** The shared goal checker accepts any heading, so the robot ends
  its return anywhere within 0.40 m of the start, facing any way.

## Reproduction

```bash
bash start_parking_demo.sh isaac rviz_layout:=compact     # or: gz; one simulator at a time
tools/parking_scenarios/run_scenario.sh isaac standard artifacts/exploration/OUT
python3 tools/parking_scenarios/scenario_metrics.py artifacts/exploration/OUT
```

[Simulation parking demonstration](parking_demo.md) has every mode and startup gate;
[parking scenarios](parking_scenarios.md) the scenes and evaluation tools.

## Group changes incorporated

The branch merged the group's `dev` at `cc86eef`: the mesh-shaped Nav2 outline and
0.05 m costmaps, goals that face their approach with a shared goal checker that
accepts any heading, the ros_gz scene-state patch, and the HUD, overlay and Isaac
shutdown fixes. Parking keeps its own tight goal checkers and adds the basket to the
outline it plans with.

## Before a real robot

Simulation results do not qualify the robot. Before any physical parking run:

- the physical command chain, stopping distance and collision-monitor gates in the
  [backlog](../BACKLOG.md#physical-command-chain-validation);
- the basket's and mast's real dimensions, and G1's reach to the basket;
- the head and torso check and G1 detection on the real D455's depth and images;
- LiDAR clearance and the camera-to-map relative error measured on hardware;
- an operator procedure for G1 nearby, and a supervised first run.
