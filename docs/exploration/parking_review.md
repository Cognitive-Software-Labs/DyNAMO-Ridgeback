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

Both simulators run one shared autonomy stack. The
[results page](autonomous_parking_results.html) lists every recorded run with its
code version; the [scenarios page](autonomous_parking_scenarios.html) shows the eight
scenes.

Latest runs on the merged code (Gazebo errors against ground truth, Isaac estimated):

| Scenario | Simulator | Outcome |
|---|---|---|
| `standard` | Isaac | `MISSION_COMPLETE`: G1's right vetoed by the LiDAR check, parked at G1's left on attempt 2, waited, returned |
| `standard` | Gazebo | `MISSION_COMPLETE`: G1 0.015-0.041 m from ground truth, both sides vetoed, parked behind G1 on attempt 3 (reach 0.479 m, basket 0.089 m from G1's lower body), returned |
| `no_feasible` | Gazebo | `MISSION_FAILED` as intended: G1 0.025 m from ground truth, no pose fits, no attempt, returned |

Earlier Gazebo runs on the same scene also ended in perception timeouts, LiDAR
vetoes beside G1, and two planning-gate failures fixed during the merge; the
historical `no_feasible` run that parked 1.72 m from G1 is kept as a false completion.

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
  about 18 min in the workstation scenes, and completed missions took 10-18 min.
- **Perception.** Detection scores of a static view vary with the exact view; the
  head and torso check is qualified in simulation only
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
