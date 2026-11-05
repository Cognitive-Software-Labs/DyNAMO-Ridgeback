# Parking scenarios, the provisional basket and G1 detection

Recorded dates: 2026-11-05
Tested revisions: `fe60a88bcf85656dac51ff1de991eac400efda03`, `28769c2d42506f871076fb16137d90144c20427c`
Provenance: partial

## Question and conditions

Does the unchanged parking mission choose G1's left or right, then rear, then
front, and never put the mast or basket under a tabletop, across scenes built
around the real robot's payload? What happens when no safe pose exists?

Every run used `start_parking_demo.sh gz|isaac demo_scenario:=<name>` (loopback DDS
domain 83, Gazebo partition `ridgeback-parking-1010-83`), one simulator at a time,
started and stopped by `tools/parking_scenarios/run_scenario.sh` with SIGTERM to
its own session. The working tree was `fe60a88` plus the scenario, basket and tool
changes later committed as `28769c2`, and an unrelated uncommitted `--parallel`
change to `start_parking_demo.sh`. Scene definitions changed during the session;
each run folder keeps the `scenarios.yaml` it ran, except `heading_90` and
`heading_away`, whose entries never changed. Only startup's simulator check
(`backends`) was added after the last run.

Evidence is in the untracked `artifacts/exploration/scenarios_2026-11-05/`: console,
`metrics.json`, `truth_track.csv` (Gazebo ground truth at about 0.5-1 Hz), screenshots,
and a bag of mission state, parking display, plan and map. Detections and
measurements are in the bags from `left_blocked` on; scans are not recorded.

Reach, bearing, pose error and clearance are Gazebo ground truth, never fed to the
mission. Reach is the task point (0.28 m behind the base) to G1's position;
pose error is the parked base against the selected goal, so it includes the SLAM
map's offset from the world. Clearance is the smallest plan distance between a
robot part (base, mast, camera, basket) and an object collision box at an
overlapping height, or G1 as a 0.19 m lower-body circle. Isaac rows are the
mission's estimated state only.

## Robot model

Two photographs of the physical robot show a grey top-open crate on the rear deck
centreline behind the mast, green inserts, the camera bar high on the mast and a
black enclosure on the mast top. The models had the mast and camera but no crate.
No crate dimension was available, so the basket is provisional: the thesis bin of
the original `~/clearpath/robot.yaml` (outside 0.40 x 0.38 m, centre x = -0.28 m,
rim 0.36 m and interior floor 0.15-0.18 m above the deck), seated on the deck. The
inserts, the enclosure and the mast base plate are not modelled.

A full Isaac re-import with the installed URDF converter (v0.3.2) rewrote unrelated
physics layers of the robot asset, built with v0.1.3; it was discarded and only
the importer's `update_camera_support` step was applied, adding five colliders.

## Final runs

| Scenario | Sim | Outcome | Sides tried | Failed attempts | Explore / mission | Reach | Pose error | Min clearance (pair) | Returned |
|---|---|---|---|---|---|---|---|---|---|
| `standard` | gz | `MISSION_FAILED` | left | route veto; retry G1 not relocalized | 32 / 172 s | - | - | 0.188 m (base-table) | yes |
| `worktable` | gz | complete | left, right, rear | two route vetoes | 19 / 174 s | 0.485 m | 0.034 m | 0.095 m (basket-G1); table 0.453 m | yes |
| `right_blocked` | gz | complete | left | - | 43 / 205 s | 0.543 m | 0.067 m | 0.149 m (base-panel) | yes |
| `left_blocked` | gz | complete | right, rear, front | stopping-path, route vetoes | 18 / 175 s | 0.559 m | 0.049 m | 0.169 m (basket-G1) | yes |
| `both_sides_blocked` | gz | `MISSION_FAILED` | rear, front | route, stopping-path vetoes; then `no_valid_candidate` | 31 / 170 s | - | - | 0.313 m (base-panel) | yes |
| `restricted` | gz | complete | rear | - | 20 / 78 s | 0.508 m | 0.052 m | 0.117 m (basket-G1); objects 0.195 m | yes |
| `no_feasible` (pen) | gz | `MISSION_FAILED` | - | G1 not localized | 1 / 9 s | - | - | 0.688 m | stopped at start |
| `heading_90` | gz | `MISSION_FAILED` | - | G1 not localized | 442 / 452 s | - | - | 2.348 m | stopped |
| `heading_away` | gz | complete | left, right, rear | two stopping-path vetoes | 29 / 269 s | 0.513 m | 0.023 m | 0.123 m (basket-G1) | yes |
| `standard` | isaac | complete | front | - | 28 / 174 s | - | 0.018 m (estimated) | LiDAR 0.283 m | yes |
| `right_blocked` | isaac | complete | front | six side routes rejected before driving | 37 / 268 s | - | - | LiDAR 0.278 m | yes |
| `left_blocked` | isaac | complete | right | - | 14 / 135 s | - | - | LiDAR 0.281 m | yes |

Where the side truth bearing is reported, `right_blocked` parked 43.5° from G1's
heading, just outside the 45° side sector by ground truth, while the estimated
heading and position put it inside. In `restricted` the donut rays beside G1 were
red: the camera overlay marked both tabletops, which the LiDARs see only as legs,
and no robot part came within 0.195 m of any object. The `worktable` table stayed
0.453 m from the base. Every attempt that a LiDAR veto stopped was retried away
from its direction or side. No run drove into an object or under a tabletop.

Isaac `right_blocked` parked in front: side poses were valid, but the first six
route checks were rejected (`path_footprint_occupied`), so front was the first
class with a clean route. The Isaac runs used the new basket asset and are the
first Isaac runs of the retry code; none needed a retry.

## Layouts that did not reach planning

| Run | Layout | Outcome | G1 detected |
|---|---|---|---|
| `gz_worktable_sideon` | G1 side-on (yaw 90), table ahead | localization timeout | 0.2% of frames, 0/51 while localizing |
| `gz_restricted_cabinet` | tables at ±1.0 m, 1.9 m cabinet behind G1 | 900 s search, no confirmation | 2 of 8869 frames |
| `gz_restricted_tables1m0` | same, low bed behind | localization timeout | 7.7%, 0/52 |
| `gz_restricted_tables1m2` | tables at ±1.2 m | localization timeout | 19.6%, 0/52 |
| `gz_no_feasible_cabinet` | 0.70 m corridor of panels, cabinet behind | stopped by the operator after 135 s of search | - |
| `gz_no_feasible_corridor` | same corridor, low bed behind | localization timeout | - |
| `gz_no_feasible_poles` | beds beside G1, two poles in front | localization timeout | 27%, 0/52; scores 0.63 falling to 0.55 |
| `gz_worktable_lockclash` | - | not started: the previous demo still held the lock | - |

With open surroundings the detector found G1 in 28-38% of frames with scores up to
0.75; in four completed runs every frame of the successful 5 s localization
window held a detection (28-31 frames). With G1 side-on, or with tall objects beside or behind it, recall fell
below 8% or scores stayed at the 0.55 threshold; localization then saw none and
the mission ended `MISSION_FAILED` before planning, without driving. The final
`no_feasible` pen failed the same way (scores 0.55-0.57), so the no-safe-pose path
was not reached there. `both_sides_blocked` reached it after two failed attempts:
`no_valid_candidate`, bounded, with a return to the start.

## Limitations

- One run per scenario; no rate. The basket, enclosure and G1's hand reach are not
  measured, so basket access is judged by the 0.30-0.60 m reach band only.
- The intended no-feasible scene was not demonstrated; `restricted` and
  `no_feasible` do not fit Isaac's corridor world and were not run there.
- Ground truth is sampled about every 1-2.5 s and can miss a brief closer approach.
- Gazebo's walking actor and lighting were not controlled; detection may vary.
