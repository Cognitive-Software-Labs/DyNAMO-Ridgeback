# Parking detection gaps: heading_90 and no_feasible

Recorded dates: 2026-11-05
Tested revisions: `622a0c015092b7832a9b3034eb01ce6b2707384a`, `2ecaecfa564209bc34da723080a295f8b1dd9e31`
Provenance: partial

## Question and conditions

Why did G1 detection stop the P7 Gazebo scenes `heading_90` and `no_feasible`
before parking planning, and can a small correction let them test what they were
built for: rotated-G1 preferences, and a safe refusal when no pose is safe?

Every run used `tools/parking_scenarios/run_scenario.sh` (loopback DDS domain 83,
one simulator at a time, SIGTERM to its own session). The recorded HEAD was
`622a0c0` in every run; the working tree carried the changes described per run
below, committed as `2ecaecf`, and the unrelated uncommitted `--parallel`
change to `start_parking_demo.sh`. Evidence is in the untracked
`artifacts/exploration/gaps_2026-11-05/`: console, `metrics.json`, ground truth,
screenshots, a small bag (state, display, plan, map, detections, mask
measurements) and camera frames. Reach, bearing, pose error and clearance are
Gazebo ground truth, never fed to the mission; Isaac rows are estimated state.

## Why detection failed

The P7 bags held no images, so frames were saved during the runs and scored
offline with the same OWLv2 checkpoint down to 0.05. Lossless frames scored
exactly what the live detector published on the same stamps (0.531, 0.542,
0.561, 0.576); JPEG frames scored a few hundredths differently.

- G1 was fully visible in both failures (screenshots and aligned depth show a
  clean silhouette): side-on or from behind in `heading_90`, upper body above a
  bed in `no_feasible`. Neither was an occlusion or field-of-view failure.
- OWLv2 scored G1 0.27-0.68 by view; side-on, rear-quarter and partly hidden
  views sat at 0.42-0.58. The parking launch ran the generic detector floor of
  0.55, above the mission adapters' original G1 floor of 0.50 (the migration plan
  had deferred aligning them).
- A static simulated view scores the same on every frame. Search confirmation
  (three of five frames) could succeed while the robot turned, then the stopped
  view sat just below 0.55 for the whole 5 s window: 0 of 51 frames.
- The depot's walking person scored at most 0.19 as `humanoid robot` across all
  runs; every other non-G1 box stayed at or below 0.22. Higher boxes that
  ground-truth labelling first called non-G1 were G1 itself at the image edge or
  its legs.

## Corrections

- The parking launch passes `detection_threshold` 0.40 in local localization
  mode; external mode keeps 0.55 because it refuses local overrides.
- Search confirmation and the first localization keep the 0.50 G1 floor. A
  relocalization of an already localized G1 accepts the same label from 0.40
  only when that sample's map position lies within the 0.75 m sample jump of the
  known G1; a stronger sample is not position-gated.
- Observation fallback: when a turn toward G1 hit its deadline, the fallback
  sent the observation goal while Nav2 still owned the cancelled turn, and the
  guard against competing goals ended the mission. The fallback now waits for
  the release, as the retry and return paths already did.

## Runs

| Run | Code | Outcome | Detail |
|---|---|---|---|
| `gz_heading_90_baseline` | `622a0c0`, detector 0.55 | `target_localization_timeout`, stopped 5.5 m from the start | confirmed at 0.557 while turning; stopped 3.0 m from G1, 5 deg off the camera axis; 0/51 frames |
| `gz_heading_90_g1floor` | detector 0.50 only | `target_localization_timeout` at relocalization | localized from the start, heading 83.1 deg (truth 90); 16 poses became valid; back at the observation pose the static view scored 0.494 on every frame |
| `gz_heading_90` | 0.40 and hysteresis | `target_localization_timeout` at relocalization | the observation pose put the side-on G1 in front of a steel column: no box at 0.40 on 50 frames |
| `gz_heading_90_run2` | 0.40 and hysteresis | complete, `g1_right`, 1 attempt | headings 83.1, 83.7, 93.5 deg; reach 0.575 m; truth bearing -42.8 deg, 2.2 deg outside the side sector; pose error 0.004 m; basket-G1 0.185 m; LiDAR 0.125 m; back 0.036 m from the start |
| `gz_heading_90_final` | final | complete, `g1_right`, 1 attempt | headings 83.1, 83.1, 94.8 deg; the first relocalization stabilized on six 0.40-0.50 detections at the observation pose; reach 0.575 m; truth bearing -41.6 deg; pose error 0.068 m; basket-G1 0.185 m; LiDAR 0.145 m; back 0.027 m from the start |
| `gz_no_feasible_racefail` | 0.40 and hysteresis | `observation_return_failed`: previous navigation goal not released | localized, heading -179.4 deg; 49 candidates all `footprint_occupied`; 240 s exploration; turn deadline, then the race; stopped behind the pen |
| `gz_no_feasible` | final | `MISSION_FAILED` after return: `no_valid_candidate` | localized, heading -179.4 deg; 53 candidates (52 occupied, 1 unknown); 240 s exploration; turned in place and relocalized (-168.8 deg); 50 candidates all occupied; no attempt; returned, 0.384 m from the start (Nav2's 0.40 m shared tolerance); 414 s |
| `gz_standard` | final | `observation_pose_not_reached`, stopped 0.04 m from the start | stopped 0.049 m from the observation pose (limit 0.040 m) |
| `gz_standard_run2` | final | complete, `g1_left`, 1 attempt | reach 0.563 m; truth bearing 50.2 deg; table 0.59 m; G1 0.169 m; back 0.046 m from the start |
| `isaac_standard` | final | complete, `g1_front`, 1 attempt | heading -176.6 deg; LiDAR 0.264 m; returned 0.007 m (estimated); as in P7 |

Only `gz_heading_90_final` used 0.40-0.50 samples: its first relocalization
held six, none above 0.50, the case that timed out in `gz_heading_90_g1floor`.
Every other localization window held only detections at 0.50 or above.
`heading_90` completed in two of three runs on the hysteresis code; its outcome
depends on where the robot stops relative to the background. In `no_feasible` the explorer stopped the base 0.018 m
(racefail) and 0.027 m (final) from a bed and a panel while mapping beside the
pen; no parking leg ran in that scene.

Frame savers of earlier runs kept running after their runs (the runner's stop
missed them) and wrote later runs' frames into their folders. Only frames
written during each run's own interval were scored; bags, consoles and ground
truth were unaffected. The runner and saver were fixed before
`gz_heading_90_final`.

## Limitations

- One or two runs per scene; no rate. The Isaac check was one standard run.
- A side-on G1 against a matching background can still score below 0.40 at the
  observation pose; the mission then ends without parking.
- No second humanoid robot was tested; Isaac's demo world has no person.
- Ground truth is sampled about every 1-2.5 s.
