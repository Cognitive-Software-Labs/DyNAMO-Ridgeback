# Parking rays judged at their own poses, and G1 heading from depth

Recorded dates: 2026-11-01
Tested revisions: `18d14959a9cacb73bd42963aeeafd5c687170773`
Provenance: partial

## Questions and conditions

Why were some donut rays red although a valid pose seemed to exist along them?
And can G1's heading be estimated from the camera reliably enough to order
parking candidates?

Rays were replayed on the recorded maps of the runs in the
[progressive planning record](2026-10-30-progressive-parking-planning.md).
Heading frames were captured in one stationary Gazebo scene: G1 and the robot
were moved with Gazebo `set_pose` to 24 G1 headings from three robot poses, plus
six table scenes. Complete missions used `start_parking_demo.sh gz` (loopback
DDS domain 83, Gazebo partition `ridgeback-parking-1010-83`), with no other
simulator running. The blocked scene moved G1 and the table right after they
spawned, before the mission started, and Gazebo reported both poses.

The final code ran from the working tree before `18d1495` was committed; only
graphify and two documentation lines changed afterwards. Frames, scripts,
metrics and bags are in the untracked `artifacts/exploration/`
`g1_heading_2026-11-01` and the `parking_2026-11-01_*_gz.*` run folders, so
provenance is partial. The desktop session changed during the runs, so screen
captures of the final runs are black. The display evidence for them is
`screenshots/published_markers.png`, drawn from the recorded RViz markers, not
live RViz.

## Rays

Rays were classified from the map cells they crossed, using each cell's
candidate. That candidate has the cell's bearing, not the ray's, and the reach
band that keep-out and the budget allow (about 0.51-0.54 m with the live
allowance) is narrower than a 0.05 m cell. Against the planner's own checks
evaluated every 5 mm along each ray, on three recorded maps:

| Drawn before | Valid pose on the ray | Rays per plan |
|---|---|---|
| Red | yes | 0-5 |
| Grey | yes | 15-70 |
| Green | no, only a neighbouring cell's pose | 1-7 |

Each ray is now judged at its own poses, at both ends of the band. A valid
candidate's own ray is always green. The drawn classes matched the reference on
all 360 rays of all three maps. Every remaining red ray on the two recorded
complete-map plans checked had an occupied cell more than 0.45 m from G1 in its
footprint, so G1's own mapped body was not the sole blocker. Candidate generation
is unchanged, so a green ray may have no cell-centred candidate on it.

## Heading

There is no pose or keypoint model in the stack, and OWLv2 at its threshold
detected G1 in only 31 of 72 views, mostly from the front or back. Depth was the
usable signal: the G1 holds its forearms forward, and everything more than
0.18 m in front of its axis lies at 0.83-0.94 m height, 0.11-0.18 m to each side,
reaching 0.37 m. Nothing lies more than 0.15 m behind the axis.

| Test (against Gazebo ground truth) | Estimates | Mean error | Maximum |
|---|---|---|---|
| 24 headings, 3 robot poses | 72/72 | 6.7° | 12.9° |
| Same, with 2 cm depth noise | 72/72 | 7.2° | 13.8° |
| Table beside or across G1's front (6 scenes) | 6/6 | 3.1° | 6.9° |
| Same 72 views, forearms removed | 0/72 | n/a | n/a |
| Same 72 views, forearms removed, table edge added | 0/72 | n/a | n/a |
| G1 removed | 0/72 | n/a | n/a |
| Live missions, 5-frame agreement (10 estimates) | 10/10 | 0.9° | 2.3° |

The cue depends on this forearms-forward posture. Isaac and hardware were not
tested.

## Missions

| Scene | Headings (truth) | Exploration | Selected | LiDAR minimum, final leg (G1 gap) | Reach to G1 truth | Mission |
|---|---|---|---|---|---|---|
| Standard, G1 faces the robot | -179.0°, -177.7° (180°) | 13.2 s, checkpoint | `g1_front` | 0.129 m (0.334 m) | 0.632 m | `MISSION_COMPLETE`, 81.6 s |
| Standard, repeat | -179.0°, 179.7° (180°) | 10.7 s, checkpoint | `g1_front` | 0.209 m (0.349 m) | 0.686 m | `MISSION_COMPLETE`, 101.8 s |
| G1 faces away, table across its front | 0.0°, 0.0° (0°) | 18.3 s, front decided | `g1_rear` | 0.148 m (0.332 m) | 0.532 m | `MISSION_COMPLETE`, 94.0 s |

In the blocked scene, G1 was seen only from behind (18-21 fingertip points). Valid
rear poses did not end exploration; it ended when no front pose was undecided.
The rear selection logged that hand access is not verified. No LiDAR veto occurred
in these three runs. That does not show the separate G1-leg clearance issue is
resolved. A G1 facing away with an open front was covered by tests only, not by a
live run.

Front parking can exceed the 0.60 m reach limit against G1's true axis. Facing the
camera, G1's position estimate is pulled toward the robot by its forearms: it read
x = 2.868-2.943 against 3.0, and 3.007-3.011 when G1 faced away. The robot stopped
within 0.03 m of its goal, yet ended 0.632-0.686 m from the true axis, because the
0.03 m reference allowance does not cover that bias. The forearm tips are about
0.37 m in front of the axis, so the bin is nearer the hands than those figures.
This is open: reach limits and the position estimate were not changed.

Two earlier runs of this work ran with the rays inside the synchronous planning
call, which took 0.54 s, then about 0.46 s. That exceeded the 0.5 s odometry age
limit and restarted motion settlement. One failed with `target_stale`, and one
passed narrowly, so the rays moved to the next control tick; planning took 0.16 s
after that.
