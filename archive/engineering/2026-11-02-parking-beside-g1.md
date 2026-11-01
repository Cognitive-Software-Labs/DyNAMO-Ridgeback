# Parking beside G1: priorities, body axis and leg clearance

Recorded dates: 2026-11-01, 2026-11-02
Tested revisions: `567168d84a0306b53a5c1075f17cdee552d11d83`
Provenance: partial

## Question and conditions

The mission must prefer G1's left and right, then behind it, then in front, and
the bin must end 0.30-0.60 m from G1's body. Front parking had ended 0.632-0.686 m
away in the [previous record](2026-11-01-parking-rays-and-g1-heading.md).

All runs used `start_parking_demo.sh gz` (loopback DDS domain 83, Gazebo partition
`ridgeback-parking-1010-83`) in the standard scene: G1 at `(3, 0)` facing -x, table
at `(3, 1.05)`. So G1's right (+y) is the table side. Each row below was the
working tree at that point. The last row is the final code, except that one test
was corrected before commit. Bags, including both LiDARs from the second run on,
console captures and Gazebo ground truth are in the untracked
`artifacts/exploration/parking_2026-11-0*` folders named in the table, so
provenance is partial. Reach is from Gazebo ground truth: the task point at the
parked pose to G1's true axis.

## Runs

| Folder | Code | Selected | Outcome |
|---|---|---|---|
| `..11-01_23-58-45` | new priorities, body axis | `g1_left` | pre-parking veto `lidar_obstacle_on_route`, returned, `MISSION_FAILED` |
| `..11-02_00-02-28` | same | none | second localization timeout, `MISSION_FAILED` |
| `..11-02_00-04-18` | same | `g1_left` | pre-parking veto `lidar_obstacle_on_route`, returned, `MISSION_FAILED` |
| `..11-02_00-12-04` | same, veto diagnostic | `g1_front` | `MISSION_COMPLETE`, reach 0.549 m |
| `..11-02_00-17-15` | both sides evaluated | `g1_left` | final-approach veto, returned, `MISSION_FAILED` |
| `..11-02_00-23-00` | final | `g1_left` | `MISSION_COMPLETE`, reach 0.562 m |

Final run: G1 heading 178.4°, bin 53° to G1's left of its heading, stop 0.027 m
from the goal, lowest LiDAR return on the final leg 0.130 m, nearest forearm tip
0.31 m from the task point. One complete side mission does not establish a rate.

In `00-12-04` exploration stopped once any side pose was valid: the only one, by
the table, failed its route, and G1's unmapped left was never evaluated. Same-class
positions now count as worth mapping.

The localization timeout in `00-02-28` was the detector reporting no G1 for all 52
frames at the observation pose, as in the
[reacquisition record](2026-10-30-gazebo-g1-reacquisition.md). All 4 in-place
relocalization attempts, in 3 of these runs, also failed, from 2.15-2.42 m, and fell
back to the observation pose. The cause was not examined. No in-place relocalization
has succeeded live so far.

## Reach

The position estimate is the centroid of visible surface points, so it depends on
G1's heading: it read x = 2.868-3.011 against a true 3.0 (in a map that itself
drifts a few centimetres). The middle of the legs' extent, corrected 0.03 m along
the view for their hidden far side, was 0.025 m from ground truth on average and
0.049 m at most over 72 views, 0.027 / 0.036 m in the six table scenes, from any
seed. It replaced the estimate in the planner; the keep-out reference stayed
generic. Both complete runs ended inside the band.

## Leg clearance

The keep-out is a circle, but G1 is wider than deep, so a side pose sits closer to
the legs than a front pose at the same reach. Against recorded LiDAR returns in
`00-17-15`, left poses had 0.064-0.141 m of clearance (first percentile), while the
map rated them all about 0.13 m, so the vetoed pose (0.084 m) had won on path
length. Counting the legs in the clearance score separates them. It is a quality
score only.

The final-approach veto was traced to a return at map `(2.912, -0.264)`, on G1's
left leg. A clearance of about 0.1 m leaves little room for the monitor's 0.05 m
margin and the 3.6 cm LiDAR noise. This remains open: the band's top (0.54 m
planned) bounds the clearance available beside G1.

The two pre-parking `lidar_obstacle_on_route` vetoes were not explained. Running
the mission's own `clearance_reason` on all 378 recorded scan pairs of `00-04-18`
did not reproduce one. The closest return to the remaining route was G1's leg
against the final footprint, about 0.09 m clear, and the route's recorded points
carry no headings, so the replay is not exact. The monitor now logs the return
behind a veto, so a recurrence can be identified. These vetoes did not recur in
the last run, which proves nothing about their rate.

Hand access remains unverified. Nothing in the mission models a grasp. A
forearm-tip distance of 0.31 m says only how far G1 would have to reach.

The no-feasible-pose return was exercised by tests only. The return leg itself ran
live after each veto. Isaac was not run.
