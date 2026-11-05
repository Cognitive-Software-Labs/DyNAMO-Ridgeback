# Parking retries, overhead obstacles and Gazebo reliability

Recorded dates: 2026-11-04, 2026-11-05
Tested revisions: `bc8c064a708e7c8e089834eb8ba980e3eae8725b`, `fd6d966d159d694ea83c093cd155b2d15b9d3674`, `c72805283da7f88a0b9e868681fa07ca91220b65`
Provenance: partial

## Question and conditions

Could a parking attempt that a LiDAR veto stops be retried safely, and how often
does the complete mission succeed in Gazebo? Two observations from the
[previous record](2026-11-02-parking-beside-g1.md) were also investigated: green
rays between red ones, and the robot moving left and right near G1.

Gazebo runs used `start_parking_demo.sh gz` (loopback DDS domain 83, partition
`ridgeback-parking-1010-83`) in the standard scene: G1 at `(3, 0)` facing -x, the
table at `(3, 1.05)` on G1's right. Each run was a fresh launch started by a
series script, recorded to a bag, with Gazebo ground truth taken at parking. The
working tree also held an uncommitted `--parallel` change to
`start_parking_demo.sh`, so provenance is partial. Scripts, consoles, bags,
ground truth and `metrics.txt` are in the untracked
`artifacts/exploration/reliability_2026-11-04_bc8c064/` and
`reliability_2026-11-05_{fd6d966,c728052}/` folders.

Reach is from ground truth: the task point (0.28 m behind the base) to G1's axis.
Minimum clearance is the nearest LiDAR return to the footprint on the final leg,
with the G1 gap of that leg.

## Obstacles above the LiDAR plane

The green band about 84-91° from G1's heading, on the table side, was a set of poses
between the table legs. The tabletop at 0.70-0.75 m is invisible to LiDARs at
0.18 m, but the 1.03 m camera mast would hit it. `bc8c064` adds a camera overlay
to the planning grid only: depth points 0.25-1.15 m high, within 1.6 m of G1 and
outside G1's 0.45 m body radius, mark a cell when there are at least three. On the
recorded maps those rays turn red. No isolated green ray was found in the Isaac
recording or in 11 Gazebo bags. Nav2's costmaps still do not see the tabletop.

## Left-right motion

Sideways odometry stayed at 0.004-0.009 m/s rms in Gazebo and 0.006 m/s in Isaac,
so the controller was not oscillating. The first exploration goal was about -171°
from G1's heading, 1.3 m away, and the next one about +80° on the far side. So
the motion was the explorer's nearest-first goal order plus failed in-place
turns in Gazebo. Nothing was changed.

## Retry selection

`bc8c064` retries a vetoed or aborted attempt up to three times in total. It
returns to the observation pose only after Nav2 has released the goal and
odometry has confirmed the stop. It then relocalizes G1 and replans without
candidates within 20° of a failed bearing. Its only run used all three attempts
on `g1_left`: every vetoing return lay on G1's left leg, inside the final
footprint, so moving 20° stayed on the same side.

`fd6d966` excludes a whole G1-relative side when the vetoing return lies within
0.45 m of G1's axis, or when that side fails twice. The exclusion applies only
while another side still has a valid pose. Its first run again used all three
attempts on `g1_left`. The third plan did reject 5 left poses (`failed_attempt_side`).
But the planner's own provisional pick was one of them, and resetting that
provisional selection to `valid` revived it, so it ranked first. The same reset
could revive a pose inside a failed bearing window. The side exclusion was also
skipped entirely when no failed bearing had been recorded. `c728052` fixes both,
with a regression test. In its runs 1 and 3, one failure on the left rejected
6 and 9 left poses, and the retry parked at `g1_front`. The right side, by the
table, had no valid pose.

## Reliability by revision

| Revision | Runs | `MISSION_COMPLETE` | Failures |
|---|---|---|---|
| `567168d`, Gazebo working trees (previous record) | 6 | 2 | 3 LiDAR vetoes, 1 localization timeout |
| `567168d`, Isaac, 2026-11-04 | 2 | 2 | none |
| `bc8c064`, Gazebo | 1 | 0 | 3 attempts on `g1_left` |
| `fd6d966`, Gazebo | 1 | 0 | 3 attempts on `g1_left` (revived pick) |
| `c728052`, Gazebo | 5 | 4 | 1 G1 localization timeout |

The `567168d` Gazebo rows mixed working-tree states and are not one revision's rate.

Per-run detail for `c728052`. All four complete runs returned to the start.

| Run | Outcome | Sides tried | Failed attempt | Exploration | Mission | Reach | Min clearance / G1 gap |
|---|---|---|---|---|---|---|---|
| `09-53-11` | complete | left, front | route veto (pre-parking) | 29.9 s | 205.9 s | 0.560 m | 0.172 / 0.329 m |
| `09-57-02` | `target_localization_timeout` | none | none | 24.0 s | 94.2 s | none | none |
| `09-59-01` | complete | left, front | stopping-path veto (parking) | 35.6 s | 246.3 s | 0.554 m | 0.106 / 0.322 m |
| `10-03-31` | complete | left | none | 27.1 s | 152.8 s | 0.594 m | 0.152 / 0.328 m |
| `10-06-28` | complete | left | none | 27.1 s | 156.1 s | 0.547 m | 0.111 / 0.341 m |

The failed `bc8c064` and `fd6d966` runs explored for 51.3 s, ended after 376.2 s and
411.1 s, and both returned to the start. The run that timed out ended in
`LOCALIZE_G1` without a parking attempt, and did not return.

## Limitations

- Five runs measure one scene and one seed. 4/5 is not a stable rate.
- Side poses still put the footprint close to G1's legs; final-leg G1 gaps were
  0.32-0.34 m. Left vetoes remain common, and they now cost a retry, not the mission.
- G1 localization can still time out in Gazebo, after failed in-place turns.
- The overlay sees only what the camera has seen, and only within 1.6 m of G1.
  Nav2 still plans through spaces under tabletops.
- `c728052` has not been run in Isaac.
