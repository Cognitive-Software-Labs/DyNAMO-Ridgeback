# G1-relative parking order: offline replay on the ward maps

Recorded dates: 2026-11-19
Tested revisions: `a7b790eb2991732e00562416dc0ea36ed11d0e0f`
Provenance: partial

## Question and method

Commit `a7b790e` ranks parking poses G1's right, left, front, rear, then
`g1_other` (diagonal or ambiguous), where `875b3af` ranked both sides equally, then
rear, front and other, and checked at most six routes per class. Does the new order
choose differently on the recorded hospital-ward maps, and does it ever pick a
lower class while a higher one has a safe route?

`tools/parking_scenarios/replay_parking_order.py` loaded both planners side by side
and replayed, for each run, the map and the G1 estimate and heading of the first
plan that reached route checks, with the last observation goal as the start. No
simulator ran. Nav2 is not available offline, so routes were judged by an
approximation: the pre-parking pose reachable through free mapped cells at least the
footprint's inscribed radius from obstacles and unknown cells, and the mission's own
swept-footprint and keep-out check on the straight final approach. Route outcomes
and lengths therefore differ from Nav2's; the order of classes does not depend on
them. Provenance is partial for that reason, and because the runs were recorded on
older code with its own G1 estimates.

## Recorded headings

Fourteen ward runs of 2026-11-17 to 2026-11-19; two never planned
(`hospital_2026-11-18/gz_no_feasible`, `isaac_standard`). Bearing is the selected
task point's bearing from G1's heading.

| Run | Heading (deg) | Valid poses by class | Old and new selection | Route checks (new) |
|---|---|---|---|---|
| `hospital_2026-11-17/gz_standard` | -95.2 | right 6, left 2, rear 11, other 3 | `g1_right`, -132 deg | 6 |
| `hospital_2026-11-17/gz_no_feasible` | -88.6 | right 10 | `g1_right`, -65 deg | 6 |
| `hospital_2026-11-18/gz_standard` | -96.0 | right 6, rear 10, other 5 | `g1_right`, -128 deg | 6 |
| `hospital_2026-11-18/gz_standard_run2` | -155.2 (the 65 deg error) | right 17, rear 1, other 4 | `g1_right`, -48 deg | 6 |
| `hospital_2026-11-18/gz_standard_final` | -96.0 | right 6, rear 9, other 4 | `g1_right`, -129 deg | 6 |
| `hospital_2026-11-18/gz_no_feasible_run2`, `_final` | -87.8, -80.0 | none | refused (`no_valid_candidate`) | 0 |
| `hospital_2026-11-18/isaac_standard_run2` | -86.7 | right 5, left 1, rear 12, other 4 | `g1_right`, -130 deg | 5 |
| `final_demo_2026-11-19/isaac_standard` | unknown | approach side 13, front 5 | `approach_side` | 6 |
| `reobservation_2026-11-19/isaac_standard` | -88.8 | right 5, rear 10, other 5 | `g1_right`, -132 deg | 5 |
| `reobservation_2026-11-19/isaac_close_start` | -86.5 | left 2, rear 12, other 3 | `g1_left`, 115 deg | 2 |
| `reobservation_2026-11-19/gz_standard` | -94.5 | right 4, rear 7, other 2 | `g1_right`, -116 deg | 4 |

Old and new selected the same pose in every run. In most maps only one side had
valid poses; in the two with both, the left poses lost to the right on reach and
clearance under the old order too. The first batch of the top class always held a safe
route, so neither the later batches nor the 16-check budget were used. The
`approach_side` run had no heading and kept the approach-relative ranking.

## Counterfactual headings

The same ten maps and G1 positions with G1 turned to every 30 deg (144 cases; G1
was not observed so, and the worktable stays where it was):

- G1's right had a safe route in 64 cases: the new order chose it in all 64, the old
  in 59; in the other 5 the old chose the left. In two of those 5 the right route
  was the longer one (2.92 against 1.00 m, 2.15 against 1.40 m).
- The new order never chose below the highest class with a safe route, and never
  refused where the old one selected.
- All 24 cases without a selection are the two `no_feasible` maps, where no pose
  passes the geometric checks.
- At most 6 routes were checked in any case, old or new.

The 15 deg heading margin relabelled none of 6,624 valid poses (10 maps x 36
headings) against the old sector labels. A 20 deg margin would relabel 736 of them
as `g1_other`, among them right-rear poses about 130 deg from the heading such as
those selected above; 25 and 30 deg relabel 1,472 and 2,208.

## Limits

The ward maps do not exercise the batch continuation or the route-check budget;
`test_parking_feasibility.py` covers them with a fake planner. Route feasibility
here is an approximation of Nav2, and no run has used the new order live.
