# Gazebo parking in the hospital ward: a trial of initial_test_world

Recorded dates: 2026-11-17
Tested revisions: `822ae6d8b45ba1947bed5c785d8da2a04c322463`, `23b70fff8d19661660e0880698453e12255fbedf`
Provenance: partial

## Question and conditions

Can the Gazebo parking demonstration move from Clearpath's warehouse (`depot`),
whose speckled floor hides G1 and Ridgeback in screenshots, to the clean hospital
ward Isaac already uses? Isaac's `parking_demo` is a USD layer over
`initial_test_world` that disables its authored distant G1. The trial commit
`822ae6d` added a Gazebo `parking_demo.sdf` (the same SDF without that G1 include,
checked element by element by a test), made it Gazebo's demo world, moved the
Gazebo workstation to Isaac's (0.5, -4.5) m facing -y in the south room, set the
spawn floor to the slab top (0.05 m) and opened the documentation views to 11 m.
Mission, localization, planning, navigation and safety code were unchanged.

Runs used `tools/parking_scenarios/run_scenario.sh` (loopback DDS domain 83, one
simulator, no other user's simulator running, `rviz_layout:=compact`, 1500 s
limit). Evidence is in the untracked `artifacts/exploration/hospital_2026-11-17/`
with `commit.txt` and `code.txt`; the tree's uncommitted `--parallel` option in
`start_parking_demo.sh` was present and unused. Errors are against Gazebo ground
truth sampled every 1-2.5 s.

## Scene

A plan-only preview (`scenes/`) created and verified exactly one G1 and the
worktable; the robot started in the corridor with the corridor wall between it and
G1. The views show white walls, a near-white floor and the room's exam table and
chair, their centres about 1.7 m and 0.9 m from G1.

## Runs

| Run | Code | Outcome | Detail |
|---|---|---|---|
| `gz_standard` | `822ae6d` | `MISSION_COMPLETE`, returned 0.22 m from the start | search 600 s; G1 0.100-0.116 m from ground truth, heading error 6.4 deg; `g1_left` and `g1_right` cancelled in the stopping path (LiDAR returns 0.113 m and 0.103 m from the robot, 0.38 m and 0.34 m from G1); `g1_rear` parked on attempt 3, arrival 0.027 m: reach 0.500 m, basket 0.110 m from G1's lower body |
| `gz_no_feasible` | `23b70ff` | `MISSION_FAILED`, returned 0.34 m from the start | search 1075 s; the depth check placed G1's head 4.11 m away and re-seeded G1 at (-0.05, -4.19), 0.55 m to G1's right and 0.3 m behind it, between G1 and the right panel; the position the mission used was 0.57 m from ground truth; a `g1_right` pose beyond the panel passed planning, and the LiDAR check cancelled its final leg in the stopping path; G1 was not relocalized (`target_localization_timeout`), so the mission returned without retrying; minimum ground-truth clearance 0.086 m (base to the right panel) |

The panels are `privacy_curtain` assets (alpha 0.95 in Gazebo). Why the head check
settled beside G1 rather than on it was not established.

## Outcome

The ward gives far clearer screenshots, and the standard scene parked. The
`no_feasible` safety scene did not refuse as intended: a wrong G1 position produced
a parking attempt that only the live LiDAR check stopped, and the localization
error in the standard run was 3-7 times the warehouse runs' (0.015-0.041 m on
2026-11-14). One run per scene shows the risk, not a rate. Gazebo kept the
warehouse (revert `541f912`); the trial's code stays reachable at `822ae6d`.
Moving Gazebo to the ward first needs the panel mislocalization understood.
