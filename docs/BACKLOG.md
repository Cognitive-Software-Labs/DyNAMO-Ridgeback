# Documentation-backed engineering backlog

Only active, evidenced gaps belong here. Implemented behaviour belongs in the
technical references; completed experiments and migrations belong in history;
unselected alternatives belong in `docs/do_not_try_again/`. Closing an item means satisfying
its completion criteria and moving durable results to the relevant reference or
history document—not retaining a crossed-out entry here.

## Physical camera validation

**Gap.** The repository selects a D455 and has parser/description tests, but no
recorded robot-side proof of the actual device, active profiles, topics, grids,
stamps, TF ownership, or optional organized cloud.

**Completion criteria.** Execute the [hardware plan](plans/camera_hardware_validation.md)
against the approved robot setup; record device identity, effective configuration,
color/depth/camera-info topics, dimensions/encodings/rates/stamps, exact-match
observations, and optical-to-base TF; verify there is one internal-camera TF
owner. If enabling `pointcloud`, additionally prove organization, color-grid
indexing, frame, and timestamps before wiring it as a RealSense input.

**Context.** [Aligned depth](target_localization/aligned_depth.md), the
[removed D435 transform](ISSUES.md#historical-the-d435-static-publisher-removed-2026-08-31),
and `common/camera_inputs.py`.

## Camera-LiDAR calibration

**Deployment gate.** Resolve this before treating polar-profiling measurements
from the physical robot as valid. Hardware bring-up and data collection may
precede it, but deployment must not rely on camera-to-LiDAR projection results
until the completion criteria below have been satisfied.

**Gap.** Simulation has exact model transforms; a documented physical
camera-to-LiDAR calibration and clock validation is absent.

**Why it matters.** Polar mask membership projects a free-running scan through
that transform. Spatial or temporal error can select the wrong beams.

**Completion criteria.** Define and execute a repeatable target/measurement
procedure; record transform convention, uncertainty, storage/ownership, and
timestamp evidence; validate projected beams on independent poses/ranges and
under robot motion; document recalibration triggers. Do not silently tune polar
segmentation to absorb an extrinsic error.

**Context.** [Polar profiling](target_localization/polar_profiling.md) and the
[hardware plan](plans/camera_hardware_validation.md).

## Isolation validation

**Gap.** 2D/3D recipes and current defaults are implemented, but the
2026-08-28 default/depth-gate transition and cross-recipe accuracy have not been
established by a controlled current comparison, especially with deep backgrounds.

**Completion criteria.** Fix one scenario YAML and revision; include clutter,
near occluders, long backgrounds, ranges, and viewpoints; compare registered
recipes and box/silhouette controls without regenerating scenes; record accuracy,
misses/coverage, runtime, memory, and parameters; include exploration only as a
qualitative follow-up. Treat a default change as a separate decision.

**Context.** [Projective-ranging isolation](target_localization/projective_ranging.md#implemented-2d-recipes),
[euclidean-reconstruction isolation](target_localization/euclidean_reconstruction.md#implemented-3d-recipes),
and the [candidate protocol](do_not_try_again/foreground_isolation.md#evaluation-protocol).

## Occlusion characterization

**Gap.** Dedicated `interocc_*`, `objocc_*`, and `objpartial_*` scenes exist,
but current defaults have not first been characterized well enough to select a
recovery design. The [depth-clustering plan](plans/occlusion_handling.md) is a
proposal, not an approved algorithm or current miss reason.

**Completion criteria.** Run current defaults on the fixed occlusion families;
separate detector, segmentation, depth/scan availability, isolation, association,
and wrong-near estimates; identify which estimator/precision/scenes require
change; decide whether the failure merits a new recipe, a wrong-object guard, an
explicit occlusion reason, or no change. Only then approve a scoped implementation
and require non-occluded regression evidence.

**Context.** [Occlusion proposal](plans/occlusion_handling.md),
[segmentation candidates](do_not_try_again/segmentation.md), and the
[scenario gallery](benchmarking/benchmark_scenarios_v2_gallery.html).

## Upstream dependency refresh

**Gap.** `.repos` pinned every dependency to an exact commit on 2026-09-07 after
a fresh clone on a second machine failed to build. Until then it tracked branch
tips, so `vcs import` cloned whatever upstream had published rather than the
revisions the `patches/` files were recorded against. The pins are now correct
and reproducible, but frozen: no dependency has been advanced since, and one
patch is known not to survive the advance.

Measured drift at the moment of pinning:

| Dependency | Pinned | Branch tip | Behind |
| --- | --- | --- | --- |
| `clearpath_common` | `9960354` | `d59fe1c` | 39 |
| `clearpath_config` | `5c92caa` | `a5f9101` | 24 |
| `clearpath_msgs` | `5d04171` | `36b9529` | 15 |
| `clearpath_simulator` | `25997cb` | `590a451` | 13 |
| `slam_toolbox` | `ec8f763` | `02afdde` | 4 |
| `m-explore-ros2` | `86742bf` | `326cf8a` | 1 |

Checked against those tips, `clearpath_gz_customizations.patch` and
`slam_toolbox_tf_namespace.patch` still applied cleanly;
`clearpath_realsense_sim_frames.patch` did not. The conflict is upstream
`0349ed7` ("Feature: Add Realsense d405 to URDF", clearpath_common#382), which
inserts a `d405` branch at the head of the `intel_realsense` macro — the same
region the patch edits. Only the textual conflict was established; whether the
newer upstream also changes camera frame ownership, and whether the patch is
still needed at all in that form, was not investigated.

**Completion criteria.** Advance the pins as one deliberate change, not
per-dependency in response to a build error. Rebase
`clearpath_realsense_sim_frames.patch` onto the new `clearpath_common` and
re-record it, or establish that upstream now supplies the colour-optical frame in
simulation and retire it. Re-verify all three patches apply to the new pins from
a clean import. Then re-run the sim gates and confirm the mask estimators still
resolve `camera_0_color_optical_frame` rather than reporting
`TF_MISS_EXTRINSIC`. A dependency bump that is not accompanied by that evidence
should not land, because the accuracy results under `artifacts/` were all
produced against the current pins.

**Context.** [`.repos`](../.repos), the patch inventory in
[README.md](../README.md#patches-and-issue-history), and
[`camera_0_color_optical_frame` ownership](target_localization/aligned_depth.md).
