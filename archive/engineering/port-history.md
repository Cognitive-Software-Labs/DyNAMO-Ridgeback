# Isaac sensor and chassis investigations — September 2026

Recorded dates: 2026-09-10, 2026-09-11, 2026-09-12

Tested revisions: unknown

Provenance: partial

This is dated evidence. Missing revisions or preserved worktree inputs limit
reproducibility; no new measurements were made during archive curation.

## Scope

This record retains the detached-sensor diagnosis and September 10 audit.
The earlier SLAM experiment is preserved once in the
[July investigation](2026-07-12-isaac-slam-investigation.md).
All geometry, defaults and implementation descriptions below refer to the
dated investigations, not to the working tree.

## Lidars detached from the articulation (2026-09-11)

Isaac navigation had planned but remained pinned at zero velocity because
`collision_monitor` repeatedly saw short returns near the edge of the front
scan. The emitters were not producing a subtle timing artefact: both lidar
links were parented under `base_link`, a bare Xform with no joint into the
articulation, while PhysX rotated `chassis_link`. The chassis therefore moved
under sensors that remained fixed in world coordinates.

The decisive spin diagnostic measured the chassis turning to 89.58 degrees
while lidar world yaw stayed at exactly zero. A fixed wall's sensor-frame
bearing had `d(bearing)/d(yaw) = -0.0025` instead of the expected -1.0, and the
distance from the stationary emitter to the rotating chassis notch swept from
0.049 m to 0.519 m—straight through the phantom-return band.

Changing both UST-10LX parents in `clearpath/robot.yaml` from `base_link` to
the coincident `chassis_link` restored rigid motion without changing published
TF. After regeneration:

| check | before | after |
|---|---:|---:|
| lidar−chassis yaw after the spin | −89.58 deg | 0.000000 deg |
| scan rotation slope | −0.0025 | −1.0352 |
| returns below 1 m while spinning | up to 6/frame | 0 |
| commanded messages over 40 s | 0 | 799 |
| odom displacement | 0.000 m | 1.1453 m |

The temporary 10-degree edge mask was removed; regression tests require the
full ±135-degree scan to remain publishable. Static self-occlusion, footprint
size, assembler binning, world geometry at spawn, and simulator mode had all
been ruled out. Every SLAM and coverage result produced while the sensors were
detached was invalidated by this diagnosis.

---

**Merger audit (2026-09-10) — two bugs found and fixed before any live run.** Proven offline with synthetic scans on an isolated domain (no Isaac needed; harness pattern: rear lidar sees one wall 3.0 m straight behind, front sees nothing, so the merged bin at `π` must read 3.3922 m — `+inf` means the rear scan was dropped):

- **The merger was launched without the `/tf` remap.** `tf2_ros.TransformListener` subscribes to the *absolute* `/tf`, so a node namespace does not move it, and the whole stack publishes into `<ns>/tf`. Its buffer was therefore permanently empty, every `odom→base_link` lookup failed, and **every rear scan whose stamp differed from the front's was silently discarded** — `merged` mode degraded to front-only. Measured: stamps 10 ms apart with TF on `<ns>/tf` only → rear bin `+inf`; add a global `/tf` publisher → 3.3922 m. Fixed by adding `remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]`, with a regression test (`test_scan_merger_node_remaps_tf_into_the_namespace`) so it cannot silently come back. **This was latent in sim** — both Isaac lidars stamp off the same tick, hitting the `abs(delta) < 1e-6` static-extrinsic fast path that never consults TF — and would have bitten on real hardware, where the two lidars have independent clocks.
- **`range_max` was copied from the sensor frame into a `base_link` message.** The lidars sit ±0.3922 m off `base_link`, so a return at the front sensor's 10.00 m range_max lands at 10.39 m from `base_link` — above the declared `range_max`, so slam_toolbox discarded it. Merged mode was losing the outermost shell of returns that `front_only` keeps. Fixed by padding `range_max` with the largest lidar offset.
- **The health log overstated itself:** a TF-miss merge still incremented `paired`, so the counters read `paired=4 tf_misses=1` for four publishes of which one carried no rear data at all. Split out `rear_dropped(no odom TF)`; the same probe at that stage reads `paired=3 rear_dropped=1`.
- Not bugs, checked: the hardcoded planar extrinsics match the robot USD chain (base_link → riser_link `(0,0,0.22)` → default_mount `(0,0,0.075)` → lidar link), identity rotation throughout, so front `(0.3922, 0, 0)` / rear `(-0.3922, 0, π)` are right; bin math (1440 bins, `angle_min=-π`, nearest-wins) is right; the correction composes to `T_front←rear` off the odom chain only, never `map→base_link`. The 5 cm rear-lidar height offset the audit flagged (a gz-only workaround) was removed the same day — see improvement 2 above; both lasers became coplanar. **Superseded 2026-09-10:** the mount was also 11.6 cm too high, so the recorded corrected plane was **0.2264** above `base_link`, not the 0.3424 this line originally recorded.

The audit did not establish fresh-boot live behavior.

---

## Vendor chassis graft (2026-09-10)

The URDF importer produced a coarse body — open gaps under the deck, **no rear end panel at all**, flat untextured materials. NVIDIA ships an authored Ridgeback in the Isaac asset catalog (`/Isaac/Robots/Clearpath/RidgebackUr/ridgeback_ur5.usd`, BSD-3-Clause, Clearpath Robotics, from `ridgeback_manipulation`) whose hull matches ours to a few mm but is closed, chamfered and textured. `tools/isaac/extract_vendor_chassis.py` pulls the chassis out; `import_ridgeback_urdf.py:graft_vendor_chassis` references it and hides the 12 imported prims it replaces.

- **Geometry needed no re-referencing.** Both models bottom out at z = −0.0262 (wheel contact plane) and agree in y to 1.4 mm, so the vendor root *is* our `base_link` and the tape-measured 0.179 lidar mount carries over untouched. Verified after regen: laser frames at `(±0.3922, 0, 0.2264)`.
- **Wheels stay ours** (articulated, they spin); the vendor drives its base as one rigid body with static wheels, so taking theirs would double them. The UR5, its mount plate, the dummy joint chain and the vendor `physicsScene` are dropped — a second articulation root would fight ours.
- **Collider: `convexHull`, decided 2026-09-10.** Measured on the chassis collision mesh: true volume 0.15968 m³, `convexHull` 0.17064 (**+6.9 %**), the AABB `Cube` it replaces 0.20018 (**+25.4 %**). The hull keeps 119 verts / 234 facets against the source's 972 / 324, so it tracks the real form; what it over-claims is the underside cavity between the wheels, which the wheel cylinders already occupy and nothing else reaches. `convexDecomposition` would chase that last 6.9 % at per-step contact cost — rejected. Compare them with `tools/isaac/inspect_robot.py --compare-colliders`.
- **Self-occlusion cleared, and an earlier theory refuted.** In `src/ridgeback_autonomy_isaac/sim/isaac/usd/worlds/empty.usda` (added for this — floor slab, nothing else, so any finite return is necessarily the robot seeing itself) both lidars read **0/1081 finite bins** across the full ±135°, before *and* after the graft. The "grazing the side cover at 0.52 m" explanation for the Nav2 stall was **wrong**; that geometry never occluded the lidars.

Two traps this uncovered, both silent:

- `import_ridgeback_urdf.py` `rmtree`s the entire committed robot directory during its flatten step, which also deletes the vendored chassis and its licence. The fix stashed and restored them across the rebuild.
- Kit runs with `--/app/fastShutdown=True`, so `app.close()` **hard-exits the process** — a post-processing step appended after `import_urdf_to_usd()` returns never runs, while the script still prints `IMPORT OK` and exits 0. Anything post-import must sit inside, before the close.

**Camera + mast corrected (2026-09-10, measured).** The D435 was floating at z = 1.145 with no supporting structure. Owner's tape: bottom face **0.740 m above the top plate** → 0.280 + 0.740 = **1.020** above `base_link`, so the model had it **12.5 cm too high**; `robot.yaml` camera z 0.85 → **0.725**. Fore/aft taken as a *fraction* of the hull (78 of 98 on the tape → 0.796 × 0.9325) giving x = **+0.2716**, because the tape's 98 cm total overran both the Clearpath mesh (0.9325) and NVIDIA's asset (0.9330) — two independent sources that agree with each other, and whose **width matches the official 793 mm spec to 0.2 mm**. The mast itself is authored in `_author_camera_mast` (not in the Clearpath description at all): 37.5 mm square, on the centreline at x = +0.1955, spanning deck 0.280 → **1.095**, i.e. 50 mm past the camera's top — the D435 is **bracketed to the mast's front face**, ~45 mm standoff, not sitting on it. Mast base clears the 0.2264 lidar plane by 53.6 mm; the modelled column omits the flared base plate, so the real clearance is smaller (still above the deck, so still clear).

**Pattern worth naming: sensors here are never mounted where the obvious surface suggests.** The 2D lidars read as deck-mounted but are recessed in a body notch (−11.6 cm). The camera reads as mast-top but is bracketed to the mast's front face. Both were modelled from an unmeasured offset onto a plausible parent, and both were wrong. Measure the mounting face, and ask *how* it attaches, before authoring an offset.

**Limitation at the time:** the graft changes rendered geometry the RTX lidar raytraces, so the GT slice wants re-checking and every coverage number is void until it is.

## Pre-profile camera checks

Curation annotation, September 18, 2026: the former robot-model reference
reported a live check on September 12 with 1280×720 `rgb8` colour and `32FC1`
depth in `camera_0_color_optical_frame`, `fx=fy=631.0000005`, `cx=640`,
`cy=360`, at 30.0 Hz in simulation time. The reported base-to-optical TF
translation was `[0.280, -0.011, 1.034]`. Those optics were later superseded by
the shared nominal profiles and had no manufacturer or device-calibration
provenance.

The same reference reported a matched 1280×720 depth/cloud check with one
point per pixel and optical-Z/depth agreement across 417,280 valid pixels.
The exact date and tested revision of that cloud check were not recorded in
the extracted passage. It described a flat image-row-major cloud, not a proof
that arbitrary flat clouds could be reshaped.

These measurements were transferred from the
[pre-split documentation snapshot](https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/6805326d9c4b60b1fcf84e700e7ab724d312179d/docs/isaac/robot-model.md).
The snapshot identifies the source prose, not a tested runtime revision. No
measurements were repeated during extraction.
