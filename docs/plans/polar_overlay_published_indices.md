# Route the 2D LiDAR overlay panel through the published beam indices

Status: **implemented 2026-09-10, not yet run in sim.** Written the same day as a
handoff off `f70482c` on `g1-distance-benchmarks`; the change list below is done
in full, the acceptance test below is not. What was built:

- `msg/PolarBeams.msg`, `POLAR_BEAMS_TOPIC = debug/target/polar_beams`,
  `build_polar_beams_message` / `union_beam_indices` / `polar_beam_booleans`
  in `common/messages.py`.
- `mask_measurement_node`: `polar_beams_topic` parameter, publisher beside
  `ray_marker_pub`, `ray_marker_records` renamed `polar_beam_records` and now
  allocating when **either** consumer is subscribed, publish beside the rays.
- `overlay_node`: subscribes to `PolarBeams`, caches beams by measurement stamp
  and scans by `(frame_id, stamp)`, `project_scan` became
  `project_published_scan` and projects the scan the message names.
- `core/rendering.py`: `polar_highlight_beams` and `lidar_select_mask` deleted,
  `scan_points_optical` dropped from `render`/`build_panel`/`make_lidar_panel`,
  the stored `mask_gate` attribute gone (the constructor argument stays for
  `select_panels`). `draw_scan_points` now draws **two** states in the
  `common/markers.py` ray colours, closing (f), and draws the in-view beams
  plain when the highlight cannot be trusted.
- Tests: 726 pass from the repo root, including a new `test_overlay_node.py`.

**Still open:** the four acceptance steps under *Verification*. Nothing has been
run in sim, so items 2, 3 and 4 — the three defects this exists to fix — are
argued from unit tests, not observed on a running stack.

## Why

Polar profiling's beam selection is computed **twice, in two processes, by two
different implementations**:

1. `perception/target_localization/mask_measurement_node.py` runs the real one.
   `fill_path_measurements` calls `localize_projected_polar_profiling(...,
   **polar_isolation)` per detection and collects a `PolarBeamRecord` for each.
   Those records drive the 3D RViz rays on `visualization/target/polar_rays`.
   This is the selection the published measurement came from.
2. `perception/target_localization/overlay_node.py` — a **separate node** —
   renders the 2D composite. Its LiDAR panel calls
   `core/rendering.py:polar_highlight_beams`, which re-derives the selection
   from scratch: its own mask, its own scan, its own projection, its own call to
   `segment_range_profile` + `merge_near_band`.

The second one is not a copy of the first. It diverges in six ways, and only the
first is new:

**(a) Parameters — live defect since `192cdd4`.** `polar_highlight_beams` calls
`segment_range_profile(planar_range_m)` and `merge_near_band(runs,
planar_range_m)` with no keyword arguments, so it always uses
`RANGE_JUMP_M_DEFAULT = 0.30` and `RANGE_BAND_M_DEFAULT = 0.35`. This was
harmless while nothing could pass anything else. It no longer is: all three
isolation settings became node parameters and launch arguments in `192cdd4`, so
a sweep run with `polar_range_band_m:=0.75` moves the estimate and the 3D rays
while the 2D panel keeps highlighting the default band.

**(b) Union versus per-detection.** `lidar_select_mask`
(`core/rendering.py:263`) builds **one** selector for the whole frame — the
silhouette union (`published_mask`) under a silhouette gate, otherwise
`rasterize_batch(batch)` over every box — then runs **one** range profile across
it and keeps **one** nearest band. The estimator runs per detection against that
detection's own `MaskRegion`. Two robots at different ranges: the estimator
ranges both, the panel blanks the far one. Independent of (a); the docstring
admits it ("Assumes one near object across the mask union").

**(c) No sufficiency floor.** `polar_highlight_beams` has no `min_valid_rays`
check, so it happily highlights a one-beam merge that the estimator rejected
with `TOO_FEW_RAYS_MERGED`. Not a corner case: after the 10 m cap was removed
(`1021909`) that reason went from 0/2774 to 201/11256 occurrences.

**(d) A second copy of the pixel test.** The estimator selects through
`select_beams` against the ROI-native `MaskRegion`. The overlay rasterizes to a
full grid and does its own `np.rint` plus frame-bounds test
(`core/rendering.py:420-428`). The two are kept in agreement by hand, the same
way `select_bbox_beams` is documented to mirror `mask._fill_box` exactly.

**(e) A different scan.** The overlay does its own TF lookup at the
*measurement* stamp and projects **its own** `latest_scan_msg`. The mask node
uses the scan it matched within `scan_match_tolerance_s`. Nothing makes those
the same scan.

**(f) No miss reasons.** The panel cannot distinguish "selected but discarded"
from "never selected", which is the distinction the 3D ray layers exist to show.

Fixing (a) alone leaves (b) through (f). That is the argument for publishing the
indices rather than forwarding the parameters.

## Rejected alternative

**Forward `polar_range_jump_m` / `polar_range_band_m` / `polar_min_valid_rays`
to `overlay_node` as three more parameters.** Cheap — no new message, no stamp
matching. Rejected because it fixes only (a), preserves a second implementation
of the algorithm that must be kept in sync by hand forever, and introduces a
footgun where a sweep has to set the same value on two nodes or silently
diverge again. If this is later adopted anyway, record it in
`docs/do_not_try_again/` with the reason.

## Design

Publish the indices the estimator actually reduced; make the panel a consumer.

### Message

New `msg/PolarBeams.msg`:

```
std_msgs/Header header          # detection/measurement stamp: matches the rendered frame
builtin_interfaces/Time scan_stamp   # the scan these indices index into
string scan_frame_id
uint32 beam_count               # len(scan.ranges) when recorded, for validation
uint32[] selected               # union over detections, before the range band
uint32[] merged                 # union over detections, what was reduced
```

Design notes for whoever implements it:

- **Two stamps, deliberately.** The header carries the measurement stamp so the
  overlay can key it exactly like `match_mask_debug` already keys the silhouette
  artifact. `scan_stamp` names the array the indices are valid against — without
  it the overlay would apply the indices to whatever scan happened to be latest,
  which is silently wrong rather than visibly wrong.
- **`beam_count` is a hard guard, not decoration.** If it disagrees with the
  cached scan's length, drop the highlight and draw plain points. Never
  highlight against a mismatched array.
- **Union across detections, not nearest-only.** The 3D rays draw only the
  nearest record because four estimate rings plus three ray layers per robot is
  unreadable. A 2D panel has no such clutter problem, so the panel showing every
  detection is strictly better than today's union-of-masks-single-band, and it
  fixes (b). This is an intentional difference from the marker behaviour and
  must be written down in the reference doc, not left to be rediscovered.
- **`selected` costs one array and buys the dropped-versus-used distinction**
  the 3D layers have, closing (f). Include it even if the first version of the
  panel only draws `merged`.
- ROS messages have no ragged arrays, so a per-detection version would need
  CSR-style offset arrays (`selected_offsets`, `merged_offsets`,
  `in_bbox_offsets`, plus `detection_index`). That is the upgrade path **if** the
  panel ever needs per-detection colour. Do not build it speculatively.

### Change list

1. **`msg/PolarBeams.msg`** — new, as above. Add to
   `rosidl_generate_interfaces` in `CMakeLists.txt:12` and add
   `builtin_interfaces` to its `DEPENDENCIES` alongside `std_msgs`.
2. **`perception/target_localization/contracts.py`** — add
   `POLAR_BEAMS_TOPIC = 'debug/target/polar_beams'`. The `debug/` prefix, not
   `visualization/`: this is a machine-readable artifact for another node, the
   sibling of `MASK_DEBUG_TOPIC`, not something RViz renders directly.
3. **`common/messages.py`** — add `build_polar_beams_message(records, scan_msg,
   header)`: union and sort the `selected` and `merged` index sets across the
   records. Put it here next to `build_measurements_message`, not in
   `markers.py`, which owns RViz `Marker` construction.
4. **`perception/target_localization/mask_measurement_node.py`**
   - declare `polar_beams_topic`, defaulting to the contract constant;
   - create the publisher inside the same `if self.needs_scan:` block that
     creates `ray_marker_pub` (line 515);
   - `ray_marker_records()` (line 833) currently allocates records only while
     the *ray* topic has a subscriber. Extend it to allocate when **either**
     publisher has one, and rename it — it is no longer about ray markers.
   - at the publish site (line 821), publish the beams message from **all**
     records, next to the existing `nearest_beam_record(...)` ray publish. Both
     read the same list; neither should recompute anything.
5. **`perception/target_localization/overlay_node.py`**
   - subscribe to `PolarBeams` under the existing `if self.wants_polar:` block;
   - cache incoming messages by header stamp in a 32-deep `OrderedDict` and
     re-render on arrival, copying `mask_debug_callback` (line 225) exactly —
     including the "re-render if this stamp matches the pending measurement"
     tail, since the beams message will lag its measurements message the same
     way the silhouette does;
   - **also cache scans by stamp.** Today only `latest_scan_msg` is kept, which
     cannot satisfy `scan_stamp`. Same bounded-cache pattern;
   - `project_scan()` (line 305) must project the scan named by the matched
     beams message rather than the latest one, and return `None` for the
     highlight when that scan is not cached — degrade to plain points, never to
     a wrong highlight;
   - pass the resulting per-beam boolean into `renderer.render(...)`.
6. **`perception/target_localization/core/rendering.py`**
   - `make_lidar_panel` takes the highlight as an argument and passes it
     straight to `draw_scan_points`;
   - **delete `polar_highlight_beams`** (line 394) and **`lidar_select_mask`**
     (line 263), plus the now-unused `segment_range_profile` / `merge_near_band`
     imports;
   - `scan_points_optical` threads through `render` → `build_panel` →
     `make_lidar_panel` for the sole benefit of the deleted function. Drop it
     from all three signatures and from the `overlay_node` call site;
   - `self.mask_gate` (line 150) becomes dead once `lidar_select_mask` goes —
     line 272 is its only reader. The **constructor argument** must stay, because
     `select_panels` (line 153) still needs it, but the stored attribute should
     go. `MASK_GATE_SILHOUETTE` may become an unused import; check.
   - `rasterize_batch` stays — `make_box_mask_panel` (line 316) still uses it.
7. **Tests**
   - delete `test_polar_highlight_beams_keeps_near_band_only`
     (`test/test_rendering.py:148`) and `test_polar_highlight_beams_none_mask_is_empty`
     (line 173);
   - add: records → message → boolean round-trip; the panel draws the highlight
     it was handed; a `beam_count` mismatch produces no highlight rather than a
     shifted one; the overlay projects the scan named by `scan_stamp` and not
     the newest one; records are allocated when only the beams topic is
     subscribed;
   - run the suite **from the repo root** or `msg` fails to import.
8. **Docs**
   - `docs/target_localization/polar_profiling.md` §7 — replace the "Known
     divergence — now live" block with the published-indices contract, and amend
     the nearest-only paragraph to state that the 2D panel now shows every
     detection while the 3D rays stay nearest-only, with the reason.
   - check whether `docs/project/context.md` enumerates the message set or the overlay's
     inputs; update if so.

## Verification

A new `.msg` regenerates interfaces, and node scripts are `install(PROGRAMS)`
copies — **`colcon build` before running anything**, or the running node is the
old file against a new message and the mismatch will look like a logic bug.

Before any sim run: `glxinfo | grep "OpenGL renderer"` (`llvmpipe` means Gazebo
is CPU-rasterizing and the camera drops to 4 Hz; RTF does not reveal it), and
`pgrep -af "gz sim"` for orphaned servers from a previous benchmark.

The acceptance test is the defect itself:

1. Bring up with `estimators:=polar_profiling` and RViz showing both the overlay
   image and the ray markers. Confirm the panel highlight and the 3D used-ray
   layer agree beam for beam at defaults.
2. Relaunch with `polar_range_band_m:=0.75`. **The panel highlight must change.**
   Today it does not — this is the whole point.
3. A scene with two targets at different ranges: the panel must highlight both
   bands. Today the far one is dropped by the union band, closing (b).
4. A frame where the estimator returns `TOO_FEW_RAYS_MERGED`: the panel must
   show the beams as selected-but-dropped rather than used, closing (c).

## Risks

- **One extra topic at scan rate.** Gated on subscriber count exactly like the
  rays, so it costs nothing unsubscribed; the payload is two `uint32` arrays of
  a few tens of entries.
- **Stamp matching can leave frames unhighlighted** when the named scan has
  aged out of the cache. Acceptable and required: the failure mode must be a
  missing highlight, never a misaligned one.
- **A new message is a rebuild dependency** for anything in the workspace with a
  stale build. Note that several sessions share this working tree — recheck
  `git status` before blaming a failure on this diff.
