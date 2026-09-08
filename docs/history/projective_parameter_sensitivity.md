# Projective Ranging — Sensitivity of the Ungrounded Constants

**Scope:** eight numeric constants on the projective-ranging path
(`docs/target_localization/projective_ranging.md`) that had no traceable
justification — no measurement, no spec sheet, no recorded experiment, and in
most cases no comment. They are round numbers someone picked. This document
records, for each one, how much it actually moves the distance estimate, the
evidence, and whether a grounding now exists.

Opened 2026-09-07. Evidence below is drawn from `artifacts/benchmarks/`
(45 `run.json`, 31 `projective_ranging` blocks, 35,641 status observations),
from the shipped simulation assets, and — for the axes no archived run varies —
from a sweep that is **not yet run**; §5 says exactly which conclusions are
still open.

Out of scope by decision:

- **`ROBOT_FRONT_OFFSET_M = 0.25` is correct and untouched.** It looks wrong
  next to the nav2 footprint (base_link→front is 0.480 m,
  `config/nav2_params.yaml`), but it is applied identically to the estimate and
  to ground truth (`true_forward_m` is `spawn_x − 0.25` in every trial CSV), so
  it cancels in every error metric.
- **Choosing a default isolation recipe.** The archive shows `otsu` ahead of
  the shipped `nearest_mode_histogram` on the full set, but selecting a default
  is a design decision, not parameter grounding. Measured and reported here;
  no default changed.

---

## 1. The constants

### 1.1 Where each one acts

All eight sit on one path: box-gated projective ranging, from a detector box to
one distance. Following that path in order, with the constant that governs each
step (full contract in
`docs/target_localization/projective_ranging.md`):

1. **Gate the detector box.** A box covering more than **#7
   `MAX_BOX_FRAME_FRACTION`** of the frame is dropped before any masking.
   *Exists to prevent:* OWLv2 occasionally boxes the whole scene at close
   range; masking with that box isolates the background wall and poisons the
   estimate, so the detection is skipped rather than measured against the room.
2. **Rasterize the box into a `rect` mask** and read the aligned depth frame at
   the detection stamp.
3. **Clean the masked depths.** Non-finite, non-positive and out-of-range
   samples are dropped. The ceiling is the tighter of the working gate and the
   depth source's declared `usable_max_m`; for the monocular source that
   ceiling is the checkpoint's maximum depth taken at **#8
   `MONOCULAR_USABLE_RANGE_FRACTION`**. **#6 `MAX_RANGE_M`** is the default
   ceiling for callers that supply none.
   *Exists to prevent:* treating a reading the sensor cannot actually resolve
   as if it were a measurement.
4. **Isolate the foreground.** A `rect` mask covers object *and* background, so
   the depths are multi-modal and a plain median can land between the peaks or
   on the background. The recipe separates them. For the default
   `nearest_mode_histogram`, in three sub-steps:
   - **4a. Histogram** the valid masked depths at **#3
     `NEAREST_MODE_BIN_WIDTH_M`**. *This sets the resolution at which surfaces
     become distinguishable* — too coarse merges the object with what is behind
     it, too fine splinters one surface across many bins.
   - **4b. Anchor** on the centre of the **nearest** bin holding at least **#2
     `NEAREST_MODE_MIN_BIN_FRACTION`** of the samples. *Exists to prevent:*
     anchoring on a handful of stray near pixels — noise, an edge bleeding onto
     a nearer surface — by demanding a bin be *populated* before it can count as
     a real surface. If no bin clears the floor the code falls back to the
     nearest **non-empty** bin, deliberately not the global mode, because at
     range the most populated bin is the background wall.
   - **4c. Keep** every pixel within **#1 `NEAR_SURFACE_BAND_M`** of that
     anchor. *This is the depth window that defines "the object"*: wide enough
     to hold the target's own front-to-back extent, narrow enough to exclude
     what is behind it.

   The alternative recipe `otsu` replaces 4a–4c with a threshold maximizing
   between-class variance over a histogram binned at **#4
   `OTSU_BIN_WIDTH_M`**, keeping everything on the near side.
5. **Guard on sufficiency.** Fewer than **#5 `MIN_VALID_SAMPLES`** surviving
   pixels reports a miss rather than an estimate — `ISOLATION_EMPTY` on the
   `rect` branch, `TOO_FEW_VALID_PIXELS` on the `tight` branch.
   *Exists to prevent:* publishing a distance derived from a handful of pixels,
   where a single outlier moves the median.
6. **Reduce.** The median of the surviving depths is `Z`; the centroid of the
   surviving pixels is the pixel to deproject.

Two groupings are worth holding onto, because the measurements later turn on
them. **#1, #2 and #3 all shape one decision** — which depths belong to the
object — and they are therefore coupled: #3 sets how many samples land in a
bin, #2 sets how many a bin needs, and #1 sets how far from the winner to
reach. **#5, #6, #7 and #8 are guards**, not estimators; each defines an
operating envelope and does nothing at all inside it. That is why the two
groups needed different questions asked of them: for the first, *what does
moving it do to the answer*; for the second, *how far is the edge*.

### 1.2 Verdicts

| # | Constant | Value | Home | Verdict |
|---|---|---|---|---|
| 1 | `NEAR_SURFACE_BAND_M` (as `band_m`) | 0.35 | `core/ranging_defaults.py` | **Wrong.** 0.75 removes 88 % of the tail at no cost — §4.1 |
| 2 | `NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT` | 0.05 | `core/depth_common.py` | **Right, and load-bearing.** Steep failure both directions — §4.2 |
| 3 | `NEAREST_MODE_BIN_WIDTH_M_DEFAULT` | 0.05 | `core/depth_common.py` | **Right.** Near optimal; safe to widen, not to narrow — §4.3 |
| 4 | `OTSU_BIN_WIDTH_M_DEFAULT` | 0.05 | `core/isolation_2d.py` | **Cosmetic.** ±4 mm over a 5× range — §4.3 |
| 5 | `MIN_VALID_SAMPLES` (as `min_valid_pixels`) | 10 | `core/ranging_defaults.py` | **Inert here, live elsewhere.** ≈ 82 m range limit on this path; gates raw points on the pointcloud path — §4.4, §1.3 |
| 6 | `MAX_RANGE_M` / `DEPTH_MAX_METERS_DEFAULT` | 10.0 | `core/ranging_defaults.py` | **Unused here, load-bearing elsewhere.** Not deletable — §2.3, §1.3 |
| 7 | `MAX_BOX_FRAME_FRACTION` | 0.60 | `measurement_pipeline.py` | **Inert**, with an analytic margin — §2.1, §3.2 |
| 8 | `MONOCULAR_USABLE_RANGE_FRACTION` | 0.9 | `core/depth_sources.py` | **Cannot fire in this world** — §3.4 |

One of the eight is wrong. Three are right, for reasons that were not written
down anywhere and in two cases are not the reasons one would guess. Two are
inert but now bounded rather than merely unobserved. One is dead code on the
production path, and one is unreachable in simulation and remains a
hardware-only question.

The two that look most alike — #3 and #4, the same literal `0.05` named
"histogram bin width" in two files — turned out to be the least alike of the
set: one moves a trial by 9.5 m, the other by 4 mm.

### 1.3 "Inert on this path" is not "unused" — do not delete on this evidence

Every verdict above is scoped to **box-gated projective ranging**. Two of these
constants live in `core/ranging_defaults.py`, whose entire purpose is to hold
values *deliberately shared across estimators*, and both are load-bearing on the
pointcloud path even though this path barely touches them:

| Constant | On the projective path | On the pointcloud path |
|---|---|---|
| #6 `MAX_RANGE_M` | Never applied: the node always passes `effective_depth_max()`, so the default argument is decoration (§2.3) | **`pointcloud_ranging.py:147`** — `valid &= planar_distance <= POINTCLOUD_MAX_METERS`. A hard 10 m ceiling with no parameter and no override |
| #5 `MIN_VALID_SAMPLES` | Never fires below 1000 (§4.4) | **`pointcloud_ranging.py:133`** — rejects an ROI holding fewer than 10 finite points, before any reduction |

#6's pointcloud ceiling is not theoretical; it decides recorded outcomes. In
`20260902_202659_isolation/pointcloud_reference`:

```
far_beyond_01   truth 10.50 m  ->  no estimate   (past the clamp)
far_clamp_01    truth  9.36 m  ->  8.83          (inside it)
far_clamp_02    truth  9.08 m  ->  8.67
```

The benchmark scenario set contains families **named after this clamp** —
`far_beyond` and `far_clamp` — so removing the constant would silently extend
the pointcloud estimator's range and turn a documented, deliberate miss into an
estimate.

So neither is a deletion candidate. The only honest cleanup #6 invites is
narrower: the *unused default argument* could be dropped from
`localize_prepared_projective_ranging`, `localize_projective_ranging`,
`localize_prepared_euclidean_reconstruction` and the two `isolation_2d` recipes,
making `depth_max` a required argument so the signature stops implying a
production default that never applies. That changes the standalone API and
several tests, and it is a readability change with no measurable effect — out of
scope here, recorded so the option is not lost.

---

## 2. Settled from the archive

### 2.1 #5 and #7 are inert at their current values

Across every archived run on the projective path, the miss codes those two
guards produce fired **zero times**:

| Reason | Count | Share |
|---|---|---|
| `OK` | 35,208 | 98.785 % |
| `NO_DEPTH_FRAME` | 293 | 0.822 % |
| `UNSET` | 140 | 0.393 % |
| `MASK_OVERSIZED_BOX` (#7) | **0** | — |
| `TOO_FEW_VALID_PIXELS` (#5) | **0** | — |
| `ISOLATION_EMPTY` (#5) | **0** | — |

`NO_DEPTH_FRAME` is the known exact-stamp race
(`docs/history/exact_stamp_depth_availability.md`), unrelated to either guard.
Both constants therefore have no effect on any recorded result. That is not
the same as "well chosen": it says only that they sit outside the operating
regime. §3.2 bounds how far #7 would have to move to bite.

A caveat that applies to #5 specifically and made the observation weaker than
it looks: until this work, `fill_path_measurements` **dropped** the
`min_valid_pixels` argument on the way into the estimator, so the guard ran at
its import-time default no matter what a caller asked. The zero count is
still a true statement about the shipped value; it was not a statement about a
parameter anyone could have varied. That is now threaded (§4).

### 2.2 #1 has a large, localized, already-measured effect

The `20260902_202659_isolation` sweep ran both 2D recipes over the same
88-scene set with everything else fixed. Joining them per trial on
`(scene_id, instance_index, repeat_index)` and keeping only trials scored by
both sides — 110 of them:

| Recipe | MAE | p95 |
|---|---|---|
| `nearest_mode_histogram` | 0.1638 | 0.8210 |
| `otsu` | 0.0609 | 0.1806 |

The gap is not spread evenly. Summed over the 110 paired trials it is
11.32 m of absolute error, and **92 % of it sits in 32 trials across six
families**:

| Family | n | nearest_mode MAE | otsu MAE | summed gap | cumulative |
|---|---|---|---|---|---|
| `interfere_pair` | 5 | 0.5469 | 0.0477 | 2.496 | 22.0 % |
| `interfere_infront` | 4 | 0.6229 | 0.0351 | 2.351 | 42.8 % |
| `chaos` | 7 | 0.3646 | 0.0834 | 1.969 | 60.2 % |
| `trap_split_pole` | 1 | 1.6499 | 0.0723 | 1.578 | 74.1 % |
| `objpartial` | 12 | 0.2915 | 0.1770 | 1.374 | 86.3 % |
| `interfere_multi` | 3 | 0.2499 | 0.0397 | 0.631 | 91.8 % |

Every one of those families puts something between the camera and the target.
The individual errors are the tell — six of the twelve worst trials sit at
**0.61–0.63 m**, the IV pole's standoff in the `interfere_*` scenes:

```
interfere_infront_01  0.620   interfere_pair_02  0.624
interfere_infront_02  0.623   interfere_pair_03  0.613
interfere_infront_03  0.623   interfere_multi_01 0.621
interfere_infront_04  0.625   (otsu: 0.016 – 0.061 on the same trials)
```

That is `nearest_significant_mode` anchoring on the occluder while
`band_m = 0.35` is too narrow to reach the target behind it — the failure the
contract doc warns about, now with a number on it.

The direction is not uniform. `otsu` is **worse** on `interfere_behind`
(+0.045), `interfere_bearing` (+0.047), `interocc_neartotal` (+0.052) and
`single_side` (+0.130): a threshold that keeps the near side of a bimodal split
has its own failure when the interference is behind the target. Neither recipe
dominates, which is why the default was left alone.

### 2.3 #6 never applies on this path, and helps where it does apply

`DEPTH_MAX_METERS_DEFAULT = 10.0` never reaches a projective-ranging
measurement: the mask node always passes `effective_depth_max()`, so on this
path the 10.0 survives only as the standalone API's default and in the depth
colorizers. It is emphatically **not** unused in the codebase — see §1.3, where
the same constant is a hard, unoverridable range ceiling on the pointcloud
estimator that determines recorded benchmark outcomes.

Where a 10 m ceiling *was* applied as a gate, the same sweep measured it as
marginally **better**, not worse: `projective_nearest_mode_10m` scored
MAE 0.1517 / p95 0.6236 against `projective_nearest_mode_unbounded` at
0.1621 / 0.6254 over the full set. That is well inside the noise floor below,
so the honest reading is "no measurable harm", not "the gate helps".

### 2.4 There is no 31 % noise floor — that was the D455 correction

The apparent floor came from two runs of the same nominal configuration
scoring MAE **0.1236** (`20260828_152011_baseline`) and **0.1621**
(`20260902_202659_isolation`), which looked like a 31 % run-to-run spread. It
is not. Adding this work's own default run to the comparison:

| Run | Commit | Date | n | MAE | p95 |
|---|---|---|---|---|---|
| `20260828_152011_baseline` | `d96b2f0` | 2026-08-28 | 108 | 0.1236 | 0.6211 |
| `20260902_202659_isolation` | `cb8061a` | 2026-09-02 | 112 | 0.1621 | 0.7134 |
| `20260907_211023` `baseline_a` | `f6e7d23` | 2026-09-07 | 112 | 0.1621 | 0.7134 |

The two later runs agree on MAE, p95 **and** scored count to every digit
recorded, five days and several commits apart. The outlier is the earlier one,
and the boundary between them is `7e31f02` (2026-08-31), the D455 correction —
which moved the Gazebo RGBD sensor onto `camera_0_color_frame` and changed
where the scene is rendered from. `AI_CONTEXT.md` already states that runs
recorded before 2026-08-31 used the D435 model and the old render pose and must
not be presented as the same setup; this is that warning showing up as a
number.

So the full-set benchmark is **reproducible, not noisy**, and a small MAE delta
between two configurations in one sweep is readable rather than drowned.

The sweep's two duplicated baseline configurations then measured the floor from
inside the run, and it is **exactly zero**: `baseline_a` and `baseline_b`
returned dMAE 0.0000 and dp95 0.0000 over 112 paired scored trials, and a
row-by-row comparison of all 135 trial records found **not one differing
estimate**. The simulator, detector, depth source and estimator are
deterministic end to end on a fixed configuration.

That is a stronger result than this document needed and it changes how the rest
of it should be read: any difference between two configurations in this sweep
is attributable to the one parameter that differs between them, with no
statistical hedging. The 0.94 h those two configurations cost bought exactly
that licence.

The examples set gives the same answer at smaller scale and is the right
cheap no-op check: box + stereoscopic at defaults returns MAE **0.0602**, which
this work's plumbing change reproduced exactly
(`artifacts/benchmarks/noop_proof_projective_params`, p95 0.0708 at 6 scored
trials, matching `20260902_115213`; the `repeats:=3` variants sit at 0.0602 /
0.0733 across four independent runs).

---

## 3. Settled by desk analysis

### 3.1 #1: the "robot depth extent" grounding does not hold

The attractive hypothesis was that 0.35 m is the G1's own fore/aft extent, so
the band spans the target and nothing more. Walking all 36 `<visual>` meshes in
`sim/models/g1/model.sdf` with each visual's pose applied
(`load_stl` → rotate by RPY → translate, accumulated into one model-frame AABB):

| Axis | Range | Extent |
|---|---|---|
| X fore/aft | −0.0726 … +0.3731 | **0.4457 m** |
| Y lateral | −0.1816 … +0.1816 | 0.3631 m |
| Z vertical | −0.0154 … +1.3074 | **1.3228 m** |

The Z extent is the honesty check: a real G1 is ~1.32 m tall, so this walk is
sound. An earlier walk that reported X 0.270 m and Z 0.846 m was missing poses,
and the 0.345 m "worst-case yawed depth" derived from it — suspiciously close
to 0.35 — was an artifact of that error.

With the correct geometry the hypothesis is **falsified in the unhelpful
direction**: the G1's fore/aft extent is 0.4457 m facing the camera and
`hypot(0.4457, 0.3631) = 0.5749 m` at worst-case yaw, both *larger* than the
0.35 m band. Even p5–p95 of the vertex distribution spans 0.407 m. The band
cannot cover the target's own depth at any orientation, so 0.35 is not the
robot's extent rounded down — it is ungrounded, and it is on the narrow side of
anything the geometry would suggest. That is consistent with the direction of
the §2.2 failure.

### 3.2 #7 would have to move a long way to bite

With `fx = fy = 443.53` on 640×480 and the camera at base_link + (0.011, 0.018,
1.028), the closest scene in `benchmark_scenarios_full.yaml` puts the target at
x = 1.09 m, i.e. 1.079 m from the camera. Projected there:

```
height  443.53 × 1.3228 / 1.079 = 543.7 px  -> clamps to the 480 px frame
width   443.53 × 0.5749 / 1.079 = 236.3 px  (worst-case yaw)
fraction = (480 × 236.3) / (480 × 640) = 36.9 %
```

Against a 0.60 threshold that is 23 points of margin at the most extreme pose
the benchmark contains. With the height already clamped, the box only reaches
0.60 once the width does — 384 px, which needs the target at 0.66 m. The guard
is a detector-failure catch, and nothing in this world approaches it, which
explains the zero count independently of having observed it.

### 3.3 #3 / #4: quantization is not what a bin-width sweep will show

The anchor `nearest_significant_mode` returns is a bin **centre**, so bin width
alone can contribute at most ±`bin_width_m`/2 = **±25 mm** to Z at the default.
Any effect larger than that in a bin-width sweep is the bin *choice* moving —
a different bin winning the "nearest significant" test — not rounding. That is
the discriminator to read the pending curves with.

### 3.4 #8 cannot fire in this world

`usable_max_m = MONOCULAR_MAX_DEPTH_FALLBACK_M × MONOCULAR_USABLE_RANGE_FRACTION
= 20 × 0.9 = 18 m`. The scenes span 1.09–10.73 m in spawn x, and `wall_east` in
`target_distance_calibration.sdf` sits at x = 12 with a 0.2 m thickness, so the
farthest surface any ray can reach is ≈ 11.9 m. Nothing in simulation ever
reaches 18 m. The 0.9 haircut is a hardware-only knob and no simulated result
can say anything about it.

---

## 4. Measured by the sweep

`artifacts/benchmarks/20260907_211023_projective_parameters`, one axis varied at
a time against a default baseline, on the same 88-scene set. Every comparison
below is a paired per-trial join restricted to trials scored by both sides, and
the noise floor inside this sweep is zero (§2.4), so the deltas are exact.

### 4.1 #1 `band_m`: 0.35 is wrong, and 0.75 is worth 88 % of the tail

| `band_m` | MAE | Δ MAE | p95 | Δ p95 |
|---|---|---|---|---|
| 0.15 | 0.1742 | +0.0121 | 0.6251 | −0.0003 |
| 0.25 | 0.1729 | +0.0108 | 0.6255 | +0.0002 |
| **0.35 (shipped)** | **0.1621** | — | **0.6254** | — |
| 0.50 | 0.1606 | −0.0015 | 0.6266 | +0.0013 |
| **0.75** | **0.0962** | **−0.0550** | **0.0776** | **−0.5460** |

The axis is a step, not a slope, and the step is exactly where the mechanism
says it should be. §2.2 measured the interference failures pinned at
**0.61–0.63 m** — the IV pole's standoff from the target. A band narrower than
that standoff cannot reach past the occluder the anchor locked onto, so 0.15,
0.25, 0.35 and 0.50 all leave the tail untouched (p95 constant to three
decimals across a 3.3× range of band widths). A band wider than it can, and
0.75 collapses the tail:

| trial | `band_m` 0.35 | `band_m` 0.75 |
|---|---|---|
| `interfere_infront_01` | 0.620 | 0.078 |
| `interfere_infront_02` | 0.623 | 0.068 |
| `interfere_infront_03` | 0.623 | 0.043 |
| `interfere_infront_04` | 0.625 | 0.061 |
| `interfere_multi_01` | 0.621 | 0.069 |
| `interfere_pair_01` | 0.615 | 0.061 |
| `interfere_pair_02` | 0.624 | 0.071 |
| `interfere_pair_03` | 0.613 | 0.060 |
| `interfere_pair_04` | 0.821 | 0.350 |

**and it costs nothing where the estimate was already right** — trials already
in the 0.06 m band stay there (0.065→0.062, 0.064→0.060, 0.063→0.059). This is
not a precision/robustness trade; it is a strictly better setting on this set.

The direction contradicts the Tier 1 geometric hypothesis in the useful way.
§3.1 found the band was *narrower* than the target's own 0.4457 m fore/aft
extent and called that ungrounded; the sweep shows the band must in fact be
sized against the **occluder standoff**, which is a property of the scene, not
of the robot. 0.75 m is not derived either — it is the first value tested above
0.62 — so this replaces an unjustified number with a measured one and a
mechanism, not with a derivation.

Not a default change: that is out of scope by decision, and one benchmark world
with one occluder geometry is thin ground for a global default. It is a
concrete, evidenced proposal for one. See §4.5 for what it does **not** settle.

### 4.2 #2 `min_bin_fraction`: 0.05 sits in a steep-sided valley

| `min_bin_fraction` | MAE | p95 | failure mode |
|---|---|---|---|
| 0.01 | 0.2447 (+0.0920) | 1.0353 (+0.4100) | thin near bins clear the floor; the anchor lands on slivers |
| **0.05 (shipped)** | **0.1621** | **0.6254** | — |
| 0.15 | 0.8605 (+0.7011) | 7.0328 (+6.4074) | only the wall clears the floor; the anchor lands on background |

The 0.15 result is the informative one, and the estimates name the cause
directly: `single_facing_01` goes from 4.831 against a truth of 4.897 to
**11.930**, and `far_mid_02` from 7.340 to **11.661**. Both are the far wall
(`wall_east` at x = 12, inner face ≈ 11.9).

The mechanism is composition, not tuning. The G1 spans 0.4457 m of depth
(§3.1), so at a 0.05 m bin width its surface is spread over roughly nine bins
and no single bin holds 15 % of the masked depths. The wall is flat: its whole
contribution lands in **one** bin, which clears 15 % easily. So at 0.15 the only
significant bin in the box is the background, and `nearest_significant_mode`
correctly returns the nearest bin that *is* significant — the wall. At 0.05 the
target's own bins clear the bar and win on nearness.

Note which safeguard does **not** save it. `nearest_significant_mode` has a
fallback for exactly this danger: when no bin clears the floor it takes the
nearest **non-empty** bin rather than the global mode, with a comment saying
this is deliberate because "at range the biggest coherent bin is the background
wall". But that fallback only runs when *nothing* is significant. Here the wall
alone is significant, so the primary path returns it and the fallback never
fires. The guard is aimed at the right failure and sits one branch away from
where this instance of it occurs — worth knowing before anyone raises this
constant expecting the fallback to catch them.

That makes this constant genuinely load-bearing, and in a way nothing
documented: it is what keeps a **diffuse** target from losing the anchor to a
**concentrated** background. It is also the same failure class the repo already
knows about on the sibling estimator — euclidean's percentile anchor is
composition-dependent with a cliff edge (`docs/history/estimator_evolution.md`).
Two anchors, two mechanisms, one shared weakness.

### 4.3 #3 / #4: the same 0.05 in two files, two different kinds of constant

The two bin widths look interchangeable on paper. They are not. Varying each
over the identical 0.02 – 0.10 range, against the ±`bin_width`/2 quantization
bound from §3.3:

| recipe | axis | MAE | p95 | max single-trial shift, 0.02 → 0.05 |
|---|---|---|---|---|
| `nearest_mode_histogram` (#3) | 0.02 | 0.2163 (+0.0514) | 0.6236 (−0.1973) | **9.5141 m** |
| | **0.05** | 0.1621 | 0.6254 | — |
| | 0.10 | 0.1634 (+0.0122) | 0.6257 (+0.0021) | — |
| `otsu` (#4) | 0.02 | 0.0608 | 0.1621 | **0.0043 m** |
| | **0.05** | 0.0608 | 0.1621 | — |
| | 0.10 | 0.0609 | 0.1630 | — |

**#4 is cosmetic.** Otsu's threshold is a global between-class-variance optimum,
so moving the bin edges barely moves it: aggregate MAE and p95 are identical to
four decimals at 0.02 and 0.05, 110 of 135 trials are bit-identical, and the 25
that differ move by at most 4.3 mm — five times *inside* the ±25 mm
quantization allowance. Across the full 5× range 0.02 → 0.10 the MAE moves by
0.0001. The constant could be almost anything in this range.

**#3 is load-bearing.** The same change moves a single trial by 9.5 m — 380×
the quantization bound — because `nearest_significant_mode` selects one specific
bin, and a bin-edge shift can hand the anchor to a different surface entirely.
The two directions are not symmetric: narrowing to 0.02 costs +0.0514 MAE, over
twice the entire quantization budget, because fewer samples per bin means fewer
bins clear the significance floor (the §4.2 mechanism, reached by another road);
widening to 0.10 costs +0.0122, which is *within* its own ±50 mm budget and
therefore consistent with pure rounding. So 0.05 sits near the optimum, and is
safe to widen but not to narrow.

This is exactly the discrimination §3.3 was computed to enable. Without the
bound stated in advance, 0.0043 and 9.5141 are two numbers; against it they are
two different failure classes.

Coupling worth recording: `bin_width` and `min_bin_fraction` act partly through
one quantity, since halving the bin width roughly halves the samples per bin and
so resembles raising the floor. `bin_width_0p02` and `min_bin_fraction_0p15`
drive `interfere_behind_01` to the *same* wall estimate of 11.672. They are not
equivalent, though — over 135 shared trials they agree on 66 and differ on 69,
so the resemblance holds only near the threshold.

### 4.4 #5 `min_valid_pixels`: inert, and now inert for a reason

| `min_valid_pixels` | scored | effect | implied range limit |
|---|---|---|---|
| **10 (shipped)** | 112 | never fires | ≈ 82 m |
| 100 | 112 | never fires; **all 135 estimates bit-identical** | ≈ 26 m |
| 1000 | 101 | rejects 10 trials; survivors bit-identical | ≈ 8 m |

At 1000 the guard fired for the first time anywhere in this repo's recorded
history — 1052 `ISOLATION_EMPTY` observations — and it rejected exactly the far
scenes: `far_clamp_01` (9.36 m), `far_clamp_02` (9.08 m) and all eight repeats
of `far_mid_01` (7.96 m). Mean true distance of the rejected trials is 8.22 m
against 3.57 m for those kept.

**This grounds the constant.** Foreground pixel count falls as 1/Z², so a pixel
threshold is a maximum-range limit in disguise. With the G1 at 1.3228 × 0.3631 m
(§3.1) and `fx = fy = 443.53`, the bounding box at 8 m is 73 × 20 = 1474 px and
the isolated foreground a fraction of that — right at 1000, which is where the
cut is observed. Scaling by √, the shipped value of 10 corresponds to a range
limit near **82 m**, and 100 to about 26 m. Both are far outside the 12 m room
and beyond what a D455 resolves, which is *why* the guard has never fired —
derived, rather than merely observed as in §2.1.

Its behaviour is also correct for a sufficiency gate: on every trial it admits,
the estimate is bit-identical to baseline. It removes trials; it never biases
the ones it passes.

### 4.5 The recipe comparison, corrected

An earlier reading of this sweep held that fixing the band made the shipped
recipe simply better than `otsu`, on the strength of its p95. Measured
like-for-like inside this one sweep, that is wrong — they are complementary:

| configuration | MAE | p95 |
|---|---|---|
| `nearest_mode_histogram`, band 0.35 (shipped) | 0.1621 | 0.6254 |
| `nearest_mode_histogram`, band 0.75 | 0.0962 | **0.0776** |
| `otsu` | **0.0608** | 0.1621 |

Fixing the band closes most of the gap and **reverses the tail** — 0.0776
against 0.1621, 52 % better — but `otsu` keeps a real typical-case advantage,
0.0608 against 0.0962, 37 % better. So the recipe difference is not purely a
band artifact. The two fail differently: `nearest_mode_histogram` with an
adequate band is robust against catastrophic occluder lock-on, while `otsu` is
more accurate in the common case and carries the heavier tail (§2.2 puts its
losses on `interfere_behind`, `interocc_neartotal`, `interfere_bearing` and
`single_side`).

Choosing between them remains out of scope. What the sweep adds is that the
choice is a genuine trade rather than a fix for a badly sized constant.

## 5. What changed in the code

Four of the constants were not node parameters and not launch arguments, so no
benchmark configuration could vary them — the reason half this document is
desk analysis rather than measurement. They are now plumbed end to end,
**each defaulting to the constant it replaces**, so an unset run is the run
that was already happening:

| Layer | Change |
|---|---|
| `core/isolation_2d.py` | New `build_isolation_2d(name, *, bin_width_m, band_m, min_bin_fraction)`, mirroring `build_isolation_3d`. Binds only the kwargs the selected recipe accepts (`otsu` takes `bin_width_m` alone), so one launch argument spans both recipes. With nothing set it returns the registry entry itself. |
| `measurement_pipeline.py` | `fill_path_measurements` threads `min_valid_pixels` into `localize_prepared_projective_ranging`. It was dropped here, which is why the guard was unreachable. |
| `mask_measurement_node.py` | Declares `isolation_2d_bin_width_m`, `isolation_2d_band_m`, `isolation_2d_min_bin_fraction`, `min_valid_pixels`; builds the recipe through the factory (still validating the name first). |
| `target_localization/launch.py` | The four names join `CONFIG_LAUNCH_ARGUMENT_NAMES`; `mask_measurement_node()` gains four keyword arguments. Exploration passes none of them and keeps the node defaults. |
| `target_benchmark_config.launch.py` | Four `DeclareLaunchArgument`s at the current constants, forwarded to the mask node and recorded on the runner. |
| `target_distance_benchmark_runner_node.py` | Declares all four record-only, as `mask_depth_max_meters` already was, so `run.json`'s `parameters` block carries the value a config actually ran at. |
| `sweep.py` | The four join `KNOB_ESTIMATORS` against `projective_ranging`, so a config that sets one without selecting that estimator is rejected at load. |

New tests cover the factory's kwarg binding and unknown-name rejection, the
identity property when nothing is set, a bound `band_m` changing what the
recipe keeps, and a raised `min_valid_pixels` actually producing
`TOO_FEW_VALID_PIXELS` (tight branch) and `ISOLATION_EMPTY` (rect branch).

---

## 6. Limits of this evidence

The sweep ran to completion — 15 configurations, `config/benchmark_sweep_
projective_parameters.yaml`, 2026-09-07/08, sweep directory
`20260907_211023_projective_parameters`. What it does **not** establish:

- **One world, one occluder geometry.** The §4.1 result turns on the IV pole's
  0.62 m standoff in `target_distance_calibration.sdf`. A band of 0.75 m is
  correct *for scenes whose occluders stand about that far in front of the
  target*. It is not a derivation, and a world with 1.5 m standoffs would need a
  different number — which is precisely why this is written as a proposal rather
  than a default change. What generalises is the **mechanism**: the band must
  exceed the occluder standoff, not the target's own depth extent.
- **A wider band is not free in general.** On this set it cost nothing, but a
  band wide enough to span occluder and target is also wide enough to admit
  background in a scene where the object sits close to a wall. The 88-scene set
  has `interfere_behind` and `interocc_*` families that probe exactly that and
  0.75 did not lose on them — but two of the three `interfere_behind` trials
  that constrain this are single-repeat.
- **Simulation only.** Stereo depth here is a noiseless render. Every
  significance-floor and bin-width conclusion depends on how the target's depths
  distribute across bins, and real D455 depth is noisier and sparser at range —
  which would push in the same direction as narrowing the bins (§4.3), the
  direction already shown to be harmful. These numbers should be re-taken on
  hardware before any of them is treated as a tuned value.
- **#8 remains untouched.** `MONOCULAR_USABLE_RANGE_FRACTION = 0.9` cannot fire
  in a 12 m room (§3.4) and no simulated run can say anything about it.
- **#6 is unused *here*, not unused.** §2.3 shows the 10 m constant never
  reaches a projective-ranging measurement, but §1.3 shows it is a hard range
  ceiling on the pointcloud estimator that already decides recorded outcomes.
  Nothing in this document licenses deleting it, and its correctness *as a
  pointcloud range limit* was never measured — that is a separate question this
  sweep did not ask.

Preconditions for re-running are in `docs/ISSUES.md`: confirm
`glxinfo | grep "OpenGL renderer"` does not report `llvmpipe` (Gazebo then
CPU-rasterizes and the camera runs at 4 Hz instead of 28, which RTF does not
reveal); run `cleanup.sh` **before** the supervisor and not between its
configurations; use `record_video:=false`; and expect to restart Gazebo for a
sweep longer than about five hours — this one crashed in the GUI's Ogre2 render
thread at 6.4 h and was resumed.
