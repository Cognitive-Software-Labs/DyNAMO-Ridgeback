# BENCHMARKING — leftover G1 links in Isaac: findings and fix proposals

Written 2026-11-11 (Europe/Berlin). Local investigation note; not committed.
Tested code: branch `dev` at `cc86eef` plus the uncommitted working tree at
that time. Nothing under `src/` or `tools/benchmark/qualification/` was edited.
Analysis and reproducer scripts live in this session's scratchpad,
`/tmp/claude-1009/-home-stefi--claude-spares-spare-yphiC1/9e53f215-8936-4048-8162-e9a0ac94faf5/scratchpad/`.
The scratchpad may disappear; every number below is reproducible from the
bundles named here with the method described.

**Status: investigation and reproducer done, no fix chosen.** Section 4 lists
the options for the user to choose from.

**Summary.**
- **What leaks.** Each leftover is one prim of a deleted G1. The USD prim is
  gone, but its Fabric copy is never removed, and the Fabric Scene Delegate
  keeps rendering it at the G1's final pose for the rest of the session.
- **Reproduced standalone,** with the tracked `UsdScene` code: about 1 leak per
  500 G1 deletes with new prim paths.
- **Repairable and preventable.** Removing the prim from Fabric repaired every
  case. A fixed prim path (0/3500) and a spawn-once pool (0/1800) prevented it.
- **Ruled out as causes:** physics, `freeze_dynamics`, instancing, skinning and
  materials.
- **Effect on results.** The restart11 box/stereo 644 mm maximum is the
  leftover. The ~100 mm monocular medians are not.

## 1. Confirmed facts

### 1.1 The three leftovers, re-detected from the captures

Same-implementation session pairs were compared in stereo depth: one session
nearer by more than 0.15 m, in connected blobs. The comparisons were restart6
r1 and restart8 r2 against restart11 r2, and new r1 against new r2. The table
in the task brief is confirmed:

| Session | Leftover in view from | Source G1 | Leftover link (confidence) | Clean pair |
|---|---|---|---|---|
| restart6 r1 (realtime) | condition 1 trial 54 (`objpartial_04`); every later trial where in view, all of conditions 2–4 | `objpartial_01` target_0, (8.36, 0.31), yaw 3.62 | `left_shoulder_yaw_link`, upper arm (high) | restart11 r2 |
| restart8 r2 (realtime) | condition 1 trial 54 | `objpartial_01` target_0 | about 6×6×6 cm piece in the left wrist region, `left_wrist_yaw_link`/`left_wrist_pitch_link` (medium) | restart11 r2 |
| new r1 (PathTracing) | condition 2, from trial 20 (`multi_04`) | `multi_03` target_0, (10.86, 0.10), yaw 3.08 | `right_shoulder_roll_link` (high) | new r2 |
| restart11 r2, new r2 | none | — | — | — |

The world coordinates quoted in the task brief come from the capture
back-projection `pw = pc @ R.T + T + base`. That frame is offset from the
simulator world. Fitting the authored G1 mesh to its own rendered points, in
four trials, needs about +0.19 m in x and −0.03 to −0.04 m in z, with 0.9–1.3 cm
median residual after the fit. The floor, whose top is at z = +0.05, appears at
z = −0.024. The offset is constant across trials and sessions, so the
comparisons below that stay in that one frame are unaffected. Do not quote
those coordinates as simulator poses. A likely explanation is that `base`
already sits at the camera, so adding `T` counts the camera mount twice; this
was not verified.

### 1.2 Each leftover is an exact copy of one visual link at the source G1's last pose

- **new r1.** In conditions 3 and 4, `multi_03` re-spawns its G1s at identical
  poses. Those frames are **bit-identical** to r2's in RGB and depth (0 pixels
  differ) while the leftover exists, so the leftover coincides exactly with the
  re-spawned G1's surface. Its points lie on the visible `multi_03` G1 surface:
  median depth difference 0.0 cm, 88 % within 3 cm. A single-link test against
  the fitted source G1 gives `right_shoulder_roll_link` a median of 1.3 cm; the
  neighbouring shoulder links score 2.3 and 3.3 cm. Shape-only 6-DOF fitting of
  all 47 visual meshes also ranks the shoulder-roll link first: rms 5.0 mm,
  extents 7.3×6.9×13.4 cm against the mesh's 7.3×8.1×13.7 cm.
- **restart6 r1.** In condition 2, frames of the re-spawned `objpartial_01`
  repeats have depth identical to restart11's (0 px differ). The leftover
  points lie on the source G1's visible surface (median 0.1 cm, 76 % within
  3 cm). 67 % of them are nearest to `left_shoulder_yaw_link` (median 1.2 cm),
  which is also the best shape fit: extents 7.9×8.5×15.1 cm against
  7.5×8.1×14.5 cm.
- **restart8 r2.** The leftover lies inside the source G1's envelope, behind
  the visible surface from the `objpartial_01` camera, about 4 cm deeper. Its
  points are nearest to the left wrist yaw/pitch links, at a 3.1 cm median.
  The roughly 6 cm D-shaped silhouette matches `left_wrist_yaw_link`
  (5.4×5.9×6.0 cm). This identification is weaker because the piece is never
  seen from the source view.
- The leftovers are **arm links, not hands**. The Dex1 hand meshes, which have
  separate materials and include the 171k-vertex `hand_base_link`, were not the
  leftover in any session. "Hand-height" was correct; "hand" was not.

### 1.3 Nothing marks the leftover meshes as special

All three are ordinary `Mesh` prims at `<link>/visuals/<link>/mesh`. They are
not instanced and bind the shared `/g1/Looks/DefaultMaterial`, as do 30 other
links. The G1 contains **no UsdSkel prims**. Only the 40 `collisions` subtrees
are instanceable (40 prototypes). Those subtrees have `guide` purpose and are
never rendered; for example, the shoulder-roll collider is a `Cylinder`, while
the leftover matches the visual mesh. Every link carries `PhysicsRigidBodyAPI`
and `PhysicsMassAPI`, with rigid bodies disabled by `freeze_dynamics`. Joints
are deactivated and the articulation is disabled. The three leftover links
have the same composition as the 44 links that never stayed. The reproducer
confirms that any prim of the asset can leak (section 3).

### 1.4 Prim paths are never reused for targets

A target's name is `bench_<run_label>_<trial_id>_target_<i>`. `run_label` is a
timestamp unique per condition (four per session), and repeats carry `_repN`
in `trial_id`. Across all capture metadata of the three affected sessions there
are **zero duplicate target names**. An immediate re-spawn of the same scene
gets a new prim path at the same pose. Background props do reuse names
(`bench_<label>_bg_<prop>` is dropped and restored per scene), but no prop
leftover was found.

### 1.5 Formation window and timing

- **new r1.** RGB is bit-identical to r2 in every batch through `multi_03`
  (trial 19) of condition 2; 8 pixels differ from `multi_04` (trial 20). The
  leftover formed during `multi_03`'s teardown or `multi_04`'s setup.
- **restart6 r1 and restart8 r2.** Depth matches restart11 exactly through
  trial 52. The source G1 pose exists only in trials 44–51, so the leftover
  formed at one of the eight `objpartial_01` teardowns. Which repeat cannot be
  told, because the next repeat's identical G1 hides it.
- **Update cadence.** Every scene operation is one queued request drained once
  per `app.update()`. `delete_entity` then confirms absence with a separate
  `GetEntities` call, so a `RemovePrim` and any later spawn are at least two
  updates apart, and capture follows a 2 s settle. No delete and spawn share an
  update.
- **Repeat gap.** Between back-to-back `objpartial_01` repeats, the G1 is
  removed and re-spawned at the identical pose about 0.3–0.5 s later; restart6
  log, "background removed none, restored none". That is the shortest
  delete-to-identical-geometry gap in the run. Normal scene changes take
  1.4–2.2 s with prop churn.

### 1.6 Rate and concentration

The five full sessions deleted about 2,290 G1s: four sessions at 540 each and
restart8 r2 at 133, its only condition. Three leftovers were seen, about one
per 760 deletes. This is a **lower bound**, because a leftover counts only if a
later camera view shows it. Two of the three formed in the `objpartial_01`
same-pose repeat window, which holds 136 of the ~2,290 deletes (17 passes × 8).
Under a uniform per-delete rate, the chance of two or more of three landing
there is about 0.01. That window is also early in the session: G1 deletes
#61–68 of 540. The new r1 leftover formed at delete #158–159. None of the
three formed in the later ~70 % of any session, which is p ≈ 0.005 under a
uniform rate. The reproducer (section 3) attributes the concentration to
session position and new prim paths, not to the same pose.

### 1.7 Renderer and Kit context

- Both realtime RTX (restart6, restart8) and deterministic PathTracing (new r1)
  produced leftovers.
- Isaac Sim 6.1.0-rc.26, Kit 110.3, `isaacsim.exp.base.python.kit`.
- **Fabric Scene Delegate is on**: `app.useFabricSceneDelegate = true`, with
  `omni.hydra.usdrt_delegate` 7.5.2 and `usdrt.scenegraph` 7.6.3. The renderer
  therefore reads Fabric, not USD.
- `omni.physx.fabric` is **not loaded**, so PhysX does not write transforms into
  Fabric. `app.asyncRendering = false`.
- Sessions ran windowed (`headless=false`) with `syncUsdLoads` and
  `materialSyncLoads` on.
- The Kit log of new r1 shows no warning or error near the deletes. USD
  diagnostics are muted (`/persistent/app/usd/muteUsdDiagnostics`).
- The benchmark's scene-state check reads USD entity names, so a render-side
  copy is invisible to it by construction.

## 2. Root cause: ranked hypotheses

| # | Hypothesis | Verdict | Evidence |
|---|---|---|---|
| H1 | **Fabric keeps one prim of a removed subtree.** USD removes the G1, but one of its prims stays in Fabric, and FSD renders it. | **Confirmed by the reproducer (section 3)** | Benchmark: an exact copy at the last pose (1.2); persists in geometry, shading and shadow; USD gone; both render modes (1.7). Reproducer: every case is a stale Fabric prim, removing it from Fabric repairs the image, 9/9. Isaac's own `isaacsim.robot_setup.sysid` bridge also removes Fabric prims explicitly (`usdrt_stage.RemovePrim`, deepest first, then `update_world_xforms`). |
| H1a | **Trigger: Fabric tables growing with new prim paths** while a removal is processed. | Likely, about 70 % | Fixed path 0/3500 against new paths 10/5000; leaks cluster early and roughly double in spacing; benchmark leftovers all at delete #61–159 of 540. Not proven: Kit source not inspected. |
| H1b | The earlier "same pose, back-to-back" modifier | Superseded by H1a | The `objpartial_01` window is also early in the session (deletes #61–68). The reproducer re-spawns at the same pose every cycle, both in variants that leak and in variants that do not. |
| H2 | Prim path reuse **causes** it | **Ruled out; reuse prevents it** | 1.4: targets never reuse paths. Reproducer: reuse 0/3500. |
| H3 | Re-spawn at the same path while a removal is pending | **Ruled out** | Benchmark paths differ (1.4); reproducer reuse leaks nothing at a 4-frame gap. |
| H4 | PhysX objects reflected into Fabric | **Ruled out** | `omni.physx.fabric` not loaded. Reproducer leaks at the same rate with the timeline never played (3/1000) and without `freeze_dynamics` (3/1000). |
| H5 | Skinning buffers (UsdSkel) | **Ruled out** | No UsdSkel prims in the G1. |
| H6 | USD instancing or prototypes | **Ruled out** for the leftover | Visual meshes are not instanced; the instanced collision subtrees are `guide` and unrendered. |
| H7 | Material or MDL lifetime | **Ruled out** as the selector | The reproducer leaked meshes bound to `DefaultMaterial` and to the hand's `material_4C4C4C`, and collision prims that have no render material at all. |
| H8 | Temporal accumulation or render history | **Ruled out** | Constant world position from every viewpoint for hundreds of trials, present in geometric depth, casts a shadow, and appears with path-tracing caches disabled. |
| H9 | Mesh size or upload | **Ruled out** | Leaked prims range from a 252-point finger tip to the 51k-point knee, plus collision `MeshInstance` prims. |

**Bottom line.**
- USD-side bookkeeping is correct.
- Kit 6.1 rc26's USD-to-Fabric sync occasionally fails to remove one prim of a
  removed subtree. FSD then renders that prim at its final pose for the rest of
  the session.
- It is most likely triggered while Fabric's tables grow with new prim paths,
  so it is concentrated early in a session.
- Our code cannot see it, because every existing check reads USD.
- A Fabric query sees it, and a Fabric `RemovePrim` repairs it.
- This is a Kit defect worth reporting upstream with the reproducer.

## 3. Reproducer

**Script:** `repro_g1_ghost.py`; driver `run_variants.sh`; results in
`runs/<variant>/cycles.jsonl`, all in the scratchpad named at the top.

**Setup:**
- Headless Isaac 6.1 with the same kit as the runner and
  `RealTimePathTracing`.
- The `g1_distance_calibration` world, with the timeline playing.
- The tracked `usd_scene.UsdScene.spawn/delete` and `freeze_dynamics`,
  imported unmodified; only `scene_provider`'s two constants are stubbed to
  avoid ROS.
- A 320×240 `distance_to_image_plane` probe camera at the `objpartial_01`
  robot position looking at the `objpartial_01` G1 spot.

**One cycle mimics an `objpartial_01` repeat:**
1. Spawn `hospital_bed`, only in variants marked "+ bed" (`--with-object`).
2. Spawn the G1 at the same pose under a new path.
3. Run 3 update frames.
4. Delete the G1, run 1 frame, delete the bed.
5. Run 3 more frames, then check.

**The check:**
- A depth frame compared with the empty baseline: any pixel nearer by 3 cm
  or more, confirmed again 10 frames later.
- usdrt `GetPrimsWithTypeName('Mesh')` under `/scene_entities`. USD holds
  nothing there after the delete.
- On a hit: log the Fabric prims, try `usdrt RemovePrim` (deepest first),
  re-check depth.

**Measured latency:** a spawned G1 appears in the probe 3 frames after spawn
and is gone 3 frames after delete.

Runs were started only when no other user's simulator was running. Another
user's Isaac or Gazebo job sometimes started during a run; that only slowed
rendering. Transient "G1 still visible" frames from the extra lag always
cleared within the 10-frame confirmation.

| Variant (1000 cycles each unless noted) | Leftovers | What stayed (all in Fabric, USD empty) | Fabric `RemovePrim` repair |
|---|---|---|---|
| `unique` + bed: new path per cycle, same pose (objpartial mimic) | 1, at cycle 206; persisted to cycle 1000 | one Mesh, 2 px in depth. Path not recorded: this run used the first script version | not tried |
| `reuse` + bed: same prim path every cycle | 0 | — | — |
| `deactivate` + bed: `SetActive(False)`, 1 frame, then `RemovePrim` | 3, at cycles 16, 206, 390 | `right_hand_Link1_3/visuals/.../mesh` ×2 (252 points), `right_ankle_pitch_link` visual | 3/3 repaired, depth back to baseline (0 px) |
| `nofreeze` + bed: `freeze_dynamics` disabled, PhysX articulation live | 3, at cycles 9, 57, 100 | `right_knee_link` visual (49 px), `left_wrist_pitch_link` visual (10 px), `right_hip_roll_link/collisions/.../mesh` (`MeshInstance`, guide, 0 px) | 3/3 repaired |
| `settle` + bed: 30 frames after delete before the next topology change | 0 | — | — |
| `unique`, no bed (G1 only; next G1 spawn about 4 frames after the delete) | 0 | — | — |
| `reuse` + bed, 2500 more cycles | 0, so 0 in 3500 reused-path cycles in total | — | — |
| `unique` + bed, repeat with paths recorded | 0, so the cycle-206 coincidence above is not deterministic | — | — |
| `pool` + bed: spawn once, then `MakeVisible`/`MakeInvisible` and move instead of delete; 1500 + 300 cycles | 0 in 1800. The G1 renders while visible (6205 px, the same as a normal spawn) and leaves nothing when hidden | — | — |
| `unique` + bed, timeline never played (no PhysX stepping; Fabric check only) | 3, at cycles 4, 46, 101 | `pelvis_contour_link/collisions/.../mesh`, `right_shoulder_yaw_link/collisions/.../mesh`, `right_wrist_pitch_link` visual | 3/3 removed from Fabric |

Totals:

| Group | Leftovers / cycles | Rate |
|---|---|---|
| New G1 path each cycle, with bed (`unique` ×2, `deactivate`, `nofreeze`, `noplay`) | 10 / 5000 | about 1 per 500 G1 deletes |
| Same G1 path every cycle (`reuse`) | 0 / 3500 | p ≈ 0.001 against the group above |
| Spawn-once pool | 0 / 1800 | p ≈ 0.03 |
| 30-frame settle | 0 / 1000 | p ≈ 0.14, inconclusive |
| G1 only, no bed | 0 / 1000 | p ≈ 0.14, inconclusive |

Leftover cycle numbers across runs were 4, 9, 16, 46, 57, 100, 101, 206, 206 and 390. Under a
constant rate they would spread evenly over 1000 cycles. Instead they cluster
early and thin out roughly by doubling.

**What the reproducer establishes:**

1. **The defect reproduces outside the benchmark** with only the tracked
   `UsdScene` code: about 1 per 500 G1 deletes, against about 1 per 760 in the
   benchmark.
2. **The leftover is a stale Fabric prim.** Every detected case (10 of 10) was
   one prim of the deleted G1 still present in Fabric, with its points and its
   world matrix at the final pose, while the USD stage had no children under
   `/scene_entities`. Its depth pixels back-project inside that prim's Fabric
   world extent. FSD renders Fabric, so the copy keeps rendering. Hypothesis
   H1 is confirmed and located on the Fabric side, not only in RTX.
3. **Removing the stale prim from Fabric repairs it completely,** 9 of 9 tried
   (the P0 repair). A Fabric query finds every case with no rendering at all, which
   makes it the probe's first layer.
4. **The leftover can be any prim of the asset:** visual or collision (a
   `MeshInstance` with `guide` purpose), large or tiny. The reproducer left
   knee, wrist, ankle, pelvis, shoulder and finger meshes; the benchmark left
   shoulder and wrist meshes. Only visual meshes are seen, which is why the
   benchmark found arm links.
5. **Physics is not involved.** The rate stays the same without
   `freeze_dynamics` (3/1000) and with the timeline never played (3/1000).
6. **Deactivating first does not help** (3/1000).
7. **A fixed prim path prevents it** (0/3500, p ≈ 0.001). So does
   **spawn-once-and-hide** (0/1800, p ≈ 0.03).
8. **Leaks cluster early and thin out roughly by doubling** in the reproducer.
   The benchmark matches: all three leftovers formed at G1 delete #61–68
   (restart6, restart8) or #158–159 (new r1) of 540; none in the later ~70 %
   of any session (p ≈ 0.005 under a uniform rate).

**Most likely mechanism** (inferred; Kit source not inspected):
- A removal is lost when Fabric's internal path or prim tables grow while the
  removal is being processed. New prim paths grow them; a reused path does not.
- Capacity doubling would explain the clustering by cycle and session position.
- The `objpartial_01` concentration in 1.6 is then mostly an early-session
  effect, deletes #61–68, not a same-pose effect.
- Fast topology churn right after the delete may also matter: a bed or props
  removed and spawned within a few frames. Without the bed, and with a
  30-frame settle, the reproducer gave 0/1000 each, but neither is significant
  alone.
- Kit's `app.settings.fabricDefaultStageFrameHistoryCount = 3` (three frames
  of Fabric history) is an untested lead for that window.

## 4. Fix proposals (ranked) and the detection probe

Ranked by expected effect, then cost and determinism risk. Reproducer numbers
are from section 3.

| Rank | Proposal | Prevents? | Reproducer evidence | Cost | Determinism risk |
|---|---|---|---|---|---|
| P0 | Fabric audit after every delete, with an automatic Fabric repair; the trial fails if the repair does not hold | Detects and repairs | Audit found 10/10; repair 9/9 | Low | None while nothing leaks |
| P1 | **Fixed slot prim paths:** the provider maps public names onto reused paths | Yes | 0/3500 against 10/5000 | Low | Low |
| P2 | Spawn-once pool: move and hide instead of delete | Yes, for pooled assets | 0/1800 | Medium | Medium: changes render history |
| P3 | Fresh simulator per condition | Contains only | n/a | Low | Low |
| — | Deactivate before removal | No | 3/1000 | — | — |
| — | Extra settle frames | Unproven | 0/1000, p ≈ 0.14 | — | — |
| — | Unique path per spawn | **Harmful**: it is the trigger | 10/5000 | — | — |

### P0 — Fabric audit plus repair; do this whatever else is chosen

A run must never carry a leftover forward silently.

1. **After every delete:** in `UsdScene.delete`, or in the provider right after
   it, let one update pass. Then query
   `usdrt.Usd.Stage.Attach(stage_id).GetPrimsWithTypeName('Mesh')`; the prim
   types to check also include `MeshInstance` and other gprims. Look for any
   path under the deleted prim. On a hit, remove the stale prims from Fabric
   deepest first (`usdrt_stage.RemovePrim`), run one update, and query again.
   Record a typed event, `fabric_leftover_repaired` or
   `fabric_leftover_persistent`. A persistent leftover fails the delete, so the
   client's existing absence check raises instead of passing.
2. **Session-wide sweep at each trial start:** every Fabric prim under
   `/scene_entities` must belong to a live registry entry. This catches leaks
   from paths the per-delete audit did not cover, such as background props.
3. **Optional end-to-end guard:** an `instance_id_segmentation_fast` annotator
   on the robot camera. At capture time, every rendered path under
   `/scene_entities/` must be one of the current trial's entities. This guards
   against a render-side leftover that is not a Fabric prim, which has not
   been seen. Cost: one annotator at 640×480. It needs a check that RTX labels
   a stale prim with its path.

**On a persistent leftover:** mark the trial `render_leftover`, a new miss
reason, never silent, and stop the condition so a fresh session reruns it
(see P3).

**Acceptance.**
1. Unit-level: a test against a usdrt stage, if one can run in CI under
   `isaac_venv`, removes a USD prim, injects a Fabric-only prim at its path,
   and expects the audit to remove it and report it.
2. Reproducer: run the `deactivate` and `nofreeze` variants, which leaked
   6/2000, with the audit in `UsdScene.delete`. Every leftover must be reported
   and repaired, and the depth check must show 0 persistent leftovers.
3. Full session: one 4-condition Isaac session with the audit on reports any
   repairs in the run record and has no `render_leftover` trials. A same-build
   pair has box/stereo bit-identical, and silhouette/stereo and monocular
   identical wherever the RGB is.

The edits are in `src/ridgeback_autonomy_isaac/sim/isaac/usd_scene.py` and
`scene_provider.py`. Changing tracked `src/` blocks `start-offline` on the
frozen bundle (`check_frozen`). Finish or abandon that bundle's offline step
first.

### P1 — fixed slot prim paths (prevention, cheapest)

Inside the provider, `SceneEntities` maps each public entity name onto a
reusable prim path per resource, `/scene_entities/<resource>_<k>`, where `k`
is the lowest free slot. `RemovePrim` and re-spawn stay as they are. The
registry keeps public names, so `GetEntities`, `verify_scene_state` and the
benchmark see no change.

- **Expected effect:** the Fabric path set stops growing after the first
  trials, and the reproducer leaked 0/3500 with a fixed path. Background props
  already reuse names and never leaked.
- **Cost:** low. Name-to-slot bookkeeping, `prim_path()` lookups, and tests for
  slot reuse and release.
- **Risks:**
  - A stale Fabric prim at a slot path would be re-populated by the next spawn
    rather than left orphaned. That is benign, but keep P0 anyway.
  - Render history changes slightly, because paths differ from today's.
    Cross-session determinism is unaffected; the mapping is deterministic.
- **Acceptance:**
  1. A reproducer variant with slot paths and the objpartial pattern has
     0 leftovers in at least 5,000 cycles. Today's unique-path rate predicts
     about 10, so p < 1e-4.
  2. A full session with P0 on records zero repairs.

### P2 — spawn-once pool: move and hide instead of delete

Spawn the maximum number of G1s per scene (3) once per session. A spawn takes a
parked slot, sets its pose, and calls `MakeVisible`. A delete calls
`MakeInvisible`, parks it out of view, and disables its colliders. The
registry maps names to slots and hides parked ones.

- **Expected effect:** no G1 `RemovePrim` at all. Reproducer 0/1800, and the
  visible G1 renders normally (6205 px, the same as a fresh spawn).
- **Cost:** medium, more than P1. Objects and props would need their own
  pools or P1.
- **Risks:**
  - Render history changes: hidden prims exist from session start, so treat
    outputs as a new baseline.
  - RTX LiDAR must not hit hidden G1s; verify with one scan.
- **Acceptance:**
  1. A reproducer `pool` run with varying poses has 0 leftovers in at least
     5,000 cycles.
  2. A full session passes P0.
  3. A fresh session pair meets the identity criteria in P0 acceptance 3.

### P3 — fresh simulator per condition (containment)

Restart the Isaac runner between conditions.

- **Effect:** a leftover cannot cross conditions. Today restart6's
  contaminated three later conditions, and new r1's two. Because leaks cluster
  early in a session, restarts may slightly *raise* the number of early-session
  windows, so combine this with P0 or P1, never alone.
- **Cost:** startup per condition (warm caches, about 1–3 min).
- **Acceptance:** each condition starts with `/scene_entities` empty in USD and
  Fabric (P0 sweep).

### Not recommended

- **Deactivate before removal:** measured 3/1000, no better.
- **Extra settle frames:** 0/1000 is not significant. The benchmark already
  has at least two updates between operations, and the leftover persists for
  hours. Re-test only if P1 is rejected.
- **Unique prim path per spawn:** this is today's behaviour and the likely
  trigger.
- **In-process stage reload per condition:** the runner notes that the ROS 2
  bridge crashes if a stage is opened after the extension starts. P3 achieves
  the same containment safely.

**Suggested combination:**
1. P0 + P1 together: cheap, local to the provider, measured in the
   reproducer.
2. Then a full 4-condition session pair as the acceptance run.
3. P2 only if P1 fails its acceptance.
4. P3 is optional insurance.
5. Report the defect upstream to NVIDIA (Kit/Fabric) with the reproducer.

## 5. What this means for existing results

**New repeatability pair (2026-11-06,
`isaac_repeatability_prepared_20261105T234856Z`).**
- Every non-zero between-session mask-estimator delta comes from a trial whose
  RGB differs between sessions. In trials with bit-identical RGB, the
  projective and Euclidean deltas are exactly 0 in all four conditions. The RGB
  differences are this leftover and its shadow (the 2026-11-09 re-analysis
  found no other difference).
- Box/stereo is bit-identical because the leftover formed in condition 2.
- Silhouette/stereo's maximum of 4.3 mm, and the monocular tails (projective max
  1,376 mm and Euclidean 478.8 mm in box; 247.6 and 245.0 mm in silhouette), are
  leftover-driven.
- Polar deltas (max 28.5 mm) are independent: they occur with bit-identical RGB
  (unseeded RTX LiDAR noise).
- The archive record
  `archive/engineering/2026-11-06-isaac-full-repeatability.md` ("trigger
  unidentified", RGB differences "tiny") should be corrected. Its monocular
  tail is contamination, not model or renderer noise.

**restart11 qualification Isaac pair** (r1 = imported restart6 r1, r2 =
restart11 r2). Pairs were recomputed per trial from the condition CSVs, as
`evidence.repeatability` does. Each trial was classified by whether restart6's
leftover is visible in r1's frame (in the frustum and nearer than the clean
geometry) and whether it falls inside any r1 detection box:

| Condition / estimator | All: median / max (mm) | Leftover in a target box: n, max | Without in-box trials: median / max |
|---|---|---|---|
| box/stereo projective | 0.2 / **644** | 6, **644** | 0.2 / 442 |
| box/stereo Euclidean | 0.0 / **669** | 6, **669** | 0.0 / **21** |
| box/stereo polar | 2.3 / 1197 | 7, 782 | 2.2 / 1197 |
| silhouette/stereo projective / Euclidean | 0.6 / 39, 0.2 / 44 | 25, ≤38 | unchanged |
| box/monocular projective / Euclidean | 89 / 5113, 104 / 6162 | 23–24, ≤1237 | 85 / 5113, 96 / 6162 |
| silhouette/monocular projective / Euclidean | 117 / 1459, 112 / 1481 | 24–25, ≤1481 | 121 / 1459, 116 / 1140 |

- **The stereo maximum of 644 mm is the leftover.** In `chaos_03` instance 1,
  restart6's upper-arm copy sits inside the target box at 1.97 m. r1 returned
  2.007 m (nearest-mode depth on the leftover); r2 returned 2.651 m; truth is
  2.70 m. The same trial gives the Euclidean maximum of 669 mm. Without the six
  in-box trials, box/stereo Euclidean drops to 21 mm.
- **Remaining stereo outliers are not the leftover.** Projective 442 mm
  (`chaos_04` #0) and polar 1,197 mm (`chaos_02` #2) have no leftover in view;
  those are realtime-renderer differences.
- **The ~100 mm monocular median is not the leftover.** Medians in trials
  without the leftover (85–121 mm) equal those with it. This pair used the
  realtime renderer, whose RGB differs between sessions in every trial, so the
  monocular median measures realtime-render variability through the depth
  network. The leftover adds individual large outliers, for example
  `twin_swap_01` #1 at 1,380 mm with the leftover inside the box at 1.27 m, but
  does not set the median or the maximum.
- **The restart11 Isaac pair is therefore doubly unusable as a repeatability
  reference.** It carries realtime render noise and a leftover in three of
  r1's four conditions.

**Other Isaac results.** Any single-session Isaac output can carry an
undetected leftover. Known contaminated ranges:
- restart6 r1: condition 1 from trial 54, and all of conditions 2–4;
- restart8 r2: from trial 54;
- new r1: condition 2 from trial 20, and conditions 3–4.

Analyses that used these sessions should be rechecked, for example the
Isaac–Gazebo outcome-gap work (memory `isaac_gazebo_outcome_gap`). Gazebo
results are unaffected.
