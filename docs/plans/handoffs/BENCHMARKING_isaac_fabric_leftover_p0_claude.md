# BENCHMARKING — Isaac Fabric leftover audit (P0): implementation handoff (Claude)

Written: 2026-11-11, Europe/Berlin, by Claude. This is an **implementation
handoff**: the user chose fix **P0** from the leftover-G1 investigation, and
this file is the complete brief for the agent that implements it.

| | |
|---|---|
| Tested revision | `dev` at `cc86eef`, plus an unrelated dirty working tree |
| Evidence | `/home/stefi/DyNAMO/DyNAMO-Ridgeback/docs/plans/handoffs/BENCHMARKING_isaac_leftover_g1_findings.md`. Read sections 1–4 first. It and this handoff are untracked files in the main checkout, so a new worktree will not contain them; read them from that path. |
| Hardware | none; Isaac Sim only |

## 1. Problem in one paragraph

Benchmark scene control deletes spawned entities with `stage.RemovePrim`
(`UsdScene.delete`). Isaac Sim 6.1.0-rc.26 runs with the Fabric Scene Delegate
(`app.useFabricSceneDelegate = true`), so the renderer draws **Fabric**, not
USD. About once per 500–760 G1 deletes, Fabric keeps **one prim** of the
removed subtree: a visual `Mesh` or a collision `MeshInstance`, with its
points and its world matrix at the final pose. USD is clean, so every
existing check (registry, `GetEntities`, the benchmark's `state_mismatch`
check) passes. A leaked visual mesh keeps rendering in RGB and depth for the
rest of the session. That contaminated 3 of 5 full Isaac sessions, including
the restart11 qualification pair's 644 mm box/stereo maximum.

A standalone reproducer (Appendix A) leaked 10 times in 5000 cycles. Each
time, `usdrt` found the stale prim under the deleted path, and
`usdrt_stage.RemovePrim` on it restored the empty render (9 of 9 tried).
Physics, `freeze_dynamics`, deactivation, materials and instancing are ruled
out. Leaks cluster early in a session and appear only with new prim paths.

## 2. Goal and scope

**Goal:** the Isaac scene provider never leaves a deleted entity's prim in
Fabric unnoticed. It repairs any it finds, and if a repair fails, the
benchmark fails loudly through its existing paths instead of capturing with
the leftover.

**In scope:**
- `src/ridgeback_autonomy_isaac/sim/isaac/usd_scene.py`
- `src/ridgeback_autonomy_isaac/sim/isaac/scene_provider.py`
- a minimal wiring change in
  `src/ridgeback_autonomy_isaac/sim/isaac/isaac_runner.py`
- tests under `src/ridgeback_autonomy_isaac/test/`
- one reproducer tool under `tools/isaac/`
- the owning documentation (section 8)

**Out of scope:**
- **P1, P2 and P3 from the findings doc** (fixed slot paths, a spawn-once pool,
  restarts per condition).
- **Benchmark-client changes** in `ridgeback_autonomy/benchmarking/`. Their
  existing failure paths are enough (section 4, C3).
- **The optional segmentation guard** (findings doc P0, layer 3).
- **Corrections to past records:** the archive record, and the `semantics.md`
  paragraph that attributes the 1.376 m monocular tail to RGB sensitivity.
  Mention them to the user; do not fold them into this change.

## 3. Hard constraints

1. **Frozen bundle.**
   `artifacts/benchmarks/isaac_repeatability_prepared_20261105T234856Z` has
   status `live-complete-offline-pending`. Its `frozen.json` hashes every file
   under `src/<all six packages>`, plus `install/<pkg>/lib/<pkg>`,
   `clearpath/robot.yaml` and `tools/benchmark/qualification/`. Any edit or new
   file there in the **main checkout** makes `check_frozen` refuse
   `start-offline`. Therefore:
   - Work in a **separate git worktree** on a new branch from `dev`, for
     example `git worktree add ../DyNAMO-Ridgeback-p0 -b feat/isaac-fabric-audit dev`.
     Build and test there, overlaid on the main install.
     - Recipe (memory `concurrent_sessions_hazard`): source the main
       `install/setup.bash`, then the worktree's.
     - Symlink `isaac_venv` and `perception_venv` into the worktree root.
     - Verify with `ros2 pkg prefix ridgeback_autonomy_isaac`.
   - Do not touch `src/`, `install/` or `tools/benchmark/qualification/` in the
     main checkout until the user says that bundle's offline step is done or
     abandoned. Ask if unsure.
   - The main checkout has unrelated uncommitted work, including an
     `isaac_runner.py` render-policy diff. Leave it alone. The worktree starts
     from committed `dev`, so the runner there lacks that diff; keep your
     runner edit minimal so it merges cleanly.
2. **Shared machine.**
   - Before launching Isaac, check GPU and processes; the `box-health` agent
     does this.
   - Never stop another user's processes. Never run `cleanup.sh` while
     someone else's simulation runs.
   - Use a unique `ROS_DOMAIN_ID` (≤ 232) for anything ROS. Use
     `ROS_DOMAIN_ID=77` for pytest, which hangs on domain 0.
   - No full benchmark sweeps without the user's go-ahead; reproducer loops
     and short smoke runs are fine.
   - Start a run only when no other user's simulator is running.
3. **Main loop only.** Every `usdrt` call runs on the SimulationApp main loop,
   inside `SceneControlService.process_pending`. Never call it from the
   service executor thread (`scene_provider.py` module docstring).
4. **No added cost on the clean path.** When nothing leaks, the change must add
   no USD edits, no extra `app.update()` and no added latency to any service
   response. That keeps deterministic sessions bit-identical to today.
5. Commit on your branch with the repository conventions: Conventional
   Commits, no plan or audit ids in code comments. Run
   `tools/rebuild_graphify` after code changes. **Do not push or merge**
   without the user's OK.

## 4. Behaviour contract

Call a *Fabric residue* any Fabric prim whose path lies under
`/scene_entities/<name>` when `<name>` is not a live registry entity.

- **C1 — audit after every delete.** After `SceneEntities.delete(name)`
  succeeds in USD, the provider audits Fabric for residue under that entity's
  prim path. The audit runs on every update from `G` updates after the delete
  until `G + N`. `G` is the measured normal Fabric-removal latency (section 6,
  step 1); `N` defaults to 10. The audit is **asynchronous**: the
  `DeleteEntity` response and the registry removal happen exactly as today.
- **C2 — repair.** Residue is removed from Fabric deepest path first
  (`usdrt_stage.RemovePrim`) and re-checked on the next update. Every finding
  and repair is one structured log line with the prefix `SCENE_FABRIC_RESIDUE`
  and JSON fields: `event` (`found`, `repaired` or `persistent`), `entity`,
  `prims`, `attempt`, `update`, `sim_time`. The line goes to stdout, so it
  lands in `isaac_rN.log`.
- **C3 — fail loudly if repair fails.** Residue still present after `K = 3`
  repair attempts is **persistent**. The provider then lists `<name>` again
  in `GetEntities` and `GetEntitiesStates` as a zombie, with its last known
  pose, and logs `persistent` at error level. The benchmark's existing paths
  then fail without client changes:
  - the next trial's state check fails with `state_mismatch`, an unexpected
    `bench_` entity (`docs/target_distance_benchmarking/semantics.md`, run
    sequence);
  - any `delete_entity(name)` confirmation raises `CLEANUP_INCOMPLETE`;
  - `run_trial` logs the trial as failed and skips it.

  A zombie can be deleted again. That repeats the Fabric removal and drops the
  zombie only when Fabric is clean.
- **C4 — scope sweep.** A sweep lists every Fabric prim under
  `/scene_entities`. Anything not owned by a live entity, and not inside a
  recent delete's grace window (fewer than `G` updates old), is residue, with
  the same repair and zombie rules. The zombie's name is the first path
  component under the scope. The sweep runs:
  - before answering `GetEntities` and `GetEntitiesStates`, so the
    benchmark's absence confirmation and state check see the truth;
  - on the update after any spawn or delete batch.

  Measure its cost (section 6) and keep it off updates with no scene activity.
- **C5 — no clean-path change:** see constraint 4.
- **C6 — any prim type.** Residue includes collision `MeshInstance` prims with
  `guide` purpose, not only rendered meshes. They do not render, but they
  prove the leak and must not accumulate.

## 5. Suggested design

Deviate only with a written reason in your report.

**`usd_scene.py`**
- Give `UsdScene` an optional `stage_id` constructor argument. The runner
  passes `omni.usd.get_context().get_stage_id()`. Attach lazily with
  `usdrt.Usd.Stage.Attach(stage_id)`, imported inside the method, because the
  module must keep importing without Kit for the pxr-only tests.
- `fabric_prims_under(prefix) -> list[str]`:
  - Query `ustage.GetPrimsWithTypeName('Mesh')` and filter by
    `str(path).startswith(prefix + '/')` or equality. In the reproducer this
    query also returned the collision `MeshInstance` residue.
  - Check whether other gprim types (`Cylinder`, `Sphere`, `Xform`) can leak.
    Add them to the query if cheap, or state why not.
  - **Do not** rely on `GetPrimAtPath(entity_root)` plus `PrimRange`. After
    the USD delete the parent may be gone from Fabric while an orphaned
    descendant remains.
- `remove_fabric_prims(paths) -> list[str]`: deepest first; return what
  `RemovePrim` reported removed. Optionally call
  `usdrt.hierarchy.IFabricHierarchy().get_fabric_hierarchy(fabric_id, stage_id).update_world_xforms()`
  afterwards. Isaac's own `isaacsim.robot_setup.sysid` bridge does this
  (`IsaacSimSysIdBridge._remove_fabric_prim`); keep it only if a test shows
  it matters.
- When `stage_id` is `None` (pxr-only tests), both methods are no-ops that
  return empty lists.

**`scene_provider.py`**
- Extend the `NativeScene` protocol with the two methods above.
- `SceneEntities` keeps a deletion ledger: `{name: (prim_path, deleted_at_update, attempts, last_pose)}`.
  Read `last_pose` through `native.pose()` before calling `native.delete()`.
- Add `audit(update_index) -> list[dict]`, which runs C1, C2 and C3. Add
  `sweep(update_index)`, which runs C4.
- `names()`, `select()` and `states()` include zombies. `delete(name)` accepts
  a zombie and runs the Fabric removal again.
- `SceneControlService.process_pending(frame)` calls
  `self._entities.audit(...)` **before** `self._queue.drain(frame)`, so
  requests drained in the same update see the result. Run the sweep before
  answering entity queries: call it inside the queued operation for
  `_on_entities` and `_on_states`.
- Use an update counter that `process_pending` increments; `frame.sim_time`
  is not a frame count in realtime mode.

**`isaac_runner.py`**
- Construct `UsdScene(stage, stage_id=ctx.get_stage_id())`.
- No other changes; `process_pending` already runs once per update.

## 6. Steps

1. **Measure `G`.** Before writing provider code, extend the reproducer
   (Appendix A) with a mode that, for at least 200 deletes, counts Fabric
   prims under the just-deleted path at +0, +1, +2, … +6 updates after
   `RemovePrim`. `G` is the first offset where clean deletes always show 0;
   add one update of margin. Record the distribution in your report.
   - If removal is already reflected at +0, a synchronous check inside
     `delete` becomes possible. Still keep the asynchronous audit, because
     leaks may appear on later updates.
   - Also time one `fabric_prims_under` call on the benchmark world with
     background props spawned. You need that cost for C4.
2. **Unit tests first,** in `test_scene_provider.py`. That file needs the ROS
   Python because it imports `simulation_interfaces`. Extend
   `FakeNativeScene` with a fake Fabric map and residue injection. Cover:
   - a clean delete: no removal calls, no zombie, no log lines;
   - residue at `G + 2`: found, removed, gone, two log lines, no zombie;
   - residue whose removal is a no-op: a zombie after `K` attempts, listed by
     `names()` and `states()` with its last pose, and deletable again;
   - the sweep finding an orphan path from a never-registered name, which
     becomes a zombie named after its first path component;
   - the grace window: no false residue for a delete fewer than `G` updates
     old;
   - `process_pending` running the audit before drain;
   - the clean path: no extra queue operations and no extra updates.

   Keep the existing pxr-only tests in `test_usd_scene.py` passing. The new
   `UsdScene` methods are no-ops there.
3. **Implement** sections 4 and 5.
4. **Kit verification.** Add the reproducer as `tools/isaac/fabric_leftover_repro.py`
   with a `--through-provider` mode. In that mode, spawns and deletes go through
   `SceneEntities(UsdScene(stage, stage_id=...))`, and each update calls the
   provider's audit, as `process_pending` would. Run it with ROS sourced in the
   Isaac interpreter, as the runner does, so `scene_provider` imports without
   the stub. Acceptance A2 and A3 use it.
5. **Short benchmark smoke.** Start it only when the box is free, and use the
   runbook commands in `docs/target_distance_benchmarking/running_benchmarks.md`.
   The `scene_ids` argument is described in `launch_arguments.md`.
   - Isaac, `scene_ids:=objpartial_01,multi_03,chaos_03,interfere_pair_03`,
     with the background lab, for one condition.
   - Confirm zero `state_mismatch`, unchanged trial timings, and that any
     `SCENE_FABRIC_RESIDUE` lines are well-formed.
   - A full 4-condition session pair is the user's call (A5).
6. **Documentation** (section 8), graphify rebuild, commit on the branch, and a
   report to the user with every acceptance result and the measured `G`, `N`
   and `K`.

## 7. Acceptance tests

| # | Test | Pass |
|---|---|---|
| A1 | Unit tests (step 2) with `ROS_DOMAIN_ID=77`, plus the package's existing tests | All pass; the existing `test_scene_provider.py`, `test_scene_control_service.py`, `test_scene_command_queue.py` and `test_usd_scene.py` are unchanged in outcome |
| A2 | Reproducer, `--through-provider`, the leaky configurations: `--variant deactivate --with-object`, `--variant nofreeze --with-object`, `--variant unique --with-object --play 0`. At least 3000 cycles in total | The provider logs `found` and then `repaired` at least once; about 6 are expected (rate about 1 per 500). The reproducer's own independent checks, a depth comparison against the empty baseline and a Fabric query taken after the audit window, find **0** persistent residues. No `persistent` lines. |
| A3 | Reproducer control, `--variant reuse --with-object`, 1000 cycles, through the provider | 0 `found` lines, so no false positives. The clean path adds no updates; cycle time is within noise of an unpatched run. |
| A4 | Persistent path in Kit: a short run with `remove_fabric_prims` patched to a no-op, which needs a leak to occur, or with a Fabric-only prim injected under `/scene_entities/bench_x_target_0/...` | A zombie appears in `GetEntities`, a `persistent` line is logged, and a following `SceneControlClient.delete_entity` call raises `CLEANUP_INCOMPLETE` |
| A5 | User's call: one fresh 4-condition Isaac session pair on the branch | No `persistent` lines and no `state_mismatch` trials. Report any `repaired` lines. Box/stereo projective and Euclidean bit-identical between sessions; silhouette/stereo and monocular deltas 0 wherever RGB is bit-identical |

## 8. Documentation, in the same commit as the code

- `docs/target_distance_benchmarking/semantics.md`, run-sequence and
  state-check paragraph: one or two sentences saying that the Isaac adapter
  audits its render scene (Fabric) after deletes, repairs residue, and exposes
  unrepairable residue as an unexpected `bench_` entity. That is why a
  `state_mismatch` can name an entity the run already deleted.
- `docs/troubleshooting.md`: an entry for `SCENE_FABRIC_RESIDUE persistent`,
  giving the meaning, the Kit version, and the action (fresh simulator
  session; report with the reproducer).
- Ask the `doc-steward` agent to confirm these owners before committing
  (`docs/project/documentation.md`).
- Do not edit the handoff or findings files beyond a status line. The user
  decides when they move to `archive/engineering/`.

## 9. Known unknowns

- Whether Fabric reflects a `RemovePrim` immediately or only after an update.
  Step 1 measures it.
- Whether a stale prim can surface on an update *later* than the audit window.
  The C4 sweep before every entity query is the safety net. If you ever see a
  `found` from the sweep rather than the audit, report it.
- Whether any USD prim types other than meshes leak and matter.
- Root mechanism in Kit (inferred: Fabric table growth with new paths). It is
  not needed for P0. The findings doc suggests reporting it upstream with the
  reproducer; that is the user's decision.

## Appendix A — reproducer (verbatim copy of the investigation script)

The original lives in a session scratchpad that may disappear. To use it from
a worktree, set `REPO` to that checkout. The `scene_provider` stub exists only
so the script runs without ROS; drop it in `--through-provider` mode.

**Run:**
`OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 fabric_leftover_repro.py --variant deactivate --with-object --cycles 1000 --out <dir>`

**Output:** `<dir>/cycles.jsonl`, one record per cycle, plus `event` lines
(`FABRIC_LEFTOVER`, `GHOST`, `repair_fabric_remove`, `done`). Kit corrupted
its heap in `app.close()` during the investigation, so the script ends with
`os._exit(0)`.

```python
#!/usr/bin/env python3
"""Spawn/delete the benchmark G1 through usd_scene.UsdScene many times and
look for render-side leftovers (depth vs empty baseline + Fabric query)."""
from __future__ import annotations

import argparse, json, math, os, sys, time, types

ap = argparse.ArgumentParser()
ap.add_argument('--variant', default='unique',
                choices=['unique', 'reuse', 'deactivate', 'nofreeze', 'settle', 'churn', 'pool'])
ap.add_argument('--cycles', type=int, default=1000)
ap.add_argument('--present-frames', type=int, default=3)
ap.add_argument('--gap-frames', type=int, default=3)
ap.add_argument('--settle-frames', type=int, default=30, help='gap frames for --variant settle')
ap.add_argument('--with-object', action='store_true', help='spawn/delete hospital_bed like objpartial_01')
ap.add_argument('--spot', default='8.36,0.31,3.62')
ap.add_argument('--cam', default='4.20,0.00,1.19')
ap.add_argument('--renderer', default='RealTimePathTracing')
ap.add_argument('--headless', default='true')
ap.add_argument('--max-minutes', type=float, default=25.0)
ap.add_argument('--repair', type=int, default=1, help='try Fabric RemovePrim on a detected leftover')
ap.add_argument('--play', type=int, default=1, help='0: never start the timeline (no PhysX stepping)')
ap.add_argument('--out', required=True)
args, kit_args = ap.parse_known_args()
sys.argv = [sys.argv[0], *kit_args]
os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')

REPO = os.environ.get('REPO', '/home/stefi/DyNAMO/DyNAMO-Ridgeback')
ISAAC = f'{REPO}/src/ridgeback_autonomy_isaac/sim/isaac'
G1 = f'{ISAAC}/usd/models/g1/g1.usda'
BED = f'{ISAAC}/usd/models/hospital_bed/hospital_bed.usda'
WORLD = f'{ISAAC}/usd/worlds/g1_distance_calibration.usda'
W, H, FX = 320, 240, 192.0
os.makedirs(args.out, exist_ok=True)
LOG = open(os.path.join(args.out, 'cycles.jsonl'), 'a')

def log(**kw):
    kw['wall'] = round(time.time(), 3)
    LOG.write(json.dumps(kw) + '\n'); LOG.flush()
    if kw.get('event'):
        print('REPRO', json.dumps(kw), flush=True)

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({'headless': args.headless == 'true', 'renderer': args.renderer,
                     'anti_aliasing': 0, 'fast_shutdown': False})

import numpy as np  # noqa: E402
import omni.timeline, omni.usd  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import usdrt  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, PhysxSchema  # noqa: E402

# usd_scene imports two names from scene_provider, which pulls in ROS message
# types at import; stub exactly those two names so the code under test
# (UsdScene.spawn/delete, freeze_dynamics) is the tracked file, unmodified.
stub = types.ModuleType('scene_provider')
class InvalidResource(Exception):
    pass
stub.InvalidResource = InvalidResource
stub.SPAWNED_ENTITY_SCOPE = '/scene_entities'
sys.modules['scene_provider'] = stub
sys.path.insert(0, ISAAC)
import usd_scene  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(WORLD)
for _ in range(30):
    app.update()
stage = ctx.get_stage()
if not any(p.GetTypeName() == 'PhysicsScene' for p in stage.Traverse()):
    UsdPhysics.Scene.Define(stage, '/physicsScene')
if args.variant == 'nofreeze':
    usd_scene.freeze_dynamics = lambda prim: 0   # module global used by spawn()
scene = usd_scene.UsdScene(stage)

sx, sy, syaw = (float(v) for v in args.spot.split(','))
cx, cy, cz = (float(v) for v in args.cam.split(','))
cam = UsdGeom.Camera.Define(stage, '/repro_cam')
cam.CreateFocalLengthAttr(FX * 0.02)
cam.CreateHorizontalApertureAttr(W * 0.02)
cam.CreateVerticalApertureAttr(H * 0.02)
cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 100.0))
view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(cx, cy, cz), Gf.Vec3d(sx, sy, 0.9), Gf.Vec3d(0, 0, 1))
xf = UsdGeom.Xformable(cam.GetPrim()); xf.ClearXformOpOrder()
xf.AddTransformOp().Set(view.GetInverse())
CAM2W = np.array(view.GetInverse(), float)   # row-vector convention: p_w = p_c @ CAM2W

rp = rep.create.render_product('/repro_cam', (W, H))
ann = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
ann.attach(rp)
timeline = omni.timeline.get_timeline_interface()
timeline.set_end_time(1.0e9); timeline.set_looping(False)
if args.play:
    timeline.play()

def step(n=1):
    for _ in range(n):
        app.update()

def depth():
    d = ann.get_data()
    if isinstance(d, dict):
        d = d.get('data')
    d = np.asarray(d, dtype=np.float32).squeeze()
    return d if d.shape == (H, W) else None

def backproject(v, u, z):
    pc = np.stack([(u - W / 2) * z / FX, -(v - H / 2) * z / FX, -z, np.ones_like(z)], -1)
    return (pc @ CAM2W)[:, :3]

ustage = usdrt.Usd.Stage.Attach(ctx.get_stage_id())
def fabric_under(prefix='/scene_entities'):
    """Fabric Mesh prims (MeshInstance residue showed up here too) under prefix."""
    out = []
    try:
        for p in ustage.GetPrimsWithTypeName('Mesh'):
            s = str(p)
            if s.startswith(prefix):
                out.append(s)
    except Exception as exc:  # API drift: record, never fatal
        return [f'ERR {exc}']
    return out

def usd_children(prefix='/scene_entities'):
    p = stage.GetPrimAtPath(prefix)
    return [str(c.GetPath()) for c in p.GetChildren()] if p and p.IsValid() else []

quat = (0.0, 0.0, math.sin(syaw / 2), math.cos(syaw / 2))

# warm up and baseline
step(60)
base = None
for _ in range(60):
    step(1); base = depth()
    if base is not None and np.isfinite(base).any():
        break
if base is None and not args.play:
    base = np.full((H, W), np.nan, np.float32)   # no-play: Fabric query is the only detector
assert base is not None, 'no depth from annotator'
step(10); d0 = depth(); base = d0.copy() if d0 is not None else base
np.save(os.path.join(args.out, 'baseline.npy'), base)
log(event='baseline', finite=int(np.isfinite(base).sum()),
    median=float(np.nanmedian(base)) if np.isfinite(base).any() else None, play=args.play)

def ghost_pixels(d, ref):
    if not args.play or d is None:
        return np.zeros((H, W), bool)
    ok = np.isfinite(d) & np.isfinite(ref)
    return ok & (d < ref - 0.03)

# latency probe: frames until the spawned G1 appears / disappears in depth
lat = {}
p0 = '/scene_entities/bench_repro_latency_target_0'
scene.spawn(p0, G1, (sx, sy, 0.05), quat)
for k in range(1, 15):
    step(1); n = int(ghost_pixels(depth(), base).sum())
    if n > 50:
        lat['appear_frames'] = k; break
scene.delete(p0)
for k in range(1, 15):
    step(1); n = int(ghost_pixels(depth(), base).sum())
    if n == 0:
        lat['vanish_frames'] = k; break
log(event='latency', **lat)
gap = max(args.gap_frames, lat.get('vanish_frames', 3) + 1)
if args.variant == 'settle':
    gap = args.settle_frames

POOL = '/scene_entities/bench_repro_pool_target_0'
PARK = (sx, sy + 30.0, 0.05)    # far outside the probe camera's view
if args.variant == 'pool':
    scene.spawn(POOL, G1, PARK, quat)
    UsdGeom.Imageable(stage.GetPrimAtPath(POOL)).MakeInvisible()
    pool_translate = UsdGeom.Xformable(stage.GetPrimAtPath(POOL)).GetOrderedXformOps()[0]
    step(10)
    base = depth().copy()      # baseline with the parked, hidden pool G1 present
    log(event='pool_ready', translate_op=str(pool_translate.GetOpName()))

t0 = time.time(); ghosts = 0; known_fab = set()
for i in range(args.cycles):
    if time.time() - t0 > args.max_minutes * 60:
        log(event='timebox', cycle=i); break
    name = 'bench_repro_target_0' if args.variant == 'reuse' else f'bench_repro_{i:05d}_target_0'
    path = f'/scene_entities/{name}'
    obj = f'/scene_entities/bench_repro_{i:05d}_obj_0'
    if args.with_object:
        scene.spawn(obj, BED, (sx - 1.11, sy - 0.43, 0.05), (0, 0, math.sin(0.305), math.cos(0.305)))
        step(1)
    if args.variant == 'pool':
        pool_translate.Set(Gf.Vec3d(sx, sy, 0.05))
        UsdGeom.Imageable(stage.GetPrimAtPath(POOL)).MakeVisible()
    else:
        scene.spawn(path, G1, (sx, sy, 0.05), quat)
    step(args.present_frames)
    present_px = int(ghost_pixels(depth(), base).sum())   # the G1 must be visible here
    if args.variant == 'churn':
        scene.spawn(path + '_b', G1, (sx + 1.5, sy - 1.0, 0.05), quat); step(1)
    if args.variant == 'deactivate':
        stage.GetPrimAtPath(path).SetActive(False); step(1)
    if args.variant == 'pool':
        UsdGeom.Imageable(stage.GetPrimAtPath(POOL)).MakeInvisible()
        pool_translate.Set(Gf.Vec3d(*PARK))
        ok = True
    else:
        ok = scene.delete(path)
    if args.variant == 'churn':
        scene.delete(path + '_b')
    step(1)
    if args.with_object:
        scene.delete(obj)
    step(gap - 1)
    d = depth(); m = ghost_pixels(d, base); n = int(m.sum())
    fab = [p for p in fabric_under() if not p.startswith(POOL)]
    rec = dict(cycle=i, deleted=bool(ok), present_px=present_px, ghost_px=n, fabric_meshes=len(fab),
               usd_children=len(usd_children()))
    new_fab = [p for p in fab if p not in known_fab]
    if new_fab:
        info = []
        for fp in new_fab[:20]:
            try:
                fpr = ustage.GetPrimAtPath(fp)
                def val(nm):
                    try:
                        a = fpr.GetAttribute(nm)
                        v = a.Get() if a else None
                        return None if v is None else str(v)[:120]
                    except Exception as exc:
                        return f'ERR {exc}'
                pts = fpr.GetAttribute('points').Get() if fpr and fpr.GetAttribute('points') else None
                info.append(dict(path=fp, type=str(fpr.GetTypeName()) if fpr else None,
                                 n_points=(len(pts) if pts is not None else None),
                                 purpose=val('purpose'), world_extent=val('_worldExtent'),
                                 world_matrix=val('omni:fabric:worldMatrix')))
            except Exception as exc:
                info.append(dict(path=fp, err=str(exc)))
        log(event='FABRIC_LEFTOVER', cycle=i, depth_px=n, prims=info)
    if n > 0 or new_fab:
        step(10); d2 = depth(); m2 = ghost_pixels(d2, base); n2 = int(m2.sum())
        rec['confirm_px'] = n2
        if n2 > 0 or new_fab:
            ghosts += 1
            pts_info = {}
            if n2 > 0:
                v, u = np.nonzero(m2)
                w = backproject(v, u, d2[v, u])
                pts_info = dict(world_min=w.min(0).round(3).tolist(), world_max=w.max(0).round(3).tolist())
            np.save(os.path.join(args.out, f'ghost_{i:05d}_depth.npy'), d2)
            log(event='GHOST', cycle=i, px=n2, fabric_new=new_fab[:50], usd_children=usd_children(),
                path=path, **pts_info)
            repaired = None
            if new_fab and not new_fab[0].startswith('ERR') and args.repair:
                for fp in sorted(new_fab, key=lambda s: s.count('/'), reverse=True):
                    try:
                        log(event='fabric_remove', prim=fp, result=str(ustage.RemovePrim(fp)))
                    except Exception as exc:
                        log(event='fabric_remove_error', prim=fp, err=str(exc))
                step(10); repaired = int(ghost_pixels(depth(), base).sum())
                log(event='repair_fabric_remove', remaining_px=repaired, fabric_after=fabric_under()[:20])
            known_fab.update(fabric_under())
            if repaired is None or repaired > 0:
                base = depth().copy()   # re-baseline so later leftovers are counted separately
                log(event='rebaseline', cycle=i)
    log(**rec)
    if i % 50 == 0:
        print(f'cycle {i} ghosts {ghosts} rate {(i + 1) / max(time.time() - t0, 1e-9):.2f}/s', flush=True)

log(event='done', cycles=i + 1, ghosts=ghosts, seconds=round(time.time() - t0, 1), variant=args.variant,
    gap_frames=gap, present_frames=args.present_frames, with_object=args.with_object)
LOG.close()
sys.stdout.flush()
os._exit(0)   # Kit 6.1 rc26 corrupted its heap in app.close() during the investigation
```

**Investigation results with this script,** 1000 cycles unless noted:

| Variant | Leftovers |
|---|---|
| `unique` + bed | 1, and 0 in a repeat |
| `deactivate` + bed | 3 |
| `nofreeze` + bed | 3 |
| `--play 0` + bed | 3 |
| `reuse` + bed | 0 / 3500 |
| `pool` + bed | 0 / 1800 |
| `settle` + bed | 0 |
| G1 only, no bed | 0 |

A cycle runs at about 3–5 per second, plus about 40 s of Kit startup.
