# Isaac port — open issues

Live register for the Isaac Sim 6.0 port. `port-plan.md` holds the phase plan
and what each phase delivered; this file holds what is **currently wrong**.
`../../ISSUES.md` is the repo-wide troubleshooting archive — issues move there
once they are solved and the writeup is worth keeping.

Last reviewed: **2026-09-11**. Branch `feat/isaac-sim-6-port`.

**Branch state, corrected 2026-09-11.** `feat/isaac-vendor-chassis` was not
"the same commit, redundant" — it was the branch actually carrying the newest
work. The `isaac` worktree was checked out on it, so the two handoff-doc
commits (`74583fbe`, `16afbda8`) landed there while `feat/isaac-sim-6-port`
and `origin/feat/isaac-sim-6-port` both sat behind at `15348256`.
`feat/isaac-sim-6-port` has since been fast-forwarded over them (no rewrite;
it was a strict ancestor) and the worktree now tracks it. `origin` is still
behind — these commits are **unpushed**. Check `git rev-parse --abbrev-ref HEAD`
before committing in that worktree.

---

## 🔴 Blocking

### 1. `collision_monitor` phantom returns — PRIMARY BAND FIXED, a second band remains

> **2026-09-11.** Root cause of the *original* latch found, fixed and verified;
> the robot now drives. It re-stalls later on a **second, distinct** phantom
> band that had never been recorded. Details in "What was settled" below —
> read that before re-testing anything here.

**Original symptom.** In `hospital` and `warehouse_full`, the robot planned but
never drove. Net odom displacement over 25 s was `0.000 m`, and
`collision_monitor` logged `Robot to approach ... away from collision`
*continuously*.

**Status after the fix** (`warehouse_full`, full stack, deterministic,
camera off, domain 78): robot drives to odom `(0.090, 0.357)` with ~41 deg of
rotation, the explorer issues frontier goals, and the collision log drops from
continuous to **11 occurrences**. It then re-stalls, with nav2 cycling
Spin/DriveOnHeading recoveries that exceed their time allowance.

#### What was settled on 2026-09-11, by measurement

**Cause of the original band.** Both lidars sit at a tip of the chassis's
diamond notch, whose edges run at **exactly ±45/±135 deg** — so each unit's
extreme ray is *tangent* to its own notch edge, which passes **34.8 mm** from
the emitter. Stationary, the rays miss it. Rotating, they clip it: returns at
**0.40–0.48 m, bearings +129…+135 deg, in ~9% of frames, up to 6 points —
exactly `collision_monitor`'s `min_points: 6`**. That closed a self-sustaining
loop: rotation → ≥6 phantom points → `cmd_vel` zeroed → nav2 recovers by
spinning → more rotation.

Reproduced **on demand without nav2**: a bare `cmd_vel` rotation against the
sim-only layer (`tools/isaac/stall_probe.py`, and the spin/probe recipe in
`../../tools/benchmark/README.md`). Clean stationary, phantoms while rotating,
both directions, clean again on stop.

**Fix.** `LidarScanAssembler.EDGE_MASK_DEG = 10.0` (`sim/isaac/ros_io.py`)
drops the outer 10 deg of each 270 deg window — 40 of 1081 bins per end, 7.4%
of the arc. The band does **not** end sharply (it thins inward: 6 points/frame
past 129 deg, 1–2 at 126.5), so 7 deg left a residual at its own boundary.
The two units are mounted back-to-back, so each masked sector lies inside the
other's arc: the merged scan, both costmaps and `collision_monitor` (all of
which take both scans as observation sources) keep full 360 deg coverage.
Regression test: `test/test_lidar_scan_assembler.py`. Masking beats raising
`min_points` — the tangency is real geometry that a real UST-10LX would also
see, while the *seam* is an artefact of the two-prim 180-deg/tick workaround
the 6.0.1 bridge forces on us (§7).

**Newly ruled out, each by measurement** (do not re-test):

| hypothesis | test | result |
|---|---|---|
| published footprint oversized/offset | echoed all three topics while stalled | **dead** — `collision_monitor/footprint_approach` is the configured octagon ±0.466/±0.395 plus nav2's default `footprint_padding: 0.01` → ±0.476/±0.405. Costmap copies are that same octagon rigidly placed in odom/map (all vertex radii about the centroid are 0.5322/0.5485) |
| assembler mis-bins edge azimuths | raw bridge clouds | **dead** — the hits exist pre-assembler, identical bearings/ranges, in `sensors/lidar2d_0/points_l` |
| robot's own visible body | triangle-sliced the robot USD at the scan plane, ray-cast the full arc (`tools/isaac/self_occlusion_check.py`) | **0/1081 bins** self-occluded. Slicer validated: 178–204 segments spanning y ±0.392 at z ≤ 0.20, collapsing to 8 at 0.2264 |
| robot collision meshes | same slice | **dead** — `vendor_chassis/.../collisions/*` are `purpose=guide`, so the RTX lidar (render meshes only) never sees them; they slice to the same diamond anyway |
| world geometry at the spawn | sliced `full_warehouse.usd` at the plane with prim attribution and no max-extent skip (`tools/isaac/world_probe.py`) | **nearest geometry 5.534 m** (`SM_WallA_6M14` at x=+5.46). GT map's `free` verdict is correct |
| rig asymmetry (prim config/pose) | dumped all four `OmniLidar` prims | **dead** — co-located at z=0.2264, identical config (single emitter, `elevationDeg=[0]`, `elevationErrorStd=0`, `nearRangeM=0.06`, `rangeAccuracyM=0.04`); only `startAzimuthOffsetDeg` differs (0 vs −135) |
| "only under the full launch stack" | bare runner vs sim-only vs full, same world/USD | **the framing was wrong** — it is *motion*, not the launch. Bare and sim-only are byte-identical (`points_l` degmax +114.78, nearest 4.895 m) because nothing commands the robot; the launch merely provides the rotation |

`rangeAccuracyM = 0.04` explains the ±2 cm scatter in the returns — do not
read that scatter as a surface shape.

#### The remaining band (open)

Once moving, `lidar2d_0/scan` shows ~15 returns at **+94.5…+97.5 deg, 0.42–
0.57 m**, which the ±125 deg mask does not cover. These cannot be world
geometry either: the robot had travelled only 0.37 m, so the nearest real
geometry was still ~5.2 m away. Candidate worth checking first: the side
covers' **top** face (`visuals/mesh_10` / `mesh_13`, both topping at z=0.220)
sits only **6.4 mm** below the 0.2264 scan plane, so a near-horizontal ray
grazes it — a tangency in elevation rather than azimuth, which would explain
why it appears at a bearing the azimuth mask cannot reach.

Beware when reading base_link coordinates near ±90 deg: `cos(95°) ≈ 0` pins
the computed x near the lidar's own 0.3922 offset, which looks like a flat
vertical surface and is not one.

**Next steps, in order.**
1. Test the side-cover elevation-tangency hypothesis above — if it holds, the
   fix is geometric (raise the scan plane or lower the covers), not a mask.
2. Re-run the full stack and confirm the robot explores rather than
   re-stalling; only then are the §3 baselines measurable.

```
cmd_vel_nav       267 msgs   (controller output, healthy)
cmd_vel_smoothed  201 msgs   (smoother output, healthy)
cmd_vel             0 msgs   <- collision_monitor emits nothing
```

`collision_monitor` logs `Robot to approach for 1.200000 seconds away from
collision` continuously. Its `FootprintApproach` polygon has `min_points: 6`
and it sees **13** lidar returns inside the projected footprint.

**The returns are not real.** All 13 sit at bearings **+132.00…+135.00°** —
the last 13 bins of the 270° window and nowhere else — all at ~0.53 m, all
mapping to `base_link` y ≈ +0.38…+0.40 (the robot's **left flank**), and the
ground-truth map says `free` at every corresponding world position. The rear
lidar shows zero. In `hospital` it was the rear lidar instead: whichever unit
faces geometry.

*(2026-09-11: "whichever unit faces geometry" was a red herring — it is
whichever unit's notch edge the current rotation clips.)*

**Ruled out, each by measurement (2026-09-10):**

| hypothesis | test | result |
|---|---|---|
| lidars see the robot's own body | `empty.usda`, nothing in scene | **0/1081** both lidars, before *and* after the chassis graft |
| grazing the side cover at 0.52 m | same | dead — this theory was wrong, see PORT_PLAN |
| real world geometry | bare runner, `warehouse_full` | **0 returns < 0.90 m**, 556 finite front / 154 rear |
| `--sim-mode deterministic` | bare runner + deterministic | **0 returns < 0.90 m** |
| assembler mis-bins edge azimuths | `ros_io.py:87` | bins outside `[0, 1081)` are correctly dropped |
| robot spawned against an obstacle | GT map, 3×3 m around spawn | all `free` |

**What that leaves.** ~~The phantom returns appear **only under the full launch
stack**, never under the bare runner in the same world with the same robot
USD. So it is something the launch adds or configures, not the world, not the
robot, not the sensor model.~~

**Superseded 2026-09-11.** The observation was right, the inference wrong.
Nothing the launch *configures* matters — the runner args are effectively
identical, and bare and sim-only runs are byte-identical. What the launch adds
is a robot that *rotates*, and rotation is the trigger. The row above marked
"grazing the side cover" as dead was also too strong: a tangency of exactly
this kind is the confirmed cause, just against the notch edge rather than the
side cover.

All three of the 2026-09-10 next steps (read the published footprint, diff the
runner args, check the raw clouds pre-assembler) were carried out on
2026-09-11 and are folded into the tables above. The footprint was innocent.

---

### 2. The robot floats 49.8 mm on every stock world

**Symptom.** Wheels visibly off the ground in `warehouse`, `warehouse_full`,
`office`, `hospital`.

```
wheel mesh bottom (base_link)   -0.0262
--spawn-z default               +0.0760
=> wheel bottom in world        +0.0498

mock_hospital floor top 0.0500  ->  -0.0002   seated
stock worlds  floor     0.0000  ->  +0.0498   FLOATING
```

**Cause.** `isaac_runner.py --spawn-z` defaults to `0.076`, tuned for
`mock_hospital`, whose floor slab top is at z = 0.05 (the argument's own help
text says so). Stock worlds put their floor at z = 0. Wheels carry no
colliders and the drive rig has no vertical joint, so nothing seats it.

**Why it matters more than the visual.** The scan plane rides up with the
robot, so in every stock world the lidars sample **5 cm higher** than the real
robot would:

| | now | if seated |
|---|---|---|
| stock worlds | 0.3024 | **0.2523** |
| mock_hospital | 0.3024 | 0.3023 |

The ground-truth maps regenerated on 2026-09-10 were sliced at 0.3024 to match
the *floating* robot. Self-consistent, so coverage numbers are not internally
broken — but both sides are 5 cm off from a correctly seated robot.

**Fix shape.** `spawn_z = floor_z + 0.0259` per world, and `LIDAR_PLANE_Z`
stops being a single constant: it becomes `floor_z + 0.2523`. Needs a per-world
floor height (repo worlds 0.05, stock worlds 0.0) or a floor query at spawn.

**Measured correction (2026-09-11).** Those two constants are 0.3 mm off. The
`0.0259` came from the runner's own help text (axle `0.05` − radius `0.0759`),
but the wheel *mesh* in the committed USD bottoms at **−0.02617** in
`base_link` (all four identical; bbox height 0.15234, so the radius is
0.07617, not 0.0759). Per the repo's own "verify against the mesh, not the
primitive" rule the mesh wins:

    spawn_z       = floor_z + 0.02617
    LIDAR_PLANE_Z = floor_z + 0.25257     (front lidar is 0.2264 above base_link)

0.3 mm is far below the 0.05 m map resolution, so it changes no GT map — but
use the measured pair so the numbers stop disagreeing between documents.

**Not the stall cause** — the bare runner floats identically and shows no
phantom returns.

---

## 🟠 Correctness debt

### 3. No Isaac baseline has ever been rerun

The original task. Still zero numbers produced. Sensor geometry changed **four
times** on 2026-09-10 (coplanar lidars, −11.6 cm mount, vendor chassis graft,
camera + mast + D455), and issue 1 means exploration cannot run at all in the
worlds that matter. Every coverage figure in `port-plan.md` predates all of it
and should be treated as void, not as a comparison point.

### 4. Ground-truth maps need one more regeneration

They are correct for the floating robot (issue 2) and correct for the current
rendered geometry. Seating the robot moves the slice to 0.2523 for stock
worlds and invalidates them again. Do the seat fix first, then regenerate once.

### 5. Hull collider physics unvalidated

The chassis collider changed from an AABB `Cube` to the vendor `convexHull`
(6.9% over-volume vs the box's 25.4%). Nothing has driven the robot into a
wall to confirm it stops where it should.

### 6. P5 A/B still needs multi-seed on a quiet box

Coverage variance ran 51–83% historically. Single runs prove nothing, and this
box is never genuinely single-tenant. Needs 3–5 seeds per condition.

---

## 🔵 Scheduled

### 7. Migrate to Isaac Sim 6.1

Not broken — planned. Several workarounds in this port exist only because of
6.0.1 defects and are candidates to delete on 6.1: the `LidarScanAssembler`
plus two-prim-per-lidar rig (the bridge ignores the azimuth ROI and fires
180°/tick), the importer's dropped meshes and instanceable STL slots, the
`fastShutdown` hard-exit, and the collider debug draw that will not enable.

**Sequenced after the first baseline, deliberately.** The upgrade is only
measurable against one, and issue 3 means none exists. Migrating first changes
the platform with nothing to compare against. Order: unblock nav → seat the
robot → one clean 6.0.1 baseline → migrate → rerun the identical benchmark.

Resequence only if issue 1 proves to be a 6.0.1 sensor-pipeline defect rather
than config — check the 6.1 notes for the `laser_scan` ROI fix. See
`port-plan.md` §P9.

**2026-09-11: this clause did NOT fire.** Issue 1's primary band is the
lidar's extreme ray being tangent to the chassis's own notch edge at ±135 deg
— real geometry that a real UST-10LX shares, at the *contract window* edge,
not at the 0 deg seam between the two prims. So it is not a 6.0.1 sensor
defect and does not justify migrating early. Keep the order: finish issue 1's
second band → seat the robot → one clean 6.0.1 baseline → migrate.

---

## 🟡 Known and accepted

- **`mock_hospital` has no ground-truth map.** Deliberate (`69b5182e`) — it is
  a Cube/Sphere world the mesh slicer skips, and the world is being retired.
  Its coverage HUD reads `n/a`.
- **`robot_geometry.svg` is hand-plotted** and does not self-correct when
  geometry moves. The D455 swap needed a manual redraw of the plan view
  (90 mm → 124 mm). Check it after any mounting change.
- **The mast's flared base plate is not modelled** — only the bare 37.5 mm
  column. The quoted 53.6 mm clearance above the scan plane is therefore
  optimistic. Still above the deck, so still clear.
- **PhysX collider debug draw cannot be enabled programmatically.** The carb
  keys, the `omni.physx` visualization interface and starting the timeline all
  report success and still draw nothing. Use the PhysX toolbar toggle.
  `inspect_robot.py --compare-colliders` sets the scene up regardless.
- **Payload layers churn 6 lines on every regen** — the importer stamps its
  `/tmp` staging paths into `doc` metadata. Cosmetic.
- **`.repos` pins branches, not commits.** A fresh `vcs import` pulls whatever
  upstream HEAD is that day; on 2026-09-10 all five deps came down different
  and `clearpath_gz_customizations.patch` stopped applying. Re-pin by hand
  after any import.

---

## Landmines that cost real time

- **`pkill -f` / `ps | grep <pattern>` matches the calling shell.** Hit three
  times in one session. The bracket trick (`[i]saac_runner`) guards against
  matching `grep`, not against matching your own command line when the launch
  command contains the pattern. Keep kills in a separate call from launches.
- **Kit runs with `--/app/fastShutdown=True`**, so `app.close()` hard-exits.
  Anything appended after `import_urdf_to_usd()` returns never runs, while the
  script still prints `IMPORT OK` and exits 0.
- **The importer `rmtree`s the whole committed robot directory** during its
  flatten step, taking any vendored asset with it. `import_ridgeback_urdf.py`
  now stashes and restores those.
- **The importer marks STL-derived visual slots instanceable**, and USD refuses
  to author into an instance proxy. Masked for months by the D435's DAE; the
  D455's bare STL exposed it.
- **Isaac stamps front and rear lidar identically**, so the scan merger always
  takes its static-extrinsic fast path. The motion-compensation branch is dead
  code in sim and is only exercised on hardware.
- **A stale `ros2 daemon` hides namespaced topics from the CLI.** `ros2 daemon
  stop`, or `--no-daemon`. rclpy probes are unaffected.
- **X11 capture needs the window raised**; an obscured window returns
  "Resource temporarily unavailable". `wmctrl -a` was unreliable here,
  `xdotool windowactivate` worked.

---

## Recently closed (2026-09-10)

| issue | commit |
|---|---|
| Scan merger dropped every rear scan (missing `/tf` remap) | `2ed1674a` |
| Rear lidar 5 cm above front (gz-era workaround) | `1969a707` |
| Lidars mounted 11.6 cm too high | `a33111c2` |
| GT maps sliced at the old lidar plane | `15987825` |
| Unknown map cells counted as free space | `5dd4e60e` |
| Imported chassis shell coarse, no rear panel | `eaea0674` |
| Graft wiped by regen | `70a5faa6` |
| Camera 12.5 cm too high, mast missing | `d26edef5` |
| Camera was a D435, is a D455 | `53a924a6` |
