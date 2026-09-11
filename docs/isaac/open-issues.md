# Isaac port — open issues

Live register for the Isaac Sim 6.0 port. `port-plan.md` holds the phase plan
and what each phase delivered; this file holds what is **currently wrong**.
`../../ISSUES.md` is the repo-wide troubleshooting archive — issues move there
once they are solved and the writeup is worth keeping.

Last reviewed: **2026-09-10**. Branch `feat/isaac-sim-6-port` @ `53a924a6`
(pushed). `feat/isaac-vendor-chassis` points at the same commit and is
redundant.

---

## 🔴 Blocking

### 1. Nav2 never moves the robot — `collision_monitor` latches on phantom returns

**Symptom.** In `hospital` and `warehouse_full`, the robot plans but never
drives. Net odom displacement over 25 s is `0.000 m`.

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

**Ruled out, each by measurement:**

| hypothesis | test | result |
|---|---|---|
| lidars see the robot's own body | `empty.usda`, nothing in scene | **0/1081** both lidars, before *and* after the chassis graft |
| grazing the side cover at 0.52 m | same | dead — this theory was wrong, see PORT_PLAN |
| real world geometry | bare runner, `warehouse_full` | **0 returns < 0.90 m**, 556 finite front / 154 rear |
| `--sim-mode deterministic` | bare runner + deterministic | **0 returns < 0.90 m** |
| assembler mis-bins edge azimuths | `ros_io.py:87` | bins outside `[0, 1081)` are correctly dropped |
| robot spawned against an obstacle | GT map, 3×3 m around spawn | all `free` |

**What that leaves.** The phantom returns appear **only under the full launch
stack**, never under the bare runner in the same world with the same robot
USD. So it is something the launch adds or configures, not the world, not the
robot, not the sensor model.

**Live hypothesis — the published footprint.** `collision_monitor` does not
consult the PhysX collider at all; it projects
`local_costmap/published_footprint`. That topic has **never been read**. Every
"inside the footprint" claim so far, including the table above, used an
*assumed* 0.9325 × 0.7932 hull. If the published footprint is oversized or
offset, returns that clear the real robot still trip `FootprintApproach`.

**Next steps, in order.**
1. Echo `/r100_0001/local_costmap/published_footprint` while stalled and
   compare against the measured hull. This is the cheapest test and it is the
   one that has been skipped twice.
2. Diff the remaining runner args between bare and launch: `--animate-g1`,
   `--rtf`, `--sensor-hz`, `--namespace`, `--livestream`. `--sim-mode` is
   already cleared.
3. If the footprint is fine, subscribe to the raw `points`/`points_l` clouds
   and check whether the 0.53 m hits exist pre-assembler.

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
