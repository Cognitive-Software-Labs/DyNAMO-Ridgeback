# Realistic simulation environments

The `realistic` scene profile adds furnished worlds and a varied pedestrian
cast to the crowd runner. Its default world is a modern open-plan office; the
earlier office, warehouse and courtyard worlds remain available. The existing
F1/F2/F3 benchmark results apply to their recorded profiles; they do not
qualify these worlds or characters. The observation robot remains stationary.

## Worlds

| `--environment` | Contents |
|---|---|
| `modern_office` (default) | Light-oak floor, warm-white walls, full-height south glazing with slim mullions, linear LED ceiling runs, window workstations with monitors and task chairs, two glazed meeting rooms with oak tables, wall displays and pendants, a lounge with sofa, mid-century chairs, rug and cube shelving, and an oak-slatted reception wall with a marble-topped counter |
| `office` | Earlier lobby: wood floor, cove lights, glazed frontage, reception, desks, lounge furniture and room signs |
| `warehouse` | Concrete floor, roof/skylight and clerestory windows, storage racks, crates, barrels, aisle markings, loading door and pedestrian signs |
| `courtyard` | Paved walkways, planted grass/shrub beds, trees, benches, street lamps, bins, wayfinding and windowed building facades |

The modern office keeps pedestrian lanes at x 0.5-21.5 m and |y| <= 4.2 m
clear of furniture; tests check every route against collision footprints.
Workstations, partitions, glazing, sofa and task chairs are procedural boxes
with Poly Haven surfaces; lounge chairs, coffee and side tables, shelving,
credenza, pendants and plants are Poly Haven models. Monitors, wall displays,
LED runs and a faint ceiling glow are emissive. Isaac lights the office with
the daylight HDRI, a low south-west sun through the glazing, window fill and
one rectangular light per LED run. Gazebo uses a directional sun, eight
ceiling point lights and emissive fixtures, and keeps texture pixels unscaled
by palette colours in this world.

## People and motion

The cast is fourteen distinct Microsoft Rocketbox (MIT) characters with the
publisher's own outfits: business suits, blazers, a waistcoat, casual jackets,
knitwear, dresses and jeans, across a range of ages, skin tones, builds and
hair. Per-person scales map the publisher skeletons onto statures of about
1.63-1.85 m. Assignments follow cast order, so a crowd of eight bystanders
shows nine different people. Look-alike scenarios deliberately give the
leader and look-alike the same character and scale. Appearance assignments
are rendering/evaluation metadata, never a recognition input.

Each character has two clips baked from the publisher's neutral motion
capture: a 1.2 s walk cycle (about 1.0 m/s stride for men, 1.2 m/s for women
before stature scaling) and an 8 s idle loop whose last second blends back to
its start. While a scripted segment moves, gait phase follows travelled
distance, so stance feet stay planted at any path speed; while a person stands
(including before their first step and after their route ends) the idle loop
plays. Deterministic per-person phase offsets keep pairs, including
look-alikes, from walking or breathing in step. Isaac selects cached frames;
Gazebo's `CrowdMotion` plugin switches each actor between its `walk` and
`idle` animations using the same rule and offsets.

In the modern office all six scenario families walk at about 1.1 m/s (one
brisk crosser at 1.35 m/s) with rounded corners, between 1.6 m and about 13 m
in front of the stationary robot, so its camera sees people large and from the
front as well as from behind: the alone route jogs left, turns around and
returns towards the robot; the passer crosses about 3.5 m ahead, between robot
and leader; look-alikes walk side by side or swap sides; the leader joins a
standing group of three; and the crowd follows the explainer page's hall
choreography: the leader curves away down the hall, a brisk crosser cuts
between robot and leader, a pair crosses the view, walkers pass the robot on
both sides, one walker converges towards the leader and two people stand
chatting. Everyone stays at least 0.8 m apart at
crowd sizes 2, 4 and 8. The earlier
realistic worlds keep their recorded routes, which walk at 0.2-0.5 m/s.
All routes are scripted rather than reactive pedestrian navigation.

## Rendering differences

Both simulators consume the shared layout and cast. Isaac uses evaluated mesh
animation and textured PreviewSurface materials with an OmniGlass surface for
glazing; Gazebo uses native Collada actors with per-person gait speed. Fixture
UVs keep wall textures horizontal and tile floor materials in metres. Isaac
preserves source normal strength, metallic/roughness texture channels and
dielectric specular strength; Gazebo shading is simpler and does not
reproduce Isaac's fabric, glass or global illumination.

The earlier Innerscene and Maya characters stay in the repository so earlier
recorded realistic populations remain scoreable; all realistic worlds now use
the Rocketbox cast.

Gazebo interiors add ceiling point lights and emissive fixture surfaces. Its
fixture exporter namespaces Collada identifiers and gives copied architectural
maps unique basenames in each run's `fixtures/textures/` directory, preventing
Ogre's shared resource registry from reusing one texture for another surface; Rocketbox image identifiers are likewise the
unique texture names. The static converter removes loose wire edges (Collada
`<lines>`), which Gazebo would draw as unlit white strokes over tree foliage.
Trees use a 180,000-triangle visual mesh and cylindrical trunk collisions in
both backends; foliage has no collision. The Gazebo launch bridges
`/world/leader_follow_scene/clock` to ROS `/clock`.

## Run a scenario

Build and source the workspace as described in [simulation](simulation.md).
Then choose one world and scenario:

```bash
ros2 run ridgeback_leader_follow crowd_scene \
  --backend gz --scene-profile realistic --environment modern_office \
  --scenario crossing --gui

isaac_venv/bin/python -m ridgeback_leader_follow.crowd_runner \
  --backend isaac --scene-profile realistic --environment courtyard \
  --scenario crowd --crowd-size 8 --gui
```

| Scenario | CLI name | Behaviour |
|---|---|---|
| Alone | `leader_alone` | Straight sections and turns (a turn-around in the modern office) |
| Crossing | `crossing` | A passer-by crosses between observer and leader |
| Look-alike | `lookalike_parallel` | Matching-looking pedestrian walks beside the leader |
| Swap | `lookalike_crossing` | Leader and look-alike cross paths |
| Stop in a group | `leader_stop` | Leader stops among standing pedestrians |
| Crowd | `crowd` | Combined paths with `--crowd-size 2`, `4` or `8` bystanders |

`--environment` requires `--scene-profile realistic`; its default is `modern_office`.
The modern office uses the natural-pace routes described above. In the earlier
realistic worlds the other five routes reuse the original fixture paths and
only the alone route adds turns. Baseline and F3 routes are unchanged.
All normal recording, private-domain and owned-process cleanup options apply.

## Asset setup and conversion

Large mesh, animation, GLB and HDR files use Git LFS. After cloning, install
Git LFS and run `git lfs install` followed by `git lfs pull` from the repository.
Downloaded weights, virtual environments and raw run output remain under the
existing ignored directories.

Licences, sources and conversion hashes are recorded in
[`sim/assets/realistic/NOTICE.md`](../../src/ridgeback_leader_follow/sim/assets/realistic/NOTICE.md),
its adjacent `sources.json` (CC0 Poly Haven props, surfaces and HDRI; powered
by [Poly Haven](https://polyhaven.com)) and `rocketbox/sources.json` (MIT
Microsoft Rocketbox characters and clips, pinned to one publisher commit).
The earlier Innerscene/MakeHuman characters and Maya keep their attributions.

To rebuild from publisher files, use system Python with `requests`, then the
project's Blender 4.2.9 binary:

```bash
python3 src/ridgeback_leader_follow/sim/fetch_realistic_assets.py \
  src/ridgeback_leader_follow/sim/assets/realistic

artifacts/leader-following/tools/blender-4.2.9-linux-x64/blender -b \
  --python-exit-code 1 \
  --python src/ridgeback_leader_follow/sim/prepare_realistic_assets.py -- \
  src/ridgeback_leader_follow/sim/assets/realistic/props/sofa_03
```

Repeat conversion for each directory under `props/`. For each directory under
`humans/`, append `human` after the directory argument. Blender evaluates the
walk and exports the Gazebo skin; existing Maya assets are reused.

Rocketbox characters are fetched into an ignored source cache (about 1.2 GB of
FBX/TGA files), their textures converted into the asset tree, then baked one
character at a time; the roster lives in `rocketbox_cast.py`:

```bash
python3 src/ridgeback_leader_follow/sim/fetch_rocketbox_humans.py \
  artifacts/leader-following/rocketbox-source \
  src/ridgeback_leader_follow/sim/assets/realistic/rocketbox

artifacts/leader-following/tools/blender-4.2.9-linux-x64/blender -b \
  --python-exit-code 1 \
  --python src/ridgeback_leader_follow/sim/prepare_rocketbox_human.py -- \
  artifacts/leader-following/rocketbox-source business_female_01 \
  src/ridgeback_leader_follow/sim/assets/realistic/rocketbox/business_female_01
```

The bake bakes the FBX object transform into the rest pose, retargets each
clip by copying world bone orientations onto the avatar's own bone offsets
(pelvis motion scaled by leg length), removes the walk's net root travel,
faces +X and seats the lowest sole on z=0. The facing turn stays on an unscaled
armature node, because Gazebo keeps the root joint's rest orientation for
actors. Vertices without skin weights inherit their nearest weighted
neighbour's weights. The bake fails if the keyed skeleton departs from the
retarget by more than 2 mm or the idle loop does not close.
Static conversion verifies the exported world-space pose against the vertex
cache, and reduces source meshes above 250,000 polygons to a 180,000-triangle
budget. Material names are normalized during import to disambiguate duplicate
source names. Original wayfinding textures are regenerated with
`python3 src/ridgeback_leader_follow/sim/prepare_scene_signs.py`. Compare
regenerated files against the source/conversion receipts before replacing a
recorded population. Publisher content may change despite retaining its URL.

## Visual review and validation

Render a world without starting ROS nodes. Each sampled scenario time yields
an overview, a human-height view, a close cast view and the robot camera's
view (its mount pose and 640x480 field of view):

```bash
OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python \
  src/ridgeback_leader_follow/sim/preview_realistic.py \
  --output artifacts/leader-following/realistic-previews \
  --environments modern_office --scenario crowd --times 4 9
```

Tests cover static obstacle clearance for every route in every layout,
modern-office pace and person-to-person separation, cast diversity and
statures, look-alike assignment, Rocketbox receipt hashes and cache/skin
consistency, idle selection, Gazebo walk/idle animations and emissive
fixtures, Collada texture paths and preservation of commanded paths.
The scene/sensor qualifier also accepts `--scene-profile realistic`,
`--environments`, `--scenarios` and `--crowd-sizes` filters; it defaults to the
modern office. It does not run recognition. For example:

```bash
ros2 run ridgeback_leader_follow qualify_crowd --scene-profile realistic \
  --environments modern_office --scenarios crowd --crowd-sizes 8 \
  --backends gz --ros-domain 210
```

Offline mesh annotations reconstruct the recorded route and use each person's
recorded character, stature, gait speed and, when baked, idle loop and phase
offsets. Realistic recordings require the saved walk and idle caches with
matching SHA-256; evaluation refuses missing or mismatched geometry.
`benchmarking.annotations --stationary-base` uses recorded TF to remove
odometry drift for a surveyed stationary observer. This option is not valid
for a moving robot. These inputs are used only for scoring.

All six scenario families passed 20-second scene/sensor playback in the modern office in both simulators (twelve cases; crowd with eight bystanders). Earlier worlds have not been replayed with the new cast.

Rendering quality, playback validation and recognition performance are
separate claims. The images below are actual simulator renders. They are
detailed and naturally lit but not photographic: Rocketbox meshes have about
7,000-10,000 triangles, faces do not animate, hands keep one grip, and clothing
does not simulate cloth. Walk-to-idle switches are instantaneous; turns are
rounded paths rather than biomechanical turning steps. Pedestrians do not
react to the robot or each other. No recognition result exists for this cast.

![Isaac robot-camera view of the modern office: oncoming and departing pedestrians between glazed meeting rooms and window workstations](assets/environments/modern-office-robot-camera.jpg)

The Isaac robot camera (mount pose and 640x480 field of view) during the
eight-bystander crowd: faces of oncoming walkers, a glazed meeting room on the
left and window workstations on the right.

![Isaac overview of the modern office with walking and waiting pedestrians](assets/environments/modern-office-overview.jpg)

![The leader standing with a group of three in idle poses near the reception wall](assets/environments/modern-office-group.jpg)

The stop scenario: the leader has joined three standing people, all playing
the idle loop with different phases.

![Two identical look-alike pedestrians walking side by side, out of step](assets/environments/modern-office-lookalikes.jpg)

Look-alikes share one character deliberately; their gait phases differ.

![All fourteen Rocketbox characters standing in two rows](assets/environments/modern-office-cast.jpg)

![Gazebo robot camera in the modern office with the Rocketbox crowd](assets/environments/modern-office-gazebo.jpg)

Gazebo renders the same layout and cast with simpler shading and no global illumination.

The earlier worlds, rendered with the earlier four-character cast:

![Earlier furnished office lobby with textured floor and reception furniture](assets/environments/office.png)

![Warehouse aisle with racks, crates, barrels and pedestrians](assets/environments/warehouse.png)

![Courtyard with trees, benches and windowed buildings](assets/environments/courtyard.png)

## Archived evidence

- [Realistic worlds, asset conversion and startup validation](../../archive/engineering/2026-11-05-realistic-worlds.md) — tested population, tree/clock startup failures and verification limits.

- [Material, lighting and vegetation review](../../archive/engineering/2026-11-05-visual-materials.md) — visual population and limited runtime evidence.

- [Gazebo material isolation and full playback review](../../archive/engineering/2026-11-05-gazebo-material-isolation.md) — texture collision diagnosis and scenario coverage.

- [Modern office and Rocketbox pedestrians](../../archive/engineering/2026-11-05-modern-office-rocketbox.md) — selection, retargeting findings and twelve-case playback.
