# Realistic simulation environments

The `realistic` scene profile adds furnished worlds and a varied pedestrian
cast to the crowd runner. It is under visual development. The existing F1/F2/F3
benchmark results apply to their recorded profiles; they do not qualify these
new worlds or characters. The observation robot remains stationary.

## Worlds and people

| `--environment` | Contents |
|---|---|
| `office` | Wood floor, glazed frontage, reception counter, desks, monitors, lounge furniture, plants and doors |
| `warehouse` | Concrete floor, storage racks, crates, barrels, aisle markings and loading door |
| `courtyard` | Paved walkways, planted beds, trees, benches, street lamps and surrounding building facades |

Four source characters are available: Sam and Maya in casual clothing, Harper
and Jordan in business clothing. Clothing colours and uniform stature scaling
provide additional variants. The cast reuses these four models; it is not a
population of independently modelled people. Look-alike scenarios deliberately
assign the leader and look-alike the same model, scale and clothing settings.
Appearance assignments are rendering/evaluation metadata, never a recognition
input. All routes are scripted rather than reactive pedestrian navigation.

Both simulators consume the shared layout and cast. Isaac uses evaluated mesh
animation and textured PreviewSurface materials; Gazebo uses native Collada
actors with per-person gait speed. Rendering and material support differ.

## Run a scenario

Build and source the workspace as described in [simulation](simulation.md).
Then choose one world and scenario:

```bash
ros2 run ridgeback_leader_follow crowd_scene \
  --backend gz --scene-profile realistic --environment office \
  --scenario crossing --gui

isaac_venv/bin/python -m ridgeback_leader_follow.crowd_runner \
  --backend isaac --scene-profile realistic --environment courtyard \
  --scenario crowd --crowd-size 8 --gui
```

| Scenario | CLI name | Behaviour |
|---|---|---|
| Alone | `leader_alone` | Straight sections and turns in the realistic profile |
| Crossing | `crossing` | A passer-by crosses between observer and leader |
| Look-alike | `lookalike_parallel` | Matching-looking pedestrian walks beside the leader |
| Swap | `lookalike_crossing` | Leader and look-alike cross paths |
| Stop in a group | `leader_stop` | Leader stops among standing pedestrians |
| Crowd | `crowd` | Combined paths with `--crowd-size 2`, `4` or `8` bystanders |

`--environment` requires `--scene-profile realistic`; its default is `office`.
The other five routes reuse the original fixture paths. Only the realistic
alone route adds turns; original baseline and F3 routes are unchanged.
All normal recording, private-domain and owned-process cleanup options apply.

## Asset setup and conversion

Large mesh, animation, GLB and HDR files use Git LFS. After cloning, install
Git LFS and run `git lfs install` followed by `git lfs pull` from the repository.
Downloaded weights, virtual environments and raw run output remain under the
existing ignored directories.

The CC0 sources and conversion hashes are recorded in
[`sim/assets/realistic/NOTICE.md`](../../src/ridgeback_leader_follow/sim/assets/realistic/NOTICE.md)
and its adjacent `sources.json`. Powered by [Poly Haven](https://polyhaven.com).
The three additional characters are from Innerscene/MakeHuman. Maya's existing
attribution remains in its asset directory.

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
walk and exports the Gazebo skin; existing Maya assets are reused. Compare
regenerated files against the source/conversion receipts before replacing a
recorded population. Publisher content may change despite retaining its URL.

## Visual review and validation

Render the three worlds without starting ROS nodes:

```bash
OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python \
  src/ridgeback_leader_follow/sim/preview_realistic.py \
  --output artifacts/leader-following/realistic-previews
```

The renderer closes after writing overview and human-height PNGs and USD
snapshots. Current checks cover static obstacle clearance across all six
scenario families in all three layouts, cast assignment, turn geometry,
Collada texture paths, actor scales and preservation of commanded paths.
Lighting and texture orientation still need visual refinement. Full runtime
sensor smoke checks and new recognition qualification remain pending.
