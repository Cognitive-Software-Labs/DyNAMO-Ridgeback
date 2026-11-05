# Realistic scene materials, lighting and vegetation

Recorded dates: 2026-11-05
Tested revisions: `8da8eaf406c9256175fe1f3969d8b65219098a82`
Provenance: partial

## Tested populations

The visual review tested a dirty working tree based on the revision above.
The [source snapshot and receipts](assets/2026-11-05-visual-materials/source-and-receipts.tar.gz)
and [tracked patch](assets/2026-11-05-visual-materials/working-tree.patch)
preserve the implementation changes. The snapshot includes JSON conversion and
publisher receipts, but not a complete simulator installation, GPU driver,
dependency lock or duplicate binary assets. This is partial provenance.
Raw logs and renders remain in the local artifact directory
`artifacts/leader-following/realistic-office-evaluation/`.

The final renderer produced overview, human-height and cast views of all three
worlds. Review found improved suit colour visibility, ceiling illumination,
wood grain and planted beds. This remains a simulation appearance improvement,
not a claim of photographic realism. Four source humans are reused across the
cast. Gazebo's flat business-clothing tint differs from Isaac fabric shading.
Source grass/shrub alpha maps are preserved separately and exported through
Collada transparency as well as Isaac opacity textures.

[Runtime receipts](assets/2026-11-05-visual-materials/runtime-summary.json)
cover one simulated second each of an eight-bystander courtyard in Gazebo and
Isaac, and an eight-bystander office in Isaac. All manifests completed and all
processes exited zero. These checks establish loading, sensor readiness and
short playback, not complete scenario execution, recognition or pixel parity.
Isaac emitted existing robot mass/inertia warnings and a Matplotlib Axes3D
import warning; neither prevented completion. No physical robot was exercised.

## Earlier recognition diagnostic

Before the material pass, two 20-second office swap recordings were generated
from the clean base revision. Both scene/sensor checks passed. Evaluation then
used the dirty-tree recorded-character annotation changes, per-person scales,
saved mesh hashes and the stationary-observer TF anchor. The selected ROS
tracker/identity pipeline was replayed with OSNet similarity threshold 0.65,
margin 0.15, acquisition interval 1 second, tie tolerances 0.25/0.1 metres,
maximum age 3 seconds and observation timeout 0.25 seconds.

| Backend | Scored frames | Correct locked | Wrong locked | Correct-lock coverage |
|---|---:|---:|---:|---:|
| Gazebo | 294 | 232 | 0 | 78.91% |
| Isaac | 350 | 302 | 0 | 86.29% |

The [identity summary](assets/2026-11-05-visual-materials/ros-summary.json),
[scene qualification](assets/2026-11-05-visual-materials/qualification.json) and
[runtime source hashes](assets/2026-11-05-visual-materials/runtime-source-hashes.json)
preserve the diagnostic scope. No camera-exit frames occurred in these cases.
They do not qualify the later material/lighting/plant population, other worlds,
all six scenario families, moving-robot following or hardware timing.
