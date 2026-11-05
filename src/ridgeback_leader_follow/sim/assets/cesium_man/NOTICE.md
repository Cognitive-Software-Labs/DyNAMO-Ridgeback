# Cesium Man pedestrian asset

Cesium Man, © 2017 Cesium, is licensed under
[Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0/).
The Cesium logo is a trademark; no trademark rights or endorsement are implied.
The upstream license notice is preserved in `LICENSE.md`.

Source: [Khronos glTF Sample Assets](https://github.com/KhronosGroup/glTF-Sample-Assets/tree/8f492390ddc7a2ae221a1369ef560589cf8c70fb/Models/CesiumMan).
`CesiumMan.glb` is the unmodified upstream binary. `asset_revision.txt` pins
the source revision. `prepare_human.py` converts it with Blender 4.2.9 to
`walk.dae` and its textures for Gazebo, and the evaluated `walk.npz` mesh
animation for Isaac. The conversion scales the character to 1.75 m and seats
its feet at z=0. `conversion.json` records the tool version and asset hashes.

All derived meshes, animation samples and textures retain CC-BY-4.0. Identical
instances deliberately provide the two dressed-alike people in F1 scenarios.
These synthetic humans do not establish performance on real people.
