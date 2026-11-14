# bg_trash_bin attribution

`meshes/TrashBin.obj, meshes/TrashBin.mtl, meshes/TrashBin_Diffuse.png` come from the Gazebo Fuel model
**OpenRobotics/TrashBin** (version 2):

- Source: https://app.gazebosim.org/OpenRobotics/fuel/models/TrashBin/2
- Author: Roselle Carmen
- License: Creative Commons Attribution 4.0 International (CC BY 4.0),
  https://creativecommons.org/licenses/by/4.0/

Changes: the mesh is unmodified; `model.sdf` adds a box collision.
`bg_trash_bin.usda` is generated from `src/ridgeback_autonomy_gz/sim/models/bg_trash_bin/model.sdf` by
`tools/isaac/sdf2usd.py`: the same OBJ polygons, texture coordinates and normals,
with the textures copied unmodified into `textures/`.

