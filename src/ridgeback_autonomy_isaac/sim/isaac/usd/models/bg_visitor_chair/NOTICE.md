# bg_visitor_chair attribution

`meshes/VisitorChair.obj, meshes/VisitorChair.mtl, meshes/VisitorChair.png` come from the Gazebo Fuel model
**OpenRobotics/VisitorChair** (version 3):

- Source: https://app.gazebosim.org/OpenRobotics/fuel/models/VisitorChair/3
- Author: Wan Yi Seow
- License: Creative Commons Attribution 4.0 International (CC BY 4.0),
  https://creativecommons.org/licenses/by/4.0/

Changes: the mesh is unmodified; `model.sdf` turns it to face +y and adds a box collision.
`bg_visitor_chair.usda` is generated from `src/ridgeback_autonomy_gz/sim/models/bg_visitor_chair/model.sdf` by
`tools/isaac/sdf2usd.py`: the same OBJ polygons, texture coordinates and normals,
with the textures copied unmodified into `textures/`.

