# bg_metal_cabinet attribution

`meshes/MetalCabinet.obj, meshes/MetalCabinet.mtl, meshes/MetalCabinet.png` come from the Gazebo Fuel model
**OpenRobotics/MetalCabinet** (version 2):

- Source: https://app.gazebosim.org/OpenRobotics/fuel/models/MetalCabinet/2
- Author: Wan Yi Seow
- License: Creative Commons Attribution 4.0 International (CC BY 4.0),
  https://creativecommons.org/licenses/by/4.0/

Changes: the mesh is unmodified; `model.sdf` adds a box collision.
`bg_metal_cabinet.usda` is generated from `src/ridgeback_autonomy_gz/sim/models/bg_metal_cabinet/model.sdf` by
`tools/isaac/sdf2usd.py`: the same OBJ polygons, texture coordinates and normals,
with the textures copied unmodified into `textures/`.

