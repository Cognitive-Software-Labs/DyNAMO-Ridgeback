# bg_clinic_cart attribution

`meshes/Cart_BP.obj, meshes/Cart_BP.mtl, meshes/Cart_BP_Diffuse.png, meshes/Carts_Wheels_Diffuse.png` come from the Gazebo Fuel model
**OpenRobotics/BPCart** (version 2):

- Source: https://app.gazebosim.org/OpenRobotics/fuel/models/BPCart/2
- Author: Open Robotics
- License: Creative Commons Attribution 4.0 International (CC BY 4.0),
  https://creativecommons.org/licenses/by/4.0/

Changes: the mesh is unmodified. `Cart_BP.mtl` drops its two `map_Bump` lines, whose `.tga` normal maps are not part of the Fuel model; the unused normal and specular PNGs and the collision mesh are not vendored. `model.sdf` centres the mesh on its footprint and adds a box collision.
`bg_clinic_cart.usda` is generated from `src/ridgeback_autonomy_gz/sim/models/bg_clinic_cart/model.sdf` by
`tools/isaac/sdf2usd.py`: the same OBJ polygons, texture coordinates and normals,
with the textures copied unmodified into `textures/`.

