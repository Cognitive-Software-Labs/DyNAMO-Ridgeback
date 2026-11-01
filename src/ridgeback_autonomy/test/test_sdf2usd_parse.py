"""Parse-layer tests for tools/isaac/sdf2usd.py (no pxr required).

The emit/check layers run under isaac_venv (pxr); the converter's --check
mode covers them end-to-end. Here we pin the dependency-free parse layer:
the SDF subset extraction and the pose/AABB math the check mode trusts.
"""
import importlib.util
import math
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[3]
_SDF2USD = _REPO_ROOT / "tools" / "isaac" / "sdf2usd.py"
_GZ_SIM = _REPO_ROOT / "src" / "ridgeback_autonomy_gz" / "sim"
_WORLDS = _GZ_SIM / "worlds"
_MODELS = _GZ_SIM / "models"

spec = importlib.util.spec_from_file_location("sdf2usd", _SDF2USD)
sdf2usd = importlib.util.module_from_spec(spec)
sys.modules["sdf2usd"] = sdf2usd  # dataclasses resolve hints via sys.modules
spec.loader.exec_module(sdf2usd)


@pytest.fixture(scope="module")
def hospital():
    return sdf2usd.parse_world(_WORLDS / "initial_test_world.sdf")


def test_world_inventory(hospital):
    assert hospital.name == "initial_test_world"
    assert len(hospital.models) == 32
    assert len(hospital.includes) == 1
    assert len(hospital.lights) == 2

    geoms = [g for m in hospital.models for g in m.geoms]
    assert len(geoms) == 100
    assert sum(g.kind == "box" for g in geoms) == 98
    assert sum(g.kind == "sphere" for g in geoms) == 2
    assert all(m.static for m in hospital.models)


def test_named_wall_exact(hospital):
    wall = next(m for m in hospital.models if m.name == "wall_north")
    assert wall.pose == (5.0, 8.0, 1.4, 0.0, 0.0, 0.0)
    col = next(g for g in wall.geoms if g.is_collision)
    assert col.size == (22.0, 0.2, 2.8)
    vis = next(g for g in wall.geoms if not g.is_collision)
    assert vis.material.diffuse == (0.98, 0.98, 0.99, 1.0)


def test_per_geom_pose_and_yawed_model(hospital):
    table = next(m for m in hospital.models if m.name == "exam_table_n1")
    assert table.pose[5] == 0.35  # yawed model
    top = next(g for g in table.geoms if g.name == "top")
    assert top.pose == (0.0, 0.0, 0.25, 0.0, 0.0, 0.0)
    assert top.size == (1.35, 0.82, 0.10)


def test_transparent_material(hospital):
    details = next(m for m in hospital.models if m.name == "hospital_details")
    curtain = next(g for g in details.geoms if g.name == "privacy_curtain_s2")
    assert curtain.material.diffuse[3] == 0.55


def test_g1_include(hospital):
    inc = hospital.includes[0]
    assert inc.uri == "model://g1"
    assert inc.name == "default_g1_far_right_room"
    # z = floor top (0.05): G1 stands ON the slab, not sunk to its ankles
    assert inc.pose == (13.2, 4.65, 0.05, 0.0, 0.0, 3.14159)


def test_lights(hospital):
    sun = next(li for li in hospital.lights if li.name == "sun")
    assert sun.kind == "directional"
    assert sun.direction == (-0.25, 0.15, -1.0)
    sphere_light = next(li for li in hospital.lights if li.name == "sphere_light")
    assert sphere_light.kind == "point"
    assert sphere_light.attenuation_range == 12.0


def test_rpy_to_quat_yaw_only():
    w, x, y, z = sdf2usd.rpy_to_quat(0.0, 0.0, math.pi / 2)
    assert abs(w - math.cos(math.pi / 4)) < 1e-12
    assert abs(z - math.sin(math.pi / 4)) < 1e-12
    assert x == 0.0 and y == 0.0


def test_world_aabb_of_axis_aligned_wall(hospital):
    wall = next(m for m in hospital.models if m.name == "wall_north")
    lo, hi = sdf2usd.model_world_aabb(wall)
    assert lo == pytest.approx((5 - 11, 8 - 0.1, 1.4 - 1.4))
    assert hi == pytest.approx((5 + 11, 8 + 0.1, 1.4 + 1.4))


def test_world_aabb_respects_yaw():
    model = sdf2usd.Model(
        name="m", static=True, pose=(0, 0, 0, 0, 0, math.pi / 4),
        geoms=[sdf2usd.Geom(name="g", kind="box", size=(2.0, 2.0, 2.0),
                            pose=(0.0,) * 6, is_collision=False, material=None)],
    )
    lo, hi = sdf2usd.model_world_aabb(model)
    half_diag = math.sqrt(2)
    assert hi[0] == pytest.approx(half_diag)
    assert hi[1] == pytest.approx(half_diag)
    assert hi[2] == pytest.approx(1.0)


def test_calibration_world_parses():
    world = sdf2usd.parse_world(_WORLDS / "target_distance_calibration.sdf")
    assert len(world.models) == 5
    assert all(m.static for m in world.models)


def test_standalone_static_model_parses_with_material_and_ground_origin():
    model = sdf2usd.parse_model(_MODELS / "hospital_bed" / "model.sdf")

    assert model.name == "hospital_bed"
    assert model.static
    assert len(model.geoms) == 2
    visual = next(geom for geom in model.geoms if not geom.is_collision)
    collision = next(geom for geom in model.geoms if geom.is_collision)
    assert visual.kind == collision.kind == "box"
    assert visual.material.diffuse == (0.78, 0.88, 0.95, 1.0)
    assert collision.material is None
    lo, hi = sdf2usd.model_world_aabb(model)
    assert lo == pytest.approx((-0.9, -0.45, 0.0))
    assert hi == pytest.approx((0.9, 0.45, 0.5))


def test_cylinders_parse_and_bound_the_iv_pole_at_ground_origin():
    model = sdf2usd.parse_model(_MODELS / "iv_pole" / "model.sdf")

    assert len(model.geoms) == 4
    assert all(geom.kind == "cylinder" for geom in model.geoms)
    assert {geom.size for geom in model.geoms} == {(0.025, 1.75), (0.225, 0.05)}
    lo, hi = sdf2usd.model_world_aabb(model)
    assert lo == pytest.approx((-0.225, -0.225, 0.0))
    assert hi == pytest.approx((0.225, 0.225, 1.8))


def test_link_local_transform_is_composed_with_geometry_pose(tmp_path):
    sdf = tmp_path / "local_transform.sdf"
    sdf.write_text(
        """<sdf version=\"1.9\"><model name=\"local\"><static>true</static>
        <link name=\"link\"><pose>1 2 3 0 0 1.5707963267948966</pose>
        <visual name=\"visual\"><pose>1 0 0.5 0 0 0</pose>
        <geometry><box><size>2 4 1</size></box></geometry></visual>
        </link></model></sdf>""",
        encoding="utf-8",
    )

    model = sdf2usd.parse_model(sdf)
    assert model.geoms[0].pose == pytest.approx(
        (1.0, 3.0, 3.5, 0.0, 0.0, math.pi / 2))
    lo, hi = sdf2usd.model_world_aabb(model)
    assert lo == pytest.approx((-1.0, 2.0, 3.0))
    assert hi == pytest.approx((3.0, 4.0, 4.0))


def test_parse_sdf_wraps_standalone_model_as_one_model_stage():
    world = sdf2usd.parse_sdf(_MODELS / "cabinet" / "model.sdf")

    assert world.name == "cabinet"
    assert [model.name for model in world.models] == ["cabinet"]
    assert world.includes == []
    assert world.lights == []


def test_standalone_parser_refuses_a_world_document():
    with pytest.raises(ValueError, match="expected a standalone"):
        sdf2usd.parse_model(_WORLDS / "target_distance_calibration.sdf")


def test_unsupported_geometry_is_refused(tmp_path):
    sdf = tmp_path / "unsupported.sdf"
    sdf.write_text(
        """<sdf version=\"1.9\"><model name=\"unsupported\"><static>true</static>
        <link name=\"link\"><visual name=\"visual\"><geometry>
        <mesh><uri>model://mesh/file.dae</uri></mesh>
        </geometry></visual></link></model></sdf>""",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="uri 'model://mesh/file.dae' must be relative"):
        sdf2usd.parse_model(sdf)


_MTL = """# exporter comment
newmtl wood
Ka 1 1 1
Kd 0.8 0.7 0.6
Ks 0.5 0.5 0.5
Ns 96
illum 2
map_Kd wood.png
newmtl paint
Kd 0.2 0.3 0.4
d 1.0
"""

# A unit quad in two materials: one textured quad, one triangle and one
# pentagon, with relative (negative) indices on the last face.
_OBJ = """mtllib part.mtl
o part
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
v 0.5 1.5 0
v 2 0 0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
usemtl wood
s off
f 1/1/1 2/2/1 3/3/1 4/4/1
usemtl paint
f 2//1 6//1 3//1
f -6//-1 -5//-1 -4//-1 -2//-1 -3//-1
"""


def _mesh_model(tmp_path, obj=_OBJ, mtl=_MTL, geometry_extra="", pose="1 2 0 0 0 1.5707963267948966",
                collision=False):
    (tmp_path / "meshes").mkdir(exist_ok=True)
    (tmp_path / "meshes" / "part.obj").write_text(obj, encoding="utf-8")
    (tmp_path / "meshes" / "part.mtl").write_text(mtl, encoding="utf-8")
    (tmp_path / "meshes" / "wood.png").write_bytes(b"png")
    tag = "collision" if collision else "visual"
    sdf = tmp_path / "model.sdf"
    sdf.write_text(
        f"""<sdf version="1.9"><model name="m"><static>true</static><link name="link">
        <{tag} name="part"><pose>{pose}</pose><geometry><mesh>
        <uri>meshes/part.obj</uri><scale>2 3 4</scale>{geometry_extra}
        </mesh></geometry></{tag}></link></model></sdf>""",
        encoding="utf-8",
    )
    return sdf


def test_an_obj_mesh_keeps_its_polygons_uvs_normals_and_materials(tmp_path):
    model = sdf2usd.parse_model(_mesh_model(tmp_path))
    (geom,) = model.geoms
    mesh = geom.mesh

    assert geom.kind == "mesh" and geom.size == (2.0, 3.0, 4.0)
    assert geom.material is None
    assert [group.material for group in mesh.groups] == ["wood", "paint"]
    wood, paint = mesh.groups
    assert wood.face_counts == [4] and wood.points == [0, 1, 2, 3]
    assert wood.uvs == [0, 1, 2, 3] and wood.normals == [0, 0, 0, 0]
    assert paint.face_counts == [3, 5]
    assert paint.points == [1, 5, 2, 0, 1, 2, 4, 3]
    assert paint.uvs is None and paint.normals == [0] * 8
    assert mesh.materials["wood"].diffuse == (0.8, 0.7, 0.6)
    assert mesh.materials["wood"].specular == (0.5, 0.5, 0.5)
    assert mesh.materials["wood"].texture == tmp_path / "meshes" / "wood.png"
    assert mesh.materials["paint"].texture is None


def test_mesh_bounds_apply_scale_then_the_visual_pose(tmp_path):
    lo, hi = sdf2usd.model_world_aabb(sdf2usd.parse_model(_mesh_model(tmp_path)))

    # Scaled x 0..4, y 0..4.5; yawed +90 deg about (1, 2): x' = 1 - y, y' = 2 + x.
    assert lo == pytest.approx((-3.5, 2.0, 0.0))
    assert hi == pytest.approx((1.0, 6.0, 0.0))


def test_texture_names_must_not_collide(tmp_path):
    model = sdf2usd.parse_model(_mesh_model(tmp_path))
    other = tmp_path / "other"
    other.mkdir()
    (other / "wood.png").write_bytes(b"different")
    second = sdf2usd.parse_model(_mesh_model(other))
    world = sdf2usd.World("w", (0, 0, -9.81), (0.4,) * 4, [model, second], [], [])

    with pytest.raises(ValueError, match="share the name 'wood.png'"):
        sdf2usd.world_textures(world)


@pytest.mark.parametrize("obj, mtl, message", [
    (_OBJ.replace("usemtl wood\n", ""), _MTL, "face before any usemtl"),
    (_OBJ.replace("usemtl paint", "usemtl missing"), _MTL, "no MTL definition"),
    (_OBJ.replace("f 2//1", "f 2/1/1"), _MTL, "mixes faces with and without texture"),
    (_OBJ.replace("f 2//1 6//1 3//1", "f 2 6 3 9"), _MTL, "index 9 out of range"),
    (_OBJ + "l 1 2\n", _MTL, "unsupported OBJ statement 'l'"),
    (_OBJ, _MTL + "map_Bump normal.tga\n", "unsupported MTL statement 'map_Bump'"),
    (_OBJ, _MTL.replace("d 1.0", "d 0.5"), "transparent materials"),
    (_OBJ, _MTL.replace("map_Kd wood.png", "map_Kd -s 2 2 wood.png"), "options are unsupported"),
    (_OBJ, _MTL.replace("wood.png", "absent.png"), "absent.png not found"),
])
def test_obj_and_mtl_features_outside_the_subset_are_refused(tmp_path, obj, mtl, message):
    with pytest.raises(ValueError, match=message):
        sdf2usd.parse_model(_mesh_model(tmp_path, obj=obj, mtl=mtl))


def test_mesh_collisions_and_non_obj_meshes_are_refused(tmp_path):
    with pytest.raises(ValueError, match="mesh collision 'part' unsupported"):
        sdf2usd.parse_model(_mesh_model(tmp_path, collision=True))
    sdf = _mesh_model(tmp_path)
    sdf.write_text(sdf.read_text().replace("part.obj", "part.stl"))
    with pytest.raises(ValueError, match="only OBJ meshes"):
        sdf2usd.parse_model(sdf)
    with pytest.raises(ValueError, match=r"unsupported \['submesh'\]"):
        sdf2usd.parse_model(_mesh_model(
            tmp_path, geometry_extra="<submesh><name>a</name></submesh>"))


def test_packaged_background_props_parse_with_their_fuel_meshes():
    workstation = sdf2usd.parse_model(_MODELS / "bg_workstation" / "model.sdf")
    meshes = [g for g in workstation.geoms if g.kind == "mesh"]

    assert [g.name for g in meshes] == ["desk", "monitor", "keyboard"]
    assert all(not g.is_collision for g in meshes)
    assert meshes[0].mesh.materials["wood"].texture.name == "Desk_Diffuse.png"
    lo, hi = sdf2usd.model_world_aabb(workstation)
    # A 0.75 m desk top and the monitor above it, centred on the footprint.
    assert lo[2] == pytest.approx(0.0, abs=1e-6)
    assert hi[2] == pytest.approx(1.192, abs=1e-3)
    assert lo[0] == pytest.approx(-hi[0], abs=1e-3)
    assert lo[1] == pytest.approx(-hi[1], abs=1e-3)
