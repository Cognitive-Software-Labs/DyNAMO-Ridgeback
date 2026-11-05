"""Matching F1 corridor and licensed, skinned pedestrians in both simulators."""
from pathlib import Path
from xml.etree import ElementTree as ET

WORLD_NAME = 'leader_follow_scene'
PHYSICS_DT = 0.001


def asset_directory():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('ridgeback_leader_follow')) / 'sim/assets/cesium_man'


def body_parts(color):
    skin = (0.65, 0.43, 0.30)
    return (
        ('torso', (0., 0., 1.15), (0.28, 0.44, 0.60), color),
        ('head', (0., 0., 1.60), (0.22, 0.22, 0.28), skin),
        ('left_leg', (0., 0.13, 0.425), (0.17, 0.17, 0.85), (0.12, 0.12, 0.15)),
        ('right_leg', (0., -0.13, 0.425), (0.17, 0.17, 0.85), (0.12, 0.12, 0.15)),
        ('left_arm', (0., 0.30, 1.12), (0.14, 0.14, 0.64), color),
        ('right_arm', (0., -0.30, 1.12), (0.14, 0.14, 0.64), color),
    )


def fixtures():
    return (
        ('floor', (6., 0., -0.05), (20., 6., 0.1), (0.7, 0.7, 0.7)),
        ('north_wall', (6., 2.8, 1.25), (20., 0.15, 2.5), (0.9, 0.9, 0.9)),
        ('south_wall', (6., -2.8, 1.25), (20., 0.15, 2.5), (0.9, 0.9, 0.9)),
    )


def _box(link, name, position, size, color):
    for tag in ('visual', 'collision'):
        item = ET.SubElement(link, tag, name=name)
        ET.SubElement(item, 'pose').text = ' '.join(map(str, (*position, 0, 0, 0)))
        ET.SubElement(ET.SubElement(item, 'geometry'), 'box')
        item.find('geometry/box').append(ET.Element('size'))
        item.find('geometry/box/size').text = ' '.join(map(str, size))
        if tag == 'visual':
            material = ET.SubElement(item, 'material')
            for field in ('ambient', 'diffuse'):
                ET.SubElement(material, field).text = ' '.join(map(str, (*color, 1)))


def gazebo_world(scenario, skins=None, gait_speed=None):
    """Baseline world; a profile may supply per-person skins and their gait speed."""
    from ament_index_python.packages import get_package_prefix
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name=WORLD_NAME)
    for short, system in (('physics', 'Physics'), ('user-commands', 'UserCommands'),
                          ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu')):
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{short}-system',
                      name=f'gz::sim::systems::{system}')
    sensors = ET.SubElement(world, 'plugin', filename='gz-sim-sensors-system',
                            name='gz::sim::systems::Sensors')
    ET.SubElement(sensors, 'render_engine').text = 'ogre2'
    physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = str(PHYSICS_DT)
    ET.SubElement(physics, 'real_time_factor').text = '1'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 8 0 0 0'
    ET.SubElement(light, 'diffuse').text = '1 1 1 1'
    ET.SubElement(light, 'direction').text = '-0.3 0.2 -1'
    for name, pos, size, color in fixtures():
        model = ET.SubElement(world, 'model', name=name)
        ET.SubElement(model, 'static').text = 'true'
        _box(ET.SubElement(model, 'link', name='body'), name, pos, size, color)
    library = Path(get_package_prefix('ridgeback_leader_follow')) / 'lib/ridgeback_leader_follow/libCrowdMotion.so'
    if not library.is_file():
        raise RuntimeError('Build ridgeback_leader_follow with the Gazebo dependency profile first')
    controller = ET.SubElement(world, 'plugin', filename=str(library),
                                name='leader_follow::CrowdMotion')
    ET.SubElement(controller, 'duration').text = str(scenario.duration)
    if gait_speed is not None:
        ET.SubElement(controller, 'gait_speed').text = repr(float(gait_speed))
    for person in scenario.people:
        actor = ET.SubElement(world, 'actor', name=person.name)
        ET.SubElement(actor, 'pose').text = '0 0 0 0 0 0'
        mesh = str(skins[person.name] if skins else asset_directory() / 'walk.dae')
        skin = ET.SubElement(actor, 'skin')
        ET.SubElement(skin, 'filename').text = mesh
        animation = ET.SubElement(actor, 'animation', name='walk')
        ET.SubElement(animation, 'filename').text = mesh
        ET.SubElement(animation, 'interpolate_x').text = 'false'
        config = ET.SubElement(controller, 'person')
        ET.SubElement(config, 'name').text = person.name
        for p in person.waypoints:
            ET.SubElement(config, 'waypoint').text = f'{p.time} {p.x} {p.y}'
    ET.indent(sdf)
    return ET.tostring(sdf, encoding='unicode')


def create_isaac_scene(stage, scenario):
    """Author matching geometry; must run after SimulationApp starts."""
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
    import numpy as np

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdLux.DomeLight.Define(stage, '/CrowdLight').CreateIntensityAttr(1000.)
    roots = {}
    data = np.load(asset_directory() / 'walk.npz')
    material = UsdShade.Material.Define(stage, '/HumanMaterial')
    shader = UsdShade.Shader.Define(stage, '/HumanMaterial/surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    texture = UsdShade.Shader.Define(stage, '/HumanMaterial/texture')
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(asset_directory() / 'Image_0.jpg')))
    uv_reader = UsdShade.Shader.Define(stage, '/HumanMaterial/uv')
    uv_reader.CreateIdAttr('UsdPrimvarReader_float2')
    uv_reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(uv_reader.ConnectableAPI(), 'result')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), 'rgb')
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')

    def box(path, position, size, color):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    for name, pos, size, color in fixtures():
        box(f'/Fixtures/{name}', pos, size, color)
    for person in scenario.people:
        root = UsdGeom.Xform.Define(stage, f'/People/{person.name}')
        translate = root.AddTranslateOp()
        rotate = root.AddRotateZOp()
        mesh = UsdGeom.Mesh.Define(stage, f'{root.GetPath()}/human')
        mesh.CreatePointsAttr(data['points'][0])
        mesh.CreateFaceVertexCountsAttr([3] * len(data['faces']))
        mesh.CreateFaceVertexIndicesAttr(data['faces'].reshape(-1))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                                               UsdGeom.Tokens.faceVarying).Set(data['uv'])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        roots[person.name] = (root, translate, rotate, mesh, data)
    return roots


def idle_offset(name, cycle):
    """Deterministic per-person phase so standing people do not breathe in unison."""
    import zlib
    return (zlib.crc32(name.encode()) % 1000) / 1000. * cycle


def motion_frame(data, person, scenario_time):
    """Clip and frame for a person: idle loop while standing (when baked), else gait.

    Gait phase follows travelled distance, so foot motion matches path speed.
    Shared by the Isaac animation and offline mesh annotations; the Gazebo
    plugin implements the same rule from the per-person values it is given.
    """
    idle = data['idle'] if 'idle' in data else None
    if idle is not None and not person.moving(scenario_time):
        cycle = float(idle['cycle_seconds'])
        phase = (scenario_time + idle_offset(person.name, cycle)) % cycle
        return idle, int(phase * float(idle['fps'])) % len(idle['points'])
    speed = float(data['gait_speed_mps']) if 'gait_speed_mps' in data else .5
    cycle = float(data['cycle_seconds'])
    # Profiles with an idle loop also desynchronize gaits, so pairs do not march in step.
    offset = idle_offset(person.name + ':gait', cycle) if idle is not None else 0.
    phase = (person.distance(scenario_time) / speed + offset) % cycle
    return data, int(phase * float(data['fps'])) % len(data['points'])


def animate_isaac_person(handle, person, scenario_time):
    import math
    from pxr import Gf
    root, translate, rotate, mesh, data = handle
    x, y, yaw = person.sample(scenario_time)
    translate.Set(Gf.Vec3d(x, y, 0.))
    rotate.Set(math.degrees(yaw))
    clip, frame = motion_frame(data, person, scenario_time)
    mesh.GetPointsAttr().Set(clip['points'][frame])
    if 'normals' in clip:
        mesh.GetNormalsAttr().Set(clip['normals'][frame])
