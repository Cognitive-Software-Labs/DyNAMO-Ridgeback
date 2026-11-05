"""F3 appearance profile: F1 corridor and paths, Maya pedestrians in distinct outfits.

Both simulators render the same Maya mesh, walk and recoloured textures, and
assign outfits through crowd_appearance. Only rendering changes: trajectories,
robot sensors and ground-truth recording are those of the baseline profile.
"""
from pathlib import Path

from ridgeback_leader_follow.crowd_appearance import assign_outfits

# Source glTF material name -> canonical texture name shared by both backends.
CANONICAL = {
    'Maya | anatomical body.eyebrow006': 'eyebrow',
    'Maya | anatomical body.eyelashes01': 'eyelashes',
    'Maya | plum satin full-length top': 'top',
    'Maya | charcoal tailored trousers': 'trousers',
    'Maya | fine hair roots': 'hair_roots',
    'Maya | anatomical body.low-poly': 'low_poly',
    'Maya | anatomical body.body': 'body',
    'Maya | anatomical body.ponytail01': 'ponytail',
    'Maya | anatomical body.teeth_base': 'teeth',
}
# Canonical name -> base-colour image extracted by prepare_lab_human.py.
SOURCE_IMAGES = {
    'eyebrow': 'image_0.png', 'eyelashes': 'image_1.png', 'top': 'image_3.png',
    'trousers': 'image_4.png', 'hair_roots': 'image_5.png', 'low_poly': 'image_6.png',
    'body': 'image_7.png', 'ponytail': 'image_8.png', 'teeth': 'image_9.png',
}
GARMENTS = ('top', 'trousers')
HAIR_PARTS = ('hair_roots', 'ponytail')
# None keeps the source texture unchanged.
HAIR_ALBEDO = {'brown': None, 'black': (.045, .040, .038), 'blonde': (.80, .64, .40)}
GEOMETRY = 'CC0 Innerscene Maya in deterministic F3 outfits; baseline corridor'


def asset_directory():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('ridgeback_leader_follow')) / 'sim/assets/maya_f3'


def maya_directory():
    return asset_directory().parent / 'maya'


def outfit_directory(root, outfit):
    return Path(root) / 'outfits' / outfit.name


def hair_directory(root, hair):
    return Path(root) / 'hair' / hair


def texture_files(outfit, maya=None, f3=None):
    """Canonical texture name -> file for one outfit."""
    maya, f3 = Path(maya or maya_directory()), Path(f3 or asset_directory())
    files = {name: maya / image for name, image in SOURCE_IMAGES.items()}
    for garment in GARMENTS:
        files[garment] = outfit_directory(f3, outfit) / f'{garment}.png'
    if HAIR_ALBEDO[outfit.hair] is not None:
        for part in HAIR_PARTS:
            files[part] = hair_directory(f3, outfit.hair) / f'{part}.png'
    return files


def manifest_entry(scenario):
    """Evidence only: which outfit each person wore. Never published to ROS."""
    return {name: outfit.name for name, outfit in assign_outfits(scenario).items()}


def gazebo_actor_skins(scenario, directory):
    """Write one Collada skin per outfit, with uniquely named textures beside it.

    Gazebo reuses a loaded mesh or texture with the same name, so linking the
    shared skin under several directories renders every actor in the first
    outfit. Each outfit therefore gets its own skin copy whose texture names
    carry the outfit name; the textures themselves are links, not copies.
    """
    directory, skins, shared = Path(directory), {}, None
    for name, outfit in assign_outfits(scenario).items():
        target = directory / outfit.name
        if not target.exists():
            if shared is None:
                shared = (asset_directory() / 'walk.dae').read_text()
            target.mkdir(parents=True)
            skin = shared
            for canonical, path in texture_files(outfit).items():
                unique = f'{outfit.name}_{canonical}.png'
                skin = skin.replace(f'<init_from>{canonical}.png</init_from>',
                                    f'<init_from>{unique}</init_from>')
                (target / unique).symlink_to(path)
            (target / f'{outfit.name}.dae').write_text(skin)
        skins[name] = target / f'{outfit.name}.dae'
    return skins


def gait_speed():
    """Metres travelled per animation second, as baked for the Isaac cache."""
    import numpy as np
    with np.load(maya_directory() / 'walk.npz') as data:
        return float(data['gait_speed_mps'])


def create_isaac_scene(stage, scenario):
    """Baseline corridor and lighting with Maya pedestrians in assigned outfits."""
    import json

    import numpy as np
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
    from ridgeback_leader_follow.crowd_geometry import fixtures

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdLux.DomeLight.Define(stage, '/CrowdLight').CreateIntensityAttr(1000.)
    for name, position, size, color in fixtures():
        cube = UsdGeom.Cube.Define(stage, f'/Fixtures/{name}')
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

    maya = maya_directory()
    with np.load(maya / 'walk.npz') as cache:
        data = {key: cache[key] for key in cache.files}
    specs = json.loads((maya / 'materials.json').read_text())
    outfits = assign_outfits(scenario)
    materials = {}

    def outfit_materials(outfit):
        if outfit.name in materials:
            return materials[outfit.name]
        textures, bound = texture_files(outfit), []
        for i, spec in enumerate(specs):
            path = f'/F3/Outfits/{outfit.name}/m{i}'
            pbr = spec['pbrMetallicRoughness']
            mat = UsdShade.Material.Define(stage, path)
            shader = UsdShade.Shader.Define(stage, path + '/Surface')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(pbr.get('roughnessFactor', .7))
            shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(pbr.get('metallicFactor', 0.))
            canonical = CANONICAL.get(spec['name'])
            if canonical is None:
                color = pbr.get('baseColorFactor', [1., 1., 1., 1.])[:3]
                shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
            else:
                tex = UsdShade.Shader.Define(stage, path + '/Texture')
                tex.CreateIdAttr('UsdUVTexture')
                tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(textures[canonical])))
                for wrap in ('wrapS', 'wrapT'):
                    tex.CreateInput(wrap, Sdf.ValueTypeNames.Token).Set('repeat')
                tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
                uv = UsdShade.Shader.Define(stage, path + '/UV')
                uv.CreateIdAttr('UsdPrimvarReader_float2')
                uv.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
                tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), 'result')
                shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(
                    tex.ConnectableAPI(), 'rgb')
                if spec.get('alphaMode') == 'MASK':
                    shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(), 'a')
                    shader.CreateInput('opacityThreshold', Sdf.ValueTypeNames.Float).Set(spec.get('alphaCutoff', .5))
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
            bound.append(mat)
        materials[outfit.name] = bound
        return bound

    roots = {}
    for person in scenario.people:
        root = UsdGeom.Xform.Define(stage, f'/People/{person.name}')
        translate, rotate = root.AddTranslateOp(), root.AddRotateZOp()
        mesh = UsdGeom.Mesh.Define(stage, f'{root.GetPath()}/human')
        mesh.CreatePointsAttr(data['points'][0])
        mesh.CreateNormalsAttr(data['normals'][0])
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        mesh.CreateFaceVertexCountsAttr([3] * len(data['faces']))
        mesh.CreateFaceVertexIndicesAttr(data['faces'].reshape(-1))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying).Set(data['uv'])
        for i, mat in enumerate(outfit_materials(outfits[person.name])):
            subset = UsdGeom.Subset.CreateGeomSubset(mesh, f'm{i}', UsdGeom.Tokens.face,
                np.flatnonzero(data['materials'] == i).tolist(), 'materialBind')
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat)
        UsdGeom.Subset.SetFamilyType(mesh, 'materialBind', UsdGeom.Tokens.partition)
        roots[person.name] = (root, translate, rotate, mesh, data)
    return roots
