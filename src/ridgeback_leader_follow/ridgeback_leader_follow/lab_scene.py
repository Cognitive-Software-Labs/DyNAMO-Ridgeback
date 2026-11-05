"""Isaac lab demonstration profile, separate from the qualified F1 fixture."""
import json
from pathlib import Path


def asset_directory():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('ridgeback_leader_follow')) / 'sim/assets/maya'


def create_scene(stage, scenario):
    import numpy as np
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    dome = UsdLux.DomeLight.Define(stage, '/Lab/Lighting/Ambient')
    dome.CreateIntensityAttr(650.)
    dome.CreateColorAttr(Gf.Vec3f(.82, .89, 1.))

    def material(path, color, roughness=.75, metallic=0.):
        mat = UsdShade.Material.Define(stage, path)
        shader = UsdShade.Shader.Define(stage, path + '/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(metallic)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
        return mat, shader

    palette = {}
    for name, color in {
        'floor': (.22, .27, .31), 'tile': (.28, .33, .37), 'wall': (.67, .71, .72),
        'trim': (.11, .18, .22), 'door': (.26, .16, .09), 'metal': (.19, .23, .25),
        'wood': (.45, .29, .15), 'screen': (.015, .065, .09), 'blue': (.035, .23, .38),
        'pot': (.27, .14, .08), 'leaf': (.07, .24, .09), 'white': (.8, .83, .81),
    }.items():
        palette[name] = material('/Lab/Materials/' + name, color)[0]

    def box(name, position, size, finish, collision=True):
        cube = UsdGeom.Cube.Define(stage, '/Lab/Fixtures/' + name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(palette[finish])
        if collision:
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return cube

    box('Ceiling', (6, 0, 3.07), (20, 6, .12), 'white')
    box('Floor', (6, 0, -.06), (20, 6, .12), 'floor')
    for i in range(20):
        for j in range(6):
            if (i+j) % 2:
                box(f'Tile_{i}_{j}', (-3.5+i, -2.5+j, .001), (.995, .995, .002), 'tile', False)
    for side, y in [('North', 2.9), ('South', -2.9)]:
        box(side+'Wall', (6, y, 1.5), (20, .16, 3), 'wall')
        box(side+'Skirting', (6, y * .967, .09), (20, .05, .18), 'trim')
        box(side+'Rail', (6, y * .967, 1.05), (20, .035, .07), 'blue')
        for i, x in enumerate((1., 5., 9., 13.)):
            inner = y * .955
            box(f'{side}DoorFrame{i}', (x, inner, 1.13), (1.22, .09, 2.26), 'trim')
            box(f'{side}Door{i}', (x, inner-y*.012, 1.1), (1.04, .06, 2.14), 'door')
            box(f'{side}DoorWindow{i}', (x, inner-y*.025, 1.57), (.53, .02, .57), 'screen', False)
            box(f'{side}DoorHandle{i}', (x+.35, inner-y*.035, .95), (.18, .07, .035), 'metal')
            box(f'{side}RoomPlate{i}', (x+.8, inner, 1.65), (.25, .05, .17), 'blue', False)
    box('EndWall', (15.9, 0, 1.5), (.16, 6, 3), 'wall')
    box('EndPanel', (15.8, 0, 1.6), (.04, 2.4, .7), 'blue')
    # Furniture stays outside the benchmark paths (|y| <= 2).
    for i, x in enumerate((3., 7., 11.)):
        box(f'DeskTop{i}', (x, 2.48, .77), (1.45, .55, .07), 'wood')
        for k, dx in enumerate((-.6, .6)):
            box(f'DeskLeg{i}_{k}', (x+dx, 2.48, .36), (.055, .43, .72), 'metal')
        box(f'Monitor{i}', (x, 2.62, 1.12), (.61, .07, .36), 'screen')
        box(f'MonitorStand{i}', (x, 2.62, .88), (.045, .05, .16), 'metal')
        box(f'Keyboard{i}', (x, 2.32, .817), (.42, .16, .025), 'metal', False)
        box(f'BenchSeat{i}', (x, -2.48, .46), (1.4, .47, .09), 'blue')
        box(f'BenchBack{i}', (x, -2.69, .73), (1.4, .06, .49), 'blue')
        for k, dx in enumerate((-.53, .53)):
            box(f'BenchLeg{i}_{k}', (x+dx, -2.48, .22), (.06, .38, .44), 'metal')
    for i, x in enumerate((-.8, 14.5)):
        box(f'Planter{i}', (x, 2.45, .24), (.4, .4, .48), 'pot')
        for j, dx in enumerate((-.1, 0., .1)):
            leaf = UsdGeom.Sphere.Define(stage, f'/Lab/Fixtures/Leaves{i}_{j}')
            leaf.AddTranslateOp().Set(Gf.Vec3d(x+dx, 2.45, .7+j*.07))
            leaf.AddScaleOp().Set(Gf.Vec3f(.16, .18, .4))
            UsdShade.MaterialBindingAPI.Apply(leaf.GetPrim()).Bind(palette['leaf'])
    for i, x in enumerate((0., 4., 8., 12.)):
        light = UsdLux.RectLight.Define(stage, f'/Lab/Lighting/Panel{i}')
        light.AddTranslateOp().Set(Gf.Vec3d(x, 0, 2.95))
        light.CreateWidthAttr(1.4)
        light.CreateHeightAttr(.6)
        light.CreateIntensityAttr(22000.)
        light.CreateColorAttr(Gf.Vec3f(1., .94, .84))
        fill = UsdLux.SphereLight.Define(stage, f'/Lab/Lighting/Fill{i}')
        fill.AddTranslateOp().Set(Gf.Vec3d(x, 0, 2.65))
        fill.CreateRadiusAttr(.15)
        fill.CreateIntensityAttr(8000.)
        fill.CreateColorAttr(Gf.Vec3f(1., .94, .84))

    directory = asset_directory()
    with np.load(directory / 'walk.npz') as cache:
        data = {key: cache[key] for key in cache.files}
    specs = json.loads((directory / 'materials.json').read_text())
    roots = {}
    for person_index, person in enumerate(scenario.people):
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
        for i, spec in enumerate(specs):
            pbr = spec['pbrMetallicRoughness']
            color = pbr.get('baseColorFactor', [1., 1., 1., 1.])[:3]
            mat, shader = material(f'/Lab/PeopleMaterials/{person.name}/m{i}', color,
                pbr.get('roughnessFactor', .7), pbr.get('metallicFactor', 0.))
            if 'image' in spec:
                tex = UsdShade.Shader.Define(stage, f'{mat.GetPath()}/Texture')
                tex.CreateIdAttr('UsdUVTexture')
                tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(directory / spec['image'])))
                tex.CreateInput('wrapS', Sdf.ValueTypeNames.Token).Set('repeat')
                tex.CreateInput('wrapT', Sdf.ValueTypeNames.Token).Set('repeat')
                tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
                uv = UsdShade.Shader.Define(stage, f'{mat.GetPath()}/UV')
                uv.CreateIdAttr('UsdPrimvarReader_float2')
                uv.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
                tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(), 'result')
                # Outfit variants share the same body/rig; look-alikes retain identical clothes.
                if 'plum satin' in spec['name'] and 'lookalike' not in scenario.name and person_index % 2:
                    tex.CreateInput('scale', Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(.35, 1.45, 1.7, 1.))
                shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
                if spec.get('alphaMode') == 'MASK':
                    shader.CreateInput('opacity', Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(), 'a')
                    shader.CreateInput('opacityThreshold', Sdf.ValueTypeNames.Float).Set(spec.get('alphaCutoff', .5))
            subset = UsdGeom.Subset.CreateGeomSubset(mesh, f'm{i}', UsdGeom.Tokens.face,
                np.flatnonzero(data['materials'] == i).tolist(), 'materialBind')
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(mat)
        UsdGeom.Subset.SetFamilyType(mesh, 'materialBind', UsdGeom.Tokens.partition)
        roots[person.name] = (root, translate, rotate, mesh, data)
    return roots


def demo_scenario(cycles):
    """Continuous out-and-back paths with staggered crossings and standing pauses."""
    from .crowd_scenarios import CrowdScenario, PersonPath, Waypoint
    if not 1 <= cycles <= 120:
        raise ValueError('demo cycles must be between 1 and 120')
    people = []
    paths = (
        ('lf_leader', 'leader', ((0, 2.5, 0), (11, 8, 0), (13, 8, 0), (24, 2.5, 0), (30, 2.5, 0))),
        ('lf_passer', 'bystander', ((0, 5, -1.8), (8, 5, -1.8), (14, 5, 1.8),
                                   (20, 5, 1.8), (26, 5, -1.8), (30, 5, -1.8))),
    )
    for name, role, path in paths:
        points = []
        for cycle in range(cycles):
            for t, x, y in path:
                if points and cycle*30+t == points[-1].time:
                    continue
                points.append(Waypoint(cycle*30+t, x, y))
        people.append(PersonPath(name, role, (.25, .45, .65), tuple(points)))
    return CrowdScenario('lab_demo', cycles*30., tuple(people))
