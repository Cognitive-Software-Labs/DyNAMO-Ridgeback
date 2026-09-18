"""Scoped URDF collision importer for the physical-drive experiment.

Fixed links are merged with full inertia transforms. Only continuous wheel
joints are accepted. Production assets are never an output of this module.
"""

from __future__ import annotations
import itertools
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]
SIGNS = [-1, 1, 1, -1]
ROLLER_COUNT = 12
ROLLER_MASS = 0.08


def transform(origin):
    t = np.eye(4)
    if origin is not None:
        t[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        t[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        ).as_matrix()
    return t


class Description:
    def __init__(self, path):
        self.root = ET.parse(path).getroot()
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.joints = {
            j.find("child").get("link"): j for j in self.root.findall("joint")
        }
        expected = {w + "_wheel_joint" for w in WHEELS}
        actual = {
            j.get("name") for j in self.joints.values() if j.get("type") != "fixed"
        }
        if actual != expected:
            raise ValueError(f"unsupported movable joints: {actual ^ expected}")
        self.wheel_positions = [self.pose(w + "_wheel_link")[:3, 3] for w in WHEELS]
        self.radius = float(
            self.links["front_left_wheel_link"]
            .find("collision/geometry/cylinder")
            .get("radius")
        )
        self.width = float(
            self.links["front_left_wheel_link"]
            .find("collision/geometry/cylinder")
            .get("length")
        )

    def pose(self, name):
        if name == "base_link":
            return np.eye(4)
        j = self.joints[name]
        return self.pose(j.find("parent").get("link")) @ transform(j.find("origin"))

    def inertial(self):
        parts = []
        for name, link in self.links.items():
            if name.endswith("_wheel_link"):
                continue
            e = link.find("inertial")
            if e is None:
                continue
            t = self.pose(name) @ transform(e.find("origin"))
            m = float(e.find("mass").get("value"))
            a = e.find("inertia")
            i = np.array(
                [
                    [float(a.get("ixx")), float(a.get("ixy")), float(a.get("ixz"))],
                    [float(a.get("ixy")), float(a.get("iyy")), float(a.get("iyz"))],
                    [float(a.get("ixz")), float(a.get("iyz")), float(a.get("izz"))],
                ]
            )
            parts.append((m, t[:3, 3], t[:3, :3] @ i @ t[:3, :3].T))
        mass = sum(m for m, _, _ in parts)
        com = sum(m * p for m, p, _ in parts) / mass
        inertia = sum(
            i + m * (np.dot(p - com, p - com) * np.eye(3) - np.outer(p - com, p - com))
            for m, p, i in parts
        )
        return mass, com, inertia


def roller_geometry(radius, width):
    # Ellipsoidal rollers, 45-degree axles. Fit their union inside the URDF
    # cylindrical envelope; shape/count/mass are explicitly unmeasured.
    a, b = width * 0.46, radius * 0.16
    points = []
    for u in np.linspace(0, np.pi, 13):
        for v in np.arange(24) * 2 * np.pi / 24:
            points.append(
                [a * np.cos(u), b * np.sin(u) * np.cos(v), b * np.sin(u) * np.sin(v)]
            )
    points = np.array(points)
    # Find the largest radial centre fitting every sampled roller vertex.
    d = np.array([1, 1, 0]) / np.sqrt(2)
    q = np.array([1, -1, 0]) / np.sqrt(2)
    offsets = (
        points[:, 0, None] * d
        + points[:, 1, None] * q
        + points[:, 2, None] * np.array([0, 0, 1])
    )
    rho = float(np.min(np.sqrt(radius**2 - offsets[:, 0] ** 2) - offsets[:, 2]))
    assert np.max(np.abs(offsets[:, 1])) <= width / 2
    return points, rho, a, b


def build(urdf, output, *, yaw=0.0, wall=False, obstacle=False):
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
    import trimesh
    from scipy.spatial import ConvexHull
    from ament_index_python.packages import get_package_share_directory

    d = Description(urdf)
    output = Path(output).resolve()
    output.relative_to(Path(__file__).resolve().parents[3] / "artifacts")
    if output.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    s = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(s, "Z")
    UsdGeom.SetStageMetersPerUnit(s, 1.0)
    root = UsdGeom.Xform.Define(s, "/World")
    s.SetDefaultPrim(root.GetPrim())
    scene = UsdPhysics.Scene.Define(s, "/World/physics")
    scene.CreateGravityDirectionAttr((0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    phys = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    phys.CreateEnableGPUDynamicsAttr(False)
    phys.CreateSolverTypeAttr("TGS")
    material = UsdPhysics.MaterialAPI.Apply(
        s.DefinePrim("/World/contactMaterial", "Material")
    )
    material.CreateStaticFrictionAttr(0.8)
    material.CreateDynamicFrictionAttr(0.8)
    material.CreateRestitutionAttr(0.0)
    source_meshes = []

    def collider(prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        p.CreateContactOffsetAttr(0.001)
        p.CreateRestOffsetAttr(0.0)
        from pxr import UsdShade

        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            UsdShade.Material(material.GetPrim()), materialPurpose="physics"
        )

    def xf(prim, t):
        UsdGeom.Xformable(prim).AddTransformOp().Set(Gf.Matrix4d(t.T.tolist()))

    def box(path, size, t):
        cube = UsdGeom.Cube.Define(s, path)
        cube.CreateSizeAttr(1.0)
        xf(cube.GetPrim(), t @ np.diag([*size, 1.0]))
        collider(cube.GetPrim())
        return cube.GetPrim()

    box(
        "/World/floor",
        (12, 12, 0.1),
        np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.05], [0, 0, 0, 1.0]]),
    )
    if wall:
        t = np.eye(4)
        t[:3, 3] = [1.1, 0, 0.75]
        box("/World/wall", (0.2, 6, 1.5), t)
    if obstacle:
        t = np.eye(4)
        t[:3, 3] = [0.7, 0.2755, 0.01]
        box("/World/obstacle", (0.04, 0.12, 0.02), t)
    robot = UsdGeom.Xform.Define(s, "/World/robot")
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
    pa = PhysxSchema.PhysxArticulationAPI.Apply(robot.GetPrim())
    pa.CreateEnabledSelfCollisionsAttr(False)
    pa.CreateSolverPositionIterationCountAttr(16)
    pa.CreateSolverVelocityIterationCountAttr(4)
    rot = Rotation.from_euler("z", yaw).as_matrix()
    spawn = d.radius - d.wheel_positions[0][2] + 0.01

    def world(t):
        result = t.copy()
        result[:3, :3] = rot @ t[:3, :3]
        result[:3, 3] = rot @ t[:3, 3] + [0, 0, spawn]
        return result

    def body(path, t, mass, com=None, inertia=None):
        p = UsdGeom.Xform.Define(s, path).GetPrim()
        xf(p, world(t))
        UsdPhysics.RigidBodyAPI.Apply(p)
        mi = UsdPhysics.MassAPI.Apply(p)
        mi.CreateMassAttr(float(mass))
        if com is not None:
            mi.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            eig, axes = np.linalg.eigh(inertia)
            if np.linalg.det(axes) < 0:
                axes[:, 0] *= -1
            q = Rotation.from_matrix(axes).as_quat()
            mi.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, eig)))
            mi.CreatePrincipalAxesAttr(
                Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3])))
            )
        PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateSleepThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr(0.0)
        return p

    mass, com, inertia = d.inertial()
    chassis = body("/World/robot/chassis", np.eye(4), mass, com, inertia)
    points = []
    for name, link in d.links.items():
        if name.endswith("_wheel_link"):
            continue
        # Preserve source frame markers for sensor transform checks.
        marker = UsdGeom.Xform.Define(s, f"/World/robot/chassis/frames/{name}")
        xf(marker.GetPrim(), d.pose(name))
        for idx, c in enumerate(link.findall("collision")):
            t = d.pose(name) @ transform(c.find("origin"))
            g = c.find("geometry")
            path = f"/World/robot/chassis/{name}_{idx}"
            if g.find("box") is not None:
                size = np.fromstring(g.find("box").get("size"), sep=" ")
                box(path, size, t)
                local = np.array(
                    list(itertools.product(*[[-x / 2, x / 2] for x in size]))
                )
            elif g.find("mesh") is not None:
                e = g.find("mesh")
                pkg, rel = e.get("filename").removeprefix("package://").split("/", 1)
                asset = Path(get_package_share_directory(pkg)) / rel
                source_meshes.append(str(asset))
                mesh = trimesh.load(asset, force="mesh")
                local = np.asarray(mesh.vertices) * np.fromstring(
                    e.get("scale", "1 1 1"), sep=" "
                )
                p = UsdGeom.Mesh.Define(s, path)
                p.CreatePointsAttr(local.tolist())
                p.CreateFaceVertexCountsAttr([3] * len(mesh.faces))
                p.CreateFaceVertexIndicesAttr(mesh.faces.flatten().tolist())
                xf(p.GetPrim(), t)
                collider(p.GetPrim())
                UsdPhysics.MeshCollisionAPI.Apply(p.GetPrim()).CreateApproximationAttr(
                    "convexHull"
                )
            else:
                raise ValueError(f"unsupported fixed collision: {name}")
            points.extend((local @ t[:3, :3].T + t[:3, 3]).tolist())
    roller_points, rho, a, b = roller_geometry(d.radius, d.width)
    faces = ConvexHull(roller_points).simplices

    def joint(path, b0, b1, pos0, pos1, axis="Y", orientation=None):
        j = UsdPhysics.RevoluteJoint.Define(s, path)
        j.CreateBody0Rel().SetTargets([b0])
        j.CreateBody1Rel().SetTargets([b1])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, pos1)))
        if orientation is not None:
            q = Rotation.from_matrix(orientation).as_quat()
            j.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), Gf.Vec3f(*map(float, q[:3]))))
        return j

    for name, pos, sign in zip(WHEELS, d.wheel_positions, SIGNS):
        t = np.eye(4)
        t[:3, 3] = pos
        hubpath = f"/World/robot/{name}_hub"
        wheel = d.links[name + "_wheel_link"]
        wm = float(wheel.find("inertial/mass").get("value"))
        hubmass = wm - ROLLER_COUNT * ROLLER_MASS
        body(
            hubpath,
            t,
            hubmass,
            inertia=np.diag(
                [
                    hubmass * (3 * (d.radius * 0.55) ** 2 + d.width**2) / 12,
                    hubmass * (d.radius * 0.55) ** 2 / 2,
                    hubmass * (3 * (d.radius * 0.55) ** 2 + d.width**2) / 12,
                ]
            ),
        )
        j = joint(
            f"/World/robot/{name}_wheel_joint",
            "/World/robot/chassis",
            hubpath,
            pos,
            [0, 0, 0],
        )
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(8.0)
        drive.CreateMaxForceAttr(20.0)
        for k in range(ROLLER_COUNT):
            angle = 2 * np.pi * k / ROLLER_COUNT
            rad = np.array([np.cos(angle), 0, np.sin(angle)])
            tan = np.array([-np.sin(angle), 0, np.cos(angle)])
            axis = (tan + sign * np.array([0, 1, 0])) / np.sqrt(2)
            other = np.cross(rad, axis)
            r = np.column_stack([axis, other, rad])
            centre = rho * rad
            t = np.eye(4)
            t[:3, :3] = r
            t[:3, 3] = pos + centre
            path = f"/World/robot/{name}_roller_{k}"
            body(
                path,
                t,
                ROLLER_MASS,
                inertia=np.diag(
                    [
                        2 * ROLLER_MASS * b * b / 5,
                        ROLLER_MASS * (a * a + b * b) / 5,
                        ROLLER_MASS * (a * a + b * b) / 5,
                    ]
                ),
            )
            m = UsdGeom.Mesh.Define(s, path + "/shape")
            m.CreatePointsAttr(roller_points.tolist())
            m.CreateFaceVertexCountsAttr([3] * len(faces))
            m.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
            collider(m.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr(
                "convexHull"
            )
            joint(
                f"/World/robot/{name}_roller_joint_{k}",
                hubpath,
                path,
                centre,
                [0, 0, 0],
                axis="X",
                orientation=r,
            )
            points.extend((roller_points @ r.T + pos + centre).tolist())
    s.GetRootLayer().Save()
    return {
        "mass_chassis_kg": mass,
        "centre_of_mass": com.tolist(),
        "inertia": inertia.tolist(),
        "wheel_positions": np.array(d.wheel_positions).tolist(),
        "radius": d.radius,
        "width": d.width,
        "rollers_per_wheel": ROLLER_COUNT,
        "roller_mass": ROLLER_MASS,
        "roller_half_axes": [a, b, b],
        "roller_radial_centre": rho,
        "source_meshes": source_meshes,
        "collision_points": points,
        "spawn_z": spawn,
        "friction": 0.8,
        "contact_offset": 0.001,
        "rest_offset": 0.0,
        "wheel_drive_damping": 8.0,
        "wheel_max_torque": 20.0,
        "solver_iterations": [16, 4],
    }
