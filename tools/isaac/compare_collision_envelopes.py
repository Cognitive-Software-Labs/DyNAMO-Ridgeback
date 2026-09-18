#!/usr/bin/env python3
"""Static description/USD/Nav2 comparison; never changes simulator or Nav2 inputs.

Run with Isaac Python and a sourced workspace. --urdf is a freshly expanded
shared description. Output includes figures, source snapshots and hashes.
This compares authored geometry, not live Gazebo contacts or hardware.
"""

from pathlib import Path
import argparse, hashlib, itertools, json, shutil, subprocess, xml.etree.ElementTree as ET
import numpy as np
import yaml, trimesh
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--urdf", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
OUT = args.output.resolve()
OUT.relative_to(ROOT / "artifacts")
OUT.mkdir(parents=True, exist_ok=False)
source_paths = [
    args.urdf.resolve(),
    ROOT / "clearpath/robot.yaml",
    ROOT / "src/ridgeback_autonomy/config/nav2_params.yaml",
]
shutil.copy2(args.urdf, OUT / "robot.urdf")
shutil.copy2(ROOT / "clearpath/robot.yaml", OUT / "robot.yaml")
shutil.copy2(
    ROOT / "src/ridgeback_autonomy/config/nav2_params.yaml", OUT / "nav2_params.yaml"
)
shutil.copy2(__file__, OUT / "comparison-source.py")
(OUT / "source.patch").write_bytes(
    subprocess.check_output(["git", "diff", "HEAD"], cwd=ROOT)
)
COL = {"gz": "#297bd1", "isaac": "#df8728", "old": "#bd365d", "new": "#008d77"}


def box(size):
    return np.array(list(itertools.product(*[[-v / 2, v / 2] for v in size])))


def cylinder(r, h, axis="Z"):
    a = np.arange(256) * 2 * np.pi / 256
    p = np.array(
        [[r * np.cos(t), r * np.sin(t), z] for z in [-h / 2, h / 2] for t in a]
    )
    return p if axis == "Z" else p[:, [2, 0, 1]] if axis == "X" else p[:, [0, 2, 1]]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    t[:3, 3] = xyz
    return t


def origin(e):
    if e is None:
        return np.eye(4)
    return transform(
        [float(x) for x in e.get("xyz", "0 0 0").split()],
        [float(x) for x in e.get("rpy", "0 0 0").split()],
    )


def apply(p, t):
    return p @ t[:3, :3].T + t[:3, 3]


def mesh(uri):
    from ament_index_python.packages import get_package_share_directory

    pkg, rel = uri.removeprefix("package://").split("/", 1)
    path = Path(get_package_share_directory(pkg)) / rel
    source_paths.append(path.resolve())
    return np.asarray(trimesh.load(path, force="mesh").vertices)


u = ET.parse(args.urdf).getroot()
joints = {j.find("child").get("link"): j for j in u.findall("joint")}


def pose(link):
    if link == "base_link":
        return np.eye(4)
    j = joints[link]
    return pose(j.find("parent").get("link")) @ origin(j.find("origin"))


gz = []
for l in u.findall("link"):
    for c in l.findall("collision"):
        g = c.find("geometry")
        m = g.find("mesh")
        b = g.find("box")
        cy = g.find("cylinder")
        if m is not None:
            p = mesh(m.get("filename")) * np.array(
                [float(v) for v in m.get("scale", "1 1 1").split()]
            )
        elif b is not None:
            p = box([float(v) for v in b.get("size").split()])
        elif cy is not None:
            p = cylinder(float(cy.get("radius")), float(cy.get("length")))
        else:
            raise ValueError(ET.tostring(g))
        gz.append(
            (l.get("name"), apply(p, pose(l.get("name")) @ origin(c.find("origin"))))
        )
s = Usd.Stage.Open(
    str(
        ROOT
        / "src/ridgeback_autonomy_isaac/sim/isaac/usd/robots/ridgeback_r100/ridgeback_r100.usda"
    )
)
base = s.GetPrimAtPath(s.GetDefaultPrim().GetPath().AppendPath("Geometry/base_link"))
inv = (
    UsdGeom.Xformable(base)
    .ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    .GetInverse()
)
source_paths.extend(Path(l.realPath) for l in s.GetUsedLayers() if l.realPath)
isaac = []
for p in Usd.PrimRange.Stage(s, Usd.TraverseInstanceProxies()):
    if (
        not p.HasAPI(UsdPhysics.CollisionAPI)
        or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() == False
    ):
        continue
    if p.IsA(UsdGeom.Mesh):
        v = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
    elif p.IsA(UsdGeom.Cube):
        v = box([UsdGeom.Cube(p).GetSizeAttr().Get()] * 3)
    elif p.IsA(UsdGeom.Cylinder):
        cy = UsdGeom.Cylinder(p)
        v = cylinder(
            cy.GetRadiusAttr().Get(),
            cy.GetHeightAttr().Get(),
            str(cy.GetAxisAttr().Get()),
        )
    else:
        raise ValueError(str(p.GetPath()))
    t = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()) * inv
    isaac.append(
        (
            str(p.GetPath()),
            np.array([list(t.Transform(Gf.Vec3d(*map(float, x)))) for x in v]),
        )
    )
params = yaml.safe_load(
    (ROOT / "src/ridgeback_autonomy/config/nav2_params.yaml").read_text()
)
old = np.array(
    yaml.safe_load(
        params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    )
)
assert np.array_equal(
    old,
    np.array(
        yaml.safe_load(
            params["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"]
        )
    ),
), "local/global footprints differ"
assert (
    np.sum(old[:, 0] * np.roll(old[:, 1], -1) - old[:, 1] * np.roll(old[:, 0], -1)) > 0
), "expected counterclockwise footprint"


def hull(p):
    return p[ConvexHull(p).vertices]


allp = np.concatenate([p for _, p in gz + isaac])
new = hull(allp[:, :2])
# Outward-oriented signed edge distances: positive means outside current polygon.
e = np.roll(old, -1, axis=0) - old
outside = np.max(
    (
        e[:, 1, None] * (allp[:, 0] - old[:, 0, None])
        - e[:, 0, None] * (allp[:, 1] - old[:, 1, None])
    )
    / np.linalg.norm(e, axis=1)[:, None]
)
plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "font.size": 11,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.titleweight": "bold",
        "figure.facecolor": "#f7f9fc",
        "axes.facecolor": "#fff",
    }
)


def outlines(ax, data, axes, color, alpha=0.06):
    for _, p in data:
        poly = hull(p[:, axes])
        ax.add_patch(
            Polygon(poly, facecolor=color, edgecolor=color, alpha=alpha, lw=0.8)
        )
        q = np.vstack([poly, poly[0]])
        ax.plot(q[:, 0], q[:, 1], color=color, lw=0.7, alpha=0.55)


def poly(ax, p, color, ls="-", lw=2):
    q = np.vstack([p, p[0]])
    ax.plot(q[:, 0], q[:, 1], color=color, ls=ls, lw=lw)


def style(ax, x, y):
    ax.set_aspect("equal")
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    ax.grid(alpha=0.15)


legend = [
    Line2D([], [], color=COL["gz"], label="Gazebo description collisions"),
    Line2D([], [], color=COL["isaac"], label="Isaac authored collisions"),
    Line2D([], [], color=COL["old"], ls="--", lw=2, label="Current Nav2 (unpadded)"),
    Line2D(
        [],
        [],
        color=COL["new"],
        lw=2,
        label="Union of current projections (diagnostic)",
    ),
]
fig, axs = plt.subplots(1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": [1.15, 1]})
for ax in axs:
    outlines(ax, gz, [0, 1], COL["gz"])
    outlines(ax, isaac, [0, 1], COL["isaac"])
    poly(ax, old, COL["old"], "--")
    poly(ax, new, COL["new"])
    style(ax, "Forward x (m)", "Left y (m)")
axs[0].set(xlim=(-0.54, 0.54), ylim=(-0.46, 0.46), title="Top view • complete robot")
axs[0].annotate(
    f"Rear extends {max(0, old[:,0].min()-allp[:,0].min())*1000:.1f} mm\nbeyond current footprint",
    xy=(-0.47066, 0),
    xytext=(-0.37, 0.13),
    fontsize=9,
    arrowprops={"arrowstyle": "->", "color": "#475467"},
)
axs[0].annotate("Front →", (0.32, -0.43))
axs[0].plot(0, 0, "+", color="#344054")
axs[0].text(0.01, 0.015, "base_link", fontsize=9)
axs[1].set(
    xlim=(-0.49, -0.30),
    ylim=(0.25, 0.415),
    title="Rear-left corner • enlarged difference",
)
fig.suptitle(
    "Current footprint vs. current simulator geometry", fontsize=20, x=0.055, ha="left"
)
fig.legend(
    handles=legend,
    loc="lower center",
    ncol=2,
    bbox_to_anchor=(0.5, 0.065),
    frameon=False,
)
fig.text(
    0.055,
    0.025,
    "STATIC AUDIT  •  Union is diagnostic, not a proposed footprint or shared robot model. No padding.",
    fontsize=10,
    color="#475467",
)
fig.tight_layout(rect=(0.025, 0.23, 0.98, 0.92))
fig.savefig(OUT / "topdown.png", dpi=180)
fig.savefig(OUT / "topdown.svg")
plt.close(fig)
fig, axs = plt.subplots(1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [1.15, 1]})
for ax in axs:
    outlines(ax, gz, [0, 2], COL["gz"])
    outlines(ax, isaac, [0, 2], COL["isaac"])
    style(ax, "Forward x (m)", "Height z relative to base_link (m)")
axs[0].set(xlim=(-0.54, 0.54), ylim=(-0.16, 1.18), title="Side view • collision shapes")
for pts, y, color, ls in [
    (old, -0.07, COL["old"], "--"),
    (new, -0.12, COL["new"], "-"),
]:
    axs[0].plot([pts[:, 0].min(), pts[:, 0].max()], [y, y], color=color, ls=ls, lw=3)
    for x in [pts[:, 0].min(), pts[:, 0].max()]:
        axs[0].plot([x, x], [y - 0.012, y + 0.012], color=color)
axs[0].annotate(
    "Mast and bracket\nIsaac only today",
    xy=(0.1955, 0.72),
    xytext=(-0.49, 0.82),
    arrowprops={"arrowstyle": "->", "color": "#475467"},
    fontsize=10,
)
axs[0].annotate(
    "D455",
    xy=(0.272, 1.035),
    xytext=(0.34, 1.13),
    arrowprops={"arrowstyle": "->"},
    fontsize=10,
)
axs[0].text(
    -0.5,
    0.38,
    "Footprint spans shown below the robot;\nvertical placement is illustrative.",
    fontsize=9,
    color="#475467",
)
axs[1].set(
    xlim=(-0.50, 0.50),
    ylim=(-0.085, 0.32),
    title="Body detail • rear / front extent changes",
)
for pts, color, ls in [(old, COL["old"], "--"), (new, COL["new"], "-")]:
    for x in [pts[:, 0].min(), pts[:, 0].max()]:
        axs[1].axvline(x, color=color, ls=ls, lw=1.5)
axs[1].text(
    -0.48,
    -0.075,
    "Vertical lines mark x extents only—not 3D walls.",
    fontsize=9,
    color="#475467",
)
fig.suptitle(
    "Side view • what contributes to the projected outline",
    fontsize=20,
    x=0.055,
    ha="left",
)
fig.legend(
    handles=legend,
    loc="lower center",
    ncol=2,
    bbox_to_anchor=(0.5, 0.055),
    frameon=False,
)
fig.text(
    0.055,
    0.018,
    "Authored geometry in base_link at zero joint positions. Static extraction; physics contact margins and hardware are not qualified.",
    fontsize=10,
    color="#475467",
)
fig.tight_layout(rect=(0.025, 0.16, 0.98, 0.92))
fig.savefig(OUT / "side.png", dpi=180)
fig.savefig(OUT / "side.svg")
plt.close(fig)
result = {
    "revision": subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
    ).strip(),
    "old": old.tolist(),
    "diagnostic_union": new.tolist(),
    "max_outside_old_m": float(outside),
    "bounds": {
        k: {
            "min": np.concatenate([p for _, p in data]).min(axis=0).tolist(),
            "max": np.concatenate([p for _, p in data]).max(axis=0).tolist(),
            "colliders": len(data),
        }
        for k, data in [("gazebo_description", gz), ("isaac_authored", isaac)]
    },
}
(OUT / "measurements.json").write_text(json.dumps(result, indent=2))

result["inputs_sha256"] = {
    str(p): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in sorted(set(source_paths))
}
result["parts"] = {
    label: [
        {
            "name": name,
            "minimum": p.min(axis=0).tolist(),
            "maximum": p.max(axis=0).tolist(),
            "max_outside_unpadded_m": float(
                max(
                    0,
                    np.max(
                        (
                            e[:, 1, None] * (p[:, 0] - old[:, 0, None])
                            - e[:, 0, None] * (p[:, 1] - old[:, 1, None])
                        )
                        / np.linalg.norm(e, axis=1)[:, None]
                    ),
                )
            ),
        }
        for name, p in data
    ]
    for label, data in [("gazebo_description", gz), ("isaac_authored", isaac)]
}
(OUT / "measurements.json").write_text(json.dumps(result, indent=2) + "\n")
