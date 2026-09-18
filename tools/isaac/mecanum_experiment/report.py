#!/usr/bin/env python3
"""Summarize boot evidence and render state-replay videos (not camera footage)."""

import argparse, json, math
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
from matplotlib.animation import FFMpegWriter
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from model import roller_geometry, WHEELS, SIGNS, ROLLER_COUNT


def load_rows(path):
    with open(path) as f:
        return [json.loads(line) for line in f]


def render(run, output, videos=False):
    from pxr import Usd, UsdGeom, Gf, UsdPhysics

    output.mkdir(parents=True, exist_ok=True)
    rows = load_rows(run / "samples.jsonl")
    meta = json.loads((run / "geometry.json").read_text())
    ctl = json.loads((run / "controller.json").read_text())
    names = ctl["all_dofs"]
    summary = json.loads((run / "summary.json").read_text())
    labels = list(dict.fromkeys(r["case"] for r in rows))
    colors = plt.cm.tab20(np.linspace(0, 1, len(labels)))
    plt.rcParams.update(
        {"font.size": 10, "axes.spines.top": False, "axes.spines.right": False}
    )
    fig, axs = plt.subplots(2, 2, figsize=(13, 9))
    offset = 0
    for label, color in zip(labels, colors):
        data = [r for r in rows if r["case"] == label]
        t = np.array([r["time"] for r in data])
        xy = np.array([r["position"][:2] for r in data])
        v = np.array([r["body_velocity"] for r in data])
        targets = np.array([r["targets"] for r in data])
        wheels = np.array([r["wheel_velocities"] for r in data])
        axs[0, 0].plot(
            xy[:, 0] - xy[0, 0], xy[:, 1] - xy[0, 1], label=label, color=color
        )
        axs[0, 1].plot(t + offset, v[:, 0], color=color)
        axs[0, 1].plot(t + offset, v[:, 1], color=color, ls="--")
        axs[1, 0].plot(
            t + offset, np.sqrt(np.mean((wheels - targets) ** 2, axis=1)), color=color
        )
        axs[1, 1].plot(t + offset, np.rad2deg([r["pitch"] for r in data]), color=color)
        offset += t[-1] + 1
    axs[0, 0].set(
        title="Trajectories from each case start",
        xlabel="x (m)",
        ylabel="y (m)",
        aspect="equal",
    )
    axs[0, 0].legend(fontsize=6, ncol=2)
    axs[0, 1].set(
        title="Actual body speed: x solid, y dashed",
        xlabel="Concatenated case time (s)",
        ylabel="m/s",
    )
    axs[1, 0].set(
        title="Wheel target tracking RMS error",
        xlabel="Concatenated case time (s)",
        ylabel="rad/s",
    )
    axs[1, 1].set(
        title="Chassis pitch", xlabel="Concatenated case time (s)", ylabel="degrees"
    )
    for ax in axs.flat:
        ax.grid(alpha=0.2)
    fig.suptitle(
        f"{summary['model']} · {summary['physics_hz']} Hz · {run.name}", fontsize=16
    )
    fig.tight_layout()
    fig.savefig(output / "motion-summary.png", dpi=170)
    plt.close(fig)
    if not videos:
        return
    physical = summary["model"] == "physical"
    stage = Usd.Stage.Open(
        str(run / ("candidate.usda" if physical else "effective-stage.usda"))
    )
    ch = stage.GetPrimAtPath(
        "/World/robot/chassis"
        if physical
        else "/World/robot/Geometry/base_link/chassis_link"
    )
    inv = UsdGeom.Xformable(ch).ComputeLocalToWorldTransform(0).GetInverse()
    fixed = []
    for p in Usd.PrimRange(ch):
        if (
            not p.HasAPI(UsdPhysics.CollisionAPI)
            or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() == False
        ):
            continue
        if p.IsA(UsdGeom.Mesh):
            pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        else:
            import itertools

            h = UsdGeom.Cube(p).GetSizeAttr().Get() / 2
            pts = list(itertools.product([-h, h], repeat=3))
        transform = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0) * inv
        fixed.append(
            np.array([list(transform.Transform(Gf.Vec3d(*map(float, x)))) for x in pts])
        )
    rp, rho, _, _ = roller_geometry(meta["radius"], meta["width"])
    selected = []
    for label in labels:
        data = [r for r in rows if r["case"] == label]
        selected.extend(data[::5])
    for title, dims, limits in [
        ("topdown", [0, 1], ((-1.5, 1.5), (-1.5, 1.5))),
        ("side", [0, 2], ((-1.5, 1.5), (-0.1, 1.2))),
    ]:
        fig, ax = plt.subplots(figsize=(10, 6))
        collection = PolyCollection(
            [], facecolors="#cfdfed", edgecolors="#34536d", linewidths=0.5
        )
        ax.add_collection(collection)
        ax.set(
            xlim=limits[0],
            ylim=limits[1],
            aspect="equal",
            xlabel="World x (m)",
            ylabel="World " + ("y" if title == "topdown" else "z") + " (m)",
        )
        ax.grid(alpha=0.2)
        wall = Rectangle(
            (1, -3 if title == "topdown" else 0),
            0.2,
            6 if title == "topdown" else 1.5,
            color="#b54d47",
            alpha=0.5,
        )
        ax.add_patch(wall)
        obstacle = Rectangle(
            (0.68, 0.2155 if title == "topdown" else 0),
            0.04,
            0.12 if title == "topdown" else 0.02,
            color="#c98a24",
        )
        ax.add_patch(obstacle)
        text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")
        fig.suptitle(
            f"{summary['model'].title()} drive • recorded-state replay", fontsize=14
        )
        fig.text(
            0.05,
            0.02,
            "Reconstructed from measured chassis and joint poses. Experimental roller geometry; not a hardware validation.",
            fontsize=9,
        )
        writer = FFMpegWriter(
            fps=6,
            metadata={"title": "Recorded-state geometry replay"},
            codec="libx264",
            extra_args=["-pix_fmt", "yuv420p", "-crf", "23"],
        )
        with writer.saving(fig, str(output / (title + ".mp4")), 100):
            for row in selected:
                q = row["quaternion"]
                r = Rotation.from_quat([*q[1:], q[0]]).as_matrix()
                pos = np.array(row["position"])
                parts = list(fixed)
                joint = dict(zip(names, row["joint_positions"]))
                for name, centre, sign in (
                    zip(WHEELS, meta["wheel_positions"], SIGNS) if physical else []
                ):
                    hub = Rotation.from_euler(
                        "y", joint[name + "_wheel_joint"]
                    ).as_matrix()
                    for k in range(ROLLER_COUNT):
                        theta = 2 * np.pi * k / ROLLER_COUNT
                        rad = np.array([np.cos(theta), 0, np.sin(theta)])
                        tan = np.array([-np.sin(theta), 0, np.cos(theta)])
                        axis = (tan + sign * np.array([0, 1, 0])) / np.sqrt(2)
                        orient = np.column_stack([axis, np.cross(rad, axis), rad])
                        spin = Rotation.from_euler(
                            "x", joint[name + f"_roller_joint_{k}"]
                        ).as_matrix()
                        parts.append(
                            (rp @ spin.T @ orient.T + rho * rad) @ hub.T + centre
                        )
                polys = []
                for pts in parts:
                    projected = (pts @ r.T + pos)[:, dims]
                    polys.append(projected[ConvexHull(projected).vertices])
                wall.set_visible(row["case"].startswith("wall_"))
                obstacle.set_visible(row["case"] == "low_obstacle")
                collection.set_verts(polys)
                text.set_text(
                    f"{row['case']}  |  {row['time']:.2f} s\nvx {row['body_velocity'][0]:+.3f}  vy {row['body_velocity'][1]:+.3f} m/s"
                )
                if row["case"] == "strafe_+0.2" and abs(row["time"] - 3) < 0.01:
                    fig.savefig(output / (title + ".png"), dpi=160)
                writer.grab_frame()
        plt.close(fig)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--videos", action="store_true")
    a = p.parse_args()
    render(a.run.resolve(), a.output.resolve(), a.videos)
