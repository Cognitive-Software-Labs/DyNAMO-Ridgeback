#!/usr/bin/env python3
"""Numerically qualify Isaac Sim 6.1 native stereo depth on a 3 m plane.

Run from the workspace root with the ROS workspace sourced:

    OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python3 \
      tools/isaac/depth_probe.py

An explicitly aimed camera looks at the front face of a thin cube 3 m away,
making the central native depth values directly comparable with the known
geometric distance. The probe verifies the geometric render independently,
then requires nonzero native output with the project's D455 noise model.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np


parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
parser.add_argument("--expected-version", default="6.1.0.0")
parser.add_argument("--plane-distance", type=float, default=3.0)
args, kit_args = parser.parse_known_args()
sys.argv = [sys.argv[0], *kit_args]
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

from importlib.metadata import version  # noqa: E402
from isaacsim import SimulationApp  # noqa: E402


app = SimulationApp({
    "headless": True,
    "fast_shutdown": False,
    # Native depth consumes a full-resolution depth render variable. DLSS
    # renders internally below the authored resolution and is not a valid
    # qualification path for the sensor post-process.
    "anti_aliasing": 0,
})

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.sensors.experimental.rtx import (  # noqa: E402
    RtxCamera,
    SingleViewDepthCameraSensor,
)
from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402


WIDTH = 640
HEIGHT = 480
FOCAL_LENGTH_PX = 384.0
BASELINE_MM = 95.0
MAX_DISPARITY_PX = 123.0
NOISE_MEAN_PX = 0.25
NOISE_SIGMA_PX = 0.25


def capture(sensor, annotator: str, *, max_updates: int = 90) -> np.ndarray:
    """Return the first full positive depth frame after renderer warmup."""
    last_stats = "no full-size frame"
    for update in range(1, max_updates + 1):
        app.update()
        data, _info = sensor.get_data(annotator)
        if data is None:
            continue
        if hasattr(data, "numpy"):
            data = data.numpy()
        array = np.asarray(data, dtype=np.float32)
        if array.size != HEIGHT * WIDTH:
            continue
        array = array.reshape(HEIGHT, WIDTH)
        finite = np.isfinite(array)
        positive = finite & (array > 0.0)
        if finite.any():
            last_stats = (
                f"finite={int(finite.sum())}/{array.size}, "
                f"positive={int(positive.sum())}, "
                f"range=[{float(array[finite].min()):.6g}, "
                f"{float(array[finite].max()):.6g}]")
        if update >= 15 and positive.any():
            return np.array(array, copy=True)
    raise RuntimeError(
        f"no positive {annotator} frame after {max_updates} updates "
        f"({last_stats})")


def centre(array: np.ndarray) -> np.ndarray:
    """Return the central 20% where range and image-plane depth agree <1%."""
    y_margin = int(array.shape[0] * 0.4)
    x_margin = int(array.shape[1] * 0.4)
    return array[y_margin:-y_margin, x_margin:-x_margin]


def main() -> None:
    installed = version("isaacsim")
    if installed != args.expected_version:
        raise RuntimeError(
            f"isaacsim version {installed}, expected {args.expected_version}")

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()
    world = stage.DefinePrim("/World", "Xform")
    stage.SetDefaultPrim(world)

    camera_prim = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera_prim.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))
    focal_mm = 24.0
    camera_prim.CreateFocalLengthAttr(focal_mm)
    camera_prim.CreateHorizontalApertureAttr(
        WIDTH * focal_mm / FOCAL_LENGTH_PX)
    camera_prim.CreateVerticalApertureAttr(
        HEIGHT * focal_mm / FOCAL_LENGTH_PX)

    # Use a regular USD primitive, matching NVIDIA's sensor tests, so the
    # qualification cannot fail due to hand-authored mesh topology/winding.
    target = UsdGeom.Cube.Define(stage, "/World/DepthTarget")
    target.CreateSizeAttr(2.0)
    target.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -args.plane_distance - 0.05))
    target.AddScaleOp().Set(Gf.Vec3f(10.0, 10.0, 0.05))

    ViewportManager.set_camera_view(
        "/World/Camera",
        eye=[0.0, 0.0, 0.0],
        target=[0.0, 0.0, -args.plane_distance],
    )
    for _ in range(3):
        app.update()

    camera = RtxCamera(
        "/World/Camera", tick_rate=30.0,
        reset_xform_op_properties=False)
    sensor = SingleViewDepthCameraSensor(
        camera,
        resolution=(HEIGHT, WIDTH),
        annotators=["distance_to_image_plane", "depth_sensor_distance"],
    )
    render_product_prim = stage.GetPrimAtPath(sensor.render_product.GetPath())
    if "OmniRtxPostDebugSettingsAPI_1" not in render_product_prim.GetAppliedSchemas():
        if not render_product_prim.ApplyAPI("OmniRtxPostDebugSettingsAPI_1"):
            raise RuntimeError("failed to apply render-product post-AA schema")
    render_product_prim.GetAttribute("omni:rtx:post:aa:op").Set("none")
    print(
        "RENDER PRODUCT AA: "
        f"schemas={render_product_prim.GetAppliedSchemas()}, "
        f"op={render_product_prim.GetAttribute('omni:rtx:post:aa:op').Get()}",
        flush=True,
    )
    sensor.set_sensor_baseline(BASELINE_MM)
    sensor.set_sensor_focal_length(FOCAL_LENGTH_PX)
    sensor.set_sensor_size(float(WIDTH))
    sensor.set_sensor_maximum_disparity(MAX_DISPARITY_PX)
    sensor.set_sensor_disparity_confidence(0.99)
    sensor.set_sensor_disparity_noise_downscale(1.0)
    sensor.set_sensor_distance_cutoffs(0.1, 100.0)
    sensor.set_enabled_outlier_removal(False)
    sensor.set_sensor_noise_parameters(
        noise_mean=NOISE_MEAN_PX,
        noise_sigma=NOISE_SIGMA_PX,
    )
    sensor.set_enabled_post_processing(True)

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    try:
        geometric = capture(sensor, "distance_to_image_plane")
        geometric_centre = centre(geometric)
        geometric_valid = np.isfinite(geometric_centre) & (geometric_centre > 0.0)
        print(
            "GEOMETRIC DEPTH OK: "
            f"valid={float(np.mean(geometric_valid)):.6f}, "
            f"median={float(np.median(geometric_centre[geometric_valid])):.6f} m",
            flush=True,
        )

        native = capture(sensor, "depth_sensor_distance")
        native_centre = centre(native)
        native_valid = np.isfinite(native_centre) & (native_centre > 0.0)
        native_valid_fraction = float(np.mean(native_valid))
        if not native_valid.any():
            raise RuntimeError("native centre contains no positive depth")
        native_median = float(np.median(native_centre[native_valid]))
        native_relative_error = abs(native_median - args.plane_distance) / args.plane_distance
        if native_valid_fraction < 0.99:
            raise RuntimeError(
                f"native centre validity {native_valid_fraction:.6f} < 0.99")
        if native_relative_error > 0.05:
            raise RuntimeError(
                f"native median {native_median:.6f} m differs from "
                f"{args.plane_distance:.6f} m by {native_relative_error:.3%}")

        overlap = native_valid & geometric_valid
        residual = native_centre[overlap] - geometric_centre[overlap]
        if residual.size < 1000:
            raise RuntimeError(
                f"only {residual.size} overlapping native/geometric centre pixels")
        residual_std = float(np.std(residual))
        residual_p99 = float(np.percentile(np.abs(residual), 99.0))
        if residual_std <= 1e-5:
            raise RuntimeError(
                f"D455 residual is constant (std={residual_std:.9f} m)")
        if residual_p99 >= 0.5:
            raise RuntimeError(
                f"D455 residual p99 {residual_p99:.6f} m is unbounded")

        print(json.dumps({
            "isaacsim_version": installed,
            "resolution": [WIDTH, HEIGHT],
            "plane_distance_m": args.plane_distance,
            "native_centre_valid_fraction": native_valid_fraction,
            "native_centre_median_m": native_median,
            "native_centre_relative_error": native_relative_error,
            "d455_overlap_pixels": int(residual.size),
            "d455_residual_mean_m": float(np.mean(residual)),
            "d455_residual_std_m": residual_std,
            "d455_residual_abs_p99_m": residual_p99,
        }, indent=2, sort_keys=True), flush=True)
        print("DEPTH PROBE OK: native 6.1 depth selected", flush=True)
    finally:
        timeline.stop()
        sensor._invalidate_sensor()


try:
    main()
except BaseException as exc:
    print(f"DEPTH PROBE FAILED: {type(exc).__name__}: {exc}",
          file=sys.stderr, flush=True)
    # Kit's fast shutdown and some extension teardown paths can swallow the
    # intended status or segfault. The OS still releases all GPU resources;
    # exiting here preserves a trustworthy qualification result for CI.
    os._exit(1)
app.close()
