"""GPU sensor rigs for the Isaac runner (P4).

Everything here publishes through OmniGraph bridge helper nodes (the GPU
path); plain-rclpy I/O lives in ros_io.py. Attached after the robot is
referenced and the ros2 bridge extension is enabled, before play.

Lidars: two Hokuyo UST-10LX on the lidar2d_{0,1}_laser frames. Isaac 6.0
replaced JSON lidar profiles with OmniLidar prims carrying
OmniSensorGenericLidarCoreAPI attributes — ust10lx_2d.json (committed
spec) is read here and authored onto the prims.

Publish topics follow the frozen contract: sensors/lidar2d_{i}/scan in
the robot namespace, frame lidar2d_{i}_laser, default (RELIABLE/VOLATILE/
KEEP_LAST 10) QoS from the bridge nodes.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

LIDAR_SPEC_PATH = Path(__file__).resolve().parent / "ust10lx_2d.json"
# single source of camera intrinsics (also consumed by perception)
CAMERA_CONFIG_PATH = (Path(__file__).resolve().parents[2]
                      / "config/camera_config.json")

# USD cameras look down -Z with +Y up in image; this quaternion (w,x,y,z)
# orients that to the ROS optical convention (+Z forward, +Y down) when
# the parent link is x-forward/z-up — i.e. the optical frame the static
# camera_optical_tf publishes, rotated pi about X.
_OPTICAL_QUAT_WXYZ = (0.5, 0.5, -0.5, -0.5)


def _find_prim_by_name(stage, root_path: str, name: str):
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim
    raise RuntimeError(f"prim named {name} not found under {root_path}")


def _author_spec(prim, attributes: dict) -> None:
    """Author OmniSensor attributes; values are already USD-typed enough
    (int/float/str/bool/lists) that Usd type coercion handles them."""
    from pxr import Vt

    for name, value in attributes.items():
        attr = prim.GetAttribute(name)
        if not attr:
            raise RuntimeError(f"{prim.GetPath()}: no attribute {name} "
                               f"(schema not applied?)")
        if isinstance(value, list):
            if all(isinstance(v, int) for v in value):
                value = Vt.UIntArray(value) if min(value) >= 0 \
                    else Vt.IntArray(value)
            else:
                value = Vt.FloatArray([float(v) for v in value])
        attr.Set(value)


def attach_lidars(stage, robot_root: str = "/ridgeback",
                  namespace: str = "r100_0001") -> list:
    """Create both UST-10LX rigs + bridge laser_scan publishers.

    Returns the created OmniLidar prim paths (for tests/diagnostics).
    """
    import omni.graph.core as og
    import omni.replicator.core as rep

    spec = json.loads(LIDAR_SPEC_PATH.read_text())["attributes"]

    created = []
    nodes = [("tick", "omni.graph.action.OnPlaybackTick")]
    connects = []
    values = []
    for i in (0, 1):
        laser = _find_prim_by_name(stage, robot_root, f"lidar2d_{i}_laser")
        lidar_path = laser.GetPath().AppendChild("rtx_lidar")
        lidar = stage.DefinePrim(lidar_path, "OmniLidar")
        if not lidar.ApplyAPI("OmniSensorGenericLidarCoreAPI"):
            raise RuntimeError(f"cannot apply lidar core API at {lidar_path}")
        _author_spec(lidar, spec)
        created.append(str(lidar_path))

        # render product drives the RTX sensor pipeline; resolution is a
        # placeholder for non-camera sensors
        rp = rep.create.render_product(str(lidar_path), [32, 32])
        rp_path = rp.path if hasattr(rp, "path") else str(rp)

        node = f"lidar{i}"
        nodes.append((node, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
        connects.append(("tick.outputs:tick", f"{node}.inputs:execIn"))
        values += [
            (f"{node}.inputs:renderProductPath", rp_path),
            (f"{node}.inputs:type", "laser_scan"),
            (f"{node}.inputs:topicName", f"sensors/lidar2d_{i}/scan"),
            (f"{node}.inputs:frameId", f"lidar2d_{i}_laser"),
            (f"{node}.inputs:nodeNamespace", f"/{namespace}"),
            (f"{node}.inputs:queueSize", 10),
        ]

    og.Controller.edit(
        {"graph_path": "/ros_lidar_graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.CONNECT: connects,
            og.Controller.Keys.SET_VALUES: values,
        },
    )
    print(f"lidars attached: {created}", flush=True)
    return created


def attach_camera(stage, robot_root: str = "/ridgeback",
                  namespace: str = "r100_0001") -> str:
    """D455-native camera: one USD camera at the color optical pose
    renders color+depth+points at true D455 720p intrinsics (depth
    aligned to color, matching the gz-era single-camera setup and the
    real driver's align mode). Intrinsics come from camera_config.json.
    """
    import omni.graph.core as og
    import omni.replicator.core as rep
    from pxr import Gf, UsdGeom

    cam_cfg = json.loads(CAMERA_CONFIG_PATH.read_text())["camera"]
    width, height = int(cam_cfg["width"]), int(cam_cfg["height"])
    fx, fy = float(cam_cfg["fx"]), float(cam_cfg["fy"])

    link = _find_prim_by_name(stage, robot_root, "camera_0_link")
    cam_path = link.GetPath().AppendChild("d455_color")
    cam = UsdGeom.Camera.Define(stage, cam_path)
    # optical pose: same translate the camera_optical_tf static publishes
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.015, 0.0))
    w, x, y, z = _OPTICAL_QUAT_WXYZ
    xf.AddOrientOp().Set(Gf.Quatf(w, x, y, z))
    # pinhole intrinsics: fx = width * focalLength / horizontalAperture
    focal = 24.0
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(width * focal / fx)
    cam.CreateVerticalApertureAttr(height * focal / fy)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))

    rp = rep.create.render_product(str(cam_path), [width, height])
    rp_path = rp.path if hasattr(rp, "path") else str(rp)

    frame = "camera_0_color_optical_frame"
    base = "sensors/camera_0"
    nodes = [("cam_tick", "omni.graph.action.OnPlaybackTick")]
    connects = []
    values = []
    helpers = [
        ("cam_rgb", "rgb", f"{base}/color/image"),
        ("cam_depth", "depth", f"{base}/depth/image"),
        ("cam_points", "depth_pcl", f"{base}/points"),
    ]
    for node, kind, topic in helpers:
        nodes.append((node, "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connects.append(("cam_tick.outputs:tick", f"{node}.inputs:execIn"))
        values += [
            (f"{node}.inputs:renderProductPath", rp_path),
            (f"{node}.inputs:type", kind),
            (f"{node}.inputs:topicName", topic),
            (f"{node}.inputs:frameId", frame),
            (f"{node}.inputs:nodeNamespace", f"/{namespace}"),
            (f"{node}.inputs:queueSize", 10),
        ]
    # both infos are the color intrinsics (aligned depth), like the gz
    # bridge published them
    for node, topic in [("cam_info_color", f"{base}/color/camera_info"),
                        ("cam_info_depth", f"{base}/depth/camera_info")]:
        nodes.append((node, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"))
        connects.append(("cam_tick.outputs:tick", f"{node}.inputs:execIn"))
        values += [
            (f"{node}.inputs:renderProductPath", rp_path),
            (f"{node}.inputs:topicName", topic),
            (f"{node}.inputs:frameId", frame),
            (f"{node}.inputs:nodeNamespace", f"/{namespace}"),
            (f"{node}.inputs:queueSize", 10),
        ]

    og.Controller.edit(
        {"graph_path": "/ros_camera_graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.CONNECT: connects,
            og.Controller.Keys.SET_VALUES: values,
        },
    )
    hfov = math.degrees(2 * math.atan(width / (2 * fx)))
    print(f"camera attached: {cam_path} {width}x{height} fx={fx} "
          f"(hfov {hfov:.1f} deg)", flush=True)
    return str(cam_path)
