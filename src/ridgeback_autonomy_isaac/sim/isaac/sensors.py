"""Runtime ROS wiring for the robot package's GPU sensor prims (P4).

The sensor PRIMS live in the committed robot USD — the import script
(tools/isaac/import_ridgeback_urdf.py) bakes 2x UST-10LX OmniLidar prims
onto the lidar2d_{0,1}_laser frames and a D455-intrinsics camera at the
generated D455 colour frame, from the committed specs (ust10lx_2d.json and
the shared nominal D455 profile contract). This module only binds render
products and ROS2 bridge helper publishers to those prims at runtime;
plain-rclpy I/O lives in ros_io.py.

Publish topics follow the frozen contract, default (RELIABLE/VOLATILE/
KEEP_LAST 10) QoS from the bridge nodes.

Lidar publishes point_cloud (Cartesian returns), NOT laser_scan: the
6.0.1 laser_scan writer hardcodes a 360-deg FOV for ROTARY sensors and
mislabels our 270-deg ROI arc (details at the helper node below).
ros_io.LidarScanAssembler bins the clouds into the contract LaserScan.
"""
from __future__ import annotations

from ridgeback_autonomy.common.camera_profiles import (
    DEFAULT_CAMERA_PROFILE,
    resolve_camera_profile,
)

REGEN_HINT = ("robot USD predates baked sensor prims — regenerate with "
              "tools/isaac/import_ridgeback_urdf.py")


def _find_prim_by_name(stage, root_path: str, name: str):
    from pxr import Usd

    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim
    raise RuntimeError(f"prim named {name} not found under {root_path}: "
                       + REGEN_HINT)


def _render_product(prim_path: str, resolution):
    import omni.replicator.core as rep

    rp = rep.create.render_product(str(prim_path), resolution)
    return rp.path if hasattr(rp, "path") else str(rp)


def attach_lidars(stage, robot_root: str = "/ridgeback",
                  namespace: str = "r100_0001") -> list:
    """Bind render products + bridge laser_scan publishers to the two
    baked UST-10LX prims. Returns their prim paths."""
    import omni.graph.core as og

    created = []
    nodes = [("tick", "omni.graph.action.OnPlaybackTick")]
    connects = []
    values = []
    for i in (0, 1):
        laser = _find_prim_by_name(stage, robot_root, f"lidar2d_{i}_laser")
        # point_cloud, NOT laser_scan: the 6.0.1 laser_scan writer hardcodes
        # a 360-deg FOV for ROTARY sensors (_read_laser_scan_metadata in
        # OgnROS2RtxLidarHelper.py), so our 270-deg ROI arc gets stretched
        # across 360 deg of bin labels — scans over-rotate by 4/3 and SLAM
        # cooks on any rotation. The Cartesian returns are sensor-frame
        # correct (verified against the analytic world grid, ~2.6 cm), so
        # ros_io.py bins them into the contract LaserScan instead.
        #
        # TWO prims per lidar: the generic rotary model only fires a
        # 180-deg drum transit per tick from startAzimuthOffsetDeg (valid
        # subset thereof), regardless of tickRate/emitter tricks —
        # measured, not documented. rtx_lidar (offset 0) covers
        # [-135, 0]; rtx_lidar_l (offset -135) covers [0, +135].
        for prim_name, suffix in (("rtx_lidar", ""), ("rtx_lidar_l", "_l")):
            lidar = laser.GetChild(prim_name)
            if not lidar:
                raise RuntimeError(f"{laser.GetPath()}: no {prim_name} "
                                   f"child — " + REGEN_HINT)
            created.append(str(lidar.GetPath()))
            rp_path = _render_product(lidar.GetPath(), [32, 32])
            node = f"lidar{i}_pc{suffix}"
            nodes.append((node, "isaacsim.ros2.bridge.ROS2RtxLidarHelper"))
            connects.append(("tick.outputs:tick", f"{node}.inputs:execIn"))
            values += [
                (f"{node}.inputs:renderProductPath", rp_path),
                (f"{node}.inputs:type", "point_cloud"),
                (f"{node}.inputs:topicName",
                 f"sensors/lidar2d_{i}/points{suffix}"),
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
                  namespace: str = "r100_0001",
                  camera_profile: str = DEFAULT_CAMERA_PROFILE) -> str:
    """Bind one render product + color/depth/points/camera_info publishers
    to the baked D455 camera prim (depth aligned to color, like the gz
    setup and the real driver's align mode)."""
    import omni.graph.core as og
    from pxr import UsdGeom

    cam = _find_prim_by_name(stage, robot_root, "d455_color")
    profile = resolve_camera_profile(camera_profile)
    width, height = profile.width, profile.height
    tick_rate = profile.fps
    width_attr = cam.GetAttribute("dynamo:resolutionWidth")
    height_attr = cam.GetAttribute("dynamo:resolutionHeight")
    tick_attr = cam.GetAttribute("omni:sensor:tickRate")
    if not width_attr or not height_attr or not tick_attr:
        raise RuntimeError(f"{cam.GetPath()}: camera contract metadata missing — "
                           + REGEN_HINT)
    width_attr.Set(width)
    height_attr.Set(height)
    tick_attr.Set(tick_rate)
    if width <= 0 or height <= 0 or tick_rate <= 0:
        raise RuntimeError(f"{cam.GetPath()}: invalid camera contract "
                           f"{width}x{height}@{tick_rate} Hz")

    c = UsdGeom.Camera(cam)
    # Resolution and aperture form one contract. Reusing the baked aperture at
    # another aspect ratio would publish stretched, incorrect intrinsics.
    focal = 24.0
    c.GetFocalLengthAttr().Set(focal)
    c.GetHorizontalApertureAttr().Set(
        width * focal / profile.focal_length_px)
    c.GetVerticalApertureAttr().Set(
        height * focal / profile.focal_length_px)
    hap = c.GetHorizontalApertureAttr().Get()
    fx = width * focal / hap            # sanity print only
    rp_path = _render_product(cam.GetPath(), [width, height])

    frame = "camera_0_color_optical_frame"
    base = "sensors/camera_0"
    nodes = [("cam_tick", "omni.graph.action.OnPlaybackTick")]
    connects = []
    values = []
    for node, kind, topic in [
        ("cam_rgb", "rgb", f"{base}/color/image"),
        ("cam_depth", "depth", f"{base}/depth/image"),
        ("cam_points", "depth_pcl", f"{base}/points"),
    ]:
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
    print(f"camera attached: {cam.GetPath()} profile={profile.name} "
          f"{width}x{height}@{tick_rate:g} Hz "
          f"fx={fx:.0f}", flush=True)
    return str(cam.GetPath())
