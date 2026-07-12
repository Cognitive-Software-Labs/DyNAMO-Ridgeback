"""Runtime ROS wiring for the robot package's GPU sensor prims (P4).

The sensor PRIMS live in the committed robot USD — the import script
(tools/isaac/import_ridgeback_urdf.py) bakes 2x UST-10LX OmniLidar prims
onto the lidar2d_{0,1}_laser frames and a D455-intrinsics camera at the
color optical pose, from the committed specs (ust10lx_2d.json and
config/camera_config.json). This module only binds render products and
ROS2 bridge helper publishers to those prims at runtime; plain-rclpy I/O
lives in ros_io.py.

Publish topics follow the frozen contract, default (RELIABLE/VOLATILE/
KEEP_LAST 10) QoS from the bridge nodes.

Lidar publishes point_cloud (Cartesian returns), NOT laser_scan: the
6.0.1 laser_scan writer hardcodes a 360-deg FOV for ROTARY sensors and
mislabels our 270-deg ROI arc (details at the helper node below).
ros_io.LidarScanAssembler bins the clouds into the contract LaserScan.
"""
from __future__ import annotations

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
                  namespace: str = "r100_0001") -> str:
    """Bind one render product + color/depth/points/camera_info publishers
    to the baked D455 camera prim (depth aligned to color, like the gz
    setup and the real driver's align mode)."""
    import omni.graph.core as og
    from pxr import UsdGeom

    link = _find_prim_by_name(stage, robot_root, "camera_0_link")
    cam = link.GetChild("d455_color")
    if not cam:
        raise RuntimeError(f"{link.GetPath()}: no d455_color child — "
                           + REGEN_HINT)
    # resolution back-derived from the baked intrinsics: fx = w*f/hAp
    c = UsdGeom.Camera(cam)
    focal = c.GetFocalLengthAttr().Get()
    hap = c.GetHorizontalApertureAttr().Get()
    vap = c.GetVerticalApertureAttr().Get()
    fx = 1280 * focal / hap            # sanity print only
    width, height = 1280, int(round(1280 * vap / hap))
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
    print(f"camera attached: {cam.GetPath()} {width}x{height} "
          f"fx={fx:.0f}", flush=True)
    return str(cam.GetPath())
