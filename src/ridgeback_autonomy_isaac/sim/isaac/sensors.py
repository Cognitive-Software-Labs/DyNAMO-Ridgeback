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

from ridgeback_common.camera_profiles import (
    DEFAULT_CAMERA_PROFILE,
    DEFAULT_DEPTH_FIDELITY,
    resolve_camera_profile,
)

REGEN_HINT = ("robot USD predates baked sensor prims — regenerate with "
              "tools/isaac/import_ridgeback_urdf.py")

# D455 depth model. Geometry/range values come from the D400-series datasheet.
# Isaac Sim 6.1's native SingleViewDepthCameraSensor returns all-zero output on
# this Blackwell host, including with NVIDIA's authored D455 asset, so the
# production path applies the same disparity-domain artifacts to geometric
# depth from the physical left imager and explicitly aligns it to color.
D455_BASELINE_MM = 95.0
D455_MAX_DISPARITY_PX = 123.0
D455_DISPARITY_NOISE_MEAN_PX = 0.25
D455_DISPARITY_NOISE_SIGMA_PX = 0.25
D455_DISPARITY_SUBPIXEL_STEPS = 32.0
D455_DEPTH_TO_COLOR_X_M = -0.059


def _align_d455_depth(geometric_depth, profile, disparity_noise_px):
    """Apply D455 disparity artifacts and align left-imager depth to color.

    The input and output use optical coordinates (X right, Y down, Z forward).
    Invalid output pixels are zero and point-cloud samples are NaN. The nominal
    simulator profile is rectified pinhole; real-device distortion remains a
    hardware-calibration concern rather than an invented simulation constant.
    """
    import numpy as np

    depth = np.asarray(geometric_depth, dtype=np.float32).squeeze()
    if depth.shape != (profile.height, profile.width):
        raise ValueError(
            f"geometric D455 depth shape {depth.shape}, expected "
            f"{(profile.height, profile.width)}")
    noise = np.asarray(disparity_noise_px, dtype=np.float32)
    if noise.shape != depth.shape:
        raise ValueError(
            f"D455 disparity noise shape {noise.shape}, expected {depth.shape}")

    baseline_m = D455_BASELINE_MM / 1000.0
    focal_depth = profile.depth_focal_length_px
    valid = np.isfinite(depth) & (depth >= profile.minimum_depth_m)
    disparity = np.zeros_like(depth)
    disparity[valid] = focal_depth * baseline_m / depth[valid]
    disparity += noise
    disparity = (
        np.rint(disparity * D455_DISPARITY_SUBPIXEL_STEPS)
        / D455_DISPARITY_SUBPIXEL_STEPS
    )
    valid &= (disparity > 0.0) & (disparity <= D455_MAX_DISPARITY_PX)

    measured = np.zeros_like(depth)
    measured[valid] = focal_depth * baseline_m / disparity[valid]
    valid &= np.isfinite(measured) & (measured >= profile.minimum_depth_m)

    rows, cols = np.indices(depth.shape, dtype=np.float32)
    x_depth = (cols - profile.cx) * measured / focal_depth
    y_depth = (rows - profile.cy) * measured / focal_depth
    # The color imager is 59 mm right of the left/depth imager. Expressing a
    # point from the depth optical frame in the color optical frame therefore
    # subtracts 59 mm from optical X.
    x_color = x_depth + D455_DEPTH_TO_COLOR_X_M
    y_color = y_depth
    z_color = measured
    u_color = np.rint(profile.focal_length_px * x_color / np.maximum(z_color, 1e-12)
                       + profile.cx).astype(np.int64)
    v_color = np.rint(profile.focal_length_px * y_color / np.maximum(z_color, 1e-12)
                       + profile.cy).astype(np.int64)
    valid &= (
        (u_color >= 0) & (u_color < profile.width)
        & (v_color >= 0) & (v_color < profile.height)
    )

    source = np.flatnonzero(valid)
    target = (v_color.ravel()[source] * profile.width
              + u_color.ravel()[source])
    z = z_color.ravel()[source]
    # Sort by destination pixel, then increasing Z; the first sample for each
    # pixel is the nearest visible surface.
    order = np.lexsort((z, target))
    target = target[order]
    source = source[order]
    _, first = np.unique(target, return_index=True)
    target = target[first]
    source = source[first]

    aligned_depth = np.zeros(depth.size, dtype=np.float32)
    aligned_points = np.full((depth.size, 3), np.nan, dtype=np.float32)
    aligned_depth[target] = z_color.ravel()[source]
    aligned_points[target, 0] = x_color.ravel()[source]
    aligned_points[target, 1] = y_color.ravel()[source]
    aligned_points[target, 2] = z_color.ravel()[source]
    return (
        aligned_depth.reshape(depth.shape),
        aligned_points.reshape((*depth.shape, 3)),
    )


def _attach_d455_ros_writer(render_product_path: str, sink, profile):
    """Turn geometric left-imager depth into aligned D455-like ROS output."""
    import numpy as np
    import omni.replicator.core as rep

    class D455RosWriter(rep.Writer):
        def __init__(self):
            self.version = "1.0.0"
            self.annotators = [
                "distance_to_image_plane",
                "IsaacReadSimulationTime",
            ]
            self._next_publish_time = None
            self._rng = np.random.default_rng(0)

        def write_metadata(self):
            # Live ROS transport has no dataset metadata artifact.
            self._is_metadata_written = True

        def write(self, data):
            def payload(name):
                key = next(key for key in data if key.startswith(name))
                return data[key]

            def array(name):
                value = payload(name)
                if isinstance(value, dict) and "data" in value:
                    value = value["data"]
                return np.asarray(value)

            sim_time = float(payload(
                "IsaacReadSimulationTime")["simulationTime"])
            if (self._next_publish_time is not None
                    and sim_time + 1e-9 < self._next_publish_time):
                return
            if (self._next_publish_time is None
                    or sim_time + 1e-9 < self._next_publish_time - 1.0):
                self._next_publish_time = sim_time
            while self._next_publish_time <= sim_time + 1e-9:
                self._next_publish_time += 1.0 / profile.fps

            geometric = np.asarray(
                array("distance_to_image_plane"), dtype=np.float32).squeeze()
            noise = self._rng.normal(
                D455_DISPARITY_NOISE_MEAN_PX,
                D455_DISPARITY_NOISE_SIGMA_PX,
                geometric.shape,
            ).astype(np.float32)
            depth, points = _align_d455_depth(geometric, profile, noise)
            sink(sim_time, depth, points)

    writer = D455RosWriter()
    writer.attach(render_product_path)
    return writer


class _D455DepthHandle:
    """Keep the Replicator writer alive and tear it down before rclpy."""

    def __init__(self, writer):
        self.writer = writer

    def detach(self):
        self.writer.detach()


def _create_d455_depth(stage, depth_frame_path: str, profile, depth_sink):
    """Render from the physical left imager and attach the D455 model."""
    from pxr import Gf, UsdGeom

    camera_path = str(depth_frame_path) + "/d455_depth"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, -0.5, -0.5))
    focal_mm = 24.0
    camera.CreateFocalLengthAttr(focal_mm)
    camera.CreateHorizontalApertureAttr(
        profile.width * focal_mm / profile.depth_focal_length_px)
    camera.CreateVerticalApertureAttr(
        profile.height * focal_mm / profile.depth_focal_length_px)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))
    render_product = _render_product(camera.GetPath(), [profile.width, profile.height])
    writer = _attach_d455_ros_writer(render_product, depth_sink, profile)
    return render_product, _D455DepthHandle(writer)


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
        # One complete rotary cloud per physical sensor. The ROS assembler
        # clips to +/-135 degrees; RTX angular clipping is not used.
        for prim_name, suffix in (("rtx_lidar", ""),):
            lidar = laser.GetChild(prim_name)
            if not lidar:
                raise RuntimeError(f"{laser.GetPath()}: no {prim_name} "
                                   f"child — " + REGEN_HINT)
            created.append(str(lidar.GetPath()))
            print(f"lidar configuration {lidar.GetPath()}: " + repr({
                attr.GetName(): attr.Get() for attr in lidar.GetAttributes()
                if attr.GetName().startswith("omni:sensor:")
                and attr.HasAuthoredValueOpinion()}), flush=True)
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
                  camera_profile: str = DEFAULT_CAMERA_PROFILE,
                  depth_fidelity: str = DEFAULT_DEPTH_FIDELITY,
                  depth_sink=None) -> tuple[str, object | None]:
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
    color_rp_path = _render_product(cam.GetPath(), [width, height])
    depth_handle = None
    if depth_fidelity == "d455":
        if depth_sink is None:
            raise ValueError("d455 depth fidelity requires a ROS depth sink")
        depth_frame = _find_prim_by_name(
            stage, robot_root, "camera_0_depth_frame")
        _, depth_handle = _create_d455_depth(
            stage, str(depth_frame.GetPath()), profile, depth_sink)

    frame = "camera_0_color_optical_frame"
    base = "sensors/camera_0"
    nodes = [("cam_tick", "omni.graph.action.OnPlaybackTick")]
    connects = []
    values = []
    camera_products = [("cam_rgb", "rgb", f"{base}/color/image")]
    if depth_fidelity == "ideal":
        camera_products += [
            ("cam_depth", "depth", f"{base}/depth/image"),
            ("cam_points", "depth_pcl", f"{base}/points"),
        ]
    elif depth_fidelity == "d455":
        pass
    else:
        raise ValueError(
            f"unknown depth fidelity {depth_fidelity!r}; expected ideal or d455")
    for node, kind, topic in camera_products:
        nodes.append((node, "isaacsim.ros2.bridge.ROS2CameraHelper"))
        connects.append(("cam_tick.outputs:tick", f"{node}.inputs:execIn"))
        values += [
            (f"{node}.inputs:renderProductPath", color_rp_path),
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
            (f"{node}.inputs:renderProductPath", color_rp_path),
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
          f"depth_fidelity={depth_fidelity} "
          f"{width}x{height}@{tick_rate:g} Hz "
          f"fx={fx:.0f}", flush=True)
    return str(cam.GetPath()), depth_handle
