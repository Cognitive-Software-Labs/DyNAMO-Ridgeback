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
    DEFAULT_DEPTH_FIDELITY,
    resolve_camera_profile,
)

REGEN_HINT = ("robot USD predates baked sensor prims — regenerate with "
              "tools/isaac/import_ridgeback_urdf.py")

# D455 depth model. Geometry/range values come from the D400-series datasheet;
# the disparity noise/confidence defaults match Isaac Sim 6's certified D455
# asset and native Single View Depth Camera model. The certified asset still
# carries the schema's generic 55 mm baseline, so use the actual D455 95 mm
# baseline here instead of copying that known-wrong field.
D455_BASELINE_MM = 95.0
D455_MAX_DISPARITY_PX = 123.0
D455_CONFIDENCE_THRESHOLD = 0.0
D455_DISPARITY_NOISE_MEAN_PX = 0.25
D455_DISPARITY_NOISE_SIGMA_PX = 0.25
D455_DISPARITY_NOISE_DOWNSCALE_PX = 1.0


def _attach_d455_ros_writer(render_product_path: str, sink, fps: float):
    """Forward Isaac's host-only processed depth AOVs to the ROS sink.

    Isaac Sim 6's stock ROS camera helper is hard-wired to the ideal
    ``DistanceToImagePlaneSD`` render variable. The native stereo-depth AOVs
    intentionally use a host-memory pipeline and have no compatible pointer
    adapter, so a small Replicator writer is the narrowest honest bridge.
    """
    import numpy as np
    import omni.replicator.core as rep

    class D455RosWriter(rep.Writer):
        def __init__(self):
            self.version = "1.0.0"
            self.annotators = [
                "DepthSensorDistance",
                "DepthSensorPointCloudPosition",
                "IsaacReadSimulationTime",
            ]
            self._next_publish_time = None

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
                self._next_publish_time += 1.0 / fps

            depth = np.asarray(array("DepthSensorDistance"), dtype=np.float32)
            points = np.asarray(
                array("DepthSensorPointCloudPosition"), dtype=np.float32)
            sink(sim_time, depth, points[..., :3])

    writer = D455RosWriter()
    writer.attach(render_product_path)
    return writer


class _D455DepthHandle:
    """Keep the native sensor alive and tear it down before rclpy."""

    def __init__(self, sensor, writer):
        self.sensor = sensor
        self.writer = writer

    def detach(self):
        self.writer.detach()
        self.sensor._invalidate_sensor()


def _create_d455_depth(camera_path: str, profile, depth_sink):
    """Create/configure Isaac's supported native stereo-depth sensor."""
    import carb
    from isaacsim.sensors.experimental.rtx import (
        RtxCamera,
        SingleViewDepthCameraSensor,
    )

    # Disable automatic RTX settings schemas before the render product is
    # created. Once USD render settings are enabled by the depth annotator,
    # an already-auto-applied schema's DLSS default overrides the global AA
    # setting and silently feeds a half-resolution depth texture.
    settings = carb.settings.get_settings()
    for scope in ("renderSettings", "camera", "renderProduct"):
        settings.set(
            f"/exts/omni.usd.schema.render_settings/rtx/{scope}/"
            "apiSchemas/autoApply", None)
    camera = RtxCamera(
        camera_path, tick_rate=profile.fps,
        reset_xform_op_properties=False)
    sensor = SingleViewDepthCameraSensor(
        camera,
        resolution=(profile.height, profile.width),
        annotators=[],
    )
    sensor.set_enabled_post_processing(True)
    import os
    if os.environ.get("DYNAMO_D455_DEFAULTS") != "1":
        sensor.set_sensor_baseline(D455_BASELINE_MM)
        sensor.set_sensor_focal_length(profile.depth_focal_length_px)
        sensor.set_sensor_size(float(profile.width))
        sensor.set_sensor_maximum_disparity(D455_MAX_DISPARITY_PX)
        sensor.set_sensor_disparity_confidence(D455_CONFIDENCE_THRESHOLD)
        sensor.set_sensor_noise_parameters(
            noise_mean=D455_DISPARITY_NOISE_MEAN_PX,
            noise_sigma=D455_DISPARITY_NOISE_SIGMA_PX,
        )
        sensor.set_sensor_disparity_noise_downscale(
            D455_DISPARITY_NOISE_DOWNSCALE_PX)
        sensor.set_enabled_outlier_removal(True)
        sensor.set_sensor_distance_cutoffs(
            minimum_distance=profile.minimum_depth_m)
    render_product = sensor.render_product.GetPath().pathString
    writer = _attach_d455_ros_writer(
        render_product, depth_sink, profile.fps)
    return render_product, _D455DepthHandle(sensor, writer)


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
    if depth_fidelity == "d455":
        if depth_sink is None:
            raise ValueError("d455 depth fidelity requires a ROS depth sink")
        # DLSS/DLAA may render depth at half resolution; the native depth
        # sensor explicitly requires matching full-resolution color/depth
        # textures. FXAA (2) preserves the requested profile dimensions.
        import carb
        settings = carb.settings.get_settings()
        settings.set("/rtx/post/aa/op", 2)
        settings.set("/rtx-defaults/post/aa/op", 2)
        settings.set("/rtx-transient/post/aa/limitedOps", False)
    depth_handle = None
    if depth_fidelity == "d455":
        rp_path, depth_handle = _create_d455_depth(
            str(cam.GetPath()), profile, depth_sink)
    else:
        rp_path = _render_product(cam.GetPath(), [width, height])

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
          f"depth_fidelity={depth_fidelity} "
          f"{width}x{height}@{tick_rate:g} Hz "
          f"fx={fx:.0f}", flush=True)
    return str(cam.GetPath()), depth_handle
