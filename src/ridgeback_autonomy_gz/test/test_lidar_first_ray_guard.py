"""Gazebo LiDAR first-ray guard: only the corrupted first sample is invalidated."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import time

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetRemap
import numpy as np
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from ridgeback_autonomy.mission.parking_clearance import scan_points


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]


def _load(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


guard = _load(PACKAGE_ROOT / 'nodes' / 'lidar_first_ray_guard.py')


def gazebo_scan(ranges):
    """A UST-10LX-shaped scan as Gazebo publishes it: 1081 rays over +-135 degrees."""
    msg = LaserScan()
    msg.header.frame_id = 'lidar2d_0_laser'
    msg.header.stamp.sec, msg.header.stamp.nanosec = 64, 780_000_000
    msg.angle_min, msg.angle_max = -2.35619449, 2.35619449
    msg.angle_increment = (msg.angle_max - msg.angle_min) / 1080
    msg.range_min, msg.range_max = 0.06, 10.0
    msg.ranges = [float(r) for r in ranges]
    msg.intensities = [1.0] * len(ranges)
    return msg


def test_only_the_corrupted_bins_are_invalidated_and_everything_else_is_kept():
    ranges = np.linspace(1.0, 5.0, 1081)
    msg = guard.invalidate_first_ray(gazebo_scan(ranges))
    assert math.isnan(msg.ranges[0]) and math.isnan(msg.ranges[360])
    kept = [i for i in range(1081) if i not in (0, 360)]
    assert [msg.ranges[i] for i in kept] == pytest.approx([ranges[i] for i in kept])
    assert (msg.header.frame_id, msg.header.stamp.sec, msg.header.stamp.nanosec) == (
        'lidar2d_0_laser', 64, 780_000_000)
    assert len(msg.ranges) == 1081 and len(msg.intensities) == 1081
    assert msg.angle_min == pytest.approx(-2.35619449)
    assert guard.invalidate_first_ray(LaserScan()).ranges.tolist() == []


def test_recorded_seam_phantom_is_removed_and_its_real_neighbours_kept():
    # Recorded 2026-10-30, front scan vetoing the pre-parking leg: bin 360 read
    # 1.389 m while its neighbours saw the far wall at about 6.2 m.
    ranges = np.full(1081, 6.2)
    ranges[360] = 1.389
    ranges[361] = 1.0                       # a real object one bin away stays
    msg = guard.invalidate_first_ray(gazebo_scan(ranges))
    assert math.isnan(msg.ranges[360])
    assert msg.ranges[359] == pytest.approx(6.2) and msg.ranges[361] == pytest.approx(1.0)
    bx, _ = scan_points(msg, (0.3922, 0.0, 0.0))
    assert len(bx) == 1079                  # bins 0 and 360 only


def test_short_scans_are_left_intact_beyond_their_length():
    msg = guard.invalidate_first_ray(gazebo_scan([2.0] * 10))
    assert math.isnan(msg.ranges[0]) and list(msg.ranges[1:]) == [2.0] * 9


def test_reproduced_phantom_start_cell_is_removed_and_real_edge_returns_kept():
    # Recorded 2026-10-30: base (0.852, 0.371, -0.706) in map, first ray 1.098 m,
    # its endpoint inside the robot's spawn cell (0.017, -0.001); the ray's
    # neighbours saw nothing there.
    base = (0.852, 0.371, -0.706)
    ranges = np.full(1081, np.inf)
    ranges[0] = 1.098            # range copied from near the -45 degree seam
    ranges[1] = 2.40             # a real return 0.25 degrees further round
    laser = (0.3922, 0.0, 0.0)

    def in_map(msg):
        bx, by = scan_points(msg, laser)
        c, s = math.cos(base[2]), math.sin(base[2])
        return base[0] + c * bx - s * by, base[1] + s * bx + c * by

    raw_x, raw_y = in_map(gazebo_scan(ranges))
    assert np.any(np.hypot(raw_x - 0.017, raw_y + 0.001) < 0.05)   # the artifact itself
    x, y = in_map(guard.invalidate_first_ray(gazebo_scan(ranges)))
    assert not np.any(np.hypot(x - 0.017, y + 0.001) < 0.25)
    assert len(x) == 1                                            # the real return survives


@pytest.fixture(scope='module')
def gazebo_launch():
    return _load(PACKAGE_ROOT / 'launch' / 'gazebo.launch.py')


def _context(setup_path):
    context = LaunchContext()
    context.launch_configurations.update({'setup_path': str(setup_path), 'use_sim_time': 'true'})
    return context


def _text(context, substitutions):
    return ''.join(s.perform(context) for s in substitutions)


def test_every_declared_lidar_is_relayed_through_the_guard(gazebo_launch):
    context = _context(REPO_ROOT / 'clearpath')
    *remaps, spawn, node = gazebo_launch._launch_robot(context)
    # Node-targeted rules: only the generated bridges publish to scan_gz.
    assert [(_text(context, r.src), _text(context, r.dst)) for r in remaps] == [
        ('lidar2d_0_gz_bridge:/r100_0001/sensors/lidar2d_0/scan',
         '/r100_0001/sensors/lidar2d_0/scan_gz'),
        ('lidar2d_1_gz_bridge:/r100_0001/sensors/lidar2d_1/scan',
         '/r100_0001/sensors/lidar2d_1/scan_gz'),
    ]
    assert all(isinstance(r, SetRemap) for r in remaps)
    assert isinstance(spawn, IncludeLaunchDescription) and isinstance(node, Node)
    assert node.node_package == 'ridgeback_autonomy_gz'
    assert node.node_executable == 'lidar_first_ray_guard'


def test_without_a_robot_declaration_the_spawn_is_unchanged(gazebo_launch, tmp_path):
    (spawn,) = gazebo_launch._launch_robot(_context(tmp_path))
    assert isinstance(spawn, IncludeLaunchDescription)


def test_relay_republishes_the_bridged_scan_with_only_the_first_ray_invalid():
    rclpy.init()
    try:
        node = guard.LidarFirstRayGuard()
        probe = rclpy.create_node('first_ray_guard_probe', namespace=node.get_namespace())
        received = []
        probe.create_subscription(LaserScan, 'lidar2d_0/scan', received.append,
                                  qos_profile_sensor_data)
        bridge = probe.create_publisher(LaserScan, 'lidar2d_0/scan_gz', 10)
        ranges = np.linspace(1.0, 5.0, 1081)
        deadline = time.monotonic() + 10.0
        while not received and time.monotonic() < deadline:
            bridge.publish(gazebo_scan(ranges))
            for spun in (node, probe, node):
                rclpy.spin_once(spun, timeout_sec=0.05)
        assert received, 'no relayed scan'
        assert math.isnan(received[0].ranges[0]) and math.isnan(received[0].ranges[360])
        kept = [i for i in range(1081) if i not in (0, 360)]
        assert [received[0].ranges[i] for i in kept] == pytest.approx([ranges[i] for i in kept])
        probe.destroy_node()
        node.destroy_node()
    finally:
        rclpy.shutdown()
