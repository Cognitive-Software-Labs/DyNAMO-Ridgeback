#!/usr/bin/env python3

from __future__ import annotations

from collections import OrderedDict
import json
import math
import os
import subprocess
import time
from typing import Any

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from ridgeback_autonomy.benchmarking.alignment import (
    ensure_measurement_event,
    find_exact_preview_match,
    find_nearest_preview_match,
    stamp_to_nanoseconds,
    update_measurement_event,
)
from ridgeback_autonomy.benchmarking.estimators import (
    ESTIMATOR_LABELS,
    benchmark_output_name,
    parse_estimators,
    selected_camera_estimators,
    selected_mask_estimators,
    uses_camera_estimators,
    uses_lidar_estimators,
    uses_mask_estimators,
)
from ridgeback_autonomy.benchmarking.reduction import (
    choose_representative_event,
    compute_trial_medians,
    usable_aligned_events,
)
from ridgeback_autonomy.benchmarking.rendering import BenchmarkCollageRenderer
from ridgeback_autonomy.benchmarking.summary import (
    build_summary_rows,
    write_summary_csv,
    write_trial_csv,
)
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.geometry import (
    planar_distance_from_vehicle_origin,
    yaw_from_quaternion,
)
from ridgeback_autonomy.perception.core.image_utils import (
    convert_color_image_message,
    convert_depth_to_meters_message,
)
from ridgeback_autonomy.perception.core.isolation_2d import ISOLATION_2D_DEFAULT
from ridgeback_autonomy.perception.core.isolation_3d import ISOLATION_3D_DEFAULT


FORWARD_DISTANCES_M = [1.5, 2.5, 3.5, 4.5, 5.5]
LATERAL_OFFSETS_M = [-0.75, 0.0, 0.75]
G1_SPAWN_HEIGHT_M = 0.0
G1_FACING_ROBOT_YAW_RAD = math.pi
COMMAND_TIMEOUT_SEC = 10.0
COMMAND_RETRY_SLEEP_SEC = 0.5
STREAM_WAIT_TIMEOUT_SEC = 300.0
POSE_WAIT_TIMEOUT_SEC = 120.0
DELETE_TIMEOUT_SEC = 15.0
IMAGE_MATCH_TOLERANCE_NS = 250_000_000
CAMERA_MEASUREMENT_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENT_TOPIC = 'measurements/g1/lidar'
MASK_MEASUREMENT_TOPIC = 'measurements/g1/mask'
DEPTH_SOURCE_DEFAULT = 'stereo'
MONO_DEPTH_DEBUG_TOPIC = 'debug/g1/camera/mono_depth'
DEPTH_MAX_METERS_DEFAULT = 10.0
PREVIEW_BUFFER_LIMIT = 256


def extract_json_payload(text: str) -> dict[str, Any]:
    decoder = json.JSONDecoder()
    search_from = 0
    while True:
        start = text.find('{', search_from)
        if start == -1:
            raise RuntimeError(f'Failed to parse Gazebo JSON payload: {text.strip()}')
        try:
            payload, _ = decoder.raw_decode(text[start:])
        except json.JSONDecodeError:
            search_from = start + 1
            continue
        if isinstance(payload, dict):
            return payload
        search_from = start + 1


class G1DistanceBenchmarkRunner(Node):
    def __init__(self) -> None:
        super().__init__('g1_distance_benchmark_runner')

        pkg_share = get_package_share_directory('ridgeback_autonomy')
        self.g1_model_sdf = os.path.join(pkg_share, 'sim', 'models', 'g1', 'model.sdf')
        workspace_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
        default_output_dir = os.path.join(workspace_root, 'benchmark-results')

        self.declare_parameter('world', 'g1_distance_calibration')
        self.declare_parameter('repeats', 5)
        self.declare_parameter('output_dir', default_output_dir)
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('capture_sec', 10.0)
        self.declare_parameter('estimators', 'rgb,sensor_depth,depth_anything,pointcloud,lidar')
        self.declare_parameter('camera_measurement_topic', CAMERA_MEASUREMENT_TOPIC)
        self.declare_parameter('lidar_measurement_topic', LIDAR_MEASUREMENT_TOPIC)
        self.declare_parameter('mask_measurement_topic', MASK_MEASUREMENT_TOPIC)
        # The three config axes of the mask rows, stamped into their output
        # names so path_a/path_b runs are self-describing (must match the
        # values passed to the aligned depth + mask nodes for the same run).
        self.declare_parameter('depth_source', DEPTH_SOURCE_DEFAULT)
        self.declare_parameter('isolation_2d', ISOLATION_2D_DEFAULT)
        self.declare_parameter('isolation_3d', ISOLATION_3D_DEFAULT)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('mono_depth_debug_topic', MONO_DEPTH_DEBUG_TOPIC)
        self.declare_parameter('depth_max_meters', DEPTH_MAX_METERS_DEFAULT)

        self.world = str(self.get_parameter('world').value)
        self.repeats = int(self.get_parameter('repeats').value)
        self.output_dir = os.path.abspath(os.path.expanduser(str(self.get_parameter('output_dir').value)))
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.capture_sec = float(self.get_parameter('capture_sec').value)
        self.selected_estimators = parse_estimators(str(self.get_parameter('estimators').value))
        self.camera_measurement_topic = str(self.get_parameter('camera_measurement_topic').value)
        self.lidar_measurement_topic = str(self.get_parameter('lidar_measurement_topic').value)
        self.mask_measurement_topic = str(self.get_parameter('mask_measurement_topic').value)
        self.depth_source = str(self.get_parameter('depth_source').value)
        self.isolation_2d = str(self.get_parameter('isolation_2d').value)
        self.isolation_3d = str(self.get_parameter('isolation_3d').value)
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.mono_depth_debug_topic = str(self.get_parameter('mono_depth_debug_topic').value)
        self.depth_max_meters = float(self.get_parameter('depth_max_meters').value)

        self.namespace_name = self.get_namespace().strip('/')
        self.robot_model_name = (
            f'{self.namespace_name}/robot' if self.namespace_name else 'robot'
        )
        self.pose_info_topic = f'/world/{self.world}/pose/info'
        self.run_label = time.strftime('%Y%m%d_%H%M%S', time.localtime())
        self.run_output_dir = os.path.join(self.output_dir, self.run_label)
        self.images_dir = os.path.join(self.run_output_dir, 'images')
        os.makedirs(self.images_dir, exist_ok=False)

        # Self-describing output names: mask rows fold in source + isolation
        # recipe (benchmark_output_name); everything else keeps its plain key.
        self.estimator_output_names = {
            estimator: benchmark_output_name(
                estimator, self.depth_source, self.isolation_2d, self.isolation_3d)
            for estimator in self.selected_estimators
        }
        self.estimator_csv_paths = {
            estimator: os.path.join(
                self.run_output_dir, f'{self.estimator_output_names[estimator]}.csv')
            for estimator in self.selected_estimators
        }
        self.summary_csv_path = os.path.join(self.run_output_dir, 'comparison_summary.csv')

        self.command_env = os.environ.copy()
        self.command_env.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
        os.makedirs(self.command_env['ROS_LOG_DIR'], exist_ok=True)

        self.collage_renderer = BenchmarkCollageRenderer(self.depth_max_meters)

        self.needs_camera = uses_camera_estimators(self.selected_estimators)
        self.needs_lidar = uses_lidar_estimators(self.selected_estimators)
        self.needs_mask = uses_mask_estimators(self.selected_estimators)
        self.selected_camera_estimators = set(selected_camera_estimators(self.selected_estimators))
        self.selected_mask_estimators = set(selected_mask_estimators(self.selected_estimators))
        self.needs_sensor_depth_preview = 'sensor_depth' in self.selected_estimators
        self.needs_depth_anything_preview = 'depth_anything' in self.selected_estimators

        self.camera_measurement_seen = False
        self.lidar_measurement_seen = False
        self.mask_measurement_seen = False
        self.color_stream_seen = False
        self.depth_stream_seen = False
        self.depth_anything_stream_seen = False

        self.capture_active = False
        self.capture_events = {}
        self.color_preview_buffer: OrderedDict[int, Any] = OrderedDict()
        self.sensor_depth_preview_buffer: OrderedDict[int, Any] = OrderedDict()
        self.depth_anything_preview_buffer: OrderedDict[int, Any] = OrderedDict()

        self.last_color_decode_warning = None
        self.last_depth_decode_warning = None
        self.last_depth_anything_decode_warning = None

        self.create_subscription(
            G1Measurements,
            self.camera_measurement_topic,
            self.on_camera_measurement,
            10,
        )
        self.create_subscription(
            G1Measurements,
            self.lidar_measurement_topic,
            self.on_lidar_measurement,
            10,
        )
        self.create_subscription(
            G1Measurements,
            self.mask_measurement_topic,
            self.on_mask_measurement,
            10,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.on_color_image,
            qos_profile_sensor_data,
        )
        if self.needs_sensor_depth_preview:
            self.create_subscription(
                Image,
                self.depth_topic,
                self.on_depth_image,
                qos_profile_sensor_data,
            )
        if self.needs_depth_anything_preview:
            self.create_subscription(
                Image,
                self.mono_depth_debug_topic,
                self.on_depth_anything_image,
                qos_profile_sensor_data,
            )

    def on_camera_measurement(self, msg: G1Measurements) -> None:
        self.camera_measurement_seen = True
        if not self.capture_active:
            return

        event = ensure_measurement_event(self.capture_events, msg)
        update_measurement_event(event, msg, self.selected_camera_estimators)
        self.attach_buffered_previews(event)

    def on_lidar_measurement(self, msg: G1Measurements) -> None:
        self.lidar_measurement_seen = True
        if not self.capture_active:
            return

        event = ensure_measurement_event(self.capture_events, msg)
        update_measurement_event(event, msg, {'lidar'})
        self.attach_buffered_previews(event)

    def on_mask_measurement(self, msg: G1Measurements) -> None:
        self.mask_measurement_seen = True
        if not self.capture_active:
            return

        event = ensure_measurement_event(self.capture_events, msg)
        update_measurement_event(event, msg, self.selected_mask_estimators)
        self.attach_buffered_previews(event)

    def on_color_image(self, msg: Image) -> None:
        self.color_stream_seen = True
        if not self.capture_active:
            return

        try:
            frame = convert_color_image_message(msg)
        except Exception as exc:
            self.log_warning_once(
                'last_color_decode_warning',
                f'Failed to decode color frame from "{self.resolved_topic(self.color_topic)}": {exc}',
            )
            return

        preview = self.collage_renderer.make_color_preview(frame)
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.store_buffered_preview(self.color_preview_buffer, stamp_ns, preview)
        self.backfill_previews_from_buffers()

    def on_depth_image(self, msg: Image) -> None:
        self.depth_stream_seen = True
        if not self.capture_active:
            return

        try:
            depth_meters = convert_depth_to_meters_message(msg)
        except Exception as exc:
            self.log_warning_once(
                'last_depth_decode_warning',
                f'Failed to decode depth frame from "{self.resolved_topic(self.depth_topic)}": {exc}',
            )
            return

        preview = self.collage_renderer.make_depth_preview(depth_meters)
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.store_buffered_preview(self.sensor_depth_preview_buffer, stamp_ns, preview)
        self.backfill_previews_from_buffers()

    def on_depth_anything_image(self, msg: Image) -> None:
        self.depth_anything_stream_seen = True
        if not self.capture_active:
            return

        try:
            depth_meters = convert_depth_to_meters_message(msg)
        except Exception as exc:
            self.log_warning_once(
                'last_depth_anything_decode_warning',
                f'Failed to decode Depth-Anything frame from "{self.resolved_topic(self.mono_depth_debug_topic)}": {exc}',
            )
            return

        preview = self.collage_renderer.make_depth_preview(depth_meters)
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        self.store_buffered_preview(self.depth_anything_preview_buffer, stamp_ns, preview)
        self.backfill_previews_from_buffers()

    def run(self) -> None:
        self.get_logger().info(
            'Starting multi-estimator distance benchmark '
            f'for {", ".join(ESTIMATOR_LABELS[est] for est in self.selected_estimators)} '
            f'in world "{self.world}" for robot "{self.robot_model_name}".'
        )
        self.wait_for_required_streams()
        self.wait_for_entity_pose(self.robot_model_name, POSE_WAIT_TIMEOUT_SEC)

        estimator_rows = {estimator: [] for estimator in self.selected_estimators}
        included_trials = 0
        skipped_trials = 0

        for trial in self.build_trials():
            trial_rows = self.run_trial(trial)
            if trial_rows is None:
                skipped_trials += 1
                continue

            included_trials += 1
            for estimator, row in trial_rows.items():
                estimator_rows[estimator].append(row)

        for estimator in self.selected_estimators:
            write_trial_csv(self.estimator_csv_paths[estimator], estimator_rows[estimator])

        summary_rows = build_summary_rows(estimator_rows)
        for row in summary_rows:
            row['estimator'] = self.estimator_output_names.get(
                row['estimator'], row['estimator'])
        write_summary_csv(self.summary_csv_path, summary_rows)

        self.log_summary(summary_rows, included_trials, skipped_trials)
        self.get_logger().info(f'Benchmark run written to {self.run_output_dir}')

    def build_trials(self) -> list[dict[str, Any]]:
        trials = []
        for repeat_index in range(self.repeats):
            for forward_m in FORWARD_DISTANCES_M:
                for lateral_m in LATERAL_OFFSETS_M:
                    trials.append({
                        'repeat_index': repeat_index + 1,
                        'trial_id': f'pos_{len(trials) + 1:03d}',
                        'spawn_forward_m': forward_m,
                        'spawn_lateral_m': lateral_m,
                    })
        return trials

    def run_trial(self, trial: dict[str, Any]) -> dict[str, dict[str, Any]] | None:
        trial_id = trial['trial_id']
        model_name = f'benchmark_g1_{self.run_label}_{trial_id}'

        spawn_world_x = float(trial['spawn_forward_m'])
        spawn_world_y = float(trial['spawn_lateral_m'])
        spawn_yaw_rad = G1_FACING_ROBOT_YAW_RAD

        try:
            self.spawn_g1(model_name, spawn_world_x, spawn_world_y, G1_SPAWN_HEIGHT_M, spawn_yaw_rad)
            self.wait_for_entity_pose(model_name, POSE_WAIT_TIMEOUT_SEC)
            self.spin_for(self.settle_sec)

            true_pose = self.compute_ground_truth(model_name)
            capture = self.capture_measurement_window(self.capture_sec)
            usable_events = capture['usable_events']
            if not usable_events:
                self.get_logger().info(
                    f'{trial_id} skipped | reason=no_common_usable_events | '
                    f'raw_events={capture["total_events"]}'
                )
                return None

            trial_medians = compute_trial_medians(usable_events, self.selected_estimators)
            representative_event = choose_representative_event(
                usable_events,
                self.selected_estimators,
                trial_medians,
            )
            image_path = self.save_trial_collage(
                trial_id,
                representative_event,
                trial_medians,
                true_pose['distance_m'],
            )

            rows = {}
            for estimator in self.selected_estimators:
                estimate = trial_medians[estimator]
                abs_error = abs(estimate - true_pose['distance_m'])
                rel_error = abs_error / true_pose['distance_m'] if true_pose['distance_m'] > 0.0 else None
                rows[estimator] = {
                    'trial_id': trial_id,
                    'repeat_index': trial['repeat_index'],
                    'spawn_forward_m': trial['spawn_forward_m'],
                    'spawn_lateral_m': trial['spawn_lateral_m'],
                    'spawn_world_x': spawn_world_x,
                    'spawn_world_y': spawn_world_y,
                    'spawn_yaw_rad': spawn_yaw_rad,
                    'true_forward_m': true_pose['forward_m'],
                    'true_lateral_m': true_pose['lateral_m'],
                    'true_distance_m': true_pose['distance_m'],
                    'estimator': self.estimator_output_names[estimator],
                    'trial_estimate_m': estimate,
                    'abs_error_m': abs_error,
                    'rel_error': rel_error,
                    'usable_aligned_events': len(usable_events),
                    'image_path': image_path,
                }

            self.log_trial(trial_id, trial_medians, true_pose['distance_m'], len(usable_events), image_path)
            return rows
        except Exception as exc:
            self.get_logger().error(f'{trial_id} failed: {exc}')
            return None
        finally:
            try:
                self.delete_g1(model_name)
            except Exception as exc:
                self.get_logger().warn(f'Cleanup failed for "{model_name}": {exc}')

    def capture_measurement_window(self, duration_sec: float) -> dict[str, Any]:
        self.capture_events = {}
        self.clear_preview_buffers()
        self.capture_active = True
        try:
            self.spin_for(duration_sec)
        finally:
            self.capture_active = False

        usable_events = usable_aligned_events(self.capture_events, self.selected_estimators)
        return {
            'total_events': len(self.capture_events),
            'usable_events': usable_events,
        }

    def save_trial_collage(
        self,
        trial_id: str,
        representative_event,
        trial_medians: dict[str, float],
        true_distance_m: float,
    ) -> str:
        collage = self.collage_renderer.render_trial_collage(
            trial_id,
            representative_event,
            self.selected_estimators,
            trial_medians,
            true_distance_m,
        )
        output_path = os.path.abspath(os.path.join(self.images_dir, f'{trial_id}.png'))
        if not cv2.imwrite(output_path, collage):
            raise RuntimeError(f'Failed to write collage image to {output_path}')
        return output_path

    def clear_preview_buffers(self) -> None:
        self.color_preview_buffer.clear()
        self.sensor_depth_preview_buffer.clear()
        self.depth_anything_preview_buffer.clear()

    def store_buffered_preview(
        self,
        preview_buffer: OrderedDict[int, Any],
        stamp_ns: int,
        preview,
    ) -> None:
        preview_buffer[stamp_ns] = preview
        preview_buffer.move_to_end(stamp_ns)
        while len(preview_buffer) > PREVIEW_BUFFER_LIMIT:
            preview_buffer.popitem(last=False)

    def backfill_previews_from_buffers(self) -> None:
        if not self.capture_active:
            return

        for event in self.capture_events.values():
            self.attach_buffered_previews(event)

    def attach_buffered_previews(self, event) -> None:
        self.apply_preview_match(
            event,
            'color',
            find_exact_preview_match(self.color_preview_buffer, event.stamp_ns),
        )

        if self.needs_sensor_depth_preview:
            self.apply_preview_match(
                event,
                'sensor_depth',
                find_nearest_preview_match(
                    self.sensor_depth_preview_buffer,
                    event.stamp_ns,
                    IMAGE_MATCH_TOLERANCE_NS,
                ),
            )

        if self.needs_depth_anything_preview:
            self.apply_preview_match(
                event,
                'depth_anything',
                find_exact_preview_match(self.depth_anything_preview_buffer, event.stamp_ns),
            )

    def apply_preview_match(self, event, prefix: str, match) -> None:
        setattr(event.preview, f'{prefix}_nearest_stamp_ns', match.nearest_stamp_ns)
        setattr(event.preview, f'{prefix}_nearest_delta_ms', match.nearest_delta_ms)

        if match.image_bgr is None or match.matched_stamp_ns is None:
            return

        preview_attribute = f'{prefix}_bgr'
        stamp_attribute = f'{prefix}_stamp_ns'
        delta_attribute = f'{prefix}_delta_ms'
        current_delta_ms = getattr(event.preview, delta_attribute)
        if current_delta_ms is not None and match.matched_delta_ms is not None and current_delta_ms <= match.matched_delta_ms:
            return

        setattr(event.preview, preview_attribute, match.image_bgr)
        setattr(event.preview, stamp_attribute, match.matched_stamp_ns)
        setattr(event.preview, delta_attribute, match.matched_delta_ms)

    def wait_for_required_streams(self) -> None:
        if self.needs_camera:
            self.wait_for_flag(
                lambda: self.camera_measurement_seen,
                f'measurement stream on "{self.resolved_topic(self.camera_measurement_topic)}"',
            )
        if self.needs_lidar:
            self.wait_for_flag(
                lambda: self.lidar_measurement_seen,
                f'measurement stream on "{self.resolved_topic(self.lidar_measurement_topic)}"',
            )
        if self.needs_mask:
            self.wait_for_flag(
                lambda: self.mask_measurement_seen,
                f'measurement stream on "{self.resolved_topic(self.mask_measurement_topic)}"',
            )

        self.wait_for_flag(
            lambda: self.color_stream_seen,
            f'color stream on "{self.resolved_topic(self.color_topic)}"',
        )
        if self.needs_sensor_depth_preview:
            self.wait_for_flag(
                lambda: self.depth_stream_seen,
                f'depth stream on "{self.resolved_topic(self.depth_topic)}"',
            )
        if self.needs_depth_anything_preview:
            self.wait_for_flag(
                lambda: self.depth_anything_stream_seen,
                f'Depth-Anything stream on "{self.resolved_topic(self.mono_depth_debug_topic)}"',
            )

    def wait_for_flag(self, condition, description: str) -> None:
        self.get_logger().info(f'Waiting for {description}.')
        deadline = time.monotonic() + STREAM_WAIT_TIMEOUT_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if condition():
                return
        raise RuntimeError(f'Timed out waiting for {description}')

    def wait_for_entity_pose(self, model_name: str, timeout_sec: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                snapshot = self.get_pose_snapshot()
            except RuntimeError:
                time.sleep(COMMAND_RETRY_SLEEP_SEC)
                continue
            pose = snapshot.get(model_name)
            if pose is not None:
                return pose
            time.sleep(COMMAND_RETRY_SLEEP_SEC)
        raise RuntimeError(f'Timed out waiting for Gazebo pose of "{model_name}"')

    def compute_ground_truth(self, model_name: str) -> dict[str, float]:
        snapshot = self.get_pose_snapshot()
        robot_pose = snapshot.get(self.robot_model_name)
        target_pose = snapshot.get(model_name)

        if robot_pose is None:
            raise RuntimeError(f'Robot pose "{self.robot_model_name}" not present on {self.pose_info_topic}')
        if target_pose is None:
            raise RuntimeError(f'Target pose "{model_name}" not present on {self.pose_info_topic}')

        dx_world = target_pose['position']['x'] - robot_pose['position']['x']
        dy_world = target_pose['position']['y'] - robot_pose['position']['y']
        robot_yaw = yaw_from_quaternion(
            robot_pose['orientation']['x'],
            robot_pose['orientation']['y'],
            robot_pose['orientation']['z'],
            robot_pose['orientation']['w'],
        )
        forward_m, lateral_m, distance_m = planar_distance_from_vehicle_origin(
            dx_world,
            dy_world,
            robot_yaw,
        )
        return {
            'forward_m': forward_m,
            'lateral_m': lateral_m,
            'distance_m': distance_m,
        }

    def spawn_g1(self, model_name: str, x_m: float, y_m: float, z_m: float, yaw_rad: float) -> None:
        command = [
            '/opt/ros/jazzy/lib/ros_gz_sim/create',
            '-world', self.world,
            '-file', self.g1_model_sdf,
            '-name', model_name,
            '-x', f'{x_m:.6f}',
            '-y', f'{y_m:.6f}',
            '-z', f'{z_m:.6f}',
            '-Y', f'{yaw_rad:.6f}',
        ]
        deadline = time.monotonic() + POSE_WAIT_TIMEOUT_SEC
        last_error = None
        while time.monotonic() < deadline:
            try:
                self.run_command(
                    command,
                    timeout_sec=COMMAND_TIMEOUT_SEC,
                    description=f'spawn {model_name}',
                )
                return
            except RuntimeError as exc:
                last_error = exc
                time.sleep(COMMAND_RETRY_SLEEP_SEC)
        raise RuntimeError(f'Failed to spawn "{model_name}": {last_error}')

    def delete_g1(self, model_name: str) -> None:
        deadline = time.monotonic() + DELETE_TIMEOUT_SEC
        command = [
            'gz',
            'service',
            '-s', f'/world/{self.world}/remove/blocking',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', f'name: "{model_name}" type: MODEL',
        ]

        while time.monotonic() < deadline:
            result = self.try_command(command, timeout_sec=COMMAND_TIMEOUT_SEC)
            if result.returncode == 0:
                break
            time.sleep(COMMAND_RETRY_SLEEP_SEC)

        absence_deadline = time.monotonic() + DELETE_TIMEOUT_SEC
        while time.monotonic() < absence_deadline:
            try:
                snapshot = self.get_pose_snapshot()
            except RuntimeError:
                time.sleep(COMMAND_RETRY_SLEEP_SEC)
                continue
            if model_name not in snapshot:
                return
            time.sleep(COMMAND_RETRY_SLEEP_SEC)

        self.get_logger().warn(f'Entity "{model_name}" was not removed before timeout.')

    def get_pose_snapshot(self) -> dict[str, dict[str, Any]]:
        command = [
            'gz',
            'topic',
            '-e',
            '-t', self.pose_info_topic,
            '--json-output',
            '-n', '1',
        ]
        result = self.run_command(
            command,
            timeout_sec=COMMAND_TIMEOUT_SEC,
            description=f'read {self.pose_info_topic}',
        )
        payload = self.extract_json_payload(result.stdout)
        poses = payload.get('pose', [])
        return {
            pose['name']: self.normalize_pose(pose)
            for pose in poses
            if isinstance(pose, dict) and pose.get('name')
        }

    def normalize_pose(self, pose: dict[str, Any]) -> dict[str, Any]:
        position = pose.get('position') or {}
        orientation = pose.get('orientation') or {}
        return {
            **pose,
            'position': {
                'x': float(position.get('x', 0.0)),
                'y': float(position.get('y', 0.0)),
                'z': float(position.get('z', 0.0)),
            },
            'orientation': {
                'x': float(orientation.get('x', 0.0)),
                'y': float(orientation.get('y', 0.0)),
                'z': float(orientation.get('z', 0.0)),
                'w': float(orientation.get('w', 1.0)),
            },
        }

    def extract_json_payload(self, text: str) -> dict[str, Any]:
        return extract_json_payload(text)

    def run_command(
        self,
        command: list[str],
        timeout_sec: float,
        description: str,
    ) -> subprocess.CompletedProcess[str]:
        result = self.try_command(command, timeout_sec=timeout_sec)
        if result.returncode != 0:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            raise RuntimeError(
                f'Failed to {description}: exit={result.returncode} '
                f'stdout="{stdout}" stderr="{stderr}"'
            )
        return result

    def try_command(
        self,
        command: list[str],
        timeout_sec: float,
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            env=self.command_env,
            timeout=timeout_sec,
        )

    def resolved_topic(self, topic: str) -> str:
        if topic.startswith('/'):
            return topic
        if self.namespace_name:
            return f'/{self.namespace_name}/{topic}'
        return f'/{topic}'

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def log_warning_once(self, attribute_name: str, warning: str) -> None:
        if warning == getattr(self, attribute_name):
            return
        setattr(self, attribute_name, warning)
        self.get_logger().warn(warning)

    def log_trial(
        self,
        trial_id: str,
        trial_medians: dict[str, float],
        true_distance_m: float,
        usable_events: int,
        image_path: str,
    ) -> None:
        parts = [
            trial_id,
            f'true={true_distance_m:.3f}m',
            f'usable_events={usable_events}',
            f'image={os.path.basename(image_path)}',
        ]
        for estimator in self.selected_estimators:
            parts.append(f'{estimator}={trial_medians[estimator]:.3f}m')
        self.get_logger().info(' | '.join(parts))

    def log_summary(
        self,
        summary_rows: list[dict[str, Any]],
        included_trials: int,
        skipped_trials: int,
    ) -> None:
        self.get_logger().info(
            f'Included trials: {included_trials} | skipped trials: {skipped_trials}'
        )
        for row in summary_rows:
            if row['trial_count'] == 0:
                self.get_logger().info(f'{row["estimator"]}: no comparable trials.')
                continue
            # row['estimator'] is the output name here (mask rows are already
            # renamed to their self-describing form); fall back to it when it
            # is not one of the fixed ESTIMATOR_LABELS keys.
            self.get_logger().info(
                f'{ESTIMATOR_LABELS.get(row["estimator"], row["estimator"])} | '
                f'n={row["trial_count"]} | '
                f'MAE={row["mean_abs_error_m"]:.3f}m | '
                f'MedianAE={row["median_abs_error_m"]:.3f}m | '
                f'P95AE={row["p95_abs_error_m"]:.3f}m | '
                f'MeanRel={row["mean_rel_error"]:.3f}'
            )


def main() -> int:
    rclpy.init()
    node = G1DistanceBenchmarkRunner()

    try:
        node.run()
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except Exception as exc:  # pragma: no cover - surfaced in logs for runtime debugging
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
