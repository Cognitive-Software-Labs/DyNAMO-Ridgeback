#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import math
import os
import statistics
import subprocess
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image

from ridgeback_autonomy.benchmarking.metrics import (
    PRIMARY_METRIC_KEYS,
    PRIMARY_METRIC_LABELS,
)
from ridgeback_autonomy.common.messages import (
    first_finite_positive,
    snapshot_measurements_message,
)
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.geometry import (
    planar_distance_from_vehicle_origin,
    yaw_from_quaternion,
)
from ridgeback_autonomy.perception.core.image_utils import convert_color_image_message


NEGATIVE_CONTROL_TRIALS = 10
FORWARD_DISTANCES_M = [1.5, 2.5, 3.5, 4.5, 5.5]
LATERAL_OFFSETS_M = [-0.75, 0.0, 0.75]
G1_SPAWN_HEIGHT_M = 0.0
G1_FACING_ROBOT_YAW_RAD = math.pi
COMMAND_TIMEOUT_SEC = 10.0
COMMAND_RETRY_SLEEP_SEC = 0.5
STREAM_WAIT_TIMEOUT_SEC = 300.0
POSE_WAIT_TIMEOUT_SEC = 120.0
DELETE_TIMEOUT_SEC = 15.0


class G1DistanceBenchmarkRunner(Node):
    def __init__(self) -> None:
        super().__init__('g1_distance_benchmark_runner')

        pkg_share = get_package_share_directory('ridgeback_autonomy')
        self.g1_model_sdf = os.path.join(pkg_share, 'sim', 'models', 'g1', 'model.sdf')

        self.declare_parameter('world', 'g1_distance_calibration')
        self.declare_parameter('repeats', 5)
        self.declare_parameter('output_csv', '/tmp/g1_distance_benchmark_camera.csv')
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('capture_sec', 3.0)
        self.declare_parameter('measurement_topic', 'measurements/g1/camera')
        self.declare_parameter('primary_metric', 'rgb')
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('failed_frame_dir', '')
        self.declare_parameter('save_failed_frames', True)

        self.world = str(self.get_parameter('world').value)
        self.repeats = int(self.get_parameter('repeats').value)
        self.output_csv = str(self.get_parameter('output_csv').value)
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.capture_sec = float(self.get_parameter('capture_sec').value)
        self.measurement_topic = str(self.get_parameter('measurement_topic').value)
        self.primary_metric = str(self.get_parameter('primary_metric').value)
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.failed_frame_dir = str(self.get_parameter('failed_frame_dir').value)
        self.save_failed_frames = bool(self.get_parameter('save_failed_frames').value)
        if self.primary_metric not in PRIMARY_METRIC_KEYS:
            supported = ', '.join(sorted(PRIMARY_METRIC_KEYS))
            raise ValueError(
                f'Unsupported primary_metric "{self.primary_metric}". '
                f'Expected one of: {supported}'
            )

        self.namespace_name = self.get_namespace().strip('/')
        self.robot_model_name = (
            f'{self.namespace_name}/robot' if self.namespace_name else 'robot'
        )
        self.pose_info_topic = f'/world/{self.world}/pose/info'
        self.run_label = time.strftime('%Y%m%d_%H%M%S')

        output_dir = os.path.dirname(self.output_csv) or '.'
        os.makedirs(output_dir, exist_ok=True)
        self.output_csv = os.path.abspath(self.output_csv)
        if not self.failed_frame_dir:
            csv_root, _ = os.path.splitext(self.output_csv)
            self.failed_frame_dir = os.path.join(
                f'{csv_root}_frames',
                self.run_label,
            )
        self.failed_frame_dir = os.path.abspath(self.failed_frame_dir)

        self.command_env = os.environ.copy()
        self.command_env.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
        os.makedirs(self.command_env['ROS_LOG_DIR'], exist_ok=True)
        if self.save_failed_frames:
            os.makedirs(self.failed_frame_dir, exist_ok=True)

        self.latest_measurement_msg = None
        self.capture_samples: list[dict[str, Any]] = []
        self.capture_active = False
        self.capture_last_measurement_snapshot = None
        self.capture_last_color_frame = None
        self.capture_last_color_stamp_ns = None
        self.last_color_decode_warning = None
        self.color_stream_seen = False

        self.create_subscription(
            G1Measurements,
            self.measurement_topic,
            self.on_measurement,
            10,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.on_color_image,
            10,
        )

    def on_measurement(self, msg: G1Measurements) -> None:
        self.latest_measurement_msg = msg
        snapshot = snapshot_measurements_message(msg)
        if not self.capture_active:
            return

        self.capture_last_measurement_snapshot = snapshot
        sample = {
            'count': int(msg.count),
            'detected': bool(msg.detected),
            'rgb_distance_m': first_finite_positive(msg.rgb_distance_m),
            'sensor_depth_distance_m': first_finite_positive(msg.sensor_depth_distance_m),
            'mono_depth_distance_m': first_finite_positive(msg.mono_depth_distance_m),
            'lidar_distance_m': first_finite_positive(msg.lidar_distance_m),
            'pointcloud_distance_m': first_finite_positive(msg.pointcloud_distance_m),
        }
        self.capture_samples.append(sample)

    def on_color_image(self, msg: Image) -> None:
        self.color_stream_seen = True
        if not self.capture_active:
            return

        try:
            frame = convert_color_image_message(msg)
        except Exception as exc:
            warning = f'Failed to decode color frame from "{self.resolved_color_topic()}": {exc}'
            if warning != self.last_color_decode_warning:
                self.last_color_decode_warning = warning
                self.get_logger().warn(warning)
            return

        self.capture_last_color_frame = frame
        self.capture_last_color_stamp_ns = self.stamp_to_nanoseconds(msg.header.stamp)

    def run(self) -> None:
        self.get_logger().info(
            f'Starting static {self.primary_metric} distance benchmark in world "{self.world}" '
            f'for robot "{self.robot_model_name}".'
        )
        self.wait_for_measurement_stream(STREAM_WAIT_TIMEOUT_SEC)
        if self.save_failed_frames:
            self.wait_for_color_stream(STREAM_WAIT_TIMEOUT_SEC)
        self.wait_for_entity_pose(self.robot_model_name, POSE_WAIT_TIMEOUT_SEC)

        rows = []
        csv_columns = [
            'trial_id',
            'trial_kind',
            'repeat_index',
            'status',
            'failure_reason',
            'spawn_forward_m',
            'spawn_lateral_m',
            'spawn_world_x',
            'spawn_world_y',
            'spawn_yaw_rad',
            'true_forward_m',
            'true_lateral_m',
            'true_distance_m',
            'rgb_distance_m',
            'sensor_depth_distance_m',
            'mono_depth_distance_m',
            'lidar_distance_m',
            'pointcloud_distance_m',
            'primary_metric',
            'primary_distance_m',
            'abs_error_m',
            'rel_error',
            'primary_abs_error_m',
            'primary_rel_error',
            'total_frames',
            'usable_frames',
            'ambiguous_frames',
            'saved_frame_path',
            'saved_frame_stamp_ns',
            'saved_frame_type',
        ]

        with open(self.output_csv, 'w', encoding='utf-8', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=csv_columns)
            writer.writeheader()

            for trial in self.build_trials():
                row = self.run_trial(trial)
                writer.writerow(row)
                csv_file.flush()
                rows.append(row)

        self.print_summary(rows)
        self.get_logger().info(f'Benchmark CSV written to {self.output_csv}')

    def build_trials(self) -> list[dict[str, Any]]:
        trials = []

        for index in range(NEGATIVE_CONTROL_TRIALS):
            trials.append({
                'trial_kind': 'negative_control',
                'repeat_index': 0,
                'trial_id': f'neg_{index + 1:02d}',
                'spawn_forward_m': None,
                'spawn_lateral_m': None,
            })

        for repeat_index in range(self.repeats):
            for forward_m in FORWARD_DISTANCES_M:
                for lateral_m in LATERAL_OFFSETS_M:
                    trial_index = len(trials) + 1
                    trials.append({
                        'trial_kind': 'positive',
                        'repeat_index': repeat_index + 1,
                        'trial_id': f'pos_{trial_index:03d}',
                        'spawn_forward_m': forward_m,
                        'spawn_lateral_m': lateral_m,
                    })

        return trials

    def run_trial(self, trial: dict[str, Any]) -> dict[str, Any]:
        trial_kind = trial['trial_kind']
        trial_id = trial['trial_id']
        repeat_index = trial['repeat_index']

        row = {
            'trial_id': trial_id,
            'trial_kind': trial_kind,
            'repeat_index': repeat_index,
            'status': '',
            'failure_reason': '',
            'spawn_forward_m': trial['spawn_forward_m'],
            'spawn_lateral_m': trial['spawn_lateral_m'],
            'spawn_world_x': None,
            'spawn_world_y': None,
            'spawn_yaw_rad': None,
            'true_forward_m': None,
            'true_lateral_m': None,
            'true_distance_m': None,
            'rgb_distance_m': None,
            'sensor_depth_distance_m': None,
            'mono_depth_distance_m': None,
            'lidar_distance_m': None,
            'pointcloud_distance_m': None,
            'primary_metric': self.primary_metric,
            'primary_distance_m': None,
            'abs_error_m': None,
            'rel_error': None,
            'primary_abs_error_m': None,
            'primary_rel_error': None,
            'total_frames': 0,
            'usable_frames': 0,
            'ambiguous_frames': 0,
            'saved_frame_path': '',
            'saved_frame_stamp_ns': '',
            'saved_frame_type': '',
        }

        if trial_kind == 'negative_control':
            capture = self.capture_measurement_window(self.capture_sec)
            self.fill_capture_metrics(row, capture)
            if capture['usable_frames'] == 0 and capture['ambiguous_frames'] == 0:
                row['status'] = 'ok'
            else:
                row['status'] = 'ambiguous'
                row['failure_reason'] = 'unexpected_detection_in_negative_control'
            self.log_trial(row)
            return row

        forward_m = float(trial['spawn_forward_m'])
        lateral_m = float(trial['spawn_lateral_m'])
        spawn_world_x = forward_m
        spawn_world_y = lateral_m
        spawn_yaw_rad = G1_FACING_ROBOT_YAW_RAD
        row['spawn_world_x'] = spawn_world_x
        row['spawn_world_y'] = spawn_world_y
        row['spawn_yaw_rad'] = spawn_yaw_rad

        model_name = f'benchmark_g1_{self.run_label}_{trial_id}'
        capture = None
        try:
            try:
                self.spawn_g1(model_name, spawn_world_x, spawn_world_y, G1_SPAWN_HEIGHT_M, spawn_yaw_rad)
                self.wait_for_entity_pose(model_name, POSE_WAIT_TIMEOUT_SEC)
                self.spin_for(self.settle_sec)

                true_pose = self.compute_ground_truth(model_name)
                row['true_forward_m'] = true_pose['forward_m']
                row['true_lateral_m'] = true_pose['lateral_m']
                row['true_distance_m'] = true_pose['distance_m']

                capture = self.capture_measurement_window(self.capture_sec)
                self.fill_capture_metrics(row, capture)

                if capture['usable_frames'] > 0:
                    row['status'] = 'ok'
                    row['rgb_distance_m'] = capture['rgb_distance_m']
                    row['sensor_depth_distance_m'] = capture['sensor_depth_distance_m']
                    row['mono_depth_distance_m'] = capture['mono_depth_distance_m']
                    row['lidar_distance_m'] = capture['lidar_distance_m']
                    row['pointcloud_distance_m'] = capture['pointcloud_distance_m']
                    row['primary_distance_m'] = capture['primary_distance_m']
                    row['abs_error_m'] = abs(row['primary_distance_m'] - row['true_distance_m'])
                    if row['true_distance_m'] > 0.0:
                        row['rel_error'] = row['abs_error_m'] / row['true_distance_m']
                    row['primary_abs_error_m'] = row['abs_error_m']
                    row['primary_rel_error'] = row['rel_error']
                elif capture['ambiguous_frames'] > 0:
                    row['status'] = 'ambiguous'
                    row['failure_reason'] = 'detections_present_but_not_single_target'
                else:
                    row['status'] = 'missed'
                    row['failure_reason'] = capture['miss_reason']
            except Exception as exc:
                row['status'] = 'ambiguous'
                row['failure_reason'] = self.format_failure_reason(exc)
                self.get_logger().error(f'{trial_id} failed: {exc}')
        finally:
            try:
                self.delete_g1(model_name)
            except Exception as exc:
                self.get_logger().warn(f'Cleanup failed for "{model_name}": {exc}')

        self.maybe_save_failed_frame(row, capture)
        self.log_trial(row)
        return row

    def fill_capture_metrics(self, row: dict[str, Any], capture: dict[str, Any]) -> None:
        row['total_frames'] = capture['total_frames']
        row['usable_frames'] = capture['usable_frames']
        row['ambiguous_frames'] = capture['ambiguous_frames']

    def format_failure_reason(self, exc: Exception) -> str:
        detail = str(exc).strip().replace('\n', ' ')
        detail = '_'.join(detail.split())
        if not detail:
            detail = exc.__class__.__name__
        detail = detail[:120]
        return f'runner_error:{detail}'

    def capture_measurement_window(self, duration_sec: float) -> dict[str, Any]:
        self.capture_samples = []
        self.capture_last_measurement_snapshot = None
        self.capture_last_color_frame = None
        self.capture_last_color_stamp_ns = None
        self.capture_active = True
        try:
            self.spin_for(duration_sec)
        finally:
            self.capture_active = False

        primary_key = PRIMARY_METRIC_KEYS[self.primary_metric]
        usable_samples = [
            sample for sample in self.capture_samples
            if sample['detected'] and sample['count'] == 1 and sample[primary_key] is not None
        ]
        ambiguous_frames = sum(
            1 for sample in self.capture_samples
            if sample['detected'] and sample['count'] != 1
        )
        detected_frames = sum(1 for sample in self.capture_samples if sample['detected'])

        result = {
            'total_frames': len(self.capture_samples),
            'usable_frames': len(usable_samples),
            'ambiguous_frames': ambiguous_frames,
            'detected_frames': detected_frames,
            'rgb_distance_m': None,
            'sensor_depth_distance_m': None,
            'mono_depth_distance_m': None,
            'lidar_distance_m': None,
            'pointcloud_distance_m': None,
            'primary_distance_m': None,
            'last_measurement': self.capture_last_measurement_snapshot,
            'last_frame': self.capture_last_color_frame,
            'last_frame_stamp_ns': self.capture_last_color_stamp_ns,
            'miss_reason': 'no_detection',
        }

        if not usable_samples:
            if detected_frames > 0:
                result['miss_reason'] = f'no_{self.primary_metric}_measurement'
            return result

        result['rgb_distance_m'] = self.safe_median(
            [sample['rgb_distance_m'] for sample in usable_samples]
        )
        result['sensor_depth_distance_m'] = self.safe_median(
            [sample['sensor_depth_distance_m'] for sample in usable_samples]
        )
        result['mono_depth_distance_m'] = self.safe_median(
            [sample['mono_depth_distance_m'] for sample in usable_samples]
        )
        result['lidar_distance_m'] = self.safe_median(
            [sample['lidar_distance_m'] for sample in usable_samples]
        )
        result['pointcloud_distance_m'] = self.safe_median(
            [sample['pointcloud_distance_m'] for sample in usable_samples]
        )
        result['primary_distance_m'] = result[primary_key]
        return result

    def wait_for_measurement_stream(self, timeout_sec: float) -> None:
        self.get_logger().info(
            f'Waiting for measurement stream on "{self.resolved_measurement_topic()}".'
        )
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.latest_measurement_msg is not None:
                return
        raise RuntimeError(
            f'Timed out waiting for measurement messages on {self.resolved_measurement_topic()}'
        )

    def wait_for_color_stream(self, timeout_sec: float) -> None:
        self.get_logger().info(
            f'Waiting for color stream on "{self.resolved_color_topic()}".'
        )
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.color_stream_seen:
                return
        raise RuntimeError(
            f'Timed out waiting for color messages on {self.resolved_color_topic()}'
        )

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
        start = text.find('{')
        end = text.rfind('}')
        if start == -1 or end == -1 or end <= start:
            raise RuntimeError(f'Failed to parse Gazebo JSON payload: {text.strip()}')
        return json.loads(text[start:end + 1])

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

    def resolved_measurement_topic(self) -> str:
        if self.measurement_topic.startswith('/'):
            return self.measurement_topic
        if self.namespace_name:
            return f'/{self.namespace_name}/{self.measurement_topic}'
        return f'/{self.measurement_topic}'

    def resolved_color_topic(self) -> str:
        if self.color_topic.startswith('/'):
            return self.color_topic
        if self.namespace_name:
            return f'/{self.namespace_name}/{self.color_topic}'
        return f'/{self.color_topic}'

    def spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def safe_median(self, values: list[float | None]) -> float | None:
        filtered = [value for value in values if value is not None]
        if not filtered:
            return None
        return float(statistics.median(filtered))

    def maybe_save_failed_frame(self, row: dict[str, Any], capture: dict[str, Any] | None) -> None:
        if not self.save_failed_frames:
            return
        if row['trial_kind'] != 'positive':
            return
        if row['status'] not in ('missed', 'ambiguous'):
            return
        if capture is None:
            return

        frame = capture.get('last_frame')
        if frame is None:
            return

        os.makedirs(self.failed_frame_dir, exist_ok=True)
        annotated = self.render_failed_frame(frame, row, capture)
        output_path = os.path.abspath(
            os.path.join(self.failed_frame_dir, f'{row["trial_id"]}_{row["status"]}.png')
        )
        if not cv2.imwrite(output_path, annotated):
            raise RuntimeError(f'Failed to write annotated frame to {output_path}')

        row['saved_frame_path'] = output_path
        stamp_ns = capture.get('last_frame_stamp_ns')
        row['saved_frame_stamp_ns'] = str(stamp_ns) if stamp_ns is not None else ''
        row['saved_frame_type'] = 'annotated_rgb'

    def render_failed_frame(
        self,
        frame: np.ndarray,
        row: dict[str, Any],
        capture: dict[str, Any],
    ) -> np.ndarray:
        panel = frame.copy()
        status = row['status']
        status_color = (0, 165, 255) if status == 'ambiguous' else (0, 0, 255)
        last_measurement = capture.get('last_measurement') or {}
        bbox_list = last_measurement.get('bboxes', [])

        for index, bbox in enumerate(bbox_list):
            x1, y1, x2, y2 = bbox
            cv2.rectangle(panel, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                panel,
                f'G1 #{index + 1}',
                (x1, max(24, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        status_label = status
        if status == 'missed' and row['failure_reason'] == 'no_detection':
            status_label = 'missed/no_detection'

        lines = [
            f'Trial: {row["trial_id"]}',
            f'Status: {status_label}',
            f'Primary: {row["primary_metric"]}',
            f'True: {self.format_distance(row["true_distance_m"])}',
            f'PrimaryD: {self.format_distance(row["primary_distance_m"])}',
            f'RGB: {self.format_distance(row["rgb_distance_m"])}',
            f'SensorDepth: {self.format_distance(row["sensor_depth_distance_m"])}',
            f'MonoDepth: {self.format_distance(row["mono_depth_distance_m"])}',
            f'LiDAR: {self.format_distance(row["lidar_distance_m"])}',
            f'PointCloud: {self.format_distance(row["pointcloud_distance_m"])}',
            (
                f'LastMeas: detected={last_measurement.get("detected", False)} '
                f'count={last_measurement.get("count", 0)}'
            ),
            (
                f'Frames: total={row["total_frames"]} usable={row["usable_frames"]} '
                f'ambiguous={row["ambiguous_frames"]}'
            ),
        ]
        if row['failure_reason']:
            lines.append(f'Reason: {row["failure_reason"]}')

        self.draw_label_block(panel, lines, status_color)
        return panel

    def draw_label_block(
        self,
        panel: np.ndarray,
        lines: list[str],
        status_color: tuple[int, int, int],
    ) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 1
        line_height = 24
        padding = 12
        widths = [
            cv2.getTextSize(line, font, font_scale, thickness)[0][0]
            for line in lines
        ]
        block_width = max(widths, default=0) + padding * 2
        block_height = len(lines) * line_height + padding * 2
        cv2.rectangle(panel, (8, 8), (8 + block_width, 8 + block_height), (20, 20, 20), -1)
        cv2.rectangle(panel, (8, 8), (8 + block_width, 8 + block_height), status_color, 2)

        text_y = 8 + padding + 16
        for line in lines:
            cv2.putText(
                panel,
                line,
                (8 + padding, text_y),
                font,
                font_scale,
                (255, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            text_y += line_height

    def format_distance(self, value: float | None) -> str:
        if value is None:
            return 'N/A'
        return f'{value:.3f}m'

    def stamp_to_nanoseconds(self, stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def print_summary(self, rows: list[dict[str, Any]]) -> None:
        negative_rows = [row for row in rows if row['trial_kind'] == 'negative_control']
        positive_rows = [row for row in rows if row['trial_kind'] == 'positive']
        ok_positive_rows = [row for row in positive_rows if row['status'] == 'ok']

        negative_ok = sum(1 for row in negative_rows if row['status'] == 'ok')
        positive_ok = len(ok_positive_rows)
        positive_total = len(positive_rows)
        detection_rate = positive_ok / positive_total if positive_total else 0.0

        self.get_logger().info(
            f'Negative controls: {negative_ok}/{len(negative_rows)} clean '
            f'({len(negative_rows) - negative_ok} unexpected detections).'
        )
        self.get_logger().info(
            f'Positive detection rate: {positive_ok}/{positive_total} '
            f'({detection_rate:.1%}).'
        )

        abs_errors = [
            row['primary_abs_error_m']
            for row in ok_positive_rows
            if row['primary_abs_error_m'] is not None
        ]
        if not abs_errors:
            self.get_logger().warn('No successful positive trials were available for error statistics.')
            return

        mean_abs_error = statistics.fmean(abs_errors)
        median_abs_error = statistics.median(abs_errors)
        percentile_95_error = float(np.percentile(np.asarray(abs_errors, dtype=np.float64), 95))
        metric_label = PRIMARY_METRIC_LABELS[self.primary_metric]

        self.get_logger().info(
            f'{metric_label} error stats: '
            f'MAE={mean_abs_error:.3f} m, '
            f'MedianAE={median_abs_error:.3f} m, '
            f'P95AE={percentile_95_error:.3f} m.'
        )

        for forward_m in FORWARD_DISTANCES_M:
            bucket_rows = [
                row for row in ok_positive_rows
                if row['spawn_forward_m'] == forward_m and row['primary_abs_error_m'] is not None
            ]
            if not bucket_rows:
                self.get_logger().info(f'Bucket {forward_m:.1f} m: no successful trials.')
                continue
            bucket_errors = [row['primary_abs_error_m'] for row in bucket_rows]
            self.get_logger().info(
                f'Bucket {forward_m:.1f} m: '
                f'n={len(bucket_rows)}, '
                f'MAE={statistics.fmean(bucket_errors):.3f} m, '
                f'MedianAE={statistics.median(bucket_errors):.3f} m.'
            )

    def log_trial(self, row: dict[str, Any]) -> None:
        parts = [
            f'{row["trial_id"]}',
            f'type={row["trial_kind"]}',
            f'status={row["status"]}',
        ]
        if row['true_distance_m'] is not None:
            parts.append(f'true={row["true_distance_m"]:.3f}m')
        if row['primary_distance_m'] is not None:
            parts.append(f'{row["primary_metric"]}={row["primary_distance_m"]:.3f}m')

        for metric_key in (
            'rgb_distance_m',
            'sensor_depth_distance_m',
            'mono_depth_distance_m',
            'lidar_distance_m',
            'pointcloud_distance_m',
        ):
            value = row[metric_key]
            if value is not None:
                parts.append(f'{metric_key}={value:.3f}m')

        if row['primary_abs_error_m'] is not None:
            parts.append(f'abs_err={row["primary_abs_error_m"]:.3f}m')
        parts.append(f'usable={row["usable_frames"]}')
        parts.append(f'ambiguous={row["ambiguous_frames"]}')
        if row['failure_reason']:
            parts.append(f'reason={row["failure_reason"]}')
        if row['saved_frame_path']:
            parts.append(f'frame={os.path.basename(row["saved_frame_path"])}')
        self.get_logger().info(' | '.join(parts))


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
