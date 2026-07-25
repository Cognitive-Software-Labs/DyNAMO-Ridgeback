#!/usr/bin/env python3

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import json
import os
import subprocess
import time
from typing import Any

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
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
    GROUND_TRUTH_TOPIC,
    MASK_GATE_DEFAULT,
    benchmark_display_name,
    benchmark_output_name,
    parse_estimators,
    parse_mask_gate,
    selected_camera_estimators,
    selected_mask_estimators,
    uses_camera_estimators,
    uses_lidar_estimators,
    uses_mask_estimators,
)
from ridgeback_autonomy.benchmarking.reduction import (
    choose_representative_event,
    compute_status_histogram,
    compute_trial_medians,
    format_status_tally,
    merge_status_histograms,
    usable_aligned_events,
)
from ridgeback_autonomy.benchmarking.association import GtPoint, assign_to_ground_truth
from ridgeback_autonomy.benchmarking.rendering import BenchmarkCollageRenderer
from ridgeback_autonomy.benchmarking.scenarios import Scene, load_scenarios
from ridgeback_autonomy.benchmarking.scoring import build_instance_estimate, score_scene
from ridgeback_autonomy.benchmarking.summary import (
    build_coverage_rows,
    build_summary_rows,
    write_coverage_csv,
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


def ground_truth_point_message(true_pose: dict[str, float]) -> PointStamped:
    """Pack a trial's ground truth as x=lateral, y=forward, z=distance.

    The overlay's ``current_truth`` unpacks the same convention; the values are
    the base-frame planar measurement every benchmark row shares, not a 3D
    point in any TF frame.
    """

    msg = PointStamped()
    msg.point.x = float(true_pose['lateral_m'])
    msg.point.y = float(true_pose['forward_m'])
    msg.point.z = float(true_pose['distance_m'])
    return msg


# Robots and objects spawn with their model origin on the floor plane; each
# model bakes in its own vertical offset so origin-at-z=0 sits it on the ground.
G1_SPAWN_HEIGHT_M = 0.0
COMMAND_TIMEOUT_SEC = 10.0
COMMAND_RETRY_SLEEP_SEC = 0.5
STREAM_WAIT_TIMEOUT_SEC = 300.0
POSE_WAIT_TIMEOUT_SEC = 120.0
DELETE_TIMEOUT_SEC = 15.0
IMAGE_MATCH_TOLERANCE_NS = 250_000_000
CAMERA_MEASUREMENT_TOPIC = 'measurements/g1/camera'
LIDAR_MEASUREMENT_TOPIC = 'measurements/g1/lidar'
MASK_MEASUREMENT_TOPIC = 'measurements/g1/mask'
DEPTH_SOURCE_DEFAULT = 'stereoscopic'
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


@dataclass
class GtInstance:
    """Ground truth for one spawned robot in a scene (from the sim's true pose).

    The sim's true pose is used only here (and for the scoring assignment); it
    never feeds the estimators, which are sensor-only.
    """

    index: int
    model_name: str
    world_x: float
    world_y: float
    forward_m: float
    lateral_m: float
    distance_m: float


class G1DistanceBenchmarkRunner(Node):
    def __init__(self) -> None:
        super().__init__('g1_distance_benchmark_runner')

        pkg_share = get_package_share_directory('ridgeback_autonomy')
        self.g1_model_sdf = os.path.join(pkg_share, 'sim', 'models', 'g1', 'model.sdf')
        self.models_dir = os.path.join(pkg_share, 'sim', 'models')
        default_scenario = os.path.join(pkg_share, 'config', 'benchmark_scenarios.yaml')
        workspace_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
        default_output_dir = os.path.join(workspace_root, 'benchmark-results')

        self.declare_parameter('world', 'g1_distance_calibration')
        self.declare_parameter('scenario', '')
        self.declare_parameter('repeats', 5)
        self.declare_parameter('output_dir', default_output_dir)
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('capture_sec', 10.0)
        self.declare_parameter('estimators', 'rgb,sensor_depth,depth_anything,pointcloud,lidar')
        self.declare_parameter('camera_measurement_topic', CAMERA_MEASUREMENT_TOPIC)
        self.declare_parameter('lidar_measurement_topic', LIDAR_MEASUREMENT_TOPIC)
        self.declare_parameter('mask_measurement_topic', MASK_MEASUREMENT_TOPIC)
        # The config axes of the mask rows, stamped into their output names so
        # projective_ranging / euclidean_reconstruction runs are
        # self-describing (must match the values passed to the aligned depth +
        # mask nodes for the same run).
        self.declare_parameter('depth_source', DEPTH_SOURCE_DEFAULT)
        self.declare_parameter('isolation_2d', ISOLATION_2D_DEFAULT)
        self.declare_parameter('isolation_3d', ISOLATION_3D_DEFAULT)
        self.declare_parameter('mask_gate', MASK_GATE_DEFAULT)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('mono_depth_debug_topic', MONO_DEPTH_DEBUG_TOPIC)
        self.declare_parameter('depth_max_meters', DEPTH_MAX_METERS_DEFAULT)

        self.world = str(self.get_parameter('world').value)
        scenario_param = str(self.get_parameter('scenario').value).strip()
        self.scenario_path = (
            os.path.abspath(os.path.expanduser(scenario_param))
            if scenario_param else default_scenario
        )
        self.scenes = load_scenarios(self.scenario_path)
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
        self.mask_gate = parse_mask_gate(str(self.get_parameter('mask_gate').value))
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

        # Self-describing names: the underscore ``output`` form (gate, source,
        # path, isolation) names the CSV file; the spaced ``display`` prose
        # (gate, source, path) goes in logs and the summary column.
        self.estimator_output_names = {
            estimator: benchmark_output_name(
                estimator, self.depth_source, self.isolation_2d, self.isolation_3d,
                self.mask_gate)
            for estimator in self.selected_estimators
        }
        self.estimator_display_names = {
            estimator: benchmark_display_name(estimator, self.depth_source, self.mask_gate)
            for estimator in self.selected_estimators
        }
        self.estimator_csv_paths = {
            estimator: os.path.join(
                self.run_output_dir, f'{self.estimator_output_names[estimator]}.csv')
            for estimator in self.selected_estimators
        }
        self.summary_csv_path = os.path.join(self.run_output_dir, 'comparison_summary.csv')
        self.coverage_csv_path = os.path.join(self.run_output_dir, 'coverage.csv')
        # Per-estimator status-code tallies over every captured event across the
        # whole run (misses included, unlike the usable-aligned rows).
        self.status_aggregate: dict[str, dict[int, int]] = {}

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

        # Ground truth for the overlay's reference line, republished at ~1 Hz
        # while a capture window is active so the overlay's age gate drops the
        # line between trials.
        self.ground_truth_pub = self.create_publisher(
            PointStamped, GROUND_TRUTH_TOPIC, 10)
        self.active_truth: dict[str, float] | None = None
        self.last_truth_publish_monotonic = 0.0

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
            f'for {", ".join(self.estimator_display_names[est] for est in self.selected_estimators)} '
            f'in world "{self.world}" for robot "{self.robot_model_name}".'
        )
        self.wait_for_required_streams()
        self.wait_for_entity_pose(self.robot_model_name, POSE_WAIT_TIMEOUT_SEC)

        estimator_rows = {estimator: [] for estimator in self.selected_estimators}
        included_trials = 0
        skipped_trials = 0
        total_missed_instances = 0
        total_extra_detections = 0

        for trial in self.build_trials():
            result = self.run_trial(trial)
            if result is None:
                skipped_trials += 1
                continue

            included_trials += 1
            for estimator, instance_rows in result['rows'].items():
                estimator_rows[estimator].extend(instance_rows)
            total_missed_instances += result['missed_count']
            total_extra_detections += result['extra_count']

        for estimator in self.selected_estimators:
            write_trial_csv(self.estimator_csv_paths[estimator], estimator_rows[estimator])

        summary_rows = build_summary_rows(
            estimator_rows, total_missed_instances, total_extra_detections,
            self.status_aggregate)
        for row in summary_rows:
            row['estimator'] = self.estimator_display_names.get(
                row['estimator'], row['estimator'])
        write_summary_csv(self.summary_csv_path, summary_rows)

        coverage_rows = build_coverage_rows(self.status_aggregate)
        for row in coverage_rows:
            row['estimator'] = self.estimator_display_names.get(
                row['estimator'], row['estimator'])
        write_coverage_csv(self.coverage_csv_path, coverage_rows)

        self.log_summary(summary_rows, included_trials, skipped_trials)
        self.get_logger().info(f'Benchmark run written to {self.run_output_dir}')

    def build_trials(self) -> list[dict[str, Any]]:
        trials: list[dict[str, Any]] = []
        for scene in self.scenes:
            effective_repeats = scene.repeats_override or self.repeats
            for repeat_index in range(effective_repeats):
                trial_id = (
                    scene.id if effective_repeats == 1
                    else f'{scene.id}_rep{repeat_index + 1}'
                )
                trials.append({
                    'trial_id': trial_id,
                    'repeat_index': repeat_index + 1,
                    'scene': scene,
                })
        return trials

    def run_trial(self, trial: dict[str, Any]) -> dict[str, dict[str, Any]] | None:
        trial_id = trial['trial_id']
        scene: Scene = trial['scene']
        entity_prefix = f'bench_{self.run_label}_{trial_id}'

        spawned_names: list[str] = []
        try:
            robot_models = self.spawn_scene(scene, entity_prefix, spawned_names)
            self.spin_for(self.settle_sec)

            gt_instances = self.compute_scene_ground_truth(robot_models)
            # Overlay truth line is a single reference: republish the nearest
            # instance's planar truth (multi-robot scenes pick the closest).
            nearest = min(gt_instances, key=lambda gt: gt.distance_m)
            self.active_truth = {
                'lateral_m': nearest.lateral_m,
                'forward_m': nearest.forward_m,
                'distance_m': nearest.distance_m,
            }
            capture = self.capture_measurement_window(self.capture_sec)
            merge_status_histograms(self.status_aggregate, capture['status_histogram'])
            usable_events = capture['usable_events']
            if not usable_events:
                self.get_logger().info(
                    f'{trial_id} skipped | no_common_usable_events | '
                    f'raw_events={capture["total_events"]} | '
                    f'{format_status_tally(capture["status_histogram"], self.selected_estimators)}'
                )
                return None

            # Frame-level scalar medians drive the representative-frame choice
            # and the collage (always finite for the selected estimators). The
            # per-instance medians below drive the scored rows.
            scalar_medians = compute_trial_medians(usable_events, self.selected_estimators)
            if len(gt_instances) == 1:
                # Single robot: the per-instance medians ARE the scalar medians,
                # so a one-robot scene reproduces the historical grid benchmark.
                instance_medians = {gt_instances[0].index: scalar_medians}
                missed_gt: tuple[int, ...] = ()
                extra_count = 0
            else:
                instance_medians, missed_gt, extra_count = score_scene(
                    usable_events, gt_instances, self.selected_estimators)

            representative_event = choose_representative_event(
                usable_events,
                self.selected_estimators,
                scalar_medians,
            )
            box_annotations = self.build_box_annotations(representative_event, gt_instances)
            image_path = self.save_trial_collage(
                trial_id,
                representative_event,
                scalar_medians,
                gt_instances[0].distance_m,
                box_annotations,
                len(missed_gt),
            )

            rows: dict[str, list[dict[str, Any]]] = {
                estimator: [] for estimator in self.selected_estimators}
            for gt in gt_instances:
                medians = instance_medians.get(gt.index, {})
                for estimator in self.selected_estimators:
                    estimate = medians.get(estimator)
                    if estimate is None:
                        continue  # this instance had no finite value for this estimator
                    abs_error = abs(estimate - gt.distance_m)
                    rel_error = abs_error / gt.distance_m if gt.distance_m > 0.0 else None
                    rows[estimator].append({
                        'trial_id': trial_id,
                        'repeat_index': trial['repeat_index'],
                        'scene_id': scene.id,
                        'instance_index': gt.index,
                        'spawn_forward_m': gt.world_x,
                        'spawn_lateral_m': gt.world_y,
                        'spawn_world_x': gt.world_x,
                        'spawn_world_y': gt.world_y,
                        'spawn_yaw_rad': scene.robots[gt.index].yaw,
                        'true_forward_m': gt.forward_m,
                        'true_lateral_m': gt.lateral_m,
                        'true_distance_m': gt.distance_m,
                        'estimator': self.estimator_display_names[estimator],
                        'trial_estimate_m': estimate,
                        'abs_error_m': abs_error,
                        'rel_error': rel_error,
                        'usable_aligned_events': len(usable_events),
                        'image_path': image_path,
                    })

            self.log_trial(
                trial_id, scalar_medians, gt_instances[0].distance_m,
                len(usable_events), image_path, len(missed_gt), extra_count)
            return {'rows': rows, 'missed_count': len(missed_gt), 'extra_count': extra_count}
        except Exception as exc:
            self.get_logger().error(f'{trial_id} failed: {exc}')
            return None
        finally:
            for model_name in spawned_names:
                try:
                    self.despawn_model(model_name)
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
            self.active_truth = None

        usable_events = usable_aligned_events(self.capture_events, self.selected_estimators)
        return {
            'total_events': len(self.capture_events),
            'usable_events': usable_events,
            'status_histogram': compute_status_histogram(
                self.capture_events, self.selected_estimators),
        }

    def build_box_annotations(
        self,
        representative_event,
        gt_instances: list[GtInstance],
    ) -> list[dict | None]:
        """Map each box in the representative frame to its ground-truth instance.

        Sensor-only association on the one displayed frame, purely for the
        collage labels (``#i e<est>/t<true>``). Unmatched boxes stay ``None`` and
        render as ``extra``.
        """

        instances = [
            build_instance_estimate(detection, index, self.selected_estimators)
            for index, detection in enumerate(representative_event.detections)
        ]
        gt_points = [
            GtPoint(index=gt.index, forward_m=gt.forward_m, lateral_m=gt.lateral_m,
                    distance_m=gt.distance_m)
            for gt in gt_instances
        ]
        assignment = assign_to_ground_truth(instances, gt_points)
        true_by_gt = {gt.index: gt.distance_m for gt in gt_instances}
        annotations: list[dict | None] = [None] * len(representative_event.detections)
        for gt_index, det_index in assignment.matches:
            annotations[det_index] = {
                'instance_index': gt_index,
                'true_distance_m': true_by_gt[gt_index],
            }
        return annotations

    def save_trial_collage(
        self,
        trial_id: str,
        representative_event,
        trial_medians: dict[str, float],
        true_distance_m: float,
        box_annotations: list[dict | None] | None = None,
        missed_count: int = 0,
    ) -> str:
        collage = self.collage_renderer.render_trial_collage(
            trial_id,
            representative_event,
            self.selected_estimators,
            trial_medians,
            true_distance_m,
            box_annotations,
            missed_count,
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

    def compute_scene_ground_truth(
        self,
        robot_models: list[tuple[int, str, Any]],
    ) -> list[GtInstance]:
        """Ground truth per spawned robot, from one pose snapshot.

        Truth is used here only to produce reference distances (and later the
        scoring assignment); it never reaches the estimators.
        """

        snapshot = self.get_pose_snapshot()
        robot_pose = snapshot.get(self.robot_model_name)
        if robot_pose is None:
            raise RuntimeError(
                f'Robot pose "{self.robot_model_name}" not present on {self.pose_info_topic}')
        robot_yaw = yaw_from_quaternion(
            robot_pose['orientation']['x'],
            robot_pose['orientation']['y'],
            robot_pose['orientation']['z'],
            robot_pose['orientation']['w'],
        )

        instances: list[GtInstance] = []
        for index, model_name, _robot in robot_models:
            target_pose = snapshot.get(model_name)
            if target_pose is None:
                raise RuntimeError(
                    f'Target pose "{model_name}" not present on {self.pose_info_topic}')
            dx_world = target_pose['position']['x'] - robot_pose['position']['x']
            dy_world = target_pose['position']['y'] - robot_pose['position']['y']
            forward_m, lateral_m, distance_m = planar_distance_from_vehicle_origin(
                dx_world,
                dy_world,
                robot_yaw,
            )
            instances.append(GtInstance(
                index=index,
                model_name=model_name,
                world_x=target_pose['position']['x'],
                world_y=target_pose['position']['y'],
                forward_m=forward_m,
                lateral_m=lateral_m,
                distance_m=distance_m,
            ))
        return instances

    def object_model_sdf(self, model: str) -> str:
        path = os.path.join(self.models_dir, model, 'model.sdf')
        if not os.path.isfile(path):
            raise RuntimeError(f'Object model "{model}" has no model.sdf at {path}')
        return path

    def spawn_scene(
        self,
        scene: Scene,
        entity_prefix: str,
        spawned_names: list[str],
    ) -> list[tuple[int, str, Any]]:
        """Spawn every robot + object in a scene; record names for teardown.

        Names are appended to ``spawned_names`` before the pose wait so a spawn
        that never settles is still despawned by the caller's ``finally``.
        Returns ``(index, model_name, RobotSpec)`` per robot for ground truth.
        """

        robot_models: list[tuple[int, str, Any]] = []
        for index, robot in enumerate(scene.robots):
            model_name = f'{entity_prefix}_g1_{index}'
            spawned_names.append(model_name)
            self.spawn_model(
                self.g1_model_sdf, model_name, robot.x, robot.y, G1_SPAWN_HEIGHT_M, robot.yaw)
            self.wait_for_entity_pose(model_name, POSE_WAIT_TIMEOUT_SEC)
            robot_models.append((index, model_name, robot))

        for index, obj in enumerate(scene.objects):
            model_name = f'{entity_prefix}_obj_{index}'
            spawned_names.append(model_name)
            self.spawn_model(
                self.object_model_sdf(obj.model), model_name, obj.x, obj.y,
                G1_SPAWN_HEIGHT_M, obj.yaw)
            self.wait_for_entity_pose(model_name, POSE_WAIT_TIMEOUT_SEC)

        return robot_models

    def spawn_model(
        self,
        model_ref: str,
        model_name: str,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
    ) -> None:
        command = [
            '/opt/ros/jazzy/lib/ros_gz_sim/create',
            '-world', self.world,
            '-file', model_ref,
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

    def despawn_model(self, model_name: str) -> None:
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
            self.publish_active_truth()
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

    def publish_active_truth(self) -> None:
        """Republish the trial ground truth at ~1 Hz during capture."""

        if not self.capture_active or self.active_truth is None:
            return
        now = time.monotonic()
        if now - self.last_truth_publish_monotonic < 1.0:
            return
        self.last_truth_publish_monotonic = now
        msg = ground_truth_point_message(self.active_truth)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.ground_truth_pub.publish(msg)

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
        missed_instances: int = 0,
        extra_detections: int = 0,
    ) -> None:
        parts = [
            trial_id,
            f'true={true_distance_m:.3f}m',
            f'usable_events={usable_events}',
            f'image={os.path.basename(image_path)}',
        ]
        if missed_instances or extra_detections:
            parts.append(f'missed={missed_instances}')
            parts.append(f'extra={extra_detections}')
        for estimator in self.selected_estimators:
            value = trial_medians.get(estimator)
            parts.append(
                f'{estimator}={value:.3f}m' if value is not None else f'{estimator}=NA')
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
            # row['estimator'] is already the display prose (set above).
            self.get_logger().info(
                f'{row["estimator"]} | '
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
