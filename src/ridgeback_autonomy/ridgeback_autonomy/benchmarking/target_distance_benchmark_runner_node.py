#!/usr/bin/env python3

from __future__ import annotations

from collections import OrderedDict
import contextlib
import copy
import json
import math
import os
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters
from ridgeback_common.detector_config import (
    DETECTOR_ENVIRONMENT_KEYS, require_matching_detector_settings,
    resolve_detector_settings,
)
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from rclpy.time import Time

from ridgeback_autonomy.benchmarking.alignment import (
    attach_exact_preview,
    ensure_measurement_event,
    store_buffered_preview,
    update_measurement_event,
)
from ridgeback_common.camera_profiles import DEFAULT_CAMERA_PROFILE
from ridgeback_common.ros_signals import init_owned, shielded, spin_step
from ridgeback_localization.estimator_registry import (
    MASK_GATE_DEFAULT,
    parse_estimators,
    parse_mask_gate,
    selected_mask_estimators,
    selected_pointcloud_estimators,
    uses_mask_estimators,
    uses_pointcloud_estimators,
)
from ridgeback_localization.contracts import (
    GROUND_TRUTH_TOPIC,
    MASK_MEASUREMENTS_TOPIC,
    POINTCLOUD_MEASUREMENTS_TOPIC,
)
from ridgeback_autonomy.benchmarking.naming import (
    benchmark_display_name,
    benchmark_output_name,
    benchmark_run_folder_name,
)
from ridgeback_autonomy.benchmarking.paths import (
    default_output_directory,
)
from ridgeback_autonomy.benchmarking.reduction import (
    choose_representative_event,
    compute_trial_medians,
    format_status_tally,
    merge_status_histograms,
    restrict_events_to_stamps,
    summarize_capture_events,
)
from ridgeback_autonomy.benchmarking.rendering import BenchmarkCollageRenderer
from ridgeback_autonomy.benchmarking.recording import (
    DEFAULT_FPS,
    DEFAULT_MAX_SECONDS,
    ScreenRecorder,
    find_window_id,
)
from ridgeback_autonomy.benchmarking.replay import (
    REPLAY_CAPTURE_BATCHES_DEFAULT,
    ReplayDatasetWriter,
    sha256_file,
)
from ridgeback_autonomy.benchmarking.replay_artifacts import (
    SensorCaptureWriter,
    dependency_versions,
    sha256_file as artifact_sha256_file,
)
from ridgeback_autonomy.benchmarking.sampled_capture import (
    SAMPLED_REFERENCE_DIRNAME,
    SAMPLED_WINDOWS_NAME,
    SAMPLED_WINDOWS_SCHEMA,
    SENSOR_CAPTURE_MODE_QUOTA,
    SENSOR_CAPTURE_MODE_SAMPLED_WINDOW,
    live_measurement_record,
    live_record_expects_depth,
    live_record_expects_scan,
    parse_sensor_capture_mode,
)
from ridgeback_autonomy.benchmarking.process_utils import (
    format_commit,
    git_provenance,
)
from ridgeback_autonomy.benchmarking.report import render_run_report
from ridgeback_autonomy.benchmarking.scenarios import (
    Scene, load_scenarios, parse_scene_ids, select_scenes,
)
from ridgeback_autonomy.benchmarking.background import resolve_scene_background
from ridgeback_autonomy.benchmarking.background_session import background_scene
from ridgeback_autonomy.benchmarking.scene_state import (
    ExpectedPose, SceneStateMismatch, check_scene_state,
)
from ridgeback_autonomy.benchmarking.asset_catalog import (
    DEFAULT_TARGET,
    load_asset_catalog,
    scene_object_models,
)
from ridgeback_autonomy.benchmarking.backends import (
    DEFAULT_BACKEND,
    benchmark_backend,
    decode_backend_settings,
    simulator_provenance,
)
from ridgeback_autonomy.benchmarking.scene_control import (
    BENCHMARK_ENTITY_PREFIX,
    POSE_TOLERANCE_M,
    YAW_TOLERANCE_RAD,
    SpawnSpec,
    create_scene_control_client,
    planar_pose,
    yaw_difference,
)
from ridgeback_autonomy.benchmarking.scoring import (
    MISS_OUTCOMES,
    score_scene,
)
from ridgeback_autonomy.benchmarking.simulation import (
    GroundTruthInstance,
    compute_ground_truth_instances,
)
from ridgeback_autonomy.benchmarking.summary import (
    build_run_document,
    build_summary_rows,
    write_run_json,
    write_trial_csv,
)
from ridgeback_autonomy.benchmarking.trial_results import (
    build_box_annotations,
    build_trial_result,
    build_trials,
    dominant_miss_reasons,
)
from ridgeback_interfaces.msg import PolarBeams, TargetDetections, TargetMeasurements
from ridgeback_common.stamps import stamp_to_nanoseconds
from ridgeback_localization.core.depth_common import (
    DEPTH_GATE_DISABLED,
    NEAREST_MODE_BIN_WIDTH_M_DEFAULT,
    NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT,
)
from ridgeback_localization.core.image_utils import (
    convert_color_image_message,
    decode_color_to_rgb,
)
from ridgeback_localization.core.depth_sources import decode_depth_to_meters
from ridgeback_localization.core.intrinsics import intrinsics_from_camera_info
from ridgeback_localization.core.mask import clamp_box
from ridgeback_common.messages import batch_from_detections_message
from ridgeback_common.tf_utils import lookup_transform_components
from ridgeback_localization.contracts import (
    ALIGNED_DEPTH_DEBUG_TOPIC,
    ALIGNED_DEPTH_SOURCE_DEBUG_TOPIC,
    POLAR_BEAMS_TOPIC,
    RAW_DETECTIONS_TOPIC,
)
from ridgeback_localization.synchronization import (
    COLOR_BUFFER_DEPTH_DEFAULT,
    SCAN_MATCH_BUFFER_DEPTH,
    SCAN_MATCH_TOLERANCE_S_DEFAULT,
    StampedMessageBuffer,
)
from ridgeback_localization.core.isolation_2d import (
    ISOLATION_2D_DEFAULT,
    NEAREST_MODE_BAND_M_DEFAULT,
)
from ridgeback_localization.core.isolation_3d import (
    BASE_ABOVE_FLOOR_M_DEFAULT,
    FLOOR_MARGIN_M_DEFAULT,
    ISOLATION_3D_DEFAULT,
)
from ridgeback_localization.core.ranging_defaults import (
    FRONT_PERCENTILE,
    INLIER_AHEAD_MARGIN_M,
    INLIER_BEHIND_MARGIN_M,
    MIN_VALID_SAMPLES,
)
from ridgeback_localization.core.vehicle_frame import (
    ROBOT_FRONT_OFFSET_M,
)
from ridgeback_localization.ground_truth import (
    ground_truth_point_message,
)


STREAM_WAIT_TIMEOUT_SEC = 300.0
# Republish period for the trial truth, well inside the consumers'
# ``TRUTH_MAX_AGE_S`` so the line survives a dropped message and is never much
# older than the scene it describes.
TRUTH_PUBLISH_PERIOD_SEC = 0.2
DEPTH_SOURCE_DEFAULT = 'stereoscopic'

# WM_CLASS of the window to record. RViz holds the whole picture once the
# perception overlay is published into it, so one window is the whole run.
RECORD_WINDOW_CLASS_DEFAULT = 'rviz'
# RViz starts alongside this node, so allow it a moment to map its window.
RECORD_WINDOW_WAIT_SEC = 20.0
# Declared by rclpy itself, not by the launch file, so it is not part of a
# run's configuration. ``use_sim_time`` is deliberately NOT excluded: the
# launch sets it and it changes how stamps are interpreted.
RCLPY_INTERNAL_PARAMETERS = frozenset({'start_type_description_service'})


def placement_dx(background: Any, scene: Scene) -> float:
    """How far along world x a scene is placed; 0 without a background."""

    return 0.0 if background is None else background.dx_m(scene.id)


def format_summary_log_row(row: dict[str, Any]) -> str:
    """Format one aggregate row without assuming it has accuracy metrics."""

    if row['trial_count'] == 0:
        return f'{row["estimator"]}: no comparable trials.'
    if row['scored_count'] == 0:
        return (
            f'{row["estimator"]} | n={row["trial_count"]} | '
            'scored=0 | no accuracy metrics.'
        )
    return (
        f'{row["estimator"]} | '
        f'n={row["trial_count"]} | '
        f'MAE={row["mean_abs_error_m"]:.3f}m | '
        f'MedianAE={row["median_abs_error_m"]:.3f}m | '
        f'P95AE={row["p95_abs_error_m"]:.3f}m | '
        f'MeanRel={row["mean_rel_error"]:.3f}'
    )


class TargetDistanceBenchmarkRunner(Node):
    def __init__(self) -> None:
        super().__init__('target_distance_benchmark_runner')

        pkg_share = get_package_share_directory('ridgeback_autonomy')
        default_scenario = os.path.join(pkg_share, 'config', 'benchmark_scenarios_full.yaml')
        # Kept on the node: the git provenance recorded with each run is read
        # from this directory, not just the default output path.
        self.workspace_root = os.path.abspath(
            os.path.join(pkg_share, '..', '..', '..', '..'))
        default_output_dir = default_output_directory(self.workspace_root)

        from ridgeback_common.detector_config import (
            DETECTOR_DEFAULT, DETECTION_MODEL_DEFAULT, DETECTION_THRESHOLD_DEFAULT,
            TARGET_LABELS_DEFAULT,
        )
        self.declare_parameter('detector', DETECTOR_DEFAULT)
        self.declare_parameter('detection_model', DETECTION_MODEL_DEFAULT)
        self.declare_parameter('target_labels', TARGET_LABELS_DEFAULT)
        self.declare_parameter('detection_threshold', DETECTION_THRESHOLD_DEFAULT)
        self.declare_parameter('detector_fps', 10.0)
        self.declare_parameter('detector_debug', False)
        self.declare_parameter('verify_detector_identity', False)
        self.declare_parameter('world', 'target_distance_calibration')
        self.declare_parameter('backend', DEFAULT_BACKEND)
        # Recorded, not applied: the persistent environment's backend-only
        # settings, as the launch encoded them.
        self.declare_parameter('backend_settings', '')
        self.declare_parameter('asset_catalog', '')
        self.declare_parameter('scene_control_service_namespace', '')
        self.declare_parameter('scenario', '')
        # A packaged background layout: each scene is placed by its dx and,
        # with background_props, captured among the layout's props; false is
        # the control arm, placed identically in the bare room. An empty
        # background keeps every scene at the origin in the bare room.
        self.declare_parameter('background', '')
        self.declare_parameter('background_props', True)
        # Comma-separated scene ids to run, in that order; empty runs every
        # scene of the scenario file in file order.
        self.declare_parameter('scene_ids', '')
        self.declare_parameter('repeats', 5)
        self.declare_parameter('output_dir', default_output_dir)
        self.declare_parameter('run_dir_name', '')
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('capture_sec', 10.0)
        self.declare_parameter('replay_dataset_dir', '')
        self.declare_parameter('sensor_capture_dir', '')
        self.declare_parameter('capture_batches', REPLAY_CAPTURE_BATCHES_DEFAULT)
        self.declare_parameter('capture_drain_sec', 2.0)
        self.declare_parameter('capture_timeout_sec', 30.0)
        # ``quota`` ends a sensor-capture window at capture_batches raw batches.
        # ``sampled_window`` keeps the window at capture_sec and freezes only
        # its first capture_batches raw batches; see sampled_capture.py.
        self.declare_parameter('sensor_capture_mode', SENSOR_CAPTURE_MODE_QUOTA)
        self.declare_parameter('raw_detections_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('aligned_depth_debug_topic', ALIGNED_DEPTH_DEBUG_TOPIC)
        self.declare_parameter(
            'aligned_depth_source_topic', ALIGNED_DEPTH_SOURCE_DEBUG_TOPIC)
        self.declare_parameter('polar_beams_topic', POLAR_BEAMS_TOPIC)
        self.declare_parameter('camera_info_topic', 'sensors/camera_0/color/camera_info')
        self.declare_parameter('camera_profile', DEFAULT_CAMERA_PROFILE)
        # The same scan the mask node measures polar profiling from, and the
        # same nearest-stamp tolerance it matches with: a capture that froze a
        # different beam set, or paired it by a different rule, could not be
        # compared against the live polar numbers it is meant to reproduce.
        self.declare_parameter('scan_topic', 'sensors/lidar2d_0/scan')
        self.declare_parameter('scan_match_tolerance_s', SCAN_MATCH_TOLERANCE_S_DEFAULT)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('estimators', 'all')
        self.declare_parameter('pointcloud_measurement_topic', POINTCLOUD_MEASUREMENTS_TOPIC)
        self.declare_parameter('mask_measurement_topic', MASK_MEASUREMENTS_TOPIC)
        # The config axes of the mask rows, stamped into their output names so
        # projective_ranging / euclidean_reconstruction runs are
        # self-describing (must match the values passed to the aligned depth +
        # mask nodes for the same run).
        self.declare_parameter('depth_source', DEPTH_SOURCE_DEFAULT)
        self.declare_parameter('isolation_2d', ISOLATION_2D_DEFAULT)
        self.declare_parameter('isolation_3d', ISOLATION_3D_DEFAULT)
        self.declare_parameter('mask_gate', MASK_GATE_DEFAULT)
        self.declare_parameter('color_topic', 'sensors/camera_0/color/image')
        # Declared so declared_parameters() records the gate the mask rows ran
        # under. The runner itself never applies it.
        self.declare_parameter('mask_depth_max_meters', DEPTH_GATE_DISABLED)
        # Record-only for the same reason: a recipe sweep varies these per
        # config, and run.json is where the value a config actually ran at is
        # read back from. Both rows' recipe settings are recorded, so an
        # archived euclidean run is as reconstructable as a projective one.
        self.declare_parameter(
            'isolation_2d_bin_width_m', NEAREST_MODE_BIN_WIDTH_M_DEFAULT)
        self.declare_parameter('isolation_2d_band_m', NEAREST_MODE_BAND_M_DEFAULT)
        self.declare_parameter(
            'isolation_2d_min_bin_fraction', NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT)
        self.declare_parameter('min_valid_pixels', MIN_VALID_SAMPLES)
        self.declare_parameter(
            'isolation_3d_floor_margin_m', FLOOR_MARGIN_M_DEFAULT)
        self.declare_parameter('isolation_3d_percentile', FRONT_PERCENTILE)
        self.declare_parameter('isolation_3d_ahead_m', INLIER_AHEAD_MARGIN_M)
        self.declare_parameter('isolation_3d_behind_m', INLIER_BEHIND_MARGIN_M)
        self.declare_parameter(
            'isolation_3d_bin_width_m', NEAREST_MODE_BIN_WIDTH_M_DEFAULT)
        self.declare_parameter(
            'isolation_3d_min_bin_fraction', NEAREST_MODE_MIN_BIN_FRACTION_DEFAULT)

        # Screen recording of the RViz window for the length of the run. The
        # collages freeze one frame per trial; this keeps the motion around it.
        self.declare_parameter('record_video', True)
        self.declare_parameter('record_fps', DEFAULT_FPS)
        self.declare_parameter('record_max_sec', DEFAULT_MAX_SECONDS)
        self.declare_parameter('record_window_class', RECORD_WINDOW_CLASS_DEFAULT)

        # The configuration layer starts after the raw-detection readiness
        # gate. Query that live producer before creating any run output so a
        # direct split launch cannot record a checkpoint it did not run.
        requested_detector_settings = {
            name: self.get_parameter(name).value
            for name in DETECTOR_ENVIRONMENT_KEYS}
        self.detector_identity_verified = bool(
            self.get_parameter('verify_detector_identity').value)
        self.effective_detector_settings = (
            require_matching_detector_settings(
                requested_detector_settings, self.read_running_detector_settings())
            if self.detector_identity_verified else
            resolve_detector_settings(requested_detector_settings))
        self.world = str(self.get_parameter('world').value)
        self.backend = benchmark_backend(self.get_parameter('backend').value)
        self.backend_settings = decode_backend_settings(
            str(self.get_parameter('backend_settings').value))
        scenario_param = str(self.get_parameter('scenario').value).strip()
        self.scenario_path = (
            os.path.abspath(os.path.expanduser(scenario_param))
            if scenario_param else default_scenario
        )
        self.scene_ids = parse_scene_ids(str(self.get_parameter('scene_ids').value))
        self.scenes = select_scenes(load_scenarios(self.scenario_path), self.scene_ids)
        self.background_name = str(self.get_parameter('background').value).strip()
        self.background_props = bool(self.get_parameter('background_props').value)
        if not self.background_name and not self.background_props:
            raise ValueError(
                'background_props:=false is the control arm of a background; '
                'name the background it is placed by.')
        self.scene_background = None
        if self.background_name:
            self.scene_background = resolve_scene_background(
                self.background_name,
                os.path.join(pkg_share, 'config'),
                self.scenario_path,
                [scene.id for scene in self.scenes],
            )
            if self.scene_background.layout.world != self.world:
                raise ValueError(
                    f'Background "{self.background_name}" furnishes world '
                    f'"{self.scene_background.layout.world}", not "{self.world}".')
        self.background_session = None
        self.background_spawn_sec: float | None = None
        self.background_adjustments: list[dict[str, Any]] = []
        self.robot_home_record: dict[str, Any] = {}
        self.asset_catalog_path = os.path.abspath(os.path.expanduser(
            str(self.get_parameter('asset_catalog').value).strip()))
        self.scene_control_service_namespace = str(
            self.get_parameter('scene_control_service_namespace').value).strip()
        catalog = load_asset_catalog(self.asset_catalog_path)
        self.asset_selection = catalog.select(
            self.world, DEFAULT_TARGET, scene_object_models(self.scenes),
            self.scene_background.layout.models
            if self.scene_background and self.background_props else ())
        self.namespace_name = self.get_namespace().strip('/')
        self.robot_model_name = (
            f'{self.namespace_name}/robot' if self.namespace_name else 'robot'
        )
        # Only a placed run may move the robot; every other run leaves it
        # where the simulator put it.
        self.scene_control = create_scene_control_client(
            self, self.scene_control_service_namespace,
            robot_entity=self.robot_model_name if self.scene_background else None)
        self.repeats = int(self.get_parameter('repeats').value)
        self.output_dir = os.path.abspath(os.path.expanduser(str(self.get_parameter('output_dir').value)))
        self.run_dir_name = str(self.get_parameter('run_dir_name').value).strip()
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.capture_sec = float(self.get_parameter('capture_sec').value)
        self.replay_dataset_dir = str(self.get_parameter('replay_dataset_dir').value).strip()
        self.sensor_capture_dir = str(self.get_parameter('sensor_capture_dir').value).strip()
        self.capture_batches = int(self.get_parameter('capture_batches').value)
        self.capture_drain_sec = float(self.get_parameter('capture_drain_sec').value)
        self.capture_timeout_sec = float(self.get_parameter('capture_timeout_sec').value)
        self.sensor_capture_mode = parse_sensor_capture_mode(
            self.get_parameter('sensor_capture_mode').value)
        self.selected_estimators = parse_estimators(str(self.get_parameter('estimators').value))
        self.pointcloud_measurement_topic = str(
            self.get_parameter('pointcloud_measurement_topic').value)
        self.mask_measurement_topic = str(self.get_parameter('mask_measurement_topic').value)
        self.depth_source = str(self.get_parameter('depth_source').value)
        self.isolation_2d = str(self.get_parameter('isolation_2d').value)
        self.isolation_3d = str(self.get_parameter('isolation_3d').value)
        self.mask_gate = parse_mask_gate(str(self.get_parameter('mask_gate').value))
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.raw_detections_topic = str(self.get_parameter('raw_detections_topic').value)
        self.aligned_depth_debug_topic = str(self.get_parameter('aligned_depth_debug_topic').value)
        self.aligned_depth_source_topic = str(
            self.get_parameter('aligned_depth_source_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.scan_match_tolerance_s = float(
            self.get_parameter('scan_match_tolerance_s').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.record_video = bool(self.get_parameter('record_video').value)
        self.record_window_class = str(self.get_parameter('record_window_class').value)
        self.recorder = ScreenRecorder(
            fps=int(self.get_parameter('record_fps').value),
            max_seconds=int(self.get_parameter('record_max_sec').value),
            log=self.get_logger().warning,
        )

        self.run_started_at = time.localtime()
        self.run_started_monotonic = time.monotonic()
        self.run_label = time.strftime('%Y%m%d_%H%M%S', self.run_started_at)
        # The folder carries the axes that change the results, so a results
        # directory reads without opening anything.
        run_folder = self.run_dir_name or benchmark_run_folder_name(
            self.run_label,
            self.scenario_path,
            self.mask_gate,
            self.depth_source,
            self.selected_estimators,
            self.background_name,
            self.background_props,
        )
        self.run_output_dir = os.path.join(self.output_dir, run_folder)
        self.images_dir = os.path.join(self.run_output_dir, 'images')
        os.makedirs(self.images_dir, exist_ok=False)

        self.replay_writer: ReplayDatasetWriter | None = None
        self.sensor_writer: SensorCaptureWriter | None = None
        if self.replay_dataset_dir and self.sensor_capture_dir:
            raise ValueError(
                'replay_dataset_dir and sensor_capture_dir are mutually exclusive.')
        if (self.sensor_capture_mode == SENSOR_CAPTURE_MODE_SAMPLED_WINDOW
                and not self.sensor_capture_dir):
            raise ValueError(
                'sensor_capture_mode:=sampled_window samples into a typed sensor '
                'capture; set sensor_capture_dir.')
        if self.replay_dataset_dir:
            if self.capture_batches < 1:
                raise ValueError('capture_batches must be positive when replay_dataset_dir is set.')
            if self.selected_estimators != ('projective_ranging',):
                raise ValueError('Replay capture V1 requires estimators:=projective_ranging.')
            if self.mask_gate != 'box' or self.depth_source != 'stereoscopic':
                raise ValueError('Replay capture V1 requires mask_gate:=box and depth_source:=stereoscopic.')
            self.replay_writer = ReplayDatasetWriter(self.replay_dataset_dir, {
                'dataset_id': self.run_label,
                'scenario_path': self.scenario_path,
                'scenario_sha256': sha256_file(self.scenario_path),
                'background': self.background_identity(),
                'capture_parameters': self.declared_parameters(),
                'source': git_provenance(self.workspace_root),
            })
        elif self.sensor_capture_dir:
            if self.capture_batches < 1:
                raise ValueError(
                    'capture_batches must be positive when sensor_capture_dir is set.')
            if not any(
                estimator in {'projective_ranging', 'euclidean_reconstruction'}
                for estimator in self.selected_estimators
            ):
                raise ValueError(
                    'Sensor capture requires a depth-image measurement estimator '
                    'so exact aligned depth is published.')
            if (self.depth_source != 'stereoscopic'
                    and self.sensor_capture_mode != SENSOR_CAPTURE_MODE_SAMPLED_WINDOW):
                raise ValueError(
                    'Quota sensor capture requires stereoscopic depth; '
                    'sensor_capture_mode:=sampled_window records other sources '
                    'with the usable ceiling the live node declared.')
            source = git_provenance(self.workspace_root)
            self.sensor_writer = SensorCaptureWriter(
                self.sensor_capture_dir,
                producer={
                    **source,
                    'model': 'live-sensor-capture',
                    'model_revision': None,
                    'dependencies': dependency_versions(('numpy',)),
                    'parameters': self.declared_parameters(),
                },
                metadata={
                    'dataset_id': self.run_label,
                    'scenario_path': self.scenario_path,
                    'scenario_sha256': sha256_file(self.scenario_path),
                    'background': self.background_identity(),
                },
            )

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
        self.run_json_path = os.path.join(self.run_output_dir, 'run.json')
        self.report_path = os.path.join(self.run_output_dir, 'summary.md')
        self.video_path = os.path.join(self.run_output_dir, 'video', 'run.mp4')
        # Per-estimator status-code tallies over every captured box across the
        # whole run (misses included, unlike the usable-aligned rows). Feeds the
        # summary's observation columns.
        self.status_aggregate: dict[str, dict[int, int]] = {}

        self.collage_renderer = BenchmarkCollageRenderer()

        self.needs_pointcloud = uses_pointcloud_estimators(self.selected_estimators)
        self.needs_mask = uses_mask_estimators(self.selected_estimators)
        self.selected_pointcloud_estimators = set(
            selected_pointcloud_estimators(self.selected_estimators))
        self.selected_mask_estimators = set(selected_mask_estimators(self.selected_estimators))

        self.pointcloud_measurement_seen = False
        self.mask_measurement_seen = False
        self.color_stream_seen = False

        self.capture_active = False
        self.capture_events = {}
        self.color_preview_buffer: OrderedDict[int, Any] = OrderedDict()
        self.replay_capture_events: OrderedDict[int, dict[str, Any]] = OrderedDict()
        self.replay_depth_buffer: OrderedDict[int, np.ndarray] = OrderedDict()
        # Undecoded color frames, held outside capture windows too and decoded
        # only for the stamps a frozen batch claims. Same depth as the mask
        # node's color buffer, so a frame outlives the detector latency.
        self.sensor_color_buffer: OrderedDict[int, Image] = OrderedDict()
        self.replay_camera_info_buffer: OrderedDict[int, CameraInfo] = OrderedDict()
        self.static_replay_camera_info: CameraInfo | None = None
        # The live source's description of each published depth frame, keyed
        # by the frame's stamp like the depth itself.
        self.replay_depth_source_buffer: OrderedDict[int, dict] = OrderedDict()
        # The scan stamp each live polar measurement actually used, by batch.
        self.live_scan_stamp_buffer: OrderedDict[int, int] = OrderedDict()
        # Sampled-window state. The window is open while the full live
        # population is being measured; afterwards only the frozen sample may
        # still complete.
        self.live_window_open = False
        self.window_raw_batch_stamps: set[int] = set()
        self.sampled_live_events: dict = {}
        self.sampled_window_records: list[dict[str, Any]] = []
        self.sampled_reference_rows: dict[str, list[dict[str, Any]]] = {
            estimator: [] for estimator in self.selected_estimators}
        self.sampled_capture_summary: dict[str, Any] | None = None
        self.last_depth_source_warning = None
        # The scan free-runs on its own clock, so unlike RGB and depth it never
        # shares the detection stamp and cannot be exact-matched.
        self.sensor_scan_buffer: StampedMessageBuffer | None = None
        self.replay_tf_buffer: Buffer | None = None
        self.replay_tf_listener: TransformListener | None = None
        self.replay_last_fallback_frame: str | None = None
        self.replay_last_scan_fallback_frame: str | None = None

        self.last_color_decode_warning = None

        self.create_subscription(
            TargetMeasurements,
            self.pointcloud_measurement_topic,
            self.on_pointcloud_measurement,
            10,
        )
        self.create_subscription(
            TargetMeasurements,
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
        if self.capture_artifact_enabled():
            self.replay_tf_buffer = Buffer(node=self)
            self.replay_tf_listener = TransformListener(
                self.replay_tf_buffer, self, spin_thread=False)
            self.create_subscription(
                TargetDetections,
                self.raw_detections_topic,
                self.on_replay_raw_detections,
                10,
            )
            self.create_subscription(
                Image,
                self.aligned_depth_debug_topic,
                self.on_replay_aligned_depth,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.on_replay_camera_info,
                qos_profile_sensor_data,
            )
        if self.sensor_writer is not None:
            self.create_subscription(
                String,
                self.aligned_depth_source_topic,
                self.on_replay_depth_source,
                10,
            )
        if self.sampled_window_capture():
            # Replay must use the scan selected from the live worker's snapshot,
            # even when a closer scan arrives after that selection.
            self.create_subscription(
                PolarBeams,
                str(self.get_parameter('polar_beams_topic').value),
                self.on_live_polar_beams,
                10,
            )
        # Both capture formats carry a scan now: the sensor capture from payload
        # version 2, the legacy dataset from schema version 2. Gated on capture
        # at all rather than on which writer, since neither can record beams it
        # never subscribed to.
        if self.capture_artifact_enabled():
            self.sensor_scan_buffer = StampedMessageBuffer(SCAN_MATCH_BUFFER_DEPTH)
            self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.on_replay_scan,
                qos_profile_sensor_data,
            )

        # Ground truth for the display surfaces' reference line, republished
        # while a capture window is active so their age gates drop the line
        # between trials. Depth 1: only the newest value is ever wanted, and a
        # deeper queue lets a slow consumer read a previous trial's truth.
        self.ground_truth_pub = self.create_publisher(
            PointStamped, GROUND_TRUTH_TOPIC, 1)
        self.active_truth: dict[str, float] | None = None
        self.active_trial_id = ''
        self.last_truth_publish_monotonic = 0.0

    def on_pointcloud_measurement(self, msg: TargetMeasurements) -> None:
        self.pointcloud_measurement_seen = True
        if not self.capture_active:
            return

        event = ensure_measurement_event(self.capture_events, msg)
        update_measurement_event(event, msg, self.selected_pointcloud_estimators)
        attach_exact_preview(event, 'color', self.color_preview_buffer)

    def on_mask_measurement(self, msg: TargetMeasurements) -> None:
        self.mask_measurement_seen = True
        if not self.capture_active:
            return
        if self.sampled_window_capture():
            self.record_sampled_live_measurement(msg)
            # After the deadline the full window is frozen, as an uncaptured
            # run's is: a late result completes the sample, nothing else.
            if not self.live_window_open:
                return

        event = ensure_measurement_event(self.capture_events, msg)
        update_measurement_event(event, msg, self.selected_mask_estimators)
        attach_exact_preview(event, 'color', self.color_preview_buffer)

    def on_color_image(self, msg: Image) -> None:
        self.color_stream_seen = True
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        if getattr(self, 'sensor_writer', None) is not None:
            # Buffered outside capture windows too: a window's first raw batch
            # is usually for a frame that arrived before the window opened,
            # its detection landing only after the detector's latency.
            store_buffered_preview(
                self.sensor_color_buffer, stamp_ns, msg, limit=COLOR_BUFFER_DEPTH_DEFAULT)
            if self.capture_active:
                event = self.replay_capture_events.get(stamp_ns)
                if event is not None:
                    self.attach_replay_rgb(event, msg)
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
        store_buffered_preview(self.color_preview_buffer, stamp_ns, preview)
        self.backfill_previews_from_buffers()

    def attach_replay_rgb(self, event: dict[str, Any], msg: Image) -> None:
        """Decode one exact-stamp color frame into its frozen sensor event."""

        try:
            rgb = np.ascontiguousarray(decode_color_to_rgb(msg))
        except ValueError as exc:
            self.log_warning_once(
                'last_color_decode_warning',
                f'Failed to decode exact replay RGB from '
                f'"{self.resolved_topic(self.color_topic)}": {exc}',
            )
            return
        self.store_replay_rgb(event, rgb)

    def on_replay_camera_info(self, msg: CameraInfo) -> None:
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        if stamp_ns == 0:
            self.static_replay_camera_info = msg
            for event in self.replay_capture_events.values():
                if event['intrinsics'] is None:
                    self.store_replay_intrinsics(event, msg, stamp_ns=0)
            return
        store_buffered_preview(self.replay_camera_info_buffer, stamp_ns, msg)
        event = self.replay_capture_events.get(stamp_ns)
        if event is not None:
            self.store_replay_intrinsics(event, msg, stamp_ns=stamp_ns)

    def on_replay_scan(self, msg: LaserScan) -> None:
        """Buffer every scan; attach to frozen batches only inside a window.

        Buffered outside windows too, as the mask node does: the scan nearest a
        window's first raw batch usually arrived before the window opened, and
        without it that batch keeps no scan, or a farther one than the live
        polar row measured.
        """

        if self.sensor_scan_buffer is None:
            return
        self.sensor_scan_buffer.store(msg)
        if not self.capture_active:
            return
        for event in self.replay_capture_events.values():
            self.attach_replay_scan(event)

    def on_replay_raw_detections(self, msg: TargetDetections) -> None:
        """Freeze detector identity before a measurement recipe can change it."""

        if not self.capture_active:
            return
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        if self.sampled_window_capture():
            # The drain completes the sample; it never admits to it.
            if not self.live_window_open:
                return
            self.window_raw_batch_stamps.add(stamp_ns)
        if len(self.replay_capture_events) >= self.capture_batches:
            return
        if stamp_ns in self.replay_capture_events:
            return
        batch = batch_from_detections_message(msg)
        event = {
            'stamp_ns': stamp_ns,
            'frame_id': str(msg.header.frame_id),
            'detected': bool(msg.detected),
            'count': int(msg.count),
            'image_width': int(msg.image_width),
            'image_height': int(msg.image_height),
            'intrinsics': None,
            'intrinsics_stamp_ns': None,
            'camera_rotation': None,
            'camera_translation': None,
            'front_offset_m': ROBOT_FRONT_OFFSET_M,
            # Stereo depth has no source-defined finite usable ceiling.
            'depth_usable_max_m': None,
            # Polar profiling's inputs: the scan itself, and the extrinsic that
            # puts its beams in the frame the masks live in. Separate keys for
            # the same reason the camera's are separate from its intrinsics --
            # either can arrive without the other. Common to both writers: the
            # scan is a per-event channel, and neither format's other evidence
            # decides whether beams arrived.
            'scan': None,
            'scan_rotation': None,
            'scan_translation': None,
            'detections': [],
        }
        if getattr(self, 'sensor_writer', None) is not None:
            event['rgb'] = None
            event['depth_m'] = None
            # What the live node said produced this event's depth, and the
            # ceiling that came with it (``depth_usable_max_m`` above).
            event['depth_source'] = None
            if self.sampled_window_capture():
                event['live_scan_stamp_ns'] = None
            event['base_above_floor_m'] = BASE_ABOVE_FLOOR_M_DEFAULT
            event['detections'] = [
                {
                    'bbox_xyxy': list(detection.bbox_xyxy),
                    'label': detection.label,
                    'score': detection.score,
                }
                for detection in batch.detections
            ]
            color_msg = self.sensor_color_buffer.get(stamp_ns)
            if color_msg is not None:
                self.attach_replay_rgb(event, color_msg)
        else:
            event['detections'] = [
                {
                    'bbox_xyxy': list(detection.bbox_xyxy),
                    'label': detection.label,
                    'score': detection.score,
                    'depth_roi': None,
                }
                for detection in batch.detections
            ]
        self.replay_capture_events[stamp_ns] = event
        camera_info = (
            self.replay_camera_info_buffer.get(stamp_ns)
            or self.static_replay_camera_info
        )
        if camera_info is not None:
            camera_stamp_ns = stamp_to_nanoseconds(camera_info.header.stamp)
            self.store_replay_intrinsics(event, camera_info, stamp_ns=camera_stamp_ns)
        self.backfill_replay_event_context(event)
        depth_m = self.replay_depth_buffer.get(stamp_ns)
        if depth_m is not None:
            self.store_replay_depth(event, depth_m)
        source = getattr(self, 'replay_depth_source_buffer', {}).get(stamp_ns)
        if source is not None:
            self.store_replay_depth_source(event, source)
        if self.sampled_window_capture():
            live_scan = self.live_scan_stamp_buffer.get(stamp_ns)
            if live_scan is not None:
                event['live_scan_stamp_ns'] = live_scan
                self.attach_replay_scan(event)
            # A live result can beat its own raw batch here; the sample keeps
            # its own copy, since the full window's event may not change later.
            for live in self.capture_events.values():
                if live.stamp_ns == stamp_ns and live.key not in self.sampled_live_events:
                    self.sampled_live_events[live.key] = copy.deepcopy(live)
            if len(self.replay_capture_events) >= self.capture_batches:
                # Full: nothing outside the sample is worth holding any longer.
                self.replay_depth_buffer.clear()
                self.replay_depth_source_buffer.clear()
                self.live_scan_stamp_buffer.clear()

    def on_live_polar_beams(self, msg: PolarBeams) -> None:
        if not self.capture_active:
            return
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        scan_stamp_ns = stamp_to_nanoseconds(msg.scan_stamp)
        event = self.replay_capture_events.get(stamp_ns)
        if event is not None:
            event['live_scan_stamp_ns'] = scan_stamp_ns
            self.attach_replay_scan(event)
        elif self.sample_admission_open():
            store_buffered_preview(self.live_scan_stamp_buffer, stamp_ns, scan_stamp_ns)

    def sampled_window_capture(self) -> bool:
        return (getattr(self, 'sensor_capture_mode', SENSOR_CAPTURE_MODE_QUOTA)
                == SENSOR_CAPTURE_MODE_SAMPLED_WINDOW
                and getattr(self, 'sensor_writer', None) is not None)

    def sample_admission_open(self) -> bool:
        """Whether context for a not-yet-sampled stamp is still worth buffering."""

        if not self.sampled_window_capture():
            return True
        return (self.live_window_open
                and len(self.replay_capture_events) < self.capture_batches)

    def record_sampled_live_measurement(self, msg: TargetMeasurements) -> None:
        """Keep the live result of a sampled batch, during the window or its drain."""

        if stamp_to_nanoseconds(msg.header.stamp) not in self.replay_capture_events:
            return
        event = ensure_measurement_event(self.sampled_live_events, msg)
        update_measurement_event(event, msg, self.selected_mask_estimators)

    def on_replay_aligned_depth(self, msg: Image) -> None:
        if not self.capture_active:
            return
        stamp_ns = stamp_to_nanoseconds(msg.header.stamp)
        event = self.replay_capture_events.get(stamp_ns)
        if event is None and not self.sample_admission_open():
            return
        try:
            depth_m = decode_depth_to_meters(msg)
        except ValueError as exc:
            self.get_logger().warning(f'Replay capture skipped unusable aligned depth: {exc}')
            return
        if event is None:
            store_buffered_preview(self.replay_depth_buffer, stamp_ns, depth_m)
            return
        self.store_replay_depth(event, depth_m)

    def on_replay_depth_source(self, msg: String) -> None:
        if not self.capture_active:
            return
        try:
            record = json.loads(msg.data)
            stamp_ns = int(record['stamp_ns'])
        except (ValueError, KeyError, TypeError) as exc:
            self.log_warning_once(
                'last_depth_source_warning',
                f'Sensor capture skipped an unreadable depth-source record: {exc}')
            return
        event = self.replay_capture_events.get(stamp_ns)
        if event is None:
            if self.sample_admission_open():
                store_buffered_preview(self.replay_depth_source_buffer, stamp_ns, record)
            return
        self.store_replay_depth_source(event, record)

    def store_replay_depth_source(self, event: dict[str, Any], record: dict) -> None:
        """Attach the live source's own description of this event's depth.

        Refused, never repaired, when it does not describe this run's source:
        a frame frozen with another source's ceiling would be cleaned to the
        wrong range, and a monocular frame without a finite one would read as
        stereo's unbounded depth.
        """

        source = record.get('depth_source')
        ceiling = record.get('usable_max_m')
        if source != self.depth_source or (
                source != 'stereoscopic' and ceiling is None):
            self.log_warning_once(
                'last_depth_source_warning',
                f'Sensor capture refused depth-source record {source!r} with '
                f'ceiling {ceiling!r} for a {self.depth_source} run.')
            return
        event['depth_source'] = {
            key: value for key, value in record.items()
            if key not in ('stamp_ns', 'frame_id')}
        event['depth_usable_max_m'] = None if ceiling is None else float(ceiling)

    def store_replay_depth(self, event: dict[str, Any], depth_m: np.ndarray) -> None:
        if self.sensor_writer is not None:
            if depth_m.shape != (event['image_height'], event['image_width']):
                self.get_logger().warning(
                    f'Sensor capture depth grid {depth_m.shape} differs from detection grid '
                    f'({event["image_height"]}, {event["image_width"]}); recording no match.')
                return
            event['depth_m'] = np.array(depth_m, dtype=np.float32, copy=True)
            return
        self.store_replay_depth_rois(event, depth_m)

    def store_replay_rgb(self, event: dict[str, Any], rgb: np.ndarray) -> None:
        expected = (event['image_height'], event['image_width'], 3)
        if rgb.shape != expected:
            self.get_logger().warning(
                f'Sensor capture RGB grid {rgb.shape} differs from detection grid '
                f'{expected}; recording no match.')
            return
        event['rgb'] = np.array(rgb, dtype=np.uint8, copy=True)

    def store_replay_intrinsics(
        self,
        event: dict[str, Any],
        camera_info: CameraInfo,
        *,
        stamp_ns: int,
    ) -> None:
        camera = intrinsics_from_camera_info(camera_info)
        expected = (event['image_width'], event['image_height'])
        if (camera.width, camera.height) != expected:
            self.get_logger().warning(
                f'Sensor capture camera-info grid {(camera.width, camera.height)} '
                f'differs from detection grid {expected}; recording no match.')
            return
        event['intrinsics'] = {
            'fx': camera.fx, 'fy': camera.fy, 'cx': camera.cx, 'cy': camera.cy,
            'width': camera.width, 'height': camera.height,
        }
        event['intrinsics_stamp_ns'] = int(stamp_ns)

    def store_replay_depth_rois(self, event: dict[str, Any], depth_m: np.ndarray) -> None:
        """Attach one exact-stamp depth frame to its frozen detection ROIs."""

        if depth_m.shape != (event['image_height'], event['image_width']):
            self.get_logger().warning(
                f'Replay capture depth grid {depth_m.shape} differs from detection grid '
                f'({event["image_height"]}, {event["image_width"]}); recording no match.')
            return
        for detection in event['detections']:
            x1, y1, x2, y2 = clamp_box(
                tuple(detection['bbox_xyxy']), event['image_height'], event['image_width'])
            detection['depth_roi'] = np.array(depth_m[y1:y2, x1:x2], dtype=np.float32, copy=True)

    def attach_replay_scan(self, event: dict[str, Any]) -> None:
        """Freeze the actual live selection, or a provisional nearest scan.

        A sampled batch's PolarBeams message names the scan the worker had when
        it processed that batch. Later scans cannot improve that frozen choice.
        Quota captures and batches without a live choice retain nearest matching.
        """

        if self.sensor_scan_buffer is None:
            return
        stamp_ns = int(event['stamp_ns'])
        attached = event.get('scan')
        live_stamp = event.get('live_scan_stamp_ns')
        if live_stamp is not None:
            if attached is not None and int(attached['stamp_ns']) == live_stamp:
                self.backfill_replay_scan_extrinsic(event)
                return
            scan_msg = self.sensor_scan_buffer.lookup(
                Time(nanoseconds=int(live_stamp)).to_msg())
            # A missing selected scan is missing evidence, never another scan.
            event['scan'] = None
            event['scan_rotation'] = None
            event['scan_translation'] = None
        else:
            scan_msg = self.sensor_scan_buffer.lookup_nearest(
                Time(nanoseconds=stamp_ns).to_msg(), self.scan_match_tolerance_s)
        if scan_msg is None:
            return
        scan_stamp_ns = stamp_to_nanoseconds(scan_msg.header.stamp)
        if live_stamp is None and attached is not None and (
            abs(int(attached['stamp_ns']) - stamp_ns)
            <= abs(scan_stamp_ns - stamp_ns)
        ):
            return
        event['scan'] = {
            'ranges': np.asarray(scan_msg.ranges, dtype=np.float32),
            'angle_min': float(scan_msg.angle_min),
            'angle_max': float(scan_msg.angle_max),
            'angle_increment': float(scan_msg.angle_increment),
            'range_min': float(scan_msg.range_min),
            'range_max': float(scan_msg.range_max),
            'frame_id': str(scan_msg.header.frame_id),
            'stamp_ns': int(scan_stamp_ns),
        }
        self.backfill_replay_scan_extrinsic(event)

    def backfill_replay_scan_extrinsic(self, event: dict[str, Any]) -> None:
        """Freeze scan -> camera optical at the detection stamp, as live does.

        The mount is rigid, so this resolves as soon as TF is up and does not
        change afterwards; it is still looked up at the event's own stamp so a
        capture never depends on when the lookup happened to run.
        """

        scan = event.get('scan')
        if scan is None or self.replay_tf_buffer is None:
            return
        if (event['scan_rotation'] is not None
                and event['scan_translation'] is not None):
            return
        try:
            rotation, translation, fallback = lookup_transform_components(
                self.replay_tf_buffer,
                event['frame_id'],
                scan['frame_id'],
                Time(nanoseconds=int(event['stamp_ns'])),
                self.get_logger(),
                self.replay_last_scan_fallback_frame,
            )
        except TransformException:
            return
        self.replay_last_scan_fallback_frame = fallback
        event['scan_rotation'] = np.asarray(rotation, dtype=float).tolist()
        event['scan_translation'] = np.asarray(translation, dtype=float).tolist()

    def backfill_replay_event_context(self, event: dict[str, Any]) -> None:
        """Fill capture context that may arrive after the raw detector batch."""
        if self.sensor_scan_buffer is not None:
            self.attach_replay_scan(event)
            self.backfill_replay_scan_extrinsic(event)
        if (event['camera_rotation'] is not None
                and event['camera_translation'] is not None):
            return
        if self.replay_tf_buffer is None:
            return
        try:
            rotation, translation, fallback = lookup_transform_components(
                self.replay_tf_buffer,
                self.base_frame,
                event['frame_id'],
                Time(nanoseconds=int(event['stamp_ns'])),
                self.get_logger(),
                self.replay_last_fallback_frame,
            )
        except TransformException:
            return
        self.replay_last_fallback_frame = fallback
        event['camera_rotation'] = np.asarray(rotation, dtype=float).tolist()
        event['camera_translation'] = np.asarray(translation, dtype=float).tolist()

    def backfill_replay_capture_context(self) -> None:
        for event in self.replay_capture_events.values():
            self.backfill_replay_event_context(event)

    def replay_capture_is_complete(self) -> bool:
        """Whether every frozen batch has both replay input and a live result.

        A raw-detection callback can run before its aligned-depth callback on
        the ROS executor.  Counting raw batches alone therefore produces a
        replay file that looks complete but cannot reproduce the live score.
        Likewise, stopping on depth alone can omit the matching live measurement
        and make a parity comparison grade different event sets.
        """
        if len(self.replay_capture_events) < self.capture_batches:
            return False
        if not all(
            event['intrinsics'] is not None
            and event['camera_rotation'] is not None
            and event['camera_translation'] is not None
            for event in self.replay_capture_events.values()
        ):
            return False
        if getattr(self, 'sensor_writer', None) is not None:
            if not all(
                event.get('rgb') is not None and event.get('depth_m') is not None
                for event in self.replay_capture_events.values()
            ):
                return False
        elif not all(
                not event['detected']
                or all(detection['depth_roi'] is not None for detection in event['detections'])
                for event in self.replay_capture_events.values()):
            return False
        measurement_stamps = {
            event.stamp_ns for event in self.capture_events.values()
        }
        return set(self.replay_capture_events).issubset(measurement_stamps)

    def replay_scan_capture_is_complete(self) -> bool:
        """Whether every frozen batch carries a scan AND its extrinsic.

        Deliberately not part of ``replay_capture_is_complete``: a scan that
        does not land must not fail a capture the depth path can still use, and
        must not strand the mask caches parented to it. This only tells the
        drain there is still something worth waiting for.
        """

        if self.sensor_scan_buffer is None:
            return True
        return all(
            event.get('scan') is not None
            and event.get('scan_rotation') is not None
            and event.get('scan_translation') is not None
            for event in self.replay_capture_events.values()
        )

    def sampled_capture_is_complete(self) -> bool:
        """Whether every sampled batch has its inputs and its live result.

        Unlike the quota predicate, depth is awaited only where the live node
        published one: an empty batch never has depth, and neither has a frame
        whose live result says the depth source never ran.
        """

        live_stamps = {event.stamp_ns for event in self.sampled_live_events.values()}
        live_records = {
            event.stamp_ns: live_measurement_record(event, self.selected_estimators)
            for event in self.sampled_live_events.values()}
        for stamp_ns, event in self.replay_capture_events.items():
            if stamp_ns not in live_stamps:
                return False
            if (event['intrinsics'] is None or event['camera_rotation'] is None
                    or event['camera_translation'] is None or event.get('rgb') is None):
                return False
            if (event['detections']
                    and live_record_expects_depth(live_records.get(stamp_ns))
                    and (event.get('depth_m') is None
                         or event.get('depth_source') is None)):
                return False
            if live_record_expects_scan(live_records.get(stamp_ns)):
                chosen = event.get('live_scan_stamp_ns')
                if chosen is None or (event.get('scan') or {}).get('stamp_ns') != chosen:
                    return False
        return True

    def log_scan_capture_coverage(self) -> None:
        """Report how much of this trial's evidence polar profiling could use.

        Logged on every sensor-capture trial, complete or not: the scan is
        captured opportunistically, so its landing rate is a measurement to be
        taken, not a failure to be reported only when something else broke.
        """

        if self.sensor_scan_buffer is None:
            return
        events = self.replay_capture_events.values()
        with_scan = sum(event.get('scan') is not None for event in events)
        with_extrinsic = sum(
            event.get('scan_rotation') is not None
            and event.get('scan_translation') is not None
            for event in events)
        self.get_logger().info(
            f'Sensor capture scan coverage: {with_scan}/{len(events)} batch(es) '
            f'carry a scan, {with_extrinsic}/{len(events)} carry its extrinsic.')

    def log_code_provenance(self) -> None:
        """Say which code is about to produce these results, before it does.

        A dirty tree gets a banner rather than a line: a benchmark takes many
        minutes, and finding out afterwards that the results cannot be traced
        to a commit means running it again. A clean tree still reports its
        commit -- silence there would be indistinguishable from the check
        having failed to run.
        """

        provenance = git_provenance(self.workspace_root)
        commit = provenance['commit']
        branch = provenance['branch'] or 'unknown'
        dirty_count = provenance['dirty_count']

        if commit is None:
            self.get_logger().warn(
                'Code provenance unavailable (not a git checkout, or git is missing): '
                'this run cannot be traced back to a commit.')
            return

        if dirty_count:
            rule = '=' * 78
            self.get_logger().warn(
                f'\n{rule}\n'
                f'  UNCOMMITTED CHANGES: {dirty_count} file(s) differ from {commit} '
                f'({branch}).\n'
                f'  These results will NOT be reproducible from that commit alone.\n'
                f'  Commit before benchmarking if this run matters.\n'
                f'{rule}')
            return

        self.get_logger().info(f'Code provenance: {commit} on {branch} (clean tree).')

    def run(self) -> None:
        self.log_code_provenance()
        self.get_logger().info(
            'Starting multi-estimator distance benchmark '
            f'for {", ".join(self.estimator_display_names[est] for est in self.selected_estimators)} '
            f'in world "{self.world}" for robot "{self.robot_model_name}" '
            f'on backend {self.backend.name} '
            f'(native world "{self.asset_selection.world.native_name}"), '
            f'background {self.background_label()}.'
        )
        self.scene_control.wait_for_services(STREAM_WAIT_TIMEOUT_SEC)
        self.scene_control.require_capabilities(self.asset_selection.spawn_formats)
        self.get_logger().info(
            'Scene-control services are available; waiting for the simulation '
            'clock to advance before the first scene operation.')
        clock = self.scene_control.wait_for_advancing_clock(STREAM_WAIT_TIMEOUT_SEC)
        timeline = self.scene_control.readiness_timeline()
        self.get_logger().info(
            f'Simulation clock advancing at {clock.advanced_stamp_ns / 1e9:.3f} s, '
            f'{timeline["clock_advance_after_services_sec"]:.3f} s after the '
            'scene-control services appeared.')
        self.scene_control.wait_until_present([self.robot_model_name])
        self.refuse_leftover_entities()
        self.bring_robot_home()
        self.wait_for_required_streams()

        with self.background_context():
            # Started once the streams are up, so RViz is certain to exist by
            # now, and stopped in the finally below so an exception or a Ctrl-C
            # still leaves a playable file covering everything up to the
            # interruption.
            self.start_recording()
            try:
                self.run_trials()
            finally:
                self.stop_recording()
        self.log_readiness_timeline()

    def refuse_leftover_entities(self) -> None:
        """Refuse to start on top of another run's benchmark entities.

        A sweep reaps them between configurations, but a single run has no
        reaper, and leftover props or targets would stand in every scene.
        """

        leftovers = self.scene_control.find_benchmark_entities()
        if leftovers:
            shown = ', '.join(leftovers[:5]) + (', ...' if len(leftovers) > 5 else '')
            raise RuntimeError(
                f'{len(leftovers)} benchmark entities are already in the simulation '
                f'({shown}): an earlier run left them behind. Restart the simulator, '
                'or let a sweep reap them, before starting a run.')

    def bring_robot_home(self) -> None:
        """Start a placed run with the robot at home; report it elsewhere otherwise.

        An interrupted placed run can leave the robot at its last scene's
        placement. A placed run moves it back -- safe now, since no benchmark
        entity exists -- and any other run, which never moves the robot,
        warns that its scenes are not where the scenario file puts them.
        """

        snapshot = self.scene_control.get_scene_snapshot([self.robot_model_name])
        x, y, z, yaw = planar_pose(snapshot.poses[self.robot_model_name])
        at_home = (math.hypot(x, y) <= POSE_TOLERANCE_M
                   and yaw_difference(yaw, 0.0) <= YAW_TOLERANCE_RAD)
        record: dict[str, Any] = {
            'measured': {'x': x, 'y': y, 'z': z, 'yaw': yaw}, 'moved': False}
        if not at_home:
            where = f'({x:.3f}, {y:.3f}, yaw {yaw:.3f})'
            if self.scene_background is None:
                self.get_logger().warning(
                    f'The robot stands at {where}, not at home; this run does not '
                    'move it, so its scenes are offset by as much.')
            else:
                self.get_logger().warning(
                    f'The robot stands at {where}, not at home (an earlier run was '
                    'interrupted); moving it home.')
                record['move'] = self.scene_control.move_robot(0.0, 0.0, 0.0).as_record()
                record['moved'] = True
        self.robot_home_record = {'start': record}

    def return_robot_home(self, deadline: float | None) -> None:
        """End a placed run with the robot at home, after its props are gone.

        Under an interrupt only the budget left after the props' removal is
        spent; what it cannot finish, the next placed run's start repairs.
        """

        timeout = None if deadline is None else deadline - time.monotonic()
        if timeout is not None and timeout <= 0.0:
            self.get_logger().warning(
                'Robot left at its last placement: the interrupt budget is spent; '
                'the next placed run moves it home.')
            return
        try:
            move = self.scene_control.move_robot(0.0, 0.0, 0.0, timeout_sec=timeout)
        except Exception as exc:
            self.get_logger().warning(
                f'Robot not returned home ({exc}); the next placed run moves it home.')
            raise
        self.robot_home_record['end'] = move.as_record()

    def background_label(self) -> str:
        if self.scene_background is None:
            return 'none (bare room, robot at the origin)'
        if not self.background_props:
            return f'"{self.background_name}" control arm (no props, scenes placed)'
        return f'"{self.background_name}" ({len(self.scene_background.layout.props)} props, scenes placed)'

    def background_identity(self) -> dict[str, Any] | None:
        """The background a capture was taken in; None for the bare room."""

        if self.scene_background is None:
            return None
        return {
            'name': self.background_name,
            'props_spawned': self.background_props,
            'backdrop_gap_m': self.scene_background.layout.placement.backdrop_gap_m,
            'layout_sha256': self.scene_background.layout.sha256,
            'drops_sha256': self.scene_background.drops.sha256,
        }

    @contextlib.contextmanager
    def background_context(self):
        """Spawn the background for the run's trials, then remove it and return home."""

        if self.scene_background is None:
            yield
            return
        self.get_logger().info(f'Setting up background {self.background_label()}.')
        with background_scene(
            self.scene_control,
            self.scene_background,
            self.asset_selection.background,
            self.run_label,
            props=self.background_props,
            after_removal=self.return_robot_home,
        ) as session:
            self.background_session = session
            self.background_spawn_sec = session.spawn_sec
            self.get_logger().info(f'Background spawned in {session.spawn_sec:.1f} s.')
            try:
                yield
            finally:
                self.background_session = None
        self.get_logger().info('Background removed and robot returned home.')

    def adjust_background(self, trial: dict[str, Any]) -> None:
        """Remove this scene's drops, place the robot, restore the rest.

        Runs before the scene's objects and targets spawn. The robot is moved
        in every trial, even to the placement it already has, so every trial
        takes the same steps.
        """

        background = getattr(self, 'scene_background', None)
        session = getattr(self, 'background_session', None)
        if background is None or session is None:
            return
        scene_id = trial['scene'].id
        dx = background.dx_m(scene_id)
        adjustment = session.adjust(
            scene_id, move_robot=lambda: self.scene_control.move_robot(dx, 0.0, 0.0))
        record = {
            'trial_id': trial['trial_id'],
            'dx_m': dx,
            'anchor': background.anchor(scene_id).as_record(),
            **adjustment.as_record(),
        }
        self.background_adjustments.append(record)
        self.get_logger().info(
            f'{trial["trial_id"]} placed at dx {dx:.2f} m in {adjustment.move_sec:.2f} s; '
            f'background removed {", ".join(adjustment.removed) or "none"}, restored '
            f'{", ".join(adjustment.restored) or "none"} '
            f'in {adjustment.duration_sec:.2f} s.')

    def verify_scene_state(self, trial: dict[str, Any], specs: list[SpawnSpec]) -> None:
        """Refuse to capture a placed scene unless the simulation holds exactly it.

        One snapshot after settling: the benchmark entities present must be
        this scene's kept props, objects and targets, each at its commanded
        pose, with the robot at its placement. The result and the state's
        fingerprint join the trial's background record.
        """

        background = getattr(self, 'scene_background', None)
        if background is None:
            return
        scene: Scene = trial['scene']
        dx = background.dx_m(scene.id)
        entity_prefix = f'bench_{self.run_label}_{trial["trial_id"]}_'
        expected = {self.robot_model_name: ExpectedPose('robot', dx, 0.0, 0.0)}
        session = getattr(self, 'background_session', None)
        if self.background_props and session is not None:
            for prop in background.kept(scene.id):
                expected[f'{session.prefix}{prop.id}'] = ExpectedPose(
                    f'bg_{prop.id}', prop.x, prop.y, prop.yaw)
        for spec in specs:
            expected[spec.name] = ExpectedPose(
                spec.name.removeprefix(entity_prefix), spec.x, spec.y, spec.yaw)
        snapshot = self.scene_control.get_scene_snapshot(
            [self.robot_model_name], include_prefix=BENCHMARK_ENTITY_PREFIX)
        check = check_scene_state(
            snapshot.poses, expected, entity_prefix=BENCHMARK_ENTITY_PREFIX,
            position_tolerance_m=POSE_TOLERANCE_M, yaw_tolerance_rad=YAW_TOLERANCE_RAD)
        record = next(
            (record for record in reversed(self.background_adjustments)
             if record['trial_id'] == trial['trial_id']), None)
        if record is not None:
            record['state_check'] = check.as_record()
        if not check.ok:
            raise SceneStateMismatch(check)

    def background_provenance(self) -> dict[str, Any] | None:
        if self.scene_background is None:
            return None
        return {
            **self.scene_background.provenance(),
            'props_spawned': self.background_props,
            'spawn_sec': self.background_spawn_sec,
            'robot_home': dict(self.robot_home_record),
            'trials': list(self.background_adjustments),
        }

    def log_readiness_timeline(self) -> None:
        """One line separating simulator start-up delay from provider readiness."""

        timeline = self.scene_control.readiness_timeline()

        def wall(name: str) -> str:
            value = timeline[name]
            if value is None:
                return 'never'
            milliseconds = int(value % 1 * 1000)
            return f'{time.strftime("%H:%M:%S", time.localtime(value))}.{milliseconds:03d}'

        self.get_logger().info(
            f'Readiness: services {wall("services_available_wall")}, clock first seen '
            f'{wall("clock_first_message_wall")}, clock advancing '
            f'{wall("clock_advanced_wall")}, first spawn '
            f'{wall("first_spawn_requested_wall")}.')

    def run_trials(self) -> None:
        estimator_rows = {estimator: [] for estimator in self.selected_estimators}
        included_trials = 0
        skipped_trials = 0
        # Per-estimator tally of the four outcomes, so the summary can report
        # WHY an instance went unscored instead of pooling every failure mode.
        # Also the only miss counter: the summary derives its total from these.
        outcome_counts = {
            estimator: {outcome: 0 for outcome in MISS_OUTCOMES}
            for estimator in self.selected_estimators
        }
        total_extra_detections = 0

        for trial in build_trials(self.scenes, self.repeats):
            result = self.run_trial(trial)
            if result is None:
                skipped_trials += 1
                continue

            included_trials += 1
            for estimator, instance_rows in result['rows'].items():
                estimator_rows[estimator].extend(instance_rows)
            for estimator, counts in result['outcome_counts'].items():
                for outcome in MISS_OUTCOMES:
                    outcome_counts[estimator][outcome] += counts.get(outcome, 0)
            total_extra_detections += result['extra_count']

        for estimator in self.selected_estimators:
            write_trial_csv(self.estimator_csv_paths[estimator], estimator_rows[estimator])
        self.sampled_capture_summary = self.write_sampled_reference()

        summary_rows = build_summary_rows(
            estimator_rows, total_extra_detections, outcome_counts, self.status_aggregate)
        self.write_run_outputs(
            summary_rows, estimator_rows, included_trials, skipped_trials)

        if self.capture_artifact_enabled():
            replay_metadata = {
                'live_run_dir': self.run_output_dir,
                'trials_included': included_trials,
                'trials_skipped': skipped_trials,
                'capture_wall_time_sec': time.monotonic() - self.run_started_monotonic,
            }
            if self.sampled_capture_summary is not None:
                replay_metadata['sampled_reference'] = self.sampled_capture_summary
            writer = self.replay_writer or self.sensor_writer
            if skipped_trials:
                writer.mark_incomplete(**replay_metadata)
            else:
                writer.finalize(**replay_metadata)

        self.log_summary(summary_rows, included_trials, skipped_trials)
        self.get_logger().info(f'Benchmark run written to {self.run_output_dir}')

    def start_recording(self) -> None:
        """Begin recording the RViz window, if recording is on and it is there.

        Never raises and never blocks the run: a benchmark costs 40 minutes, and
        a screen recorder is not a reason to lose one.
        """

        if not self.record_video:
            return
        window_id = find_window_id(
            self.record_window_class,
            timeout_s=RECORD_WINDOW_WAIT_SEC,
            log=self.get_logger().warning,
        )
        if window_id is None:
            return
        self.recorder.start(window_id, self.video_path)

    def stop_recording(self) -> None:
        if self.recorder.active:
            self.recorder.stop()

    def read_running_detector_settings(self) -> dict:
        """Read the warm detector's declared startup identity over ROS."""
        namespace = self.get_namespace().rstrip('/')
        service = f'{namespace}/target_detector/get_parameters'
        client = self.create_client(GetParameters, service)
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f'Running detector parameters unavailable: {service}')
            names = sorted(DETECTOR_ENVIRONMENT_KEYS)
            request = GetParameters.Request()
            request.names = names
            future = client.call_async(request)
            # spin_step, like every other wait here, so an owned signal that
            # arrives during the query is raised after the step, not inside it.
            deadline = time.monotonic() + 10.0
            while not future.done() and rclpy.ok():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                spin_step(self, timeout_sec=min(0.1, remaining))
            if not future.done() or future.exception() is not None:
                raise RuntimeError(f'Cannot read running detector parameters: {service}')
            values = future.result().values
            if len(values) != len(names):
                raise RuntimeError(f'Incomplete running detector parameters: {service}')
            return {name: parameter_value_to_python(value)
                    for name, value in zip(names, values)}
        finally:
            self.destroy_client(client)

    def declared_parameters(self) -> dict[str, str]:
        """Every ROS parameter this node declared, as text.

        Read off the node rather than hand-listed, so the recorded
        configuration cannot drift as parameters are added. These are the
        launch arguments that reach the runner; launch-only toggles that do not
        affect measurement (rviz, overlay) never become parameters here.
        """

        parameters = {
            name: str(parameter.value)
            for name, parameter in sorted(self.get_parameters_by_prefix('').items())
            if name not in RCLPY_INTERNAL_PARAMETERS
        }
        if hasattr(self, 'effective_detector_settings'):
            parameters.update({name: str(value)
                               for name, value in self.effective_detector_settings.items()})
        return parameters

    def write_run_outputs(
        self,
        summary_rows: list[dict[str, Any]],
        estimator_rows: dict[str, list[dict[str, Any]]],
        included_trials: int,
        skipped_trials: int,
    ) -> None:
        """Both run-level views of the same numbers.

        ``summary.md`` is the readable one (see ``report.py``); ``run.json`` is
        the machine-readable one, carrying full precision plus the provenance
        and parameters that would otherwise exist only as prose.
        """

        provenance = git_provenance(self.workspace_root)
        parameters = self.declared_parameters()
        scenes = len({
            row['scene_id'] for rows in estimator_rows.values() for row in rows})
        instances = len({
            (row['trial_id'], row['instance_index'])
            for rows in estimator_rows.values() for row in rows
        })

        simulation = self.simulation_provenance()
        metadata = {
            'Started': time.strftime('%Y-%m-%d %H:%M:%S', self.run_started_at),
            'Commit': format_commit(provenance),
            'Branch': provenance['branch'] or 'unknown',
            'Scenario': self.scenario_path,
            'Target labels': self.effective_detector_settings['target_labels'],
            'Detector': self.effective_detector_settings['detector'],
            'Detection model': self.effective_detector_settings['detection_model'],
            'Detection threshold': self.effective_detector_settings['detection_threshold'],
            'Detector FPS': self.effective_detector_settings['detector_fps'],
            'Detector identity verified': self.detector_identity_verified,
            'Backend': self.backend.name,
            'World': (
                f'{self.world} (native {simulation["native_world"]})'
                if simulation['native_world'] != self.world else self.world),
            'Asset catalog': (
                f'{simulation["asset_catalog"]["schema"]} '
                f'sha256 {simulation["asset_catalog"]["sha256"]}'),
            'Background': self.background_label(),
        }
        # Named while the recorder is still running (it is stopped after the
        # report is written), so this is the path, not a claim the file is ready.
        if self.recorder.active:
            metadata['Video'] = os.path.relpath(self.video_path, self.run_output_dir)
        markdown = render_run_report(
            run_label=self.run_label,
            scenario_path=self.scenario_path,
            summary_rows=summary_rows,
            estimator_rows=estimator_rows,
            status_histograms=self.status_aggregate,
            display_names=self.estimator_display_names,
            included_trials=included_trials,
            skipped_trials=skipped_trials,
            scenes=scenes,
            instances=instances,
            metadata=metadata,
            parameters=parameters,
        )
        with open(self.report_path, 'w', encoding='utf-8') as report_file:
            report_file.write(markdown)

        # Raw fields, not the report's prose: ``uncommitted_files`` is a number
        # so "runs from a clean tree" is a filter, not a string match.
        write_run_json(self.run_json_path, build_run_document(
            summary_rows,
            run_metadata={
                'label': self.run_label,
                'started': time.strftime('%Y-%m-%d %H:%M:%S', self.run_started_at),
                'commit': provenance['commit'],
                'branch': provenance['branch'],
                'uncommitted_files': provenance['dirty_count'],
                'scenario': self.scenario_path,
                'target_labels': self.effective_detector_settings['target_labels'],
                'detector': self.effective_detector_settings['detector'],
                'detection_model': self.effective_detector_settings['detection_model'],
                'detection_threshold': self.effective_detector_settings['detection_threshold'],
                'detector_fps': self.effective_detector_settings['detector_fps'],
                'detector_identity_verified': self.detector_identity_verified,
                'backend': self.backend.name,
                'world': self.world,
                'native_world': simulation['native_world'],
                'simulation': simulation,
                'background': self.background_provenance(),
                'scenes': scenes,
                'instances': instances,
                'trials_included': included_trials,
                'trials_skipped': skipped_trials,
                **({'sampled_capture': self.sampled_capture_summary}
                   if getattr(self, 'sampled_capture_summary', None) else {}),
            },
            parameters=parameters,
            display_names=self.estimator_display_names,
            status_histograms=self.status_aggregate,
        ))

    def simulation_provenance(self) -> dict[str, Any]:
        """Which simulator, world, assets and settings produced this run."""

        capabilities = self.scene_control.capabilities
        return {
            'backend': self.backend.name,
            'public_world': self.world,
            'native_world': self.asset_selection.world.native_name,
            'settings': dict(self.backend_settings),
            'camera_profile': str(self.get_parameter('camera_profile').value),
            **simulator_provenance(self.backend),
            'scene_control_service_namespace': self.scene_control_service_namespace,
            'features': None if capabilities is None else {
                'features': sorted(capabilities.features),
                'spawn_formats': list(capabilities.spawn_formats),
                'custom_info': capabilities.custom_info,
            },
            'asset_catalog': self.asset_selection.provenance(),
            'readiness': self.scene_control.readiness_timeline(),
        }

    def run_trial(self, trial: dict[str, Any]) -> dict[str, dict[str, Any]] | None:
        trial_id = trial['trial_id']
        scene: Scene = trial['scene']
        entity_prefix = f'bench_{self.run_label}_{trial_id}'
        trial['placement_dx_m'] = placement_dx(getattr(self, 'scene_background', None), scene)

        specs, robot_models = self.scene_spawn_specs(scene, entity_prefix)
        try:
            # Remove this scene's drops, place the robot, restore the rest;
            # then objects before targets, settle, and check the room.
            self.adjust_background(trial)
            with self.scene_control.scene(specs):
                self.spin_for(self.settle_sec)
                self.verify_scene_state(trial, specs)

                gt_instances = self.compute_scene_ground_truth(robot_models)
                # Overlay truth line is a single reference: republish the nearest
                # instance's planar truth (multi-robot scenes pick the closest).
                nearest = min(gt_instances, key=lambda gt: gt.distance_m)
                self.active_truth = {
                    'lateral_m': nearest.lateral_m,
                    'forward_m': nearest.forward_m,
                    'distance_m': nearest.distance_m,
                }
                self.active_trial_id = trial_id
                capture = self.capture_measurement_window(self.capture_sec)
                if self.capture_artifact_enabled():
                    self.write_replay_trial(trial, gt_instances, capture['replay_events'])
                if 'sampled_window' in capture:
                    self.score_sampled_reference(trial, scene, gt_instances, capture)
                merge_status_histograms(self.status_aggregate, capture['status_histogram'])
                usable_by_estimator = capture['usable_by_estimator']
                usable_events = capture['usable_events']
                if not usable_events:
                    # Union empty: the detector never yielded a usable frame for ANY
                    # estimator. The trial ran and found nothing, so every GT
                    # instance counts as missed by every estimator — a skip
                    # (return None) is reserved for infrastructure failures. There
                    # is no frame to render, so the rows carry no collage path.
                    self.get_logger().info(
                        f'{trial_id} all_instances_missed | no_usable_detections | '
                        f'raw_events={capture["total_events"]} | '
                        f'{format_status_tally(capture["status_histogram"], self.selected_estimators)}'
                    )
                    scene_score = score_scene(
                        usable_by_estimator, gt_instances, self.selected_estimators,
                        detector_fired=capture['any_detected'])
                    return build_trial_result(
                        trial, scene, gt_instances, scene_score,
                        self.selected_estimators, self.estimator_display_names,
                        usable_by_estimator, '', capture['total_events'],
                        captured_events=tuple(self.capture_events.values()))

                # Frame-level scalar medians drive the representative-frame choice
                # and the collage only. Partial by design: an estimator with no
                # usable events simply has no key here. The scored rows come from
                # the per-instance medians below.
                scalar_medians = compute_trial_medians(usable_by_estimator)
                # One scoring path for every scene size: each estimator associates
                # its own estimates to the ground-truth robots, so one estimator's
                # gross error can no longer erase the instance for the rest.
                scene_score = score_scene(
                    usable_by_estimator, gt_instances, self.selected_estimators,
                    detector_fired=capture['any_detected'])

                representative_event = choose_representative_event(
                    usable_events,
                    self.selected_estimators,
                    scalar_medians,
                )
                box_annotations = build_box_annotations(
                    representative_event, gt_instances, self.selected_estimators)
                image_path = self.save_trial_collage(
                    trial_id,
                    representative_event,
                    scalar_medians,
                    gt_instances[0].distance_m,
                    box_annotations,
                    len(scene_score.detector_missed),
                    dominant_miss_reasons(
                        self.selected_estimators,
                        capture['status_histogram'],
                        scalar_medians,
                    ),
                )

                self.log_trial(
                    trial_id, scalar_medians, gt_instances[0].distance_m,
                    len(usable_events), image_path,
                    len(scene_score.detector_missed), scene_score.extra_count)
                return build_trial_result(
                    trial, scene, gt_instances, scene_score,
                    self.selected_estimators, self.estimator_display_names,
                    usable_by_estimator, image_path, capture['total_events'],
                    captured_events=tuple(self.capture_events.values()))
        except Exception as exc:
            self.get_logger().error(f'{trial_id} failed: {exc}')
            return None

    def score_sampled_reference(
        self,
        trial: dict[str, Any],
        scene: Scene,
        gt_instances: list[GroundTruthInstance],
        capture: dict[str, Any],
    ) -> None:
        """Score the sampled batches' live results alone, as the full window is.

        The same reduction, association and row builder, over only the live
        records of the stored stamps: the reference a replay of the sample must
        reproduce. The full-window rows stay what an uncaptured run reports.
        """

        events = capture['sampled_live_events']
        sampled = summarize_capture_events(events, self.selected_estimators)
        score = score_scene(
            sampled['usable_by_estimator'], gt_instances, self.selected_estimators,
            detector_fired=sampled['any_detected'])
        result = build_trial_result(
            trial, scene, gt_instances, score,
            self.selected_estimators, self.estimator_display_names,
            sampled['usable_by_estimator'], '', sampled['total_events'],
            captured_events=tuple(events.values()))
        for estimator, rows in result['rows'].items():
            self.sampled_reference_rows[estimator].extend(rows)
        self.sampled_window_records.append(capture['sampled_window'])

    def write_sampled_reference(self) -> dict[str, Any] | None:
        """Write the sampled live reference beside the full-window results."""

        if not self.sampled_window_capture():
            return None
        root = os.path.join(self.run_output_dir, SAMPLED_REFERENCE_DIRNAME)
        os.makedirs(root, exist_ok=True)
        files = {}
        for estimator in self.selected_estimators:
            name = f'{self.estimator_output_names[estimator]}.csv'
            path = os.path.join(root, name)
            write_trial_csv(path, self.sampled_reference_rows[estimator])
            files[name] = artifact_sha256_file(path)
        payloads = {
            entry['trial_id']: entry['sha256']
            for entry in self.sensor_writer.manifest['trials']}
        windows = [
            {**record, 'payload_sha256': payloads.get(record['trial_id'])}
            for record in self.sampled_window_records]
        measurement = [record['measurement_sec'] for record in windows]
        summary = {
            'mode': SENSOR_CAPTURE_MODE_SAMPLED_WINDOW,
            'reference_dir': SAMPLED_REFERENCE_DIRNAME,
            'sensor_capture_dir': self.sensor_capture_dir,
            'requested_samples': self.capture_batches,
            'trials': len(windows),
            'insufficient_sample_trials': sum(
                record['sample_coverage'] != 'complete' for record in windows),
            'incomplete_trials': sum(not record['complete'] for record in windows),
            'sampled_events': sum(record['actual_samples'] for record in windows),
            'events_without_live_record': sum(
                record['missing']['live_record'] for record in windows),
            'measurement_sec_min': min(measurement, default=None),
            'measurement_sec_max': max(measurement, default=None),
            'drain_sec_max': max((record['drain_sec'] for record in windows), default=None),
            'csv_sha256': files,
        }
        path = os.path.join(root, SAMPLED_WINDOWS_NAME)
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump({
                'schema': SAMPLED_WINDOWS_SCHEMA,
                'run_label': self.run_label,
                'summary': summary,
                'windows': windows,
            }, handle, indent=2, sort_keys=True)
            handle.write('\n')
        summary['windows_sha256'] = artifact_sha256_file(path)
        return summary

    def capture_measurement_window(self, duration_sec: float) -> dict[str, Any]:
        self.capture_events = {}
        self.clear_preview_buffers()
        self.replay_capture_events.clear()
        self.replay_depth_buffer.clear()
        if getattr(self, 'sensor_writer', None) is not None:
            self.replay_depth_source_buffer.clear()
        if self.sampled_window_capture():
            self.sampled_live_events = {}
            self.window_raw_batch_stamps = set()
            self.live_scan_stamp_buffer.clear()
        # The color, camera-info and scan buffers are kept: the first raw batch
        # is usually for a frame received before this point. Matching stays
        # exact-stamp (the scan nearest within tolerance), so a batch only ever
        # receives the inputs of its own instant.
        self.capture_active = True
        sampled_window = None
        try:
            if not self.capture_artifact_enabled():
                self.spin_for(duration_sec)
            elif self.sampled_window_capture():
                sampled_window = self.capture_sampled_window(duration_sec)
            else:
                deadline = time.monotonic() + self.capture_timeout_sec
                while (rclpy.ok() and time.monotonic() < deadline
                       and len(self.replay_capture_events) < self.capture_batches):
                    spin_step(self, timeout_sec=0.1)
                    self.publish_active_truth()
                if len(self.replay_capture_events) < self.capture_batches:
                    raise RuntimeError(
                        f'Replay capture saw {len(self.replay_capture_events)}/{self.capture_batches} '
                        'raw detection batches before capture_timeout_sec.')
                drain_deadline = time.monotonic() + max(0.0, self.capture_drain_sec)
                # The scan is drained for but not required: leaving early the
                # moment the depth path is complete would measure the exit
                # condition rather than how often a scan actually lands, and
                # that landing rate is what decides whether a missing scan
                # should ever block a capture.
                while (rclpy.ok() and time.monotonic() < drain_deadline
                       and not (self.replay_capture_is_complete()
                                and self.replay_scan_capture_is_complete())):
                    spin_step(self, timeout_sec=min(
                        0.1, max(0.0, drain_deadline - time.monotonic())))
                    self.backfill_replay_capture_context()
                    self.publish_active_truth()
                self.capture_events = restrict_events_to_stamps(
                    self.capture_events, set(self.replay_capture_events))
                self.log_scan_capture_coverage()
                if not self.replay_capture_is_complete():
                    if self.sensor_writer is not None:
                        missing_depth = sum(
                            event.get('depth_m') is None
                            for event in self.replay_capture_events.values())
                        missing_rgb = sum(
                            event.get('rgb') is None
                            for event in self.replay_capture_events.values())
                    else:
                        missing_depth = sum(
                            1
                            for event in self.replay_capture_events.values()
                            if event['detected'] and any(
                                detection['depth_roi'] is None
                                for detection in event['detections'])
                        )
                        missing_rgb = 0
                    measurement_stamps = {
                        event.stamp_ns for event in self.capture_events.values()
                    }
                    missing_measurements = len(
                        set(self.replay_capture_events) - measurement_stamps)
                    missing_context = sum(
                        1
                        for event in self.replay_capture_events.values()
                        if event['intrinsics'] is None
                        or event['camera_rotation'] is None
                        or event['camera_translation'] is None
                    )
                    self.get_logger().warning(
                        'Replay capture drain ended with '
                        f'{missing_depth} raw batch(es) missing exact depth and '
                        f'{missing_rgb} missing exact RGB and '
                        f'{missing_context} missing camera context and '
                        f'{missing_measurements} missing a live measurement; '
                        'the absent evidence is preserved in the dataset.')
        finally:
            self.capture_active = False
            self.live_window_open = False
            self.active_truth = None
            self.active_trial_id = ''

        summary = summarize_capture_events(
            self.capture_events, self.selected_estimators)
        if self.capture_artifact_enabled():
            summary['replay_events'] = list(self.replay_capture_events.values())
        if sampled_window is not None:
            summary['sampled_window'] = sampled_window
            summary['sampled_live_events'] = dict(self.sampled_live_events)
        return summary

    def capture_sampled_window(self, duration_sec: float) -> dict[str, Any]:
        """Measure the full window live; freeze only its first raw batches.

        The window is the uncaptured run's own loop, so reaching the sample
        quota changes nothing about what is measured or for how long. Its
        deadline freezes the live population and the sample together; the
        bounded drain after it may only complete batches already sampled.
        """

        trial_id = self.active_trial_id
        started = time.monotonic()
        self.live_window_open = True
        try:
            self.spin_for(duration_sec)
        finally:
            self.live_window_open = False
        measurement_sec = time.monotonic() - started
        sampled_stamps = list(self.replay_capture_events)
        self.replay_depth_buffer.clear()
        self.replay_depth_source_buffer.clear()
        self.live_scan_stamp_buffer.clear()

        drain_started = time.monotonic()
        drain_deadline = drain_started + max(0.0, self.capture_drain_sec)
        while (rclpy.ok() and time.monotonic() < drain_deadline
               and not (self.sampled_capture_is_complete()
                        and self.replay_scan_capture_is_complete())):
            spin_step(self, timeout_sec=min(
                0.1, max(0.0, drain_deadline - time.monotonic())))
            self.backfill_replay_capture_context()
            self.publish_active_truth()
        drain_sec = time.monotonic() - drain_started
        complete = (self.sampled_capture_is_complete()
                    and self.replay_scan_capture_is_complete())
        missing = self.finalize_sampled_events()
        self.log_scan_capture_coverage()
        if len(sampled_stamps) < self.capture_batches:
            self.get_logger().warning(
                f'{trial_id}: insufficient sample coverage, '
                f'{len(sampled_stamps)}/{self.capture_batches} raw batches in '
                f'{measurement_sec:.2f} s; the live window was not extended.')
        if not complete:
            self.get_logger().warning(
                f'{trial_id}: sampled-window drain ended incomplete '
                f'({", ".join(f"{key}={value}" for key, value in missing.items())}); '
                'the absent evidence is preserved in the dataset.')
        return {
            'trial_id': trial_id,
            'requested_measurement_sec': float(duration_sec),
            'measurement_sec': measurement_sec,
            'drain_sec': drain_sec,
            'live_raw_batches': len(self.window_raw_batch_stamps),
            'live_measurement_events': len(self.capture_events),
            'requested_samples': self.capture_batches,
            'actual_samples': len(sampled_stamps),
            'sample_coverage': (
                'complete' if len(sampled_stamps) >= self.capture_batches
                else 'insufficient'),
            'sampled_stamps_ns': sampled_stamps,
            'sampled_live_events': len(self.sampled_live_events),
            'complete': complete,
            'missing': missing,
        }

    def finalize_sampled_events(self) -> dict[str, int]:
        """Attach each sampled batch's live record; count what never arrived.

        Depth whose source description never arrived is dropped rather than
        frozen with an unknown range: the replay would otherwise read it as
        unbounded, which is stereo's contract and not necessarily this one's.
        """

        records: dict[int, list[dict]] = {}
        for live in self.sampled_live_events.values():
            records.setdefault(live.stamp_ns, []).append(
                live_measurement_record(live, self.selected_estimators))
        missing = {
            'live_record': 0, 'rgb': 0, 'depth': 0, 'depth_source': 0,
            'camera_context': 0, 'scan': 0,
        }
        for stamp_ns, event in self.replay_capture_events.items():
            matches = records.get(stamp_ns, [])
            if len(matches) > 1:
                self.get_logger().warning(
                    f'Sampled batch {stamp_ns} has {len(matches)} live results; '
                    'keeping none rather than choosing one.')
            event['live_measurement'] = matches[0] if len(matches) == 1 else None
            if event['live_measurement'] is None:
                missing['live_record'] += 1
            if event.get('depth_m') is not None and event.get('depth_source') is None:
                event['depth_m'] = None
                missing['depth_source'] += 1
            if event.get('rgb') is None:
                missing['rgb'] += 1
            # Depth is missing only where the live node published one; a batch
            # it never measured is counted once, as a missing live record.
            if (event['detections'] and event.get('depth_m') is None
                    and event['live_measurement'] is not None
                    and live_record_expects_depth(event['live_measurement'])):
                missing['depth'] += 1
            if (event['intrinsics'] is None or event['camera_rotation'] is None
                    or event['camera_translation'] is None):
                missing['camera_context'] += 1
            if event.get('scan') is None or event.get('scan_rotation') is None:
                missing['scan'] += 1
        return missing

    def write_replay_trial(
        self,
        trial: dict[str, Any],
        ground_truth_instances: list[GroundTruthInstance],
        events: list[dict[str, Any]],
    ) -> None:
        if not self.capture_artifact_enabled():
            return
        scene: Scene = trial['scene']
        writer = self.replay_writer or self.sensor_writer
        background = getattr(self, 'scene_background', None)
        props = getattr(self, 'background_props', True)
        writer.write_trial({
            'trial_id': trial['trial_id'],
            'repeat_index': trial['repeat_index'],
            'scene_id': scene.id,
            **({
                'placement_dx_m': background.dx_m(scene.id),
                'background_dropped': list(background.scene_drops[scene.id]) if props else [],
            } if background is not None else {}),
            'ground_truth': [
                {
                    'index': instance.index,
                    'model_name': instance.model_name,
                    'world_x': instance.world_x,
                    'world_y': instance.world_y,
                    'forward_m': instance.forward_m,
                    'lateral_m': instance.lateral_m,
                    'distance_m': instance.distance_m,
                    'spawn_yaw_rad': scene.robots[instance.index].yaw,
                }
                for instance in ground_truth_instances
            ],
        }, events)

    def save_trial_collage(
        self,
        trial_id: str,
        representative_event,
        trial_medians: dict[str, float],
        true_distance_m: float,
        box_annotations: list[dict | None] | None = None,
        missed_count: int = 0,
        miss_reasons: dict[str, str] | None = None,
    ) -> str:
        collage = self.collage_renderer.render_trial_collage(
            trial_id,
            representative_event,
            self.selected_estimators,
            trial_medians,
            true_distance_m,
            box_annotations,
            missed_count,
            miss_reasons,
        )
        output_path = os.path.abspath(os.path.join(self.images_dir, f'{trial_id}.png'))
        if not cv2.imwrite(output_path, collage):
            raise RuntimeError(f'Failed to write collage image to {output_path}')
        return output_path

    def clear_preview_buffers(self) -> None:
        self.color_preview_buffer.clear()

    def capture_artifact_enabled(self) -> bool:
        return (
            getattr(self, 'replay_writer', None) is not None
            or getattr(self, 'sensor_writer', None) is not None)

    def backfill_previews_from_buffers(self) -> None:
        if not self.capture_active:
            return

        for event in self.capture_events.values():
            attach_exact_preview(event, 'color', self.color_preview_buffer)

    def wait_for_required_streams(self) -> None:
        if self.needs_pointcloud:
            self.wait_for_flag(
                lambda: self.pointcloud_measurement_seen,
                f'measurement stream on "{self.resolved_topic(self.pointcloud_measurement_topic)}"',
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

    def wait_for_flag(self, condition, description: str) -> None:
        self.get_logger().info(f'Waiting for {description}.')
        deadline = time.monotonic() + STREAM_WAIT_TIMEOUT_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            spin_step(self, timeout_sec=0.2)
            if condition():
                return
        raise RuntimeError(f'Timed out waiting for {description}')

    def compute_scene_ground_truth(
        self,
        robot_models: list[tuple[int, str, Any]],
    ) -> list[GroundTruthInstance]:
        """Ground truth per spawned robot, from one pose snapshot.

        Truth is used here only to produce reference distances (and later the
        scoring assignment); it never reaches the estimators.
        """

        names = [self.robot_model_name, *(name for _index, name, _robot in robot_models)]
        snapshot = self.scene_control.get_scene_snapshot(names)
        return compute_ground_truth_instances(
            snapshot.poses, self.robot_model_name, robot_models)

    def scene_spawn_specs(
        self,
        scene: Scene,
        entity_prefix: str,
    ) -> tuple[list[SpawnSpec], list[tuple[int, str, Any]]]:
        """Resolve objects first, then targets, preserving deterministic order.

        The scene is complete before any target appears in it; rollback
        removes the targets first. A placed scene shifts every entity by its
        dx along world x, as the robot was shifted, so nothing moves relative
        to the robot.
        """

        dx = placement_dx(getattr(self, 'scene_background', None), scene)
        specs = []
        for index, obj in enumerate(scene.objects):
            model_name = f'{entity_prefix}_obj_{index}'
            specs.append(SpawnSpec.for_resource(
                self.asset_selection.objects[obj.model],
                model_name,
                obj.x + dx,
                obj.y,
                obj.yaw,
            ))

        robot_models: list[tuple[int, str, Any]] = []
        for index, robot in enumerate(scene.robots):
            model_name = f'{entity_prefix}_target_{index}'
            specs.append(SpawnSpec.for_resource(
                self.asset_selection.target, model_name, robot.x + dx, robot.y, robot.yaw))
            robot_models.append((index, model_name, robot))

        return specs, robot_models

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
            spin_step(self, timeout_sec=min(0.1, remaining))

    def publish_active_truth(self) -> None:
        """Republish the trial ground truth during capture.

        Faster than the consumers' age gate by a wide margin: at one message
        per gate period a single drop leaves the line unserviced for a third of
        its lifetime, and the number on screen is up to a whole period stale
        before it is even sent.
        """

        if not self.capture_active or self.active_truth is None:
            return
        now = time.monotonic()
        if now - self.last_truth_publish_monotonic < TRUTH_PUBLISH_PERIOD_SEC:
            return
        self.last_truth_publish_monotonic = now
        msg = ground_truth_point_message(self.active_truth, self.active_trial_id)
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
            # row['estimator'] is already the display prose (set above).
            self.get_logger().info(format_summary_log_row(row))


def main() -> int:
    # Owned signals keep the context valid after Ctrl-C, so an interrupted
    # trial can still delete the entities it spawned.
    init_owned()
    node = TargetDistanceBenchmarkRunner()

    try:
        node.run()
        return 0
    except (KeyboardInterrupt, ExternalShutdownException) as exc:
        logger = node.get_logger()
        logger.warning('Benchmark interrupted.')
        # A failed or skipped removal of the interrupted scene, entity by entity.
        for note in getattr(exc, '__notes__', ()):
            logger.warning(note)
        return 130
    except Exception as exc:  # pragma: no cover - surfaced in logs for runtime debugging
        node.get_logger().error(str(exc))
        return 1
    finally:
        with shielded(final=True):
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
