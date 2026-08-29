#!/usr/bin/env python3

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
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
    stamp_to_nanoseconds,
    update_measurement_event,
)
from ridgeback_autonomy.benchmarking.estimators import (
    GROUND_TRUTH_TOPIC,
    MASK_GATE_DEFAULT,
    benchmark_display_name,
    benchmark_output_name,
    benchmark_run_folder_name,
    parse_estimators,
    parse_mask_gate,
    selected_mask_estimators,
    selected_pointcloud_estimators,
    uses_mask_estimators,
    uses_pointcloud_estimators,
)
from ridgeback_autonomy.benchmarking.reduction import (
    choose_representative_event,
    compute_status_histogram,
    compute_trial_medians,
    dominant_miss_reason,
    format_status_tally,
    merge_status_histograms,
    union_usable_events,
    usable_events_by_estimator,
)
from ridgeback_autonomy.benchmarking.association import GtPoint, assign_to_ground_truth
from ridgeback_autonomy.benchmarking.rendering import BenchmarkCollageRenderer
from ridgeback_autonomy.benchmarking.recording import (
    DEFAULT_FPS,
    DEFAULT_MAX_SECONDS,
    ScreenRecorder,
    find_window_id,
)
from ridgeback_autonomy.benchmarking.process_utils import (
    extract_json_payload,
    format_commit,
    git_provenance,
    run_command,
    try_command,
)
from ridgeback_autonomy.benchmarking.report import render_run_report
from ridgeback_autonomy.benchmarking.scenarios import Scene, load_scenarios
from ridgeback_autonomy.benchmarking.scoring import (
    MISS_OUTCOMES,
    OUTCOME_DETECTOR_MISS,
    OUTCOME_NO_VALUE,
    SceneScore,
    build_display_instance_estimate,
    score_scene,
)
from ridgeback_autonomy.benchmarking.summary import (
    build_run_document,
    build_summary_rows,
    write_run_json,
    write_trial_csv,
)
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.perception.core.image_utils import convert_color_image_message
from ridgeback_autonomy.perception.core.vehicle_frame import (
    planar_measurement_from_vehicle_front,
    yaw_from_quaternion,
)
from ridgeback_autonomy.perception.core.isolation_2d import ISOLATION_2D_DEFAULT
from ridgeback_autonomy.perception.core.isolation_3d import ISOLATION_3D_DEFAULT


def ground_truth_point_message(true_pose: dict[str, float], trial_id: str) -> PointStamped:
    """Pack a trial's ground truth as x=lateral, y=forward, z=distance.

    ``estimators.truth_reading`` unpacks the same convention; the values are
    the base-frame planar measurement every benchmark row shares, not a 3D
    point in any TF frame. ``header.frame_id`` carries the trial id for that
    same reason -- it is not a TF frame either, and a truth number that names
    its own trial can be checked against the scene the display is showing.
    """

    msg = PointStamped()
    msg.header.frame_id = trial_id
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
# Republish period for the trial truth, well inside the consumers'
# ``TRUTH_MAX_AGE_S`` so the line survives a dropped message and is never much
# older than the scene it describes.
TRUTH_PUBLISH_PERIOD_SEC = 0.2
POINTCLOUD_MEASUREMENT_TOPIC = 'measurements/g1/pointcloud'
MASK_MEASUREMENT_TOPIC = 'measurements/g1/mask'
DEPTH_SOURCE_DEFAULT = 'stereoscopic'
DEPTH_GATE_DISABLED = 0.0

# WM_CLASS of the window to record. RViz holds the whole picture once the
# perception overlay is published into it, so one window is the whole run.
RECORD_WINDOW_CLASS_DEFAULT = 'rviz'
# RViz starts alongside this node, so allow it a moment to map its window.
RECORD_WINDOW_WAIT_SEC = 20.0
PREVIEW_BUFFER_LIMIT = 256
# Declared by rclpy itself, not by the launch file, so it is not part of a
# run's configuration. ``use_sim_time`` is deliberately NOT excluded: the
# launch sets it and it changes how stamps are interpreted.
RCLPY_INTERNAL_PARAMETERS = frozenset({'start_type_description_service'})


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
        default_scenario = os.path.join(pkg_share, 'config', 'benchmark_scenarios_full.yaml')
        # Kept on the node: the git provenance recorded with each run is read
        # from this directory, not just the default output path.
        self.workspace_root = os.path.abspath(
            os.path.join(pkg_share, '..', '..', '..', '..'))
        default_output_dir = os.path.join(self.workspace_root, 'benchmark-results')

        self.declare_parameter('world', 'g1_distance_calibration')
        self.declare_parameter('scenario', '')
        self.declare_parameter('repeats', 5)
        self.declare_parameter('output_dir', default_output_dir)
        self.declare_parameter('run_dir_name', '')
        self.declare_parameter('settle_sec', 2.0)
        self.declare_parameter('capture_sec', 10.0)
        self.declare_parameter('estimators', 'all')
        self.declare_parameter('pointcloud_measurement_topic', POINTCLOUD_MEASUREMENT_TOPIC)
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
        # Declared so declared_parameters() records the gate the mask rows ran
        # under. The runner itself never applies it.
        self.declare_parameter('mask_depth_max_meters', DEPTH_GATE_DISABLED)

        # Screen recording of the RViz window for the length of the run. The
        # collages freeze one frame per trial; this keeps the motion around it.
        self.declare_parameter('record_video', True)
        self.declare_parameter('record_fps', DEFAULT_FPS)
        self.declare_parameter('record_max_sec', DEFAULT_MAX_SECONDS)
        self.declare_parameter('record_window_class', RECORD_WINDOW_CLASS_DEFAULT)

        self.world = str(self.get_parameter('world').value)
        scenario_param = str(self.get_parameter('scenario').value).strip()
        self.scenario_path = (
            os.path.abspath(os.path.expanduser(scenario_param))
            if scenario_param else default_scenario
        )
        self.scenes = load_scenarios(self.scenario_path)
        self.repeats = int(self.get_parameter('repeats').value)
        self.output_dir = os.path.abspath(os.path.expanduser(str(self.get_parameter('output_dir').value)))
        self.run_dir_name = str(self.get_parameter('run_dir_name').value).strip()
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.capture_sec = float(self.get_parameter('capture_sec').value)
        self.selected_estimators = parse_estimators(str(self.get_parameter('estimators').value))
        self.pointcloud_measurement_topic = str(
            self.get_parameter('pointcloud_measurement_topic').value)
        self.mask_measurement_topic = str(self.get_parameter('mask_measurement_topic').value)
        self.depth_source = str(self.get_parameter('depth_source').value)
        self.isolation_2d = str(self.get_parameter('isolation_2d').value)
        self.isolation_3d = str(self.get_parameter('isolation_3d').value)
        self.mask_gate = parse_mask_gate(str(self.get_parameter('mask_gate').value))
        self.color_topic = str(self.get_parameter('color_topic').value)
        self.record_video = bool(self.get_parameter('record_video').value)
        self.record_window_class = str(self.get_parameter('record_window_class').value)
        self.recorder = ScreenRecorder(
            fps=int(self.get_parameter('record_fps').value),
            max_seconds=int(self.get_parameter('record_max_sec').value),
            log=self.get_logger().warning,
        )

        self.namespace_name = self.get_namespace().strip('/')
        self.robot_model_name = (
            f'{self.namespace_name}/robot' if self.namespace_name else 'robot'
        )
        self.pose_info_topic = f'/world/{self.world}/pose/info'
        self.run_started_at = time.localtime()
        self.run_label = time.strftime('%Y%m%d_%H%M%S', self.run_started_at)
        # The folder carries the axes that change the results, so a results
        # directory reads without opening anything.
        run_folder = self.run_dir_name or benchmark_run_folder_name(
            self.run_label,
            self.scenario_path,
            self.mask_gate,
            self.depth_source,
            self.selected_estimators,
        )
        self.run_output_dir = os.path.join(self.output_dir, run_folder)
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
        self.run_json_path = os.path.join(self.run_output_dir, 'run.json')
        self.report_path = os.path.join(self.run_output_dir, 'summary.md')
        self.video_path = os.path.join(self.run_output_dir, 'video', 'run.mp4')
        # Per-estimator status-code tallies over every captured box across the
        # whole run (misses included, unlike the usable-aligned rows). Feeds the
        # summary's observation columns.
        self.status_aggregate: dict[str, dict[int, int]] = {}

        self.command_env = os.environ.copy()
        self.command_env.setdefault('ROS_LOG_DIR', '/tmp/ros_logs')
        os.makedirs(self.command_env['ROS_LOG_DIR'], exist_ok=True)

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

        self.last_color_decode_warning = None

        self.create_subscription(
            G1Measurements,
            self.pointcloud_measurement_topic,
            self.on_pointcloud_measurement,
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

        # Ground truth for the display surfaces' reference line, republished
        # while a capture window is active so their age gates drop the line
        # between trials. Depth 1: only the newest value is ever wanted, and a
        # deeper queue lets a slow consumer read a previous trial's truth.
        self.ground_truth_pub = self.create_publisher(
            PointStamped, GROUND_TRUTH_TOPIC, 1)
        self.active_truth: dict[str, float] | None = None
        self.active_trial_id = ''
        self.last_truth_publish_monotonic = 0.0

    def on_pointcloud_measurement(self, msg: G1Measurements) -> None:
        self.pointcloud_measurement_seen = True
        if not self.capture_active:
            return

        event = ensure_measurement_event(self.capture_events, msg)
        update_measurement_event(event, msg, self.selected_pointcloud_estimators)
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
            f'in world "{self.world}" for robot "{self.robot_model_name}".'
        )
        self.wait_for_required_streams()
        self.wait_for_entity_pose(self.robot_model_name, POSE_WAIT_TIMEOUT_SEC)

        # Started once the streams are up, so RViz is certain to exist by now,
        # and stopped in the finally below so an exception or a Ctrl-C still
        # leaves a playable file covering everything up to the interruption.
        self.start_recording()
        try:
            self.run_trials()
        finally:
            self.stop_recording()

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

        for trial in self.build_trials():
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

        summary_rows = build_summary_rows(
            estimator_rows, total_extra_detections, outcome_counts, self.status_aggregate)
        self.write_run_outputs(
            summary_rows, estimator_rows, included_trials, skipped_trials)

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

    def declared_parameters(self) -> dict[str, str]:
        """Every ROS parameter this node declared, as text.

        Read off the node rather than hand-listed, so the recorded
        configuration cannot drift as parameters are added. These are the
        launch arguments that reach the runner; launch-only toggles that do not
        affect measurement (rviz, overlay) never become parameters here.
        """

        return {
            name: str(parameter.value)
            for name, parameter in sorted(self.get_parameters_by_prefix('').items())
            if name not in RCLPY_INTERNAL_PARAMETERS
        }

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

        metadata = {
            'Started': time.strftime('%Y-%m-%d %H:%M:%S', self.run_started_at),
            'Commit': format_commit(provenance),
            'Branch': provenance['branch'] or 'unknown',
            'Scenario': self.scenario_path,
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
                'scenes': scenes,
                'instances': instances,
                'trials_included': included_trials,
                'trials_skipped': skipped_trials,
            },
            parameters=parameters,
            display_names=self.estimator_display_names,
            status_histograms=self.status_aggregate,
        ))

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
            self.active_trial_id = trial_id
            capture = self.capture_measurement_window(self.capture_sec)
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
                return self.build_trial_result(
                    trial, scene, gt_instances, scene_score, usable_by_estimator, '',
                    capture['status_histogram'], capture['total_events'])

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
            box_annotations = self.build_box_annotations(representative_event, gt_instances)
            image_path = self.save_trial_collage(
                trial_id,
                representative_event,
                scalar_medians,
                gt_instances[0].distance_m,
                box_annotations,
                len(scene_score.detector_missed),
                self.dominant_miss_reasons(capture['status_histogram'], scalar_medians),
            )

            self.log_trial(
                trial_id, scalar_medians, gt_instances[0].distance_m,
                len(usable_events), image_path,
                len(scene_score.detector_missed), scene_score.extra_count)
            return self.build_trial_result(
                trial, scene, gt_instances, scene_score, usable_by_estimator, image_path,
                capture['status_histogram'], capture['total_events'])
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
            self.active_trial_id = ''

        usable_by_estimator = usable_events_by_estimator(
            self.capture_events, self.selected_estimators)
        return {
            'total_events': len(self.capture_events),
            # Whether the detector produced ANY detection this window, which
            # separates "nothing was there to measure" from "the estimators
            # could not measure what was there".
            'any_detected': any(
                event.detected for event in self.capture_events.values()),
            'usable_by_estimator': usable_by_estimator,
            'usable_events': union_usable_events(usable_by_estimator),
            'status_histogram': compute_status_histogram(
                self.capture_events, self.selected_estimators),
        }

    def build_trial_result(
        self,
        trial: dict[str, Any],
        scene: Scene,
        gt_instances: list[GtInstance],
        scene_score: SceneScore,
        usable_by_estimator: dict[str, list],
        image_path: str,
        status_histogram: dict[str, dict[int, int]],
        frames_captured: int,
    ) -> dict[str, Any]:
        """One row per (instance, estimator), scored or not.

        A miss keeps its row with null estimate/error columns, its ``outcome``
        naming which of the four it was, and -- for ``no_value`` -- the dominant
        ``miss_reason`` behind it. The failure stays attached to the scene that
        caused it instead of collapsing into a run-level count.
        """

        rows: dict[str, list[dict[str, Any]]] = {
            estimator: [] for estimator in self.selected_estimators}
        for gt in gt_instances:
            medians = scene_score.medians.get(gt.index, {})
            outcomes = scene_score.outcomes.get(gt.index, {})
            for estimator in self.selected_estimators:
                estimate = medians.get(estimator)
                if estimate is not None:
                    abs_error = abs(estimate - gt.distance_m)
                    rel_error = abs_error / gt.distance_m if gt.distance_m > 0.0 else None
                else:
                    abs_error = None
                    rel_error = None
                outcome = outcomes.get(estimator, OUTCOME_DETECTOR_MISS)
                # A MissReason only exists for no_value (the estimator's own
                # node reported why). gate_miss and detector_miss are already
                # fully described by the outcome itself.
                miss_reason = (
                    dominant_miss_reason(status_histogram.get(estimator))
                    if outcome == OUTCOME_NO_VALUE else None
                )
                rows[estimator].append({
                    'trial_id': trial['trial_id'],
                    'repeat_index': trial['repeat_index'],
                    'scene_id': scene.id,
                    'instance_index': gt.index,
                    'spawn_world_x': gt.world_x,
                    'spawn_world_y': gt.world_y,
                    'spawn_yaw_rad': scene.robots[gt.index].yaw,
                    'true_forward_m': gt.forward_m,
                    'true_lateral_m': gt.lateral_m,
                    'true_distance_m': gt.distance_m,
                    'estimator': self.estimator_display_names[estimator],
                    'outcome': outcome,
                    'miss_reason': miss_reason,
                    'trial_estimate_m': estimate,
                    'abs_error_m': abs_error,
                    'rel_error': rel_error,
                    'usable_aligned_events': len(usable_by_estimator.get(estimator, [])),
                    'frames_captured': frames_captured,
                    'image_path': image_path,
                })

        return {
            'rows': rows,
            'missed_count': len(scene_score.detector_missed),
            'outcome_counts': {
                estimator: scene_score.outcome_counts(estimator)
                for estimator in self.selected_estimators
            },
            'extra_count': scene_score.extra_count,
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
            build_display_instance_estimate(detection, index, self.selected_estimators)
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

    def dominant_miss_reasons(
        self,
        status_histogram: dict[str, dict[int, int]],
        trial_medians: dict[str, float],
    ) -> dict[str, str]:
        """For estimators with no median this trial: the most frequent non-OK
        reason from the capture window, for the collage panel."""

        reasons: dict[str, str] = {}
        for estimator in self.selected_estimators:
            if estimator in trial_medians:
                continue
            reason = dominant_miss_reason(status_histogram.get(estimator))
            if reason is not None:
                reasons[estimator] = reason
        return reasons

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
            forward_m, lateral_m, distance_m = planar_measurement_from_vehicle_front(
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
            try:
                result = self.try_command(command, timeout_sec=COMMAND_TIMEOUT_SEC)
            except RuntimeError:
                time.sleep(COMMAND_RETRY_SLEEP_SEC)
                continue
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
        return run_command(
            command,
            timeout_sec=timeout_sec,
            description=description,
            env=self.command_env,
        )

    def try_command(
        self,
        command: list[str],
        timeout_sec: float,
    ) -> subprocess.CompletedProcess[str]:
        return try_command(
            command,
            timeout_sec=timeout_sec,
            env=self.command_env,
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
