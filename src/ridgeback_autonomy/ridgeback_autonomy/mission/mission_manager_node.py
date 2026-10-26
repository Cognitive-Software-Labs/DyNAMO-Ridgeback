#!/usr/bin/env python3

from __future__ import annotations

import ast
import math
import os
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from statistics import median

import numpy as np
import yaml
import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ridgeback_autonomy.common.tf_utils import rotation_matrix_from_quaternion
from ridgeback_autonomy.msg import G1Detections
from ridgeback_autonomy.msg import G1Measurements
from ridgeback_autonomy.mission.parking_geometry import (
    base_pose_from_task_point,
    normalize_angle,
    planar_path_length,
    preparking_pose,
    rotated_footprint_vertices,
    task_point_from_base_pose,
)
from ridgeback_autonomy.perception.core.geometry import (
    ROBOT_FRONT_OFFSET_M,
    extract_organized_xyz,
    map_bbox_between_images,
)
from ridgeback_autonomy.perception.core.image_utils import convert_depth_to_meters_message
from ridgeback_autonomy.perception.core.time_sync import (
    prune_timestamped_buffer,
    select_closest_timestamped,
    stamp_seconds,
)
from tf2_ros import Buffer, TransformException, TransformListener


class MissionState(Enum):
    WAITING_FOR_START_POSE = 'waiting_for_start_pose'
    SEARCH_G1 = 'search_g1'
    STOPPING_EXPLORATION = 'stopping_exploration'
    CENTER_G1 = 'center_g1'
    LOCALIZE_G1 = 'localize_g1'
    TRANSFORM_G1_TO_MAP = 'transform_g1_to_map'
    G1_LOCALIZED = 'g1_localized'
    DONUT_READY = 'donut_ready'
    DONUT_CLASSIFIED = 'donut_classified'
    FOOTPRINT_CHECKED = 'footprint_checked'
    PARKING_POSES_VALIDATED = 'parking_poses_validated'
    NAV2_FEASIBILITY_CHECKED = 'nav2_feasibility_checked'
    PARKING_POSE_SELECTED = 'parking_pose_selected'
    NAVIGATING_TO_G1_OBSERVATION_POSE = 'navigating_to_g1_observation_pose'
    NAVIGATING_TO_PREPARKING_POSE = 'navigating_to_preparking_pose'
    NAVIGATING_TO_PARKING_POSE = 'navigating_to_parking_pose'
    PARKED = 'parked'
    RETURNING_TO_START = 'returning_to_start'
    MISSION_COMPLETE = 'mission_complete'
    MISSION_FAILED = 'mission_failed'


@dataclass(frozen=True)
class FootMidpointEstimate:
    midpoint_base: tuple[float, float]
    foot_1_base: tuple[float, float]
    foot_2_base: tuple[float, float]
    total_lower_body_points: int
    cluster_1_count: int
    cluster_2_count: int
    cluster_separation_m: float
    cluster_balance_ratio: float
    range_spread_m: float
    height_spread_m: float
    source: str
    stamp: TimeMsg


@dataclass(frozen=True)
class G1ObservationFraming:
    bbox_bottom_px: float
    image_height_px: int
    bottom_margin_px: float
    bbox_height_px: float
    lower_body_depth_points: int
    lower_body_cloud_points: int
    usable: bool
    reason: str


@dataclass(frozen=True)
class G1BodyPositionEstimate:
    body_base: tuple[float, float, float]
    body_map: tuple[float, float]
    valid_points: int
    roi_xyxy: tuple[int, int, int, int]
    range_spread_m: float
    height_spread_m: float
    source: str
    stamp: TimeMsg


@dataclass
class ParkingCandidateDecision:
    marker_id: int
    candidate: tuple[float, float, float]
    status: str = 'not_evaluated'
    reason: str = 'awaiting_validation'
    path_length_m: float | None = None
    preference_label: str | None = None
    reach_error_m: float | None = None
    clearance_m: float | None = None


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager_node')
        namespace_name = self.get_namespace().strip('/')
        default_rear_bin_frame = (
            f'{namespace_name}/robot/rear_bin_floor_link'
            if namespace_name
            else 'rear_bin_floor_link'
        )

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('g1_detection_topic', 'detections/g1/raw')
        self.declare_parameter('g1_camera_measurement_topic', 'measurements/g1/camera')
        self.declare_parameter('camera_info_topic', 'sensors/camera_0/color/camera_info')
        self.declare_parameter('depth_topic', 'sensors/camera_0/depth/image')
        self.declare_parameter('pointcloud_topic', 'sensors/camera_0/points')
        self.declare_parameter('camera_optical_frame', 'camera_0_color_optical_frame')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('stop_exploration_on_g1', True)
        self.declare_parameter('exploration_stop_service', 'explore/set_enabled')
        self.declare_parameter('exploration_active_topic', 'explore/active')
        self.declare_parameter('g1_localization_timeout_s', 5.0)
        self.declare_parameter('g1_observation_trigger_s', 6.0)
        self.declare_parameter('g1_observation_distance_m', 1.6)
        self.declare_parameter('g1_observation_distance_step_m', 0.4)
        self.declare_parameter('g1_observation_distance_count', 3)
        self.declare_parameter('g1_observation_bottom_margin_px', 24)
        self.declare_parameter('g1_observation_lower_body_min_points', 40)
        self.declare_parameter('enable_translating_g1_observation', False)
        self.declare_parameter('g1_centering_angular_tolerance_rad', 0.10)
        self.declare_parameter('g1_centering_max_corrections', 2)
        self.declare_parameter('g1_centering_detection_timeout_s', 4.0)
        self.declare_parameter('g1_estimate_marker_topic', 'visualization/g1/estimates')
        self.declare_parameter('rear_bin_frame', default_rear_bin_frame)
        self.declare_parameter('compute_path_to_pose_action', 'compute_path_to_pose')
        self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose')
        self.declare_parameter('task_point_x_base_m', -0.280)
        self.declare_parameter('task_point_y_base_m', 0.000)
        self.declare_parameter('preparking_offset_m', 0.50)
        self.declare_parameter('parking_wait_duration_s', 5.0)
        self.declare_parameter(
            'nav2_params_path',
            os.path.join(
                get_package_share_directory('ridgeback_autonomy'),
                'config',
                'nav2_params.yaml',
            ),
        )
        self.declare_parameter('reach_min_m', 0.30)
        self.declare_parameter('reach_max_m', 0.60)
        self.declare_parameter('g1_foot_midpoint_debug_enabled', False)
        self.declare_parameter('g1_debug_world_x_m', 7.2)
        self.declare_parameter('g1_debug_world_y_m', -4.2)
        self.declare_parameter('g1_debug_world_yaw_rad', 1.5708)
        self.declare_parameter('g1_left_foot_center_x_m', 0.038260)
        self.declare_parameter('g1_left_foot_center_y_m', 0.118556)
        self.declare_parameter('g1_right_foot_center_x_m', 0.038260)
        self.declare_parameter('g1_right_foot_center_y_m', -0.118556)
        self.declare_parameter('g1_foot_cluster_min_points', 12)
        self.declare_parameter('g1_foot_cluster_min_balance_ratio', 0.25)
        self.declare_parameter('g1_foot_cluster_min_separation_m', 0.18)
        self.declare_parameter('g1_foot_cluster_max_separation_m', 0.35)
        self.declare_parameter('g1_foot_to_old_estimate_max_distance_m', 0.50)
        self.declare_parameter('g1_body_sample_count', 5)
        self.declare_parameter('g1_body_min_points', 30)
        self.declare_parameter('g1_body_roi_x_min_fraction', 0.35)
        self.declare_parameter('g1_body_roi_x_max_fraction', 0.65)
        self.declare_parameter('g1_body_roi_y_min_fraction', 0.25)
        self.declare_parameter('g1_body_roi_y_max_fraction', 0.60)
        self.declare_parameter('g1_body_min_height_m', 0.60)
        self.declare_parameter('g1_body_max_height_m', 1.80)
        self.declare_parameter('g1_body_sample_max_jump_m', 0.75)
        self.declare_parameter('g1_body_bbox_edge_margin_px', 8)
        self.declare_parameter('g1_body_to_old_estimate_max_distance_m', 0.75)
        self.declare_parameter('g1_sensor_sync_tolerance_s', 0.10)
        self.declare_parameter('g1_sensor_buffer_size', 30)
        self.declare_parameter('g1_sensor_buffer_max_age_s', 2.0)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('parking_path_sample_spacing_m', 0.10)
        self.declare_parameter('parking_candidate_arrows_enabled', True)
        self.declare_parameter('parking_candidate_labels_enabled', False)
        self.declare_parameter('parking_candidate_scores_enabled', False)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.robot_base_frame = str(self.get_parameter('robot_base_frame').value)
        self.g1_detection_topic = str(self.get_parameter('g1_detection_topic').value)
        self.g1_camera_measurement_topic = str(
            self.get_parameter('g1_camera_measurement_topic').value
        )
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)
        self.camera_optical_frame = str(self.get_parameter('camera_optical_frame').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.stop_exploration_on_g1 = bool(
            self.get_parameter('stop_exploration_on_g1').value
        )
        self.exploration_stop_service = str(
            self.get_parameter('exploration_stop_service').value
        )
        self.exploration_active_topic = str(
            self.get_parameter('exploration_active_topic').value
        )
        self.g1_localization_timeout_s = float(
            self.get_parameter('g1_localization_timeout_s').value
        )
        self.g1_observation_trigger_s = float(
            self.get_parameter('g1_observation_trigger_s').value
        )
        self.g1_observation_distance_m = float(
            self.get_parameter('g1_observation_distance_m').value
        )
        self.g1_observation_distance_step_m = float(
            self.get_parameter('g1_observation_distance_step_m').value
        )
        self.g1_observation_distance_count = max(
            1,
            int(self.get_parameter('g1_observation_distance_count').value),
        )
        self.g1_observation_bottom_margin_px = max(
            0,
            int(self.get_parameter('g1_observation_bottom_margin_px').value),
        )
        self.g1_observation_lower_body_min_points = max(
            1,
            int(self.get_parameter('g1_observation_lower_body_min_points').value),
        )
        self.enable_translating_g1_observation = bool(
            self.get_parameter('enable_translating_g1_observation').value
        )
        self.g1_centering_angular_tolerance_rad = max(
            0.01,
            float(self.get_parameter('g1_centering_angular_tolerance_rad').value),
        )
        self.g1_centering_max_corrections = max(
            0,
            int(self.get_parameter('g1_centering_max_corrections').value),
        )
        self.g1_centering_detection_timeout_s = max(
            0.5,
            float(self.get_parameter('g1_centering_detection_timeout_s').value),
        )
        self.g1_estimate_marker_topic = str(
            self.get_parameter('g1_estimate_marker_topic').value
        )
        self.rear_bin_frame = str(self.get_parameter('rear_bin_frame').value)
        self.compute_path_to_pose_action = str(
            self.get_parameter('compute_path_to_pose_action').value
        )
        self.navigate_to_pose_action = str(
            self.get_parameter('navigate_to_pose_action').value
        )
        self.task_point_x_base_m = float(
            self.get_parameter('task_point_x_base_m').value
        )
        self.task_point_y_base_m = float(
            self.get_parameter('task_point_y_base_m').value
        )
        self.preparking_offset_m = float(
            self.get_parameter('preparking_offset_m').value
        )
        self.parking_wait_duration_s = float(
            self.get_parameter('parking_wait_duration_s').value
        )
        self.nav2_params_path = str(self.get_parameter('nav2_params_path').value)
        self.reach_min_m = float(self.get_parameter('reach_min_m').value)
        self.reach_max_m = float(self.get_parameter('reach_max_m').value)
        self.g1_foot_midpoint_debug_enabled = bool(
            self.get_parameter('g1_foot_midpoint_debug_enabled').value
        )
        self.g1_debug_world_x_m = float(self.get_parameter('g1_debug_world_x_m').value)
        self.g1_debug_world_y_m = float(self.get_parameter('g1_debug_world_y_m').value)
        self.g1_debug_world_yaw_rad = float(
            self.get_parameter('g1_debug_world_yaw_rad').value
        )
        self.g1_left_foot_center_x_m = float(
            self.get_parameter('g1_left_foot_center_x_m').value
        )
        self.g1_left_foot_center_y_m = float(
            self.get_parameter('g1_left_foot_center_y_m').value
        )
        self.g1_right_foot_center_x_m = float(
            self.get_parameter('g1_right_foot_center_x_m').value
        )
        self.g1_right_foot_center_y_m = float(
            self.get_parameter('g1_right_foot_center_y_m').value
        )
        self.g1_foot_cluster_min_points = int(
            self.get_parameter('g1_foot_cluster_min_points').value
        )
        self.g1_foot_cluster_min_balance_ratio = float(
            self.get_parameter('g1_foot_cluster_min_balance_ratio').value
        )
        self.g1_foot_cluster_min_separation_m = float(
            self.get_parameter('g1_foot_cluster_min_separation_m').value
        )
        self.g1_foot_cluster_max_separation_m = float(
            self.get_parameter('g1_foot_cluster_max_separation_m').value
        )
        self.g1_foot_to_old_estimate_max_distance_m = float(
            self.get_parameter('g1_foot_to_old_estimate_max_distance_m').value
        )
        self.g1_body_sample_count = max(
            1,
            int(self.get_parameter('g1_body_sample_count').value),
        )
        self.g1_body_min_points = max(
            1,
            int(self.get_parameter('g1_body_min_points').value),
        )
        self.g1_body_roi_x_min_fraction = float(
            self.get_parameter('g1_body_roi_x_min_fraction').value
        )
        self.g1_body_roi_x_max_fraction = float(
            self.get_parameter('g1_body_roi_x_max_fraction').value
        )
        self.g1_body_roi_y_min_fraction = float(
            self.get_parameter('g1_body_roi_y_min_fraction').value
        )
        self.g1_body_roi_y_max_fraction = float(
            self.get_parameter('g1_body_roi_y_max_fraction').value
        )
        self.g1_body_min_height_m = float(
            self.get_parameter('g1_body_min_height_m').value
        )
        self.g1_body_max_height_m = float(
            self.get_parameter('g1_body_max_height_m').value
        )
        self.g1_body_sample_max_jump_m = float(
            self.get_parameter('g1_body_sample_max_jump_m').value
        )
        self.g1_body_bbox_edge_margin_px = max(
            0,
            int(self.get_parameter('g1_body_bbox_edge_margin_px').value),
        )
        self.g1_body_to_old_estimate_max_distance_m = float(
            self.get_parameter('g1_body_to_old_estimate_max_distance_m').value
        )
        self.g1_sensor_sync_tolerance_s = max(
            0.0,
            float(self.get_parameter('g1_sensor_sync_tolerance_s').value),
        )
        self.g1_sensor_buffer_size = max(
            2,
            int(self.get_parameter('g1_sensor_buffer_size').value),
        )
        self.g1_sensor_buffer_max_age_s = max(
            self.g1_sensor_sync_tolerance_s,
            float(self.get_parameter('g1_sensor_buffer_max_age_s').value),
        )
        self.occupied_threshold = int(
            self.get_parameter('occupied_threshold').value
        )
        self.parking_path_sample_spacing_m = max(
            0.01,
            float(self.get_parameter('parking_path_sample_spacing_m').value),
        )
        self.parking_candidate_arrows_enabled = bool(
            self.get_parameter('parking_candidate_arrows_enabled').value
        )
        self.parking_candidate_labels_enabled = bool(
            self.get_parameter('parking_candidate_labels_enabled').value
        )
        self.parking_candidate_scores_enabled = bool(
            self.get_parameter('parking_candidate_scores_enabled').value
        )
        self.footprint_points = self._load_nav2_footprint(self.nav2_params_path)
        self.nav2_planner_id = self._load_nav2_planner_id(self.nav2_params_path)
        self.footprint_min_x_m = min(point[0] for point in self.footprint_points)
        self.footprint_max_x_m = max(point[0] for point in self.footprint_points)
        self.footprint_min_y_m = min(point[1] for point in self.footprint_points)
        self.footprint_max_y_m = max(point[1] for point in self.footprint_points)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            self.compute_path_to_pose_action,
        )
        self.navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action,
        )
        self.exploration_stop_client = self.create_client(
            SetBool,
            self.exploration_stop_service,
        )
        visualization_qos = QoSProfile(depth=1)
        visualization_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.state = MissionState.WAITING_FOR_START_POSE
        self.start_pose: PoseStamped | None = None
        self.latest_map: OccupancyGrid | None = None
        self.g1_detection_vote_window_size = 5
        self.g1_detection_vote_required = 3
        self.g1_detection_vote_window = deque(maxlen=self.g1_detection_vote_window_size)
        self.g1_last_confirmation_vote = (0, 0)
        self.latest_camera_info: CameraInfo | None = None
        self.depth_buffer = deque(maxlen=self.g1_sensor_buffer_size)
        self.pointcloud_buffer = deque(maxlen=self.g1_sensor_buffer_size)
        self.g1_sync_last_log_s = {'depth': 0.0, 'cloud': 0.0}
        self.latest_g1_detection_bbox: tuple[float, float, float, float] | None = None
        self.latest_g1_detection_score: float | None = None
        self.latest_g1_detection_image_size: tuple[int, int] | None = None
        self.latest_g1_detection_stamp: TimeMsg | None = None
        self.recent_g1_detection_bboxes = deque(maxlen=10)
        self.g1_old_ground_estimate_samples: list[tuple[float, float]] = []
        self.g1_old_ground_estimate_x_base_m: float | None = None
        self.g1_old_ground_estimate_y_base_m: float | None = None
        self.g1_old_ground_estimate_x_map_m: float | None = None
        self.g1_old_ground_estimate_y_map_m: float | None = None
        self.g1_ground_x_base_m: float | None = None
        self.g1_ground_y_base_m: float | None = None
        self.g1_map_x_m: float | None = None
        self.g1_map_y_m: float | None = None
        self.g1_sensor_foot_midpoint_samples: list[
            tuple[float, float, tuple[float, float], tuple[float, float]]
        ] = []
        self.g1_sensor_foot_midpoint_x_base_m: float | None = None
        self.g1_sensor_foot_midpoint_y_base_m: float | None = None
        self.g1_sensor_foot_midpoint_x_map_m: float | None = None
        self.g1_sensor_foot_midpoint_y_map_m: float | None = None
        self.g1_sensor_foot_1_map_m: tuple[float, float] | None = None
        self.g1_sensor_foot_2_map_m: tuple[float, float] | None = None
        self.g1_body_position_samples: list[tuple[float, float]] = []
        self.g1_body_position_x_map_m: float | None = None
        self.g1_body_position_y_map_m: float | None = None
        self.g1_position_source: str | None = None
        self.parking_pose_candidates: list[tuple[float, float, float]] = []
        self.parking_candidate_decisions: dict[
            tuple[float, float, float], ParkingCandidateDecision
        ] = {}
        self.nav2_reachable_candidates: list[tuple[float, float, float]] = []
        self.nav2_unreachable_candidates: list[tuple[float, float, float]] = []
        self.nav2_reachable_path_lengths: dict[tuple[float, float, float], float] = {}
        self.nav2_path_rejection_counts: dict[str, int] = {}
        self.blocked_occupied_task_points: list[Point] = []
        self.blocked_unknown_task_points: list[Point] = []
        self.blocked_footprint_task_points: list[Point] = []
        self.blocked_path_task_points: list[Point] = []
        self.selected_parking_pose: dict[str, float] | None = None
        self.preparking_pose: dict[str, float] | None = None
        self.actual_preparking_pose: dict[str, float] | None = None
        self.actual_parking_pose: dict[str, float] | None = None
        self.latest_rgb_estimate_markers: list[tuple[float, float, str, int]] = []
        self.exploration_active: bool | None = None
        self.exploration_stop_accepted = False
        self.exploration_stop_request_in_flight = False
        self.exploration_stop_timer = None
        self.exploration_stop_reason: str | None = None
        self.exploration_resume_request_in_flight = False
        self.exploration_resume_retry_timer = None
        self.g1_committed = False
        self.g1_commit_robot_pose: tuple[float, float, float] | None = None
        self.g1_observation_attempted = False
        self.g1_observation_pose_reached = False
        self.g1_observation_start_s: float | None = None
        self.g1_observation_trigger_timer = None
        self.g1_observation_validation_wait_timer = None
        self.g1_observation_candidates: list[dict[str, float | str]] = []
        self.g1_observation_candidate_index = 0
        self.g1_observation_pose: dict[str, float] | None = None
        self.g1_current_observation_label: str | None = None
        self.g1_observation_view_check_pending = False
        self.g1_centering_corrections = 0
        self.g1_centering_goal_generation = 0
        self.g1_centering_goal_in_flight = False
        self.g1_centering_min_detection_stamp_s = 0.0
        self.g1_centering_detection_timer = None
        self.g1_localization_generation = 0
        self.g1_localization_min_stamp_s = 0.0
        self.g1_localization_failure_reasons: set[str] = set()
        self.nav2_candidate_index = 0
        self.nav2_wait_timer = None
        self.preparking_validation_wait_timer = None
        self.navigate_wait_timer = None
        self.return_navigation_wait_timer = None
        self.preparking_navigation_goal_sent = False
        self.preparking_navigation_complete = False
        self.parking_navigation_goal_sent = False
        self.parking_navigation_complete = False
        self.parking_wait_timer = None
        self.parking_wait_completed = False
        self.parking_wait_started = False
        self.return_goal_sent = False
        self.return_navigation_complete = False
        self.last_navigation_feedback_log_time_ns = 0
        self.g1_map_transform_timer = None
        self.g1_localization_timeout_timer = None
        self.parking_pipeline_start_s: float | None = None
        self.parking_pipeline_stage_starts: dict[str, float] = {}
        self.parking_pipeline_durations: dict[str, float] = {}
        self.parking_pipeline_candidates_checked = 0
        self.reachability_donut_segments = 72

        self.start_pose_timer = self.create_timer(0.5, self._try_save_start_pose)
        self.reachability_donut_publisher = self.create_publisher(
            MarkerArray,
            'visualization/reachability_donut',
            visualization_qos,
        )
        self.reachability_occupancy_publisher = self.create_publisher(
            MarkerArray,
            'visualization/reachability_occupancy',
            visualization_qos,
        )
        self.reachability_blocked_clean_publisher = self.create_publisher(
            MarkerArray,
            'visualization/reachability_blocked_clean',
            visualization_qos,
        )
        self.reachability_footprint_publisher = self.create_publisher(
            MarkerArray,
            'visualization/reachability_footprint',
            visualization_qos,
        )
        self.parking_pose_candidates_publisher = self.create_publisher(
            MarkerArray,
            'visualization/parking_pose_candidates',
            visualization_qos,
        )
        self.nav2_feasible_parking_poses_publisher = self.create_publisher(
            MarkerArray,
            'visualization/nav2_feasible_parking_poses',
            visualization_qos,
        )
        self.selected_parking_pose_publisher = self.create_publisher(
            MarkerArray,
            'visualization/selected_parking_pose',
            visualization_qos,
        )
        self.thesis_parking_publisher = self.create_publisher(
            MarkerArray,
            'visualization/thesis_parking',
            visualization_qos,
        )
        self.g1_detection_subscription = self.create_subscription(
            G1Detections,
            self.g1_detection_topic,
            self._on_g1_detections,
            10,
        )
        self.g1_camera_measurement_subscription = self.create_subscription(
            G1Measurements,
            self.g1_camera_measurement_topic,
            self._on_g1_camera_measurements,
            10,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            10,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            self.depth_topic,
            self._on_depth_image,
            qos_profile_sensor_data,
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            self.pointcloud_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._on_map,
            map_qos,
        )
        self.exploration_active_subscription = self.create_subscription(
            Bool,
            self.exploration_active_topic,
            self._on_exploration_active,
            visualization_qos,
        )
        self.g1_estimate_marker_subscription = self.create_subscription(
            MarkerArray,
            self.g1_estimate_marker_topic,
            self._on_g1_estimate_markers,
            10,
        )

        self.get_logger().info(
            f'Waiting for mission start pose from {self.map_frame} to '
            f'{self.robot_base_frame}'
        )
        self.get_logger().info(
            'Task-point offset: '
            f'dx={self.task_point_x_base_m:.3f}, '
            f'dy={self.task_point_y_base_m:.3f}'
        )
        self.get_logger().info(
            'G1 localization sources: sensor-derived foot midpoint, '
            'with RGB-D body projection fallback'
        )

    def _load_nav2_footprint(self, nav2_params_path: str) -> list[tuple[float, float]]:
        with open(nav2_params_path, 'r', encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)

        footprint_value = config['global_costmap']['global_costmap']['ros__parameters'][
            'footprint'
        ]
        footprint = ast.literal_eval(str(footprint_value))
        points = [
            (float(point[0]), float(point[1]))
            for point in footprint
        ]
        if len(points) < 3:
            raise ValueError('Nav2 footprint must contain at least 3 points')
        for x_m, y_m in points:
            if not math.isfinite(x_m) or not math.isfinite(y_m):
                raise ValueError('Nav2 footprint contains non-finite coordinates')
        return points

    def _load_nav2_planner_id(self, nav2_params_path: str) -> str:
        with open(nav2_params_path, 'r', encoding='utf-8') as config_file:
            config = yaml.safe_load(config_file)

        planner_plugins = config['planner_server']['ros__parameters']['planner_plugins']
        if not planner_plugins:
            raise ValueError('Nav2 planner_server.planner_plugins is empty')
        return str(planner_plugins[0])

    def _try_save_start_pose(self) -> None:
        if self.start_pose is not None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation.x = transform.transform.rotation.x
        pose.pose.orientation.y = transform.transform.rotation.y
        pose.pose.orientation.z = transform.transform.rotation.z
        pose.pose.orientation.w = transform.transform.rotation.w

        self.start_pose = pose
        self.get_logger().info(
            'Saved mission start pose: '
            f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}'
        )

        previous_state = self.state
        self.state = MissionState.SEARCH_G1
        self._reset_g1_detection_vote_window()
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self.get_logger().info('Waiting for confirmed G1 detection')

        self.start_pose_timer.cancel()

    def _publish_reachability_donut(self) -> None:
        if self.state != MissionState.G1_LOCALIZED:
            return
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return

        self._timing_start('donut_generation')
        marker_array = MarkerArray()
        marker_array.markers.append(
            self._reachability_fill_marker(self.g1_map_x_m, self.g1_map_y_m)
        )
        marker_array.markers.append(
            self._reachability_outline_marker(
                marker_id=1,
                radius_m=self.reach_min_m,
                center_x_m=self.g1_map_x_m,
                center_y_m=self.g1_map_y_m,
            )
        )
        marker_array.markers.append(
            self._reachability_outline_marker(
                marker_id=2,
                radius_m=self.reach_max_m,
                center_x_m=self.g1_map_x_m,
                center_y_m=self.g1_map_y_m,
            )
        )
        marker_array.markers.append(
            self._donut_center_debug_marker(self.g1_map_x_m, self.g1_map_y_m)
        )
        foot_midpoint_marker = self._g1_foot_midpoint_debug_marker()
        if foot_midpoint_marker is not None:
            marker_array.markers.append(foot_midpoint_marker)
        self.reachability_donut_publisher.publish(marker_array)

        self.get_logger().info(
            'Published reachability donut: '
            f'center=({self.g1_map_x_m:.3f}, {self.g1_map_y_m:.3f}), '
            f'Rmin={self.reach_min_m:.3f} m, '
            f'Rmax={self.reach_max_m:.3f} m'
        )
        self.get_logger().info('G1 reachability donut created')

        previous_state = self.state
        self.state = MissionState.DONUT_READY
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._timing_stop('donut_generation')
        self._try_classify_reachability_occupancy()

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self._try_classify_reachability_occupancy()

    def _on_g1_estimate_markers(self, msg: MarkerArray) -> None:
        rgb_markers = []
        for marker in msg.markers:
            if marker.ns != 'g1_estimates/rgb':
                continue
            if marker.type != Marker.SPHERE:
                continue
            if marker.action == Marker.DELETE:
                continue
            rgb_markers.append((
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                str(marker.header.frame_id),
                int(marker.id),
            ))
        if rgb_markers:
            self.latest_rgb_estimate_markers = sorted(
                rgb_markers,
                key=lambda item: item[3],
            )

    def _try_classify_reachability_occupancy(self) -> None:
        if self.state != MissionState.DONUT_READY:
            return
        if self.latest_map is None:
            return
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return

        self._timing_start('occupancy_filter')
        marker_array, free_count, occupied_count, unknown_count, free_task_points = (
            self._build_reachability_occupancy_markers(self.latest_map)
        )
        self.reachability_occupancy_publisher.publish(marker_array)
        self.blocked_occupied_task_points = list(marker_array.markers[1].points)
        self.blocked_unknown_task_points = list(marker_array.markers[2].points)
        self.blocked_footprint_task_points = []
        self.blocked_path_task_points = []
        self._publish_clean_reachability_validity()

        self.get_logger().info(
            'Reachability occupancy (task points): '
            f'free={free_count}, occupied={occupied_count}, '
            f'unknown={unknown_count} cells'
        )
        self.get_logger().info(
            'Donut candidates: '
            f'total={free_count + occupied_count + unknown_count}, '
            f'blocked={occupied_count + unknown_count}, '
            f'valid={free_count}'
        )

        previous_state = self.state
        self.state = MissionState.DONUT_CLASSIFIED
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._timing_stop('occupancy_filter')
        self._check_footprint_feasibility(free_task_points)

    def _build_reachability_occupancy_markers(
        self,
        grid: OccupancyGrid,
    ) -> tuple[MarkerArray, int, int, int, list[Point]]:
        free_marker = self._reachability_occupancy_marker(
            marker_id=0,
            red=0.1,
            green=0.9,
            blue=0.2,
            alpha=0.75,
            grid=grid,
        )
        occupied_marker = self._reachability_occupancy_marker(
            marker_id=1,
            red=1.0,
            green=0.0,
            blue=0.0,
            alpha=1.0,
            grid=grid,
        )
        unknown_marker = self._reachability_occupancy_marker(
            marker_id=2,
            red=1.0,
            green=0.12,
            blue=0.0,
            alpha=0.85,
            grid=grid,
        )

        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        origin = grid.info.origin
        rotation = rotation_matrix_from_quaternion(origin.orientation)

        for row in range(height):
            local_y = (float(row) + 0.5) * resolution
            row_offset = row * width
            for col in range(width):
                value = int(grid.data[row_offset + col])
                local_x = (float(col) + 0.5) * resolution
                map_x = float(
                    origin.position.x
                    + rotation[0][0] * local_x
                    + rotation[0][1] * local_y
                )
                map_y = float(
                    origin.position.y
                    + rotation[1][0] * local_x
                    + rotation[1][1] * local_y
                )
                distance = math.hypot(
                    map_x - self.g1_map_x_m,
                    map_y - self.g1_map_y_m,
                )
                if distance < self.reach_min_m or distance > self.reach_max_m:
                    continue

                point = Point(x=map_x, y=map_y, z=0.08)
                if value == -1:
                    unknown_marker.points.append(point)
                elif value >= self.occupied_threshold:
                    occupied_marker.points.append(point)
                elif 0 <= value < self.occupied_threshold:
                    free_marker.points.append(point)

        marker_array = MarkerArray()
        marker_array.markers = [free_marker, occupied_marker, unknown_marker]
        return (
            marker_array,
            len(free_marker.points),
            len(occupied_marker.points),
            len(unknown_marker.points),
            list(free_marker.points),
        )

    def _reachability_occupancy_marker(
        self,
        marker_id: int,
        red: float,
        green: float,
        blue: float,
        alpha: float,
        grid: OccupancyGrid,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reachability_occupancy'
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(grid.info.resolution) * 1.2
        marker.scale.y = float(grid.info.resolution) * 1.2
        marker.scale.z = 0.03
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _reachability_blocked_clean_markers(
        self,
        occupied_points: list[Point],
        unknown_points: list[Point],
        footprint_points: list[Point],
        path_points: list[Point],
        grid: OccupancyGrid,
    ) -> MarkerArray:
        occupied_marker = self._reachability_blocked_clean_marker(
            marker_id=0,
            namespace='reachability_blocked_occupied',
            red=1.0,
            green=0.0,
            blue=0.0,
            alpha=0.95,
            grid=grid,
        )
        occupied_marker.points = [
            Point(x=point.x, y=point.y, z=0.13)
            for point in occupied_points
        ]

        unknown_marker = self._reachability_blocked_clean_marker(
            marker_id=1,
            namespace='reachability_blocked_unknown',
            red=1.0,
            green=0.20,
            blue=0.0,
            alpha=0.85,
            grid=grid,
        )
        unknown_marker.points = [
            Point(x=point.x, y=point.y, z=0.14)
            for point in unknown_points
        ]

        footprint_marker = self._reachability_blocked_clean_marker(
            marker_id=2,
            namespace='reachability_blocked_footprint',
            red=0.95,
            green=0.0,
            blue=0.12,
            alpha=0.90,
            grid=grid,
        )
        footprint_marker.points = [
            Point(x=point.x, y=point.y, z=0.15)
            for point in footprint_points
        ]

        path_marker = self._reachability_blocked_clean_marker(
            marker_id=3,
            namespace='reachability_blocked_path',
            red=0.80,
            green=0.0,
            blue=0.30,
            alpha=0.90,
            grid=grid,
        )
        path_marker.points = [
            Point(x=point.x, y=point.y, z=0.16)
            for point in path_points
        ]

        return MarkerArray(
            markers=[
                occupied_marker,
                unknown_marker,
                footprint_marker,
                path_marker,
            ]
        )

    def _publish_clean_reachability_validity(self) -> None:
        if self.latest_map is None:
            return
        self.reachability_blocked_clean_publisher.publish(
            self._reachability_blocked_clean_markers(
                occupied_points=self.blocked_occupied_task_points,
                unknown_points=self.blocked_unknown_task_points,
                footprint_points=self.blocked_footprint_task_points,
                path_points=self.blocked_path_task_points,
                grid=self.latest_map,
            )
        )

    def _reachability_blocked_clean_marker(
        self,
        marker_id: int,
        namespace: str,
        red: float,
        green: float,
        blue: float,
        alpha: float,
        grid: OccupancyGrid,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(grid.info.resolution) * 1.4
        marker.scale.y = float(grid.info.resolution) * 1.4
        marker.scale.z = 0.05
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _check_footprint_feasibility(self, free_task_points: list[Point]) -> None:
        if self.state != MissionState.DONUT_CLASSIFIED:
            return
        if self.latest_map is None:
            return
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return

        safe_marker = self._reachability_footprint_marker(
            marker_id=0,
            red=0.1,
            green=0.95,
            blue=0.2,
            alpha=0.95,
            grid=self.latest_map,
        )
        invalid_marker = self._reachability_footprint_marker(
            marker_id=1,
            red=1.0,
            green=0.05,
            blue=0.0,
            alpha=0.95,
            grid=self.latest_map,
        )

        rejection_counts = {
            'occupied': 0,
            'unknown': 0,
            'out_of_bounds': 0,
            'other': 0,
        }
        representative_invalid: tuple[Point, str] | None = None
        representative_priority = {
            'occupied': 0,
            'unknown': 1,
            'out_of_bounds': 2,
            'other': 3,
        }
        valid_candidates: list[tuple[float, float, float]] = []
        invalid_after_rotation: list[tuple[float, float, float]] = []
        self.parking_candidate_decisions = {}

        for task_point in free_task_points:
            base_pose_start_s = time.monotonic()
            base_x_m, base_y_m, yaw = self._base_pose_from_task_point(
                task_point.x,
                task_point.y,
            )
            task_x_check, task_y_check = self._task_point_from_base_pose(
                base_x_m,
                base_y_m,
                yaw,
            )
            self._timing_add_duration(
                'base_pose_generation',
                time.monotonic() - base_pose_start_s,
            )
            if (
                abs(task_x_check - task_point.x) > 1e-6
                or abs(task_y_check - task_point.y) > 1e-6
            ):
                rejection_reason = 'other'
            else:
                footprint_start_s = time.monotonic()
                footprint_polygon = self._rotated_footprint_vertices(
                    base_x_m,
                    base_y_m,
                    yaw,
                )
                rejection_reason = self._rotated_footprint_rejection_reason(
                    footprint_polygon,
                    self.latest_map,
                )
                self._timing_add_duration(
                    'footprint_check',
                    time.monotonic() - footprint_start_s,
                )
            candidate = (base_x_m, base_y_m, yaw)
            if rejection_reason is None:
                valid_candidates.append(candidate)
                self._register_parking_candidate_decision(
                    candidate,
                    status='not_evaluated',
                    reason='awaiting_nav2_feasibility',
                )
                safe_marker.points.append(
                    Point(x=task_point.x, y=task_point.y, z=0.14)
                )
            else:
                invalid_after_rotation.append(candidate)
                status = self._parking_candidate_status_for_rejection(
                    rejection_reason
                )
                self._register_parking_candidate_decision(
                    candidate,
                    status=status,
                    reason=f'footprint_{rejection_reason}',
                )
                invalid_marker.points.append(
                    Point(x=task_point.x, y=task_point.y, z=0.14)
                )
                rejection_counts[rejection_reason] += 1
                if (
                    representative_invalid is None
                    or representative_priority[rejection_reason]
                    < representative_priority[representative_invalid[1]]
                ):
                    representative_invalid = (
                        Point(x=base_x_m, y=base_y_m, z=0.0),
                        rejection_reason,
                    )

        self.parking_pose_candidates = valid_candidates
        self.blocked_footprint_task_points = list(invalid_marker.points)
        self._publish_clean_reachability_validity()

        self.reachability_footprint_publisher.publish(
            MarkerArray(markers=[safe_marker, invalid_marker])
        )
        self._publish_parking_candidate_decisions()
        self._log_parking_candidate_decisions('footprint')

        self.get_logger().info(
            'Footprint feasibility: '
            f'safe={len(valid_candidates)}, '
            f'invalid={len(invalid_after_rotation)} task-point cells'
        )
        self.get_logger().info(
            'Footprint rejection reasons: '
            f'occupied={rejection_counts["occupied"]}, '
            f'unknown={rejection_counts["unknown"]}, '
            f'out_of_bounds={rejection_counts["out_of_bounds"]}, '
            f'other={rejection_counts["other"]}'
        )
        if representative_invalid is not None:
            center, reason = representative_invalid
            self.get_logger().info(
                'Representative derived-base footprint rejection: '
                f'base_center=({center.x:.3f}, {center.y:.3f}), '
                f'reason={reason.upper()}'
            )

        self.get_logger().info(
            'Rotated parking poses: '
            f'valid={len(valid_candidates)}, '
            f'invalid_after_rotation={len(invalid_after_rotation)}'
        )
        self.get_logger().info(
            'Parking candidates after footprint check: '
            f'total={len(free_task_points)}, '
            f'valid={len(valid_candidates)}, '
            f'invalid={len(invalid_after_rotation)}'
        )
        if valid_candidates:
            x_m, y_m, yaw = valid_candidates[0]
            self.get_logger().info(
                'Representative parking pose: '
                f'x={x_m:.3f}, y={y_m:.3f}, yaw={yaw:.3f} rad'
            )

        previous_state = self.state
        self.state = MissionState.FOOTPRINT_CHECKED
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )

        previous_state = self.state
        self.state = MissionState.PARKING_POSES_VALIDATED
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._start_nav2_feasibility_check()

    def _base_pose_from_task_point(
        self,
        task_x_m: float,
        task_y_m: float,
    ) -> tuple[float, float, float]:
        return base_pose_from_task_point(
            task_x_m,
            task_y_m,
            self.g1_map_x_m,
            self.g1_map_y_m,
            self.task_point_x_base_m,
            self.task_point_y_base_m,
        )

    def _task_point_from_base_pose(
        self,
        base_x_m: float,
        base_y_m: float,
        yaw: float,
    ) -> tuple[float, float]:
        return task_point_from_base_pose(
            base_x_m,
            base_y_m,
            yaw,
            self.task_point_x_base_m,
            self.task_point_y_base_m,
        )

    def _reachability_footprint_marker(
        self,
        marker_id: int,
        red: float,
        green: float,
        blue: float,
        alpha: float,
        grid: OccupancyGrid,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reachability_footprint'
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(grid.info.resolution) * 1.4
        marker.scale.y = float(grid.info.resolution) * 1.4
        marker.scale.z = 0.04
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _debug_footprint_outline_marker(self, marker_id: int, center: Point) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reachability_footprint_debug'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        corners = [
            (center.x + self.footprint_min_x_m, center.y + self.footprint_min_y_m),
            (center.x + self.footprint_max_x_m, center.y + self.footprint_min_y_m),
            (center.x + self.footprint_max_x_m, center.y + self.footprint_max_y_m),
            (center.x + self.footprint_min_x_m, center.y + self.footprint_max_y_m),
            (center.x + self.footprint_min_x_m, center.y + self.footprint_min_y_m),
        ]
        marker.points = [
            Point(x=x_m, y=y_m, z=0.22)
            for x_m, y_m in corners
        ]
        return marker

    def _debug_footprint_center_marker(self, marker_id: int, center: Point) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reachability_footprint_debug'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = center.x
        marker.pose.position.y = center.y
        marker.pose.position.z = 0.22
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _start_nav2_feasibility_check(self) -> None:
        if self.state != MissionState.PARKING_POSES_VALIDATED:
            return

        self.nav2_reachable_candidates = []
        self.nav2_unreachable_candidates = []
        self.nav2_reachable_path_lengths = {}
        self.nav2_path_rejection_counts = {}
        self.selected_parking_pose = None
        self.preparking_pose = None
        self.actual_preparking_pose = None
        self.actual_parking_pose = None
        self.preparking_navigation_goal_sent = False
        self.preparking_navigation_complete = False
        self.parking_navigation_goal_sent = False
        self.parking_navigation_complete = False
        self.return_goal_sent = False
        self.return_navigation_complete = False
        self.nav2_candidate_index = 0
        self.parking_pipeline_candidates_checked = len(self.parking_pose_candidates)
        self._timing_start('nav2_feasibility')

        if not self.parking_pose_candidates:
            self._finish_nav2_feasibility_check()
            return

        self.get_logger().info('Waiting for ComputePathToPose action server')
        self.nav2_wait_timer = self.create_timer(
            0.5,
            self._try_start_nav2_feasibility_goals,
        )
        self._try_start_nav2_feasibility_goals()

    def _try_start_nav2_feasibility_goals(self) -> None:
        if self.state != MissionState.PARKING_POSES_VALIDATED:
            if self.nav2_wait_timer is not None:
                self.nav2_wait_timer.cancel()
                self.nav2_wait_timer = None
            return
        if not self.compute_path_client.server_is_ready():
            return

        if self.nav2_wait_timer is not None:
            self.nav2_wait_timer.cancel()
            self.nav2_wait_timer = None

        self._send_next_nav2_feasibility_goal()

    def _send_next_nav2_feasibility_goal(self) -> None:
        if self.state != MissionState.PARKING_POSES_VALIDATED:
            return

        if self.nav2_candidate_index >= len(self.parking_pose_candidates):
            self._finish_nav2_feasibility_check()
            return

        candidate = self.parking_pose_candidates[self.nav2_candidate_index]
        x_m, y_m, yaw = candidate
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._candidate_pose_stamped(x_m, y_m, yaw)
        goal_msg.planner_id = self.nav2_planner_id
        goal_msg.use_start = False

        goal_future = self.compute_path_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(
            lambda future, checked_candidate=candidate: (
                self._on_compute_path_goal_response(future, checked_candidate)
            )
        )

    def _on_compute_path_goal_response(self, future, candidate: tuple[float, float, float]) -> None:
        if self.state != MissionState.PARKING_POSES_VALIDATED:
            return

        try:
            goal_handle = future.result()
        except Exception:
            self._record_nav2_unreachable_and_continue(candidate)
            return

        if not goal_handle.accepted:
            self._record_nav2_unreachable_and_continue(candidate)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, checked_candidate=candidate: (
                self._on_compute_path_result(future, checked_candidate)
            )
        )

    def _on_compute_path_result(self, future, candidate: tuple[float, float, float]) -> None:
        if self.state != MissionState.PARKING_POSES_VALIDATED:
            return

        try:
            action_result = future.result()
        except Exception:
            self._record_nav2_unreachable_and_continue(candidate)
            return

        result = action_result.result
        path_is_valid = (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == ComputePathToPose.Result.NONE
            and len(result.path.poses) > 0
        )
        path_rejection_reason = None
        if path_is_valid and self.latest_map is not None:
            path_rejection_reason = self._path_footprint_rejection_reason(
                result.path,
                self.latest_map,
            )
            path_is_valid = path_rejection_reason is None

        if path_is_valid:
            self.nav2_reachable_candidates.append(candidate)
            path_length_m = self._path_length_m(result.path)
            self.nav2_reachable_path_lengths[candidate] = path_length_m
            self._update_parking_candidate_decision(
                candidate,
                status='valid',
                reason='nav2_reachable',
                path_length_m=path_length_m,
            )
        else:
            self.nav2_unreachable_candidates.append(candidate)
            reason = path_rejection_reason or 'planning_failure'
            self.nav2_path_rejection_counts[reason] = (
                self.nav2_path_rejection_counts.get(reason, 0) + 1
            )
            self._update_parking_candidate_decision(
                candidate,
                status=self._parking_candidate_status_for_rejection(reason),
                reason=reason,
            )

        self.nav2_candidate_index += 1
        self._send_next_nav2_feasibility_goal()

    def _record_nav2_unreachable_and_continue(
        self,
        candidate: tuple[float, float, float],
        reason: str = 'request_failure',
    ) -> None:
        self.nav2_unreachable_candidates.append(candidate)
        self.nav2_path_rejection_counts[reason] = (
            self.nav2_path_rejection_counts.get(reason, 0) + 1
        )
        self._update_parking_candidate_decision(
            candidate,
            status=self._parking_candidate_status_for_rejection(reason),
            reason=reason,
        )
        self.nav2_candidate_index += 1
        self._send_next_nav2_feasibility_goal()

    def _finish_nav2_feasibility_check(self) -> None:
        if self.state != MissionState.PARKING_POSES_VALIDATED:
            return

        self._timing_stop('nav2_feasibility')
        self.nav2_feasible_parking_poses_publisher.publish(
            self._nav2_feasible_parking_pose_markers(
                self.nav2_reachable_candidates,
                self.nav2_unreachable_candidates,
            )
        )
        self.blocked_path_task_points = [
            Point(x=task_x, y=task_y, z=0.16)
            for task_x, task_y in (
                self._task_point_from_base_pose(x_m, y_m, yaw)
                for x_m, y_m, yaw in self.nav2_unreachable_candidates
            )
        ]
        self._publish_clean_reachability_validity()
        self._publish_parking_candidate_decisions()
        self._log_parking_candidate_decisions('nav2_feasibility')

        self.get_logger().info(
            'Nav2 parking feasibility: '
            f'reachable={len(self.nav2_reachable_candidates)}, '
            f'unreachable={len(self.nav2_unreachable_candidates)}'
        )
        self.get_logger().info(
            'Parking candidates after Nav2 feasibility: '
            f'total={len(self.nav2_reachable_candidates) + len(self.nav2_unreachable_candidates)}, '
            f'reachable={len(self.nav2_reachable_candidates)}, '
            f'unreachable={len(self.nav2_unreachable_candidates)}'
        )
        if self.nav2_path_rejection_counts:
            self.get_logger().info(
                'Parking path rejection reasons: '
                + ', '.join(
                    f'{reason}={count}'
                    for reason, count in sorted(
                        self.nav2_path_rejection_counts.items()
                    )
                )
            )
        if self.nav2_reachable_candidates:
            x_m, y_m, yaw = self.nav2_reachable_candidates[0]
            self.get_logger().info(
                'Representative Nav2-reachable pose: '
                f'x={x_m:.3f}, y={y_m:.3f}, yaw={yaw:.3f} rad'
            )

        previous_state = self.state
        self.state = MissionState.NAV2_FEASIBILITY_CHECKED
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._select_final_parking_pose()

    def _select_final_parking_pose(self) -> None:
        if self.state != MissionState.NAV2_FEASIBILITY_CHECKED:
            return
        self._timing_start('candidate_selection')
        if not self.nav2_reachable_candidates:
            self.get_logger().info('No Nav2-reachable parking pose available')
            self._timing_stop('candidate_selection')
            if self.g1_committed:
                self._fail_mission(
                    'No feasible parking pose around G1; mission failed safely'
                )
            return
        if self.latest_map is None:
            self.get_logger().warn('Cannot select parking pose without OccupancyGrid')
            self._timing_stop('candidate_selection')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            self.get_logger().warn('Cannot select parking pose without G1 map position')
            self._timing_stop('candidate_selection')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return

        resolution = float(self.latest_map.info.resolution)
        r_preferred_m = (self.reach_min_m + self.reach_max_m) / 2.0
        unsafe_cell_centers = self._unsafe_grid_cell_centers(self.latest_map)

        candidate_metrics = []
        for candidate in self.nav2_reachable_candidates:
            x_m, y_m, yaw = candidate
            task_x_m, task_y_m = self._task_point_from_base_pose(x_m, y_m, yaw)
            preference_rank, preference_label, preference_basis = (
                self._parking_candidate_preference(task_x_m, task_y_m)
            )
            task_distance_to_g1_m = math.hypot(
                task_x_m - self.g1_map_x_m,
                task_y_m - self.g1_map_y_m,
            )
            reach_error_m = abs(task_distance_to_g1_m - r_preferred_m)
            clearance_m = self._rotated_footprint_clearance_m(
                candidate,
                self.latest_map,
                unsafe_cell_centers,
            )
            path_length_m = self.nav2_reachable_path_lengths.get(candidate, math.inf)
            candidate_metrics.append({
                'candidate': candidate,
                'x': x_m,
                'y': y_m,
                'yaw': yaw,
                'task_x': task_x_m,
                'task_y': task_y_m,
                'distance_to_g1': task_distance_to_g1_m,
                'reach_error': reach_error_m,
                'clearance': clearance_m,
                'path_length': path_length_m,
                'preference_rank': preference_rank,
                'preference_label': preference_label,
                'preference_basis': preference_basis,
            })
            self._update_parking_candidate_decision(
                candidate,
                path_length_m=path_length_m,
                preference_label=preference_label,
                reach_error_m=reach_error_m,
                clearance_m=clearance_m,
            )

        best_preference_rank = min(
            metric['preference_rank'] for metric in candidate_metrics
        )
        preference_preferred = [
            metric for metric in candidate_metrics
            if metric['preference_rank'] == best_preference_rank
        ]

        best_reach_error_m = min(
            metric['reach_error'] for metric in preference_preferred
        )
        reach_preferred = [
            metric for metric in preference_preferred
            if metric['reach_error'] <= best_reach_error_m + resolution
        ]

        best_clearance_m = max(metric['clearance'] for metric in reach_preferred)
        clearance_preferred = [
            metric for metric in reach_preferred
            if metric['clearance'] >= best_clearance_m - resolution
        ]

        best_path_length_m = min(metric['path_length'] for metric in clearance_preferred)
        path_preferred = [
            metric for metric in clearance_preferred
            if metric['path_length'] <= best_path_length_m + resolution
        ]
        selected = path_preferred[0]

        preference_counts = {}
        for metric in candidate_metrics:
            label = str(metric['preference_label'])
            preference_counts[label] = preference_counts.get(label, 0) + 1

        if self._g1_orientation_yaw_map() is None:
            self.get_logger().info(
                'G1 orientation unavailable; selecting safest feasible parking '
                'candidate with approach-relative side/front/back priority'
            )
        else:
            self.get_logger().info(
                'Sensor-derived G1 orientation available; applying side/front/back '
                'candidate priority'
            )
        self.get_logger().info(
            'Parking candidate preference buckets: '
            + ', '.join(
                f'{label}={count}'
                for label, count in sorted(preference_counts.items())
            )
        )

        self.selected_parking_pose = {
            'x': selected['x'],
            'y': selected['y'],
            'yaw': selected['yaw'],
            'task_x': selected['task_x'],
            'task_y': selected['task_y'],
            'distance_to_g1': selected['distance_to_g1'],
            'reach_error': selected['reach_error'],
            'clearance': selected['clearance'],
            'path_length': selected['path_length'],
            'preference_rank': selected['preference_rank'],
            'preference_label': selected['preference_label'],
            'preference_basis': selected['preference_basis'],
        }
        selected_candidate = (
            selected['x'],
            selected['y'],
            selected['yaw'],
        )
        self._update_parking_candidate_decision(
            selected_candidate,
            status='selected',
            reason='selected',
        )
        selected_decision = self.parking_candidate_decisions.get(
            selected_candidate
        )
        if selected_decision is not None:
            self.get_logger().info(
                f'Parking candidate selected: C{selected_decision.marker_id:02d}'
            )
        self._publish_parking_candidate_decisions()
        self._log_parking_candidate_decisions('selection')
        self.get_logger().info(
            'Parking pose selection: '
            f'input={len(candidate_metrics)}, '
            f'preference_class={selected["preference_label"]}, '
            f'preference_basis={selected["preference_basis"]}, '
            f'preference_preferred={len(preference_preferred)}, '
            f'reach_preferred={len(reach_preferred)}, '
            f'clearance_preferred={len(clearance_preferred)}'
        )
        self.get_logger().info(
            'Parking selection criteria: preference class, preferred-reach '
            'error (within one map cell), footprint clearance (within one map '
            'cell), then Nav2 path length (within one map cell)'
        )
        self.get_logger().info(
            'Selected parking pose: '
            f'base_x={self.selected_parking_pose["x"]:.3f}, '
            f'base_y={self.selected_parking_pose["y"]:.3f}, '
            f'yaw={self.selected_parking_pose["yaw"]:.3f} rad, '
            f'task_x={self.selected_parking_pose["task_x"]:.3f}, '
            f'task_y={self.selected_parking_pose["task_y"]:.3f}, '
            'task_distance_to_g1='
            f'{self.selected_parking_pose["distance_to_g1"]:.3f} m, '
            f'reach_error={self.selected_parking_pose["reach_error"]:.3f} m, '
            f'clearance={self.selected_parking_pose["clearance"]:.3f} m, '
            f'path_length={self.selected_parking_pose["path_length"]:.3f} m'
        )
        self._publish_thesis_parking_markers()

        previous_state = self.state
        self.state = MissionState.PARKING_POSE_SELECTED
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._timing_stop('candidate_selection')
        self._prepare_preparking_navigation()

    def _parking_candidate_preference(
        self,
        task_x_m: float,
        task_y_m: float,
    ) -> tuple[int, str, str]:
        """Classify a task point for selection priority.

        True humanoid left/right requires a reliable sensor-derived G1 orientation.
        Until that exists, use the committed encounter direction as a real,
        observable reference and prefer lateral viewpoints around G1 before the
        approach side and finally the opposite side.
        """
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return 1, 'front', 'unavailable'

        task_angle = math.atan2(
            task_y_m - self.g1_map_y_m,
            task_x_m - self.g1_map_x_m,
        )

        g1_orientation_yaw = self._g1_orientation_yaw_map()
        if g1_orientation_yaw is not None:
            reference_yaw = g1_orientation_yaw
            basis = 'g1_orientation'
            side_label = 'side'
            front_label = 'front'
            back_label = 'back'
        else:
            reference = self._approach_reference_pose()
            if reference is None:
                return 1, 'front', 'approach_unavailable'
            reference_x_m, reference_y_m, _ = reference
            if math.hypot(
                reference_x_m - self.g1_map_x_m,
                reference_y_m - self.g1_map_y_m,
            ) < 0.10:
                return 1, 'front', 'approach_unavailable'
            reference_yaw = math.atan2(
                reference_y_m - self.g1_map_y_m,
                reference_x_m - self.g1_map_x_m,
            )
            basis = 'approach_relative'
            side_label = 'approach_side'
            front_label = 'approach_front'
            back_label = 'approach_back'

        relative = self._normalize_angle(task_angle - reference_yaw)
        side_error = abs(abs(relative) - math.pi / 2.0)
        front_error = abs(relative)
        back_error = abs(abs(relative) - math.pi)

        if side_error <= front_error and side_error <= back_error:
            return 0, side_label, basis
        if front_error <= back_error:
            return 1, front_label, basis
        return 2, back_label, basis

    def _g1_orientation_yaw_map(self) -> float | None:
        return None

    def _approach_reference_pose(self) -> tuple[float, float, float] | None:
        if self.g1_commit_robot_pose is not None:
            return self.g1_commit_robot_pose
        if self.start_pose is not None:
            return (
                float(self.start_pose.pose.position.x),
                float(self.start_pose.pose.position.y),
                self._yaw_from_quaternion(self.start_pose.pose.orientation),
            )
        return self._current_base_pose_in_map()

    def _prepare_preparking_navigation(self) -> None:
        if self.state != MissionState.PARKING_POSE_SELECTED:
            return
        self._timing_start('preparking_validation')
        if self.selected_parking_pose is None:
            self.get_logger().info(
                'No selected parking pose available; navigation not started'
            )
            self._timing_stop('preparking_validation')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return
        if self.latest_map is None:
            self.get_logger().warn('Pre-parking pose unavailable: no OccupancyGrid')
            self._timing_stop('preparking_validation')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return

        final_x_m = self.selected_parking_pose['x']
        final_y_m = self.selected_parking_pose['y']
        final_yaw = self.selected_parking_pose['yaw']
        pre_x_m, pre_y_m, pre_yaw = preparking_pose(
            final_x_m,
            final_y_m,
            final_yaw,
            self.preparking_offset_m,
        )

        self.get_logger().info(
            'Pre-parking pose: '
            f'x={pre_x_m:.3f}, y={pre_y_m:.3f}, yaw={pre_yaw:.3f} rad'
        )

        footprint_polygon = self._rotated_footprint_vertices(
            pre_x_m,
            pre_y_m,
            pre_yaw,
        )
        rejection_reason = self._rotated_footprint_rejection_reason(
            footprint_polygon,
            self.latest_map,
        )
        if rejection_reason is not None:
            self.get_logger().warn(
                'Pre-parking pose unavailable: '
                f'footprint {rejection_reason}'
            )
            self.selected_parking_pose_publisher.publish(
                self._selected_parking_pose_markers(self.selected_parking_pose)
            )
            self._publish_thesis_parking_markers()
            self._timing_stop('preparking_validation')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return

        tentative_preparking_pose = {
            'x': pre_x_m,
            'y': pre_y_m,
            'yaw': pre_yaw,
        }
        self._validate_preparking_path(tentative_preparking_pose)

    def _validate_preparking_path(self, preparking_pose: dict[str, float]) -> None:
        if self.state != MissionState.PARKING_POSE_SELECTED:
            return
        if not self.compute_path_client.server_is_ready():
            if self.preparking_validation_wait_timer is None:
                self.preparking_validation_wait_timer = self.create_timer(
                    0.5,
                    lambda: self._validate_preparking_path(preparking_pose),
                )
            return

        if self.preparking_validation_wait_timer is not None:
            self.preparking_validation_wait_timer.cancel()
            self.preparking_validation_wait_timer = None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._candidate_pose_stamped(
            preparking_pose['x'],
            preparking_pose['y'],
            preparking_pose['yaw'],
        )
        goal_msg.planner_id = self.nav2_planner_id
        goal_msg.use_start = False

        goal_future = self.compute_path_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(
            lambda future, checked_pose=preparking_pose: (
                self._on_preparking_path_goal_response(future, checked_pose)
            )
        )

    def _on_preparking_path_goal_response(
        self,
        future,
        preparking_pose: dict[str, float],
    ) -> None:
        if self.state != MissionState.PARKING_POSE_SELECTED:
            return

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().warn(f'Pre-parking pose unavailable: {error}')
            self._timing_stop('preparking_validation')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Pre-parking pose unavailable: path goal rejected')
            self._timing_stop('preparking_validation')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, checked_pose=preparking_pose: (
                self._on_preparking_path_result(future, checked_pose)
            )
        )

    def _on_preparking_path_result(
        self,
        future,
        preparking_pose: dict[str, float],
    ) -> None:
        if self.state != MissionState.PARKING_POSE_SELECTED:
            return

        try:
            action_result = future.result()
        except Exception as error:
            self.get_logger().warn(f'Pre-parking pose unavailable: {error}')
            self._timing_stop('preparking_validation')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return

        result = action_result.result
        path_is_valid = (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == ComputePathToPose.Result.NONE
            and len(result.path.poses) > 0
        )
        path_rejection_reason = None
        if path_is_valid and self.latest_map is not None:
            path_rejection_reason = self._path_footprint_rejection_reason(
                result.path,
                self.latest_map,
            )
            path_is_valid = path_rejection_reason is None
        if not path_is_valid:
            detail = path_rejection_reason or 'no Nav2 path'
            self.get_logger().warn(
                f'Pre-parking pose unavailable: {detail}'
            )
            self._timing_stop('preparking_validation')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'No feasible parking pose around G1; returning to start'
                )
            return

        self.preparking_pose = preparking_pose
        path_length_m = self._path_length_m(result.path)
        self.selected_parking_pose_publisher.publish(
            self._selected_parking_pose_markers(self.selected_parking_pose)
        )
        self._publish_thesis_parking_markers()
        self.get_logger().info(
            'Pre-parking pose Nav2 path valid: '
            f'path_length={path_length_m:.3f} m'
        )
        self._timing_stop('preparking_validation')
        self._start_preparking_navigation()

    def _start_preparking_navigation(self) -> None:
        if self.state != MissionState.PARKING_POSE_SELECTED:
            return
        if self.preparking_pose is None:
            self.get_logger().warn('Pre-parking pose unavailable')
            return
        if self.preparking_navigation_goal_sent:
            return

        if not self.navigate_to_pose_client.server_is_ready():
            self.get_logger().info('Waiting for NavigateToPose action server')
            if self.navigate_wait_timer is None:
                self.navigate_wait_timer = self.create_timer(
                    0.5,
                    self._try_send_preparking_goal,
                )
            return

        self._send_preparking_goal()

    def _try_send_preparking_goal(self) -> None:
        if self.state != MissionState.PARKING_POSE_SELECTED:
            if self.navigate_wait_timer is not None:
                self.navigate_wait_timer.cancel()
                self.navigate_wait_timer = None
            return
        if self.preparking_pose is None:
            self.get_logger().warn('Pre-parking pose unavailable')
            if self.navigate_wait_timer is not None:
                self.navigate_wait_timer.cancel()
                self.navigate_wait_timer = None
            return
        if not self.navigate_to_pose_client.server_is_ready():
            return

        if self.navigate_wait_timer is not None:
            self.navigate_wait_timer.cancel()
            self.navigate_wait_timer = None
        self._send_preparking_goal()

    def _send_preparking_goal(self) -> None:
        if self.preparking_pose is None or self.preparking_navigation_goal_sent:
            return

        x_m = self.preparking_pose['x']
        y_m = self.preparking_pose['y']
        yaw = self.preparking_pose['yaw']

        goal = NavigateToPose.Goal()
        goal.pose = self._candidate_pose_stamped(x_m, y_m, yaw)
        self.preparking_navigation_goal_sent = True
        self.last_navigation_feedback_log_time_ns = 0

        self.get_logger().info(
            'Sending pre-parking pose to Nav2: '
            f'x={x_m:.3f}, y={y_m:.3f}, yaw={yaw:.3f}'
        )
        self._log_parking_pipeline_timing()

        previous_state = self.state
        self.state = MissionState.NAVIGATING_TO_PREPARKING_POSE
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )

        send_future = self.navigate_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self._on_parking_navigation_feedback,
        )
        send_future.add_done_callback(self._on_preparking_navigation_goal_response)

    def _start_parking_navigation(self) -> None:
        if self.state not in (
            MissionState.PARKING_POSE_SELECTED,
            MissionState.NAVIGATING_TO_PREPARKING_POSE,
        ):
            return
        if self.selected_parking_pose is None:
            self.get_logger().info(
                'No selected parking pose available; navigation not started'
            )
            return
        if self.parking_navigation_goal_sent:
            return

        if not self.navigate_to_pose_client.server_is_ready():
            self.get_logger().info('Waiting for NavigateToPose action server')
            if self.navigate_wait_timer is None:
                self.navigate_wait_timer = self.create_timer(
                    0.5,
                    self._try_send_selected_parking_goal,
                )
            return

        self._send_selected_parking_goal()

    def _try_send_selected_parking_goal(self) -> None:
        if self.state not in (
            MissionState.PARKING_POSE_SELECTED,
            MissionState.NAVIGATING_TO_PREPARKING_POSE,
        ):
            if self.navigate_wait_timer is not None:
                self.navigate_wait_timer.cancel()
                self.navigate_wait_timer = None
            return
        if self.selected_parking_pose is None:
            self.get_logger().info(
                'No selected parking pose available; navigation not started'
            )
            if self.navigate_wait_timer is not None:
                self.navigate_wait_timer.cancel()
                self.navigate_wait_timer = None
            return
        if not self.navigate_to_pose_client.server_is_ready():
            return

        if self.navigate_wait_timer is not None:
            self.navigate_wait_timer.cancel()
            self.navigate_wait_timer = None
        self._send_selected_parking_goal()

    def _send_selected_parking_goal(self) -> None:
        if self.selected_parking_pose is None or self.parking_navigation_goal_sent:
            return

        x_m = self.selected_parking_pose['x']
        y_m = self.selected_parking_pose['y']
        yaw = self.selected_parking_pose['yaw']

        goal = NavigateToPose.Goal()
        goal.pose = self._candidate_pose_stamped(x_m, y_m, yaw)
        self.parking_navigation_goal_sent = True
        self.last_navigation_feedback_log_time_ns = 0

        self.get_logger().info(
            'Sending selected base pose to Nav2: '
            f'base_x={x_m:.3f}, base_y={y_m:.3f}, yaw={yaw:.3f}'
        )

        previous_state = self.state
        self.state = MissionState.NAVIGATING_TO_PARKING_POSE
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )

        send_future = self.navigate_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self._on_parking_navigation_feedback,
        )
        send_future.add_done_callback(self._on_parking_navigation_goal_response)

    def _on_parking_navigation_feedback(self, feedback_msg) -> None:
        navigation_goal = self._active_navigation_goal_pose()
        if navigation_goal is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if (
            self.last_navigation_feedback_log_time_ns != 0
            and now_ns - self.last_navigation_feedback_log_time_ns < 2_000_000_000
        ):
            return
        self.last_navigation_feedback_log_time_ns = now_ns

        feedback = feedback_msg.feedback
        distance_remaining_m = float(feedback.distance_remaining)
        goal_x_m = navigation_goal['x']
        goal_y_m = navigation_goal['y']
        goal_yaw = navigation_goal['yaw']

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            self.get_logger().info(
                'Navigation feedback: '
                f'distance_remaining={distance_remaining_m:.3f} m'
            )
            return

        current_x_m = float(transform.transform.translation.x)
        current_y_m = float(transform.transform.translation.y)
        current_yaw = self._yaw_from_quaternion(transform.transform.rotation)
        xy_error_m = math.hypot(
            goal_x_m - current_x_m,
            goal_y_m - current_y_m,
        )
        yaw_error = self._normalize_angle(goal_yaw - current_yaw)
        self.get_logger().info(
            'Navigation feedback: '
            f'distance_remaining={distance_remaining_m:.3f} m, '
            f'current_base=({current_x_m:.3f}, {current_y_m:.3f}, '
            f'{current_yaw:.3f} rad), '
            f'goal_error_xy={xy_error_m:.3f} m, '
            f'goal_error_yaw={yaw_error:.3f} rad'
        )

    def _active_navigation_goal_pose(self) -> dict[str, float] | None:
        if self.state == MissionState.NAVIGATING_TO_G1_OBSERVATION_POSE:
            return self.g1_observation_pose
        if self.state == MissionState.NAVIGATING_TO_PREPARKING_POSE:
            return self.preparking_pose
        if self.state == MissionState.NAVIGATING_TO_PARKING_POSE:
            return self.selected_parking_pose
        if self.state == MissionState.RETURNING_TO_START:
            return self._start_pose_as_goal_dict()
        return None

    def _on_preparking_navigation_goal_response(self, future) -> None:
        if self.state != MissionState.NAVIGATING_TO_PREPARKING_POSE:
            return

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Pre-parking navigation failed: {error}')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'Pre-parking navigation failed; returning to start'
                )
            return

        if not goal_handle.accepted:
            self.get_logger().error('Pre-parking navigation failed: goal rejected')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'Pre-parking navigation failed; returning to start'
                )
            return

        self.get_logger().info('NavigateToPose goal accepted')
        goal_handle.get_result_async().add_done_callback(
            self._on_preparking_navigation_result
        )

    def _on_preparking_navigation_result(self, future) -> None:
        if self.state != MissionState.NAVIGATING_TO_PREPARKING_POSE:
            return

        try:
            action_result = future.result()
        except Exception as error:
            self.get_logger().error(f'Pre-parking navigation failed: {error}')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'Pre-parking navigation failed; returning to start'
                )
            return

        result = action_result.result
        if (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == NavigateToPose.Result.NONE
        ):
            self.preparking_navigation_complete = True
            self.get_logger().info('Pre-parking navigation succeeded')
            if self.preparking_pose is not None:
                self._record_navigation_pose_accuracy(
                    'Pre-parking',
                    self.preparking_pose,
                    'actual_preparking_pose',
                )
            self._log_preparking_alignment()
            self._send_selected_parking_goal()
            return

        error_msg = result.error_msg if result.error_msg else 'no error message'
        self.get_logger().error(
            'Pre-parking navigation failed: '
            f'status={action_result.status}, '
            f'error_code={result.error_code}, '
            f'error_msg={error_msg}'
        )
        if self.g1_committed:
            self._return_to_start_from_current_state(
                'Pre-parking navigation failed; returning to start'
            )

    def _log_preparking_alignment(self) -> None:
        if self.selected_parking_pose is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            self.get_logger().info('Pre-parking alignment: yaw_error_to_final=unknown')
            return

        current_yaw = self._yaw_from_quaternion(transform.transform.rotation)
        yaw_error = self._normalize_angle(
            self.selected_parking_pose['yaw'] - current_yaw
        )
        self.get_logger().info(
            f'Pre-parking alignment: yaw_error_to_final={yaw_error:.3f} rad'
        )

    def _on_parking_navigation_goal_response(self, future) -> None:
        if self.state != MissionState.NAVIGATING_TO_PARKING_POSE:
            return

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Parking navigation failed: {error}')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'Parking navigation failed; returning to start'
                )
            return

        if not goal_handle.accepted:
            self.get_logger().error('Parking navigation failed: goal rejected')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'Parking navigation failed; returning to start'
                )
            return

        self.get_logger().info('NavigateToPose goal accepted')
        goal_handle.get_result_async().add_done_callback(
            self._on_parking_navigation_result
        )

    def _on_parking_navigation_result(self, future) -> None:
        if self.state != MissionState.NAVIGATING_TO_PARKING_POSE:
            return

        try:
            action_result = future.result()
        except Exception as error:
            self.get_logger().error(f'Parking navigation failed: {error}')
            if self.g1_committed:
                self._return_to_start_from_current_state(
                    'Parking navigation failed; returning to start'
                )
            return

        result = action_result.result
        if (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == NavigateToPose.Result.NONE
        ):
            self.parking_navigation_complete = True
            self.get_logger().info('Parking navigation succeeded')
            if self.selected_parking_pose is not None:
                self._record_navigation_pose_accuracy(
                    'Final parking',
                    self.selected_parking_pose,
                    'actual_parking_pose',
                )
                self.get_logger().info(
                    'Parking base goal reached: '
                    f'base_x={self.selected_parking_pose["x"]:.3f}, '
                    f'base_y={self.selected_parking_pose["y"]:.3f}, '
                    f'yaw={self.selected_parking_pose["yaw"]:.3f} rad'
                )
                self._log_parking_geometry_diagnostics()
            self._enter_parked_state()
            return

        error_msg = result.error_msg if result.error_msg else 'no error message'
        self.get_logger().error(
            'Parking navigation failed: '
            f'status={action_result.status}, '
            f'error_code={result.error_code}, '
            f'error_msg={error_msg}'
        )
        if self.g1_committed:
            self._return_to_start_from_current_state(
                'Parking navigation failed; returning to start'
            )

    def _enter_parked_state(self) -> None:
        if self.state != MissionState.NAVIGATING_TO_PARKING_POSE:
            return

        previous_state = self.state
        self.state = MissionState.PARKED
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self.get_logger().info('Ridgeback parked successfully')
        self._start_parking_wait()

    def _start_parking_wait(self) -> None:
        if self.state != MissionState.PARKED:
            return
        if self.parking_wait_started or self.parking_wait_completed:
            return

        self.parking_wait_started = True
        self.get_logger().info(
            f'Parking wait started: {self.parking_wait_duration_s:.1f} s'
        )
        self.parking_wait_timer = self.create_timer(
            self.parking_wait_duration_s,
            self._complete_parking_wait,
        )

    def _complete_parking_wait(self) -> None:
        if self.parking_wait_timer is not None:
            self.parking_wait_timer.cancel()
            self.parking_wait_timer = None

        if self.state != MissionState.PARKED or self.parking_wait_completed:
            return

        self.parking_wait_completed = True
        self.get_logger().info('Parking wait completed')
        self._start_return_to_start_navigation()

    def _start_return_to_start_navigation(self) -> None:
        if self.state != MissionState.PARKED:
            return
        if not self.parking_navigation_complete:
            return
        if not self.parking_wait_started or not self.parking_wait_completed:
            return
        if self.start_pose is None:
            self.get_logger().error('Return-to-start navigation failed: no saved start pose')
            return
        if self.return_goal_sent or self.return_navigation_complete:
            return

        self._transition_to_return_to_start()

    def _return_to_start_from_current_state(self, reason: str) -> None:
        if self.state in (
            MissionState.RETURNING_TO_START,
            MissionState.MISSION_COMPLETE,
            MissionState.MISSION_FAILED,
        ):
            return
        self.get_logger().warn(reason)
        self._cancel_g1_localization_timeout()
        self._cancel_g1_observation_trigger()
        self._cancel_g1_centering_detection_timeout()
        self.g1_centering_goal_generation = (
            getattr(self, 'g1_centering_goal_generation', 0) + 1
        )
        self.g1_centering_goal_in_flight = False
        if self.g1_observation_validation_wait_timer is not None:
            self.g1_observation_validation_wait_timer.cancel()
            self.g1_observation_validation_wait_timer = None
        if self.preparking_validation_wait_timer is not None:
            self.preparking_validation_wait_timer.cancel()
            self.preparking_validation_wait_timer = None
        if self.nav2_wait_timer is not None:
            self.nav2_wait_timer.cancel()
            self.nav2_wait_timer = None
        if self.navigate_wait_timer is not None:
            self.navigate_wait_timer.cancel()
            self.navigate_wait_timer = None

        if self.start_pose is None:
            self.get_logger().error(
                'Return-to-start navigation failed: no saved start pose'
            )
            return

        self._transition_to_return_to_start()

    def _fail_mission(self, reason: str) -> None:
        if self.state in (
            MissionState.MISSION_COMPLETE,
            MissionState.MISSION_FAILED,
        ):
            return
        self.get_logger().error(reason)
        self._cancel_g1_localization_timeout()
        self._cancel_g1_observation_trigger()
        self._cancel_g1_centering_detection_timeout()
        self.g1_centering_goal_generation = (
            getattr(self, 'g1_centering_goal_generation', 0) + 1
        )
        self.g1_centering_goal_in_flight = False
        for timer_name in (
            'exploration_stop_timer',
            'g1_observation_validation_wait_timer',
            'preparking_validation_wait_timer',
            'nav2_wait_timer',
            'navigate_wait_timer',
            'return_navigation_wait_timer',
        ):
            timer = getattr(self, timer_name, None)
            if timer is not None:
                timer.cancel()
                setattr(self, timer_name, None)
        previous_state = self.state
        self.state = MissionState.MISSION_FAILED
        self.get_logger().error(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )

    def _transition_to_return_to_start(self) -> None:
        previous_state = self.state
        self.state = MissionState.RETURNING_TO_START
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._send_return_to_start_goal()

    def _send_return_to_start_goal(self) -> None:
        if self.state != MissionState.RETURNING_TO_START:
            return
        if self.start_pose is None:
            self.get_logger().error('Return-to-start navigation failed: no saved start pose')
            return
        if self.return_goal_sent:
            return
        if not self.navigate_to_pose_client.server_is_ready():
            if self.return_navigation_wait_timer is None:
                self.get_logger().info('Waiting for NavigateToPose action server')
                self.return_navigation_wait_timer = self.create_timer(
                    0.5,
                    self._send_return_to_start_goal,
                )
            return

        if self.return_navigation_wait_timer is not None:
            self.return_navigation_wait_timer.cancel()
            self.return_navigation_wait_timer = None

        return_goal = NavigateToPose.Goal()
        return_goal.pose = self._return_start_pose_stamped()
        start_yaw = self._yaw_from_quaternion(self.start_pose.pose.orientation)
        self.return_goal_sent = True
        self.last_navigation_feedback_log_time_ns = 0

        self.get_logger().info(
            'Returning to saved start pose: '
            f'x={self.start_pose.pose.position.x:.3f}, '
            f'y={self.start_pose.pose.position.y:.3f}, '
            f'yaw={start_yaw:.3f} rad'
        )
        self.get_logger().info('Sending return-to-start goal to Nav2')

        send_future = self.navigate_to_pose_client.send_goal_async(
            return_goal,
            feedback_callback=self._on_parking_navigation_feedback,
        )
        send_future.add_done_callback(self._on_return_navigation_goal_response)

    def _return_start_pose_stamped(self) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.start_pose.header.frame_id
        pose_stamped.pose.position.x = self.start_pose.pose.position.x
        pose_stamped.pose.position.y = self.start_pose.pose.position.y
        pose_stamped.pose.position.z = self.start_pose.pose.position.z
        pose_stamped.pose.orientation.x = self.start_pose.pose.orientation.x
        pose_stamped.pose.orientation.y = self.start_pose.pose.orientation.y
        pose_stamped.pose.orientation.z = self.start_pose.pose.orientation.z
        pose_stamped.pose.orientation.w = self.start_pose.pose.orientation.w
        return pose_stamped

    def _start_pose_as_goal_dict(self) -> dict[str, float] | None:
        if self.start_pose is None:
            return None
        return {
            'x': self.start_pose.pose.position.x,
            'y': self.start_pose.pose.position.y,
            'yaw': self._yaw_from_quaternion(self.start_pose.pose.orientation),
        }

    def _on_return_navigation_goal_response(self, future) -> None:
        if self.state != MissionState.RETURNING_TO_START:
            return

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Return-to-start navigation failed: {error}')
            return

        if not goal_handle.accepted:
            self.get_logger().error('Return-to-start navigation failed: goal rejected')
            return

        self.get_logger().info('NavigateToPose goal accepted')
        goal_handle.get_result_async().add_done_callback(
            self._on_return_navigation_result
        )

    def _on_return_navigation_result(self, future) -> None:
        if self.state != MissionState.RETURNING_TO_START:
            return

        try:
            action_result = future.result()
        except Exception as error:
            self.get_logger().error(f'Return-to-start navigation failed: {error}')
            return

        result = action_result.result
        if (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == NavigateToPose.Result.NONE
        ):
            self.return_navigation_complete = True
            self.get_logger().info('Return-to-start navigation succeeded')
            self._log_return_to_start_error()
            previous_state = self.state
            self.state = MissionState.MISSION_COMPLETE
            self.get_logger().info(
                f'Mission state transition: {previous_state.name} -> {self.state.name}'
            )
            self.get_logger().info('Returned to start; mission complete')
            self.get_logger().info('Mission complete')
            return

        error_msg = result.error_msg if result.error_msg else 'no error message'
        self.get_logger().error(
            'Return-to-start navigation failed: '
            f'status={action_result.status}, '
            f'error_code={result.error_code}, '
            f'error_msg={error_msg}'
        )

    def _log_return_to_start_error(self) -> None:
        if self.start_pose is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            self.get_logger().info('Return-to-start final error: unavailable')
            return

        current_x_m = float(transform.transform.translation.x)
        current_y_m = float(transform.transform.translation.y)
        current_yaw = self._yaw_from_quaternion(transform.transform.rotation)
        start_x_m = float(self.start_pose.pose.position.x)
        start_y_m = float(self.start_pose.pose.position.y)
        start_yaw = self._yaw_from_quaternion(self.start_pose.pose.orientation)
        xy_error_m = math.hypot(current_x_m - start_x_m, current_y_m - start_y_m)
        yaw_error = self._normalize_angle(current_yaw - start_yaw)
        self.get_logger().info(
            'Return-to-start final error: '
            f'xy={xy_error_m:.3f} m, yaw={yaw_error:.3f} rad'
        )

    def _log_g1_reference_diagnostics(self) -> None:
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return

        self.get_logger().info(
            'G1 reference diagnostics: '
            f'donut_center=({self.g1_map_x_m:.3f}, {self.g1_map_y_m:.3f})'
        )
        if not self.latest_rgb_estimate_markers:
            self.get_logger().info('G1 reference diagnostics: no RGB estimate markers available')
            self._log_g1_foot_midpoint_diagnostics()
            return

        points = self.latest_rgb_estimate_markers[:2]
        for index, (x_m, y_m, frame_id, marker_id) in enumerate(points, start=1):
            distance_m = math.hypot(x_m - self.g1_map_x_m, y_m - self.g1_map_y_m)
            self.get_logger().info(
                'G1 reference diagnostics: '
                f'red_marker_{index}=({x_m:.3f}, {y_m:.3f}), '
                f'frame={frame_id}, id={marker_id}, '
                f'distance_to_donut_center={distance_m:.3f} m'
            )

        if len(points) >= 2:
            mid_x_m = (points[0][0] + points[1][0]) / 2.0
            mid_y_m = (points[0][1] + points[1][1]) / 2.0
            midpoint_distance_m = math.hypot(
                mid_x_m - self.g1_map_x_m,
                mid_y_m - self.g1_map_y_m,
            )
            self.get_logger().info(
                'G1 reference diagnostics: '
                f'red_midpoint=({mid_x_m:.3f}, {mid_y_m:.3f}), '
                f'distance_to_donut_center={midpoint_distance_m:.3f} m'
            )

        self._log_g1_foot_midpoint_diagnostics()

    def _debug_g1_foot_points_in_map(
        self,
    ) -> tuple[tuple[float, float], tuple[float, float], float, float] | None:
        if not self.g1_foot_midpoint_debug_enabled:
            return None

        values = (
            self.g1_debug_world_x_m,
            self.g1_debug_world_y_m,
            self.g1_debug_world_yaw_rad,
            self.g1_left_foot_center_x_m,
            self.g1_left_foot_center_y_m,
            self.g1_right_foot_center_x_m,
            self.g1_right_foot_center_y_m,
        )
        if not all(math.isfinite(value) for value in values):
            return None

        cos_yaw = math.cos(self.g1_debug_world_yaw_rad)
        sin_yaw = math.sin(self.g1_debug_world_yaw_rad)

        def transform(local_x_m: float, local_y_m: float) -> tuple[float, float]:
            return (
                self.g1_debug_world_x_m + cos_yaw * local_x_m - sin_yaw * local_y_m,
                self.g1_debug_world_y_m + sin_yaw * local_x_m + cos_yaw * local_y_m,
            )

        left = transform(self.g1_left_foot_center_x_m, self.g1_left_foot_center_y_m)
        right = transform(self.g1_right_foot_center_x_m, self.g1_right_foot_center_y_m)
        mid_x_m = (left[0] + right[0]) / 2.0
        mid_y_m = (left[1] + right[1]) / 2.0
        return left, right, mid_x_m, mid_y_m

    def _log_g1_foot_midpoint_diagnostics(self) -> None:
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return

        foot_points = self._debug_g1_foot_points_in_map()
        if foot_points is None:
            return

        left, right, mid_x_m, mid_y_m = foot_points
        error_x_m = self.g1_map_x_m - mid_x_m
        error_y_m = self.g1_map_y_m - mid_y_m
        error_distance_m = math.hypot(error_x_m, error_y_m)
        bbox_text = 'bbox=n/a'
        if self.latest_g1_detection_bbox is not None:
            x1, y1, x2, y2 = self.latest_g1_detection_bbox
            bottom_u = (x1 + x2) / 2.0
            bottom_v = y2
            bbox_text = (
                f'bbox=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f}), '
                f'bottom_center=({bottom_u:.1f}, {bottom_v:.1f})'
            )
        self.get_logger().info(
            'G1 foot midpoint diagnostics: '
            f'left_foot=({left[0]:.3f}, {left[1]:.3f}), '
            f'right_foot=({right[0]:.3f}, {right[1]:.3f}), '
            f'foot_midpoint=({mid_x_m:.3f}, {mid_y_m:.3f}), '
            f'mission_g1_center=({self.g1_map_x_m:.3f}, {self.g1_map_y_m:.3f}), '
            f'center_error=({error_x_m:.3f}, {error_y_m:.3f}), '
            f'center_error_distance={error_distance_m:.3f} m, '
            f'{bbox_text}'
        )

    def _log_g1_sensor_foot_midpoint_diagnostics(self) -> None:
        if (
            self.g1_sensor_foot_midpoint_x_map_m is None
            or self.g1_sensor_foot_midpoint_y_map_m is None
            or self.g1_sensor_foot_1_map_m is None
            or self.g1_sensor_foot_2_map_m is None
        ):
            return

        self.get_logger().info(
            'Estimated G1 lower-leg/foot clusters: '
            f'foot_1=({self.g1_sensor_foot_1_map_m[0]:.3f}, '
            f'{self.g1_sensor_foot_1_map_m[1]:.3f}), '
            f'foot_2=({self.g1_sensor_foot_2_map_m[0]:.3f}, '
            f'{self.g1_sensor_foot_2_map_m[1]:.3f})'
        )

        foot_points = self._debug_g1_foot_points_in_map()
        if foot_points is None:
            self.get_logger().info(
                'New sensor foot midpoint: '
                f'x={self.g1_sensor_foot_midpoint_x_map_m:.3f}, '
                f'y={self.g1_sensor_foot_midpoint_y_map_m:.3f}'
            )
            return

        _, _, truth_x_m, truth_y_m = foot_points
        new_error_x_m = self.g1_sensor_foot_midpoint_x_map_m - truth_x_m
        new_error_y_m = self.g1_sensor_foot_midpoint_y_map_m - truth_y_m
        new_error_distance_m = math.hypot(new_error_x_m, new_error_y_m)

        old_text = 'old_ground_estimate_error=n/a'
        if (
            self.g1_old_ground_estimate_x_map_m is not None
            and self.g1_old_ground_estimate_y_map_m is not None
        ):
            old_error_x_m = self.g1_old_ground_estimate_x_map_m - truth_x_m
            old_error_y_m = self.g1_old_ground_estimate_y_map_m - truth_y_m
            old_error_distance_m = math.hypot(old_error_x_m, old_error_y_m)
            old_text = (
                'old_ground_estimate='
                f'({self.g1_old_ground_estimate_x_map_m:.3f}, '
                f'{self.g1_old_ground_estimate_y_map_m:.3f}), '
                f'old_error=({old_error_x_m:.3f}, {old_error_y_m:.3f}), '
                f'old_error_distance={old_error_distance_m:.3f} m'
            )

        self.get_logger().info(
            'G1 foot-midpoint localization: '
            f'foot1=({self.g1_sensor_foot_1_map_m[0]:.3f}, '
            f'{self.g1_sensor_foot_1_map_m[1]:.3f}), '
            f'foot2=({self.g1_sensor_foot_2_map_m[0]:.3f}, '
            f'{self.g1_sensor_foot_2_map_m[1]:.3f}), '
            f'sensor_midpoint=({self.g1_sensor_foot_midpoint_x_map_m:.3f}, '
            f'{self.g1_sensor_foot_midpoint_y_map_m:.3f}), '
            f'mission_center=({self.g1_map_x_m:.3f}, {self.g1_map_y_m:.3f})'
        )
        self.get_logger().info(
            'G1 foot-midpoint validation: '
            f'ground_truth=({truth_x_m:.3f}, {truth_y_m:.3f}), '
            f'{old_text}, '
            f'sensor_midpoint=({self.g1_sensor_foot_midpoint_x_map_m:.3f}, '
            f'{self.g1_sensor_foot_midpoint_y_map_m:.3f}), '
            f'center_error=({new_error_x_m:.3f}, {new_error_y_m:.3f}), '
            f'center_error_distance={new_error_distance_m:.3f} m'
        )

    def _log_parking_geometry_diagnostics(self) -> None:
        if self.selected_parking_pose is None:
            return

        selected_base_x_m = self.selected_parking_pose['x']
        selected_base_y_m = self.selected_parking_pose['y']
        selected_yaw = self.selected_parking_pose['yaw']
        selected_task_x_m = self.selected_parking_pose['task_x']
        selected_task_y_m = self.selected_parking_pose['task_y']
        recomputed_task_x_m, recomputed_task_y_m = self._task_point_from_base_pose(
            selected_base_x_m,
            selected_base_y_m,
            selected_yaw,
        )
        selected_task_error_m = math.hypot(
            recomputed_task_x_m - selected_task_x_m,
            recomputed_task_y_m - selected_task_y_m,
        )
        self.get_logger().info(
            'Parking geometry diagnostics: '
            f'selected_base=({selected_base_x_m:.3f}, '
            f'{selected_base_y_m:.3f}, {selected_yaw:.3f} rad), '
            f'selected_task=({selected_task_x_m:.3f}, {selected_task_y_m:.3f}), '
            f'task_recompute_error={selected_task_error_m:.6f} m'
        )

        base_tf = self._lookup_transform_optional(self.map_frame, self.robot_base_frame)
        rear_bin_tf = self._lookup_rear_bin_in_map()
        base_to_rear_tf = self._lookup_transform_optional(
            self.robot_base_frame,
            self.rear_bin_frame,
        )
        if base_to_rear_tf is None and self.rear_bin_frame != 'rear_bin_floor_link':
            base_to_rear_tf = self._lookup_transform_optional(
                self.robot_base_frame,
                'rear_bin_floor_link',
            )

        if base_tf is not None:
            actual_base_x_m = float(base_tf.transform.translation.x)
            actual_base_y_m = float(base_tf.transform.translation.y)
            actual_base_yaw = self._yaw_from_quaternion(base_tf.transform.rotation)
            base_xy_error_m = math.hypot(
                actual_base_x_m - selected_base_x_m,
                actual_base_y_m - selected_base_y_m,
            )
            base_yaw_error = self._normalize_angle(actual_base_yaw - selected_yaw)
            yaw_task_shift_m = 2.0 * math.hypot(
                self.task_point_x_base_m,
                self.task_point_y_base_m,
            ) * abs(math.sin(base_yaw_error / 2.0))
            self.get_logger().info(
                'Parking geometry diagnostics: '
                f'actual_base=({actual_base_x_m:.3f}, '
                f'{actual_base_y_m:.3f}, {actual_base_yaw:.3f} rad), '
                f'base_xy_error={base_xy_error_m:.3f} m, '
                f'base_yaw_error={base_yaw_error:.3f} rad, '
                f'yaw_induced_task_shift={yaw_task_shift_m:.3f} m'
            )
        else:
            self.get_logger().info('Parking geometry diagnostics: actual base TF unavailable')

        if base_to_rear_tf is not None:
            offset = base_to_rear_tf.transform.translation
            self.get_logger().info(
                'Parking geometry diagnostics: '
                f'base_link_to_rear_bin=({offset.x:.3f}, {offset.y:.3f}, '
                f'{offset.z:.3f})'
            )
        else:
            self.get_logger().info(
                'Parking geometry diagnostics: base_link -> rear_bin_floor_link TF unavailable'
            )

        if rear_bin_tf is None:
            self.get_logger().info(
                'Parking geometry diagnostics: map -> rear_bin_floor_link TF unavailable'
            )
            return

        rear_bin_x_m = float(rear_bin_tf.transform.translation.x)
        rear_bin_y_m = float(rear_bin_tf.transform.translation.y)
        task_xy_error_m = math.hypot(
            rear_bin_x_m - selected_task_x_m,
            rear_bin_y_m - selected_task_y_m,
        )
        self.get_logger().info(
            'Parking geometry diagnostics: '
            f'actual_rear_bin=({rear_bin_x_m:.3f}, {rear_bin_y_m:.3f}), '
            f'task_xy_error={task_xy_error_m:.3f} m'
        )

        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return
        theoretical_distance_m = math.hypot(
            selected_task_x_m - self.g1_map_x_m,
            selected_task_y_m - self.g1_map_y_m,
        )
        actual_distance_m = math.hypot(
            rear_bin_x_m - self.g1_map_x_m,
            rear_bin_y_m - self.g1_map_y_m,
        )
        inside = self.reach_min_m <= actual_distance_m <= self.reach_max_m
        self.get_logger().info(
            'Parking geometry diagnostics: '
            f'theoretical_task_distance_to_g1={theoretical_distance_m:.3f} m, '
            f'actual_bin_distance_to_g1={actual_distance_m:.3f} m, '
            f'annulus_result={"INSIDE" if inside else "OUTSIDE"}'
        )

    def _lookup_rear_bin_in_map(self):
        transform = self._lookup_transform_optional(self.map_frame, self.rear_bin_frame)
        if transform is not None:
            return transform
        if self.rear_bin_frame != 'rear_bin_floor_link':
            return self._lookup_transform_optional(self.map_frame, 'rear_bin_floor_link')
        return None

    def _current_base_pose_in_map(self) -> tuple[float, float, float] | None:
        transform = self._lookup_transform_optional(self.map_frame, self.robot_base_frame)
        if transform is None:
            return None

        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            self._yaw_from_quaternion(transform.transform.rotation),
        )

    def _record_navigation_pose_accuracy(
        self,
        label: str,
        target_pose: dict[str, float],
        storage_attribute: str,
    ) -> None:
        actual_pose = self._current_base_pose_in_map()
        if actual_pose is None:
            self.get_logger().warn(
                f'{label} accuracy unavailable: map -> {self.robot_base_frame} TF unavailable'
            )
            return

        actual_x_m, actual_y_m, actual_yaw = actual_pose
        target_x_m = target_pose['x']
        target_y_m = target_pose['y']
        target_yaw = target_pose['yaw']
        dx_m = actual_x_m - target_x_m
        dy_m = actual_y_m - target_y_m
        xy_error_m = math.hypot(dx_m, dy_m)
        yaw_error = self._normalize_angle(actual_yaw - target_yaw)
        diagnostic = {
            'target_x': target_x_m,
            'target_y': target_y_m,
            'target_yaw': target_yaw,
            'actual_x': actual_x_m,
            'actual_y': actual_y_m,
            'actual_yaw': actual_yaw,
            'dx': dx_m,
            'dy': dy_m,
            'xy_error': xy_error_m,
            'yaw_error': yaw_error,
        }
        setattr(self, storage_attribute, diagnostic)
        self.get_logger().info(
            f'{label} accuracy: '
            f'target_x={target_x_m:.3f}, '
            f'target_y={target_y_m:.3f}, '
            f'target_yaw={target_yaw:.3f} rad, '
            f'actual_x={actual_x_m:.3f}, '
            f'actual_y={actual_y_m:.3f}, '
            f'actual_yaw={actual_yaw:.3f} rad, '
            f'dx={dx_m:.3f}, '
            f'dy={dy_m:.3f}, '
            f'xy_error={xy_error_m:.3f} m, '
            f'yaw_error={yaw_error:.3f} rad'
        )
        self._publish_thesis_parking_markers()

    def _lookup_transform_optional(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

    @staticmethod
    def _path_length_m(path) -> float:
        return planar_path_length(
            (pose.pose.position.x, pose.pose.position.y)
            for pose in path.poses
        )

    def _path_footprint_rejection_reason(
        self,
        path,
        grid: OccupancyGrid,
    ) -> str | None:
        if path is None or not path.poses:
            return 'empty_path'
        if path.header.frame_id and path.header.frame_id != self.map_frame:
            return 'wrong_frame'

        spacing_m = min(
            self.parking_path_sample_spacing_m,
            max(float(grid.info.resolution), 0.01),
        )
        poses = path.poses
        for index, current in enumerate(poses):
            current_x = float(current.pose.position.x)
            current_y = float(current.pose.position.y)
            current_yaw = self._yaw_from_quaternion(current.pose.orientation)
            if index + 1 < len(poses):
                following = poses[index + 1]
                next_x = float(following.pose.position.x)
                next_y = float(following.pose.position.y)
                next_yaw = self._yaw_from_quaternion(following.pose.orientation)
            else:
                next_x = current_x
                next_y = current_y
                next_yaw = current_yaw

            segment_length_m = math.hypot(next_x - current_x, next_y - current_y)
            sample_count = max(1, int(math.ceil(segment_length_m / spacing_m)))
            yaw_delta = self._normalize_angle(next_yaw - current_yaw)
            for sample_index in range(sample_count):
                ratio = float(sample_index) / float(sample_count)
                sample_x = current_x + (next_x - current_x) * ratio
                sample_y = current_y + (next_y - current_y) * ratio
                sample_yaw = self._normalize_angle(current_yaw + yaw_delta * ratio)
                polygon = self._rotated_footprint_vertices(
                    sample_x,
                    sample_y,
                    sample_yaw,
                )
                reason = self._rotated_footprint_rejection_reason(polygon, grid)
                if reason is not None:
                    return f'unsafe_path_{reason}'

        final_pose = poses[-1].pose
        final_polygon = self._rotated_footprint_vertices(
            float(final_pose.position.x),
            float(final_pose.position.y),
            self._yaw_from_quaternion(final_pose.orientation),
        )
        final_reason = self._rotated_footprint_rejection_reason(final_polygon, grid)
        return None if final_reason is None else f'unsafe_path_{final_reason}'

    def _reset_parking_pipeline_timing(self) -> None:
        self.parking_pipeline_start_s = time.monotonic()
        self.parking_pipeline_stage_starts = {}
        self.parking_pipeline_durations = {
            'exploration_stop': 0.0,
            'g1_localization': 0.0,
            'map_transform': 0.0,
            'donut_generation': 0.0,
            'occupancy_filter': 0.0,
            'base_pose_generation': 0.0,
            'footprint_check': 0.0,
            'nav2_feasibility': 0.0,
            'candidate_selection': 0.0,
            'preparking_validation': 0.0,
        }
        self.parking_pipeline_candidates_checked = 0

    def _timing_start(self, stage: str) -> None:
        if self.parking_pipeline_start_s is None:
            return
        self.parking_pipeline_stage_starts[stage] = time.monotonic()

    def _timing_stop(self, stage: str) -> None:
        if self.parking_pipeline_start_s is None:
            return
        start_s = self.parking_pipeline_stage_starts.pop(stage, None)
        if start_s is None:
            return
        self._timing_add_duration(stage, time.monotonic() - start_s)

    def _timing_add_duration(self, stage: str, duration_s: float) -> None:
        if self.parking_pipeline_start_s is None:
            return
        self.parking_pipeline_durations[stage] = (
            self.parking_pipeline_durations.get(stage, 0.0) + duration_s
        )

    def _log_parking_pipeline_timing(self) -> None:
        if self.parking_pipeline_start_s is None:
            return

        durations = self.parking_pipeline_durations
        candidates_checked = self.parking_pipeline_candidates_checked
        nav2_duration_s = durations.get('nav2_feasibility', 0.0)
        avg_path_request_s = (
            nav2_duration_s / candidates_checked
            if candidates_checked > 0
            else 0.0
        )
        total_s = time.monotonic() - self.parking_pipeline_start_s
        self.get_logger().info(
            '\n===== PARKING PIPELINE TIMING =====\n'
            f'exploration_stop:      {durations.get("exploration_stop", 0.0):.2f} s\n'
            f'g1_localization:       {durations.get("g1_localization", 0.0):.2f} s\n'
            f'map_transform:         {durations.get("map_transform", 0.0):.2f} s\n'
            f'donut_generation:      {durations.get("donut_generation", 0.0):.2f} s\n'
            f'occupancy_filter:      {durations.get("occupancy_filter", 0.0):.2f} s\n'
            f'base_pose_generation:  {durations.get("base_pose_generation", 0.0):.2f} s\n'
            f'footprint_check:       {durations.get("footprint_check", 0.0):.2f} s\n'
            f'nav2_feasibility:      {nav2_duration_s:.2f} s\n'
            f'candidates_checked:    {candidates_checked}\n'
            f'avg_path_request:      {avg_path_request_s:.3f} s\n'
            f'candidate_selection:   {durations.get("candidate_selection", 0.0):.2f} s\n'
            f'preparking_validation: {durations.get("preparking_validation", 0.0):.2f} s\n'
            f'\nTOTAL G1 -> PREPARKING: {total_s:.2f} s'
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y ** 2
            + quaternion.z ** 2
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return normalize_angle(angle_rad)

    def _unsafe_grid_cell_centers(self, grid: OccupancyGrid) -> list[Point]:
        unsafe_points = []
        width = int(grid.info.width)
        height = int(grid.info.height)
        for row in range(height):
            row_offset = row * width
            for col in range(width):
                value = int(grid.data[row_offset + col])
                if value == -1 or value >= self.occupied_threshold:
                    unsafe_points.append(self._grid_cell_center_to_map(grid, col, row))
        return unsafe_points

    def _rotated_footprint_clearance_m(
        self,
        candidate: tuple[float, float, float],
        grid: OccupancyGrid,
        unsafe_cell_centers: list[Point],
    ) -> float:
        x_m, y_m, yaw = candidate
        polygon = self._rotated_footprint_vertices(x_m, y_m, yaw)
        clearance_m = self._map_boundary_clearance_m(polygon, grid)
        half_cell_diagonal_m = math.sqrt(2.0) * float(grid.info.resolution) * 0.5

        for unsafe_center in unsafe_cell_centers:
            distance_m = self._point_to_polygon_distance_m(
                unsafe_center.x,
                unsafe_center.y,
                polygon,
            )
            clearance_m = min(
                clearance_m,
                max(0.0, distance_m - half_cell_diagonal_m),
            )
        return clearance_m

    def _map_boundary_clearance_m(
        self,
        polygon: list[Point],
        grid: OccupancyGrid,
    ) -> float:
        local_vertices = [
            self._map_to_grid_local(grid, vertex.x, vertex.y)
            for vertex in polygon
        ]
        resolution = float(grid.info.resolution)
        map_width_m = float(grid.info.width) * resolution
        map_height_m = float(grid.info.height) * resolution
        min_local_x = min(point[0] for point in local_vertices)
        max_local_x = max(point[0] for point in local_vertices)
        min_local_y = min(point[1] for point in local_vertices)
        max_local_y = max(point[1] for point in local_vertices)
        return max(
            0.0,
            min(
                min_local_x,
                min_local_y,
                map_width_m - max_local_x,
                map_height_m - max_local_y,
            ),
        )

    def _point_to_polygon_distance_m(
        self,
        point_x_m: float,
        point_y_m: float,
        polygon: list[Point],
    ) -> float:
        if self._point_in_polygon(point_x_m, point_y_m, polygon):
            return 0.0

        distance_m = math.inf
        previous = polygon[-1]
        for current in polygon:
            distance_m = min(
                distance_m,
                self._point_to_segment_distance_m(
                    point_x_m,
                    point_y_m,
                    previous.x,
                    previous.y,
                    current.x,
                    current.y,
                ),
            )
            previous = current
        return distance_m

    @staticmethod
    def _point_to_segment_distance_m(
        point_x_m: float,
        point_y_m: float,
        start_x_m: float,
        start_y_m: float,
        end_x_m: float,
        end_y_m: float,
    ) -> float:
        segment_x = end_x_m - start_x_m
        segment_y = end_y_m - start_y_m
        segment_length_squared = segment_x ** 2 + segment_y ** 2
        if segment_length_squared == 0.0:
            return math.hypot(point_x_m - start_x_m, point_y_m - start_y_m)

        t = (
            (point_x_m - start_x_m) * segment_x
            + (point_y_m - start_y_m) * segment_y
        ) / segment_length_squared
        t = min(1.0, max(0.0, t))
        projection_x = start_x_m + t * segment_x
        projection_y = start_y_m + t * segment_y
        return math.hypot(point_x_m - projection_x, point_y_m - projection_y)

    def _selected_parking_pose_markers(
        self,
        selected_pose: dict[str, float],
    ) -> MarkerArray:
        marker_array = MarkerArray()
        marker_array.markers.append(
            self._selected_parking_pose_arrow_marker(selected_pose)
        )
        marker_array.markers.append(
            self._selected_parking_pose_footprint_marker(selected_pose)
        )
        marker_array.markers.append(
            self._selected_parking_task_point_marker(selected_pose)
        )
        marker_array.markers.append(
            self._selected_parking_task_offset_marker(selected_pose)
        )
        if self.preparking_pose is not None:
            marker_array.markers.append(
                self._preparking_pose_arrow_marker(self.preparking_pose)
            )
            marker_array.markers.append(
                self._preparking_pose_footprint_marker(self.preparking_pose)
            )
        return marker_array

    def _publish_thesis_parking_markers(self) -> None:
        if self.selected_parking_pose is None:
            return

        marker_array = MarkerArray()
        marker_array.markers.extend([
            self._thesis_selected_footprint_fill_marker(self.selected_parking_pose),
            self._thesis_selected_footprint_outline_marker(self.selected_parking_pose),
            self._thesis_selected_orientation_arrow_marker(self.selected_parking_pose),
            self._thesis_selected_task_point_marker(self.selected_parking_pose),
            self._thesis_selected_task_offset_marker(self.selected_parking_pose),
        ])
        if self.preparking_pose is not None:
            marker_array.markers.extend([
                self._thesis_preparking_arrow_marker(self.preparking_pose),
                self._thesis_preparking_footprint_marker(self.preparking_pose),
            ])
        if self.actual_preparking_pose is not None:
            marker_array.markers.extend(
                self._thesis_actual_stopped_pose_markers(
                    self.actual_preparking_pose,
                    'thesis_actual_preparking_pose',
                    1.0,
                    0.0,
                    1.0,
                    0.66,
                )
            )
        if self.actual_parking_pose is not None:
            marker_array.markers.extend(
                self._thesis_actual_stopped_pose_markers(
                    self.actual_parking_pose,
                    'thesis_actual_parking_pose',
                    1.0,
                    0.0,
                    0.45,
                    0.72,
                )
            )

        self.thesis_parking_publisher.publish(marker_array)

    def _thesis_selected_footprint_fill_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'thesis_selected_parking_pose'
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.72
        marker.color.b = 0.0
        marker.color.a = 0.42
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        vertices = self._rotated_footprint_vertices(
            selected_pose['x'],
            selected_pose['y'],
            selected_pose['yaw'],
        )
        if len(vertices) == 4:
            marker.points.extend([
                Point(x=vertices[0].x, y=vertices[0].y, z=0.18),
                Point(x=vertices[1].x, y=vertices[1].y, z=0.18),
                Point(x=vertices[2].x, y=vertices[2].y, z=0.18),
                Point(x=vertices[0].x, y=vertices[0].y, z=0.18),
                Point(x=vertices[2].x, y=vertices[2].y, z=0.18),
                Point(x=vertices[3].x, y=vertices[3].y, z=0.18),
            ])
        return marker

    def _thesis_selected_footprint_outline_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'thesis_selected_parking_pose'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.color.r = 1.0
        marker.color.g = 0.82
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        vertices = self._rotated_footprint_vertices(
            selected_pose['x'],
            selected_pose['y'],
            selected_pose['yaw'],
        )
        marker.points = [
            Point(x=vertex.x, y=vertex.y, z=0.22)
            for vertex in vertices
        ]
        if vertices:
            marker.points.append(Point(x=vertices[0].x, y=vertices[0].y, z=0.22))
        return marker

    def _thesis_selected_orientation_arrow_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = self._selected_parking_pose_arrow_marker(selected_pose)
        marker.ns = 'thesis_selected_parking_pose'
        marker.id = 2
        marker.pose.position.z = 0.58
        marker.scale.x = 0.58
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.a = 1.0
        return marker

    def _thesis_selected_task_point_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = self._selected_parking_task_point_marker(selected_pose)
        marker.ns = 'thesis_selected_parking_pose'
        marker.id = 3
        marker.pose.position.z = 0.60
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        return marker

    def _thesis_selected_task_offset_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = self._selected_parking_task_offset_marker(selected_pose)
        marker.ns = 'thesis_selected_parking_pose'
        marker.id = 4
        marker.scale.x = 0.03
        marker.points = [
            Point(x=selected_pose['x'], y=selected_pose['y'], z=0.60),
            Point(x=selected_pose['task_x'], y=selected_pose['task_y'], z=0.60),
        ]
        return marker

    def _thesis_preparking_arrow_marker(
        self,
        preparking_pose: dict[str, float],
    ) -> Marker:
        marker = self._preparking_pose_arrow_marker(preparking_pose)
        marker.ns = 'thesis_preparking_pose'
        marker.id = 0
        marker.pose.position.z = 0.42
        marker.scale.x = 0.38
        marker.scale.y = 0.07
        marker.scale.z = 0.07
        marker.color.a = 0.9
        return marker

    def _thesis_preparking_footprint_marker(
        self,
        preparking_pose: dict[str, float],
    ) -> Marker:
        marker = self._preparking_pose_footprint_marker(preparking_pose)
        marker.ns = 'thesis_preparking_pose'
        marker.id = 1
        marker.scale.x = 0.03
        marker.color.a = 0.85
        marker.points = [
            Point(x=point.x, y=point.y, z=0.36)
            for point in marker.points
        ]
        return marker

    def _selected_parking_pose_arrow_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        yaw = selected_pose['yaw']
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'selected_parking_pose'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = selected_pose['x']
        marker.pose.position.y = selected_pose['y']
        marker.pose.position.z = 0.48
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 0.50
        marker.scale.y = 0.10
        marker.scale.z = 0.10
        marker.color.r = 1.0
        marker.color.g = 0.72
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _selected_parking_pose_footprint_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'selected_parking_pose'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.72
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        vertices = self._rotated_footprint_vertices(
            selected_pose['x'],
            selected_pose['y'],
            selected_pose['yaw'],
        )
        marker.points = [
            Point(x=vertex.x, y=vertex.y, z=0.44)
            for vertex in vertices
        ]
        if vertices:
            marker.points.append(Point(x=vertices[0].x, y=vertices[0].y, z=0.44))
        return marker

    def _selected_parking_task_point_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'selected_parking_pose'
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = selected_pose['task_x']
        marker.pose.position.y = selected_pose['task_y']
        marker.pose.position.z = 0.50
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.14
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _selected_parking_task_offset_marker(
        self,
        selected_pose: dict[str, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'selected_parking_pose'
        marker.id = 3
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.025
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.points = [
            Point(x=selected_pose['x'], y=selected_pose['y'], z=0.50),
            Point(x=selected_pose['task_x'], y=selected_pose['task_y'], z=0.50),
        ]
        return marker

    def _preparking_pose_arrow_marker(
        self,
        preparking_pose: dict[str, float],
    ) -> Marker:
        yaw = preparking_pose['yaw']
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'preparking_pose'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = preparking_pose['x']
        marker.pose.position.y = preparking_pose['y']
        marker.pose.position.z = 0.46
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 0.42
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.0
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _preparking_pose_footprint_marker(
        self,
        preparking_pose: dict[str, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'preparking_pose'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 0.35
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        vertices = self._rotated_footprint_vertices(
            preparking_pose['x'],
            preparking_pose['y'],
            preparking_pose['yaw'],
        )
        marker.points = [
            Point(x=vertex.x, y=vertex.y, z=0.40)
            for vertex in vertices
        ]
        if vertices:
            marker.points.append(Point(x=vertices[0].x, y=vertices[0].y, z=0.40))
        return marker

    def _thesis_actual_stopped_pose_markers(
        self,
        pose_diagnostic: dict[str, float],
        namespace: str,
        red: float,
        green: float,
        blue: float,
        z_m: float,
    ) -> list[Marker]:
        actual_x_m = pose_diagnostic['actual_x']
        actual_y_m = pose_diagnostic['actual_y']
        actual_yaw = pose_diagnostic['actual_yaw']

        footprint = Marker()
        footprint.header.frame_id = self.map_frame
        footprint.header.stamp = self.get_clock().now().to_msg()
        footprint.ns = namespace
        footprint.id = 0
        footprint.type = Marker.LINE_STRIP
        footprint.action = Marker.ADD
        footprint.pose.orientation.w = 1.0
        footprint.scale.x = 0.055
        footprint.color.r = red
        footprint.color.g = green
        footprint.color.b = blue
        footprint.color.a = 1.0
        footprint.lifetime.sec = 0
        footprint.lifetime.nanosec = 0

        vertices = self._rotated_footprint_vertices(actual_x_m, actual_y_m, actual_yaw)
        footprint.points = [
            Point(x=vertex.x, y=vertex.y, z=z_m)
            for vertex in vertices
        ]
        if vertices:
            footprint.points.append(Point(x=vertices[0].x, y=vertices[0].y, z=z_m))

        arrow = Marker()
        arrow.header.frame_id = self.map_frame
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = namespace
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = actual_x_m
        arrow.pose.position.y = actual_y_m
        arrow.pose.position.z = z_m + 0.08
        arrow.pose.orientation.z = math.sin(actual_yaw / 2.0)
        arrow.pose.orientation.w = math.cos(actual_yaw / 2.0)
        arrow.scale.x = 0.42
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.r = red
        arrow.color.g = green
        arrow.color.b = blue
        arrow.color.a = 1.0
        arrow.lifetime.sec = 0
        arrow.lifetime.nanosec = 0

        return [footprint, arrow]

    def _candidate_pose_stamped(self, x_m: float, y_m: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _nav2_feasible_parking_pose_markers(
        self,
        reachable_candidates: list[tuple[float, float, float]],
        unreachable_candidates: list[tuple[float, float, float]],
    ) -> MarkerArray:
        marker_array = MarkerArray()
        for marker_id, (x_m, y_m, yaw) in enumerate(reachable_candidates):
            marker_array.markers.append(
                self._nav2_reachable_arrow_marker(marker_id, x_m, y_m, yaw)
            )
        marker_array.markers.append(
            self._nav2_unreachable_marker(100000, unreachable_candidates)
        )
        return marker_array

    def _nav2_reachable_arrow_marker(
        self,
        marker_id: int,
        x_m: float,
        y_m: float,
        yaw: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'nav2_feasible_parking_poses_reachable'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x_m
        marker.pose.position.y = y_m
        marker.pose.position.z = 0.34
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 0.32
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _nav2_unreachable_marker(
        self,
        marker_id: int,
        unreachable_candidates: list[tuple[float, float, float]],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'nav2_feasible_parking_poses_unreachable'
        marker.id = marker_id
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.06
        marker.color.r = 0.45
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.95
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.points = [
            Point(x=x_m, y=y_m, z=0.30)
            for x_m, y_m, _ in unreachable_candidates
        ]
        return marker

    def _rotated_footprint_vertices(
        self,
        center_x_m: float,
        center_y_m: float,
        yaw: float,
    ) -> list[Point]:
        return [
            Point(x=x_m, y=y_m, z=0.0)
            for x_m, y_m in rotated_footprint_vertices(
                self.footprint_points,
                center_x_m,
                center_y_m,
                yaw,
            )
        ]

    def _rotated_footprint_rejection_reason(
        self,
        polygon: list[Point],
        grid: OccupancyGrid,
    ) -> str | None:
        if len(polygon) < 3:
            return 'other'

        local_vertices = [
            self._map_to_grid_local(grid, vertex.x, vertex.y)
            for vertex in polygon
        ]

        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        map_width_m = float(width) * resolution
        map_height_m = float(height) * resolution
        for local_x, local_y in local_vertices:
            if (
                local_x < 0.0
                or local_y < 0.0
                or local_x >= map_width_m
                or local_y >= map_height_m
            ):
                return 'out_of_bounds'

        min_local_x = min(point[0] for point in local_vertices)
        max_local_x = max(point[0] for point in local_vertices)
        min_local_y = min(point[1] for point in local_vertices)
        max_local_y = max(point[1] for point in local_vertices)
        min_col = max(0, int(math.floor(min_local_x / resolution)) - 1)
        max_col = min(width - 1, int(math.ceil(max_local_x / resolution)) + 1)
        min_row = max(0, int(math.floor(min_local_y / resolution)) - 1)
        max_row = min(height - 1, int(math.ceil(max_local_y / resolution)) + 1)

        half_cell_diagonal = math.sqrt(2.0) * resolution * 0.5
        has_occupied = False
        has_unknown = False
        has_other = False
        for row in range(min_row, max_row + 1):
            row_offset = row * width
            for col in range(min_col, max_col + 1):
                cell_center = self._grid_cell_center_to_map(grid, col, row)
                if (
                    self._point_to_polygon_distance_m(
                        cell_center.x,
                        cell_center.y,
                        polygon,
                    ) > half_cell_diagonal
                ):
                    continue

                value_index = row_offset + col
                if value_index >= len(grid.data):
                    return 'out_of_bounds'
                value = int(grid.data[value_index])
                if value >= self.occupied_threshold:
                    has_occupied = True
                elif value == -1:
                    has_unknown = True
                elif value < 0:
                    has_other = True

        if has_occupied:
            return 'occupied'
        if has_unknown:
            return 'unknown'
        if has_other:
            return 'other'
        return None

    @staticmethod
    def _parking_candidate_status_for_rejection(reason: str) -> str:
        if reason in ('occupied', 'unsafe_path_occupied'):
            return 'collision'
        return 'rejected'

    @staticmethod
    def _parking_candidate_color(status: str) -> tuple[float, float, float, float]:
        return {
            'valid': (0.10, 0.90, 0.20, 0.95),
            'collision': (0.95, 0.05, 0.03, 0.95),
            'rejected': (1.00, 0.78, 0.05, 0.95),
            'selected': (0.05, 0.35, 1.00, 1.00),
        }.get(status, (0.55, 0.55, 0.58, 0.85))

    def _register_parking_candidate_decision(
        self,
        candidate: tuple[float, float, float],
        status: str,
        reason: str,
    ) -> None:
        if candidate in self.parking_candidate_decisions:
            self._update_parking_candidate_decision(
                candidate,
                status=status,
                reason=reason,
            )
            return
        self.parking_candidate_decisions[candidate] = ParkingCandidateDecision(
            marker_id=len(self.parking_candidate_decisions),
            candidate=candidate,
            status=status,
            reason=reason,
        )

    def _update_parking_candidate_decision(
        self,
        candidate: tuple[float, float, float],
        status: str | None = None,
        reason: str | None = None,
        path_length_m: float | None = None,
        preference_label: str | None = None,
        reach_error_m: float | None = None,
        clearance_m: float | None = None,
    ) -> None:
        decision = self.parking_candidate_decisions.get(candidate)
        if decision is None:
            return
        if status is not None:
            decision.status = status
        if reason is not None:
            decision.reason = reason
        if path_length_m is not None:
            decision.path_length_m = path_length_m
        if preference_label is not None:
            decision.preference_label = preference_label
        if reach_error_m is not None:
            decision.reach_error_m = reach_error_m
        if clearance_m is not None:
            decision.clearance_m = clearance_m

    def _publish_parking_candidate_decisions(self) -> None:
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.map_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        if self.parking_candidate_arrows_enabled:
            for decision in sorted(
                self.parking_candidate_decisions.values(),
                key=lambda item: item.marker_id,
            ):
                marker_array.markers.append(
                    self._parking_candidate_decision_arrow(decision)
                )
                if (
                    self.parking_candidate_labels_enabled
                    or self.parking_candidate_scores_enabled
                    or decision.status == 'selected'
                ):
                    marker_array.markers.append(
                        self._parking_candidate_decision_label(decision)
                    )

        self.parking_pose_candidates_publisher.publish(marker_array)

    def _parking_candidate_decision_arrow(
        self,
        decision: ParkingCandidateDecision,
    ) -> Marker:
        x_m, y_m, yaw = decision.candidate
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'parking_candidate_decisions'
        marker.id = decision.marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x_m
        marker.pose.position.y = y_m
        marker.pose.position.z = 0.30
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        selected = decision.status == 'selected'
        marker.scale.x = 0.44 if selected else 0.24
        marker.scale.y = 0.09 if selected else 0.045
        marker.scale.z = 0.09 if selected else 0.045
        red, green, blue, alpha = self._parking_candidate_color(decision.status)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _parking_candidate_decision_label(
        self,
        decision: ParkingCandidateDecision,
    ) -> Marker:
        x_m, y_m, _ = decision.candidate
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'parking_candidate_decision_labels'
        marker.id = decision.marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x_m
        marker.pose.position.y = y_m
        marker.pose.position.z = 0.48
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.13
        red, green, blue, alpha = self._parking_candidate_color(decision.status)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

        prefix = 'SELECTED ' if decision.status == 'selected' else ''
        marker.text = f'{prefix}#{decision.marker_id}'
        if self.parking_candidate_labels_enabled:
            marker.text += f' {decision.status}: {decision.reason}'
        if self.parking_candidate_scores_enabled:
            metrics = []
            if decision.preference_label is not None:
                metrics.append(f'class={decision.preference_label}')
            if decision.reach_error_m is not None:
                metrics.append(f'reach_err={decision.reach_error_m:.2f}')
            if decision.clearance_m is not None:
                metrics.append(f'clear={decision.clearance_m:.2f}')
            if decision.path_length_m is not None:
                metrics.append(f'path={decision.path_length_m:.2f}')
            if metrics:
                marker.text += '\n' + ' '.join(metrics)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _log_parking_candidate_decisions(self, stage: str) -> None:
        status_counts = {
            'not_evaluated': 0,
            'valid': 0,
            'collision': 0,
            'rejected': 0,
            'selected': 0,
        }
        rejection_counts: dict[str, int] = {}
        for decision in self.parking_candidate_decisions.values():
            status_counts[decision.status] = status_counts.get(decision.status, 0) + 1
            if decision.status in ('collision', 'rejected'):
                rejection_counts[decision.reason] = (
                    rejection_counts.get(decision.reason, 0) + 1
                )
        reasons = ', '.join(
            f'{reason}={count}'
            for reason, count in sorted(rejection_counts.items())
        ) or 'none'
        self.get_logger().info(
            f'Parking candidate decisions [{stage}]: '
            f'generated={len(self.parking_candidate_decisions)}, '
            f'not_evaluated={status_counts["not_evaluated"]}, '
            f'valid={status_counts["valid"]}, '
            f'collision_rejected={status_counts["collision"]}, '
            f'other_rejected={status_counts["rejected"]}, '
            f'selected={status_counts["selected"]}; reasons={reasons}'
        )

    def _is_footprint_safe_at(
        self,
        center_x_m: float,
        center_y_m: float,
        grid: OccupancyGrid,
    ) -> bool:
        return self._footprint_rejection_reason(center_x_m, center_y_m, grid) is None

    def _footprint_rejection_reason(
        self,
        center_x_m: float,
        center_y_m: float,
        grid: OccupancyGrid,
    ) -> str | None:
        footprint_min_x = center_x_m + self.footprint_min_x_m
        footprint_max_x = center_x_m + self.footprint_max_x_m
        footprint_min_y = center_y_m + self.footprint_min_y_m
        footprint_max_y = center_y_m + self.footprint_max_y_m

        footprint_corners = [
            (footprint_min_x, footprint_min_y),
            (footprint_min_x, footprint_max_y),
            (footprint_max_x, footprint_min_y),
            (footprint_max_x, footprint_max_y),
        ]
        local_corners = [
            self._map_to_grid_local(grid, corner_x, corner_y)
            for corner_x, corner_y in footprint_corners
        ]

        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        map_width_m = float(width) * resolution
        map_height_m = float(height) * resolution
        for local_x, local_y in local_corners:
            if (
                local_x < 0.0
                or local_y < 0.0
                or local_x >= map_width_m
                or local_y >= map_height_m
            ):
                return 'out_of_bounds'

        min_local_x = min(point[0] for point in local_corners)
        max_local_x = max(point[0] for point in local_corners)
        min_local_y = min(point[1] for point in local_corners)
        max_local_y = max(point[1] for point in local_corners)
        min_col = max(0, int(math.floor(min_local_x / resolution)) - 1)
        max_col = min(width - 1, int(math.ceil(max_local_x / resolution)) + 1)
        min_row = max(0, int(math.floor(min_local_y / resolution)) - 1)
        max_row = min(height - 1, int(math.ceil(max_local_y / resolution)) + 1)

        half_cell_diagonal = math.sqrt(2.0) * resolution * 0.5
        has_occupied = False
        has_unknown = False
        has_other = False
        for row in range(min_row, max_row + 1):
            row_offset = row * width
            for col in range(min_col, max_col + 1):
                cell_center = self._grid_cell_center_to_map(grid, col, row)
                if not self._point_in_axis_aligned_footprint(
                    cell_center.x,
                    cell_center.y,
                    footprint_min_x - half_cell_diagonal,
                    footprint_max_x + half_cell_diagonal,
                    footprint_min_y - half_cell_diagonal,
                    footprint_max_y + half_cell_diagonal,
                ):
                    continue

                value_index = row_offset + col
                if value_index >= len(grid.data):
                    return 'out_of_bounds'
                value = int(grid.data[value_index])
                if value >= self.occupied_threshold:
                    has_occupied = True
                elif value == -1:
                    has_unknown = True
                elif value < 0:
                    has_other = True

        if has_occupied:
            return 'occupied'
        if has_unknown:
            return 'unknown'
        if has_other:
            return 'other'
        return None

    def _map_to_grid_local(
        self,
        grid: OccupancyGrid,
        map_x_m: float,
        map_y_m: float,
    ) -> tuple[float, float]:
        origin = grid.info.origin
        rotation = rotation_matrix_from_quaternion(origin.orientation)
        dx = map_x_m - origin.position.x
        dy = map_y_m - origin.position.y
        return (
            float(rotation[0][0] * dx + rotation[1][0] * dy),
            float(rotation[0][1] * dx + rotation[1][1] * dy),
        )

    def _grid_cell_center_to_map(
        self,
        grid: OccupancyGrid,
        col: int,
        row: int,
    ) -> Point:
        resolution = float(grid.info.resolution)
        local_x = (float(col) + 0.5) * resolution
        local_y = (float(row) + 0.5) * resolution
        origin = grid.info.origin
        rotation = rotation_matrix_from_quaternion(origin.orientation)
        return Point(
            x=float(
                origin.position.x
                + rotation[0][0] * local_x
                + rotation[0][1] * local_y
            ),
            y=float(
                origin.position.y
                + rotation[1][0] * local_x
                + rotation[1][1] * local_y
            ),
            z=0.0,
        )

    @staticmethod
    def _point_in_axis_aligned_footprint(
        x_m: float,
        y_m: float,
        min_x_m: float,
        max_x_m: float,
        min_y_m: float,
        max_y_m: float,
    ) -> bool:
        return (
            min_x_m <= x_m <= max_x_m
            and min_y_m <= y_m <= max_y_m
        )

    @staticmethod
    def _point_in_polygon(x_m: float, y_m: float, polygon: list[Point]) -> bool:
        inside = False
        previous = polygon[-1]
        for current in polygon:
            if MissionManagerNode._point_on_segment(
                x_m,
                y_m,
                previous.x,
                previous.y,
                current.x,
                current.y,
            ):
                return True
            if (current.y > y_m) != (previous.y > y_m):
                intersection_x = (
                    (previous.x - current.x)
                    * (y_m - current.y)
                    / (previous.y - current.y)
                    + current.x
                )
                if x_m < intersection_x:
                    inside = not inside
            previous = current
        return inside

    @staticmethod
    def _point_on_segment(
        point_x_m: float,
        point_y_m: float,
        start_x_m: float,
        start_y_m: float,
        end_x_m: float,
        end_y_m: float,
    ) -> bool:
        cross = (
            (point_y_m - start_y_m) * (end_x_m - start_x_m)
            - (point_x_m - start_x_m) * (end_y_m - start_y_m)
        )
        if abs(cross) > 1e-9:
            return False

        dot = (
            (point_x_m - start_x_m) * (end_x_m - start_x_m)
            + (point_y_m - start_y_m) * (end_y_m - start_y_m)
        )
        if dot < 0.0:
            return False

        squared_length = (
            (end_x_m - start_x_m) ** 2
            + (end_y_m - start_y_m) ** 2
        )
        return dot <= squared_length

    def _reachability_base_marker(self, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reachability_donut'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        return marker

    def _reachability_fill_marker(
        self,
        center_x_m: float,
        center_y_m: float,
    ) -> Marker:
        marker = self._reachability_base_marker(0, Marker.TRIANGLE_LIST)
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.25
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        for index in range(self.reachability_donut_segments):
            next_index = (index + 1) % self.reachability_donut_segments
            inner_i = self._donut_point(center_x_m, center_y_m, self.reach_min_m, index)
            outer_i = self._donut_point(center_x_m, center_y_m, self.reach_max_m, index)
            inner_next = self._donut_point(
                center_x_m,
                center_y_m,
                self.reach_min_m,
                next_index,
            )
            outer_next = self._donut_point(
                center_x_m,
                center_y_m,
                self.reach_max_m,
                next_index,
            )
            marker.points.extend([
                inner_i,
                outer_i,
                outer_next,
                inner_i,
                outer_next,
                inner_next,
            ])

        return marker

    def _reachability_outline_marker(
        self,
        marker_id: int,
        radius_m: float,
        center_x_m: float,
        center_y_m: float,
    ) -> Marker:
        marker = self._reachability_base_marker(marker_id, Marker.LINE_STRIP)
        marker.scale.x = 0.02
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 0.9

        for index in range(self.reachability_donut_segments + 1):
            marker.points.append(
                self._donut_point(
                    center_x_m,
                    center_y_m,
                    radius_m,
                    index % self.reachability_donut_segments,
                )
            )

        return marker

    def _donut_center_debug_marker(self, center_x_m: float, center_y_m: float) -> Marker:
        marker = self._reachability_base_marker(3, Marker.SPHERE)
        marker.ns = 'g1_donut_center_debug'
        marker.pose.position.x = center_x_m
        marker.pose.position.y = center_y_m
        marker.pose.position.z = 0.18
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.16
        marker.scale.y = 0.16
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _g1_sensor_foot_midpoint_marker(self) -> Marker | None:
        if (
            self.g1_sensor_foot_midpoint_x_map_m is None
            or self.g1_sensor_foot_midpoint_y_map_m is None
        ):
            return None

        marker = self._reachability_base_marker(5, Marker.SPHERE)
        marker.ns = 'g1_sensor_foot_midpoint'
        marker.pose.position.x = self.g1_sensor_foot_midpoint_x_map_m
        marker.pose.position.y = self.g1_sensor_foot_midpoint_y_map_m
        marker.pose.position.z = 0.22
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 0.0
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0
        return marker

    def _publish_g1_sensor_foot_midpoint_marker(self) -> None:
        marker = self._g1_sensor_foot_midpoint_marker()
        if marker is None:
            return

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.reachability_donut_publisher.publish(marker_array)

    def _g1_foot_midpoint_debug_marker(self) -> Marker | None:
        foot_points = self._debug_g1_foot_points_in_map()
        if foot_points is None:
            return None

        _, _, mid_x_m, mid_y_m = foot_points
        marker = self._reachability_base_marker(4, Marker.SPHERE)
        marker.ns = 'g1_foot_midpoint_ground_truth'
        marker.pose.position.x = mid_x_m
        marker.pose.position.y = mid_y_m
        marker.pose.position.z = 0.20
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def _donut_point(
        self,
        center_x_m: float,
        center_y_m: float,
        radius_m: float,
        index: int,
    ) -> Point:
        theta = 2.0 * math.pi * index / self.reachability_donut_segments
        return Point(
            x=center_x_m + radius_m * math.cos(theta),
            y=center_y_m + radius_m * math.sin(theta),
            z=0.03,
        )

    def _on_g1_detections(self, msg: G1Detections) -> None:
        if self.state == MissionState.SEARCH_G1:
            self._record_latest_g1_detection(msg)
            self._process_g1_detection_confirmation(msg)
        elif self.state == MissionState.CENTER_G1 and self.g1_committed:
            self._process_g1_centering_detection(msg)

    def _on_g1_camera_measurements(self, msg: G1Measurements) -> None:
        if self.state in (
            MissionState.SEARCH_G1,
            MissionState.LOCALIZE_G1,
        ):
            self._process_g1_camera_ground_sample(msg)
        elif self.state in (
            MissionState.TRANSFORM_G1_TO_MAP,
            MissionState.G1_LOCALIZED,
            MissionState.DONUT_READY,
            MissionState.DONUT_CLASSIFIED,
            MissionState.FOOTPRINT_CHECKED,
            MissionState.PARKING_POSES_VALIDATED,
            MissionState.NAV2_FEASIBILITY_CHECKED,
            MissionState.PARKING_POSE_SELECTED,
        ):
            self._process_g1_sensor_foot_midpoint_sample(msg)

    def _process_g1_detection_confirmation(self, msg: G1Detections) -> None:
        detected = bool(msg.detected and msg.count > 0)
        self.g1_detection_vote_window.append(detected)
        window_values = [1 if value else 0 for value in self.g1_detection_vote_window]
        positives = sum(window_values)
        self.get_logger().info(
            f'G1 detection vote: detected={str(detected).lower()}, '
            f'window={window_values}, positives={positives}/'
            f'{self.g1_detection_vote_window_size}'
        )

        if (
            len(self.g1_detection_vote_window) == self.g1_detection_vote_window_size
            and positives == 0
        ):
            self._reset_g1_localization_state()

        if len(self.g1_detection_vote_window) < self.g1_detection_vote_required:
            return
        if positives < self.g1_detection_vote_required:
            return

        self.get_logger().info(
            f'G1 confirmed by temporal vote: {positives}/'
            f'{self.g1_detection_vote_window_size} recent detections'
        )
        self.g1_last_confirmation_vote = (
            positives,
            len(self.g1_detection_vote_window),
        )
        self._handle_target_found('g1_detector_temporal_vote')

    def _record_latest_g1_detection(self, msg: G1Detections) -> None:
        selection = self._select_highest_score_detection(msg)
        if selection is None:
            return
        bbox, score = selection
        self.latest_g1_detection_bbox = bbox
        self.latest_g1_detection_score = score
        self.latest_g1_detection_image_size = (
            int(msg.image_width),
            int(msg.image_height),
        )
        self.latest_g1_detection_stamp = msg.header.stamp

    def _handle_target_found(self, source: str) -> None:
        previous_state = self.state
        self._reset_parking_pipeline_timing()
        self._reset_g1_detection_vote_window()
        self.g1_committed = bool(self.stop_exploration_on_g1)
        self.g1_commit_robot_pose = (
            self._current_base_pose_in_map() if self.g1_committed else None
        )
        self.g1_observation_attempted = False
        self.g1_observation_pose_reached = False
        self.g1_observation_pose = None
        self.g1_current_observation_label = None
        self.g1_observation_view_check_pending = False
        self.g1_observation_candidates = []
        self.g1_observation_candidate_index = 0
        self.g1_centering_corrections = 0
        self.g1_centering_goal_generation += 1
        self.g1_centering_goal_in_flight = False
        self.g1_localization_failure_reasons.clear()
        self.exploration_stop_reason = None

        if not self.stop_exploration_on_g1:
            self._enter_g1_localization(previous_state)
            return

        self.get_logger().info(
            f'Target found from {source}; committing to G1 parking mission'
        )
        self.get_logger().info('G1 confirmed during exploration')
        self.get_logger().info('G1 locked; switching from exploration to parking mode')
        if self.g1_commit_robot_pose is not None:
            self.get_logger().info(
                'G1 lock reference pose: '
                f'base=({self.g1_commit_robot_pose[0]:.3f}, '
                f'{self.g1_commit_robot_pose[1]:.3f}, '
                f'{self.g1_commit_robot_pose[2]:.3f} rad)'
            )
        self.get_logger().info('G1 committed; exploration will not resume')
        self._log_g1_commit_diagnostics()
        self.get_logger().info(
            'G1 confirmed; stopping exploration before focused localization'
        )
        self.state = MissionState.STOPPING_EXPLORATION
        self.exploration_stop_reason = 'g1_commit'
        self._timing_start('exploration_stop')
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._start_exploration_handoff()

    def _log_g1_commit_diagnostics(self) -> None:
        bbox = self.latest_g1_detection_bbox
        image_size = self.latest_g1_detection_image_size
        if bbox is None or image_size is None:
            self.get_logger().info(
                'G1 commit: bbox unavailable, current_robot_pose='
                f'{self.g1_commit_robot_pose}'
            )
            return
        width_px, height_px = image_size
        margins = self._g1_bbox_edge_margins(bbox, width_px, height_px)
        center_error = self._bbox_horizontal_center_error(bbox, width_px)
        margin_text = 'n/a' if margins is None else (
            f'left={margins[0]:.1f}, right={margins[1]:.1f}, '
            f'top={margins[2]:.1f}, bottom={margins[3]:.1f}'
        )
        self.get_logger().info(
            'G1 commit: '
            f'vote={self.g1_last_confirmation_vote[0]}/'
            f'{self.g1_last_confirmation_vote[1]}, '
            f'score={self.latest_g1_detection_score}, bbox={bbox}, '
            f'horizontal_center_error={center_error:.3f}, '
            f'{margin_text}, current_robot_pose={self.g1_commit_robot_pose}'
        )

    def _reset_g1_detection_vote_window(self) -> None:
        self.g1_detection_vote_window.clear()

    def _reset_g1_localization_state(self) -> None:
        self.latest_g1_detection_bbox = None
        self.recent_g1_detection_bboxes.clear()
        self.g1_old_ground_estimate_samples = []
        self.g1_old_ground_estimate_x_base_m = None
        self.g1_old_ground_estimate_y_base_m = None
        self.g1_old_ground_estimate_x_map_m = None
        self.g1_old_ground_estimate_y_map_m = None
        self.g1_ground_x_base_m = None
        self.g1_ground_y_base_m = None
        self.g1_map_x_m = None
        self.g1_map_y_m = None
        self.g1_sensor_foot_midpoint_samples = []
        self.g1_sensor_foot_midpoint_x_base_m = None
        self.g1_sensor_foot_midpoint_y_base_m = None
        self.g1_sensor_foot_midpoint_x_map_m = None
        self.g1_sensor_foot_midpoint_y_map_m = None
        self.g1_sensor_foot_1_map_m = None
        self.g1_sensor_foot_2_map_m = None
        self.g1_body_position_samples = []
        self.g1_body_position_x_map_m = None
        self.g1_body_position_y_map_m = None
        self.g1_position_source = None
        self.parking_candidate_decisions = {}
        if hasattr(self, 'parking_pose_candidates_publisher'):
            self._publish_parking_candidate_decisions()
        self.g1_observation_view_check_pending = False
        self.g1_localization_min_stamp_s = 0.0
        self.g1_localization_failure_reasons.clear()

    @staticmethod
    def _bbox_horizontal_center_error(
        bbox: tuple[float, float, float, float],
        image_width_px: int,
    ) -> float:
        if image_width_px <= 0:
            return float('inf')
        center_x_px = (float(bbox[0]) + float(bbox[2])) / 2.0
        return (center_x_px - image_width_px / 2.0) / (image_width_px / 2.0)

    def _begin_g1_centering(self, previous_state: MissionState) -> None:
        self._cancel_g1_localization_timeout()
        self._cancel_g1_centering_detection_timeout()
        self._clear_g1_foot_midpoint_samples()
        self.g1_centering_corrections = 0
        self.g1_centering_goal_in_flight = False
        self.g1_localization_failure_reasons.clear()
        self.state = MissionState.CENTER_G1
        self.g1_centering_min_detection_stamp_s = self._now_seconds()
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self.get_logger().info(
            'Exploration fully stopped; centering G1 in camera before localization'
        )

        bbox = self.latest_g1_detection_bbox
        image_size = self.latest_g1_detection_image_size
        if bbox is not None and image_size is not None:
            if self._g1_bbox_needs_centering(bbox, image_size[0]):
                self._send_g1_centering_goal(bbox)
                return
        self._start_g1_centering_detection_timeout()

    def _process_g1_centering_detection(self, msg: G1Detections) -> None:
        if self.g1_centering_goal_in_flight:
            return
        selection = self._select_highest_score_detection(msg)
        if selection is None:
            self._record_g1_localization_failure('no fresh G1 detection')
            return
        detection_stamp_s = self._stamp_seconds(msg.header.stamp)
        if detection_stamp_s < self.g1_centering_min_detection_stamp_s:
            return

        self._record_latest_g1_detection(msg)
        bbox, _ = selection
        width_px = int(msg.image_width)
        height_px = int(msg.image_height)
        margins = self._g1_bbox_edge_margins(bbox, width_px, height_px)
        bearing = self._compute_bbox_horizontal_bearing_in_base(bbox)
        bearing_rad = (
            None if bearing is None else math.atan2(bearing[1], bearing[0])
        )
        center_error = self._bbox_horizontal_center_error(bbox, width_px)
        pose = self._current_base_pose_in_map()
        margins_text = 'n/a' if margins is None else (
            f'left={margins[0]:.1f}, right={margins[1]:.1f}, '
            f'top={margins[2]:.1f}, bottom={margins[3]:.1f}'
        )
        self.get_logger().info(
            'G1 centering view: '
            f'pose={pose}, bearing={bearing_rad}, '
            f'horizontal_center_error={center_error:.3f}, {margins_text}'
        )

        if not self._g1_bbox_needs_centering(bbox, width_px):
            self._start_stationary_g1_localization(msg.header.stamp)
            return
        if self.g1_centering_corrections >= self.g1_centering_max_corrections:
            self._handle_g1_centering_failure(
                'G1 remains outside centered camera view after bounded corrections'
            )
            return
        self._send_g1_centering_goal(bbox)

    def _g1_bbox_needs_centering(
        self,
        bbox: tuple[float, float, float, float],
        image_width_px: int,
    ) -> bool:
        bearing = self._compute_bbox_horizontal_bearing_in_base(bbox)
        if bearing is None:
            return True
        bearing_rad = math.atan2(bearing[1], bearing[0])
        left_margin_px = float(bbox[0])
        right_margin_px = float(image_width_px) - float(bbox[2])
        side_clipped = min(left_margin_px, right_margin_px) < float(
            self.g1_body_bbox_edge_margin_px
        )
        return (
            abs(bearing_rad) > self.g1_centering_angular_tolerance_rad
            or side_clipped
        )

    def _send_g1_centering_goal(
        self,
        bbox: tuple[float, float, float, float],
    ) -> None:
        if self.state != MissionState.CENTER_G1 or self.g1_centering_goal_in_flight:
            return
        if self.g1_centering_corrections >= self.g1_centering_max_corrections:
            self._handle_g1_centering_failure(
                'G1 centering correction limit reached'
            )
            return
        current_pose = self._current_base_pose_in_map()
        bearing = self._compute_bbox_horizontal_bearing_in_base(bbox)
        if current_pose is None or bearing is None:
            self._handle_g1_centering_failure(
                'G1 centering unavailable: pose/bearing TF unavailable'
            )
            return
        if not self.navigate_to_pose_client.server_is_ready():
            self._handle_g1_centering_failure(
                'G1 centering unavailable: NavigateToPose server not ready'
            )
            return

        bearing_rad = math.atan2(bearing[1], bearing[0])
        target_yaw = self._normalize_angle(current_pose[2] + bearing_rad)
        goal = NavigateToPose.Goal()
        goal.pose = self._candidate_pose_stamped(
            current_pose[0],
            current_pose[1],
            target_yaw,
        )
        self.g1_centering_corrections += 1
        self.g1_centering_goal_generation += 1
        generation = self.g1_centering_goal_generation
        self.g1_centering_goal_in_flight = True
        self._cancel_g1_centering_detection_timeout()
        self._clear_g1_foot_midpoint_samples()
        self.get_logger().info(
            'Centering G1 in place: '
            f'x={current_pose[0]:.3f}, y={current_pose[1]:.3f}, '
            f'current_yaw={current_pose[2]:.3f}, '
            f'bearing={bearing_rad:.3f}, target_yaw={target_yaw:.3f}, '
            f'correction={self.g1_centering_corrections}/'
            f'{self.g1_centering_max_corrections}'
        )
        future = self.navigate_to_pose_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, token=generation: self._on_g1_centering_goal_response(
                done, token
            )
        )

    def _on_g1_centering_goal_response(self, future, generation: int) -> None:
        if (
            self.state != MissionState.CENTER_G1
            or generation != self.g1_centering_goal_generation
        ):
            return
        try:
            goal_handle = future.result()
        except Exception as error:
            self.g1_centering_goal_in_flight = False
            self._handle_g1_centering_failure(
                f'G1 centering goal failed: {error}'
            )
            return
        if not goal_handle.accepted:
            self.g1_centering_goal_in_flight = False
            self._handle_g1_centering_failure('G1 centering goal rejected')
            return
        goal_handle.get_result_async().add_done_callback(
            lambda done, token=generation: self._on_g1_centering_result(done, token)
        )

    def _on_g1_centering_result(self, future, generation: int) -> None:
        if (
            self.state != MissionState.CENTER_G1
            or generation != self.g1_centering_goal_generation
        ):
            return
        self.g1_centering_goal_in_flight = False
        try:
            action_result = future.result()
        except Exception as error:
            self._handle_g1_centering_failure(
                f'G1 centering navigation failed: {error}'
            )
            return
        result = action_result.result
        if not (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == NavigateToPose.Result.NONE
        ):
            self._handle_g1_centering_failure(
                'G1 centering navigation failed: '
                f'status={action_result.status}, error_code={result.error_code}'
            )
            return

        self._clear_g1_foot_midpoint_samples()
        self.g1_centering_min_detection_stamp_s = self._now_seconds()
        self.get_logger().info(
            'G1 centering rotation complete; waiting for a fresh detection'
        )
        self._start_g1_centering_detection_timeout()

    def _start_g1_centering_detection_timeout(self) -> None:
        self._cancel_g1_centering_detection_timeout()
        self.g1_centering_detection_timer = self.create_timer(
            self.g1_centering_detection_timeout_s,
            self._on_g1_centering_detection_timeout,
        )

    def _cancel_g1_centering_detection_timeout(self) -> None:
        timer = getattr(self, 'g1_centering_detection_timer', None)
        if timer is not None:
            timer.cancel()
            self.g1_centering_detection_timer = None

    def _on_g1_centering_detection_timeout(self) -> None:
        self._cancel_g1_centering_detection_timeout()
        if self.state != MissionState.CENTER_G1:
            return
        self._handle_g1_centering_failure(
            'G1 centering failed: no fresh G1 detection after rotation'
        )

    def _start_stationary_g1_localization(self, detection_stamp: TimeMsg) -> None:
        if self.state != MissionState.CENTER_G1:
            return
        self._cancel_g1_centering_detection_timeout()
        self._clear_g1_foot_midpoint_samples()
        self.g1_localization_generation += 1
        self.g1_localization_min_stamp_s = self._stamp_seconds(detection_stamp)
        previous_state = self.state
        self.get_logger().info(
            'G1 centered; collecting fresh timestamp-matched localization samples'
        )
        self._log_g1_localization_viewpoint()
        self._enter_g1_localization(previous_state, start_timeout=True)

    def _handle_g1_centering_failure(self, message: str) -> None:
        self._cancel_g1_centering_detection_timeout()
        self.g1_centering_goal_generation += 1
        self.g1_centering_goal_in_flight = False
        self.get_logger().warn(message)
        if self.enable_translating_g1_observation:
            self._start_translating_g1_observation_or_return()
            return
        self._return_to_start_from_current_state(
            f'{message}; unable to localize G1 safely; returning to start'
        )

    def _start_translating_g1_observation_or_return(self) -> None:
        if (
            self.g1_old_ground_estimate_x_map_m is None
            or self.g1_old_ground_estimate_y_map_m is None
        ):
            self._return_to_start_from_current_state(
                'Unable to localize G1 safely; returning to start'
            )
            return
        self.g1_observation_attempted = True
        previous_state = self.state
        self.state = MissionState.STOPPING_EXPLORATION
        self.exploration_stop_reason = 'observation'
        self.exploration_stop_accepted = True
        self.get_logger().info(
            'Experimental translating G1 observation enabled'
        )
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._start_g1_observation_validation()

    def _log_g1_localization_viewpoint(self) -> None:
        bbox = self.latest_g1_detection_bbox
        image_size = self.latest_g1_detection_image_size
        bearing = None
        margins = None
        if bbox is not None:
            bearing_vector = self._compute_bbox_horizontal_bearing_in_base(bbox)
            if bearing_vector is not None:
                bearing = math.atan2(bearing_vector[1], bearing_vector[0])
            if image_size is not None:
                margins = self._g1_bbox_edge_margins(
                    bbox, image_size[0], image_size[1]
                )
        detection_stamp_s = (
            None
            if self.latest_g1_detection_stamp is None
            else self._stamp_seconds(self.latest_g1_detection_stamp)
        )
        depth_dt = self._matched_buffer_dt(self.depth_buffer, detection_stamp_s)
        cloud_dt = self._matched_buffer_dt(self.pointcloud_buffer, detection_stamp_s)
        self.get_logger().info(
            'G1 localization viewpoint: '
            f'pose={self._current_base_pose_in_map()}, bearing={bearing}, '
            f'bbox={bbox}, edge_margins={margins}, '
            f'matched_depth_dt={depth_dt}, matched_pointcloud_dt={cloud_dt}, '
            f'generation={self.g1_localization_generation}'
        )

    def _matched_buffer_dt(self, samples, detection_stamp_s: float | None):
        if detection_stamp_s is None:
            return None
        match = select_closest_timestamped(
            samples,
            detection_stamp_s,
            self.g1_sensor_sync_tolerance_s,
        )
        return None if match is None else match[1]

    def _now_seconds(self) -> float:
        return float(self.get_clock().now().nanoseconds) * 1e-9

    def _enter_g1_localization(
        self,
        previous_state: MissionState,
        start_timeout: bool = True,
    ) -> None:
        self._cancel_g1_localization_timeout()
        self._cancel_g1_observation_trigger()
        self.state = MissionState.LOCALIZE_G1
        self._timing_start('g1_localization')
        self.g1_observation_start_s = time.monotonic()
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        if self.stop_exploration_on_g1 and not self.g1_committed:
            self._start_g1_observation_trigger()
        if start_timeout:
            self._start_g1_localization_timeout()

    def _start_g1_observation_trigger(self) -> None:
        if self.g1_observation_trigger_timer is not None:
            self.g1_observation_trigger_timer.cancel()
        self.g1_observation_trigger_timer = self.create_timer(
            0.5,
            self._maybe_trigger_g1_observation,
        )

    def _cancel_g1_observation_trigger(self) -> None:
        if self.g1_observation_trigger_timer is not None:
            self.g1_observation_trigger_timer.cancel()
            self.g1_observation_trigger_timer = None

    def _maybe_trigger_g1_observation(self) -> None:
        if self.state != MissionState.LOCALIZE_G1:
            self._cancel_g1_observation_trigger()
            return
        if self.g1_sensor_foot_midpoint_x_map_m is not None:
            self._cancel_g1_observation_trigger()
            return
        if self.g1_observation_attempted:
            self._cancel_g1_observation_trigger()
            return
        if self.g1_observation_start_s is None:
            return

        elapsed_s = time.monotonic() - self.g1_observation_start_s
        if elapsed_s < self.g1_observation_trigger_s:
            return
        if (
            self.g1_old_ground_estimate_x_map_m is None
            or self.g1_old_ground_estimate_y_map_m is None
        ):
            if self.g1_committed:
                self._cancel_g1_observation_trigger()
                self._return_to_start_from_current_state(
                    'Unable to localize G1 safely; returning to start'
                )
            return

        self.g1_observation_attempted = True
        self._cancel_g1_observation_trigger()
        if self.g1_committed:
            self.get_logger().info(
                'Direct G1 localization unsuccessful; trying side observation'
            )
        else:
            self.get_logger().info(
                'G1 localization incomplete after '
                f'{elapsed_s:.1f} s; requesting side observation'
            )
        previous_state = self.state
        self.state = MissionState.STOPPING_EXPLORATION
        self.exploration_stop_reason = 'observation'
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        if self.exploration_active is False:
            self.exploration_stop_accepted = True
            self._complete_exploration_handoff_if_ready()
        else:
            self._timing_start('exploration_stop')
            self._start_exploration_handoff()

    def _start_g1_localization_timeout(self) -> None:
        if not self.stop_exploration_on_g1:
            return

        self.get_logger().info(
            f'G1 localization started; timeout={self.g1_localization_timeout_s:.1f} s'
        )
        if self.g1_localization_timeout_timer is not None:
            self.g1_localization_timeout_timer.cancel()
        self.g1_localization_timeout_timer = self.create_timer(
            self.g1_localization_timeout_s,
            self._on_g1_localization_timeout,
        )

    def _cancel_g1_localization_timeout(self) -> None:
        if self.g1_localization_timeout_timer is not None:
            self.g1_localization_timeout_timer.cancel()
            self.g1_localization_timeout_timer = None

    def _on_g1_localization_timeout(self) -> None:
        self._cancel_g1_localization_timeout()
        if self.state != MissionState.LOCALIZE_G1:
            return

        if self.g1_committed:
            reasons = sorted(self.g1_localization_failure_reasons)
            reason_text = ', '.join(reasons) if reasons else 'inconsistent samples or timeout'
            self.get_logger().warn(
                'G1 stationary localization timed out: '
                f'{reason_text}'
            )
            self._clear_g1_foot_midpoint_samples()
            if self.enable_translating_g1_observation:
                self._start_translating_g1_observation_or_return()
                return
            self._return_to_start_from_current_state(
                'Unable to obtain reliable G1 position; returning to start'
            )
            return

        self.get_logger().warn('G1 localization timed out; resuming exploration')
        self._reset_g1_localization_state()
        self._reset_g1_detection_vote_window()
        self.exploration_stop_accepted = False
        self.exploration_stop_request_in_flight = False
        self._request_exploration_resume()

    def _request_exploration_resume(self) -> None:
        if self.g1_committed:
            self._cancel_exploration_resume_retry()
            return
        if self.state != MissionState.LOCALIZE_G1:
            self._cancel_exploration_resume_retry()
            return
        if self.exploration_resume_request_in_flight:
            return
        if not self.exploration_stop_client.service_is_ready():
            if self.exploration_resume_retry_timer is None:
                self.exploration_resume_retry_timer = self.create_timer(
                    0.5,
                    self._request_exploration_resume,
                )
            return

        self._cancel_exploration_resume_retry()
        request = SetBool.Request()
        request.data = True
        self.exploration_resume_request_in_flight = True
        future = self.exploration_stop_client.call_async(request)
        future.add_done_callback(self._on_exploration_resume_response)

    def _cancel_exploration_resume_retry(self) -> None:
        if self.exploration_resume_retry_timer is not None:
            self.exploration_resume_retry_timer.cancel()
            self.exploration_resume_retry_timer = None

    def _on_exploration_resume_response(self, future) -> None:
        self.exploration_resume_request_in_flight = False
        if self.g1_committed:
            return
        if self.state != MissionState.LOCALIZE_G1:
            return

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Failed to re-enable exploration: {error}')
            self._request_exploration_resume()
            return

        if not response.success:
            message = response.message if response.message else 'no error message'
            self.get_logger().error(f'Failed to re-enable exploration: {message}')
            self._request_exploration_resume()
            return

        self.get_logger().info('Exploration re-enabled')
        previous_state = self.state
        self.state = MissionState.SEARCH_G1
        self.g1_committed = False
        self.g1_commit_robot_pose = None
        self._reset_g1_detection_vote_window()
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self.get_logger().info('Waiting for confirmed G1 detection')

    def _on_exploration_active(self, msg: Bool) -> None:
        self.exploration_active = bool(msg.data)
        self._complete_exploration_handoff_if_ready()

    def _start_exploration_handoff(self) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return
        self.exploration_stop_accepted = False
        self.exploration_stop_request_in_flight = False
        if self.exploration_stop_timer is None:
            self.exploration_stop_timer = self.create_timer(
                0.5,
                self._poll_exploration_handoff,
            )
        self._poll_exploration_handoff()

    def _poll_exploration_handoff(self) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            if self.exploration_stop_timer is not None:
                self.exploration_stop_timer.cancel()
                self.exploration_stop_timer = None
            return

        if not self.exploration_stop_accepted:
            self._request_exploration_stop()
            return

        self._complete_exploration_handoff_if_ready()

    def _request_exploration_stop(self) -> None:
        if self.exploration_stop_request_in_flight:
            return
        if not self.exploration_stop_client.service_is_ready():
            return

        request = SetBool.Request()
        request.data = False
        self.exploration_stop_request_in_flight = True
        self.get_logger().info('Stopping exploration')
        future = self.exploration_stop_client.call_async(request)
        future.add_done_callback(self._on_exploration_stop_response)

    def _on_exploration_stop_response(self, future) -> None:
        self.exploration_stop_request_in_flight = False
        if self.state != MissionState.STOPPING_EXPLORATION:
            return

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'Failed to stop exploration: {error}')
            return

        if not response.success:
            message = response.message if response.message else 'no error message'
            self.get_logger().error(f'Failed to stop exploration: {message}')
            return

        self.exploration_stop_accepted = True
        if response.message:
            self.get_logger().info(response.message)
        self._complete_exploration_handoff_if_ready()

    def _complete_exploration_handoff_if_ready(self) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return
        if not self.exploration_stop_accepted:
            return
        if self.exploration_active is not False:
            return

        if self.exploration_stop_timer is not None:
            self.exploration_stop_timer.cancel()
            self.exploration_stop_timer = None
        self._timing_stop('exploration_stop')
        previous_state = self.state
        if self.exploration_stop_reason == 'observation':
            self.get_logger().info(
                'Exploration stopped; validating G1 observation pose'
            )
            self._start_g1_observation_validation()
            return

        if self.exploration_stop_reason == 'g1_commit':
            self._begin_g1_centering(previous_state)
            return

        self.get_logger().info('Exploration stopped; parking mission taking control')
        self._begin_g1_map_transform(previous_state)

    def _start_g1_observation_validation(self) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return
        if self.latest_map is None:
            self._handle_g1_observation_failure(
                'G1 observation unavailable: no OccupancyGrid'
            )
            return

        if not self.g1_observation_candidates:
            candidates = self._generate_g1_observation_candidates()
            if not candidates:
                self._handle_g1_observation_failure(
                    'No valid G1 observation pose'
                )
                return

            self.g1_observation_candidates = candidates
            self.g1_observation_candidate_index = 0
        self._validate_next_g1_observation_candidate()

    def _generate_g1_observation_candidates(self) -> list[dict[str, float | str]]:
        current_pose = self._current_base_pose_in_map()
        if current_pose is None:
            self.get_logger().warn(
                'G1 observation unavailable: current base pose unavailable'
            )
            return []
        if (
            self.g1_old_ground_estimate_x_map_m is None
            or self.g1_old_ground_estimate_y_map_m is None
        ):
            self.get_logger().warn(
                'G1 observation unavailable: coarse G1 map estimate unavailable'
            )
            return []

        robot_x_m, robot_y_m, _ = current_pose
        g1_x_m = self.g1_old_ground_estimate_x_map_m
        g1_y_m = self.g1_old_ground_estimate_y_map_m
        dx_m = g1_x_m - robot_x_m
        dy_m = g1_y_m - robot_y_m
        distance_m = math.hypot(dx_m, dy_m)
        if distance_m < 0.25:
            self.get_logger().warn(
                'G1 observation unavailable: robot-to-G1 direction too small'
            )
            return []

        ux = dx_m / distance_m
        uy = dy_m / distance_m
        px = -uy
        py = ux
        candidates = []
        for side_label, sign in (('A', 1.0), ('B', -1.0)):
            for distance_index in range(self.g1_observation_distance_count):
                observation_distance_m = (
                    self.g1_observation_distance_m
                    + distance_index * self.g1_observation_distance_step_m
                )
                if observation_distance_m <= 0.0:
                    continue

                label = f'{side_label}@{observation_distance_m:.1f}m'
                obs_x_m = g1_x_m + sign * px * observation_distance_m
                obs_y_m = g1_y_m + sign * py * observation_distance_m
                yaw = math.atan2(g1_y_m - obs_y_m, g1_x_m - obs_x_m)
                self.get_logger().info(
                    f'G1 observation candidate {label}: '
                    f'x={obs_x_m:.3f}, y={obs_y_m:.3f}, yaw={yaw:.3f}, '
                    f'distance={observation_distance_m:.3f} m'
                )
                candidates.append({
                    'label': label,
                    'x': obs_x_m,
                    'y': obs_y_m,
                    'yaw': yaw,
                    'distance': observation_distance_m,
                })
        return candidates

    def _validate_next_g1_observation_candidate(self) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return
        if self.latest_map is None:
            self._handle_g1_observation_failure(
                'G1 observation unavailable: no OccupancyGrid'
            )
            return

        while self.g1_observation_candidate_index < len(self.g1_observation_candidates):
            candidate = self.g1_observation_candidates[
                self.g1_observation_candidate_index
            ]
            label = str(candidate['label'])
            x_m = float(candidate['x'])
            y_m = float(candidate['y'])
            yaw = float(candidate['yaw'])
            footprint_polygon = self._rotated_footprint_vertices(x_m, y_m, yaw)
            rejection_reason = self._rotated_footprint_rejection_reason(
                footprint_polygon,
                self.latest_map,
            )
            if rejection_reason is not None:
                self.get_logger().info(
                    f'G1 observation candidate {label} rejected: '
                    f'footprint {rejection_reason}'
                )
                self.g1_observation_candidate_index += 1
                continue

            self._validate_g1_observation_path(candidate)
            return

        self._handle_g1_observation_failure('No valid G1 observation pose')

    def _validate_g1_observation_path(self, candidate: dict[str, float | str]) -> None:
        if not self.compute_path_client.server_is_ready():
            if self.g1_observation_validation_wait_timer is None:
                self.g1_observation_validation_wait_timer = self.create_timer(
                    0.5,
                    lambda: self._validate_g1_observation_path(candidate),
                )
            return

        if self.g1_observation_validation_wait_timer is not None:
            self.g1_observation_validation_wait_timer.cancel()
            self.g1_observation_validation_wait_timer = None

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self._candidate_pose_stamped(
            float(candidate['x']),
            float(candidate['y']),
            float(candidate['yaw']),
        )
        goal_msg.planner_id = self.nav2_planner_id
        goal_msg.use_start = False

        goal_future = self.compute_path_client.send_goal_async(goal_msg)
        goal_future.add_done_callback(
            lambda future, checked_candidate=candidate: (
                self._on_g1_observation_path_goal_response(
                    future,
                    checked_candidate,
                )
            )
        )

    def _on_g1_observation_path_goal_response(
        self,
        future,
        candidate: dict[str, float | str],
    ) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return

        label = str(candidate['label'])
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().info(
                f'G1 observation candidate {label} rejected: {error}'
            )
            self.g1_observation_candidate_index += 1
            self._validate_next_g1_observation_candidate()
            return

        if not goal_handle.accepted:
            self.get_logger().info(
                f'G1 observation candidate {label} rejected: path goal rejected'
            )
            self.g1_observation_candidate_index += 1
            self._validate_next_g1_observation_candidate()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, checked_candidate=candidate: (
                self._on_g1_observation_path_result(future, checked_candidate)
            )
        )

    def _on_g1_observation_path_result(
        self,
        future,
        candidate: dict[str, float | str],
    ) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return

        label = str(candidate['label'])
        try:
            action_result = future.result()
        except Exception as error:
            self.get_logger().info(
                f'G1 observation candidate {label} rejected: {error}'
            )
            self.g1_observation_candidate_index += 1
            self._validate_next_g1_observation_candidate()
            return

        result = action_result.result
        path_is_valid = (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == ComputePathToPose.Result.NONE
            and len(result.path.poses) > 0
        )
        path_rejection_reason = None
        if path_is_valid and self.latest_map is not None:
            path_rejection_reason = self._path_footprint_rejection_reason(
                result.path,
                self.latest_map,
            )
            path_is_valid = path_rejection_reason is None
        if not path_is_valid:
            self.get_logger().info(
                f'G1 observation candidate {label} rejected: '
                f'{path_rejection_reason or "no Nav2 path"}'
            )
            self.g1_observation_candidate_index += 1
            self._validate_next_g1_observation_candidate()
            return

        observation_pose = {
            'x': float(candidate['x']),
            'y': float(candidate['y']),
            'yaw': float(candidate['yaw']),
        }
        path_length_m = self._path_length_m(result.path)
        self.get_logger().info(f'G1 observation side {label} selected')
        self.get_logger().info(
            'G1 observation pose selected: '
            f'x={observation_pose["x"]:.3f}, '
            f'y={observation_pose["y"]:.3f}, '
            f'yaw={observation_pose["yaw"]:.3f}, '
            f'path_length={path_length_m:.3f} m'
        )
        self._start_g1_observation_navigation(observation_pose)

    def _start_g1_observation_navigation(
        self,
        observation_pose: dict[str, float],
    ) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return
        if not self.navigate_to_pose_client.server_is_ready():
            self.get_logger().warn(
                'G1 observation navigation unavailable: NavigateToPose server not ready'
            )
            self._handle_g1_observation_failure(
                'G1 observation navigation unavailable'
            )
            return

        self.g1_observation_pose = observation_pose
        self.g1_current_observation_label = str(
            self.g1_observation_candidates[self.g1_observation_candidate_index]['label']
        )
        goal = NavigateToPose.Goal()
        goal.pose = self._candidate_pose_stamped(
            observation_pose['x'],
            observation_pose['y'],
            observation_pose['yaw'],
        )

        previous_state = self.state
        self.state = MissionState.NAVIGATING_TO_G1_OBSERVATION_POSE
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self.get_logger().info('Navigating to G1 observation pose')

        send_future = self.navigate_to_pose_client.send_goal_async(
            goal,
            feedback_callback=self._on_parking_navigation_feedback,
        )
        send_future.add_done_callback(self._on_g1_observation_goal_response)

    def _on_g1_observation_goal_response(self, future) -> None:
        if self.state != MissionState.NAVIGATING_TO_G1_OBSERVATION_POSE:
            return

        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'G1 observation navigation failed: {error}')
            self._advance_failed_g1_observation_candidate()
            self._try_next_g1_observation_or_return()
            return

        if not goal_handle.accepted:
            self.get_logger().error('G1 observation navigation failed: goal rejected')
            self._advance_failed_g1_observation_candidate()
            self._try_next_g1_observation_or_return()
            return

        self.get_logger().info('NavigateToPose goal accepted')
        goal_handle.get_result_async().add_done_callback(
            self._on_g1_observation_result
        )

    def _on_g1_observation_result(self, future) -> None:
        if self.state != MissionState.NAVIGATING_TO_G1_OBSERVATION_POSE:
            return

        try:
            action_result = future.result()
        except Exception as error:
            self.get_logger().error(f'G1 observation navigation failed: {error}')
            self._advance_failed_g1_observation_candidate()
            self._try_next_g1_observation_or_return()
            return

        result = action_result.result
        if (
            action_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.error_code == NavigateToPose.Result.NONE
        ):
            self.g1_observation_pose_reached = True
            self.g1_observation_candidate_index += 1
            self.g1_observation_view_check_pending = True
            self._clear_g1_foot_midpoint_samples()
            previous_state = self.state
            self.state = MissionState.LOCALIZE_G1
            self._timing_start('g1_localization')
            self._start_g1_localization_timeout()
            self.get_logger().info(
                f'Mission state transition: {previous_state.name} -> '
                f'{self.state.name}'
            )
            self.get_logger().info(
                'G1 observation pose reached; checking lower-body visibility'
            )
            return

        error_msg = result.error_msg if result.error_msg else 'no error message'
        self.get_logger().error(
            'G1 observation navigation failed: '
            f'status={action_result.status}, '
            f'error_code={result.error_code}, '
            f'error_msg={error_msg}'
        )
        self._advance_failed_g1_observation_candidate()
        self._try_next_g1_observation_or_return()

    def _advance_failed_g1_observation_candidate(self) -> None:
        if self.g1_current_observation_label is None:
            return
        if self.g1_observation_candidate_index < len(self.g1_observation_candidates):
            self.g1_observation_candidate_index += 1

    def _clear_g1_foot_midpoint_samples(self) -> None:
        self.g1_ground_x_base_m = None
        self.g1_ground_y_base_m = None
        self.g1_map_x_m = None
        self.g1_map_y_m = None
        self.g1_sensor_foot_midpoint_samples = []
        self.g1_sensor_foot_midpoint_x_base_m = None
        self.g1_sensor_foot_midpoint_y_base_m = None
        self.g1_sensor_foot_midpoint_x_map_m = None
        self.g1_sensor_foot_midpoint_y_map_m = None
        self.g1_sensor_foot_1_map_m = None
        self.g1_sensor_foot_2_map_m = None
        self.g1_body_position_samples = []
        self.g1_body_position_x_map_m = None
        self.g1_body_position_y_map_m = None
        self.g1_position_source = None

    def _resume_exploration_after_observation_failure(self) -> None:
        if self.g1_committed:
            self._handle_g1_observation_failure('No valid G1 observation pose')
            return
        self._cancel_g1_observation_trigger()
        if self.g1_observation_validation_wait_timer is not None:
            self.g1_observation_validation_wait_timer.cancel()
            self.g1_observation_validation_wait_timer = None
        self.get_logger().warn('No valid G1 observation pose; resuming exploration')
        self._reset_g1_localization_state()
        self._reset_g1_detection_vote_window()
        self.exploration_stop_reason = None
        self.exploration_stop_accepted = False
        self.exploration_stop_request_in_flight = False
        previous_state = self.state
        self.state = MissionState.LOCALIZE_G1
        if previous_state != self.state:
            self.get_logger().info(
                f'Mission state transition: {previous_state.name} -> '
                f'{self.state.name}'
            )
        self._request_exploration_resume()

    def _try_next_g1_observation_or_return(self) -> None:
        self._cancel_g1_localization_timeout()
        self._cancel_g1_observation_trigger()
        if self.g1_observation_validation_wait_timer is not None:
            self.g1_observation_validation_wait_timer.cancel()
            self.g1_observation_validation_wait_timer = None

        previous_label = self.g1_current_observation_label
        self.g1_current_observation_label = None
        if self.g1_observation_candidate_index < len(self.g1_observation_candidates):
            if previous_label:
                next_label = str(
                    self.g1_observation_candidates[
                        self.g1_observation_candidate_index
                    ]['label']
                )
                previous_side = previous_label.split('@', 1)[0]
                next_side = next_label.split('@', 1)[0]
                if previous_side == next_side:
                    self.get_logger().info(
                        f'G1 observation {previous_label} failed; '
                        'trying farther same-side viewpoint'
                    )
                else:
                    self.get_logger().info(
                        f'G1 observation side {previous_side} exhausted; '
                        f'trying side {next_side}'
                    )
            previous_state = self.state
            self.state = MissionState.STOPPING_EXPLORATION
            self.exploration_stop_reason = 'observation'
            self.exploration_stop_accepted = True
            self.get_logger().info(
                f'Mission state transition: {previous_state.name} -> '
                f'{self.state.name}'
            )
            self._start_g1_observation_validation()
            return

        self._handle_g1_observation_failure(
            'Unable to localize G1 safely; returning to start'
        )

    def _handle_g1_observation_failure(self, message: str) -> None:
        self._cancel_g1_localization_timeout()
        self._cancel_g1_observation_trigger()
        if self.g1_observation_validation_wait_timer is not None:
            self.g1_observation_validation_wait_timer.cancel()
            self.g1_observation_validation_wait_timer = None
        if self.g1_committed:
            self._return_to_start_from_current_state(message)
            return
        self.get_logger().warn(f'{message}; resuming exploration')
        self._reset_g1_localization_state()
        self._reset_g1_detection_vote_window()
        self.exploration_stop_reason = None
        self.exploration_stop_accepted = False
        self.exploration_stop_request_in_flight = False
        previous_state = self.state
        self.state = MissionState.LOCALIZE_G1
        if previous_state != self.state:
            self.get_logger().info(
                f'Mission state transition: {previous_state.name} -> '
                f'{self.state.name}'
            )
        self._request_exploration_resume()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    def _on_depth_image(self, msg: Image) -> None:
        try:
            depth_meters = convert_depth_to_meters_message(msg)
        except ValueError:
            return
        stamp_s = stamp_seconds(msg.header.stamp)
        self.depth_buffer.append((stamp_s, (msg, depth_meters)))
        prune_timestamped_buffer(
            self.depth_buffer,
            stamp_s,
            self.g1_sensor_buffer_max_age_s,
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        stamp_s = stamp_seconds(msg.header.stamp)
        self.pointcloud_buffer.append((stamp_s, msg))
        prune_timestamped_buffer(
            self.pointcloud_buffer,
            stamp_s,
            self.g1_sensor_buffer_max_age_s,
        )

    @staticmethod
    def _stamp_seconds(stamp: TimeMsg) -> float:
        return stamp_seconds(stamp)

    def _matched_depth_frame(
        self,
        detection_stamp: TimeMsg,
    ) -> tuple[Image, np.ndarray] | None:
        detection_s = self._stamp_seconds(detection_stamp)
        match = select_closest_timestamped(
            self.depth_buffer,
            detection_s,
            self.g1_sensor_sync_tolerance_s,
        )
        self._log_sensor_sync('depth', detection_s, match, self.depth_buffer)
        if match is None:
            self._record_g1_localization_failure('sensor timestamp mismatch: depth')
            return None
        return match[0]

    def _matched_pointcloud(
        self,
        detection_stamp: TimeMsg,
    ) -> PointCloud2 | None:
        detection_s = self._stamp_seconds(detection_stamp)
        match = select_closest_timestamped(
            self.pointcloud_buffer,
            detection_s,
            self.g1_sensor_sync_tolerance_s,
        )
        self._log_sensor_sync('cloud', detection_s, match, self.pointcloud_buffer)
        if match is None:
            self._record_g1_localization_failure(
                'sensor timestamp mismatch: pointcloud'
            )
            return None
        return match[0]

    def _log_sensor_sync(self, source, detection_s, match, samples) -> None:
        now_s = time.monotonic()
        if now_s - self.g1_sync_last_log_s[source] < 2.0:
            return
        self.g1_sync_last_log_s[source] = now_s
        if match is not None:
            _, dt_s = match
            self.get_logger().info(
                f'G1 sync: detection_stamp={detection_s:.3f}, '
                f'{source}_stamp={detection_s + dt_s:.3f}, dt={dt_s:+.3f} s'
            )
            return
        nearest_dt = min(
            (abs(sample_stamp_s - detection_s) for sample_stamp_s, _ in samples),
            default=None,
        )
        nearest_text = 'none' if nearest_dt is None else f'{nearest_dt:.3f} s'
        self.get_logger().warn(
            f'G1 sync rejected: detection_stamp={detection_s:.3f}, '
            f'{source}_nearest_dt={nearest_text}, '
            f'tolerance={self.g1_sensor_sync_tolerance_s:.3f} s'
        )

    def _record_g1_localization_failure(self, reason: str) -> None:
        if self.state in (MissionState.CENTER_G1, MissionState.LOCALIZE_G1):
            self.g1_localization_failure_reasons.add(reason)

    @staticmethod
    def _scaled_camera_intrinsics(
        camera_info: CameraInfo,
        target_width: int,
        target_height: int,
    ) -> tuple[float, float, float, float] | None:
        source_width = int(camera_info.width)
        source_height = int(camera_info.height)
        if (
            source_width <= 0
            or source_height <= 0
            or target_width <= 0
            or target_height <= 0
        ):
            return None

        scale_x = float(target_width) / float(source_width)
        scale_y = float(target_height) / float(source_height)
        fx = float(camera_info.k[0]) * scale_x
        fy = float(camera_info.k[4]) * scale_y
        cx = float(camera_info.k[2]) * scale_x
        cy = float(camera_info.k[5]) * scale_y
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
            return None
        if fx <= 0.0 or fy <= 0.0:
            return None
        return fx, fy, cx, cy

    def _process_g1_camera_ground_sample(self, msg: G1Measurements) -> None:
        if self.g1_committed and self.state == MissionState.LOCALIZE_G1:
            measurement_stamp_s = self._stamp_seconds(msg.header.stamp)
            if measurement_stamp_s < self.g1_localization_min_stamp_s:
                self._record_g1_localization_failure(
                    'measurement predates stationary localization viewpoint'
                )
                return
            if not msg.detected:
                self._record_g1_localization_failure('no fresh G1 detection')
                return
        sample = self._old_ground_estimate_sample_from_measurement(msg)
        state_before_foot_processing = self.state
        self._process_g1_sensor_foot_midpoint_sample(
            msg,
            old_ground_estimate_base=sample,
        )
        if self.state != state_before_foot_processing:
            return

        if self.g1_committed and self.state in (
            MissionState.LOCALIZE_G1,
            MissionState.STOPPING_EXPLORATION,
        ):
            self._process_g1_body_position_sample(
                msg,
                old_ground_estimate_base=sample,
            )
            if self.state != state_before_foot_processing:
                return

        if self.g1_old_ground_estimate_x_base_m is not None:
            return

        if sample is None:
            return

        sample_map = self._transform_base_xy_to_map(
            sample[0],
            sample[1],
            msg.header.stamp,
        )
        if sample_map is None:
            return

        self.g1_old_ground_estimate_samples.append(sample_map)
        if len(self.g1_old_ground_estimate_samples) > 5:
            self.g1_old_ground_estimate_samples.pop(0)
        if len(self.g1_old_ground_estimate_samples) < 5:
            return

        x_samples = [sample[0] for sample in self.g1_old_ground_estimate_samples]
        y_samples = [sample[1] for sample in self.g1_old_ground_estimate_samples]
        self.g1_old_ground_estimate_x_base_m = float(sample[0])
        self.g1_old_ground_estimate_y_base_m = float(sample[1])
        self.g1_old_ground_estimate_x_map_m = float(median(x_samples))
        self.g1_old_ground_estimate_y_map_m = float(median(y_samples))

        self.get_logger().info(
            'Coarse G1 map estimate stabilized: '
            f'x={self.g1_old_ground_estimate_x_map_m:.3f} m, '
            f'y={self.g1_old_ground_estimate_y_map_m:.3f} m'
        )

    def _old_ground_estimate_sample_from_measurement(
        self,
        msg: G1Measurements,
    ) -> tuple[float, float] | None:
        selection = self._select_highest_score_camera_center_measurement(msg)
        if selection is None:
            return None

        bbox, pointcloud_forward_m, pointcloud_lateral_m, sensor_depth_distance_m = selection
        self.latest_g1_detection_bbox = bbox
        self.recent_g1_detection_bboxes.append((bbox, msg.header.stamp))
        return self._compute_ground_sample_from_camera_measurement(
            bbox,
            pointcloud_forward_m,
            pointcloud_lateral_m,
            sensor_depth_distance_m,
        )

    def _begin_g1_map_transform(self, previous_state: MissionState) -> None:
        self.state = MissionState.TRANSFORM_G1_TO_MAP
        self._timing_start('map_transform')
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self.g1_map_transform_timer = self.create_timer(
            0.5,
            self._try_transform_g1_to_map,
        )
        self._try_transform_g1_to_map()

    def _process_g1_sensor_foot_midpoint_sample(
        self,
        msg: G1Measurements,
        old_ground_estimate_base: tuple[float, float] | None = None,
    ) -> None:
        if self.g1_sensor_foot_midpoint_x_map_m is not None:
            return
        if not msg.detected:
            return

        bbox = self._select_highest_score_measurement_bbox(msg)
        if bbox is None:
            return

        if self.g1_observation_view_check_pending:
            framing = self._g1_observation_framing(
                bbox,
                int(msg.image_width),
                int(msg.image_height),
                msg.header.stamp,
            )
            self._log_g1_observation_framing(framing)
            if not framing.usable:
                self.get_logger().info(
                    'G1 lower-body view unusable; trying alternate observation pose'
                )
                self.g1_observation_view_check_pending = False
                self._clear_g1_foot_midpoint_samples()
                self._try_next_g1_observation_or_return()
                return

            self.g1_observation_view_check_pending = False
            self.get_logger().info(
                'G1 observation framing usable; collecting fresh foot samples'
            )

        estimate = self._estimate_sensor_foot_midpoint_from_pointcloud(
            bbox,
            int(msg.image_width),
            int(msg.image_height),
            msg.header.stamp,
        )
        if estimate is None:
            estimate = self._estimate_sensor_foot_midpoint_from_depth(
                bbox,
                int(msg.image_width),
                int(msg.image_height),
                msg.header.stamp,
            )
        if estimate is None:
            self._record_g1_localization_failure(
                'insufficient foot points or invalid two-foot clustering'
            )
            return

        if not self._g1_foot_midpoint_sample_is_valid(
            estimate,
            old_ground_estimate_base,
        ):
            return

        mid_x_base_m, mid_y_base_m = estimate.midpoint_base
        foot_1_base = estimate.foot_1_base
        foot_2_base = estimate.foot_2_base
        midpoint_map = self._transform_base_xy_to_map(
            mid_x_base_m,
            mid_y_base_m,
            estimate.stamp,
        )
        foot_1_map = self._transform_base_xy_to_map(
            foot_1_base[0],
            foot_1_base[1],
            estimate.stamp,
        )
        foot_2_map = self._transform_base_xy_to_map(
            foot_2_base[0],
            foot_2_base[1],
            estimate.stamp,
        )
        if midpoint_map is None or foot_1_map is None or foot_2_map is None:
            return

        self.g1_sensor_foot_midpoint_samples.append(
            (midpoint_map[0], midpoint_map[1], foot_1_map, foot_2_map)
        )
        if len(self.g1_sensor_foot_midpoint_samples) < 5:
            return

        self.g1_sensor_foot_midpoint_x_base_m = mid_x_base_m
        self.g1_sensor_foot_midpoint_y_base_m = mid_y_base_m
        self.g1_sensor_foot_midpoint_x_map_m = float(
            median(sample[0] for sample in self.g1_sensor_foot_midpoint_samples)
        )
        self.g1_sensor_foot_midpoint_y_map_m = float(
            median(sample[1] for sample in self.g1_sensor_foot_midpoint_samples)
        )
        self.g1_sensor_foot_1_map_m = (
            float(median(sample[2][0] for sample in self.g1_sensor_foot_midpoint_samples)),
            float(median(sample[2][1] for sample in self.g1_sensor_foot_midpoint_samples)),
        )
        self.g1_sensor_foot_2_map_m = (
            float(median(sample[3][0] for sample in self.g1_sensor_foot_midpoint_samples)),
            float(median(sample[3][1] for sample in self.g1_sensor_foot_midpoint_samples)),
        )
        self.g1_map_x_m = self.g1_sensor_foot_midpoint_x_map_m
        self.g1_map_y_m = self.g1_sensor_foot_midpoint_y_map_m
        self.g1_ground_x_base_m = mid_x_base_m
        self.g1_ground_y_base_m = mid_y_base_m
        self.g1_position_source = 'foot_midpoint'
        self._publish_g1_sensor_foot_midpoint_marker()
        self.get_logger().info(
            'G1 foot estimate accepted: '
            f'x={self.g1_map_x_m:.3f}, y={self.g1_map_y_m:.3f}'
        )
        self.get_logger().info(
            'G1 position stabilized: '
            f'x={self.g1_map_x_m:.3f}, y={self.g1_map_y_m:.3f}, '
            f'source={self.g1_position_source}'
        )
        if self.state == MissionState.LOCALIZE_G1:
            self._cancel_g1_localization_timeout()
            self._cancel_g1_observation_trigger()
            self._timing_stop('g1_localization')
            previous_state = self.state
            if (
                self.stop_exploration_on_g1
                and not self.g1_committed
                and not self.g1_observation_pose_reached
            ):
                self.state = MissionState.STOPPING_EXPLORATION
                self.exploration_stop_reason = 'localization_success'
                self._timing_start('exploration_stop')
                self.get_logger().info('G1 localization stabilized; stopping exploration')
                self.get_logger().info(
                    f'Mission state transition: {previous_state.name} -> '
                    f'{self.state.name}'
                )
                self._start_exploration_handoff()
            else:
                if self.g1_observation_pose_reached:
                    self.get_logger().info(
                        'G1 localization stabilized from observation pose'
                    )
                if self.g1_committed:
                    self.get_logger().info(
                        'G1 localization successful; generating parking candidates'
                    )
                self._begin_g1_map_transform(previous_state)
        else:
            self._log_g1_sensor_foot_midpoint_diagnostics()

    def _g1_foot_midpoint_sample_is_valid(
        self,
        estimate: FootMidpointEstimate,
        old_ground_estimate_base: tuple[float, float] | None,
    ) -> bool:
        reasons = []
        if old_ground_estimate_base is None and (
            self.g1_old_ground_estimate_x_base_m is not None
            and self.g1_old_ground_estimate_y_base_m is not None
        ):
            old_ground_estimate_base = (
                self.g1_old_ground_estimate_x_base_m,
                self.g1_old_ground_estimate_y_base_m,
            )

        if (
            estimate.cluster_1_count < self.g1_foot_cluster_min_points
            or estimate.cluster_2_count < self.g1_foot_cluster_min_points
        ):
            reasons.append('insufficient points')

        if (
            estimate.cluster_separation_m < self.g1_foot_cluster_min_separation_m
            or estimate.cluster_separation_m > self.g1_foot_cluster_max_separation_m
        ):
            reasons.append('implausible leg separation')

        if estimate.cluster_balance_ratio < self.g1_foot_cluster_min_balance_ratio:
            reasons.append('cluster imbalance')

        consistency_distance_m = None
        if old_ground_estimate_base is None:
            reasons.append('coarse G1 estimate unavailable')
        else:
            consistency_distance_m = math.hypot(
                estimate.midpoint_base[0] - old_ground_estimate_base[0],
                estimate.midpoint_base[1] - old_ground_estimate_base[1],
            )
            if (
                consistency_distance_m
                > self.g1_foot_to_old_estimate_max_distance_m
            ):
                reasons.append('inconsistent with coarse G1 estimate')

        self.get_logger().info(
            'G1 foot localization sample: '
            f'source={estimate.source}, '
            f'total_points={estimate.total_lower_body_points}, '
            f'cluster_counts=({estimate.cluster_1_count}, '
            f'{estimate.cluster_2_count}), '
            f'separation={estimate.cluster_separation_m:.3f} m, '
            f'balance={estimate.cluster_balance_ratio:.2f}, '
            f'range_spread={estimate.range_spread_m:.3f} m, '
            f'height_spread={estimate.height_spread_m:.3f} m, '
            f'foot_1=({estimate.foot_1_base[0]:.3f}, '
            f'{estimate.foot_1_base[1]:.3f}), '
            f'foot_2=({estimate.foot_2_base[0]:.3f}, '
            f'{estimate.foot_2_base[1]:.3f}), '
            f'midpoint=({estimate.midpoint_base[0]:.3f}, '
            f'{estimate.midpoint_base[1]:.3f})'
        )
        if old_ground_estimate_base is not None and consistency_distance_m is not None:
            self.get_logger().info(
                'G1 foot/old estimate consistency: '
                f'foot_midpoint=({estimate.midpoint_base[0]:.3f}, '
                f'{estimate.midpoint_base[1]:.3f}), '
                f'old_ground_estimate=({old_ground_estimate_base[0]:.3f}, '
                f'{old_ground_estimate_base[1]:.3f}), '
                f'consistency_distance={consistency_distance_m:.3f} m'
            )

        if reasons:
            if 'inconsistent with coarse G1 estimate' in reasons:
                self.get_logger().warn(
                    'G1 foot localization sample rejected: '
                    'inconsistent with coarse G1 estimate'
                )
            self.get_logger().warn(
                'G1 foot localization sample rejected: low confidence '
                f'({", ".join(reasons)})'
            )
            return False

        self.get_logger().info(
            'G1 foot localization sample accepted: '
            f'{len(self.g1_sensor_foot_midpoint_samples) + 1}/5 valid samples'
        )
        return True

    def _process_g1_body_position_sample(
        self,
        msg: G1Measurements,
        old_ground_estimate_base: tuple[float, float] | None = None,
    ) -> None:
        if self.g1_map_x_m is not None and self.g1_map_y_m is not None:
            return
        if not msg.detected:
            return

        bbox = self._select_highest_score_measurement_bbox(msg)
        if bbox is None:
            return
        self.latest_g1_detection_bbox = bbox
        self.recent_g1_detection_bboxes.append((bbox, msg.header.stamp))

        estimates = self._g1_body_position_estimate_candidates(
            bbox,
            int(msg.image_width),
            int(msg.image_height),
            msg.header.stamp,
        )
        if not estimates:
            self._record_g1_localization_failure(
                'body ROI insufficient or sensor timestamp mismatch'
            )
            return

        for estimate in estimates:
            accepted, reason = self._g1_body_position_sample_is_valid(
                bbox,
                int(msg.image_width),
                int(msg.image_height),
                estimate,
                old_ground_estimate_base,
            )
            self.get_logger().info(
                'G1 depth-body estimate: '
                f'x={estimate.body_map[0]:.3f}, '
                f'y={estimate.body_map[1]:.3f}, '
                f'valid_points={estimate.valid_points}, '
                f'source={estimate.source}, '
                f'roi={estimate.roi_xyxy}, '
                f'range_spread={estimate.range_spread_m:.3f} m, '
                f'height_spread={estimate.height_spread_m:.3f} m'
            )
            if accepted:
                self._accept_g1_body_position_sample(estimate)
                return

            if reason == 'bbox clipped at side image edge':
                self.get_logger().warn(
                    'G1 body estimate rejected: bbox clipped at side image edge'
                )
            elif reason == 'inconsistent with coarse G1 estimate':
                self.get_logger().warn(
                    'G1 body estimate rejected: inconsistent with coarse G1 estimate'
                )
            else:
                self.get_logger().warn(f'G1 body estimate rejected: {reason}')

    def _g1_body_position_estimate_candidates(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> list[G1BodyPositionEstimate]:
        estimates: list[G1BodyPositionEstimate] = []
        for estimate in (
            self._estimate_g1_body_position_from_pointcloud(
                bbox,
                detection_image_width,
                detection_image_height,
                detection_stamp,
            ),
            self._estimate_g1_body_position_from_depth(
                bbox,
                detection_image_width,
                detection_image_height,
                detection_stamp,
            ),
            self._estimate_g1_body_position_from_raw_pointcloud_base_like(
                bbox,
                detection_image_width,
                detection_image_height,
                detection_stamp,
            ),
        ):
            if estimate is not None:
                estimates.append(estimate)
        return estimates

    def _g1_body_position_sample_is_valid(
        self,
        bbox: tuple[float, float, float, float],
        image_width_px: int,
        image_height_px: int,
        estimate: G1BodyPositionEstimate,
        old_ground_estimate_base: tuple[float, float] | None,
    ) -> tuple[bool, str]:
        margins = self._g1_bbox_edge_margins(
            bbox,
            image_width_px,
            image_height_px,
        )
        if margins is None:
            self._log_g1_body_candidate(
                bbox,
                image_width_px,
                image_height_px,
                None,
                estimate,
                None,
                None,
                False,
                'invalid image or bbox',
            )
            return False, 'invalid image or bbox'

        left_px, right_px, top_px, bottom_px = margins
        side_clipped = (
            min(left_px, right_px)
            < float(self.g1_body_bbox_edge_margin_px)
        )

        coarse_map = self._coarse_g1_estimate_map_for_body_gate(
            old_ground_estimate_base,
            estimate.stamp,
        )
        consistency_distance_m = None
        if coarse_map is not None:
            consistency_distance_m = math.hypot(
                estimate.body_map[0] - coarse_map[0],
                estimate.body_map[1] - coarse_map[1],
            )

        reason = 'accepted'
        accepted = True
        if side_clipped:
            accepted = False
            reason = 'bbox clipped at side image edge'
        elif coarse_map is None:
            accepted = False
            reason = 'coarse G1 estimate unavailable'
        elif (
            consistency_distance_m is None
            or consistency_distance_m > self.g1_body_to_old_estimate_max_distance_m
        ):
            accepted = False
            reason = 'inconsistent with coarse G1 estimate'

        self._log_g1_body_candidate(
            bbox,
            image_width_px,
            image_height_px,
            (left_px, right_px, top_px, bottom_px),
            estimate,
            coarse_map,
            consistency_distance_m,
            accepted,
            reason,
        )
        return accepted, reason

    @staticmethod
    def _g1_bbox_edge_margins(
        bbox: tuple[float, float, float, float],
        image_width_px: int,
        image_height_px: int,
    ) -> tuple[float, float, float, float] | None:
        if image_width_px <= 0 or image_height_px <= 0:
            return None
        x1, y1, x2, y2 = bbox
        if x2 <= x1 or y2 <= y1:
            return None
        return (
            float(x1),
            float(image_width_px) - float(x2),
            float(y1),
            float(image_height_px) - float(y2),
        )

    def _coarse_g1_estimate_map_for_body_gate(
        self,
        old_ground_estimate_base: tuple[float, float] | None,
        stamp: TimeMsg,
    ) -> tuple[float, float] | None:
        if old_ground_estimate_base is not None:
            return self._transform_base_xy_to_map(
                old_ground_estimate_base[0],
                old_ground_estimate_base[1],
                stamp,
            )
        if (
            self.g1_old_ground_estimate_x_map_m is not None
            and self.g1_old_ground_estimate_y_map_m is not None
        ):
            return (
                self.g1_old_ground_estimate_x_map_m,
                self.g1_old_ground_estimate_y_map_m,
            )
        if (
            self.g1_old_ground_estimate_x_base_m is not None
            and self.g1_old_ground_estimate_y_base_m is not None
        ):
            return self._transform_base_xy_to_map(
                self.g1_old_ground_estimate_x_base_m,
                self.g1_old_ground_estimate_y_base_m,
                stamp,
            )
        return None

    def _log_g1_body_candidate(
        self,
        bbox: tuple[float, float, float, float],
        image_width_px: int,
        image_height_px: int,
        margins: tuple[float, float, float, float] | None,
        estimate: G1BodyPositionEstimate,
        coarse_map: tuple[float, float] | None,
        consistency_distance_m: float | None,
        accepted: bool,
        reason: str,
    ) -> None:
        margins_text = (
            'n/a'
            if margins is None
            else (
                f'left={margins[0]:.1f}, right={margins[1]:.1f}, '
                f'top={margins[2]:.1f}, bottom={margins[3]:.1f}'
            )
        )
        coarse_text = (
            'n/a'
            if coarse_map is None
            else f'({coarse_map[0]:.3f}, {coarse_map[1]:.3f})'
        )
        consistency_text = (
            'n/a'
            if consistency_distance_m is None
            else f'{consistency_distance_m:.3f} m'
        )
        self.get_logger().info(
            'G1 depth-body candidate: '
            f'bbox=({bbox[0]:.1f}, {bbox[1]:.1f}, '
            f'{bbox[2]:.1f}, {bbox[3]:.1f}), '
            f'image=({image_width_px}x{image_height_px}), '
            f'edge_margins=({margins_text}), '
            f'map=({estimate.body_map[0]:.3f}, {estimate.body_map[1]:.3f}), '
            f'coarse={coarse_text}, '
            f'consistency_distance={consistency_text}, '
            f'accepted={str(accepted).lower()}, '
            f'reason={reason}'
        )

    def _accept_g1_body_position_sample(
        self,
        estimate: G1BodyPositionEstimate,
    ) -> None:
        if self.g1_body_position_samples:
            median_x_m = float(median(sample[0] for sample in self.g1_body_position_samples))
            median_y_m = float(median(sample[1] for sample in self.g1_body_position_samples))
            jump_m = math.hypot(
                estimate.body_map[0] - median_x_m,
                estimate.body_map[1] - median_y_m,
            )
            if jump_m > self.g1_body_sample_max_jump_m:
                self.get_logger().warn(
                    'G1 depth-body estimate rejected: '
                    f'jump={jump_m:.3f} m exceeds '
                    f'{self.g1_body_sample_max_jump_m:.3f} m'
                )
                return

        self.g1_body_position_samples.append(estimate.body_map)
        sample_count = len(self.g1_body_position_samples)
        self.get_logger().info(
            'G1 depth-body estimate accepted: '
            f'{sample_count}/{self.g1_body_sample_count} valid samples'
        )
        if sample_count < self.g1_body_sample_count:
            return

        self.g1_body_position_x_map_m = float(
            median(sample[0] for sample in self.g1_body_position_samples)
        )
        self.g1_body_position_y_map_m = float(
            median(sample[1] for sample in self.g1_body_position_samples)
        )
        self.g1_map_x_m = self.g1_body_position_x_map_m
        self.g1_map_y_m = self.g1_body_position_y_map_m
        self.g1_ground_x_base_m = estimate.body_base[0]
        self.g1_ground_y_base_m = estimate.body_base[1]
        self.g1_position_source = 'depth_body_projection'

        self.get_logger().info(
            'G1 position stabilized: '
            f'x={self.g1_map_x_m:.3f}, y={self.g1_map_y_m:.3f}, '
            f'source={self.g1_position_source}'
        )
        if self.state == MissionState.LOCALIZE_G1:
            self._cancel_g1_localization_timeout()
            self._cancel_g1_observation_trigger()
            self._timing_stop('g1_localization')
            previous_state = self.state
            self.get_logger().info(
                'G1 localization successful; generating parking candidates'
            )
            self._begin_g1_map_transform(previous_state)

    def _estimate_g1_body_position_from_pointcloud(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> G1BodyPositionEstimate | None:
        pointcloud_msg = self._matched_pointcloud(detection_stamp)
        if pointcloud_msg is None:
            return None
        if pointcloud_msg.height <= 1 or pointcloud_msg.width <= 1:
            return None

        try:
            pointcloud_xyz = extract_organized_xyz(
                pointcloud_msg,
                (int(pointcloud_msg.height), int(pointcloud_msg.width)),
            )
        except ValueError:
            return None

        roi_xyxy = self._g1_body_roi_xyxy(
            bbox,
            detection_image_width,
            detection_image_height,
            int(pointcloud_msg.width),
            int(pointcloud_msg.height),
        )
        if roi_xyxy is None:
            return None
        x1, y1, x2, y2 = roi_xyxy
        roi_points = pointcloud_xyz[y1:y2, x1:x2, :].reshape(-1, 3)
        if roi_points.shape[0] < self.g1_body_min_points:
            return None

        valid = np.all(np.isfinite(roi_points), axis=1)
        if int(np.count_nonzero(valid)) < self.g1_body_min_points:
            return None
        roi_points = roi_points[valid]

        points_base = self._transform_points_to_base(
            roi_points,
            pointcloud_msg.header.frame_id,
            pointcloud_msg.header.stamp,
        )
        if points_base is None:
            return None

        return self._estimate_g1_body_position_from_base_points(
            points_base,
            'pointcloud_body',
            pointcloud_msg.header.stamp,
            roi_xyxy,
        )

    def _estimate_g1_body_position_from_depth(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> G1BodyPositionEstimate | None:
        depth_frame = self._matched_depth_frame(detection_stamp)
        camera_info = self.latest_camera_info
        if depth_frame is None or camera_info is None:
            return None
        depth_msg, depth_meters = depth_frame
        if depth_meters.ndim != 2:
            return None

        depth_h, depth_w = depth_meters.shape[:2]
        roi_xyxy = self._g1_body_roi_xyxy(
            bbox,
            detection_image_width,
            detection_image_height,
            depth_w,
            depth_h,
        )
        if roi_xyxy is None:
            return None
        x1, y1, x2, y2 = roi_xyxy
        roi = depth_meters[y1:y2, x1:x2]
        if roi.size < self.g1_body_min_points:
            return None

        intrinsics = self._scaled_camera_intrinsics(
            camera_info,
            depth_w,
            depth_h,
        )
        if intrinsics is None:
            return None
        fx, fy, cx, cy = intrinsics

        us = np.arange(x1, x2, dtype=np.float32)
        vs = np.arange(y1, y2, dtype=np.float32)
        grid_u, grid_v = np.meshgrid(us, vs)
        z_optical = roi.astype(np.float32, copy=False)
        valid = np.isfinite(z_optical) & (z_optical > 0.0)
        if int(np.count_nonzero(valid)) < self.g1_body_min_points:
            return None

        x_optical = ((grid_u - cx) / fx) * z_optical
        y_optical = ((grid_v - cy) / fy) * z_optical
        points_optical = np.stack(
            (x_optical[valid], y_optical[valid], z_optical[valid]),
            axis=1,
        )
        if points_optical.shape[0] < self.g1_body_min_points:
            return None

        source_frame = depth_msg.header.frame_id or self.camera_optical_frame
        points_base = self._transform_points_to_base(
            points_optical,
            source_frame,
            depth_msg.header.stamp,
        )
        if points_base is None:
            return None

        return self._estimate_g1_body_position_from_base_points(
            points_base,
            'depth_body',
            depth_msg.header.stamp,
            roi_xyxy,
        )

    def _estimate_g1_body_position_from_base_points(
        self,
        points_base: np.ndarray,
        source: str,
        stamp: TimeMsg,
        roi_xyxy: tuple[int, int, int, int],
    ) -> G1BodyPositionEstimate | None:
        valid_base = np.all(np.isfinite(points_base), axis=1)
        valid_base &= points_base[:, 0] > 0.0
        valid_base &= points_base[:, 2] >= self.g1_body_min_height_m
        valid_base &= points_base[:, 2] <= self.g1_body_max_height_m
        if int(np.count_nonzero(valid_base)) < self.g1_body_min_points:
            return None

        body_points = points_base[valid_base]
        if body_points.shape[0] < self.g1_body_min_points:
            return None

        forward_low = float(np.percentile(body_points[:, 0], 15.0))
        forward_high = float(np.percentile(body_points[:, 0], 85.0))
        lateral_low = float(np.percentile(body_points[:, 1], 10.0))
        lateral_high = float(np.percentile(body_points[:, 1], 90.0))
        trimmed = (
            (body_points[:, 0] >= forward_low)
            & (body_points[:, 0] <= forward_high)
            & (body_points[:, 1] >= lateral_low)
            & (body_points[:, 1] <= lateral_high)
        )
        if int(np.count_nonzero(trimmed)) >= self.g1_body_min_points:
            body_points = body_points[trimmed]

        body_base = np.median(body_points, axis=0)
        body_x_base_m = float(body_base[0])
        body_y_base_m = float(body_base[1])
        body_z_base_m = float(body_base[2])
        if not all(
            math.isfinite(value)
            for value in (body_x_base_m, body_y_base_m, body_z_base_m)
        ):
            return None

        body_map = self._transform_base_xy_to_map(
            body_x_base_m,
            body_y_base_m,
            stamp,
        )
        if body_map is None:
            return None

        return G1BodyPositionEstimate(
            body_base=(body_x_base_m, body_y_base_m, body_z_base_m),
            body_map=body_map,
            valid_points=int(body_points.shape[0]),
            roi_xyxy=roi_xyxy,
            range_spread_m=float(np.ptp(body_points[:, 0])),
            height_spread_m=float(np.ptp(body_points[:, 2])),
            source=source,
            stamp=stamp,
        )

    def _estimate_g1_body_position_from_raw_pointcloud_base_like(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> G1BodyPositionEstimate | None:
        pointcloud_msg = self._matched_pointcloud(detection_stamp)
        if pointcloud_msg is None:
            return None
        if pointcloud_msg.height <= 1 or pointcloud_msg.width <= 1:
            return None

        try:
            pointcloud_xyz = extract_organized_xyz(
                pointcloud_msg,
                (int(pointcloud_msg.height), int(pointcloud_msg.width)),
            )
        except ValueError:
            return None

        roi_xyxy = self._g1_body_roi_xyxy(
            bbox,
            detection_image_width,
            detection_image_height,
            int(pointcloud_msg.width),
            int(pointcloud_msg.height),
        )
        if roi_xyxy is None:
            return None
        x1, y1, x2, y2 = roi_xyxy
        roi_points = pointcloud_xyz[y1:y2, x1:x2, :].reshape(-1, 3)
        if roi_points.shape[0] < self.g1_body_min_points:
            return None

        valid = np.all(np.isfinite(roi_points), axis=1)
        if int(np.count_nonzero(valid)) < self.g1_body_min_points:
            return None
        roi_points = roi_points[valid].astype(np.float32, copy=True)

        source_frame = pointcloud_msg.header.frame_id
        if not source_frame:
            source_frame = self.camera_optical_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                source_frame,
                rclpy.time.Time.from_msg(pointcloud_msg.header.stamp),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
            except TransformException:
                return None

        # Gazebo can publish an organized cloud whose numeric axes are already
        # camera-link/base-like (+x forward, +y left, +z up) while the header is
        # an optical frame. Reuse the real sensor translation but not the
        # misleading optical rotation for this diagnostic fallback.
        translation = transform.transform.translation
        points_base_like = roi_points.copy()
        points_base_like[:, 0] += float(translation.x)
        points_base_like[:, 1] += float(translation.y)
        points_base_like[:, 2] += float(translation.z)

        return self._estimate_g1_body_position_from_base_points(
            points_base_like,
            'pointcloud_body_raw_link',
            pointcloud_msg.header.stamp,
            roi_xyxy,
        )

    def _g1_body_roi_xyxy(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        target_width: int,
        target_height: int,
    ) -> tuple[int, int, int, int] | None:
        x1, y1, x2, y2 = map_bbox_between_images(
            bbox,
            detection_image_width,
            detection_image_height,
            target_width,
            target_height,
        )
        if x2 <= x1 or y2 <= y1:
            return None

        width = max(x2 - x1, 1)
        height = max(y2 - y1, 1)
        x_min_fraction = max(0.0, min(1.0, self.g1_body_roi_x_min_fraction))
        x_max_fraction = max(0.0, min(1.0, self.g1_body_roi_x_max_fraction))
        y_min_fraction = max(0.0, min(1.0, self.g1_body_roi_y_min_fraction))
        y_max_fraction = max(0.0, min(1.0, self.g1_body_roi_y_max_fraction))
        if x_max_fraction <= x_min_fraction or y_max_fraction <= y_min_fraction:
            return None

        roi_x1 = x1 + int(math.floor(width * x_min_fraction))
        roi_x2 = x1 + int(math.ceil(width * x_max_fraction))
        roi_y1 = y1 + int(math.floor(height * y_min_fraction))
        roi_y2 = y1 + int(math.ceil(height * y_max_fraction))
        roi_x1 = max(0, min(target_width - 1, roi_x1))
        roi_x2 = max(roi_x1 + 1, min(target_width, roi_x2))
        roi_y1 = max(0, min(target_height - 1, roi_y1))
        roi_y2 = max(roi_y1 + 1, min(target_height, roi_y2))
        if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
            return None
        return roi_x1, roi_y1, roi_x2, roi_y2

    def _transform_points_to_base(
        self,
        points: np.ndarray,
        source_frame: str,
        stamp: TimeMsg,
    ) -> np.ndarray | None:
        if not source_frame:
            source_frame = self.camera_optical_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                source_frame,
                rclpy.time.Time.from_msg(stamp),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
            except TransformException:
                return None

        rotation = np.asarray(
            rotation_matrix_from_quaternion(transform.transform.rotation),
            dtype=np.float32,
        )
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        return points @ rotation.T + translation

    def _select_highest_score_bbox(
        self,
        msg: G1Detections,
    ) -> tuple[float, float, float, float] | None:
        selection = self._select_highest_score_detection(msg)
        return None if selection is None else selection[0]

    def _select_highest_score_detection(
        self,
        msg: G1Detections,
    ) -> tuple[tuple[float, float, float, float], float] | None:
        if not msg.detected:
            return None

        best_score = float('-inf')
        best_bbox = None
        bbox_count = len(msg.bbox_xyxy) // 4
        detection_count = min(int(msg.count), bbox_count)

        for index in range(detection_count):
            bbox_start = index * 4
            bbox = tuple(
                float(value)
                for value in msg.bbox_xyxy[bbox_start:bbox_start + 4]
            )
            if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
                continue

            score = float(msg.scores[index]) if index < len(msg.scores) else 0.0
            if math.isfinite(score) and score > best_score:
                best_score = score
                best_bbox = bbox

        if best_bbox is None:
            return None
        return best_bbox, best_score

    def _select_highest_score_measurement_bbox(
        self,
        msg: G1Measurements,
    ) -> tuple[float, float, float, float] | None:
        if not msg.detected:
            return None

        best_score = float('-inf')
        best_bbox = None
        bbox_count = len(msg.bbox_xyxy) // 4
        detection_count = min(int(msg.count), bbox_count)
        for index in range(detection_count):
            bbox_start = index * 4
            bbox = tuple(
                float(value)
                for value in msg.bbox_xyxy[bbox_start:bbox_start + 4]
            )
            if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
                continue

            score = float(msg.scores[index]) if index < len(msg.scores) else 0.0
            if math.isfinite(score) and score > best_score:
                best_score = score
                best_bbox = bbox

        return best_bbox

    def _select_highest_score_camera_center_measurement(
        self,
        msg: G1Measurements,
    ) -> tuple[tuple[float, float, float, float], float, float, float] | None:
        if not msg.detected:
            return None

        best_score = float('-inf')
        best_selection = None
        bbox_count = len(msg.bbox_xyxy) // 4
        detection_count = min(
            int(msg.count),
            bbox_count,
            len(msg.pointcloud_forward_m),
            len(msg.pointcloud_lateral_m),
            len(msg.sensor_depth_distance_m),
        )

        for index in range(detection_count):
            bbox_start = index * 4
            bbox = tuple(
                float(value)
                for value in msg.bbox_xyxy[bbox_start:bbox_start + 4]
            )
            if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
                continue

            pointcloud_forward_m = float(msg.pointcloud_forward_m[index])
            pointcloud_lateral_m = float(msg.pointcloud_lateral_m[index])
            sensor_depth_distance_m = float(msg.sensor_depth_distance_m[index])
            if (
                not math.isfinite(pointcloud_forward_m)
                or not math.isfinite(pointcloud_lateral_m)
                or not math.isfinite(sensor_depth_distance_m)
                or sensor_depth_distance_m <= 0.0
            ):
                continue

            score = float(msg.scores[index]) if index < len(msg.scores) else 0.0
            if math.isfinite(score) and score > best_score:
                best_score = score
                best_selection = (
                    bbox,
                    pointcloud_forward_m,
                    pointcloud_lateral_m,
                    sensor_depth_distance_m,
                )

        return best_selection

    def _compute_ground_sample_from_camera_measurement(
        self,
        bbox: tuple[float, float, float, float],
        pointcloud_forward_m: float,
        pointcloud_lateral_m: float,
        sensor_depth_distance_m: float,
    ) -> tuple[float, float] | None:
        bearing = self._compute_bbox_horizontal_bearing_in_base(bbox)
        if bearing is None:
            return None

        bearing_x_base, bearing_y_base = bearing
        near_x_base_m = float(pointcloud_forward_m + ROBOT_FRONT_OFFSET_M)
        near_y_base_m = float(pointcloud_lateral_m)
        far_x_base_m = float(bearing_x_base * sensor_depth_distance_m)
        far_y_base_m = float(bearing_y_base * sensor_depth_distance_m)

        ground_x_base_m = (near_x_base_m + far_x_base_m) / 2.0
        ground_y_base_m = (near_y_base_m + far_y_base_m) / 2.0
        if not math.isfinite(ground_x_base_m) or not math.isfinite(ground_y_base_m):
            return None

        return ground_x_base_m, ground_y_base_m

    def _g1_observation_framing(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> G1ObservationFraming:
        _, y1, _, y2 = bbox
        image_height_px = max(0, int(detection_image_height))
        bbox_bottom_px = float(y2)
        bottom_margin_px = float(image_height_px) - bbox_bottom_px
        bbox_height_px = max(0.0, float(y2 - y1))
        lower_body_depth_points = self._count_lower_body_depth_points(
            bbox,
            detection_image_width,
            detection_image_height,
            detection_stamp,
        )
        lower_body_cloud_points = self._count_lower_body_pointcloud_points(
            bbox,
            detection_image_width,
            detection_image_height,
            detection_stamp,
        )
        lower_body_points = max(lower_body_depth_points, lower_body_cloud_points)
        reasons = []

        if image_height_px <= 0:
            reasons.append('invalid image height')
        if bottom_margin_px < float(self.g1_observation_bottom_margin_px):
            reasons.append('bbox clipped at bottom')
        if lower_body_points < self.g1_observation_lower_body_min_points:
            reasons.append('insufficient lower-body depth support')

        return G1ObservationFraming(
            bbox_bottom_px=bbox_bottom_px,
            image_height_px=image_height_px,
            bottom_margin_px=bottom_margin_px,
            bbox_height_px=bbox_height_px,
            lower_body_depth_points=lower_body_depth_points,
            lower_body_cloud_points=lower_body_cloud_points,
            usable=not reasons,
            reason=', '.join(reasons) if reasons else 'ok',
        )

    def _count_lower_body_depth_points(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> int:
        depth_frame = self._matched_depth_frame(detection_stamp)
        if depth_frame is None:
            return 0
        _, depth_meters = depth_frame
        if depth_meters.ndim != 2:
            return 0

        depth_h, depth_w = depth_meters.shape[:2]
        x1, y1, x2, y2 = map_bbox_between_images(
            bbox,
            detection_image_width,
            detection_image_height,
            depth_w,
            depth_h,
        )
        lower_y1 = self._lower_body_roi_y1(y1, y2)
        if x2 <= x1 or y2 <= lower_y1:
            return 0

        roi = depth_meters[lower_y1:y2, x1:x2]
        if roi.size == 0:
            return 0

        valid = np.isfinite(roi) & (roi > 0.0)
        return int(np.count_nonzero(valid))

    def _count_lower_body_pointcloud_points(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> int:
        pointcloud_msg = self._matched_pointcloud(detection_stamp)
        if pointcloud_msg is None:
            return 0
        if pointcloud_msg.height <= 1 or pointcloud_msg.width <= 1:
            return 0

        try:
            pointcloud_xyz = extract_organized_xyz(
                pointcloud_msg,
                (int(pointcloud_msg.height), int(pointcloud_msg.width)),
            )
        except ValueError:
            return 0

        x1, y1, x2, y2 = map_bbox_between_images(
            bbox,
            detection_image_width,
            detection_image_height,
            int(pointcloud_msg.width),
            int(pointcloud_msg.height),
        )
        lower_y1 = self._lower_body_roi_y1(y1, y2)
        if x2 <= x1 or y2 <= lower_y1:
            return 0

        roi_points = pointcloud_xyz[lower_y1:y2, x1:x2, :].reshape(-1, 3)
        if roi_points.shape[0] == 0:
            return 0

        valid = np.all(np.isfinite(roi_points), axis=1)
        return int(np.count_nonzero(valid))

    @staticmethod
    def _lower_body_roi_y1(y1: int, y2: int) -> int:
        bbox_height = max(y2 - y1, 1)
        lower_y1 = y1 + int(math.floor(0.55 * bbox_height))
        return max(y1, min(y2 - 1, lower_y1))

    def _log_g1_observation_framing(
        self,
        framing: G1ObservationFraming,
    ) -> None:
        self.get_logger().info(
            'G1 observation framing: '
            f'bbox_bottom={framing.bbox_bottom_px:.0f}/'
            f'{framing.image_height_px}, '
            f'bottom_margin={framing.bottom_margin_px:.0f} px, '
            f'bbox_height={framing.bbox_height_px:.0f} px, '
            f'lower_body_depth_points={framing.lower_body_depth_points}, '
            f'lower_body_cloud_points={framing.lower_body_cloud_points}, '
            f'usable={str(framing.usable).lower()}, '
            f'reason={framing.reason}'
        )

    def _estimate_sensor_foot_midpoint_from_pointcloud(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> FootMidpointEstimate | None:
        pointcloud_msg = self._matched_pointcloud(detection_stamp)
        if pointcloud_msg is None:
            return None
        if pointcloud_msg.height <= 1 or pointcloud_msg.width <= 1:
            return None

        try:
            pointcloud_xyz = extract_organized_xyz(
                pointcloud_msg,
                (int(pointcloud_msg.height), int(pointcloud_msg.width)),
            )
        except ValueError:
            return None

        x1, y1, x2, y2 = map_bbox_between_images(
            bbox,
            detection_image_width,
            detection_image_height,
            int(pointcloud_msg.width),
            int(pointcloud_msg.height),
        )
        lower_y1 = self._lower_body_roi_y1(y1, y2)

        roi_points = pointcloud_xyz[lower_y1:y2, x1:x2, :].reshape(-1, 3)
        if roi_points.shape[0] < 20:
            return None
        valid = np.all(np.isfinite(roi_points), axis=1)
        if int(np.count_nonzero(valid)) < 20:
            return None
        roi_points = roi_points[valid]

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                pointcloud_msg.header.frame_id,
                rclpy.time.Time.from_msg(pointcloud_msg.header.stamp),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    pointcloud_msg.header.frame_id,
                    rclpy.time.Time(),
                )
            except TransformException:
                return None

        rotation = np.asarray(
            rotation_matrix_from_quaternion(transform.transform.rotation),
            dtype=np.float32,
        )
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        points_base = roi_points @ rotation.T + translation

        # Keep lower-leg / foot points: above the floor plane, below knees,
        # in front of the robot, and near the visible G1 surface rather than
        # the room background.
        valid_base = np.all(np.isfinite(points_base), axis=1)
        valid_base &= points_base[:, 0] > 0.0
        valid_base &= points_base[:, 2] > 0.035
        valid_base &= points_base[:, 2] < 0.45
        if int(np.count_nonzero(valid_base)) < 20:
            return None

        lower_points = points_base[valid_base]
        anchor_x = float(np.percentile(lower_points[:, 0], 20.0))
        near = (
            (lower_points[:, 0] >= anchor_x - 0.10)
            & (lower_points[:, 0] <= anchor_x + 0.60)
        )
        if int(np.count_nonzero(near)) < 20:
            return None
        lower_near_points = lower_points[near]
        lower_xy = lower_near_points[:, :2].astype(np.float32, copy=False)

        clusters = self._cluster_two_foot_points(lower_xy)
        if clusters is None:
            return None
        foot_1_base, foot_2_base, cluster_1_count, cluster_2_count = clusters
        mid_x_base_m = (foot_1_base[0] + foot_2_base[0]) / 2.0
        mid_y_base_m = (foot_1_base[1] + foot_2_base[1]) / 2.0
        separation_m = math.hypot(
            foot_2_base[0] - foot_1_base[0],
            foot_2_base[1] - foot_1_base[1],
        )
        balance_ratio = min(cluster_1_count, cluster_2_count) / max(
            cluster_1_count,
            cluster_2_count,
        )
        return FootMidpointEstimate(
            midpoint_base=(mid_x_base_m, mid_y_base_m),
            foot_1_base=foot_1_base,
            foot_2_base=foot_2_base,
            total_lower_body_points=int(lower_xy.shape[0]),
            cluster_1_count=cluster_1_count,
            cluster_2_count=cluster_2_count,
            cluster_separation_m=separation_m,
            cluster_balance_ratio=float(balance_ratio),
            range_spread_m=float(np.ptp(lower_near_points[:, 0])),
            height_spread_m=float(np.ptp(lower_near_points[:, 2])),
            source='pointcloud',
            stamp=pointcloud_msg.header.stamp,
        )

    def _estimate_sensor_foot_midpoint_from_depth(
        self,
        bbox: tuple[float, float, float, float],
        detection_image_width: int,
        detection_image_height: int,
        detection_stamp: TimeMsg,
    ) -> FootMidpointEstimate | None:
        depth_frame = self._matched_depth_frame(detection_stamp)
        camera_info = self.latest_camera_info
        if depth_frame is None or camera_info is None:
            return None
        depth_msg, depth_meters = depth_frame
        if depth_meters.ndim != 2:
            return None

        depth_h, depth_w = depth_meters.shape[:2]
        x1, y1, x2, y2 = map_bbox_between_images(
            bbox,
            detection_image_width,
            detection_image_height,
            depth_w,
            depth_h,
        )
        lower_y1 = self._lower_body_roi_y1(y1, y2)

        roi = depth_meters[lower_y1:y2, x1:x2]
        if roi.size < 20:
            return None

        intrinsics = self._scaled_camera_intrinsics(
            camera_info,
            depth_w,
            depth_h,
        )
        if intrinsics is None:
            return None
        fx, fy, cx, cy = intrinsics

        us = np.arange(x1, x2, dtype=np.float32)
        vs = np.arange(lower_y1, y2, dtype=np.float32)
        grid_u, grid_v = np.meshgrid(us, vs)
        z_optical = roi.astype(np.float32, copy=False)
        valid = np.isfinite(z_optical) & (z_optical > 0.0)
        if int(np.count_nonzero(valid)) < 20:
            return None

        x_optical = ((grid_u - cx) / fx) * z_optical
        y_optical = ((grid_v - cy) / fy) * z_optical
        points_optical = np.stack(
            (x_optical[valid], y_optical[valid], z_optical[valid]),
            axis=1,
        )
        if points_optical.shape[0] < 20:
            return None

        source_frame = depth_msg.header.frame_id or self.camera_optical_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                source_frame,
                rclpy.time.Time.from_msg(depth_msg.header.stamp),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    source_frame,
                    rclpy.time.Time(),
                )
            except TransformException:
                return None

        rotation = np.asarray(
            rotation_matrix_from_quaternion(transform.transform.rotation),
            dtype=np.float32,
        )
        translation = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        points_base = points_optical @ rotation.T + translation
        return self._estimate_foot_midpoint_from_base_points(
            points_base,
            'depth',
            depth_msg.header.stamp,
        )

    def _estimate_foot_midpoint_from_base_points(
        self,
        points_base: np.ndarray,
        source: str,
        stamp: TimeMsg,
    ) -> FootMidpointEstimate | None:
        # Keep lower-leg / foot points: above the floor plane, below knees,
        # in front of the robot, and near the visible G1 surface rather than
        # the room background.
        valid_base = np.all(np.isfinite(points_base), axis=1)
        valid_base &= points_base[:, 0] > 0.0
        valid_base &= points_base[:, 2] > 0.035
        valid_base &= points_base[:, 2] < 0.45
        if int(np.count_nonzero(valid_base)) < 20:
            return None

        lower_points = points_base[valid_base]
        anchor_x = float(np.percentile(lower_points[:, 0], 20.0))
        near = (
            (lower_points[:, 0] >= anchor_x - 0.10)
            & (lower_points[:, 0] <= anchor_x + 0.60)
        )
        if int(np.count_nonzero(near)) < 20:
            return None
        lower_near_points = lower_points[near]
        lower_xy = lower_near_points[:, :2].astype(np.float32, copy=False)

        clusters = self._cluster_two_foot_points(lower_xy)
        if clusters is None:
            return None
        foot_1_base, foot_2_base, cluster_1_count, cluster_2_count = clusters
        mid_x_base_m = (foot_1_base[0] + foot_2_base[0]) / 2.0
        mid_y_base_m = (foot_1_base[1] + foot_2_base[1]) / 2.0
        separation_m = math.hypot(
            foot_2_base[0] - foot_1_base[0],
            foot_2_base[1] - foot_1_base[1],
        )
        balance_ratio = min(cluster_1_count, cluster_2_count) / max(
            cluster_1_count,
            cluster_2_count,
        )
        return FootMidpointEstimate(
            midpoint_base=(mid_x_base_m, mid_y_base_m),
            foot_1_base=foot_1_base,
            foot_2_base=foot_2_base,
            total_lower_body_points=int(lower_xy.shape[0]),
            cluster_1_count=cluster_1_count,
            cluster_2_count=cluster_2_count,
            cluster_separation_m=separation_m,
            cluster_balance_ratio=float(balance_ratio),
            range_spread_m=float(np.ptp(lower_near_points[:, 0])),
            height_spread_m=float(np.ptp(lower_near_points[:, 2])),
            source=source,
            stamp=stamp,
        )

    def _cluster_two_foot_points(
        self,
        points_xy: np.ndarray,
    ) -> tuple[tuple[float, float], tuple[float, float], int, int] | None:
        if points_xy.shape[0] < 20:
            return None

        spread = np.ptp(points_xy, axis=0)
        axis = 1 if spread[1] >= spread[0] else 0
        order = np.argsort(points_xy[:, axis])
        center_a = points_xy[order[0]].astype(np.float32)
        center_b = points_xy[order[-1]].astype(np.float32)
        if float(np.linalg.norm(center_a - center_b)) < 0.05:
            return None

        labels = None
        for _ in range(8):
            dist_a = np.sum(np.square(points_xy - center_a), axis=1)
            dist_b = np.sum(np.square(points_xy - center_b), axis=1)
            labels = dist_b < dist_a
            if not np.any(~labels) or not np.any(labels):
                return None
            center_a = np.median(points_xy[~labels], axis=0).astype(np.float32)
            center_b = np.median(points_xy[labels], axis=0).astype(np.float32)

        if labels is None:
            return None

        foot_a = np.median(points_xy[~labels], axis=0)
        foot_b = np.median(points_xy[labels], axis=0)
        if float(np.linalg.norm(foot_a - foot_b)) < 0.05:
            return None

        count_a = int(np.count_nonzero(~labels))
        count_b = int(np.count_nonzero(labels))
        foot_1 = (float(foot_a[0]), float(foot_a[1]))
        foot_2 = (float(foot_b[0]), float(foot_b[1]))
        if foot_2[1] < foot_1[1]:
            foot_1, foot_2 = foot_2, foot_1
            count_a, count_b = count_b, count_a
        return foot_1, foot_2, count_a, count_b

    def _transform_base_xy_to_map(
        self,
        x_base_m: float,
        y_base_m: float,
        stamp: TimeMsg | None = None,
    ) -> tuple[float, float] | None:
        lookup_time = (
            rclpy.time.Time.from_msg(stamp)
            if stamp is not None
            else rclpy.time.Time()
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                lookup_time,
            )
        except TransformException:
            return None

        rotation = rotation_matrix_from_quaternion(transform.transform.rotation)
        translation = transform.transform.translation
        x_map_m = float(
            rotation[0][0] * x_base_m
            + rotation[0][1] * y_base_m
            + translation.x
        )
        y_map_m = float(
            rotation[1][0] * x_base_m
            + rotation[1][1] * y_base_m
            + translation.y
        )
        if not math.isfinite(x_map_m) or not math.isfinite(y_map_m):
            return None
        return x_map_m, y_map_m

    def _compute_bbox_horizontal_bearing_in_base(
        self,
        bbox: tuple[float, float, float, float],
    ) -> tuple[float, float] | None:
        if self.latest_camera_info is None:
            return None

        camera_info = self.latest_camera_info
        fx = float(camera_info.k[0])
        cx = float(camera_info.k[2])
        if not math.isfinite(fx) or not math.isfinite(cx):
            return None
        if fx == 0.0:
            return None

        x1, _, x2, _ = bbox
        u = (x1 + x2) / 2.0
        ray_optical_x = (u - cx) / fx
        ray_optical_y = 0.0
        ray_optical_z = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        rotation = rotation_matrix_from_quaternion(transform.transform.rotation)
        direction_base_x = float(
            rotation[0][0] * ray_optical_x
            + rotation[0][1] * ray_optical_y
            + rotation[0][2] * ray_optical_z
        )
        direction_base_y = float(
            rotation[1][0] * ray_optical_x
            + rotation[1][1] * ray_optical_y
            + rotation[1][2] * ray_optical_z
        )
        norm = math.hypot(direction_base_x, direction_base_y)
        if norm <= 0.0 or not math.isfinite(norm):
            return None

        return direction_base_x / norm, direction_base_y / norm

    def _compute_ground_sample_from_bbox(
        self,
        bbox: tuple[float, float, float, float],
    ) -> tuple[float, float] | None:
        if self.latest_camera_info is None:
            return None

        camera_info = self.latest_camera_info
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
            return None
        if fx == 0.0 or fy == 0.0:
            return None

        x1, _, x2, y2 = bbox
        u = (x1 + x2) / 2.0
        v = y2
        ray_optical_x = (u - cx) / fx
        ray_optical_y = (v - cy) / fy
        ray_optical_z = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        rotation = rotation_matrix_from_quaternion(transform.transform.rotation)
        origin_base = transform.transform.translation
        direction_base_x = float(
            rotation[0][0] * ray_optical_x
            + rotation[0][1] * ray_optical_y
            + rotation[0][2] * ray_optical_z
        )
        direction_base_y = float(
            rotation[1][0] * ray_optical_x
            + rotation[1][1] * ray_optical_y
            + rotation[1][2] * ray_optical_z
        )
        direction_base_z = float(
            rotation[2][0] * ray_optical_x
            + rotation[2][1] * ray_optical_y
            + rotation[2][2] * ray_optical_z
        )

        if not all(
            math.isfinite(value)
            for value in (
                origin_base.x,
                origin_base.y,
                origin_base.z,
                direction_base_x,
                direction_base_y,
                direction_base_z,
            )
        ):
            return None
        if direction_base_z >= 0.0:
            return None

        t = -origin_base.z / direction_base_z
        if t <= 0.0 or not math.isfinite(t):
            return None

        ground_x_base_m = float(origin_base.x + t * direction_base_x)
        ground_y_base_m = float(origin_base.y + t * direction_base_y)
        if not math.isfinite(ground_x_base_m) or not math.isfinite(ground_y_base_m):
            return None

        return ground_x_base_m, ground_y_base_m

    def _try_transform_g1_to_map(self) -> None:
        if self.state != MissionState.TRANSFORM_G1_TO_MAP:
            return
        if (
            self.g1_sensor_foot_midpoint_x_map_m is not None
            and self.g1_sensor_foot_midpoint_y_map_m is not None
        ):
            self.g1_map_x_m = self.g1_sensor_foot_midpoint_x_map_m
            self.g1_map_y_m = self.g1_sensor_foot_midpoint_y_map_m
            self._finish_g1_map_transform()
            return
        if (
            self.g1_map_x_m is not None
            and self.g1_map_y_m is not None
            and self.g1_position_source is not None
        ):
            self._finish_g1_map_transform()
            return
        if self.g1_ground_x_base_m is None or self.g1_ground_y_base_m is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        rotation = rotation_matrix_from_quaternion(transform.transform.rotation)
        translation = transform.transform.translation
        ground_x_base_m = self.g1_ground_x_base_m
        ground_y_base_m = self.g1_ground_y_base_m

        self.g1_map_x_m = float(
            rotation[0][0] * ground_x_base_m
            + rotation[0][1] * ground_y_base_m
            + translation.x
        )
        self.g1_map_y_m = float(
            rotation[1][0] * ground_x_base_m
            + rotation[1][1] * ground_y_base_m
            + translation.y
        )
        if (
            self.g1_old_ground_estimate_x_base_m is not None
            and self.g1_old_ground_estimate_y_base_m is not None
            and (
                self.g1_old_ground_estimate_x_map_m is None
                or self.g1_old_ground_estimate_y_map_m is None
            )
        ):
            old_x_base_m = self.g1_old_ground_estimate_x_base_m
            old_y_base_m = self.g1_old_ground_estimate_y_base_m
            self.g1_old_ground_estimate_x_map_m = float(
                rotation[0][0] * old_x_base_m
                + rotation[0][1] * old_y_base_m
                + translation.x
            )
            self.g1_old_ground_estimate_y_map_m = float(
                rotation[1][0] * old_x_base_m
                + rotation[1][1] * old_y_base_m
                + translation.y
            )

        self.get_logger().info(
            'G1 foot-midpoint position in base_link: '
            f'x={ground_x_base_m:.3f} m, '
            f'y={ground_y_base_m:.3f} m'
        )
        self._finish_g1_map_transform()

    def _finish_g1_map_transform(self) -> None:
        if self.state != MissionState.TRANSFORM_G1_TO_MAP:
            return
        if self.g1_map_x_m is None or self.g1_map_y_m is None:
            return

        self.get_logger().info(
            f'G1 map position: x={self.g1_map_x_m:.3f} m, '
            f'y={self.g1_map_y_m:.3f} m, '
            f'source={self.g1_position_source or "ground_ray"}'
        )
        self._log_g1_sensor_foot_midpoint_diagnostics()
        self._log_g1_reference_diagnostics()

        if self.g1_map_transform_timer is not None:
            self.g1_map_transform_timer.cancel()
            self.g1_map_transform_timer = None

        previous_state = self.state
        self.state = MissionState.G1_LOCALIZED
        self._timing_stop('map_transform')
        self.get_logger().info(
            f'Mission state transition: {previous_state.name} -> {self.state.name}'
        )
        self._publish_reachability_donut()


def main() -> None:
    rclpy.init()
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
