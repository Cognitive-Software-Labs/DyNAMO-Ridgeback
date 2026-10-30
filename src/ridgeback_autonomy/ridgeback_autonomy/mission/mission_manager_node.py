#!/usr/bin/env python3
"""Mission coordinator: detection, handoff, localization, parking plan and drive.

After physical settlement and localization, the manager evaluates a bounded
set of global paths and stops in ``PARKING_PATH_FEASIBLE`` or a failure state.
With ``parking_navigation_enabled`` (off by default; simulation launch only) it
then completes the original mission: pre-parking pose, final parking pose,
stop and wait, return to the saved start pose. Navigation failures return the
robot to the start pose, as the original mission did. The guarded
``ParkingExecutor`` remains a separate, disabled hardware path.
"""

from __future__ import annotations

import ast
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass, replace
from enum import Enum
import json
import math
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import CollisionMonitorState, CostmapFilterInfo
from action_msgs.msg import GoalStatus, GoalStatusArray
from clearpath_platform_msgs.msg import StopStatus
from controller_manager_msgs.srv import ListControllers
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String, Bool
from rclpy.parameter import parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from rviz_2d_overlay_msgs.msg import OverlayText
from visualization_msgs.msg import MarkerArray

from ridgeback_interfaces.msg import TargetDetections, TargetMeasurements
from ridgeback_localization.contracts import (
    MASK_MEASUREMENTS_TOPIC,
    RAW_DETECTIONS_TOPIC,
)
from ridgeback_autonomy.mission.target_detection import (
    TargetDetectionAdapter,
    TargetDetectionCandidate,
)
from ridgeback_autonomy.mission.target_localization import (
    StabilizedTargetPosition,
    TargetAssociation,
    TargetLocalizationAdapter,
)
from ridgeback_autonomy.mission.parking_planner import (
    GridSnapshot,
    ParkingPlan,
    PlannerConfig,
    TargetReference,
    footprint_rejection_reason,
    keepout_radius_m,
    plan_parking,
    points_to_polygon_distance,
    validate_inputs,
)
from ridgeback_autonomy.mission.parking_geometry import preparking_pose, rotated_footprint_vertices
from ridgeback_autonomy.mission.parking_visualization import (
    clear_markers, legend_overlay, plan_markers,
)
from ridgeback_autonomy.mission.mission_state_diagram import state_diagram_overlay
from ridgeback_autonomy.mission.motion_settlement import MotionSettlement
from ridgeback_autonomy.mission.parking_clearance import (
    ClearanceLimits, ClearanceResult, clearance_reason, scan_points,
)
from ridgeback_autonomy.mission.parking_feasibility import (
    PlanningSnapshot, candidate_groups, driven_poses, path_rejection, path_length, pose_xyyaw,
    select_path_candidate,
)
from ridgeback_autonomy.mission.parking_execution import (
    ExecutionContext, ExecutionLimits, ExecutionState, ParkingExecutor,
)
from ridgeback_autonomy.mission.parking_safety import (
    ParkingSafetyMonitor, LOCK_TOPICS, MANUAL_TOPICS, COMMAND_TOPICS, keepout_mask,
    target_execution_reason,
)
from ridgeback_autonomy.mission.parking_stage_paths import ParkingStagePaths
from ridgeback_autonomy.mission.parking_nav2_profile import (
    GOAL_CHECKER_PARAMETERS, goal_checker_reason, parking_tree_identity,
)


class MissionState(Enum):
    WAITING_FOR_START_POSE = 'waiting_for_start_pose'
    SEARCH_G1 = 'search_g1'
    STOPPING_EXPLORATION = 'stopping_exploration'
    SETTLING_MOTION = 'settling_motion'
    MOTION_SETTLEMENT_FAILED = 'motion_settlement_failed'
    LOCALIZE_G1 = 'localize_g1'
    G1_LOCALIZED = 'g1_localized'
    EXPLORING_AROUND_G1 = 'exploring_around_g1'
    RETURNING_TO_OBSERVATION = 'returning_to_observation'
    PARKING_CANDIDATES_READY = 'parking_candidates_ready'
    EVALUATING_PARKING_PATHS = 'evaluating_parking_paths'
    PARKING_PATH_FEASIBLE = 'parking_path_feasible'
    PARKING_PLANNING_FAILED = 'parking_planning_failed'
    PARKING_EXECUTION_FAILED = 'parking_execution_failed'
    NAVIGATING_TO_PREPARKING = 'navigating_to_preparking'
    NAVIGATING_TO_PARKING = 'navigating_to_parking'
    PARKED = 'parked'
    RETURNING_TO_START = 'returning_to_start'
    MISSION_COMPLETE = 'mission_complete'
    MISSION_FAILED = 'mission_failed'
    MISSION_CANCELED = 'mission_canceled'


# The two parking legs, monitored live against both LiDARs.
_PARKING_LEGS = (MissionState.NAVIGATING_TO_PREPARKING, MissionState.NAVIGATING_TO_PARKING)

# States in which the manager owns a NavigateToPose goal or a parked stop.
_PARKING_DRIVE_STATES = (
    MissionState.NAVIGATING_TO_PREPARKING,
    MissionState.NAVIGATING_TO_PARKING,
    MissionState.PARKED,
    MissionState.RETURNING_TO_START,
)

# Only these prove that Nav2 has stopped executing a goal and released it.
_TERMINAL_GOAL_STATUSES = (
    GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED)


class FailureClass(Enum):
    RECOVERABLE = 'recoverable'
    TERMINAL = 'terminal'
    OPERATOR_CANCELED = 'operator_canceled'


@dataclass(frozen=True)
class MissionIssue:
    classification: FailureClass
    code: str
    reason: str


@dataclass(frozen=True)
class StateTransition:
    previous: MissionState
    current: MissionState
    reason: str
    monotonic_s: float


_ALLOWED_TRANSITIONS = {
    MissionState.WAITING_FOR_START_POSE: {
        MissionState.SEARCH_G1,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.SEARCH_G1: {
        MissionState.STOPPING_EXPLORATION,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.STOPPING_EXPLORATION: {
        MissionState.SETTLING_MOTION,
        MissionState.RETURNING_TO_OBSERVATION,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.SETTLING_MOTION: {
        MissionState.LOCALIZE_G1,
        MissionState.MOTION_SETTLEMENT_FAILED,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.MOTION_SETTLEMENT_FAILED: set(),
    MissionState.LOCALIZE_G1: {
        MissionState.G1_LOCALIZED,
        MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.EXPLORING_AROUND_G1: {
        MissionState.STOPPING_EXPLORATION, MissionState.MISSION_CANCELED},
    MissionState.RETURNING_TO_OBSERVATION: {
        MissionState.SETTLING_MOTION, MissionState.MISSION_FAILED, MissionState.MISSION_CANCELED},
    MissionState.G1_LOCALIZED: {
        MissionState.EXPLORING_AROUND_G1,
        MissionState.PARKING_CANDIDATES_READY,
        MissionState.PARKING_PLANNING_FAILED,
        MissionState.MISSION_CANCELED,
    },
    MissionState.PARKING_CANDIDATES_READY: {
        MissionState.EVALUATING_PARKING_PATHS, MissionState.PARKING_PLANNING_FAILED,
        MissionState.MISSION_CANCELED},
    MissionState.EVALUATING_PARKING_PATHS: {
        MissionState.PARKING_PATH_FEASIBLE, MissionState.PARKING_PLANNING_FAILED,
        MissionState.MISSION_CANCELED},
    MissionState.PARKING_PATH_FEASIBLE: {
        MissionState.PARKING_PLANNING_FAILED, MissionState.PARKING_EXECUTION_FAILED,
        MissionState.NAVIGATING_TO_PREPARKING, MissionState.MISSION_CANCELED},
    MissionState.PARKING_PLANNING_FAILED: set(),
    MissionState.PARKING_EXECUTION_FAILED: {MissionState.MISSION_CANCELED},
    MissionState.NAVIGATING_TO_PREPARKING: {
        MissionState.NAVIGATING_TO_PARKING, MissionState.RETURNING_TO_START,
        MissionState.MISSION_CANCELED},
    MissionState.NAVIGATING_TO_PARKING: {
        MissionState.PARKED, MissionState.RETURNING_TO_START, MissionState.MISSION_CANCELED},
    MissionState.PARKED: {MissionState.RETURNING_TO_START, MissionState.MISSION_CANCELED},
    MissionState.RETURNING_TO_START: {
        MissionState.MISSION_COMPLETE, MissionState.MISSION_FAILED,
        MissionState.MISSION_CANCELED},
    MissionState.MISSION_COMPLETE: set(),
    MissionState.MISSION_FAILED: set(),
    MissionState.MISSION_CANCELED: set(),
}

_ALLOWED_STATE_NAMES = {state.name: frozenset(target.name for target in targets)
                        for state, targets in _ALLOWED_TRANSITIONS.items()}


class MissionManagerNode(Node):
    """Coordinate the safe exploration-to-localization mission prefix."""

    def __init__(
        self,
        *,
        tf_buffer=None,
        pause_client=None,
        compute_path_client=None,
        navigation_client=None,
        goal_checker_client=None,
        resume_client=None,
        monotonic=time.monotonic,
        parameter_overrides=None,
    ) -> None:
        super().__init__(
            'mission_manager_node',
            parameter_overrides=parameter_overrides,
        )
        self._lock = threading.RLock()
        self._monotonic = monotonic

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('target_detection_topic', RAW_DETECTIONS_TOPIC)
        self.declare_parameter('target_measurement_topic', MASK_MEASUREMENTS_TOPIC)
        self.declare_parameter('exploration_pause_service', 'mission/pause')
        self.declare_parameter('exploration_status_topic', 'mission/status')
        self.declare_parameter('state_topic', 'parking_mission/state')
        self.declare_parameter('handoff_timeout_s', 10.0)
        self.declare_parameter('target_localization_timeout_s', 5.0)
        self.declare_parameter('association_history_size', 32)
        self.declare_parameter('control_period_s', 0.2)
        self.declare_parameter('startup_paused', False)
        self._startup_paused = bool(self.get_parameter('startup_paused').value)
        self.declare_parameter('parking_planning_enabled', True)
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('parking_visualization_topic', 'parking/visualization')
        self.declare_parameter('parking_legend_topic', 'parking/legend')
        self.declare_parameter('state_diagram_topic', 'parking_mission/state_diagram')
        self.declare_parameter(
            'nav2_params_path',
            os.path.join(get_package_share_directory('ridgeback_autonomy'),
                         'config', 'nav2_params.yaml'))
        # Timestamp age and explicit map-to-odom drift are checked separately.
        self.declare_parameter('parking_max_target_age_s', 5.0)
        self.declare_parameter('parking_max_map_age_s', 5.0)
        self.declare_parameter('parking_max_pose_age_s', 1.0)
        self.declare_parameter('parking_input_timeout_s', 10.0)
        # G1-centred mapping with the shared explorer before parking planning.
        self.declare_parameter('g1_exploration_enabled', True)
        # Exploration ends when every donut cell (the planner's task-point cells) is
        # known on the SLAM map; the wider footprint area out to this radius is reported.
        self.declare_parameter('g1_coverage_radius_m', 1.4)
        self.declare_parameter('g1_exploration_timeout_s', 240.0)
        self.declare_parameter('g1_exploration_protected_margin_m', 0.15)
        self.declare_parameter('exploration_resume_service', 'mission/resume')
        self.declare_parameter('explorer_status_topic', 'explore/status')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('odometry_topic', 'platform/odom/filtered')
        self.declare_parameter('motion_linear_limit_mps', 0.005)
        self.declare_parameter('motion_angular_limit_radps', 0.005)
        self.declare_parameter('motion_stable_s', 2.0)
        self.declare_parameter('motion_max_odom_age_s', 0.5)
        self.declare_parameter('motion_settlement_timeout_s', 10.0)
        self.declare_parameter('parking_path_timeout_s', 3.0)
        self.declare_parameter('parking_path_batch_timeout_s', 10.0)
        self.declare_parameter('parking_path_candidates_per_class', 6)
        self.declare_parameter('parking_max_frame_shift_m', 0.05)
        self.declare_parameter('parking_max_frame_yaw_rad', 0.03)
        self.declare_parameter('parking_execution_enabled', False)
        self.declare_parameter('parking_xy_tolerance_m', 0.08)
        self.declare_parameter('parking_yaw_tolerance_rad', 0.12)
        self.declare_parameter('preparking_offset_m', 0.50)
        self.declare_parameter('parking_navigation_timeout_s', 90.0)
        self.declare_parameter('parking_progress_timeout_s', 20.0)
        # Original mission tail: pre-parking, final pose, wait, return to start.
        self.declare_parameter('parking_navigation_enabled', False)
        self.declare_parameter('parking_wait_s', 5.0)
        self.declare_parameter('return_navigation_timeout_s', 300.0)
        self.declare_parameter('goal_checker_plugin', 'general_goal_checker')
        self.declare_parameter('parking_final_xy_tolerance_m', 0.03)
        self.declare_parameter('parking_target_reach_allowance_m', 0.03)
        # Live LiDAR clearance during the two parking legs (the collision monitor's sources).
        self.declare_parameter('parking_lidar_topics',
                               ['sensors/lidar2d_0/scan', 'sensors/lidar2d_1/scan'])
        self.declare_parameter('parking_lidar_margin_m', 0.05)
        self.declare_parameter('parking_lidar_max_age_s', 0.5)

        self.map_frame = str(self.get_parameter('map_frame').value).strip()
        self.base_frame = str(self.get_parameter('robot_base_frame').value).strip()
        self.detection_topic = str(
            self.get_parameter('target_detection_topic').value
        ).strip()
        self.measurement_topic = str(
            self.get_parameter('target_measurement_topic').value
        ).strip()
        self.pause_service_name = str(
            self.get_parameter('exploration_pause_service').value
        ).strip()
        self.guard_status_topic = str(
            self.get_parameter('exploration_status_topic').value
        ).strip()
        self.state_topic = str(self.get_parameter('state_topic').value).strip()
        self.handoff_timeout_s = float(
            self.get_parameter('handoff_timeout_s').value
        )
        self.localization_timeout_s = float(
            self.get_parameter('target_localization_timeout_s').value
        )
        self.association_history_size = int(
            self.get_parameter('association_history_size').value
        )
        control_period_s = float(self.get_parameter('control_period_s').value)
        if not self.map_frame or not self.base_frame:
            raise ValueError('map_frame and robot_base_frame must be explicit')
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (
                self.handoff_timeout_s,
                self.localization_timeout_s,
                control_period_s,
            )
        ):
            raise ValueError('mission timeouts and control period must be positive')
        if self.association_history_size < 1:
            raise ValueError('association_history_size must be positive')
        self.parking_planning_enabled = bool(
            self.get_parameter('parking_planning_enabled').value)
        self.parking_max_target_age_s = float(
            self.get_parameter('parking_max_target_age_s').value)
        self.parking_max_map_age_s = float(self.get_parameter('parking_max_map_age_s').value)
        self.parking_max_pose_age_s = float(self.get_parameter('parking_max_pose_age_s').value)
        self.parking_input_timeout_s = float(
            self.get_parameter('parking_input_timeout_s').value)
        if any(not math.isfinite(value) or value <= 0.0 for value in (
            self.parking_max_target_age_s, self.parking_max_map_age_s,
            self.parking_max_pose_age_s, self.parking_input_timeout_s,
        )):
            raise ValueError('parking freshness limits and input timeout must be finite and positive')
        # The final leg stops as soon as it is inside this tolerance, approaching
        # from outside, so the error lengthens the reach; the reference allowance
        # covers the measured 2-4 cm offset of the camera-facing G1 estimate from
        # the true foot midpoint. Both are budgeted out of the 0.60 m reach limit.
        self.final_xy_tolerance_m = float(self.get_parameter('parking_final_xy_tolerance_m').value)
        reference_allowance_m = float(
            self.get_parameter('parking_target_reach_allowance_m').value)
        if not (0.0 < self.final_xy_tolerance_m <= 0.08 and 0.0 <= reference_allowance_m < 0.1):
            raise ValueError('parking final tolerance / reach allowance out of range')
        self.planner_config = PlannerConfig(
            footprint=tuple(load_nav2_footprint(str(self.get_parameter('nav2_params_path').value))),
            reach_allowance_m=self.final_xy_tolerance_m + reference_allowance_m)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.motion = MotionSettlement(
            *[float(self.get_parameter(n).value) for n in (
                'motion_linear_limit_mps', 'motion_angular_limit_radps',
                'motion_stable_s', 'motion_max_odom_age_s')])
        for name in ('motion_settlement_timeout_s', 'parking_path_timeout_s',
                     'parking_path_batch_timeout_s', 'parking_max_frame_shift_m',
                     'parking_max_frame_yaw_rad'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)
        if any(not math.isfinite(v) or v <= 0 for v in (
            self.motion.linear_limit, self.motion.angular_limit, self.motion.stable_s,
            self.motion.max_age_s)):
            raise ValueError('motion thresholds must be finite and positive')
        self.path_candidates_per_class = int(self.get_parameter('parking_path_candidates_per_class').value)
        if not 1 <= self.path_candidates_per_class <= 12:
            raise ValueError('parking_path_candidates_per_class must be within [1, 12]')
        with open(str(self.get_parameter('nav2_params_path').value), encoding='utf-8') as stream:
            nav2_params = yaml.safe_load(stream)
        self.planner_id = nav2_params['planner_server']['ros__parameters']['planner_plugins'][0]
        self.parking_navigation_enabled = bool(
            self.get_parameter('parking_navigation_enabled').value)
        self.goal_checker_plugin = str(self.get_parameter('goal_checker_plugin').value)
        shared_checker = nav2_params['controller_server']['ros__parameters'][self.goal_checker_plugin]
        # Restored after the parking legs so exploration/return keep the shared tuning.
        self._shared_goal_tolerance = (float(shared_checker['xy_goal_tolerance']),
                                       float(shared_checker['yaw_goal_tolerance']))
        for name in ('parking_wait_s', 'return_navigation_timeout_s',
                     'parking_navigation_timeout_s'):
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
            setattr(self, name, value)

        self.tf_buffer = tf_buffer if tf_buffer is not None else Buffer(node=self)
        self.tf_listener = (
            None
            if tf_buffer is not None
            # Own node and thread: TF stays current while this node's callbacks
            # run synchronous path checks (process-wide /tf remaps still apply).
            else TransformListener(self.tf_buffer, None, spin_thread=True)
        )
        self.detection_adapter = TargetDetectionAdapter()
        self.localization_adapter = TargetLocalizationAdapter(
            self.tf_buffer,
            map_frame=self.map_frame,
            base_frame=self.base_frame,
        )

        status_qos = QoSProfile(depth=1)
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_publisher = self.create_publisher(
            String,
            self.state_topic,
            status_qos,
        )
        # Read-only RViz view of the state machine, redrawn when the state changes.
        self.state_diagram_publisher = self.create_publisher(
            OverlayText, str(self.get_parameter('state_diagram_topic').value), status_qos)
        self._state_diagram = self._state_diagram_key = None
        self.detection_subscription = self.create_subscription(
            TargetDetections,
            self.detection_topic,
            self._on_detections,
            10,
        )
        self.measurement_subscription = self.create_subscription(
            TargetMeasurements,
            self.measurement_topic,
            self._on_measurements,
            10,
        )
        self.guard_status_subscription = self.create_subscription(
            String,
            self.guard_status_topic,
            self._on_guard_status,
            status_qos,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            map_qos,
        )
        self.parking_marker_publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('parking_visualization_topic').value),
            status_qos,
        )
        # The colour legend never changes and lives outside the marker clears. It is
        # latched and resent each second: the overlay places a right/bottom-aligned panel
        # from its size, which is unknown when the first message arrives.
        self.parking_legend_publisher = self.create_publisher(
            OverlayText, str(self.get_parameter('parking_legend_topic').value), status_qos)
        self._legend = legend_overlay(self.planner_config)
        self._legend_next_s = 0.0
        self.parking_legend_publisher.publish(self._legend)
        self.pause_client = (
            pause_client
            if pause_client is not None
            else self.create_client(Trigger, self.pause_service_name)
        )
        self.resume_client = (
            resume_client if resume_client is not None else self.create_client(
                Trigger, str(self.get_parameter('exploration_resume_service').value)))
        self.focus_publisher = self.create_publisher(PointStamped, 'explore/focus', status_qos)
        self.g1_exploration_enabled = bool(self.get_parameter('g1_exploration_enabled').value)
        self.g1_coverage_radius_m = float(self.get_parameter('g1_coverage_radius_m').value)
        self.g1_exploration_timeout_s = float(self.get_parameter('g1_exploration_timeout_s').value)
        self.g1_protected_margin_m = float(
            self.get_parameter('g1_exploration_protected_margin_m').value)
        self._g1_explored = False
        self._observation_pose = None
        self._g1_exploration_deadline_s = None
        self._g1_explorer_resumed = False
        self._g1_frontiers_exhausted = False
        self._g1_exploration_started_s = None
        self.g1_exploration_result = None
        self.exploration_status_subscription = self.create_subscription(
            String, str(self.get_parameter('explorer_status_topic').value),
            self._on_exploration_status, 10)
        self.compute_path_client = (compute_path_client if compute_path_client is not None else
                                    ActionClient(self, ComputePathToPose, 'compute_path_to_pose'))
        self.odom_subscription = self.create_subscription(
            Odometry, str(self.get_parameter('odometry_topic').value), self._on_odometry, 10)

        self.state = MissionState.WAITING_FOR_START_POSE
        self.start_pose: PoseStamped | None = None
        self.confirmed_detection: TargetDetectionCandidate | None = None
        self.target_committed = False
        self.target_position: StabilizedTargetPosition | None = None
        self.latest_map: OccupancyGrid | None = None
        self.approach_pose: tuple[float, float, float] | None = None
        self.parking_plan: ParkingPlan | None = None
        self._planning_deadline_s: float | None = None
        self._motion_deadline_s = None
        self._path_generation = 0
        self._path_busy = False
        self._path_handle = None
        self._path_cancel_requested = False
        self._path_request_deadline = None
        self._path_batch_deadline = None
        self._path_groups = []
        self._path_valid = []
        self._path_queue = []
        self._paths = {}
        self._planning_snapshot = None
        self._target_map_odom = None
        self.last_issue: MissionIssue | None = None
        self.transition_history: list[StateTransition] = []
        self._associations: OrderedDict[int, TargetAssociation] = OrderedDict()

        self._handoff_generation = 0
        self._handoff_deadline_s: float | None = None
        self._pause_request_in_flight = False
        self._pause_response_received = False
        self._pause_service_seen_ready = False
        self._pause_request_failed = False
        self._pause_status_min_sequence = 0
        self._guard_status_sequence = 0
        self._latest_guard_status: dict | None = None
        self._guard_failure_signature: tuple[str, bool] | None = None
        self._guard_failure_count = 0
        self._localization_deadline_s: float | None = None

        # No parameter or topic substitutes for verified platform/ownership
        # interlocks. Their production adapter is deliberately deferred.
        self.execution_safety = None
        self.safety_monitor = ParkingSafetyMonitor()
        self._execution_odometry = None
        self._execution_odom_received_s = None
        self.navigation_client = (navigation_client if navigation_client is not None else
            ActionClient(self, NavigateToPose, 'navigate_to_pose'))
        self.execution = ParkingExecutor(
            self.navigation_client, self._execution_context, self.planner_config,
            os.path.join(get_package_share_directory('ridgeback_autonomy'),
                         'config', 'parking_navigate_to_pose.xml'),
            enabled=bool(self.get_parameter('parking_execution_enabled').value),
            limits=ExecutionLimits(
                xy_tolerance_m=float(self.get_parameter('parking_xy_tolerance_m').value),
                yaw_tolerance_rad=float(self.get_parameter('parking_yaw_tolerance_rad').value),
                preparking_offset_m=float(self.get_parameter('preparking_offset_m').value),
                navigation_timeout_s=float(self.get_parameter('parking_navigation_timeout_s').value),
                progress_timeout_s=float(self.get_parameter('parking_progress_timeout_s').value),
                settlement_timeout_s=self.motion_settlement_timeout_s,
                max_target_age_s=self.parking_max_target_age_s,
                max_map_age_s=self.parking_max_map_age_s,
                max_pose_age_s=self.parking_max_pose_age_s,
                frame_shift_m=self.parking_max_frame_shift_m,
                frame_yaw_rad=self.parking_max_frame_yaw_rad),
            on_change=self._execution_changed, safety_check=self._execution_safety_check)
        self._parking_bt_identity, _ = parking_tree_identity(self.execution.behavior_tree, self.planner_id)
        self.stage_paths = ParkingStagePaths(self.compute_path_client, self.execution,
            lambda: replace(self._execution_context(), path_request_unresolved=self._path_busy),
            self.planner_id, timeout_s=self.parking_path_timeout_s)
        self._stage_last_published = self.stage_paths.reason
        self.stage_service = self.create_service(Trigger, 'parking_mission/prepare_stage', self._prepare_stage)
        self.mask_publisher = self.create_publisher(OccupancyGrid, 'parking/g1_keepout_mask', status_qos)
        self.mask_info_publisher = self.create_publisher(CostmapFilterInfo, 'parking/g1_keepout_info', status_qos)
        self._mask_key = None
        self._safety_subscriptions = []
        for topic in LOCK_TOPICS:
            self._safety_subscriptions.append(self.create_subscription(Bool, topic,
                lambda msg, t=topic: self._on_stop_lock(t, msg), qos_profile_sensor_data))
        for topic in MANUAL_TOPICS + COMMAND_TOPICS:
            self._safety_subscriptions.append(self.create_subscription(TwistStamped, topic,
                lambda msg, t=topic: self._on_command_evidence(t, msg), 10))
        self._safety_subscriptions.append(self.create_subscription(StopStatus,
            'platform/mcu/status/stop', self._on_platform_stop, qos_profile_sensor_data))
        for topic in ('navigate_to_pose/_action/status', 'navigate_through_poses/_action/status'):
            self._safety_subscriptions.append(self.create_subscription(GoalStatusArray, topic,
                lambda msg, t=topic: self._on_navigation_status(t, msg), status_qos))
        self._platform_client = self.create_client(ListControllers, 'controller_manager/list_controllers')
        self._checker_client = self.create_client(GetParameters, 'controller_server/get_parameters')
        self._checker_lifecycle_client = self.create_client(GetState, 'controller_server/get_state')
        self._probe_inflight, self._probe_values = {}, {}
        self._last_probe_s = -math.inf
        self.collision_subscription = self.create_subscription(
            CollisionMonitorState, 'collision_monitor_state', self._on_collision_state, 10)
        self._goal_checker_set_client = (
            goal_checker_client if goal_checker_client is not None
            else self.create_client(SetParameters, 'controller_server/set_parameters'))
        self._nav_generation = 0
        # A sent goal stays owned until its terminal result; a cancel request never releases it.
        self._nav_goal_generation = None
        self._nav_handle = None
        self._nav_canceling = False
        self._nav_cancel_deadline_s = None
        self._nav_deadline_s = None
        self._nav_pending_tolerance = None
        self._parked_settled_ns = None
        self._parking_failure = None
        self._arrival_errors = {}
        self._return_after_stop = False
        self.clearance_limits = ClearanceLimits(
            margin_m=float(self.get_parameter('parking_lidar_margin_m').value),
            scan_max_age_s=float(self.get_parameter('parking_lidar_max_age_s').value))
        self._scans = {}
        self._laser_in_base = {}
        self._clearance = None
        self._clearance_legs = {}
        self.parking_lidar_topics = [str(t) for t in self.get_parameter('parking_lidar_topics').value]
        for topic in self.parking_lidar_topics:
            self.create_subscription(LaserScan, topic, lambda msg, topic=topic: self._on_scan(topic, msg),
                                     qos_profile_sensor_data)
        self._motion_fault_mark = 0
        self._motion_restart_s = None

        # Safety deadlines must still be polled if /clock pauses or stops.
        self.control_timer = self.create_timer(
            control_period_s, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.start_service = self.create_service(Trigger, 'parking_mission/start', self._start_mission)
        self._publish_state('mission manager initialized', event='snapshot')

    def _start_mission(self, request, response):
        """Release demo startup admission after the launch has verified all prerequisites.

        Start-pose capture is allowed while paused; detections are not. This
        prevents a startup resume from racing the manager's G1 pause request.
        Normal launches retain immediate admission (startup_paused defaults false).
        """
        with self._lock:
            response.success = self.start_pose is not None
            response.message = ('mission startup released' if response.success
                                else 'waiting for the initial map-to-base pose')
            if response.success:
                self._startup_paused = False
                self._publish_state(response.message, event='startup_release')
            return response

    def tick(self, now_s: float | None = None) -> None:
        """Advance timeout/service polling without blocking ROS callbacks."""

        with self._lock:
            now_s = self._monotonic() if now_s is None else float(now_s)
            if now_s >= self._legend_next_s:
                self._legend_next_s = now_s + 1.0
                self.parking_legend_publisher.publish(self._legend)
                if self._state_diagram is not None:
                    self.state_diagram_publisher.publish(self._state_diagram)
            self._poll_safety_evidence(now_s)
            self._publish_keepout_mask()
            stage_was_busy = self.stage_paths.busy
            if stage_was_busy:
                self.stage_paths.poll()
            if self.stage_paths.reason != self._stage_last_published:
                self._stage_last_published = self.stage_paths.reason
                self._publish_state(self.stage_paths.reason, event='stage_path_preparation')
            if stage_was_busy:
                return
            if self.execution.state not in (ExecutionState.IDLE, ExecutionState.FAILED):
                self.execution.poll()
                # Robot movement is expected during navigation, not frame drift.
                return
            if self._nav_cancel_deadline_s is not None:
                # Nothing drives or times out while Nav2 may still own a cancelled goal.
                if now_s >= self._nav_cancel_deadline_s:
                    self._navigation_ownership_unresolved(
                        'NavigateToPose cancellation produced no terminal result within '
                        f'{self.execution.limits.cancellation_timeout_s:.1f} s')
                return
            if self.state in _PARKING_DRIVE_STATES:
                self._poll_parking_navigation(now_s)
                return
            if self.state == MissionState.EXPLORING_AROUND_G1:
                self._poll_g1_exploration(now_s)
                return
            if self.state == MissionState.RETURNING_TO_OBSERVATION:
                if self._nav_deadline_s is not None and now_s >= self._nav_deadline_s:
                    self._cancel_navigation_goal()
                    self._navigation_finished(False, 'observation pose deadline expired')
                return
            if self.state == MissionState.SETTLING_MOTION:
                now_ns = self.get_clock().now().nanoseconds
                if self.motion.ready(now_ns, now_s):
                    self.get_logger().info('Physical motion settled; starting fresh target localization')
                    self._start_localization('fresh odometry confirms physical motion settled')
                elif now_s >= self._motion_deadline_s:
                    self.last_issue = MissionIssue(FailureClass.TERMINAL,
                        'motion_settlement_timeout', self.motion.reason)
                    self._transition(MissionState.MOTION_SETTLEMENT_FAILED, self.motion.reason)
                return
            if self.state == MissionState.PARKING_CANDIDATES_READY:
                if self.motion.ready(self.get_clock().now().nanoseconds, now_s):
                    self._motion_deadline_s = None
                    self._path_batch_deadline = now_s + self.parking_path_batch_timeout_s
                    self._mark_motion_evidence()
                    self._transition(MissionState.EVALUATING_PARKING_PATHS,
                                     'fresh post-planning odometry confirms motion settled')
                    self._next_path()
                elif now_s >= self._motion_deadline_s:
                    self._fail_planning('motion_settlement_timeout',
                        f'post-planning motion evidence unavailable: {self.motion.reason}')
                return
            if self.state in (MissionState.EVALUATING_PARKING_PATHS, MissionState.PARKING_PATH_FEASIBLE):
                reason = self._plan_invalid_reason()
                if reason:
                    self._fail_planning('parking_plan_invalidated', reason)
                elif self.state == MissionState.PARKING_PATH_FEASIBLE:
                    if self.parking_navigation_enabled:
                        self._begin_parking_when_stationary()
                elif self.state == MissionState.EVALUATING_PARKING_PATHS:
                    if (now_s >= self._path_batch_deadline or
                            self._path_busy and now_s >= self._path_request_deadline):
                        self._fail_planning('planner_timeout', 'ComputePathToPose deadline expired')
                    elif not self._path_busy:
                        self._next_path()
                return
            if self.state == MissionState.WAITING_FOR_START_POSE:
                self._try_capture_start_pose()
                return
            if self.state == MissionState.STOPPING_EXPLORATION:
                if self._handoff_deadline_s is not None and now_s >= self._handoff_deadline_s:
                    if not self._pause_service_seen_ready:
                        code = 'pause_service_unavailable'
                        reason = 'MissionGuard pause service was unavailable'
                    elif self._pause_request_failed and not self._pause_response_received:
                        code = 'pause_request_failed'
                        reason = 'MissionGuard pause requests failed'
                    else:
                        code = 'exploration_handoff_timeout'
                        reason = 'Exploration ownership remained unresolved'
                    self._fail_terminal(
                        code,
                        f'{reason} before the handoff timeout',
                    )
                    return
                self._request_exploration_pause()
                return
            if (
                self.state == MissionState.LOCALIZE_G1
                and self._localization_deadline_s is not None
                and now_s >= self._localization_deadline_s
            ):
                self._fail_terminal(
                    'target_localization_timeout',
                    'Target localization did not stabilize before the timeout',
                )
                return
            if self.state == MissionState.G1_LOCALIZED:
                self.try_plan_parking(now_s)

    def process_detection(self, msg: TargetDetections) -> None:
        with self._lock:
            if self._startup_paused:
                return
            if self.state not in (
                MissionState.SEARCH_G1,
                MissionState.STOPPING_EXPLORATION,
                MissionState.SETTLING_MOTION,
                MissionState.LOCALIZE_G1,
            ):
                self._guard_failed(
                    'target detection',
                    f'detections are ignored in {self.state.name}',
                )
                return

            result = self.detection_adapter.observe(msg)
            if not result.message_valid:
                self._record_recoverable('invalid_detection', result.reason)
                return
            if result.selected is not None:
                self._remember_association(result.stamp_ns, result.selected)

            if self.state != MissionState.SEARCH_G1:
                return
            if not result.confirmed:
                return
            if result.selected is None:
                self._fail_terminal(
                    'invalid_detection_confirmation',
                    'Detection adapter confirmed without a selected target',
                )
                return

            self.confirmed_detection = result.selected
            self.target_committed = True
            # The original ranked parking sides against the robot pose at commit.
            self.approach_pose = self._current_robot_pose()
            self.get_logger().info(
                'Target confirmed by detection adapter: '
                f'positives={result.positive_votes}/{len(result.vote_window)}, '
                f'label={result.selected.label}, score={result.selected.score:.3f}'
            )
            if self._transition(
                MissionState.STOPPING_EXPLORATION,
                'target confirmed; requesting explorer ownership release',
            ):
                self._begin_exploration_handoff()

    def process_guard_status(self, msg: String) -> None:
        with self._lock:
            self._guard_status_sequence += 1
            sequence = self._guard_status_sequence
            try:
                payload = json.loads(msg.data)
            except (TypeError, ValueError, json.JSONDecodeError):
                self._record_recoverable(
                    'invalid_guard_status',
                    'mission/status is not valid JSON',
                )
                return
            if not self._valid_guard_status(payload):
                self._record_recoverable(
                    'invalid_guard_status',
                    'mission/status is missing required ownership fields',
                )
                return
            self._latest_guard_status = payload
            if self.state != MissionState.STOPPING_EXPLORATION:
                return
            if not self._pause_response_received:
                return
            if sequence <= self._pause_status_min_sequence:
                return
            self._evaluate_guard_status(payload)

    def process_measurement(
        self,
        msg: TargetMeasurements,
        *,
        now_ns: int | None = None,
    ) -> None:
        with self._lock:
            if self.state != MissionState.LOCALIZE_G1:
                self._guard_failed(
                    'target measurement',
                    f'measurements are ignored in {self.state.name}',
                )
                return
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(
                msg.header.stamp.nanosec
            )
            association = self._associations.get(stamp_ns)
            if association is None:
                self._record_recoverable(
                    'measurement_without_detection_association',
                    'TargetMeasurements has no matching validated detection frame',
                )
                return

            result = self.localization_adapter.observe(
                msg,
                association=association,
                now_ns=(
                    self.get_clock().now().nanoseconds
                    if now_ns is None
                    else int(now_ns)
                ),
            )
            if not result.accepted:
                details = result.reason
                if result.source_failures:
                    details += ': ' + ', '.join(
                        f'{source}={reason}'
                        for source, reason in result.source_failures
                    )
                self._record_recoverable('localization_sample_rejected', details)
                return

            if result.stabilized is None:
                sample_count = self.localization_adapter.sample_count_for(
                    result.observation.source
                )
                self.get_logger().info(
                    'Target localization sample accepted: '
                    f'{sample_count}/{self.localization_adapter.sample_count}, '
                    f'source={result.observation.source}'
                )
                return

            self.target_position = result.stabilized
            self._target_map_odom = self._map_odom(Time.from_msg(msg.header.stamp))
            self._localization_deadline_s = None
            self._clear_recoverable_issue()
            self.get_logger().info(
                'Target position stabilized: '
                f'x={result.stabilized.map_x_m:.3f}, '
                f'y={result.stabilized.map_y_m:.3f}, '
                f'source={result.stabilized.source}, '
                f'reference={result.stabilized.reference}, '
                f'is_foot_midpoint={result.stabilized.is_foot_midpoint}'
            )
            if self._transition(
                MissionState.G1_LOCALIZED,
                'generic target position stabilized',
            ):
                self.get_logger().warn(
                    'Generic target position is not a validated G1 foot midpoint; '
                    'planning uses the larger keep-out margin'
                )
                self._planning_deadline_s = self._monotonic() + self.parking_input_timeout_s
                if not self._maybe_explore_around_g1():
                    self.try_plan_parking(self._monotonic(), now_ns=now_ns)

    def reset_mission(self, *, clear_start_pose: bool = True) -> None:
        """Reset manager-local state; MissionGuard resume remains explicit."""

        with self._lock:
            if self.stage_paths.busy:
                self.stage_paths.cancel('mission_reset_requested')
                self._guard_failed('mission reset', 'stage path termination pending')
                return
            if self.execution.busy or self.execution.state == ExecutionState.SETTLING:
                self.execution.cancel('operator_reset_requested')
                self._guard_failed('mission reset', 'navigation termination/physical settlement pending')
                return
            if self._nav_goal_generation is not None:
                # Nav2 may still execute the goal; a retry succeeds after its terminal result.
                self.cancel_mission('reset requested while Nav2 owns a goal')
                self._guard_failed('mission reset', 'navigation goal termination pending')
                return
            if not self.execution.reset():
                self._guard_failed('mission reset', 'fresh physical stop evidence required')
                return
            if self.state in _PARKING_DRIVE_STATES + (MissionState.RETURNING_TO_OBSERVATION,):
                self._stop_parking_navigation()
            if self.state == MissionState.EXPLORING_AROUND_G1:
                self._stop_g1_exploration()
            self._g1_explored = False
            self._observation_pose = None
            self._g1_exploration_deadline_s = None
            self._g1_explorer_resumed = self._g1_frontiers_exhausted = False
            self._g1_exploration_started_s = None
            self.g1_exploration_result = None
            self._parking_failure = None
            self._arrival_errors = {}
            self._return_after_stop = False
            self._clearance = None
            self._clearance_legs = {}
            self.stage_paths.reset()
            self.execution_safety = None
            self._execution_odometry = self._execution_odom_received_s = None
            previous = self.state
            self._handoff_generation += 1
            self._invalidate_path_requests()
            self.motion.reset()
            self._motion_deadline_s = None
            self._planning_snapshot = self._target_map_odom = None
            self.detection_adapter.reset()
            self.localization_adapter.reset()
            self.confirmed_detection = None
            self.target_committed = False
            self.target_position = None
            self.approach_pose = None
            self.parking_plan = None
            self._planning_deadline_s = None
            self.parking_marker_publisher.publish(clear_markers())
            self.last_issue = None
            self._associations.clear()
            self._clear_handoff_state()
            self._localization_deadline_s = None
            if clear_start_pose:
                self.start_pose = None
            self.state = (
                MissionState.WAITING_FOR_START_POSE
                if self.start_pose is None
                else MissionState.SEARCH_G1
            )
            self.transition_history.append(StateTransition(
                previous=previous,
                current=self.state,
                reason='explicit manager reset',
                monotonic_s=self._monotonic(),
            ))
            self.get_logger().info(
                f'Mission state transition: {previous.name} -> {self.state.name}; '
                'reason=explicit manager reset; explorer resume is still explicit'
            )
            self._publish_state('explicit manager reset')

    def cancel_mission(self, reason: str = 'explicit operator cancellation') -> None:
        with self._lock:
            self.stage_paths.cancel(reason)
            if self.state in (
                MissionState.MISSION_FAILED,
                MissionState.MISSION_CANCELED,
                MissionState.MISSION_COMPLETE,
            ):
                return
            if self.state in _PARKING_DRIVE_STATES + (MissionState.RETURNING_TO_OBSERVATION,):
                self._stop_parking_navigation()
            if self.state == MissionState.EXPLORING_AROUND_G1:
                self._stop_g1_exploration()
            self.last_issue = MissionIssue(
                FailureClass.OPERATOR_CANCELED,
                'operator_canceled',
                reason,
            )
            self._invalidate_path_requests()
            self.parking_marker_publisher.publish(clear_markers())
            self._transition(MissionState.MISSION_CANCELED, reason)
            self.execution.cancel(reason)

    def _try_capture_start_pose(self) -> bool:
        if self.start_pose is not None:
            return True
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            self._guard_failed(
                'start pose capture',
                f'{self.map_frame} -> {self.base_frame} transform unavailable',
            )
            return False

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x,
            translation.y,
            translation.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
        )
        norm = math.sqrt(
            float(rotation.x) ** 2
            + float(rotation.y) ** 2
            + float(rotation.z) ** 2
            + float(rotation.w) ** 2
        )
        if not all(math.isfinite(float(value)) for value in values) or norm < 1e-9:
            self._record_recoverable(
                'invalid_start_pose_transform',
                'map-to-base transform contains invalid values',
            )
            return False

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = float(translation.x)
        pose.pose.position.y = float(translation.y)
        pose.pose.position.z = float(translation.z)
        pose.pose.orientation.x = float(rotation.x) / norm
        pose.pose.orientation.y = float(rotation.y) / norm
        pose.pose.orientation.z = float(rotation.z) / norm
        pose.pose.orientation.w = float(rotation.w) / norm
        self.start_pose = pose
        self._clear_recoverable_issue()
        self.get_logger().info(
            'Saved mission start pose: '
            f'frame={self.map_frame}, x={pose.pose.position.x:.3f}, '
            f'y={pose.pose.position.y:.3f}'
        )
        self.detection_adapter.reset()
        return self._transition(
            MissionState.SEARCH_G1,
            'valid initial map-to-base transform captured',
        )

    def _begin_exploration_handoff(self) -> None:
        self._handoff_generation += 1
        self._handoff_deadline_s = self._monotonic() + self.handoff_timeout_s
        self._pause_request_in_flight = False
        self._pause_response_received = False
        self._pause_service_seen_ready = False
        self._pause_request_failed = False
        self._pause_status_min_sequence = self._guard_status_sequence
        self._request_exploration_pause()

    def _request_exploration_pause(self) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            return
        if self._pause_request_in_flight or self._pause_response_received:
            return
        if not self.pause_client.service_is_ready():
            return

        self._pause_service_seen_ready = True
        generation = self._handoff_generation
        self._pause_request_in_flight = True
        self.get_logger().info('Requesting exploration pause from MissionGuard')
        try:
            future = self.pause_client.call_async(Trigger.Request())
        except Exception as error:
            self._pause_request_in_flight = False
            self._pause_request_failed = True
            self._record_recoverable('pause_request_failed', str(error))
            return
        future.add_done_callback(
            lambda done, token=generation: self._on_pause_response(done, token)
        )

    def _on_pause_response(self, future, generation: int) -> None:
        with self._lock:
            if generation != self._handoff_generation:
                return
            if self.state != MissionState.STOPPING_EXPLORATION:
                return
            self._pause_request_in_flight = False
            try:
                response = future.result()
            except Exception as error:
                self._pause_request_failed = True
                self._record_recoverable('pause_request_failed', str(error))
                return

            self._pause_request_failed = False
            self._pause_response_received = True
            self._pause_status_min_sequence = self._guard_status_sequence
            if response.success:
                self.get_logger().info(
                    'MissionGuard acknowledged an immediate safe exploration handoff'
                )
                self._enter_localization('explorer ownership released by pause response')
                return
            self.get_logger().info(
                'Exploration pause latched; waiting for terminal ownership status: '
                f'{response.message}'
            )

    def _evaluate_guard_status(self, payload: dict) -> None:
        cancellation = payload['cancellation']
        if cancellation in ('failed', 'rejected') or payload['cancellation_overdue']:
            signature = (cancellation, payload['cancellation_overdue'])
            if signature == self._guard_failure_signature:
                self._guard_failure_count += 1
            else:
                self._guard_failure_signature = signature
                self._guard_failure_count = 1
            if self._guard_failure_count < 2:
                self._record_recoverable(
                    'guard_failure_pending_confirmation',
                    'MissionGuard reported an ownership failure; waiting for one '
                    'fresh repeated status before failing the mission',
                )
                return
            if payload['cancellation_overdue']:
                self._fail_terminal(
                    'exploration_cancellation_timeout',
                    f'MissionGuard cancellation is overdue: '
                    f'{payload.get("reason", "no reason")}',
                )
            else:
                self._fail_terminal(
                    'exploration_cancellation_failed',
                    f'MissionGuard cancellation state is {cancellation}: '
                    f'{payload.get("reason", "no reason")}',
                )
            return
        self._guard_failure_signature = None
        self._guard_failure_count = 0
        if not payload['pause_requested'] or not payload['handoff_safe']:
            return
        if (
            payload['goal_pending']
            or payload['goal_active']
            or cancellation != 'none'
        ):
            self._record_recoverable(
                'inconsistent_guard_status',
                'handoff_safe was true while explorer ownership remained unresolved',
            )
            return
        # A transient-local sample can be delivered late. Verify the reported
        # safe state with a fresh idempotent service call before transferring
        # Nav2 ownership.
        self._pause_response_received = False
        self._pause_status_min_sequence = self._guard_status_sequence
        self.get_logger().info(
            'MissionGuard reported handoff_safe; verifying with a fresh pause request'
        )
        self._request_exploration_pause()

    def _enter_localization(self, reason: str) -> None:
        if self.state != MissionState.STOPPING_EXPLORATION:
            self._guard_failed('localization transition', reason)
            return
        self._clear_handoff_state()
        if self._g1_explored and self._observation_pose is not None:
            # G1 must be in view again for the fresh localization.
            if self._transition(MissionState.RETURNING_TO_OBSERVATION,
                                f'{reason}; returning to the G1 observation pose'):
                self._send_navigation_goal(self._observation_pose, self.parking_navigation_timeout_s)
            return
        self.motion.reset()
        self._motion_deadline_s = self._monotonic() + self.motion_settlement_timeout_s
        self._transition(MissionState.SETTLING_MOTION,
                         f'{reason}; waiting for physical motion settlement')

    def _start_localization(self, reason: str) -> None:
        if self.state != MissionState.SETTLING_MOTION:
            return
        self._motion_deadline_s = None
        self.localization_adapter.reset()
        self._clear_recoverable_issue()
        self._localization_deadline_s = self._monotonic() + self.localization_timeout_s
        self._transition(MissionState.LOCALIZE_G1, reason)

    def _remember_association(
        self,
        stamp_ns: int,
        candidate: TargetDetectionCandidate,
    ) -> None:
        if self.confirmed_detection is not None and (
            candidate.label.strip().casefold()
            != self.confirmed_detection.label.strip().casefold()
        ):
            return
        association = TargetAssociation(
            stamp_ns=int(stamp_ns),
            label=candidate.label,
            bbox_xyxy=candidate.bbox_xyxy,
        )
        self._associations[int(stamp_ns)] = association
        self._associations.move_to_end(int(stamp_ns))
        while len(self._associations) > self.association_history_size:
            self._associations.popitem(last=False)

    def _transition(self, new_state: MissionState, reason: str) -> bool:
        if new_state == self.state:
            self._guard_failed(
                'state transition',
                f'already in {self.state.name}: {reason}',
            )
            return False
        if new_state not in _ALLOWED_TRANSITIONS[self.state]:
            self._guard_failed(
                'state transition',
                f'{self.state.name} -> {new_state.name} is not allowed: {reason}',
            )
            return False
        previous = self.state
        self.state = new_state
        transition = StateTransition(
            previous=previous,
            current=new_state,
            reason=reason,
            monotonic_s=self._monotonic(),
        )
        self.transition_history.append(transition)
        self.get_logger().info(
            f'Mission state transition: {previous.name} -> {new_state.name}; '
            f'reason={reason}'
        )
        self._publish_state(reason)
        return True

    def _record_recoverable(self, code: str, reason: str) -> None:
        self.last_issue = MissionIssue(FailureClass.RECOVERABLE, code, reason)
        self.get_logger().warning(f'Recoverable mission issue [{code}]: {reason}')
        self._publish_state(reason, event='recoverable_error')

    def _clear_recoverable_issue(self) -> None:
        if (
            self.last_issue is not None
            and self.last_issue.classification == FailureClass.RECOVERABLE
        ):
            self.last_issue = None

    def _fail_terminal(self, code: str, reason: str) -> None:
        if self.state in (
            MissionState.MISSION_FAILED,
            MissionState.MISSION_CANCELED,
            MissionState.G1_LOCALIZED,
        ):
            return
        self.last_issue = MissionIssue(FailureClass.TERMINAL, code, reason)
        self._clear_handoff_state()
        self._localization_deadline_s = None
        self.get_logger().error(f'Terminal mission failure [{code}]: {reason}')
        self._transition(MissionState.MISSION_FAILED, reason)

    def _guard_failed(self, operation: str, reason: str) -> None:
        self.get_logger().debug(f'Mission guard rejected {operation}: {reason}')
        self._publish_state(reason, event='guard_rejected')

    def _clear_handoff_state(self) -> None:
        self._handoff_deadline_s = None
        self._pause_request_in_flight = False
        self._pause_response_received = False
        self._pause_service_seen_ready = False
        self._pause_request_failed = False
        self._pause_status_min_sequence = self._guard_status_sequence
        self._guard_failure_signature = None
        self._guard_failure_count = 0

    def _publish_state_diagram(self) -> None:
        key = (self.state, len(self.transition_history))
        if key == self._state_diagram_key:
            return
        self._state_diagram_key = key
        self._state_diagram = state_diagram_overlay(
            self.state.name,
            [(t.previous.name, t.current.name, t.reason) for t in self.transition_history],
            _ALLOWED_STATE_NAMES)
        self.state_diagram_publisher.publish(self._state_diagram)

    def _publish_state(self, reason: str, *, event: str = 'transition') -> None:
        self._publish_state_diagram()
        issue = self.last_issue
        position = self.target_position
        payload = {
            'state': self.state.name,
            'state_value': self.state.value,
            'event': event,
            'reason': reason,
            'target_committed': self.target_committed,
            'start_pose_saved': self.start_pose is not None,
            'startup_paused': self._startup_paused,
            'parking_navigation_enabled': self.execution.enabled or self.parking_navigation_enabled,
            'parking_execution': self.execution.summary(),
            'parking_safety': self.safety_monitor.summary(),
            'parking_stage_path': self.stage_paths.reason,
            'parking_goal_checker': self._goal_checker_diagnostic(),
            'parking_behavior_tree': self._parking_bt_diagnostic(),
            'parking_target_admission': target_execution_reason(
                self.parking_plan.target if self.parking_plan is not None else None),
            'failure_class': None if issue is None else issue.classification.value,
            'failure_code': None if issue is None else issue.code,
            'target_reference': None if position is None else position.reference,
            'target_source': None if position is None else position.source,
            'is_foot_midpoint': None if position is None else position.is_foot_midpoint,
            'parking_plan': self._plan_summary(),
            'g1_exploration': self.g1_exploration_result,
            'parking_mission_navigation': {
                'enabled': self.parking_navigation_enabled,
                'failure': self._parking_failure,
                'arrival_errors': self._arrival_errors,
                'goal_owned': self._nav_goal_generation is not None,
                'cancel_pending': self._nav_canceling,
            },
            'motion_status': self.motion.reason,
            'parking_clearance': None if self._clearance is None else {
                'reason': self._clearance.reason, 'points': self._clearance.points,
                'nearest_m': None if self._clearance.nearest_m is None
                else round(self._clearance.nearest_m, 3),
                'g1_gap_m': None if self._clearance.g1_gap_m is None
                else round(self._clearance.g1_gap_m, 3),
                'legs': self._clearance_legs},
            'path_request_unresolved': self._path_busy,
        }
        self.state_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    @staticmethod
    def _valid_guard_status(payload) -> bool:
        if not isinstance(payload, dict):
            return False
        bool_fields = (
            'pause_requested',
            'handoff_safe',
            'goal_pending',
            'goal_active',
            'cancellation_overdue',
        )
        return (
            all(type(payload.get(field)) is bool for field in bool_fields)
            and isinstance(payload.get('cancellation'), str)
        )

    def try_plan_parking(self, now_s: float, *, now_ns: int | None = None) -> None:
        """Plan once from fresh inputs; wait for missing inputs up to a bounded timeout."""

        with self._lock:
            if self.state != MissionState.G1_LOCALIZED or not self.parking_planning_enabled:
                return
            now_ns = self.get_clock().now().nanoseconds if now_ns is None else int(now_ns)
            position = self.target_position
            target = None if position is None else TargetReference(
                x=position.map_x_m, y=position.map_y_m, frame_id=self.map_frame,
                stamp_ns=position.last_stamp_ns, reference=position.reference,
                is_foot_midpoint=position.is_foot_midpoint, source=position.source)
            try:
                grid = None if self.latest_map is None else GridSnapshot.from_msg(self.latest_map)
            except ValueError as exc:
                self._fail_planning('parking_inputs_invalid', f'map_invalid: {exc}')
                return
            # TF arrives on its own thread; its newest sample can be ahead of
            # this callback's /clock sample. Request our actual snapshot time
            # first. The fallback still passes the unchanged future/age checks.
            robot = self._frame_pose(self.base_frame, Time(nanoseconds=now_ns))
            if robot is None:
                robot = self._current_robot_pose(with_stamp=True)
            reasons = validate_inputs(
                target, grid, robot, now_ns=now_ns, map_frame=self.map_frame,
                max_target_age_s=self.parking_max_target_age_s,
                max_map_age_s=self.parking_max_map_age_s,
                max_pose_age_s=self.parking_max_pose_age_s)
            if reasons:
                # A stale or wrong-frame target cannot improve by waiting.
                waitable = {'map_unavailable', 'map_stale', 'robot_pose_unavailable',
                            'robot_pose_stale'}
                expired = (self._planning_deadline_s is not None
                           and now_s >= self._planning_deadline_s)
                if set(reasons) <= waitable and not expired:
                    # Waiting is not an error; report it without raising an issue.
                    self.get_logger().debug(f'Parking inputs pending: {reasons}')
                    self._publish_state(', '.join(reasons), event='planning_pending')
                    return
                self._fail_planning('parking_inputs_invalid', ', '.join(reasons))
                return

            approach = self.approach_pose or robot[:3]
            plan = plan_parking(grid, target, self.planner_config, approach)
            self.parking_plan = plan
            self._planning_deadline_s = None
            counts = plan.counts()
            if not plan.ok:
                self._fail_planning(plan.failure_reason, f'no feasible parking pose: {counts}')
                return
            # The geometric selection is provisional, not a path-feasible selection.
            plan.selected.status = 'valid'
            plan.selected = None
            plan.ok = False
            self._planning_snapshot = PlanningSnapshot(robot, self._target_map_odom)
            if self._target_map_odom is None:
                self._fail_planning('parking_tf_unavailable', 'source-time map-to-odom unavailable')
                return
            self._path_groups = candidate_groups(plan, approach, self.path_candidates_per_class)
            self._path_queue = []
            self._path_valid = []
            self._paths = {}
            self._path_batch_deadline = now_s + self.parking_path_batch_timeout_s
            self._clear_recoverable_issue()
            self.get_logger().info(f'Geometric parking candidates ready: {counts}')
            self._publish_parking_markers()
            self._transition(
                MissionState.PARKING_CANDIDATES_READY,
                'geometric candidates generated; path feasibility pending')
            # Geometry runs synchronously: odometry may be queued while this
            # callback works. Yield and require new real stop evidence if stale.
            if not self.motion.ready(self.get_clock().now().nanoseconds, self._monotonic()):
                self._motion_deadline_s = self._monotonic() + self.motion_settlement_timeout_s
                self.get_logger().info(
                    'Waiting for fresh post-planning odometry before path evaluation')
                return
            self._mark_motion_evidence()
            self._transition(MissionState.EVALUATING_PARKING_PATHS,
                             'bounded sequential ComputePathToPose evaluation; no driving')
            self._next_path(now_ns=now_ns)

    def _fail_planning(self, code: str, reason: str) -> None:
        self.last_issue = MissionIssue(FailureClass.TERMINAL, code, reason)
        self._invalidate_path_requests()
        self._planning_deadline_s = None
        if self.parking_plan is not None:
            self.parking_plan.ok = False
            self.parking_plan.selected = None
            self.parking_plan.selected_path = None
            self.parking_plan.failure_reason = code
        if code in ('no_free_donut_cells', 'no_valid_candidate', 'bounded_search_exhausted') and self.parking_plan is not None:
            self._publish_parking_markers()
        else:
            self.parking_marker_publisher.publish(clear_markers())
        self.get_logger().error(f'Parking planning failed [{code}]: {reason}')
        self._transition(MissionState.PARKING_PLANNING_FAILED, reason)

    def _current_robot_pose(self, *, with_stamp: bool = False):
        pose = self._frame_pose(self.base_frame, Time())
        return pose if with_stamp or pose is None else pose[:3]

    def _map_odom(self, stamp=None):
        return self._frame_pose(self.odom_frame, stamp if stamp is not None else
                                Time(nanoseconds=self.get_clock().now().nanoseconds))

    def _recent_map_odom(self, now_ns):
        """map->odom at ``now_ns``, else the newest transform within the pose-age bound.

        slam_toolbox may not have published a transform for the exact current
        time yet; one such miss must not abort an otherwise valid plan.
        """
        pose = self._map_odom(Time(nanoseconds=now_ns))
        if pose is None:
            pose = self._frame_pose(self.odom_frame, Time())
            if pose is not None and not 0 <= (now_ns - pose[3]) * 1e-9 <= self.parking_max_pose_age_s:
                pose = None
        return pose

    def _frame_pose(self, frame, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, frame, stamp)
        except TransformException:
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or norm < 1e-9:
            return None
        x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        pose = (float(t.x), float(t.y), yaw)
        stamp = transform.header.stamp
        return pose + (int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),)

    def _on_odometry(self, msg):
        with self._lock:
            self._execution_odometry = msg
            self._execution_odom_received_s = self._monotonic()
            if self.execution.state == ExecutionState.IDLE and self.state not in (
                                  MissionState.SETTLING_MOTION, MissionState.LOCALIZE_G1,
                                  MissionState.G1_LOCALIZED, MissionState.PARKING_CANDIDATES_READY,
                                  MissionState.EVALUATING_PARKING_PATHS,
                                  MissionState.PARKING_PATH_FEASIBLE) + _PARKING_DRIVE_STATES:
                return
            t = msg.twist.twist
            self.motion.observe((t.linear.x, t.linear.y, t.linear.z,
                                 t.angular.x, t.angular.y, t.angular.z),
                                Time.from_msg(msg.header.stamp).nanoseconds,
                                self.get_clock().now().nanoseconds, self._monotonic(),
                                frame_valid=(msg.child_frame_id == self.base_frame and
                                             msg.header.frame_id == self.odom_frame))

    def _prepare_stage(self, request, response):
        with self._lock:
            if self.state != MissionState.PARKING_PATH_FEASIBLE:
                response.success, response.message = False, 'no current path-feasible parking selection'
                return response
            final = self.execution.state == ExecutionState.READY_FOR_FINAL
            response.success = self.stage_paths.request(final=final)
            response.message = self.stage_paths.reason
            self._publish_state(response.message, event='stage_path_preparation')
            return response

    def _safety_intervention(self):
        reason = self.safety_monitor.intervention(self.get_clock().now().nanoseconds,
            self._monotonic(), self.execution.goal_uuid if self.execution.busy else None)
        if reason:
            if self.execution.busy or self.execution.state == ExecutionState.PREPARED:
                self.execution.cancel(reason)
            if self.stage_paths.busy:
                self.stage_paths.cancel(reason)

    def _on_stop_lock(self, topic, msg):
        with self._lock:
            self.safety_monitor.observe_lock(topic, msg.data,
                self.get_clock().now().nanoseconds, self._monotonic())
            self._safety_intervention()

    def _on_command_evidence(self, topic, msg):
        with self._lock:
            self.safety_monitor.observe_command(topic, msg,
                self.get_clock().now().nanoseconds, self._monotonic())
            self._safety_intervention()

    def _on_platform_stop(self, msg):
        with self._lock:
            self.safety_monitor.observe_stop_status(msg, self._monotonic())
            if (msg.needs_reset or not msg.stop_power_status) and self.execution.busy:
                self.execution.cancel('stop_loop_fault')

    def _on_navigation_status(self, topic, msg):
        with self._lock:
            self.safety_monitor.observe_action_status(topic, msg,
                self.get_clock().now().nanoseconds, self._monotonic())
            self._safety_intervention()

    def _read_safety_service(self, name, client, request, decode):
        if self._probe_inflight.get(name) or not client.service_is_ready():
            return
        deadline = self._monotonic() + 2.0
        self._probe_inflight[name] = True
        try:
            future = client.call_async(request)
        except Exception:
            self._probe_inflight[name] = False
            self._probe_values.pop(name, None)
            return

        def done(f):
            with self._lock:
                self._probe_inflight[name] = False
                try:
                    value = decode(f.result()) if self._monotonic() < deadline else None
                except Exception:
                    value = None
                now_ns, wall_s = self.get_clock().now().nanoseconds, self._monotonic()
                self._probe_values[name] = (value, now_ns, wall_s)
                if name == 'platform':
                    self.safety_monitor.observe_controller(value is True, now_ns, wall_s)
                    if value is not True and self.execution.busy:
                        self.execution.cancel('platform_controller_fault_or_unavailable')
                self._publish_state('read-only safety evidence updated', event='safety_probe')
        future.add_done_callback(done)

    def _poll_safety_evidence(self, wall_s):
        if wall_s - self._last_probe_s < 1.0:
            return
        self._last_probe_s = wall_s
        self._read_safety_service('platform', self._platform_client, ListControllers.Request(),
            lambda r: any(c.name == 'platform_velocity_controller' and c.state == 'active'
                          for c in r.controller))
        req = GetParameters.Request(names=list(GOAL_CHECKER_PARAMETERS))
        self._read_safety_service('checker', self._checker_client, req,
            lambda r: dict(zip(GOAL_CHECKER_PARAMETERS,
                              (parameter_value_to_python(v) for v in r.values))))
        self._read_safety_service('checker_lifecycle', self._checker_lifecycle_client,
            GetState.Request(), lambda r: r.current_state.id == 3)

    def _goal_checker_diagnostic(self):
        now_ns, wall_s = self.get_clock().now().nanoseconds, self._monotonic()
        for key in ('checker', 'checker_lifecycle'):
            item = self._probe_values.get(key)
            if (item is None or not 0 <= wall_s - item[2] <= 1.5
                    or not 0 <= (now_ns - item[1])*1e-9 <= 1.5):
                return 'parking_goal_checker_evidence_unknown_or_stale'
        if self._probe_values['checker_lifecycle'][0] is not True:
            return 'parking_controller_not_active'
        values = self._probe_values['checker'][0]
        return goal_checker_reason(values or {}, self.execution.limits.xy_tolerance_m,
                                    self.execution.limits.yaw_tolerance_rad)

    def _execution_safety_check(self, ctx, moving):
        reason = target_execution_reason(ctx.plan.target) or self._parking_bt_diagnostic()
        if reason:
            return reason
        reason = self.safety_monitor.check(ctx, self.planner_config, moving=moving,
            own_goal=self.execution.goal_uuid if self.execution.busy else None)
        return reason or self._goal_checker_diagnostic()

    def _parking_bt_diagnostic(self):
        identity, reason = parking_tree_identity(self.execution.behavior_tree, self.planner_id)
        if reason:
            return reason
        if self._parking_bt_identity is None or identity != self._parking_bt_identity:
            return 'parking_bt_changed_restart_and_requalify'
        return None

    def _publish_keepout_mask(self):
        if self.parking_plan is None or self.latest_map is None:
            return
        plan = self.parking_plan
        if plan.target is None or not 0 <= (self.get_clock().now().nanoseconds - plan.target.stamp_ns)*1e-9 <= self.parking_max_target_age_s:
            return  # Retain the last mask; never clear protection on target loss.
        try:
            grid = GridSnapshot.from_msg(self.latest_map)
            if grid.frame_id != self.map_frame:
                return
            key = (plan.target, grid.origin_x, grid.origin_y, grid.origin_yaw,
                   grid.resolution, grid.width, grid.height, plan.keepout_radius_m)
            if key == self._mask_key:
                return
            msg = keepout_mask(grid, plan.target, plan.keepout_radius_m)
        except ValueError:
            return
        info = CostmapFilterInfo()
        info.header = msg.header
        info.type, info.base, info.multiplier = 0, 0.0, 1.0
        info.filter_mask_topic = self.resolve_topic_name('parking/g1_keepout_mask')
        self.mask_publisher.publish(msg)
        self.mask_info_publisher.publish(info)
        self._mask_key = key

    def _execution_context(self):
        try:
            grid = None if self.latest_map is None else GridSnapshot.from_msg(self.latest_map)
        except ValueError:
            grid = None
        now_ns = self.get_clock().now().nanoseconds
        target_robot = None
        if self.parking_plan is not None and self.parking_plan.target is not None:
            target_robot = self._frame_pose(self.base_frame,
                Time(nanoseconds=self.parking_plan.target.stamp_ns))
        return ExecutionContext(
            plan=self.parking_plan, grid=grid,
            robot=self._current_robot_pose(with_stamp=True),
            map_odom=self._map_odom(Time(nanoseconds=now_ns)),
            snapshot=self._planning_snapshot, motion=self.motion,
            now_ns=now_ns, wall_s=self._monotonic(), safety=self.execution_safety,
            path_request_unresolved=self._path_busy or bool(getattr(self, 'stage_paths', None)
                and self.stage_paths.busy), map_frame=self.map_frame,
            odom_frame=self.odom_frame, base_frame=self.base_frame,
            odometry=self._execution_odometry,
            odometry_received_s=self._execution_odom_received_s, target_robot=target_robot)

    def prepare_parking_navigation(self, path_result=None, *, final=False):
        """Accept a terminal stage ComputePathToPose result; never invent a route."""
        with self._lock:
            if self.state != MissionState.PARKING_PATH_FEASIBLE:
                return False
            ready = self.execution.prepare(path_result, final=final)
            self.get_logger().info(
                f'Parking execution preparation: {self.execution.reason}; no goal sent')
            self._publish_state(self.execution.reason, event='execution_preparation')
            return ready

    def request_parking_navigation(self):
        """Explicit internal request, never called by a timer or launch action.

        A future trusted safety adapter and separately qualified stage path are
        mandatory even when the execution parameter is enabled.
        """
        with self._lock:
            if self.state != MissionState.PARKING_PATH_FEASIBLE:
                return False
            return self.execution.start()

    def _execution_changed(self):
        self.get_logger().info(
            f'Parking execution: {self.execution.state.name}; {self.execution.reason}')
        if (self.execution.state == ExecutionState.FAILED
                and self.state == MissionState.PARKING_PATH_FEASIBLE):
            self.last_issue = MissionIssue(FailureClass.TERMINAL,
                'parking_execution_failed', self.execution.reason)
            if self.parking_plan is not None:
                self.parking_plan.ok = False
                self.parking_plan.selected = None
                self.parking_plan.selected_path = None
                self.parking_plan.failure_reason = 'parking_execution_failed'
            self.parking_marker_publisher.publish(clear_markers())
            self._transition(MissionState.PARKING_EXECUTION_FAILED, self.execution.reason)
        self._publish_state(self.execution.reason, event='parking_execution')

    def _on_collision_state(self, msg):
        with self._lock:
            self.execution.collision_state(msg.action_type,
                self.get_clock().now().nanoseconds, self._monotonic())

    def _mark_motion_evidence(self):
        """A plan stays valid only while no motion fault follows this point."""
        self._motion_fault_mark = self.motion.faults
        self._motion_restart_s = None

    def _begin_parking_when_stationary(self, now_ns=None):
        # Driving needs complete stationary evidence, not just an unfaulted restart.
        now_ns = self.get_clock().now().nanoseconds if now_ns is None else now_ns
        if self.motion.ready(now_ns, self._monotonic()):
            self._begin_parking_navigation()

    def _plan_invalid_reason(self, now_ns=None):
        now_ns = self.get_clock().now().nanoseconds if now_ns is None else now_ns
        wall_s = self._monotonic()
        if self.motion.ready(now_ns, wall_s):
            self._motion_restart_s = None
        elif self.motion.faults != self._motion_fault_mark:
            return f'motion_evidence_lost: {self.motion.reason}'
        else:
            # Synchronous path checks can delay odometry callbacks past the
            # evidence window; the interval then restarts without any motion.
            # Wait a bounded time for it to complete instead of discarding the plan.
            if self._motion_restart_s is None:
                self._motion_restart_s = wall_s
            elif wall_s - self._motion_restart_s > self.motion_settlement_timeout_s:
                return f'motion_evidence_lost: {self.motion.reason}'
        plan = self.parking_plan
        if plan is None:
            return 'plan_unavailable'
        try:
            grid = None if self.latest_map is None else GridSnapshot.from_msg(self.latest_map)
        except ValueError as exc:
            return f'map_invalid: {exc}'
        robot = self._current_robot_pose(with_stamp=True)
        # TF keeps updating on its own thread while this node's clock callback can
        # wait behind a path check; stamps that far ahead are not future data.
        reasons = validate_inputs(plan.target, grid, robot, now_ns=now_ns,
                                  map_frame=self.map_frame, max_target_age_s=self.parking_max_target_age_s,
                                  max_map_age_s=self.parking_max_map_age_s,
                                  max_pose_age_s=self.parking_max_pose_age_s,
                                  clock_skew_s=self.parking_max_pose_age_s)
        if reasons:
            return ', '.join(reasons)
        reason = self._planning_snapshot.changed(
            robot, self._recent_map_odom(now_ns),
            self.parking_max_frame_shift_m, self.parking_max_frame_yaw_rad)
        if reason:
            return reason
        # Revalidate the selected route against the latest map, including same-stamp changes.
        if plan.selected is not None and plan.selected_path is not None:
            return path_rejection(plan.selected_path, plan.selected, robot, grid, plan.target,
                                  self.planner_config, plan.keepout_radius_m,
                                  occupied=self._start_footprint_poses())
        return None

    def _publish_parking_markers(self):
        self.parking_marker_publisher.publish(plan_markers(
            self.parking_plan, self.planner_config, self.get_clock().now().to_msg()))

    def _next_path(self, now_ns=None):
        if self.state != MissionState.EVALUATING_PARKING_PATHS or self._path_busy:
            return
        reason = self._plan_invalid_reason(now_ns)
        if reason:
            self._fail_planning('parking_plan_invalidated', reason)
            return
        if not self._path_queue:
            if self._path_valid:
                selected = select_path_candidate(self._path_valid, self.parking_plan.resolution_m)
                plan = self.parking_plan
                plan.selected, plan.selected_path, plan.ok = selected, self._paths[id(selected)], True
                selected.status = 'selected'
                reason = self._plan_invalid_reason(now_ns)
                if reason:
                    self._fail_planning('parking_plan_invalidated', reason)
                    return
                self._publish_parking_markers()
                self.get_logger().info(
                    f'Path-feasible parking pose selected: x={selected.base_x:.3f}, '
                    f'y={selected.base_y:.3f}, yaw={selected.yaw:.3f}, '
                    f'path_length={selected.path_length_m:.3f} m')
                self._transition(MissionState.PARKING_PATH_FEASIBLE,
                                 'path-feasible snapshot selected')
                if not self.parking_navigation_enabled:
                    self.prepare_parking_navigation()
                else:
                    self._begin_parking_when_stationary(now_ns)
                return
            if not self._path_groups:
                self._fail_planning('bounded_search_exhausted',
                    'No safe path among bounded candidates; untested poses may remain')
                return
            self._path_queue = list(self._path_groups.pop(0))
        if not self.compute_path_client.server_is_ready():
            return
        candidate = self._path_queue.pop(0)
        robot = self._current_robot_pose(with_stamp=True)
        request = ComputePathToPose.Goal()
        # Plan the route actually driven: Nav2 to pre-parking, then straight in.
        request.goal = self._pose_stamped(*preparking_pose(
            candidate.base_x, candidate.base_y, candidate.yaw,
            self.execution.limits.preparking_offset_m))
        request.start = self._pose_stamped(*robot[:3])
        request.use_start = True
        request.planner_id = self.planner_id
        self._path_busy = True
        self._path_request_deadline = self._monotonic() + self.parking_path_timeout_s
        generation = self._path_generation
        self.parking_plan.paths_checked += 1
        self.get_logger().info(f'Checking parking path {self.parking_plan.paths_checked}: '
                               f'x={candidate.base_x:.3f}, y={candidate.base_y:.3f}')
        try:
            future = self.compute_path_client.send_goal_async(request)
        except Exception as exc:
            self._path_busy = False
            self._fail_planning('planner_transport', str(exc))
            return
        future.add_done_callback(lambda f: self._path_goal_response(f, generation, candidate, robot))

    def _start_footprint_poses(self):
        """The saved mission start: the robot stood there, so its footprint held no obstacle."""
        return () if self.start_pose is None else (pose_xyyaw(self.start_pose.pose),)

    def _pose_stamped(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(x), float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return msg

    def _path_goal_response(self, future, generation, candidate, robot):
        with self._lock:
            try:
                handle = future.result()
            except Exception as exc:
                # Acceptance is unknown: retain unresolved ownership, do not send another request.
                if generation == self._path_generation:
                    self._fail_planning('planner_acceptance_unknown', str(exc))
                return
            if not handle.accepted:
                self._path_busy = False
                if generation == self._path_generation:
                    candidate.path_status, candidate.path_reason = 'rejected', 'goal_rejected'
                    self._publish_parking_markers()
                return
            self._path_handle = handle
            try:
                result_future = handle.get_result_async()
                result_future.add_done_callback(
                    lambda f: self._path_result(f, generation, candidate, robot, handle))
            except Exception as exc:
                if generation == self._path_generation:
                    self._fail_planning('planner_result_unavailable', str(exc))
                return
            if generation != self._path_generation or self.state != MissionState.EVALUATING_PARKING_PATHS:
                self._cancel_path_request()

    def _path_result(self, future, generation, candidate, robot, handle):
        with self._lock:
            if handle is not self._path_handle:
                return
            try:
                wrapped = future.result()
                if wrapped.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                           GoalStatus.STATUS_CANCELED):
                    raise ValueError('nonterminal path result')
            except Exception as exc:
                if generation == self._path_generation:
                    self._fail_planning('planner_result_unknown', str(exc))
                return
            self._path_handle = None
            self._path_busy = self._path_cancel_requested = False
            if generation != self._path_generation or self.state != MissionState.EVALUATING_PARKING_PATHS:
                return
            if (self._monotonic() >= self._path_request_deadline or
                    self._monotonic() >= self._path_batch_deadline):
                self._fail_planning('planner_timeout', 'late ComputePathToPose result discarded')
                return
            reason = self._plan_invalid_reason()
            if reason:
                self._fail_planning('parking_plan_invalidated', reason)
                return
            result = wrapped.result
            reason = None
            route = None
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or result.error_code != 0:
                reason = f'nav2_status_{wrapped.status}_error_{result.error_code}: {result.error_msg}'
            elif not result.path.poses:
                reason = 'empty_path'
            else:
                # Append the straight final approach so the swept-footprint and
                # keep-out checks cover the whole driven route to the parking pose.
                route = deepcopy(result.path)
                route.poses.append(self._pose_stamped(candidate.base_x, candidate.base_y,
                                                      candidate.yaw))
                grid = GridSnapshot.from_msg(self.latest_map)
                path_stamp = Time.from_msg(result.path.header.stamp).nanoseconds
                now = self.get_clock().now().nanoseconds
                if path_stamp > now or now - path_stamp > int(self.parking_max_pose_age_s * 1e9):
                    reason = 'path_timestamp'
                else:
                    reason = path_rejection(route, candidate, robot, grid, self.parking_plan.target,
                                            self.planner_config, self.parking_plan.keepout_radius_m,
                                            occupied=self._start_footprint_poses())
            candidate.path_status = 'rejected' if reason else 'valid'
            candidate.path_reason = reason
            if reason:
                self.get_logger().info(f'Parking path rejected: {reason}')
            else:
                candidate.path_length_m = path_length(route)
                self._path_valid.append(candidate)
                self._paths[id(candidate)] = route
            self._publish_parking_markers()
            # The existing control timer sends the next request: no callback recursion.

    def _cancel_path_request(self):
        if self._path_handle is not None and not self._path_cancel_requested:
            self._path_cancel_requested = True
            try:
                self._path_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f'Path cancellation unresolved: {exc}')

    def _invalidate_path_requests(self):
        self._path_generation += 1
        self._cancel_path_request()
        self._path_groups, self._path_queue, self._path_valid = [], [], []

    def destroy_node(self):
        self._invalidate_path_requests()
        if self.state in _PARKING_DRIVE_STATES or self._nav_goal_generation is not None:
            self._stop_parking_navigation()
        if self._nav_goal_generation is not None:
            self.get_logger().error('Shutting down before Nav2 confirmed a terminal NavigateToPose '
                                    'result; cancellation was requested but not confirmed')
        self.stage_paths.cancel('manager_shutdown')
        if self.execution.busy:
            self.execution.cancel('manager_shutdown')
        if self.tf_listener is not None:
            # Join the dedicated TF executor before destroying the ROS context.
            self.tf_listener.executor.shutdown()
            self.tf_listener.dedicated_listener_thread.join()
            self.tf_listener.unregister()
            self.tf_listener.node.destroy_node()
            self.tf_listener = None
        return super().destroy_node()

    # Original mission tail: pre-parking -> final -> stop and wait -> return to start.

    def _begin_parking_navigation(self):
        self._transition(MissionState.NAVIGATING_TO_PREPARKING,
                         'path-feasible parking pose selected; parking navigation enabled')
        # The original parked with a 0.08 m / 0.12 rad goal checker; the shared
        # profile is far looser, so tighten it for the two parking legs only.
        limits = self.execution.limits
        self._set_goal_tolerance((limits.xy_tolerance_m, limits.yaw_tolerance_rad),
                                 self._send_preparking_goal)

    def _send_preparking_goal(self, ok):
        if not ok:
            self._parking_navigation_failed('goal_checker_tolerance_not_applied')
            return
        sel = self.parking_plan.selected
        pose = preparking_pose(sel.base_x, sel.base_y, sel.yaw,
                               self.execution.limits.preparking_offset_m)
        self._send_navigation_goal(pose, self.parking_navigation_timeout_s)

    def _send_final_goal(self, ok):
        if not ok:
            self._parking_navigation_failed('final_goal_tolerance_not_applied')
            return
        sel = self.parking_plan.selected
        self._send_navigation_goal((sel.base_x, sel.base_y, sel.yaw),
                                   self.parking_navigation_timeout_s)

    def _set_goal_tolerance(self, tolerance, on_done):
        """Set the controller's goal checker; ``on_done(ok)`` runs for the current request only."""
        self._nav_generation += 1
        generation, state = self._nav_generation, self.state
        if not self._goal_checker_set_client.service_is_ready():
            on_done(False)
            return
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=f'{self.goal_checker_plugin}.{name}', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
            for name, value in zip(('xy_goal_tolerance', 'yaw_goal_tolerance'), tolerance)]
        self._nav_deadline_s = self._monotonic() + 5.0
        self._nav_pending_tolerance = on_done

        def done(future):
            with self._lock:
                if generation != self._nav_generation or self.state != state:
                    return
                self._nav_pending_tolerance = None
                try:
                    ok = all(result.successful for result in future.result().results)
                except Exception:
                    ok = False
                on_done(ok)

        self._goal_checker_set_client.call_async(request).add_done_callback(done)

    def _send_navigation_goal(self, pose, timeout_s):
        if self._nav_goal_generation is not None:
            # Never compete with a goal Nav2 may still be executing.
            self.get_logger().error(f'{self.state.name}: previous NavigateToPose goal not released')
            self._navigation_finished(False, 'previous navigation goal not released')
            return
        if not self.navigation_client.server_is_ready():
            self._navigation_finished(False, 'navigate_to_pose server unavailable')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose_stamped(*pose)
        self._nav_generation += 1
        generation = self._nav_generation
        self._nav_goal_generation, self._nav_handle = generation, None
        self._nav_canceling, self._nav_cancel_deadline_s = False, None
        self._nav_deadline_s = self._monotonic() + timeout_s
        self.get_logger().info(f'{self.state.name}: NavigateToPose x={pose[0]:.3f}, '
                               f'y={pose[1]:.3f}, yaw={pose[2]:.3f}')
        try:
            future = self.navigation_client.send_goal_async(goal)
        except Exception as exc:
            # The request may have reached Nav2: the goal stays owned until the cancel deadline.
            self._abandon_navigation(f'goal transport failed: {exc}')
            return
        future.add_done_callback(lambda f: self._navigation_accepted(f, generation))

    def _navigation_accepted(self, future, generation):
        with self._lock:
            if generation != self._nav_goal_generation:
                return
            try:
                handle = future.result()
            except Exception as exc:
                self._abandon_navigation(f'goal acceptance unknown: {exc}')
                return
            if handle is None or not handle.accepted:
                self._release_navigation_goal('goal rejected', ok=False)
                return
            self._nav_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f: self._navigation_result(f, generation))
            if self._nav_canceling:
                self._request_navigation_cancel()   # cancelled while acceptance was pending

    def _navigation_result(self, future, generation):
        with self._lock:
            if generation != self._nav_goal_generation:
                return
            try:
                wrapped = future.result()
                status, code = wrapped.status, wrapped.result.error_code
                detail = f'status={status}, error_code={code}, {wrapped.result.error_msg}'
            except Exception as exc:
                status, code, detail = None, None, f'result unavailable: {exc}'
            if status not in _TERMINAL_GOAL_STATUSES:
                # Without a terminal status Nav2 may still be executing the goal.
                self._abandon_navigation(f'navigation result not terminal: {detail}')
                return
            self._release_navigation_goal(
                detail, ok=status == GoalStatus.STATUS_SUCCEEDED and code == 0)

    def _release_navigation_goal(self, detail, *, ok):
        """Nav2 no longer owns the goal; a cancelled goal's outcome is never reported as the leg's."""
        canceling = self._nav_canceling
        self._nav_goal_generation = self._nav_handle = self._nav_cancel_deadline_s = None
        self._nav_canceling = False
        if not canceling:
            self._nav_deadline_s = None
            self._navigation_finished(ok, detail)
            return
        self.get_logger().info(f'Cancelled NavigateToPose released by Nav2: {detail}')
        self._publish_state(f'cancelled navigation goal released: {detail}',
                            event='navigation_released')
        if self.state == MissionState.RETURNING_TO_START and self._return_after_stop:
            self._settle_before_return()

    def _abandon_navigation(self, detail):
        """Fail the current goal while Nav2 may still own it; nothing new drives before release."""
        if self._nav_canceling:
            return   # the pending cancellation's deadline decides
        if self.state in _PARKING_LEGS:
            self._parking_navigation_failed(detail, stop_first=True)
        else:
            self._cancel_navigation_goal()
            self._navigation_finished(False, detail)

    def _navigation_finished(self, ok, detail):
        sel = self.parking_plan.selected if self.parking_plan is not None else None
        if self.state in _PARKING_LEGS:
            self.get_logger().info(f'LiDAR clearance in {self.state.name}: '
                                   f'{self._clearance_legs.get(self.state.value)}')
        if self.state == MissionState.RETURNING_TO_OBSERVATION:
            if not ok:
                self.last_issue = MissionIssue(FailureClass.TERMINAL,
                                               'observation_return_failed', detail)
                self._transition(MissionState.MISSION_FAILED,
                                 f'could not return to the G1 observation pose: {detail}')
                return
            self.motion.reset()
            self._motion_deadline_s = self._monotonic() + self.motion_settlement_timeout_s
            self._transition(MissionState.SETTLING_MOTION,
                             'observation pose reached; waiting for physical motion settlement')
            return
        if self.state == MissionState.NAVIGATING_TO_PREPARKING:
            if not ok:
                self._parking_navigation_failed(f'preparking_navigation_failed: {detail}')
                return
            self._record_arrival('preparking', preparking_pose(
                sel.base_x, sel.base_y, sel.yaw, self.execution.limits.preparking_offset_m))
            reason = self._final_pose_reason()
            if reason:
                self._parking_navigation_failed(f'final_pose_invalid: {reason}')
                return
            self._transition(MissionState.NAVIGATING_TO_PARKING, 'pre-parking pose reached')
            self._set_goal_tolerance(
                (self.final_xy_tolerance_m, self.execution.limits.yaw_tolerance_rad),
                self._send_final_goal)
        elif self.state == MissionState.NAVIGATING_TO_PARKING:
            if not ok:
                self._parking_navigation_failed(f'parking_navigation_failed: {detail}')
                return
            self._record_arrival('parking', (sel.base_x, sel.base_y, sel.yaw))
            # Require fresh evidence that the robot has stopped before the wait.
            self.motion.reset()
            self._parked_settled_ns = None
            self._nav_deadline_s = self._monotonic() + self.motion_settlement_timeout_s
            self._transition(MissionState.PARKED, 'final parking pose reached; waiting for stop')
        elif self.state == MissionState.RETURNING_TO_START:
            if ok:
                start = self.start_pose.pose
                self._record_arrival('start', (start.position.x, start.position.y,
                                               _yaw(start.orientation)))
                if self._parking_failure is None:
                    self._transition(MissionState.MISSION_COMPLETE, 'returned to start pose')
                    return
                self.last_issue = MissionIssue(FailureClass.TERMINAL, 'parking_failed',
                                               self._parking_failure)
                self._transition(MissionState.MISSION_FAILED,
                                 f'returned to start after: {self._parking_failure}')
            else:
                self.last_issue = MissionIssue(FailureClass.TERMINAL,
                                               'return_navigation_failed', detail)
                self._transition(MissionState.MISSION_FAILED, f'return to start failed: {detail}')

    def _poll_parking_navigation(self, now_s):
        if self.state in _PARKING_LEGS and self._check_parking_clearance():
            return
        if self.state == MissionState.RETURNING_TO_START and self._return_after_stop:
            if self._nav_goal_generation is not None:
                return   # the cancelled leg has not been released by Nav2 yet
            if self.motion.ready(self.get_clock().now().nanoseconds, now_s):
                self._return_after_stop = False
                self._nav_deadline_s = None
                self._set_goal_tolerance(self._shared_goal_tolerance, self._send_return_goal)
            elif now_s >= self._nav_deadline_s:
                self._return_after_stop = False
                self._nav_deadline_s = None
                self.last_issue = MissionIssue(FailureClass.TERMINAL, 'stop_not_confirmed',
                                               self.motion.reason)
                self._transition(MissionState.MISSION_FAILED,
                                 f'no confirmed stop after the parking veto: {self.motion.reason}')
            return
        if self.state == MissionState.PARKED and self._parked_settled_ns is not None:
            now_ns = self.get_clock().now().nanoseconds
            if (now_ns - self._parked_settled_ns) * 1e-9 >= self.parking_wait_s:
                self._start_return('parking wait completed')
            return
        if self.state == MissionState.PARKED and self.motion.ready(
                self.get_clock().now().nanoseconds, now_s):
            self._parked_settled_ns = self.get_clock().now().nanoseconds
            self._nav_deadline_s = None
            self.get_logger().info(f'Parked and stopped; waiting {self.parking_wait_s:.1f} s')
            self._publish_state('parked and stopped', event='parking_wait')
            return
        if self._nav_deadline_s is None or now_s < self._nav_deadline_s:
            return
        if self._nav_pending_tolerance is not None:
            on_done, self._nav_pending_tolerance = self._nav_pending_tolerance, None
            self._nav_generation += 1   # ignore a late parameter response
            on_done(False)
            return
        detail = f'{self.state.name} deadline expired'
        if self.state in _PARKING_LEGS:
            self._abandon_navigation(detail)
            return
        self._cancel_navigation_goal()
        if self.state == MissionState.RETURNING_TO_START:
            self._navigation_finished(False, detail)
        else:
            self._parking_navigation_failed(detail)

    def _parking_navigation_failed(self, reason, *, stop_first=False):
        self.get_logger().error(f'Parking failed: {reason}; returning to start')
        self._parking_failure = reason
        self._start_return(reason, stop_first=stop_first)

    def _start_return(self, reason, *, stop_first=False):
        if stop_first:
            # A cancelled leg is not resumed: the return starts only after Nav2 releases
            # the goal and fresh odometry then confirms the stop.
            self._transition(MissionState.RETURNING_TO_START, f'{reason}; stopping before the return')
            self._return_after_stop = True
            self._cancel_navigation_goal()
            if self._nav_goal_generation is None:
                self._settle_before_return()
            return
        self._transition(MissionState.RETURNING_TO_START, reason)
        self._set_goal_tolerance(self._shared_goal_tolerance, self._send_return_goal)

    def _settle_before_return(self):
        self.motion.reset()
        self._nav_deadline_s = self._monotonic() + self.motion_settlement_timeout_s

    # Live LiDAR clearance during the two parking legs.

    def _on_scan(self, topic, msg):
        with self._lock:
            self._scans[topic] = (msg, self._monotonic())
            if self.state in _PARKING_LEGS:
                self._check_parking_clearance()

    def _laser_pose(self, frame):
        if frame not in self._laser_in_base:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time()).transform
            except TransformException:
                return None
            self._laser_in_base[frame] = (t.translation.x, t.translation.y, _yaw(t.rotation))
        return self._laser_in_base[frame]

    def _check_parking_clearance(self):
        """Cancel the leg when the next movement is unsafe; True when it was cancelled."""
        reason, result = self._parking_clearance(), None
        if isinstance(reason, ClearanceResult):
            result, reason = reason, reason.reason
            self._clearance = result
            # Per-leg evidence that the check ran: count and closest distances seen.
            leg = self._clearance_legs.setdefault(self.state.value, {
                'checks': 0, 'min_nearest_m': None, 'min_g1_gap_m': None})
            leg['checks'] += 1
            for key, value in (('min_nearest_m', result.nearest_m), ('min_g1_gap_m', result.g1_gap_m)):
                if value is not None and (leg[key] is None or value < leg[key]):
                    leg[key] = round(value, 3)
        if reason is None:
            return False
        detail = '' if result is None else (
            f' (nearest return {result.nearest_m if result.nearest_m is None else round(result.nearest_m, 3)} m,'
            f' G1 gap {result.g1_gap_m if result.g1_gap_m is None else round(result.g1_gap_m, 3)} m)')
        self.get_logger().error(f'Parking LiDAR veto in {self.state.name}: {reason}{detail}')
        self._parking_navigation_failed(f'lidar_clearance: {reason}', stop_first=True)
        return True

    def _parking_clearance(self):
        now_ns, now_s = self.get_clock().now().nanoseconds, self._monotonic()
        limit = self.clearance_limits.scan_max_age_s
        xs, ys, age = [], [], 0.0
        for topic in self.parking_lidar_topics:
            entry = self._scans.get(topic)
            if entry is None:
                return f'lidar_unavailable: {topic}'
            scan, received_s = entry
            stamp_age = (now_ns - Time.from_msg(scan.header.stamp).nanoseconds) * 1e-9
            if not -limit <= stamp_age <= limit or now_s - received_s > 2.0 * limit:
                return f'lidar_stale: {topic}'
            laser = self._laser_pose(scan.header.frame_id)
            if laser is None:
                return f'lidar_frame_unavailable: {scan.header.frame_id}'
            px, py = scan_points(scan, laser)
            xs.append(px)
            ys.append(py)
            age = max(age, stamp_age)
        robot = self._current_robot_pose()
        odometry = self._execution_odometry
        if robot is None:
            return 'robot_pose_unavailable'
        if odometry is None or now_s - self._execution_odom_received_s > 2.0 * limit:
            return 'odometry_stale'
        t = odometry.twist.twist
        plan, sel = self.parking_plan, self.parking_plan.selected
        route = driven_poses(plan.selected_path, sel, robot)
        return clearance_reason(
            (np.concatenate(xs), np.concatenate(ys)), self.planner_config.footprint,
            (t.linear.x, t.linear.y, t.angular.z), age, robot, route,
            (plan.target.x, plan.target.y), plan.keepout_radius_m - self.final_xy_tolerance_m,
            self.safety_monitor.limits, self.clearance_limits)

    def _send_return_goal(self, ok):
        if not ok:
            # Returning with the tighter parking tolerance is slower, not unsafe.
            self.get_logger().warning('Shared goal tolerance not restored before return')
        start = self.start_pose.pose
        self._send_navigation_goal(
            (start.position.x, start.position.y, _yaw(start.orientation)),
            self.return_navigation_timeout_s)

    def _cancel_navigation_goal(self):
        """Request cancellation; only the goal's terminal result releases it, within a bounded wait."""
        self._nav_generation += 1   # ignore a late goal-checker response
        self._nav_deadline_s = self._nav_pending_tolerance = None
        if self._nav_goal_generation is None or self._nav_canceling:
            return
        self._nav_canceling = True
        self._nav_cancel_deadline_s = self._monotonic() + self.execution.limits.cancellation_timeout_s
        self._request_navigation_cancel()   # deferred to acceptance when no handle exists yet

    def _request_navigation_cancel(self):
        handle, generation = self._nav_handle, self._nav_goal_generation
        if handle is None:
            return
        try:
            handle.cancel_goal_async().add_done_callback(
                lambda f: self._navigation_cancel_response(f, generation))
        except Exception as exc:
            self.get_logger().error(f'NavigateToPose cancel request failed: {exc}; '
                                    'awaiting the terminal result')

    def _navigation_cancel_response(self, future, generation):
        with self._lock:
            if generation != self._nav_goal_generation:
                return
            try:
                accepted = bool(future.result().goals_canceling)
            except Exception as exc:
                accepted, note = False, f'cancel response unavailable: {exc}'
            else:
                # A rejection usually means the goal is already finishing.
                note = 'cancel accepted' if accepted else 'cancel rejected'
            # Neither answer releases ownership: only the goal's terminal result does.
            log = self.get_logger().info if accepted else self.get_logger().warning
            log(f'NavigateToPose {note}; awaiting the terminal result')

    def _navigation_ownership_unresolved(self, detail):
        """Fail explicitly while keeping the handle, so a late result is still recognised."""
        self._nav_cancel_deadline_s = self._nav_deadline_s = None
        self._return_after_stop = False
        self._request_navigation_cancel()   # one final request; the first may have been lost
        self.get_logger().error(f'Navigation goal ownership unresolved: {detail}')
        self.last_issue = MissionIssue(FailureClass.TERMINAL, 'navigation_cancel_unresolved', detail)
        if self.state == MissionState.RETURNING_TO_START:
            self._transition(MissionState.MISSION_FAILED, detail)
        else:
            self._publish_state(detail, event='navigation_ownership')

    def _stop_parking_navigation(self):
        """Cancel the owned goal and restore the shared goal checker (cancel/reset/shutdown)."""
        self._cancel_navigation_goal()
        if self._goal_checker_set_client.service_is_ready():
            self._set_goal_tolerance(self._shared_goal_tolerance, lambda ok: None)

    def _final_pose_reason(self):
        """Recheck the final footprint and G1 keep-out against the latest map before driving.

        G1 is not re-observed while driving, so the localized target keeps its
        planning timestamp; map and robot pose must still be fresh.
        """
        plan = self.parking_plan
        sel = plan.selected
        try:
            grid = None if self.latest_map is None else GridSnapshot.from_msg(self.latest_map)
        except ValueError as exc:
            return f'map_invalid: {exc}'
        reasons = validate_inputs(plan.target, grid, self._current_robot_pose(with_stamp=True),
                                  now_ns=self.get_clock().now().nanoseconds,
                                  map_frame=self.map_frame, max_target_age_s=math.inf,
                                  max_map_age_s=self.parking_max_map_age_s,
                                  max_pose_age_s=self.parking_max_pose_age_s,
                                  clock_skew_s=self.parking_max_pose_age_s)
        if reasons:
            return ', '.join(reasons)
        polygon = rotated_footprint_vertices(self.planner_config.footprint,
                                             sel.base_x, sel.base_y, sel.yaw)
        reason = footprint_rejection_reason(polygon, grid, self.planner_config.occupied_threshold)
        if reason:
            return f'footprint_{reason}'
        if points_to_polygon_distance([plan.target.x], [plan.target.y], polygon)[0] \
                < plan.keepout_radius_m:
            return 'g1_keepout'
        return None

    def _record_arrival(self, label, goal):
        robot = self._current_robot_pose()
        if robot is None:
            return
        xy = math.hypot(robot[0] - goal[0], robot[1] - goal[1])
        yaw = abs(math.atan2(math.sin(robot[2] - goal[2]), math.cos(robot[2] - goal[2])))
        self._arrival_errors[label] = {'xy_m': round(xy, 3), 'yaw_rad': round(yaw, 3)}
        self.get_logger().info(f'Arrived at {label}: error xy={xy:.3f} m, yaw={yaw:.3f} rad')

    # G1-centred mapping: the shared explorer, focused on G1 through explore/focus.

    def _g1_coverage(self):
        """Unknown cells of the donut and unknown share of the footprint area, on the SLAM map.

        The donut uses the planner's own cell rule (cell centre within the reach
        band), so it matches the plan's ``donut_unknown`` and the orange P2 rays.
        """
        position = self.target_position
        if position is None or self.latest_map is None:
            return None
        try:
            grid = GridSnapshot.from_msg(self.latest_map)
        except ValueError:
            return None
        rows, cols = np.mgrid[0:grid.height, 0:grid.width]
        x, y = grid.cell_centers(cols, rows)
        d = np.hypot(x - position.map_x_m, y - position.map_y_m)
        donut = (d >= self.planner_config.reach_min_m) & (d <= self.planner_config.reach_max_m)
        area = (d > self.planner_config.reach_max_m) & (d <= self.g1_coverage_radius_m)
        if not donut.any():
            return None
        unknown = grid.data < 0
        return {
            'donut_cells': int(np.count_nonzero(donut)),
            'donut_unknown_cells': int(np.count_nonzero(unknown & donut)),
            'footprint_area_unknown_fraction': round(
                float(np.count_nonzero(unknown & area)) / max(1, int(np.count_nonzero(area))), 4),
        }

    def _maybe_explore_around_g1(self):
        """Resume the explorer around G1 unless every donut cell is already known."""
        if not self.g1_exploration_enabled or self._g1_explored:
            return False
        coverage = self._g1_coverage()
        if coverage is None:
            return False
        if coverage['donut_unknown_cells'] == 0:
            self.g1_exploration_result = {'outcome': 'not_needed', 'reason': 'donut_already_mapped',
                                          'duration_s': 0.0, 'before': coverage, 'after': coverage}
            self.get_logger().info(f'Donut already mapped; no G1 exploration: {coverage}')
            return False
        if not self.resume_client.service_is_ready():
            self.get_logger().warning(
                f'Donut has {coverage["donut_unknown_cells"]} unknown cells, but no explorer '
                'resume service; planning with the current map')
            return False
        self._g1_explored = True
        self._g1_explorer_resumed = self._g1_frontiers_exhausted = False
        self._observation_pose = self._current_robot_pose()
        self._g1_exploration_started_s = self._monotonic()
        self._g1_exploration_deadline_s = self._g1_exploration_started_s + self.g1_exploration_timeout_s
        self.g1_exploration_result = {'outcome': 'running', 'reason': None, 'before': coverage}
        focus = PointStamped()
        focus.header.frame_id = self.map_frame
        focus.header.stamp = self.get_clock().now().to_msg()
        focus.point.x, focus.point.y = self.target_position.map_x_m, self.target_position.map_y_m
        self.focus_publisher.publish(focus)
        self._transition(MissionState.EXPLORING_AROUND_G1,
                         f'{coverage["donut_unknown_cells"]}/{coverage["donut_cells"]} donut cells '
                         'unknown; exploring around G1')
        self.resume_client.call_async(Trigger.Request()).add_done_callback(self._on_resume_response)
        return True

    def _on_resume_response(self, future):
        with self._lock:
            if self.state != MissionState.EXPLORING_AROUND_G1:
                return
            try:
                response = future.result()
                ok, message = response.success, response.message
            except Exception as exc:
                ok, message = False, str(exc)
            if not ok:
                self._finish_g1_exploration('partial', 'explorer_not_resumed', message)
                return
            self._g1_explorer_resumed = True

    def _on_exploration_status(self, msg: String) -> None:
        """The explorer has no reachable frontier left in the focus region around G1."""
        with self._lock:
            if (msg.data == 'focus_exploration_complete' and self._g1_explorer_resumed
                    and self.state == MissionState.EXPLORING_AROUND_G1):
                self._g1_frontiers_exhausted = True

    def _poll_g1_exploration(self, now_s):
        coverage = self._g1_coverage()
        robot = self._current_robot_pose()
        if robot is not None and self.target_position is not None:
            polygon = rotated_footprint_vertices(self.planner_config.footprint, *robot)
            target = TargetReference(self.target_position.map_x_m, self.target_position.map_y_m,
                                     self.map_frame, 0, self.target_position.reference,
                                     self.target_position.is_foot_midpoint, '')
            protected = keepout_radius_m(target, self.planner_config) + self.g1_protected_margin_m
            gap = points_to_polygon_distance([target.x], [target.y], polygon)[0]
            if gap < protected:
                self._finish_g1_exploration(
                    'partial', 'protected_region',
                    f'footprint {gap:.2f} m from G1 inside the protected {protected:.2f} m')
                return
        if coverage is not None and coverage['donut_unknown_cells'] == 0:
            self._finish_g1_exploration('complete', 'donut_mapped', 'every donut cell is known')
        elif self._g1_frontiers_exhausted:
            # Remaining unknown donut cells are not reachable by the explorer (occluded or
            # outside its goal annulus): plan with them; candidates there stay rejected.
            self._finish_g1_exploration('partial', 'no_reachable_frontiers',
                                        'explorer found no frontier left around G1')
        elif now_s >= self._g1_exploration_deadline_s:
            self._finish_g1_exploration('partial', 'timeout',
                                        f'{self.g1_exploration_timeout_s:.0f} s elapsed')

    def _finish_g1_exploration(self, outcome, reason, detail):
        self.focus_publisher.publish(PointStamped())   # empty frame clears the focus
        self._g1_exploration_deadline_s = None
        coverage = self._g1_coverage()
        result = dict(self.g1_exploration_result or {})
        result.update(outcome=outcome, reason=reason, after=coverage, duration_s=round(
            self._monotonic() - (self._g1_exploration_started_s or self._monotonic()), 2))
        self.g1_exploration_result = result
        summary = (f'{outcome} ({reason}): {detail}; donut unknown '
                   f'{"?" if coverage is None else coverage["donut_unknown_cells"]}'
                   f'/{"?" if coverage is None else coverage["donut_cells"]}')
        self.get_logger().info(f'G1 exploration finished: {summary}')
        if self._transition(MissionState.STOPPING_EXPLORATION, f'{summary}; pausing explorer'):
            self._begin_exploration_handoff()

    def _stop_g1_exploration(self):
        self.focus_publisher.publish(PointStamped())
        try:
            self.pause_client.call_async(Trigger.Request())
        except Exception as exc:
            self.get_logger().error(f'Explorer pause on cancel unresolved: {exc}')

    def _plan_summary(self):
        plan = self.parking_plan
        if plan is None:
            return None
        summary = {'ok': plan.ok, 'failure_reason': plan.failure_reason,
                   'keepout_radius_m': round(plan.keepout_radius_m, 3),
                   'target': [round(plan.target.x, 3), round(plan.target.y, 3)],
                   'target_reference': plan.target.reference, 'counts': plan.counts()}
        if plan.selected is not None:
            sel = plan.selected
            summary['selected'] = {
                'base': [round(sel.base_x, 3), round(sel.base_y, 3), round(sel.yaw, 4)],
                'task': [round(sel.task_x, 3), round(sel.task_y, 3)],
                'reach_m': round(sel.reach_m, 3), 'clearance_m': round(sel.clearance_m, 3),
                'preference': sel.preference_label, 'path_length_m': sel.path_length_m}
        return summary

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self.latest_map = msg

    def _on_detections(self, msg: TargetDetections) -> None:
        self.process_detection(msg)

    def _on_measurements(self, msg: TargetMeasurements) -> None:
        self.process_measurement(msg)

    def _on_guard_status(self, msg: String) -> None:
        self.process_guard_status(msg)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load_nav2_footprint(nav2_params_path: str) -> list[tuple[float, float]]:
    """The global costmap footprint, as the original mission loaded it (padding not applied)."""

    with open(nav2_params_path, encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    footprint = config['global_costmap']['global_costmap']['ros__parameters']['footprint']
    points = [(float(p[0]), float(p[1])) for p in ast.literal_eval(str(footprint))]
    if len(points) < 3 or not all(math.isfinite(v) for p in points for v in p):
        raise ValueError('Nav2 footprint must contain at least 3 finite points')
    return points


def main(args=None) -> None:
    # Python's SIGINT exception reaches finally before ROS context destruction,
    # allowing the independent TF executor to stop without racing shutdown.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
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
