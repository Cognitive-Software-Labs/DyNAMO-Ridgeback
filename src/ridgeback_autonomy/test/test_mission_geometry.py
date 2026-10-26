from __future__ import annotations

import math

import pytest
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

from ridgeback_autonomy.mission.mission_manager_node import (
    G1BodyPositionEstimate,
    MissionManagerNode,
    MissionState,
)
from builtin_interfaces.msg import Time
from ridgeback_autonomy.mission.parking_geometry import (
    base_pose_from_task_point,
    is_reach_distance_valid,
    planar_path_length,
    preparking_pose,
    rotated_footprint_vertices,
    task_point_from_base_pose,
)


FOOTPRINT = [
    (0.480, 0.3965),
    (0.480, -0.3965),
    (-0.480, -0.3965),
    (-0.480, 0.3965),
]


def _node_for_geometry() -> MissionManagerNode:
    node = object.__new__(MissionManagerNode)
    node.map_frame = 'map'
    node.footprint_points = FOOTPRINT
    node.footprint_min_x_m = -0.480
    node.footprint_max_x_m = 0.480
    node.footprint_min_y_m = -0.3965
    node.footprint_max_y_m = 0.3965
    node.occupied_threshold = 50
    node.parking_path_sample_spacing_m = 0.10
    return node


def _grid(fill: int = 0) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = 0.10
    grid.info.width = 40
    grid.info.height = 40
    grid.info.origin.position.x = -2.0
    grid.info.origin.position.y = -2.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [fill] * (grid.info.width * grid.info.height)
    return grid


def _set_cell(grid: OccupancyGrid, x_m: float, y_m: float, value: int) -> None:
    col = int((x_m - grid.info.origin.position.x) / grid.info.resolution)
    row = int((y_m - grid.info.origin.position.y) / grid.info.resolution)
    grid.data[row * grid.info.width + col] = value


def _path(points: list[tuple[float, float, float]]) -> Path:
    path = Path()
    path.header.frame_id = 'map'
    for x_m, y_m, yaw_rad in points:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x_m
        pose.pose.position.y = y_m
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        path.poses.append(pose)
    return path


@pytest.mark.parametrize(
    ('task_x', 'task_y'),
    [(0.45, 0.0), (-0.45, 0.0), (0.0, 0.45), (0.0, -0.45)],
)
def test_task_point_base_conversion_and_rear_facing_yaw(task_x, task_y) -> None:
    base_x, base_y, yaw = base_pose_from_task_point(
        task_x,
        task_y,
        0.0,
        0.0,
        -0.280,
        0.0,
    )
    recovered_x, recovered_y = task_point_from_base_pose(
        base_x,
        base_y,
        yaw,
        -0.280,
        0.0,
    )

    assert recovered_x == pytest.approx(task_x)
    assert recovered_y == pytest.approx(task_y)
    front_x = math.cos(yaw)
    front_y = math.sin(yaw)
    toward_g1_x = -task_x
    toward_g1_y = -task_y
    assert front_x * toward_g1_x + front_y * toward_g1_y < 0.0


def test_rotated_footprint_and_preparking_geometry() -> None:
    vertices = rotated_footprint_vertices(FOOTPRINT, 1.0, 2.0, math.pi / 2.0)
    assert vertices[0] == pytest.approx((1.0 - 0.3965, 2.0 + 0.480))
    assert preparking_pose(1.0, 2.0, math.pi / 2.0, 0.50) == pytest.approx(
        (1.0, 2.5, math.pi / 2.0)
    )


def test_reach_boundaries_and_path_length() -> None:
    assert is_reach_distance_valid(0.30, 0.30, 0.60)
    assert is_reach_distance_valid(0.60, 0.30, 0.60)
    assert not is_reach_distance_valid(0.299, 0.30, 0.60)
    assert not is_reach_distance_valid(0.601, 0.30, 0.60)
    assert planar_path_length([(0.0, 0.0), (3.0, 4.0), (3.0, 5.0)]) == 6.0


@pytest.mark.parametrize(
    ('cell_value', 'expected_reason'),
    [(100, 'occupied'), (-1, 'unknown')],
)
def test_rotated_footprint_rejects_unsafe_cells(cell_value, expected_reason) -> None:
    node = _node_for_geometry()
    grid = _grid()
    _set_cell(grid, 0.0, 0.0, cell_value)
    polygon = node._rotated_footprint_vertices(0.0, 0.0, math.pi / 4.0)
    assert node._rotated_footprint_rejection_reason(polygon, grid) == expected_reason


def test_rotated_footprint_rejects_partial_map_exit() -> None:
    node = _node_for_geometry()
    grid = _grid()
    polygon = node._rotated_footprint_vertices(1.90, 0.0, 0.0)
    assert node._rotated_footprint_rejection_reason(polygon, grid) == 'out_of_bounds'


def test_clearance_uses_complete_rotated_footprint() -> None:
    node = _node_for_geometry()
    grid = _grid()
    obstacle = Point(x=1.0, y=0.0, z=0.0)
    clearance = node._rotated_footprint_clearance_m(
        (0.0, 0.0, 0.0),
        grid,
        [obstacle],
    )
    assert 0.44 <= clearance <= 0.46


def test_parking_path_rejects_unknown_crossing_with_full_footprint() -> None:
    node = _node_for_geometry()
    grid = _grid()
    _set_cell(grid, 0.0, 0.35, -1)
    path = _path([(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    assert node._path_footprint_rejection_reason(path, grid) == 'unsafe_path_unknown'


def test_safe_parking_path_is_accepted() -> None:
    node = _node_for_geometry()
    path = _path([(-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)])
    assert node._path_footprint_rejection_reason(path, _grid()) is None


def test_approach_relative_side_is_preferred_without_fake_g1_yaw() -> None:
    node = _node_for_geometry()
    node.g1_map_x_m = 0.0
    node.g1_map_y_m = 0.0
    node.g1_commit_robot_pose = (-2.0, 0.0, 0.0)
    node.start_pose = None

    side = node._parking_candidate_preference(0.0, 0.45)
    front = node._parking_candidate_preference(-0.45, 0.0)
    back = node._parking_candidate_preference(0.45, 0.0)
    assert side[0] == 0
    assert front[0] == 1
    assert back[0] == 2
    assert side[2] == 'approach_relative'


def test_candidate_visualization_tracking_is_passive_and_stable() -> None:
    node = _node_for_geometry()
    node.parking_candidate_decisions = {}
    candidates = [
        (1.0, 2.0, 0.25),
        (2.0, 3.0, -0.50),
    ]
    original_candidates = list(candidates)

    node._register_parking_candidate_decision(
        candidates[0],
        status='not_evaluated',
        reason='awaiting_nav2_feasibility',
    )
    node._register_parking_candidate_decision(
        candidates[1],
        status='collision',
        reason='footprint_occupied',
    )
    node._update_parking_candidate_decision(
        candidates[0],
        status='valid',
        reason='nav2_reachable',
        path_length_m=4.2,
    )

    assert candidates == original_candidates
    assert [
        decision.marker_id
        for decision in node.parking_candidate_decisions.values()
    ] == [0, 1]
    assert node.parking_candidate_decisions[candidates[0]].status == 'valid'
    assert node.parking_candidate_decisions[candidates[0]].path_length_m == 4.2
    assert node.parking_candidate_decisions[candidates[1]].status == 'collision'


@pytest.mark.parametrize(
    ('reason', 'expected_status'),
    [
        ('occupied', 'collision'),
        ('unsafe_path_occupied', 'collision'),
        ('unknown', 'rejected'),
        ('out_of_bounds', 'rejected'),
        ('planning_failure', 'rejected'),
    ],
)
def test_candidate_rejection_color_classification(reason, expected_status) -> None:
    assert (
        MissionManagerNode._parking_candidate_status_for_rejection(reason)
        == expected_status
    )


def test_candidate_visualization_does_not_change_selection_order() -> None:
    node = _node_for_geometry()
    node.state = MissionState.NAV2_FEASIBILITY_CHECKED
    node.g1_committed = True
    node.g1_map_x_m = 0.0
    node.g1_map_y_m = 0.0
    node.g1_commit_robot_pose = (-2.0, 0.0, 0.0)
    node.start_pose = None
    node.task_point_x_base_m = -0.280
    node.task_point_y_base_m = 0.0
    node.reach_min_m = 0.30
    node.reach_max_m = 0.60
    node.latest_map = _grid()

    higher_clearance = node._base_pose_from_task_point(0.0, 0.45)
    shorter_path = node._base_pose_from_task_point(0.0, -0.45)
    node.nav2_reachable_candidates = [higher_clearance, shorter_path]
    node.nav2_reachable_path_lengths = {
        higher_clearance: 5.0,
        shorter_path: 2.0,
    }
    node.parking_candidate_decisions = {}
    for candidate in node.nav2_reachable_candidates:
        node._register_parking_candidate_decision(
            candidate,
            status='valid',
            reason='nav2_reachable',
        )

    clearances = {
        higher_clearance: 0.50,
        shorter_path: 0.25,
    }
    node._unsafe_grid_cell_centers = lambda _grid_msg: []
    node._rotated_footprint_clearance_m = (
        lambda candidate, _grid_msg, _unsafe: clearances[candidate]
    )
    node._timing_start = lambda _stage: None
    node._timing_stop = lambda _stage: None
    node._publish_parking_candidate_decisions = lambda: None
    node._log_parking_candidate_decisions = lambda _stage: None
    node._publish_thesis_parking_markers = lambda: None
    node._prepare_preparking_navigation = lambda: None

    class Logger:
        def info(self, _message):
            pass

    node.get_logger = lambda: Logger()

    node._select_final_parking_pose()

    assert node.state == MissionState.PARKING_POSE_SELECTED
    assert (
        node.selected_parking_pose['x'],
        node.selected_parking_pose['y'],
        node.selected_parking_pose['yaw'],
    ) == pytest.approx(higher_clearance)
    assert node.parking_candidate_decisions[higher_clearance].status == 'selected'
    assert node.parking_candidate_decisions[shorter_path].status == 'valid'


def test_saved_start_pose_is_never_overwritten() -> None:
    node = _node_for_geometry()
    saved = PoseStamped()
    saved.pose.position.x = 1.25
    node.start_pose = saved
    node.tf_buffer = None
    node._try_save_start_pose()
    assert node.start_pose is saved
    assert node.start_pose.pose.position.x == 1.25


def test_committed_mission_cannot_resume_exploration() -> None:
    node = _node_for_geometry()
    node.state = MissionState.LOCALIZE_G1
    node.g1_committed = True
    node.exploration_resume_retry_timer = None
    node.exploration_resume_request_in_flight = False

    class UnexpectedClient:
        def service_is_ready(self):
            raise AssertionError('committed mission attempted to resume exploration')

    node.exploration_stop_client = UnexpectedClient()
    node._request_exploration_resume()


def test_right_edge_detection_requires_in_place_centering() -> None:
    node = _node_for_geometry()
    node.g1_body_bbox_edge_margin_px = 8
    node.g1_centering_angular_tolerance_rad = 0.10
    node._compute_bbox_horizontal_bearing_in_base = lambda _bbox: (0.8, -0.6)

    assert node._g1_bbox_needs_centering((520.0, 80.0, 640.0, 470.0), 640)


def test_committed_handoff_enters_centering_before_localization() -> None:
    node = _node_for_geometry()
    node.state = MissionState.STOPPING_EXPLORATION
    node.exploration_stop_accepted = True
    node.exploration_active = False
    node.exploration_stop_timer = None
    node.exploration_stop_reason = 'g1_commit'
    transitions = []
    node._timing_stop = lambda stage: transitions.append(stage)
    node._begin_g1_centering = lambda previous: transitions.append(previous)

    node._complete_exploration_handoff_if_ready()

    assert transitions == ['exploration_stop', MissionState.STOPPING_EXPLORATION]


def test_centered_detection_starts_fresh_stationary_generation() -> None:
    node = _node_for_geometry()
    node.state = MissionState.CENTER_G1
    node.g1_localization_generation = 4
    node.g1_centering_detection_timer = None
    calls = []
    node._cancel_g1_centering_detection_timeout = lambda: None
    node._clear_g1_foot_midpoint_samples = lambda: calls.append('clear')
    node._log_g1_localization_viewpoint = lambda: calls.append('viewpoint')
    node._enter_g1_localization = (
        lambda previous_state, start_timeout=True: calls.append(
            (previous_state, start_timeout)
        )
    )
    node.get_logger = lambda: type('Logger', (), {'info': lambda _self, _msg: None})()

    stamp = Time(sec=10, nanosec=20_000_000)
    node._start_stationary_g1_localization(stamp)

    assert calls == ['clear', 'viewpoint', (MissionState.CENTER_G1, True)]
    assert node.g1_localization_generation == 5
    assert node.g1_localization_min_stamp_s == pytest.approx(10.02)


def test_centering_failure_does_not_translate_when_disabled() -> None:
    node = _node_for_geometry()
    node.enable_translating_g1_observation = False
    node.g1_centering_detection_timer = None
    node.g1_centering_goal_generation = 1
    node.g1_centering_goal_in_flight = False
    calls = []
    node._cancel_g1_centering_detection_timeout = lambda: None
    node._return_to_start_from_current_state = lambda reason: calls.append(reason)
    node._start_translating_g1_observation_or_return = lambda: calls.append('translate')
    node.get_logger = lambda: type('Logger', (), {'warn': lambda _self, _msg: None})()

    node._handle_g1_centering_failure('no fresh G1 detection')

    assert calls == [
        'no fresh G1 detection; unable to localize G1 safely; returning to start'
    ]


def test_top_clipped_centered_bbox_can_pass_body_fallback_gate() -> None:
    node = _node_for_geometry()
    node.g1_body_bbox_edge_margin_px = 8
    node.g1_body_to_old_estimate_max_distance_m = 0.75
    node.g1_old_ground_estimate_x_map_m = 2.0
    node.g1_old_ground_estimate_y_map_m = 1.0
    node.g1_old_ground_estimate_x_base_m = None
    node.g1_old_ground_estimate_y_base_m = None

    class Logger:
        def info(self, _message):
            pass

    node.get_logger = lambda: Logger()
    estimate = G1BodyPositionEstimate(
        body_base=(2.0, 1.0, 1.1),
        body_map=(2.05, 1.02),
        valid_points=100,
        roi_xyxy=(250, 20, 390, 250),
        range_spread_m=0.2,
        height_spread_m=0.5,
        source='depth_body',
        stamp=Time(sec=10),
    )

    accepted, reason = node._g1_body_position_sample_is_valid(
        (200.0, 0.0, 440.0, 400.0),
        640,
        480,
        estimate,
        None,
    )

    assert accepted
    assert reason == 'accepted'


def test_five_body_samples_start_existing_parking_pipeline() -> None:
    node = _node_for_geometry()
    node.state = MissionState.LOCALIZE_G1
    node.g1_body_sample_count = 5
    node.g1_body_sample_max_jump_m = 0.75
    node.g1_body_position_samples = []
    node.g1_map_x_m = None
    node.g1_map_y_m = None
    node.g1_localization_timeout_timer = None
    node.g1_observation_trigger_timer = None
    node.parking_pipeline_start_s = None
    node.parking_pipeline_stage_starts = {}
    node.parking_pipeline_durations = {}
    transitions = []
    node._begin_g1_map_transform = lambda previous: transitions.append(previous)

    class Logger:
        def info(self, _message):
            pass

        def warn(self, _message):
            pass

    node.get_logger = lambda: Logger()
    for index in range(5):
        estimate = G1BodyPositionEstimate(
            body_base=(2.0, 0.1, 1.1),
            body_map=(2.0 + index * 0.01, 1.0),
            valid_points=100,
            roi_xyxy=(250, 100, 390, 280),
            range_spread_m=0.2,
            height_spread_m=0.5,
            source='depth_body',
            stamp=Time(sec=10 + index),
        )
        node._accept_g1_body_position_sample(estimate)

    assert node.g1_position_source == 'depth_body_projection'
    assert node.g1_map_x_m == pytest.approx(2.02)
    assert transitions == [MissionState.LOCALIZE_G1]


def test_no_reachable_candidate_fails_safely_without_exploration_resume() -> None:
    node = _node_for_geometry()
    node.state = MissionState.NAV2_FEASIBILITY_CHECKED
    node.g1_committed = True
    node.nav2_reachable_candidates = []
    node.parking_pipeline_start_s = None
    node.parking_pipeline_stage_starts = {}
    node.parking_pipeline_durations = {}
    node.exploration_stop_timer = None
    node.g1_observation_validation_wait_timer = None
    node.preparking_validation_wait_timer = None
    node.nav2_wait_timer = None
    node.navigate_wait_timer = None
    node.return_navigation_wait_timer = None
    node._cancel_g1_localization_timeout = lambda: None
    node._cancel_g1_observation_trigger = lambda: None

    class Logger:
        def info(self, _message):
            pass

        def error(self, _message):
            pass

    node.get_logger = lambda: Logger()

    node._select_final_parking_pose()

    assert node.state == MissionState.MISSION_FAILED
