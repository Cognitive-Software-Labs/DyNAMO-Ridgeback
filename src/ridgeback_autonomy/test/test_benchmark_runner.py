from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.benchmarking.alignment import (
    EventPreview,
    MeasurementEvent,
    ensure_measurement_event,
    find_exact_preview_match,
    find_nearest_preview_match,
    update_measurement_event,
)
from ridgeback_autonomy.benchmarking.estimators import parse_estimators
from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import extract_json_payload
from ridgeback_autonomy.benchmarking.reduction import (
    choose_representative_event,
    compute_trial_medians,
    usable_aligned_events,
)
from ridgeback_autonomy.benchmarking.rendering import BenchmarkCollageRenderer
from ridgeback_autonomy.benchmarking.summary import build_summary_rows
from ridgeback_autonomy.msg import G1Measurements


def test_parse_estimators_uses_canonical_order_and_depth_anything_name() -> None:
    assert parse_estimators('') == (
        'rgb', 'sensor_depth', 'depth_anything', 'pointcloud', 'lidar',
        'path_a', 'path_b',
    )
    assert parse_estimators('pointcloud,rgb') == ('rgb', 'pointcloud')
    assert parse_estimators('path_b,rgb,path_a') == ('rgb', 'path_a', 'path_b')
    with pytest.raises(ValueError, match='depth_anything'):
        parse_estimators('mono_depth')


def test_extract_json_payload_accepts_multiple_gz_json_messages() -> None:
    payload = extract_json_payload(
        'noise before\n'
        '{"pose":[{"name":"first","position":{"x":1}}]}\n'
        '{"pose":[{"name":"second","position":{"x":2}}]}\n'
    )

    assert payload == {'pose': [{'name': 'first', 'position': {'x': 1}}]}


def test_alignment_updates_public_depth_anything_value_from_message_field() -> None:
    msg = G1Measurements()
    msg.header.frame_id = 'camera'
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34
    msg.detected = True
    msg.count = 1
    msg.image_width = 640
    msg.image_height = 480
    msg.bbox_xyxy = [1.0, 2.0, 30.0, 40.0]
    msg.rgb_distance_m = [4.5]
    msg.sensor_depth_distance_m = [4.0]
    msg.mono_depth_distance_m = [3.8]
    msg.pointcloud_distance_m = [float('nan')]
    msg.lidar_distance_m = [3.9]

    events = {}
    event = ensure_measurement_event(events, msg)
    update_measurement_event(
        event,
        msg,
        {'rgb', 'sensor_depth', 'depth_anything', 'pointcloud', 'lidar'},
    )

    assert event.detected is True
    assert event.count == 1
    assert event.bboxes == ((1, 2, 30, 40),)
    assert event.estimates['rgb'] == pytest.approx(4.5, rel=1e-6)
    assert event.estimates['sensor_depth'] == pytest.approx(4.0, rel=1e-6)
    assert event.estimates['depth_anything'] == pytest.approx(3.8, rel=1e-6)
    assert event.estimates['pointcloud'] is None
    assert event.estimates['lidar'] == pytest.approx(3.9, rel=1e-6)


def test_mask_source_message_merges_into_the_same_aligned_event() -> None:
    def make_message() -> G1Measurements:
        msg = G1Measurements()
        msg.header.frame_id = 'camera'
        msg.header.stamp.sec = 12
        msg.header.stamp.nanosec = 34
        msg.detected = True
        msg.count = 1
        msg.image_width = 640
        msg.image_height = 480
        msg.bbox_xyxy = [1.0, 2.0, 30.0, 40.0]
        return msg

    camera_msg = make_message()
    camera_msg.rgb_distance_m = [4.5]
    mask_msg = make_message()
    mask_msg.path_a_distance_m = [4.1]
    mask_msg.path_b_distance_m = [4.2]

    events = {}
    camera_event = ensure_measurement_event(events, camera_msg)
    update_measurement_event(camera_event, camera_msg, {'rgb'})
    mask_event = ensure_measurement_event(events, mask_msg)
    update_measurement_event(mask_event, mask_msg, {'path_a', 'path_b'})

    # Identical alignment key -> one merged event holding all three estimates.
    assert len(events) == 1
    assert mask_event is camera_event
    assert camera_event.estimates['rgb'] == pytest.approx(4.5, rel=1e-6)
    assert camera_event.estimates['path_a'] == pytest.approx(4.1, rel=1e-6)
    assert camera_event.estimates['path_b'] == pytest.approx(4.2, rel=1e-6)


def test_find_exact_preview_match_uses_only_exact_stamp() -> None:
    preview = np.full((4, 4, 3), 127, dtype=np.uint8)
    buffer = {100_000_000: preview}

    matched = find_exact_preview_match(buffer, 100_000_000)
    unmatched = find_exact_preview_match(buffer, 150_000_000)

    assert matched.image_bgr is preview
    assert matched.matched_stamp_ns == 100_000_000
    assert matched.matched_delta_ms == pytest.approx(0.0, rel=1e-6)
    assert unmatched.image_bgr is None
    assert unmatched.matched_stamp_ns is None
    assert unmatched.nearest_stamp_ns == 100_000_000
    assert unmatched.nearest_delta_ms == pytest.approx(50.0, rel=1e-6)


def test_find_nearest_preview_match_prefers_smallest_delta_within_tolerance() -> None:
    far_preview = np.full((2, 2, 3), 20, dtype=np.uint8)
    near_preview = np.full((2, 2, 3), 200, dtype=np.uint8)
    buffer = {
        100_000_000: far_preview,
        230_000_000: near_preview,
    }

    match = find_nearest_preview_match(buffer, 200_000_000, tolerance_ns=250_000_000)

    assert match.image_bgr is near_preview
    assert match.matched_stamp_ns == 230_000_000
    assert match.matched_delta_ms == pytest.approx(30.0, rel=1e-6)
    assert match.nearest_stamp_ns == 230_000_000


def test_find_nearest_preview_match_returns_unmatched_when_outside_tolerance() -> None:
    preview = np.full((2, 2, 3), 200, dtype=np.uint8)
    buffer = {600_000_000: preview}

    match = find_nearest_preview_match(buffer, 0, tolerance_ns=250_000_000)

    assert match.image_bgr is None
    assert match.matched_stamp_ns is None
    assert match.nearest_stamp_ns == 600_000_000
    assert match.nearest_delta_ms == pytest.approx(600.0, rel=1e-6)


def test_usable_aligned_events_require_the_full_selected_estimator_set() -> None:
    events = {
        'rgb_only': MeasurementEvent(
            key=('rgb_only',),
            stamp_ns=100,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 2.0},
            preview=EventPreview(),
        ),
        'rgb_lidar': MeasurementEvent(
            key=('rgb_lidar',),
            stamp_ns=200,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 2.1, 'lidar': 2.0},
            preview=EventPreview(),
        ),
    }

    usable = usable_aligned_events(events, ('rgb', 'lidar'))

    assert [event.key for event in usable] == [('rgb_lidar',)]


def test_representative_event_uses_minimum_total_deviation_then_timestamp() -> None:
    usable = [
        MeasurementEvent(
            key=('a',),
            stamp_ns=200,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 2.2, 'lidar': 2.0},
            preview=EventPreview(),
        ),
        MeasurementEvent(
            key=('b',),
            stamp_ns=100,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 2.0, 'lidar': 1.8},
            preview=EventPreview(),
        ),
        MeasurementEvent(
            key=('c',),
            stamp_ns=300,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 5.0, 'lidar': 5.0},
            preview=EventPreview(),
        ),
    ]

    medians = compute_trial_medians(usable, ('rgb', 'lidar'))
    representative = choose_representative_event(usable, ('rgb', 'lidar'), medians)

    assert medians == {'rgb': 2.2, 'lidar': 2.0}
    assert representative.key == ('a',)


def test_representative_event_prefers_preview_complete_candidate() -> None:
    preview = np.full((8, 8, 3), 180, dtype=np.uint8)
    usable = [
        MeasurementEvent(
            key=('missing_sensor_depth',),
            stamp_ns=100,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 2.0, 'sensor_depth': 2.0},
            preview=EventPreview(
                color_bgr=preview,
                color_stamp_ns=100,
                color_delta_ms=0.0,
            ),
        ),
        MeasurementEvent(
            key=('complete',),
            stamp_ns=200,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 2.1, 'sensor_depth': 2.1},
            preview=EventPreview(
                color_bgr=preview,
                color_stamp_ns=200,
                color_delta_ms=0.0,
                sensor_depth_bgr=preview,
                sensor_depth_stamp_ns=220,
                sensor_depth_delta_ms=20.0,
            ),
        ),
    ]

    medians = compute_trial_medians(usable, ('rgb', 'sensor_depth'))
    representative = choose_representative_event(usable, ('rgb', 'sensor_depth'), medians)

    assert representative.key == ('complete',)


def test_representative_event_falls_back_to_numeric_when_no_preview_complete_candidate() -> None:
    preview = np.full((8, 8, 3), 180, dtype=np.uint8)
    usable = [
        MeasurementEvent(
            key=('best_numeric',),
            stamp_ns=100,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 2.0, 'sensor_depth': 2.0},
            preview=EventPreview(
                color_bgr=preview,
                color_stamp_ns=100,
                color_delta_ms=0.0,
            ),
        ),
        MeasurementEvent(
            key=('worse_numeric',),
            stamp_ns=200,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'rgb': 3.0, 'sensor_depth': 3.0},
            preview=EventPreview(
                color_bgr=preview,
                color_stamp_ns=200,
                color_delta_ms=0.0,
            ),
        ),
    ]

    medians = compute_trial_medians(usable, ('rgb', 'sensor_depth'))
    representative = choose_representative_event(usable, ('rgb', 'sensor_depth'), medians)

    assert representative.key == ('best_numeric',)


def test_render_missing_preview_context_is_not_a_silent_black_panel() -> None:
    renderer = BenchmarkCollageRenderer(depth_max_meters=10.0)
    event = MeasurementEvent(
        key=('missing',),
        stamp_ns=123_000_000,
        detected=True,
        count=1,
        bboxes=((10, 20, 40, 60),),
        image_width=320,
        image_height=180,
        estimates={'sensor_depth': 2.5},
        preview=EventPreview(
            sensor_depth_nearest_stamp_ns=150_000_000,
            sensor_depth_nearest_delta_ms=27.0,
        ),
    )

    context = renderer.panel_context_for_estimator('sensor_depth', event)
    lines = renderer.build_panel_lines(
        context,
        'trial_001',
        frame_value=2.5,
        trial_median_m=2.6,
        true_distance_m=2.7,
        event_stamp_ns=event.stamp_ns,
    )

    assert context.preview_available is False
    assert context.expected_source_label == 'Sensor Depth'
    assert context.panel.mean() > 0.0
    assert 'Preview Missing' in lines
    assert 'Expected: Sensor Depth' in lines
    assert 'Nearest: 27.0ms' in lines


def test_render_pointcloud_and_lidar_panels_are_labeled_as_rgb_debug_views() -> None:
    renderer = BenchmarkCollageRenderer(depth_max_meters=10.0)
    color_panel = np.full((120, 160, 3), 90, dtype=np.uint8)
    event = MeasurementEvent(
        key=('debug_view',),
        stamp_ns=500_000_000,
        detected=True,
        count=1,
        bboxes=((10, 20, 40, 60),),
        image_width=160,
        image_height=120,
        estimates={'pointcloud': 3.0, 'lidar': 3.1},
        preview=EventPreview(
            color_bgr=color_panel,
            color_stamp_ns=500_000_000,
            color_delta_ms=0.0,
        ),
    )

    pointcloud_context = renderer.panel_context_for_estimator('pointcloud', event)
    lidar_context = renderer.panel_context_for_estimator('lidar', event)
    lines = renderer.build_panel_lines(
        pointcloud_context,
        'trial_002',
        frame_value=3.0,
        trial_median_m=3.0,
        true_distance_m=3.2,
        event_stamp_ns=event.stamp_ns,
    )

    assert pointcloud_context.preview_available is True
    assert pointcloud_context.source_label == 'RGB Debug View'
    assert lidar_context.source_label == 'RGB Debug View'
    assert 'Source: RGB Debug View' in lines
    assert 'Delta: 0.0ms' in lines


def test_build_summary_rows_aggregates_trial_level_estimator_rows() -> None:
    summary_rows = build_summary_rows({
        'rgb': [
            {'abs_error_m': 0.2, 'rel_error': 0.1},
            {'abs_error_m': 0.4, 'rel_error': 0.2},
        ],
        'lidar': [
            {'abs_error_m': 0.1, 'rel_error': 0.05},
        ],
    })

    rgb_row = next(row for row in summary_rows if row['estimator'] == 'rgb')
    lidar_row = next(row for row in summary_rows if row['estimator'] == 'lidar')

    assert rgb_row['trial_count'] == 2
    assert rgb_row['mean_abs_error_m'] == pytest.approx(0.3, rel=1e-6)
    assert rgb_row['median_abs_error_m'] == pytest.approx(0.3, rel=1e-6)
    assert rgb_row['mean_rel_error'] == pytest.approx(0.15, rel=1e-6)

    assert lidar_row['trial_count'] == 1
    assert lidar_row['p95_abs_error_m'] == pytest.approx(0.1, rel=1e-6)
