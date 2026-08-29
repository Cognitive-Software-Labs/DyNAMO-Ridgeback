from __future__ import annotations

import os

import numpy as np
import pytest

from ridgeback_autonomy.benchmarking.alignment import (
    EventPreview,
    MeasurementEvent,
    ensure_measurement_event,
    find_exact_preview_match,
    update_measurement_event,
)
from ridgeback_autonomy.benchmarking.estimators import (
    MASK_GATE_DEFAULT,
    MASK_GATES,
    benchmark_display_name,
    benchmark_output_name,
    parse_estimators,
    parse_mask_gate,
)
from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
    extract_json_payload,
    ground_truth_point_message,
)
from ridgeback_autonomy.benchmarking.reduction import (
    choose_representative_event,
    compute_trial_medians,
    usable_aligned_events,
    union_usable_events,
    usable_events_by_estimator,
)
from ridgeback_autonomy.benchmarking.rendering import BenchmarkCollageRenderer
from ridgeback_autonomy.benchmarking.summary import build_summary_rows
from ridgeback_autonomy.common.models import Detection
from ridgeback_autonomy.msg import G1Measurements


def test_parse_estimators_uses_canonical_order() -> None:
    assert parse_estimators('') == (
        'pointcloud', 'projective_ranging', 'euclidean_reconstruction',
        'polar_profiling',
    )
    assert parse_estimators('projective_ranging,pointcloud') == (
        'pointcloud', 'projective_ranging',
    )
    assert parse_estimators('euclidean_reconstruction,pointcloud,projective_ranging') == (
        'pointcloud', 'projective_ranging', 'euclidean_reconstruction',
    )


def test_parse_estimators_rejects_a_removed_estimator() -> None:
    # A stale sweep YAML or shell history naming a deleted row must fail loudly
    # rather than silently resolving to a smaller set.
    for removed in ('rgb', 'sensor_depth', 'depth_anything', 'lidar', 'mono_depth'):
        with pytest.raises(ValueError, match='Unsupported estimator'):
            parse_estimators(removed)


def test_benchmark_output_name_is_prose_order_snake_case() -> None:
    # Filesystem-safe: prose order (gate, source, path) + isolation, snake_case.
    assert benchmark_output_name(
        'projective_ranging', 'stereoscopic', 'nearest_mode_histogram', 'height_crop_range_band'
    ) == 'box_gated_stereoscopic_projective_ranging_nearest_mode_histogram'
    assert benchmark_output_name(
        'euclidean_reconstruction', 'monocular', 'otsu', 'range_band'
    ) == 'box_gated_monocular_euclidean_reconstruction_range_band'


def test_benchmark_output_name_polar_profiling_folds_gate_only() -> None:
    # Polar profiling is mask-based but LiDAR-sourced: only the gate applies,
    # not the depth source or an isolation recipe.
    assert benchmark_output_name(
        'polar_profiling', 'stereoscopic', 'nearest_mode_histogram', 'range_band'
    ) == 'box_gated_polar_profiling'


def test_benchmark_output_name_leaves_non_mask_estimators_plain() -> None:
    assert benchmark_output_name(
        'pointcloud', 'stereoscopic', 'nearest_mode_histogram', 'height_crop_range_band'
    ) == 'pointcloud'


def test_benchmark_display_name_is_spaced_prose_without_isolation() -> None:
    assert benchmark_display_name(
        'projective_ranging', 'stereoscopic'
    ) == 'box-gated stereoscopic projective ranging'
    assert benchmark_display_name(
        'euclidean_reconstruction', 'monocular'
    ) == 'box-gated monocular euclidean reconstruction'
    # Polar profiling drops the source; non-mask rows use their fixed label.
    assert benchmark_display_name('polar_profiling', 'stereoscopic') == 'box-gated polar profiling'
    assert benchmark_display_name('pointcloud', 'stereoscopic') == 'Point Cloud'


def test_parse_mask_gate_validates_and_defaults() -> None:
    assert parse_mask_gate('box') == 'box'
    assert parse_mask_gate(' silhouette ') == 'silhouette'
    assert parse_mask_gate('') == MASK_GATE_DEFAULT
    assert parse_mask_gate(None) == MASK_GATE_DEFAULT
    with pytest.raises(ValueError, match='box, silhouette'):
        parse_mask_gate('tight')


def test_silhouette_output_name_drops_the_isolation_token() -> None:
    # The tight branches never run an isolation recipe; folding one into the
    # name would describe code that did not execute.
    assert benchmark_output_name(
        'projective_ranging', 'stereoscopic', 'nearest_mode_histogram',
        'height_crop_range_band', 'silhouette'
    ) == 'silhouette_gated_stereoscopic_projective_ranging'
    assert benchmark_output_name(
        'euclidean_reconstruction', 'stereoscopic', 'nearest_mode_histogram',
        'height_crop_range_band', 'silhouette'
    ) == 'silhouette_gated_stereoscopic_euclidean_reconstruction'
    assert benchmark_output_name(
        'polar_profiling', 'stereoscopic', 'nearest_mode_histogram',
        'height_crop_range_band', 'silhouette'
    ) == 'silhouette_gated_polar_profiling'


def test_silhouette_display_name_folds_the_gate() -> None:
    assert benchmark_display_name(
        'projective_ranging', 'stereoscopic', 'silhouette'
    ) == 'silhouette-gated stereoscopic projective ranging'
    assert benchmark_display_name(
        'polar_profiling', 'stereoscopic', 'silhouette'
    ) == 'silhouette-gated polar profiling'
    assert benchmark_display_name('pointcloud', 'stereoscopic', 'silhouette') == 'Point Cloud'


def test_mask_gate_tokens_mirror_the_node_by_value() -> None:
    from ridgeback_autonomy.perception.g1_mask_measurement_node import (
        MASK_GATES as NODE_MASK_GATES,
    )

    assert MASK_GATES == NODE_MASK_GATES


def test_trial_rows_use_exactly_the_declared_csv_columns(tmp_path) -> None:
    # Schema seam: build_trial_result's rows go straight into write_trial_csv,
    # whose DictWriter raises on any key missing from TRIAL_CSV_COLUMNS. Without
    # this guard a stale column name only surfaces on the first real sim run,
    # after the whole grid has been spawned.
    from types import SimpleNamespace

    from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
        G1DistanceBenchmarkRunner,
        GtInstance,
    )
    from ridgeback_autonomy.benchmarking.scenarios import RobotSpec, Scene
    from ridgeback_autonomy.benchmarking.scoring import OUTCOME_SCORED, SceneScore
    from ridgeback_autonomy.benchmarking.summary import TRIAL_CSV_COLUMNS, write_trial_csv

    runner = SimpleNamespace(
        selected_estimators=('pointcloud',),
        estimator_display_names={'pointcloud': 'Point Cloud'},
    )
    scene = Scene(
        id='scene_a',
        robots=(RobotSpec(x=2.0, y=0.0, yaw=3.14),),
        objects=(),
        repeats_override=None,
    )
    gt = GtInstance(
        index=0, model_name='bench_r0', world_x=2.0, world_y=0.0,
        forward_m=2.0, lateral_m=0.0, distance_m=2.0,
    )
    score = SceneScore(
        medians={0: {'pointcloud': 2.05}},
        outcomes={0: {'pointcloud': OUTCOME_SCORED}},
        detector_missed=(),
        extra_count=0,
    )

    result = G1DistanceBenchmarkRunner.build_trial_result(
        runner,
        {'trial_id': 'scene_a_rep1', 'repeat_index': 1},
        scene, [gt], score, {'pointcloud': []}, '/tmp/x.png',
        {'pointcloud': {}}, 20,
    )

    row = result['rows']['pointcloud'][0]
    assert row['frames_captured'] == 20
    # A scored row has nothing to explain.
    assert row['miss_reason'] is None
    assert set(row) <= set(TRIAL_CSV_COLUMNS), (
        f'row emits columns the CSV cannot write: {sorted(set(row) - set(TRIAL_CSV_COLUMNS))}'
    )
    # Round-trip through the real writer, which is what actually raises.
    output = tmp_path / 'trial.csv'
    write_trial_csv(str(output), result['rows']['pointcloud'])
    assert 'scene_a_rep1' in output.read_text()


def test_no_value_miss_row_carries_its_reason(tmp_path) -> None:
    # The point of the miss_reason column: a failure stays attached to the scene
    # that caused it, instead of only existing in a run-level histogram.
    from types import SimpleNamespace

    from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
        G1DistanceBenchmarkRunner,
        GtInstance,
    )
    from ridgeback_autonomy.benchmarking.scenarios import RobotSpec, Scene
    from ridgeback_autonomy.benchmarking.scoring import OUTCOME_NO_VALUE, SceneScore
    from ridgeback_autonomy.common.miss_reason import MissReason

    runner = SimpleNamespace(
        selected_estimators=('polar_profiling',),
        estimator_display_names={'polar_profiling': 'box-gated polar profiling'},
    )
    scene = Scene(
        id='scan_blocked', robots=(RobotSpec(x=3.0, y=0.0, yaw=3.14),),
        objects=(), repeats_override=None,
    )
    gt = GtInstance(
        index=0, model_name='bench_r0', world_x=3.0, world_y=0.0,
        forward_m=3.0, lateral_m=0.0, distance_m=3.0,
    )
    score = SceneScore(
        medians={0: {'polar_profiling': None}},
        outcomes={0: {'polar_profiling': OUTCOME_NO_VALUE}},
        detector_missed=(),
        extra_count=0,
    )
    histogram = {'polar_profiling': {
        int(MissReason.UNSET): 12, int(MissReason.SCAN_INVALID): 8}}

    result = G1DistanceBenchmarkRunner.build_trial_result(
        runner,
        {'trial_id': 'scan_blocked_rep1', 'repeat_index': 1},
        scene, [gt], score, {'polar_profiling': []}, '', histogram, 20,
    )

    row = result['rows']['polar_profiling'][0]
    assert row['outcome'] == OUTCOME_NO_VALUE
    assert row['miss_reason'] == 'SCAN_INVALID'
    assert row['trial_estimate_m'] is None
    assert row['abs_error_m'] is None
    assert row['frames_captured'] == 20


def test_run_folder_name_leads_with_the_timestamp_then_applied_axes() -> None:
    from ridgeback_autonomy.benchmarking.estimators import (
        benchmark_run_folder_name,
        scenario_slug,
    )

    # Timestamp first so the results directory keeps sorting chronologically.
    assert benchmark_run_folder_name(
        '20260822_202851', '/x/benchmark_scenarios_full.yaml', 'silhouette',
        'stereoscopic', ('projective_ranging',),
    ) == '20260822_202851_full_silhouette_stereoscopic'

    # Mask estimator but no depth path: the depth source never applied.
    assert benchmark_run_folder_name(
        '20260822_202851', '/x/verify_gate.yaml', 'box', 'stereoscopic',
        ('pointcloud', 'polar_profiling'),
    ) == '20260822_202851_verify_gate_box'

    # No mask estimator at all: labelling it with a segmentation gate it never
    # used would be a lie about the run.
    assert benchmark_run_folder_name(
        '20260822_202851', '/x/verify_gate.yaml', 'box', 'stereoscopic',
        ('pointcloud',),
    ) == '20260822_202851_verify_gate'

    assert scenario_slug('/x/benchmark_scenarios_full.yaml') == 'full'
    assert scenario_slug('/x/My Scenes!.yaml') == 'my_scenes'


def test_runner_node_constructs_and_its_provenance_methods_run(tmp_path) -> None:
    # Guards the node wiring, not just the pure helpers: the provenance work
    # reads attributes off the node, and testing the helpers directly cannot
    # catch a method reaching for one that __init__ never set.
    import rclpy

    from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
        G1DistanceBenchmarkRunner,
    )

    scenario = tmp_path / 'scenes.yaml'
    scenario.write_text('scenes:\n  - id: s1\n    robots: [{ x: 2.0, y: 0.0 }]\n')
    output_dir = tmp_path / 'out'

    rclpy.init(args=[
        '--ros-args',
        '-p', f'scenario:={scenario}',
        '-p', f'output_dir:={output_dir}',
        '-p', 'estimators:=pointcloud',
    ])
    node = None
    try:
        node = G1DistanceBenchmarkRunner()
        node.log_code_provenance()
        parameters = node.declared_parameters()

        assert node.workspace_root
        assert parameters['estimators'] == 'pointcloud'
        assert 'start_type_description_service' not in parameters
        # Folder name carries the scenario; no mask estimator, so no gate token.
        assert os.path.basename(node.run_output_dir).endswith('_scenes')
        assert os.path.isdir(node.images_dir)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_runner_uses_explicit_run_dir_name_verbatim(tmp_path) -> None:
    import rclpy

    from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
        G1DistanceBenchmarkRunner,
    )

    scenario = tmp_path / 'scenes.yaml'
    scenario.write_text('scenes:\n  - id: s1\n    robots: [{ x: 2.0, y: 0.0 }]\n')
    output_dir = tmp_path / 'out'

    rclpy.init(args=[
        '--ros-args',
        '-p', f'scenario:={scenario}',
        '-p', f'output_dir:={output_dir}',
        '-p', 'run_dir_name:=pinned_name',
        '-p', 'estimators:=pointcloud',
    ])
    node = None
    try:
        node = G1DistanceBenchmarkRunner()

        assert node.run_output_dir == str(output_dir / 'pinned_name')
        assert node.declared_parameters()['run_dir_name'] == 'pinned_name'
        assert os.path.isdir(node.images_dir)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_format_commit_makes_a_dirty_tree_impossible_to_miss() -> None:
    from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
        format_commit,
    )

    dirty = format_commit({'commit': 'e1bdc17', 'branch': 'main', 'dirty_count': 23})
    assert '23 uncommitted file(s)' in dirty
    # The hash alone would imply a reproducibility that does not exist.
    assert 'NOT reproducible' in dirty

    assert format_commit(
        {'commit': 'e1bdc17', 'branch': 'main', 'dirty_count': 0}) == 'e1bdc17 (clean)'
    # git present but status unreadable: say so rather than claim clean.
    assert 'dirty state unknown' in format_commit(
        {'commit': 'e1bdc17', 'branch': 'main', 'dirty_count': None})
    assert format_commit({'commit': None}) == 'unknown'


def test_git_provenance_survives_a_non_repository() -> None:
    from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
        git_provenance,
    )

    # Bookkeeping must never take a benchmark down with it.
    provenance = git_provenance('/')

    assert provenance['commit'] is None
    assert format_commit_is_unknown(provenance)


def format_commit_is_unknown(provenance) -> bool:
    from ridgeback_autonomy.benchmarking.g1_distance_benchmark_runner_node import (
        format_commit,
    )

    return format_commit(provenance) == 'unknown'


def test_extract_json_payload_accepts_multiple_gz_json_messages() -> None:
    payload = extract_json_payload(
        'noise before\n'
        '{"pose":[{"name":"first","position":{"x":1}}]}\n'
        '{"pose":[{"name":"second","position":{"x":2}}]}\n'
    )

    assert payload == {'pose': [{'name': 'first', 'position': {'x': 1}}]}


def test_alignment_updates_public_estimator_values_from_message_fields() -> None:
    msg = G1Measurements()
    msg.header.frame_id = 'camera'
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34
    msg.detected = True
    msg.count = 1
    msg.image_width = 640
    msg.image_height = 480
    msg.bbox_xyxy = [1.0, 2.0, 30.0, 40.0]
    msg.pointcloud_distance_m = [float('nan')]
    msg.projective_ranging_distance_m = [4.0]
    msg.polar_profiling_distance_m = [3.9]

    events = {}
    event = ensure_measurement_event(events, msg)
    update_measurement_event(
        event,
        msg,
        {'pointcloud', 'projective_ranging', 'polar_profiling'},
    )

    assert event.detected is True
    assert event.count == 1
    assert event.bboxes == ((1, 2, 30, 40),)
    assert event.estimates['pointcloud'] is None
    assert event.estimates['projective_ranging'] == pytest.approx(4.0, rel=1e-6)
    assert event.estimates['polar_profiling'] == pytest.approx(3.9, rel=1e-6)


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

    pointcloud_msg = make_message()
    pointcloud_msg.pointcloud_distance_m = [4.5]
    mask_msg = make_message()
    mask_msg.projective_ranging_distance_m = [4.1]
    mask_msg.euclidean_reconstruction_distance_m = [4.2]

    events = {}
    pointcloud_event = ensure_measurement_event(events, pointcloud_msg)
    update_measurement_event(pointcloud_event, pointcloud_msg, {'pointcloud'})
    mask_event = ensure_measurement_event(events, mask_msg)
    update_measurement_event(mask_event, mask_msg, {'projective_ranging', 'euclidean_reconstruction'})

    # Identical alignment key -> one merged event holding all three estimates.
    assert len(events) == 1
    assert mask_event is pointcloud_event
    assert pointcloud_event.estimates['pointcloud'] == pytest.approx(4.5, rel=1e-6)
    assert pointcloud_event.estimates['projective_ranging'] == pytest.approx(4.1, rel=1e-6)
    assert pointcloud_event.estimates['euclidean_reconstruction'] == pytest.approx(4.2, rel=1e-6)


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


def test_usable_aligned_events_require_the_full_selected_estimator_set() -> None:
    events = {
        'pointcloud_only': MeasurementEvent(
            key=('pointcloud_only',),
            stamp_ns=100,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'pointcloud': 2.0},
            preview=EventPreview(),
        ),
        'both': MeasurementEvent(
            key=('both',),
            stamp_ns=200,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'pointcloud': 2.1, 'polar_profiling': 2.0},
            preview=EventPreview(),
        ),
    }

    usable = usable_aligned_events(events, ('pointcloud', 'polar_profiling'))

    assert [event.key for event in usable] == [('both',)]


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
            estimates={'pointcloud': 2.2, 'polar_profiling': 2.0},
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
            estimates={'pointcloud': 2.0, 'polar_profiling': 1.8},
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
            estimates={'pointcloud': 5.0, 'polar_profiling': 5.0},
            preview=EventPreview(),
        ),
    ]

    medians = compute_trial_medians(usable_events_by_estimator(
        {event.key: event for event in usable}, ('pointcloud', 'polar_profiling')))
    representative = choose_representative_event(
        usable, ('pointcloud', 'polar_profiling'), medians)

    assert medians == {'pointcloud': 2.2, 'polar_profiling': 2.0}
    assert representative.key == ('a',)


def test_representative_event_prefers_a_candidate_with_a_preview() -> None:
    preview = np.full((8, 8, 3), 180, dtype=np.uint8)
    usable = [
        MeasurementEvent(
            key=('no_preview',),
            stamp_ns=100,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'pointcloud': 2.0, 'projective_ranging': 2.0},
            preview=EventPreview(),
        ),
        MeasurementEvent(
            key=('has_preview',),
            stamp_ns=200,
            detected=True,
            count=1,
            bboxes=((1, 2, 3, 4),),
            image_width=640,
            image_height=480,
            estimates={'pointcloud': 2.1, 'projective_ranging': 2.1},
            preview=EventPreview(
                color_bgr=preview,
                color_stamp_ns=200,
                color_delta_ms=0.0,
            ),
        ),
    ]

    medians = compute_trial_medians(usable_events_by_estimator(
        {event.key: event for event in usable}, ('pointcloud', 'projective_ranging')))
    representative = choose_representative_event(
        usable, ('pointcloud', 'projective_ranging'), medians)

    assert representative.key == ('has_preview',)


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
            estimates={'pointcloud': 2.0, 'projective_ranging': 2.0},
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
            estimates={'pointcloud': 3.0, 'projective_ranging': 3.0},
            preview=EventPreview(
                color_bgr=preview,
                color_stamp_ns=200,
                color_delta_ms=0.0,
            ),
        ),
    ]

    medians = compute_trial_medians(usable_events_by_estimator(
        {event.key: event for event in usable}, ('pointcloud', 'projective_ranging')))
    representative = choose_representative_event(
        usable, ('pointcloud', 'projective_ranging'), medians)

    assert representative.key == ('best_numeric',)


def test_render_missing_preview_context_is_not_a_silent_black_panel() -> None:
    renderer = BenchmarkCollageRenderer()
    event = MeasurementEvent(
        key=('missing',),
        stamp_ns=123_000_000,
        detected=True,
        count=1,
        bboxes=((10, 20, 40, 60),),
        image_width=320,
        image_height=180,
        estimates={'projective_ranging': 2.5},
        preview=EventPreview(
            color_nearest_stamp_ns=150_000_000,
            color_nearest_delta_ms=27.0,
        ),
    )

    context = renderer.panel_context_for_event(event)
    lines = renderer.build_panel_lines(
        context,
        'trial_001',
        frame_value=2.5,
        trial_median_m=2.6,
        true_distance_m=2.7,
        event_stamp_ns=event.stamp_ns,
    )

    assert context.preview_available is False
    assert context.expected_source_label == 'RGB Debug View'
    assert context.panel.mean() > 0.0
    assert 'Preview Missing' in lines
    assert 'Expected: RGB Debug View' in lines
    assert 'Nearest: 27.0ms' in lines


def test_every_panel_is_the_same_colour_frame() -> None:
    renderer = BenchmarkCollageRenderer()
    color_panel = np.full((120, 160, 3), 90, dtype=np.uint8)
    event = MeasurementEvent(
        key=('debug_view',),
        stamp_ns=500_000_000,
        detected=True,
        count=1,
        bboxes=((10, 20, 40, 60),),
        image_width=160,
        image_height=120,
        estimates={'pointcloud': 3.0, 'polar_profiling': 3.1},
        preview=EventPreview(
            color_bgr=color_panel,
            color_stamp_ns=500_000_000,
            color_delta_ms=0.0,
        ),
    )

    context = renderer.panel_context_for_event(event)
    lines = renderer.build_panel_lines(
        context,
        'trial_002',
        frame_value=3.0,
        trial_median_m=3.0,
        true_distance_m=3.2,
        event_stamp_ns=event.stamp_ns,
    )

    assert context.preview_available is True
    assert context.source_label == 'RGB Debug View'
    assert 'Source: RGB Debug View' in lines
    assert 'Delta: 0.0ms' in lines


def test_build_summary_rows_aggregates_trial_level_estimator_rows() -> None:
    summary_rows = build_summary_rows({
        'pointcloud': [
            {'abs_error_m': 0.2, 'rel_error': 0.1},
            {'abs_error_m': 0.4, 'rel_error': 0.2},
        ],
        'polar_profiling': [
            {'abs_error_m': 0.1, 'rel_error': 0.05},
        ],
    })

    pointcloud_row = next(
        row for row in summary_rows if row['estimator'] == 'pointcloud')
    polar_row = next(
        row for row in summary_rows if row['estimator'] == 'polar_profiling')

    assert pointcloud_row['trial_count'] == 2
    assert pointcloud_row['mean_abs_error_m'] == pytest.approx(0.3, rel=1e-6)
    assert pointcloud_row['median_abs_error_m'] == pytest.approx(0.3, rel=1e-6)
    assert pointcloud_row['mean_rel_error'] == pytest.approx(0.15, rel=1e-6)

    assert polar_row['trial_count'] == 1
    assert polar_row['p95_abs_error_m'] == pytest.approx(0.1, rel=1e-6)


def test_ground_truth_point_message_packs_planar_truth() -> None:
    msg = ground_truth_point_message(
        {'lateral_m': -0.75, 'forward_m': 3.5, 'distance_m': 3.58}, 'bed_occluder_single')

    assert msg.point.x == pytest.approx(-0.75)
    assert msg.point.y == pytest.approx(3.5)
    assert msg.point.z == pytest.approx(3.58)


def test_ground_truth_point_message_names_its_trial() -> None:
    # The trial id rides in header.frame_id (not a TF frame -- neither is the
    # point) so a displayed truth can be checked against the scene on screen.
    # Without it, a truth left over from an earlier trial is unfalsifiable.
    msg = ground_truth_point_message(
        {'lateral_m': 0.0, 'forward_m': 3.25, 'distance_m': 3.25}, 'bed_occluder_single')

    assert msg.header.frame_id == 'bed_occluder_single'


def test_run_document_is_machine_readable_with_full_precision(tmp_path) -> None:
    # run.json replaced comparison_summary.csv because that CSV duplicated most
    # of summary.md while being a poor machine format: JSON inside a cell, two
    # granularities in one flat row, and no provenance anywhere machine-readable.
    import json

    from ridgeback_autonomy.benchmarking.scoring import OUTCOME_NO_VALUE
    from ridgeback_autonomy.benchmarking.summary import build_run_document, write_run_json
    from ridgeback_autonomy.common.miss_reason import MissReason

    summary_rows = build_summary_rows(
        {'polar_profiling': [{'abs_error_m': 0.05895917156831243, 'rel_error': 0.0158463}]},
        outcome_counts={'polar_profiling': {OUTCOME_NO_VALUE: 2}},
        status_histograms={'polar_profiling': {
            int(MissReason.OK): 16,
            int(MissReason.TOO_FEW_RAYS_SELECTED): 30,
        }},
    )
    document = build_run_document(
        summary_rows,
        run_metadata={
            'label': '20260824_112331',
            'commit': 'e1bdc17',
            'branch': 'g1-distance-benchmarks',
            'uncommitted_files': 24,
            'scenes': 2,
        },
        parameters={'mask_gate': 'box', 'repeats': '1'},
        display_names={'polar_profiling': 'box-gated polar profiling'},
        status_histograms={'polar_profiling': {
            int(MissReason.OK): 16,
            int(MissReason.TOO_FEW_RAYS_SELECTED): 30,
        }},
    )

    path = tmp_path / 'run.json'
    write_run_json(str(path), document)
    loaded = json.loads(path.read_text())

    # Provenance is queryable, not prose: the dirty count is a number.
    assert loaded['run']['commit'] == 'e1bdc17'
    assert loaded['run']['uncommitted_files'] == 24
    assert loaded['parameters']['mask_gate'] == 'box'

    entry = loaded['estimators'][0]
    # Stable key AND display name, which is what lets the runner stop rewriting
    # the key in place before writing.
    assert entry['key'] == 'polar_profiling'
    assert entry['display_name'] == 'box-gated polar profiling'
    # Full float precision, unlike the report's three decimals.
    assert entry['mean_abs_error_m'] == 0.05895917156831243
    # Nested groups instead of a flat row mixing two granularities.
    assert entry['missed'] == {'total': 2, 'detector': 0, 'gate': 0, 'no_value': 2}
    assert entry['observations']['total'] == 46
    assert entry['observations']['ok'] == 16
    # A real mapping, not JSON embedded in a CSV cell.
    assert entry['reason_histogram'] == {'OK': 16, 'TOO_FEW_RAYS_SELECTED': 30}


def test_missed_instance_count_is_derived_from_the_outcome_counts() -> None:
    # One source of truth: the total is the sum of the three miss outcomes, not
    # a separately plumbed number that could drift away from them.
    from ridgeback_autonomy.benchmarking.scoring import (
        OUTCOME_DETECTOR_MISS,
        OUTCOME_GATE_MISS,
        OUTCOME_NO_VALUE,
    )

    summary_rows = build_summary_rows(
        {'pointcloud': [{'abs_error_m': 0.1, 'rel_error': 0.05}]},
        extra_detection_count=1,
        outcome_counts={'pointcloud': {
            OUTCOME_DETECTOR_MISS: 2,
            OUTCOME_GATE_MISS: 1,
            OUTCOME_NO_VALUE: 0,
        }},
    )

    row = next(row for row in summary_rows if row['estimator'] == 'pointcloud')
    assert row['missed_instance_count'] == 3
    assert row['detector_missed_count'] == 2
    assert row['gate_missed_count'] == 1
    assert row['no_value_missed_count'] == 0
    assert row['extra_detection_count'] == 1


def _event_with_detection(detection: Detection) -> MeasurementEvent:
    return MeasurementEvent(
        key=('x',),
        stamp_ns=1,
        detected=True,
        count=1,
        bboxes=((0, 0, 10, 10),),
        image_width=64,
        image_height=48,
        detections=[detection],
        preview=EventPreview(),
    )


def test_box_label_shows_instance_estimate_and_true() -> None:
    renderer = BenchmarkCollageRenderer()
    event = _event_with_detection(Detection(
        bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9, pointcloud_distance_m=2.13))

    label, color = renderer.box_label_and_color(
        event, 0, 'pointcloud', [{'instance_index': 1, 'true_distance_m': 2.25}])

    assert label == '#1 e2.13/t2.25'
    assert color == (0, 255, 0)


def test_box_label_marks_unmatched_detection_as_extra() -> None:
    renderer = BenchmarkCollageRenderer()
    event = _event_with_detection(Detection(bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9))

    label, color = renderer.box_label_and_color(event, 0, 'pointcloud', [None])

    assert label == 'extra'
    assert color == (0, 165, 255)


def test_box_label_defaults_to_historical_label_without_annotations() -> None:
    renderer = BenchmarkCollageRenderer()
    event = _event_with_detection(Detection(bbox_xyxy=(0, 0, 10, 10), label='r', score=0.9))

    label, color = renderer.box_label_and_color(event, 0, None, None)

    assert label == 'G1 #1'
    assert color == (0, 255, 0)


def _estimates_event(key, stamp_ns, estimates):
    return MeasurementEvent(
        key=(key,),
        stamp_ns=stamp_ns,
        detected=True,
        count=1,
        bboxes=((1, 2, 3, 4),),
        image_width=640,
        image_height=480,
        estimates=estimates,
        preview=EventPreview(),
    )


def test_usable_events_by_estimator_scores_each_on_its_own_events() -> None:
    # polar blind on every frame (occluder on the scan plane): the pointcloud
    # row is still usable.
    events = {
        ('a',): _estimates_event('a', 100, {'pointcloud': 2.0, 'polar_profiling': None}),
        ('b',): _estimates_event('b', 200, {'pointcloud': 2.2}),
    }

    by_estimator = usable_events_by_estimator(events, ('pointcloud', 'polar_profiling'))

    assert [event.stamp_ns for event in by_estimator['pointcloud']] == [100, 200]
    assert by_estimator['polar_profiling'] == []
    # The union keeps the trial alive even though one estimator is at zero.
    assert [event.stamp_ns for event in union_usable_events(by_estimator)] == [100, 200]


def test_compute_trial_medians_is_partial_for_blind_estimators() -> None:
    events = {
        ('a',): _estimates_event('a', 100, {'pointcloud': 2.0}),
        ('b',): _estimates_event('b', 200, {'pointcloud': 2.4}),
    }
    by_estimator = usable_events_by_estimator(events, ('pointcloud', 'polar_profiling'))

    medians = compute_trial_medians(by_estimator)

    assert medians == {'pointcloud': pytest.approx(2.2)}
    assert 'polar_profiling' not in medians


def test_representative_event_chosen_from_union_without_common_event() -> None:
    # No event carries both estimators; the union still yields a collage frame,
    # preferring the one covering more estimators.
    both = _estimates_event('both', 300, {'pointcloud': 2.0, 'polar_profiling': 2.0})
    pointcloud_only = _estimates_event('pointcloud', 100, {'pointcloud': 2.0})
    by_estimator = {
        'pointcloud': [pointcloud_only, both], 'polar_profiling': [both]}
    union = union_usable_events(by_estimator)
    medians = compute_trial_medians(by_estimator)

    chosen = choose_representative_event(
        union, ('pointcloud', 'polar_profiling'), medians)

    assert chosen.key == ('both',)
