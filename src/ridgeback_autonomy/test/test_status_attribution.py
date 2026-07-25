from __future__ import annotations

from std_msgs.msg import Header

from ridgeback_autonomy.benchmarking.alignment import (
    MeasurementEvent,
    extract_estimator_statuses,
)
from ridgeback_autonomy.benchmarking.reduction import (
    compute_status_histogram,
    format_status_tally,
    merge_status_histograms,
)
from ridgeback_autonomy.common.messages import build_measurements_message
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.models import Detection, DetectionBatch


def _event(count: int, statuses: dict) -> MeasurementEvent:
    return MeasurementEvent(
        key=('frame', 0, 0, count, ()),
        stamp_ns=0,
        detected=True,
        count=count,
        bboxes=(),
        image_width=640,
        image_height=480,
        estimate_statuses=dict(statuses),
    )


def test_compute_status_histogram_counts_codes_over_count1_events() -> None:
    selected = ('projective_ranging', 'polar_profiling')
    events = {
        'a': _event(1, {
            'projective_ranging': int(MissReason.OK),
            'polar_profiling': int(MissReason.OK)}),
        'b': _event(1, {
            'projective_ranging': int(MissReason.OK),
            'polar_profiling': int(MissReason.SCAN_INVALID)}),
        # count != 1 is excluded from the histogram.
        'c': _event(2, {
            'projective_ranging': int(MissReason.OK),
            'polar_profiling': int(MissReason.OK)}),
    }

    histogram = compute_status_histogram(events, selected)

    assert histogram['projective_ranging'] == {int(MissReason.OK): 2}
    assert histogram['polar_profiling'] == {
        int(MissReason.OK): 1, int(MissReason.SCAN_INVALID): 1}


def test_missing_status_counts_as_unset() -> None:
    histogram = compute_status_histogram({'a': _event(1, {})}, ('projective_ranging',))

    assert histogram['projective_ranging'] == {int(MissReason.UNSET): 1}


def test_merge_status_histograms_folds_counts() -> None:
    aggregate = {'polar_profiling': {int(MissReason.OK): 3}}
    merge_status_histograms(aggregate, {'polar_profiling': {
        int(MissReason.OK): 2, int(MissReason.SCAN_INVALID): 5}})

    assert aggregate['polar_profiling'] == {
        int(MissReason.OK): 5, int(MissReason.SCAN_INVALID): 5}


def test_format_status_tally_names_the_culprit() -> None:
    histogram = {
        'projective_ranging': {int(MissReason.OK): 20},
        'polar_profiling': {int(MissReason.SCAN_INVALID): 20},
    }

    tally = format_status_tally(histogram, ('projective_ranging', 'polar_profiling'))

    assert tally == 'projective 20/20, polar 0/20 (SCAN_INVALID)'


def test_extract_estimator_statuses_mask_explicit_and_legacy_inferred() -> None:
    batch = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[Detection(
            bbox_xyxy=(1, 2, 30, 40), label='humanoid robot', score=0.9,
            projective_ranging_status=int(MissReason.OK),
            polar_profiling_status=int(MissReason.NO_SCAN),
            rgb_distance_m=2.5,            # legacy present -> inferred OK
            sensor_depth_distance_m=None,  # legacy absent -> inferred UNSET
        )],
    )

    statuses = extract_estimator_statuses(build_measurements_message(batch, Header()))

    assert statuses['projective_ranging'] == int(MissReason.OK)
    assert statuses['polar_profiling'] == int(MissReason.NO_SCAN)
    assert statuses['rgb'] == int(MissReason.OK)
    assert statuses['sensor_depth'] == int(MissReason.UNSET)
