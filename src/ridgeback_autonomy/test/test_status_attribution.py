from __future__ import annotations

from std_msgs.msg import Header

from ridgeback_autonomy.benchmarking.alignment import (
    MeasurementEvent,
    extract_estimator_statuses,
)
from ridgeback_autonomy.benchmarking.reduction import (
    compute_status_histogram,
    dominant_miss_reason,
    format_status_tally,
    merge_status_histograms,
)
from ridgeback_autonomy.common.messages import build_measurements_message
from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.common.models import Detection, DetectionBatch


def _detection(**fields) -> Detection:
    return Detection(bbox_xyxy=(0, 0, 10, 10), label='humanoid robot', score=0.9, **fields)


def _event(detections: list[Detection]) -> MeasurementEvent:
    return MeasurementEvent(
        key=('frame', 0, 0, len(detections), ()),
        stamp_ns=0,
        detected=True,
        count=len(detections),
        bboxes=tuple((0, 0, 10, 10) for _ in detections),
        image_width=640,
        image_height=480,
        detections=list(detections),
    )


def test_compute_status_histogram_counts_every_box_including_multi_robot_frames() -> None:
    selected = ('projective_ranging', 'polar_profiling')
    events = {
        'a': _event([_detection(
            projective_ranging_status=int(MissReason.OK),
            polar_profiling_status=int(MissReason.OK))]),
        'b': _event([_detection(
            projective_ranging_status=int(MissReason.OK),
            polar_profiling_status=int(MissReason.SCAN_INVALID))]),
        # A 2-robot frame contributes BOTH boxes, and each carries its own
        # status. Counting frames instead skipped this frame entirely, which is
        # why multi-robot scenes used to produce no reasons at all.
        'c': _event([
            _detection(
                projective_ranging_status=int(MissReason.OK),
                polar_profiling_status=int(MissReason.OK)),
            _detection(
                projective_ranging_status=int(MissReason.TOO_FEW_VALID_PIXELS),
                polar_profiling_status=int(MissReason.OK)),
        ]),
    }

    histogram = compute_status_histogram(events, selected)

    assert histogram['projective_ranging'] == {
        int(MissReason.OK): 3, int(MissReason.TOO_FEW_VALID_PIXELS): 1}
    assert histogram['polar_profiling'] == {
        int(MissReason.OK): 3, int(MissReason.SCAN_INVALID): 1}


def test_missing_status_counts_as_unset() -> None:
    histogram = compute_status_histogram({'a': _event([_detection()])}, ('projective_ranging',))

    assert histogram['projective_ranging'] == {int(MissReason.UNSET): 1}


def test_statusless_estimator_status_is_inferred_per_detection() -> None:
    # The pointcloud row publishes no status array, so OK/UNSET is inferred from
    # whether THAT box got a distance -- not from the frame's first box.
    event = _event([
        _detection(pointcloud_distance_m=2.5),
        _detection(pointcloud_distance_m=None),
    ])

    histogram = compute_status_histogram({'a': event}, ('pointcloud',))

    assert histogram['pointcloud'] == {
        int(MissReason.OK): 1, int(MissReason.UNSET): 1}


def test_merging_a_mask_message_keeps_its_per_detection_status() -> None:
    # The histogram reads statuses off event.detections, so the merge has to
    # carry them. When it dropped them, a polar_profiling run that scored every
    # instance still reported {"UNSET": 52} at 0.0 coverage -- values present,
    # reasons blank, the two flatly contradicting each other.
    from ridgeback_autonomy.benchmarking.alignment import (
        ensure_measurement_event,
        update_measurement_event,
    )

    batch = DetectionBatch(
        image_width=640, image_height=480,
        detections=[Detection(
            bbox_xyxy=(1, 2, 30, 40), label='humanoid robot', score=0.9,
            polar_profiling_distance_m=2.4,
            polar_profiling_forward_m=2.4,
            polar_profiling_lateral_m=0.0,
            polar_profiling_status=int(MissReason.OK),
        )],
    )
    msg = build_measurements_message(batch, Header())

    events: dict = {}
    event = ensure_measurement_event(events, msg)
    update_measurement_event(event, msg, {'polar_profiling'})

    assert event.detections[0].polar_profiling_status == int(MissReason.OK)
    histogram = compute_status_histogram(events, ('polar_profiling',))
    assert histogram['polar_profiling'] == {int(MissReason.OK): 1}


def test_dominant_miss_reason_prefers_a_specific_code_over_unset() -> None:
    # UNSET usually dominates the tally (the frame never reached the node) and
    # is never the story, so a specific reason wins even when rarer.
    assert dominant_miss_reason({
        int(MissReason.UNSET): 18,
        int(MissReason.SCAN_INVALID): 2,
    }) == 'SCAN_INVALID'
    assert dominant_miss_reason({int(MissReason.UNSET): 5}) == 'UNSET'
    # Nothing failed -> no reason to report.
    assert dominant_miss_reason({int(MissReason.OK): 20}) is None
    assert dominant_miss_reason({}) is None
    assert dominant_miss_reason(None) is None


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


def test_extract_estimator_statuses_mask_explicit_and_pointcloud_inferred() -> None:
    present = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[Detection(
            bbox_xyxy=(1, 2, 30, 40), label='humanoid robot', score=0.9,
            projective_ranging_status=int(MissReason.OK),
            polar_profiling_status=int(MissReason.NO_SCAN),
            pointcloud_distance_m=2.5,  # no status array -> inferred OK
        )],
    )

    statuses = extract_estimator_statuses(build_measurements_message(present, Header()))

    assert statuses['projective_ranging'] == int(MissReason.OK)
    assert statuses['polar_profiling'] == int(MissReason.NO_SCAN)
    assert statuses['pointcloud'] == int(MissReason.OK)

    absent = DetectionBatch(
        image_width=640,
        image_height=480,
        detections=[Detection(
            bbox_xyxy=(1, 2, 30, 40), label='humanoid robot', score=0.9,
            pointcloud_distance_m=None,  # no status array -> inferred UNSET
        )],
    )

    assert extract_estimator_statuses(
        build_measurements_message(absent, Header())
    )['pointcloud'] == int(MissReason.UNSET)
