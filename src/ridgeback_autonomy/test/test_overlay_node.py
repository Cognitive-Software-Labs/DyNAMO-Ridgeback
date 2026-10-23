"""The overlay's side of the polar beam contract.

The panel is a consumer: it draws the beams the measuring node published rather
than re-deriving a selection from its own mask and its own scan, which is how it
came to highlight a different band than the run it was labelling
(``docs/target_localization/polar_profiling.md`` Section 4).
"""

from __future__ import annotations

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, LaserScan
from std_msgs.msg import Header

from ridgeback_common.markers import PolarBeamRecord
from ridgeback_common.messages import build_polar_beams_message
from ridgeback_interfaces.msg import TargetMeasurements


CAMERA_FRAME = 'camera_0_color_optical_frame'
SCAN_FRAME = 'lidar2d_0_laser'

# Scan -> camera optical for a level mount: optical Z (forward) = scan X,
# optical X (right) = -scan Y, optical Y (down) = -scan Z.
SCAN_TO_OPTICAL = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
])


@pytest.fixture
def ros_context():
    rclpy = pytest.importorskip('rclpy')
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def overlay_node():
    from ridgeback_localization.overlay_node import (
        TargetOverlayNode,
    )

    # The default estimator set includes polar profiling, which is the axis
    # under test; nothing here depends on the other panels.
    return TargetOverlayNode()


def stub_transform(monkeypatch) -> None:
    """Pin the scan -> optical extrinsic; TF is not what these tests measure."""

    from ridgeback_localization import overlay_node as module

    monkeypatch.setattr(
        module, 'lookup_transform_components',
        lambda *args, **kwargs: (SCAN_TO_OPTICAL, np.zeros(3), None))


def scan(beam_count: int, sec: int, nanosec: int = 0) -> LaserScan:
    msg = LaserScan()
    msg.header.frame_id = SCAN_FRAME
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.angle_min = -0.5
    msg.angle_max = 0.5
    msg.angle_increment = 1.0 / max(beam_count - 1, 1)
    msg.range_min = 0.05
    msg.range_max = 25.0
    msg.ranges = [2.0] * beam_count
    return msg


def beams_message(scan_msg: LaserScan, measurement_sec: int, selected, merged):
    header = Header()
    header.frame_id = CAMERA_FRAME
    header.stamp.sec = measurement_sec
    record = PolarBeamRecord(
        detection_index=0,
        selected=np.array(selected, dtype=np.intp),
        merged=np.array(merged, dtype=np.intp),
        in_bbox=np.array(selected, dtype=np.intp),
    )
    return build_polar_beams_message([record], scan_msg, header)


def measurements(sec: int) -> TargetMeasurements:
    msg = TargetMeasurements()
    msg.header.frame_id = CAMERA_FRAME
    msg.header.stamp.sec = sec
    return msg


def camera_info() -> CameraInfo:
    info = CameraInfo()
    info.width, info.height = 64, 48
    info.k = [40.0, 0.0, 32.0, 0.0, 40.0, 24.0, 0.0, 0.0, 1.0]
    return info


def prime(node, *, scans, beams, measurement_sec) -> None:
    for scan_msg in scans:
        node.scan_callback(scan_msg)
    for beams_msg in beams:
        node.polar_beams_callback(beams_msg)
    node.latest_measurements_msg = measurements(measurement_sec)
    node.latest_color_info = camera_info()


def test_panel_uses_the_scan_the_beams_name_not_the_newest(
    ros_context, monkeypatch,
) -> None:
    # Two scans of different lengths are cached. The beams message names the
    # older one; applying its indices to the newer array would shift the
    # highlight onto beams the estimator never touched.
    stub_transform(monkeypatch)
    node = overlay_node()
    try:
        matched = scan(8, sec=5)
        prime(
            node,
            scans=[matched, scan(16, sec=6)],
            beams=[beams_message(matched, 5, [2, 3, 4], [3, 4])],
            measurement_sec=5,
        )

        uv, in_view, highlight = node.project_published_scan()

        assert uv.shape[0] == 8  # the named scan, not the 16-beam newest
        assert highlight is not None
        assert list(np.flatnonzero(highlight.used)) == [3, 4]
        # Selected but discarded by the range band: the state the panel could
        # not express while it re-derived the selection itself.
        assert list(np.flatnonzero(highlight.dropped)) == [2]
    finally:
        node.destroy_node()


def test_no_beams_message_for_the_stamp_draws_no_scan(
    ros_context, monkeypatch,
) -> None:
    stub_transform(monkeypatch)
    node = overlay_node()
    try:
        prime(node, scans=[scan(8, sec=5)], beams=[], measurement_sec=5)

        assert node.project_published_scan() == (None, None, None)
    finally:
        node.destroy_node()


def test_a_scan_aged_out_of_the_cache_draws_no_scan(
    ros_context, monkeypatch,
) -> None:
    # Required failure mode: a missing highlight, never a misaligned one.
    stub_transform(monkeypatch)
    node = overlay_node()
    try:
        prime(
            node,
            scans=[scan(8, sec=9)],
            beams=[beams_message(scan(8, sec=5), 5, [2], [2])],
            measurement_sec=5,
        )

        assert node.project_published_scan() == (None, None, None)
    finally:
        node.destroy_node()


def test_beam_count_mismatch_drops_the_highlight_but_keeps_the_points(
    ros_context, monkeypatch,
) -> None:
    # Same stamp and frame, different length: the producer indexed an array this
    # node does not have. Draw the scan, claim nothing about it.
    stub_transform(monkeypatch)
    node = overlay_node()
    try:
        prime(
            node,
            scans=[scan(8, sec=5)],
            beams=[beams_message(scan(12, sec=5), 5, [2, 3], [3])],
            measurement_sec=5,
        )

        uv, in_view, highlight = node.project_published_scan()

        assert uv is not None and in_view is not None
        assert highlight is None
    finally:
        node.destroy_node()


def test_beams_arriving_after_their_measurement_trigger_a_re_render(
    ros_context,
) -> None:
    # The beams message lags its measurements message by the estimator
    # reduction, exactly as the silhouette lags it by segmentation. Without the
    # re-render the panel would hold its unhighlighted first draw.
    node = overlay_node()
    renders: list[int] = []
    node.render_latest = lambda: renders.append(1)
    try:
        node.latest_measurements_msg = measurements(5)

        node.polar_beams_callback(beams_message(scan(8, sec=5), 5, [2], [2]))
        assert len(renders) == 1

        # A stamp nothing is waiting on is cached and nothing more.
        node.polar_beams_callback(beams_message(scan(8, sec=6), 6, [2], [2]))
        assert len(renders) == 1
    finally:
        node.destroy_node()


def test_sensor_and_beam_caches_are_bounded(ros_context) -> None:
    node = overlay_node()
    try:
        for index in range(max(node.SCAN_CACHE_DEPTH, node.IMAGE_CACHE_DEPTH) + 10):
            node.scan_callback(scan(8, sec=index))
            node.polar_beams_callback(beams_message(scan(8, sec=index), index, [], []))
            frame = solid_color_image(index, 40)
            node.color_callback(frame)
            node.aligned_depth_callback(frame)

        assert len(node.scan_cache) == node.SCAN_CACHE_DEPTH
        assert len(node.polar_beams_cache) == node.CACHE_DEPTH
        assert len(node.color_cache) == node.IMAGE_CACHE_DEPTH
        assert len(node.aligned_depth_cache) == node.IMAGE_CACHE_DEPTH
    finally:
        node.destroy_node()


def solid_color_image(sec: int, value: int):
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header.frame_id = CAMERA_FRAME
    msg.header.stamp.sec = sec
    msg.height, msg.width, msg.encoding, msg.step = 48, 64, 'rgb8', 64 * 3
    msg.data = bytes([value]) * (64 * 48 * 3)
    return msg


def capture_render(node, monkeypatch):
    drawn, published = [], []

    def render(frame, *args, **kwargs):
        drawn.append(int(frame[0, 0, 0]))
        return frame

    monkeypatch.setattr(node.renderer, 'render', render)
    monkeypatch.setattr(node.overlay_pub, 'publish', published.append)
    return drawn, published


def test_measurement_is_drawn_on_the_frame_it_was_measured_from(
    ros_context, monkeypatch,
) -> None:
    # A remote result arrives an inference latency after its frame. Drawing it
    # on the newest image would put the box where the target no longer is.
    node = overlay_node()
    drawn, published = capture_render(node, monkeypatch)
    node.color_callback(solid_color_image(10, 40))
    node.color_callback(solid_color_image(11, 200))
    node.mask_measurement_callback(measurements(10))
    assert drawn == [40]
    assert published[-1].header.stamp.sec == 10
    node.destroy_node()


def test_delayed_silhouette_result_keeps_its_scan_highlight_and_color_frame(
    ros_context, monkeypatch,
) -> None:
    # Full detector + segmentation latency is ~1.1 s. At 40 Hz the overlay's
    # former 32 scans had expired; its 32 images at 30 Hz also lost the source
    # frame and silently fell back to a newer image.
    stub_transform(monkeypatch)
    node = overlay_node()
    drawn, published = [], []

    def capture(frame, *args, **kwargs):
        drawn.append((int(frame[0, 0, 0]), kwargs))
        return frame

    monkeypatch.setattr(node.renderer, 'render', capture)
    monkeypatch.setattr(node.overlay_pub, 'publish', published.append)
    try:
        matched_scan = scan(8, sec=5)
        node.scan_callback(matched_scan)
        node.color_callback(solid_color_image(5, 40))
        for index in range(1, 49):
            sec, nanosec = divmod(index * 25_000_000, 1_000_000_000)
            node.scan_callback(scan(8, 5 + sec, nanosec))
        for index in range(1, 37):
            frame = solid_color_image(5, 200)
            sec, nanosec = divmod(round(index * 1e9 / 30), 1_000_000_000)
            frame.header.stamp.sec += sec
            frame.header.stamp.nanosec = nanosec
            node.color_callback(frame)
        node.latest_color_info = camera_info()
        node.polar_beams_callback(beams_message(matched_scan, 5, [2, 3, 4], [3, 4]))
        node.mask_measurement_callback(measurements(5))

        color, rendered = drawn[-1]
        assert color == 40
        assert rendered['scan_uv'] is not None
        assert list(np.flatnonzero(rendered['scan_highlight'].used)) == [3, 4]
        assert list(np.flatnonzero(rendered['scan_highlight'].dropped)) == [2]
        assert published[-1].header.stamp.sec == 5
        assert published[-1].header.stamp.nanosec == 0
    finally:
        node.destroy_node()


def test_measurement_without_its_frame_falls_back_to_the_newest(
    ros_context, monkeypatch,
) -> None:
    node = overlay_node()
    drawn, published = capture_render(node, monkeypatch)
    node.color_callback(solid_color_image(11, 200))
    node.mask_measurement_callback(measurements(5))
    assert drawn == [200]
    assert published[-1].header.stamp.sec == 11
    node.destroy_node()
