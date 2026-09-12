#!/usr/bin/env python3

"""Publish target-localization estimator snapshots as RViz markers and HUD text.

The node owns ROS subscriptions, the receipt-time cache, TF lookup, and the
single render clock.  Snapshot selection, HUD formatting, and marker message
construction live in dedicated modules so neither benchmarking nor another
display node needs to import this ROS orchestration layer.
"""

from __future__ import annotations

import math
import threading

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rviz_2d_overlay_msgs.msg import OverlayText
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

from ridgeback_autonomy.msg import TargetMeasurements
from ridgeback_autonomy.perception.target_localization.contracts import (
    ESTIMATE_MARKERS_TOPIC,
    GROUND_TRUTH_TOPIC,
    HUD_DISTANCES_PANEL_TOPIC,
    MASK_MEASUREMENTS_TOPIC,
    POINTCLOUD_MEASUREMENTS_TOPIC,
)
from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    parse_estimators,
)
from ridgeback_autonomy.perception.target_localization.ground_truth import (
    truth_reading,
)
from ridgeback_autonomy.perception.target_localization.hud_rendering import (
    HUD_LAYOUT_ROWS,
    HUD_LAYOUT_WIDE,
    hud_section_from_readings,
    hud_wide_text,
    parse_hud_layout,
)
from ridgeback_autonomy.perception.target_localization.marker_rendering import (
    append_delete_markers,
    append_estimator_markers,
)
from ridgeback_autonomy.perception.target_localization.visualization_readings import (
    batch_messages,
    collect_readings,
    nearest_detection_index,
    partition_measurements,
)


# A marker lifetime is both the RViz crash backstop and the producer-liveness
# budget. A reading disappears on the first render tick after its producer has
# been silent this long; explicit DELETE markers keep the HUD and floor plan in
# step, while lifetime still cleans up if this node itself dies.
MARKER_LIFETIME_SEC = 1.5

# Observation validity is separate from producer liveness. Mask measurements
# arrive with the earlier detection stamp after inference and depth work, so
# that pipeline latency must not be mistaken for producer silence.
MAX_OBSERVATION_AGE_S = 3.0

# Both surfaces render on this clock from one shared snapshot.
HUD_PUBLISH_RATE_HZ = 5.0


class TargetVisualizationNode(Node):
    def __init__(self) -> None:
        super().__init__('target_visualization_node')

        namespace_name = self.get_namespace().strip('/')
        default_base_frame = (
            f'{namespace_name}/robot/base_link'
            if namespace_name else 'robot/base_link'
        )

        self.declare_parameter('base_frame', default_base_frame)
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('marker_lifetime_sec', MARKER_LIFETIME_SEC)
        self.declare_parameter('max_observation_age_sec', MAX_OBSERVATION_AGE_S)
        self.declare_parameter('hud_publish_rate_hz', HUD_PUBLISH_RATE_HZ)
        self.declare_parameter('ground_truth_topic', GROUND_TRUTH_TOPIC)
        self.declare_parameter('hud_distances_topic', HUD_DISTANCES_PANEL_TOPIC)
        self.declare_parameter('estimators', 'all')
        self.declare_parameter('hud_layout', HUD_LAYOUT_ROWS)

        self.base_frame = self.get_parameter('base_frame').value or default_base_frame
        self.world_frame = self.get_parameter('world_frame').value
        self.marker_lifetime = self.get_parameter('marker_lifetime_sec').value
        self.max_observation_age = float(
            self.get_parameter('max_observation_age_sec').value)
        self.estimators = parse_estimators(
            str(self.get_parameter('estimators').value))
        self.hud_layout = parse_hud_layout(
            str(self.get_parameter('hud_layout').value))

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)

        self._lock = threading.Lock()
        # Receipt time answers producer liveness; the message stamp answers
        # observation validity. They differ materially for the mask producer.
        self._latest: dict[str, tuple[TargetMeasurements, int] | None] = {
            'pointcloud': None,
            'mask': None,
        }
        self._latest_truth: PointStamped | None = None

        for key, topic in (
            ('pointcloud', POINTCLOUD_MEASUREMENTS_TOPIC),
            ('mask', MASK_MEASUREMENTS_TOPIC),
        ):
            self.create_subscription(
                TargetMeasurements,
                topic,
                lambda msg, key=key: self._measurement_cb(key, msg),
                10,
            )

        # Latest-value-wins truth avoids pinning the previous trial behind a
        # subscription backlog.
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('ground_truth_topic').value),
            self._truth_cb,
            1,
        )

        self._pub = self.create_publisher(
            MarkerArray, ESTIMATE_MARKERS_TOPIC, 10)
        self._hud_pub = self.create_publisher(
            OverlayText,
            str(self.get_parameter('hud_distances_topic').value),
            10,
        )

        hud_rate = float(self.get_parameter('hud_publish_rate_hz').value)
        self.create_timer(
            1.0 / (hud_rate if hud_rate > 0.0 else HUD_PUBLISH_RATE_HZ),
            self._render,
        )

    def _measurement_cb(self, key: str, msg: TargetMeasurements) -> None:
        # Cache only. One timer later renders both surfaces from one snapshot.
        with self._lock:
            self._latest[key] = (msg, self.get_clock().now().nanoseconds)

    def _truth_cb(self, msg: PointStamped) -> None:
        with self._lock:
            self._latest_truth = msg

    def _current_truth(self):
        """Return live benchmark truth, with age judged from its stamp."""

        with self._lock:
            msg = self._latest_truth
        return truth_reading(msg, self.get_clock().now().nanoseconds)

    def _partitioned_messages(self) -> tuple[list, list]:
        now_nanoseconds = self.get_clock().now().nanoseconds
        with self._lock:
            cached = list(self._latest.values())
        return partition_measurements(
            cached,
            now_nanoseconds,
            self.marker_lifetime,
            self.max_observation_age,
        )

    def _publish_markers(self, readings: dict) -> None:
        """Publish every selected estimator from one shared reading snapshot.

        Each reading is placed using TF at its own stamp. A missing position or
        transform costs only that estimator, and emits DELETE for its previous
        ring on the same tick that the HUD turns its column into a miss.
        """

        markers = []
        marker_time = self.get_clock().now().to_msg()
        for estimator in self.estimators:
            reading = readings.get(estimator)
            placed = False
            if reading is not None:
                robot_x, robot_y, robot_yaw, actual_frame = (
                    self._robot_pose_in_world(
                        rclpy.time.Time.from_msg(reading.stamp))
                )
                if robot_x is not None:
                    placed = append_estimator_markers(
                        markers,
                        estimator,
                        reading.forward_m,
                        reading.lateral_m,
                        robot_x,
                        robot_y,
                        robot_yaw,
                        marker_time,
                        actual_frame,
                        self.marker_lifetime,
                    )
            if not placed:
                append_delete_markers(markers, estimator)

        marker_array = MarkerArray()
        marker_array.markers = markers
        self._pub.publish(marker_array)

    def _render(self) -> None:
        """Read the cache once and render both HUD and markers from it."""

        same_batch, aged = self._partitioned_messages()
        nearest = (
            nearest_detection_index(batch_messages(same_batch))
            if same_batch else 0
        )
        readings = collect_readings(
            same_batch, aged, nearest, self.estimators)

        text = OverlayText()
        if self.hud_layout == HUD_LAYOUT_WIDE:
            text.text = hud_wide_text(readings, self.estimators)
        else:
            text.text = hud_section_from_readings(
                readings, self._current_truth(), self.estimators)
        self._hud_pub.publish(text)

        self._publish_markers(readings)

    def _robot_pose_in_world(
        self,
        stamp: rclpy.time.Time,
    ) -> tuple[float | None, float | None, float | None, str | None]:
        """Look up the base pose at an observation stamp without blocking."""

        base_frames = [self.base_frame]
        if self.base_frame != 'base_link':
            base_frames.append('base_link')
        for world_frame in (self.world_frame, 'odom'):
            for base_frame in base_frames:
                try:
                    # This listener shares the node's single-threaded executor,
                    # so a non-zero wait would block the /tf callback that could
                    # satisfy it. The reading stamp is already in the past: the
                    # transform is either buffered or too old, never pending.
                    transform = self.tf_buffer.lookup_transform(
                        world_frame,
                        base_frame,
                        stamp,
                        timeout=rclpy.duration.Duration(seconds=0.0),
                    )
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    yaw = math.atan2(
                        2.0 * (
                            rotation.w * rotation.z
                            + rotation.x * rotation.y
                        ),
                        1.0 - 2.0 * (
                            rotation.y * rotation.y
                            + rotation.z * rotation.z
                        ),
                    )
                    if world_frame != self.world_frame:
                        self.get_logger().warn(
                            f'"{self.world_frame}" frame unavailable; '
                            f'publishing estimate markers in "{world_frame}" frame.',
                            throttle_duration_sec=10.0,
                        )
                    return (
                        translation.x,
                        translation.y,
                        yaw,
                        world_frame,
                    )
                except TransformException:
                    continue
        return None, None, None, None


def main() -> None:
    rclpy.init()
    node = TargetVisualizationNode()
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
