#!/usr/bin/env python3
"""Observe acquisition progress independently of potentially blocked workers."""
import json
import time
import uuid
import rclpy
from ridgeback_common.ros_signals import init_owned, shielded, spin_until_interrupted
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan, PointCloud2
from std_msgs.msg import String
from ridgeback_interfaces.msg import TargetDetections, TargetMeasurements, LocalizationHealth
from ridgeback_common.stamps import stamp_to_nanoseconds
from ridgeback_localization.contracts import (
    RAW_DETECTIONS_TOPIC, MASK_MEASUREMENTS_TOPIC, POINTCLOUD_MEASUREMENTS_TOPIC,
    HEALTH_TOPIC, WORKER_STATE_TOPIC,
)
from ridgeback_localization.estimator_registry import parse_estimators, uses_mask_estimators, uses_pointcloud_estimators
from ridgeback_localization.health import PipelineHealth


def liveness_input(image_topic, info_topic):
    """(type, topic) that proves an image stream is advancing, without its payload.

    Health only needs each frame's stamp. On every backend a camera image
    ``.../image`` is published beside ``.../camera_info`` with the same stamps, so
    watching the info keeps a full copy of colour and depth off the network:
    on r100_0160 the image subscriptions alone carried about 370 Mbit/s from
    Intel to Thor. A worker that stops receiving its images still shows up as a
    stalled output. A topic without that sibling keeps the image subscription.
    """

    if info_topic:
        return CameraInfo, info_topic
    if image_topic.endswith('/image'):
        return CameraInfo, image_topic[:-len('image')] + 'camera_info'
    return Image, image_topic


class LocalizationHealthNode(Node):
    def __init__(self, **kwargs):
        super().__init__('localization_health', **kwargs)
        defaults = dict(mode='local', estimators='all', target_labels='humanoid robot',
            color_topic='sensors/camera_0/color/image', depth_topic='sensors/camera_0/depth/image',
            camera_info_topic='sensors/camera_0/color/camera_info', scan_topic='sensors/lidar2d_0/scan',
            pointcloud_topic='sensors/camera_0/points', depth_source='stereoscopic', mask_gate='box',
            startup_timeout=120.0, input_timeout=3.0, progress_timeout=15.0, source_age=15.0)
        for name, value in defaults.items(): self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self.mode, self.labels = p('mode'), [v.strip() for v in p('target_labels').split(',') if v.strip()]
        selected = parse_estimators(p('estimators'))
        self.estimators, self.depth_source, self.mask_gate = list(selected), p('depth_source'), p('mask_gate')
        inputs = {'color': liveness_input(p('color_topic'), p('camera_info_topic'))}
        outputs = {'detections': (TargetDetections, RAW_DETECTIONS_TOPIC)}
        components = ['detector']
        if uses_mask_estimators(selected):
            inputs['camera_info'] = (CameraInfo, p('camera_info_topic'))
            if 'polar_profiling' in selected: inputs['scan'] = (LaserScan, p('scan_topic'))
            if any(x in selected for x in ('projective_ranging', 'euclidean_reconstruction')) and p('depth_source') == 'stereoscopic':
                inputs['depth'] = liveness_input(p('depth_topic'), '')
            outputs['mask'] = (TargetMeasurements, MASK_MEASUREMENTS_TOPIC)
            components.append('mask')
        if uses_pointcloud_estimators(selected):
            inputs['pointcloud'] = (PointCloud2, p('pointcloud_topic'))
            outputs['pointcloud_measurement'] = (TargetMeasurements, POINTCLOUD_MEASUREMENTS_TOPIC)
            components.append('pointcloud')
        self.health = PipelineHealth(inputs, outputs, now=time.monotonic(), **{
            key: float(p(key)) for key in ('startup_timeout','input_timeout','progress_timeout','source_age')})
        self.components = components
        self.epoch = uuid.uuid4().hex
        self.worker_epochs = {}
        self.subscriptions_ = []
        for name, (typ, topic) in {**inputs, **outputs}.items():
            self.subscriptions_.append(self.create_subscription(typ, topic,
                lambda msg, name=name: self.health.observe(name, stamp_to_nanoseconds(msg.header.stamp), time.monotonic()),
                qos_profile_sensor_data if name in inputs else 10))
        for component in components:
            self.subscriptions_.append(self.create_subscription(String, WORKER_STATE_TOPIC+'/'+component,
                self.worker_event, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.pub = self.create_publisher(LocalizationHealth, HEALTH_TOPIC, 10)
        self.create_timer(0.5, self.publish_health, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def worker_event(self, message):
        try:
            event = json.loads(message.data)
            component = event['component']
            if component not in self.components:
                raise ValueError('Unknown worker')
            self.worker_epochs[component] = event['epoch']
            if event['state'] == 'failure': self.health.failures[component] = event['detail']
            else: self.health.failures.pop(component, None)
        except (ValueError, KeyError, TypeError):
            self.get_logger().error('Invalid localization worker state')

    def publish_health(self):
        from builtin_interfaces.msg import Time
        for component in self.components:
            count = self.count_publishers(WORKER_STATE_TOPIC + '/' + component)
            key = 'process:' + component
            if count > 1:
                self.health.failures[key] = 'Duplicate worker publishers'
            elif count == 0 and (component in self.worker_epochs or
                    time.monotonic() - self.health.started >= self.health.startup_timeout):
                self.health.failures[key] = 'Worker publisher missing'
            else:
                self.health.failures.pop(key, None)
        msg = LocalizationHealth()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.session_id = self.epoch + ':' + ':'.join(f'{k}={v}' for k,v in sorted(self.worker_epochs.items()))
        msg.mode, msg.labels = self.mode, self.labels
        msg.estimators, msg.depth_source, msg.mask_gate = self.estimators, self.depth_source, self.mask_gate
        msg.state, msg.detail = self.health.state(time.monotonic(), self.get_clock().now().nanoseconds)
        for name, stream in self.health.streams.items():
            msg.streams.append(name)
            msg.progress.append(stream.count)
            stamp = max(0, stream.stamp)
            msg.source_stamps.append(Time(sec=stamp//10**9, nanosec=stamp%10**9))
        self.pub.publish(msg)


def main():
    init_owned()
    node = LocalizationHealthNode()
    try: spin_until_interrupted(node)
    except KeyboardInterrupt: pass
    finally:
        with shielded(final=True):
            node.destroy_node()
            rclpy.try_shutdown()

if __name__ == '__main__': main()
