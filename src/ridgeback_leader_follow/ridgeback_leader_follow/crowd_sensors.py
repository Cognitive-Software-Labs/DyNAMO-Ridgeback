"""F1 sensor readiness/evidence; ground truth never enters perception topics."""
import json
import math
from pathlib import Path

import numpy as np


class SensorEvidence:
    def __init__(self, output, namespace='r100_0001', record=False, odom_topic='platform/odom'):
        import rclpy
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        from sensor_msgs.msg import Image, CameraInfo, LaserScan
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node('f1_sensor_evidence', namespace=namespace)
        self.output = Path(output)
        (self.output / 'frames').mkdir()
        self.epoch = None
        self.stats = {}
        from tf2_ros import Buffer
        self.tf_buffer = Buffer()
        self.saved = set()
        self.writer = None
        self.topics = {
            'rgb': ('sensors/camera_0/color/image', Image),
            'depth': ('sensors/camera_0/depth/image', Image),
            'camera_info': ('sensors/camera_0/color/camera_info', CameraInfo),
            'front_scan': ('sensors/lidar2d_0/scan', LaserScan),
            'rear_scan': ('sensors/lidar2d_1/scan', LaserScan),
            'odom': (odom_topic, Odometry),
            'tf': ('tf', TFMessage), 'tf_static': ('tf_static', TFMessage),
        }
        if record:
            import rosbag2_py
            self.writer = rosbag2_py.SequentialWriter()
            self.writer.open(rosbag2_py.StorageOptions(uri=str(self.output / 'sensors'), storage_id='sqlite3'),
                             rosbag2_py.ConverterOptions('', ''))
        for index, (key, (topic, msg_type)) in enumerate(self.topics.items()):
            absolute = f'/{namespace}/{topic}'
            if self.writer:
                import rosbag2_py
                type_name = msg_type.__module__.split('.')[0] + '/msg/' + msg_type.__name__
                self.writer.create_topic(rosbag2_py.TopicMetadata(id=index, name=absolute,
                    type=type_name, serialization_format='cdr'))
            qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL) if key == 'tf_static' else qos_profile_sensor_data
            self.node.create_subscription(msg_type, topic,
                lambda msg, k=key, t=absolute: self.receive(k, t, msg), qos)

    def receive(self, key, topic, msg):
        from rclpy.serialization import serialize_message
        stamp = msg.header.stamp if hasattr(msg, 'header') else (
            msg.transforms[0].header.stamp if msg.transforms else None)
        if stamp is None:
            return
        now = stamp.sec + stamp.nanosec * 1e-9
        stat = self.stats.setdefault(key, {'count': 0, 'first_stamp': now,
                                          'last_stamp': now, 'valid_count': 0, 'run_count': 0})
        stat['count'] += 1
        stat['last_stamp'] = now
        valid = True
        if key in ('rgb', 'depth'):
            valid = msg.width > 0 and msg.height > 0 and len(msg.data) >= msg.height * msg.step
            stat.update(width=msg.width, height=msg.height, encoding=msg.encoding, frame=msg.header.frame_id)
            if valid and key == 'depth':
                dtype = np.dtype('>f4' if msg.is_bigendian else '<f4') if msg.encoding == '32FC1' else np.dtype('>u2' if msg.is_bigendian else '<u2')
                values = np.frombuffer(bytes(msg.data), dtype=dtype)
                valid = bool(np.any(np.isfinite(values) & (values > 0)))
        elif key.endswith('scan'):
            valid = len(msg.ranges) > 0 and any(math.isfinite(v) and v > 0 for v in msg.ranges)
            stat.update(bins=len(msg.ranges), frame=msg.header.frame_id)
        elif key == 'camera_info':
            valid = msg.width > 0 and msg.k[0] > 0
            if valid and not (self.output / 'camera_info.json').exists():
                (self.output / 'camera_info.json').write_text(json.dumps({
                    'width': msg.width, 'height': msg.height, 'k': list(msg.k),
                    'p': list(msg.p), 'frame': msg.header.frame_id}, indent=2) + '\n')
        if key in ('tf','tf_static') and hasattr(self,'tf_buffer'):
            for transform in msg.transforms:
                if key == 'tf_static':self.tf_buffer.set_transform_static(transform,'sensor-evidence')
                else:self.tf_buffer.set_transform(transform,'sensor-evidence')
        if valid:
            stat['valid_count'] += 1
        if self.writer:
            # Preserve warmup as well as latched extrinsics. Otherwise the
            # first evaluation tick may have no preceding scan in the bag.
            self.writer.write(topic, serialize_message(msg), int(now * 1e9))
        if self.epoch is not None and now >= self.epoch:
            stat['run_count'] += 1
            if key in ('rgb', 'depth', 'front_scan', 'rear_scan') and valid:
                elapsed = now - self.epoch
                for target in (0, 6, 10, 14, 19):
                    if elapsed >= target and (key, target) not in self.saved:
                        self.saved.add((key, target))
                        path = self.output / 'frames' / f'{key}_{target:02d}s'
                        if key == 'rgb':
                            from PIL import Image
                            channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
                            pixels = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
                            pixels = pixels[:, :msg.width * channels].reshape(msg.height, msg.width, channels)
                            if msg.encoding.startswith('bgr'):
                                pixels = pixels[:, :, [2, 1, 0]]
                            Image.fromarray(pixels[:, :, :3]).save(path.with_suffix('.png'))
                        elif key == 'depth':
                            np.savez_compressed(path.with_suffix('.npz'), data=np.frombuffer(bytes(msg.data),dtype=dtype).reshape(msg.height,-1)[:,:msg.width],
                                                stamp=now, scenario_time=elapsed, encoding=msg.encoding)
                        else:
                            path.with_suffix('.json').write_text(json.dumps({
                                'stamp': now, 'scenario_time': elapsed,
                                'frame': msg.header.frame_id, 'angle_min': msg.angle_min,
                                'angle_increment': msg.angle_increment,
                                'ranges': [v if math.isfinite(v) else None for v in msg.ranges]},indent=2)+'\n')

    @property
    def ready(self):
        required = ('rgb', 'depth', 'camera_info', 'front_scan', 'rear_scan', 'odom', 'tf_static')
        counts=all(self.stats.get(key, {}).get('valid_count', 0) >= (1 if key == 'tf_static' else 3)
                   for key in required)
        if not counts:return False
        from rclpy.time import Time
        return all(self.tf_buffer.can_transform('odom',self.stats[key]['frame'],Time())
                   for key in ('rgb','depth','front_scan','rear_scan'))

    def spin(self, timeout=.01):
        self.rclpy.spin_once(self.node, timeout_sec=timeout)

    def close(self):
        if self.writer:
            self.writer.close()
        (self.output / 'sensor_health.json').write_text(json.dumps(self.stats, indent=2) + '\n')
        if getattr(self, '_executor', None) is not None:
            self._executor.shutdown()
        self.node.destroy_node()
