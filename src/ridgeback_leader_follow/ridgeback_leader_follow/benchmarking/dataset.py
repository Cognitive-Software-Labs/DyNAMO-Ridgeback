"""Offline sensor access; evaluation annotations stay outside candidate adapters."""
import bisect
from pathlib import Path
import numpy as np
from ridgeback_leader_follow.fusion import rigid_matrix
from ridgeback_leader_follow.benchmarking.replay import load_schedule


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def image_array(msg):
    channels={'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4,'32FC1':1,'16UC1':1}
    if msg.encoding not in channels: raise ValueError(f'Unsupported image encoding {msg.encoding}')
    count=channels[msg.encoding]
    dtype=np.dtype(('>' if msg.is_bigendian else '<')+('f4' if msg.encoding == '32FC1' else 'u2' if msg.encoding == '16UC1' else 'u1'))
    data=np.frombuffer(bytes(msg.data),dtype=dtype).reshape(msg.height,msg.step//dtype.itemsize)
    data=data[:,:msg.width*count].reshape(msg.height,msg.width,count)
    if count == 1: return data[:,:,0].astype(np.float32)*(0.001 if msg.encoding == '16UC1' else 1.)
    data=data[:,:,:3]
    return data[:,:,::-1].copy() if msg.encoding.startswith('bgr') else data.copy()


class SensorBag:
    def __init__(self, run):
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from rclpy.duration import Duration
        from tf2_ros import Buffer
        self.run=Path(run)
        self.schedule=load_schedule(self.run)
        self.buffer=Buffer(cache_time=Duration(seconds=120.))
        self.samples={}
        reader=rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.run/'sensors'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
        static=0
        while reader.has_next():
            topic,data,_=reader.read_next()
            if not topic.endswith(('/tf','/tf_static','/image','/camera_info','/scan')): continue
            msg=deserialize_message(data,types[topic])
            if topic.endswith(('/tf','/tf_static')):
                for transform in msg.transforms:
                    if topic.endswith('/tf_static'):
                        self.buffer.set_transform_static(transform,'bag'); static+=1
                    else: self.buffer.set_transform(transform,'bag')
            else:
                # Keep image payloads serialized to avoid per-byte Python lists.
                self.samples.setdefault(topic,[]).append((stamp(msg),data if topic.endswith('/image') else msg))
        if not static: raise ValueError('No static TF in sensor bag')
        self.types=types
        for values in self.samples.values(): values.sort(key=lambda v:v[0])
        self.times={t:[s[0] for s in values] for t,values in self.samples.items()}

    def latest(self, suffix, at, max_age=.075):
        from rclpy.serialization import deserialize_message
        matches=[t for t in self.samples if t.endswith(suffix)]
        if len(matches) != 1: raise ValueError(f'Ambiguous or absent topic: {suffix}')
        topic=matches[0]
        i=bisect.bisect_right(self.times[topic],at+1e-9)-1
        if i < 0 or at-self.times[topic][i] > max_age: raise ValueError(f'Missing/stale {suffix}')
        timestamp,payload=self.samples[topic][i]
        msg=deserialize_message(payload,self.types[topic]) if isinstance(payload,bytes) else payload
        return timestamp,msg

    def transform(self, target, source, at):
        from rclpy.time import Time
        t=self.buffer.lookup_transform(target,source,Time(nanoseconds=round(at*1e9))).transform
        return rigid_matrix((t.translation.x,t.translation.y,t.translation.z),(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))

    def camera(self, at):
        rt,rgb=self.latest('/camera_0/color/image',at)
        dt,depth=self.latest('/camera_0/depth/image',rt)
        _,info=self.latest('/camera_0/color/camera_info',rt,max_age=.15)
        # The F1 sensor contract publishes depth registered into the color frame.
        # Arbitrary hardware depth must be registered upstream, never relabelled.
        if depth.header.frame_id != rgb.header.frame_id or (depth.width,depth.height) != (rgb.width,rgb.height):
            raise ValueError('RGB-D must be registered in the same optical frame')
        if info.header.frame_id != rgb.header.frame_id or (info.width,info.height) != (rgb.width,rgb.height):
            raise ValueError('Camera intrinsics do not match the color image')
        if any(abs(v) > 1e-8 for v in info.d):raise ValueError('Require rectified pinhole RGB-D inputs')
        if abs(rt-dt) > .04: raise ValueError('RGB-D timestamp mismatch')
        return rt,image_array(rgb),image_array(depth),np.array(info.k).reshape(3,3),self.transform('odom',rgb.header.frame_id,rt)
