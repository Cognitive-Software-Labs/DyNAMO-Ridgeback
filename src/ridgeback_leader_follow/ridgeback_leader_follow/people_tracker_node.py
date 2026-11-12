#!/usr/bin/env python3
"""Sensor-only RGB-D/lidar fusion publishing People; never publishes motion."""
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from leader_follow_interfaces.msg import People, Person
from tf2_ros import Buffer, TransformListener, TransformException
from ridgeback_leader_follow.benchmarking.adapters import ScanInput, ScanClustering
from ridgeback_leader_follow.benchmarking.dataset import image_array, stamp
from ridgeback_leader_follow.benchmarking.detectors import CameraDetector
from ridgeback_leader_follow.benchmarking.lidar_detectors import lidar_detector
from ridgeback_leader_follow.image_association import ImageAssociation
from ridgeback_leader_follow.fusion import LidarMovers, Measurement, PeopleFusion, box_measurement, rigid_matrix


class PeopleTrackerNode(Node):
    def __init__(self):
        super().__init__('people_tracker')
        defaults={'camera_candidate':'owlv2','model_dir':'artifacts/leader-following/f2-models',
                  'appearance_association_weight':.75,'appearance_candidate':'none','appearance_model_dir':'artifacts/leader-following/f3-models',
                  'device':'cuda','lidar_candidate':'lfe-ppn','output_frame':'odom','maximum_sensor_age':.075,
                  'association_gate':.75,'track_maximum_age':.75,'observation_timeout':.15,
                  'rgb_topic':'sensors/camera_0/color/image','depth_topic':'sensors/camera_0/depth/image',
                  'camera_info_topic':'sensors/camera_0/color/camera_info',
                  'front_scan_topic':'sensors/lidar2d_0/scan','rear_scan_topic':'sensors/lidar2d_1/scan'}
        for key,value in defaults.items(): self.declare_parameter(key,value)
        p=lambda k:self.get_parameter(k).value
        self.output_frame=p('output_frame'); self.max_age=p('maximum_sensor_age')
        self.observation_timeout=float(p('observation_timeout'))
        self.detector=CameraDetector(p('camera_candidate'),p('model_dir'),p('device'))
        self.appearance_encoder=None
        if p('appearance_candidate') != 'none':
            from ridgeback_leader_follow.benchmarking.reid_models import official_encoder
            self.appearance_encoder=official_encoder(p('appearance_candidate'),p('appearance_model_dir'),p('device'))
        if self.appearance_encoder is not None:
            from ridgeback_leader_follow.benchmarking.reid import PersonCrop
            self.detector.detect(np.zeros((480,640,3),dtype=np.uint8))
            self.appearance_encoder.encode([PersonCrop(0,np.zeros((256,128,3),dtype=np.uint8))])
            self.get_logger().info('F3 inference warmup complete')
        self.image_association=ImageAssociation(self.detector.threshold)
        self.fusion=PeopleFusion(p('association_gate'),p('track_maximum_age'),
                                 appearance_weight=p('appearance_association_weight') if self.appearance_encoder is not None else 0.)
        self.support_detector=ScanClustering(maximum_width=.4) if self.appearance_encoder is not None else None
        self.lidars={key:lidar_detector(p('lidar_candidate'),p('model_dir'),p('device')) for key in ('front','rear')}
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.queues={k:deque(maxlen=30) for k in ('rgb','depth','info','front','rear')}
        topics={'rgb':(p('rgb_topic'),Image),'depth':(p('depth_topic'),Image),
                'info':(p('camera_info_topic'),CameraInfo),
                'front':(p('front_scan_topic'),LaserScan),'rear':(p('rear_scan_topic'),LaserScan)}
        self.subscriptions_=[]
        self.sensor_group=ReentrantCallbackGroup()
        for key,(topic,kind) in topics.items():
            self.subscriptions_.append(self.create_subscription(kind,topic,lambda m,k=key:self.queues[k].append(m),qos_profile_sensor_data,callback_group=self.sensor_group))
        self.publisher=self.create_publisher(People,'leader_follow/people',10)
        # Lidar-only walkers, for collision avoidance; never leader candidates.
        self.movers=LidarMovers()
        self.mover_publisher=self.create_publisher(People,'leader_follow/moving_people',10)
        self.last_rgb=-1.
        self.last_output=-1.
        self.create_timer(.02,self.tick)

    def latest(self,key,at):
        candidates=[m for m in list(self.queues[key]) if stamp(m) <= at+1e-9 and at-stamp(m) <= self.max_age]
        if not candidates: raise ValueError(f'Missing or stale {key}')
        return max(candidates,key=stamp)

    def transform(self,source,at):
        t=self.buffer.lookup_transform(self.output_frame,source,Time(nanoseconds=round(at*1e9))).transform
        return rigid_matrix((t.translation.x,t.translation.y,t.translation.z),(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))

    def prepare_inputs(self,rgb):
        at=stamp(rgb)
        depth=self.latest('depth',at); info=self.latest('info',at)
        if depth.header.frame_id != rgb.header.frame_id or (depth.width,depth.height) != (rgb.width,rgb.height):
            raise ValueError('Require depth registered to color optical frame')
        if info.header.frame_id != rgb.header.frame_id or (info.width,info.height) != (rgb.width,rgb.height):
            raise ValueError('Camera intrinsics do not match RGB')
        if any(abs(v) > 1e-8 for v in info.d):raise ValueError('Require rectified pinhole RGB-D inputs')
        if at-stamp(depth) > .04: raise ValueError('RGB-D skew exceeds 40 ms')
        matrix=self.transform(rgb.header.frame_id,at)
        scans=[self.latest(k,at) for k in ('front','rear')]
        scan_matrices=[self.transform(m.header.frame_id,stamp(m)) for m in scans]
        return depth,info,matrix,scans,scan_matrices

    def lidar_measurements(self,scans,scan_matrices):
        # Deduplicate overlapping raw lidars after transforming into the output frame.
        lidar_points=[];support_points=[]
        for key,scan,tf in zip(('front','rear'),scans,scan_matrices):
            sensor=ScanInput(stamp(scan),scan.header.frame_id,scan.angle_min,scan.angle_increment,
                             tuple(scan.ranges),scan.range_min,scan.range_max)
            for x,y in self.lidars[key].detect(sensor):
                point=(tf@np.array([x,y,0.,1.]))[:2]
                if all(np.linalg.norm(point-old) > .25 for old in lidar_points): lidar_points.append(point)
            if self.support_detector is not None:
                for x,y in self.support_detector.detect(sensor):
                    point=(tf@np.array([x,y,0.,1.]))[:2]
                    if all(np.linalg.norm(point-old) > .25 for old in support_points):support_points.append(point)
        return ([Measurement(float(x),float(y),.09,.5,'support') for x,y in support_points]+
                [Measurement(float(x),float(y),.09,.5,'lidar') for x,y in lidar_points])

    def lidar_only(self):
        """Publish observed scan tracks while RGB-D/TF synchronization catches up."""
        for scan in sorted(list(self.queues['front']),key=stamp,reverse=True):
            at=stamp(scan)
            if at <= self.last_output:continue
            try:
                rear=self.latest('rear',at)
                scans=[scan,rear]
                matrices=[self.transform(m.header.frame_id,stamp(m)) for m in scans]
            except (ValueError,TransformException):
                continue
            self.publish_measurements(at,self.lidar_measurements(scans,matrices))
            return True
        return False

    def tick(self):
        if not self.queues['rgb']: return
        newest=self.queues['rgb'][-1]; at=stamp(newest)
        if at < self.last_rgb-1.:
            self.fusion=PeopleFusion(self.fusion.gate,self.fusion.max_age,self.fusion.acceleration,self.fusion.appearance_weight)
            self.last_rgb=-1.;self.last_output=-1.
            self.image_association=ImageAssociation(self.detector.threshold)
        now=self.get_clock().now()
        error=None
        selected=None
        for candidate in sorted(list(self.queues['rgb']),key=stamp,reverse=True):
            if stamp(candidate) <= max(self.last_rgb,self.last_output):continue
            try:
                prepared=self.prepare_inputs(candidate)
            except (ValueError,TransformException) as exc:
                error=exc
                continue
            selected=(candidate,prepared)
            break
        if selected is None:
            if self.appearance_encoder is not None and self.lidar_only():return
            # A heartbeat must not invalidate an already complete sensor bundle.
            if now.nanoseconds*1e-9-self.last_output > self.observation_timeout:
                empty=People();empty.header.stamp=now.to_msg();empty.header.frame_id=self.output_frame
                self.publisher.publish(empty);self.last_output=now.nanoseconds*1e-9
            if error is not None:self.get_logger().warning(str(error),throttle_duration_sec=2.)
            return
        rgb,(depth,info,matrix,scans,scan_matrices)=selected
        at=stamp(rgb)
        self.last_rgb=at
        if self.fusion.stamp is not None and at <= self.fusion.stamp:
            self.fusion=PeopleFusion(self.fusion.gate,self.fusion.max_age,self.fusion.acceleration,self.fusion.appearance_weight)  # A simulator clock reset starts a new track session.
        measurements=[]
        pixels=image_array(rgb); metric=image_array(depth)
        detections=list(self.image_association.update(self.detector.detect(pixels),pixels.shape[:2]))
        features={}
        if self.appearance_encoder is not None:
            from ridgeback_leader_follow.benchmarking.reid import person_crops
            crops=person_crops(pixels,[(identity,box[:4]) for identity,box in detections],horizontal_margin=2)
            features=self.appearance_encoder.encode(crops)
        for camera_id,box in detections:
            m=box_measurement(box[:4],box[4],metric,info.k,matrix)
            if m is not None:
                m.camera_id=camera_id
                m.appearance=tuple(float(v) for v in features.get(camera_id,()))
                measurements.append(m)
        measurements.extend(self.lidar_measurements(scans,scan_matrices))
        self.publish_measurements(at,measurements)

    def publish_measurements(self,at,measurements):
        result=People(); result.header.stamp=Time(nanoseconds=round(at*1e9)).to_msg(); result.header.frame_id=self.output_frame
        for track in self.fusion.update(at,measurements):
            person=Person(); person.id=track.identity
            person.position.x,person.position.y=map(float,track.state[:2])
            person.velocity.x,person.velocity.y=map(float,track.state[2:])
            person.confidence=float(track.confidence); person.appearance=list(track.appearance)
            result.people.append(person)
        # Published first, so a consumer of both sees the movers for this stamp.
        moving=People(); moving.header=result.header
        for track in self.movers.update(at,[(m.x,m.y) for m in self.fusion.unmatched_lidar]):
            person=Person(); person.id=track.identity
            person.position.x,person.position.y=map(float,track.state[:2])
            person.velocity.x,person.velocity.y=map(float,track.state[2:])
            moving.people.append(person)
        self.mover_publisher.publish(moving)
        self.publisher.publish(result)
        self.last_output=at


def main():
    rclpy.init()
    node=None;executor=None
    try:
        node=PeopleTrackerNode()
        executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node);executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if executor is not None:executor.shutdown()
        if node is not None: node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__': main()
