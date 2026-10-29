#!/usr/bin/env python3
"""Bounded simulation readiness and demo scene setup. Never sends navigation goals.

The existing launch owns every process. A successful prepare exit permits its
one mission manager to start; release resumes the existing, initially latched
MissionGuard only after that manager has saved its start pose. Failure exits
nonzero and the launch shuts down its own processes.
"""

import argparse
import json
import math
import time

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatusArray
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import GetEntities
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from ridgeback_interfaces.msg import TargetDetections, TargetMeasurements
from ridgeback_localization.contracts import RAW_DETECTIONS_TOPIC, MASK_MEASUREMENTS_TOPIC
from ridgeback_autonomy.benchmarking.asset_catalog import load_asset_catalog
from ridgeback_autonomy.benchmarking.backends import benchmark_backend, asset_catalog_path
from ridgeback_autonomy.benchmarking.scene_control import create_scene_control_client, SpawnSpec


NAV2_NODES = ('controller_server', 'smoother_server', 'planner_server', 'behavior_server',
              'velocity_smoother', 'collision_monitor', 'bt_navigator')
SENSOR_FRAMES = ('lidar2d_0_laser', 'lidar2d_1_laser', 'camera_0_color_optical_frame')
DEMO_G1 = 'bench_parking_demo_g1'
DEMO_TABLE = 'bench_parking_demo_table'
FAILED_STATES = {'MISSION_FAILED', 'MISSION_CANCELED', 'MOTION_SETTLEMENT_FAILED',
                 'PARKING_PLANNING_FAILED', 'PARKING_EXECUTION_FAILED'}


def demo_specs(backend, *, share=get_package_share_directory):
    provider = benchmark_backend(backend)
    catalog = load_asset_catalog(asset_catalog_path(provider, package_share=share))
    # Origin heights follow the selected demo world's floor, not benchmark
    # calibration placement. The original SDF and ground-origin USD are reused.
    floor = 0.05 if backend == 'isaac' else 0.0
    return [SpawnSpec(DEMO_G1, catalog.targets['g1'].uri,
                      catalog.targets['g1'].resource_format, 3.0, 0.0, floor, math.pi),
            SpawnSpec(DEMO_TABLE, catalog.objects['table'].uri,
                      catalog.objects['table'].resource_format, 3.0, 1.05, floor, 0.0)]


def check_demo_entities(names):
    conflicts = [name for name in names if 'g1' in name.lower() or name == DEMO_TABLE]
    if conflicts:
        raise RuntimeError(f'demo refuses existing G1/table entities: {conflicts}; '
                           'start a fresh demo world; no existing entity was deleted')


def spawn_demo_scene(scene, specs, existing_names, spawned):
    check_demo_entities(existing_names)
    scene.require_capabilities(spec.resource_format for spec in specs)
    # The common client checks response result codes and verifies each entity
    # in simulator state before dispatching the next spawn.
    scene.spawn_entities(specs, spawned)


def missing_readiness(now, samples, clock_advanced, lifecycle, guard, *, hold_explorer):
    """Receipt freshness supplements publisher discovery; all deadlines use steady time."""
    missing = []
    for name in ('clock', 'front_scan', 'rear_scan', 'odometry', 'detections', 'measurements'):
        if name not in samples or now - samples[name] > 2.0:
            missing.append(f'fresh {name}')
    if not clock_advanced:
        missing.append('advancing /clock')
    missing += [f'{name} active' for name in NAV2_NODES
                if name not in lifecycle or lifecycle[name][0] != 3
                or now - lifecycle[name][1] > 3.0]
    # The stationary test stub is deliberately latched, without a heartbeat;
    # prepare verifies it with a fresh pause service response before proceeding.
    stationary = guard is not None and guard[0].get('ownership_mode') == 'stationary_mock'
    if guard is None or not stationary and now - guard[1] > 2.0:
        missing.append('fresh mission/status')
    elif hold_explorer and not guard[0].get('paused'):
        raise RuntimeError('demo explorer was not initially paused')
    return missing


class SimulationStartup(Node):
    def __init__(self, args):
        super().__init__('parking_startup_' + args.phase)
        self.args = args
        self.deadline = time.monotonic() + args.timeout
        self.samples, self.lifecycle, self.sample_stamps = {}, {}, {}
        self.pending = set()
        self.guard = self.mission = None
        self.active_goals = {}
        self.clock_ns = None
        self.clock_advanced = False
        self.clock_last_advanced_s = -math.inf
        self.spawned = []
        self.scene = None
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        for key, topic in (('front_scan', 'sensors/lidar2d_0/scan'),
                           ('rear_scan', 'sensors/lidar2d_1/scan')):
            self.create_subscription(LaserScan, topic,
                lambda m, k=key: self.on_sample(k, m), qos_profile_sensor_data)
        self.create_subscription(Odometry, args.odometry_topic,
                                 lambda m: self.on_sample('odometry', m), qos_profile_sensor_data)
        self.create_subscription(TargetDetections, RAW_DETECTIONS_TOPIC,
                                 lambda m: self.on_sample('detections', m), 10)
        self.create_subscription(TargetMeasurements, MASK_MEASUREMENTS_TOPIC,
                                 lambda m: self.on_sample('measurements', m), 10)
        self.create_subscription(String, 'mission/status', self.on_guard, latched)
        self.create_subscription(String, 'parking_mission/state', self.on_mission, latched)
        for topic in ('navigate_to_pose/_action/status', 'navigate_through_poses/_action/status'):
            self.create_subscription(GoalStatusArray, topic,
                lambda m, topic=topic: self.on_goals(topic, m), latched)
        self.pause = self.create_client(Trigger, 'mission/pause')
        self.resume = self.create_client(Trigger, 'mission/resume')
        self.start_mission = self.create_client(Trigger, 'parking_mission/start')
        self.lifecycle_clients = {name: self.create_client(GetState, name + '/get_state')
                                  for name in NAV2_NODES}

    def remaining(self):
        return max(0.0, self.deadline - time.monotonic())

    def call(self, client, request):
        if not client.service_is_ready():
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        future = client.call_async(request)
        until = time.monotonic() + min(5.0, self.remaining())
        while rclpy.ok() and not future.done() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            client.remove_pending_request(future)
            raise RuntimeError(f'service response deadline: {client.srv_name}')
        return future.result()

    def wait(self, missing):
        last = None
        while rclpy.ok():
            reasons = missing()
            if not reasons:
                return
            detail = ', '.join(reasons)
            if detail != last:
                self.get_logger().info('Waiting for ' + detail)
                last = detail
            if not self.remaining():
                raise RuntimeError('startup deadline expired; missing: ' + detail)
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError('ROS shut down during startup')

    def managers(self):
        ns = self.get_namespace().rstrip('/') or '/'
        return [name for name, namespace in self.get_node_names_and_namespaces()
                if name == 'mission_manager_node' and namespace == ns]

    def on_clock(self, msg):
        ns = msg.clock.sec * 10**9 + msg.clock.nanosec
        if self.clock_ns is not None and ns > self.clock_ns:
            self.clock_last_advanced_s = time.monotonic()
        self.clock_advanced = self.clock_advanced or (self.clock_ns is not None and ns > self.clock_ns)
        self.clock_ns = ns
        self.samples['clock'] = time.monotonic()

    def on_sample(self, key, msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        previous = self.sample_stamps.get(key)
        self.sample_stamps[key] = stamp
        if previous is None or stamp <= previous:
            return
        if key == 'odometry' and (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link'):
            return
        if key in ('front_scan', 'rear_scan'):
            frame = 'lidar2d_0_laser' if key == 'front_scan' else 'lidar2d_1_laser'
            if msg.header.frame_id != frame or not msg.ranges:
                return
        if self.clock_ns is not None and stamp > 0 and abs(self.clock_ns - stamp) <= 2_000_000_000:
            self.samples[key] = time.monotonic()

    def on_guard(self, msg):
        try:
            self.guard = (json.loads(msg.data), time.monotonic())
        except ValueError:
            pass

    def on_mission(self, msg):
        try:
            self.mission = json.loads(msg.data)
        except ValueError:
            pass

    def on_goals(self, topic, msg):
        self.active_goals[topic] = {tuple(s.goal_info.goal_id.uuid) for s in msg.status_list
                                    if s.status in (1, 2, 3)}

    def readiness(self):
        if self.managers():
            raise RuntimeError('mission manager already exists; refusing a second manager')
        if self.args.hold_explorer and any(self.active_goals.values()):
            raise RuntimeError('navigation was active before demo startup completed')
        now = time.monotonic()
        for name, client in self.lifecycle_clients.items():
            if (name not in self.pending and client.service_is_ready()
                    and (name not in self.lifecycle or now - self.lifecycle[name][1] > 1.0)):
                self.pending.add(name)
                def done(f, name=name):
                    self.pending.discard(name)
                    try:
                        self.lifecycle[name] = (f.result().current_state.id, time.monotonic())
                    except Exception:
                        self.lifecycle.pop(name, None)
                client.call_async(GetState.Request()).add_done_callback(done)
        clock_advancing = self.clock_advanced and now - self.clock_last_advanced_s <= 2.0
        missing = missing_readiness(now, self.samples, clock_advancing, self.lifecycle,
                                    self.guard, hold_explorer=self.args.hold_explorer)
        if not self.pause.service_is_ready():
            missing.append('MissionGuard mission/pause')
        if self.args.hold_explorer and not self.resume.service_is_ready():
            missing.append('MissionGuard mission/resume')
        for frame in SENSOR_FRAMES:
            try:
                self.tf_buffer.lookup_transform('base_link', frame, Time())
            except TransformException:
                missing.append('TF base_link -> ' + frame)
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            stamp = tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec
            if self.clock_ns is None or abs(self.clock_ns - stamp) > 2_000_000_000:
                missing.append('fresh TF map -> base_link')
        except TransformException:
            missing.append('TF map -> base_link')
        return missing

    def prepare(self):
        if self.managers():
            raise RuntimeError('mission manager already exists; refusing a second manager')
        if self.args.demo:
            provider = benchmark_backend(self.args.backend)
            self.scene = create_scene_control_client(self, provider.scene_control_service_namespace,
                call_timeout_sec=5.0, ready_timeout_sec=15.0)
            self.get_logger().info('Waiting for simulator scene services and advancing /clock')
            self.scene.wait_for_services(self.remaining())
            self.scene.wait_for_advancing_clock(self.remaining())
            specs = demo_specs(self.args.backend)
            entities = self.create_client(GetEntities,
                provider.scene_control_service_namespace + '/get_entities')
            response = self.call(entities, GetEntities.Request())
            if response.result.result != Result.RESULT_OK:
                raise RuntimeError('cannot inspect scene for duplicate G1: ' + response.result.error_message)
            spawn_demo_scene(self.scene, specs, response.entities, self.spawned)
            self.get_logger().info('Created and verified G1 and table: ' + ', '.join(self.spawned))
        self.wait(self.readiness)
        if self.args.hold_explorer or self.guard[0].get('ownership_mode') == 'stationary_mock':
            response = self.call(self.pause, Trigger.Request())
            if not response.success:
                raise RuntimeError('initial MissionGuard handoff not safe: ' + response.message)
        for frame in SENSOR_FRAMES:
            tf = self.tf_buffer.lookup_transform('base_link', frame, Time()).transform
            self.get_logger().info(f'Sensor TF {frame}: {tf}')
        self.get_logger().info('STARTUP_READY: clock, scene, sensors, perception, TF, Nav2 and handoff ready')

    def release(self):
        def missing():
            count = len(self.managers())
            if count > 1:
                raise RuntimeError(f'{count} mission managers found; exactly one required')
            if self.mission and self.mission.get('state') in FAILED_STATES:
                raise RuntimeError('mission failed before release: ' + str(self.mission))
            return ([] if count == 1 and self.mission and self.mission.get('start_pose_saved')
                    else ['exactly one mission manager with a saved start pose'])
        self.wait(missing)
        if self.args.demo and not self.mission.get('startup_paused'):
            raise RuntimeError('demo manager did not start with admission paused')
        if self.args.hold_explorer:
            # Only the existing guard's normal resume service releases the
            # startup latch; optional localization stays optional afterward.
            def resume_when_healthy():
                if self.mission.get('target_committed'):
                    return []  # manager owns the G1 handoff now; never resume over it
                if not self.resume.service_is_ready():
                    return ['mission/resume']
                if (not self.guard or self.guard[0].get('required')
                        and not self.guard[0].get('healthy')):
                    return ['advancing localization health before exploration release']
                response = self.call(self.resume, Trigger.Request())
                return [] if response.success else [response.message]
            self.wait(resume_when_healthy)
        if self.args.demo:
            self.wait(lambda: [] if self.start_mission.service_is_ready() else ['parking_mission/start'])
            response = self.call(self.start_mission, Trigger.Request())
            if not response.success:
                raise RuntimeError('mission startup release refused: ' + response.message)
        self.get_logger().info('MISSION_READY: exactly one manager; saved start pose; startup admission released')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', choices=('prepare', 'release'), required=True)
    parser.add_argument('--backend', choices=('gz', 'isaac'), required=True)
    parser.add_argument('--odometry-topic', required=True)
    parser.add_argument('--timeout', type=float, default=300.0)
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--hold-explorer', action='store_true')
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    rclpy.init(args=ros_args)
    node = SimulationStartup(args)
    code = 0
    try:
        getattr(node, args.phase)()
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 130
    except Exception as exc:
        node.get_logger().error(f'STARTUP_FAILED: {exc}')
        code = 1
    finally:
        if code and node.scene and node.spawned and rclpy.ok():
            for error in node.scene.rollback(node.spawned, deadline=time.monotonic() + 3.0):
                node.get_logger().warning(str(error))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
