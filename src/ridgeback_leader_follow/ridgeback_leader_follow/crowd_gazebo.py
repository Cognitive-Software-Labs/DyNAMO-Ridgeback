"""F1 Gazebo backend: native actors, production robot sensors, observed truth."""
import math
import os
import queue
import subprocess
import time

from ridgeback_leader_follow.crowd_geometry import gazebo_world
from ridgeback_leader_follow.crowd_sensors import SensorEvidence


def wait_for_follower(drive, monitor):
    """Keep the crowd at its start until the follow stack publishes, then settle."""
    node = monitor.node
    deadline = time.monotonic() + drive.startup_timeout
    while not (node.count_publishers('cmd_vel') > 0 and node.count_publishers('leader_follow/leader') > 0):
        if time.monotonic() > deadline:
            raise TimeoutError('Follow stack (cmd_vel and leader_follow/leader publishers) did not start')
        monitor.spin(.05)
    settle = time.monotonic() + drive.settle_seconds / drive.real_time_factor
    while time.monotonic() < settle:
        monitor.spin(.05)


ROBOT_MODEL = 'r100_0001/robot'


# Gazebo's mecanum wheel model reports roughly twice the true heading change
# when the base strafes while turning (measured 102 deg against 52 deg), and
# Clearpath's EKF takes absolute yaw from wheel odometry. Closed-loop runs
# therefore fuse only wheel velocities and take yaw from the IMU rate.
DRIVE_EKF = {
    'odom0_config': [False, False, False, False, False, False,
                     True, True, False, False, False, False,
                     False, False, False],
    'imu0': 'sensors/imu_0/data',
    'imu0_config': [False, False, False, False, False, False,
                    False, False, False, False, False, True,
                    True, False, False],
    'imu0_differential': False,
    'imu0_queue_size': 10,
    'imu0_remove_gravitational_acceleration': True,
}


def drive_localization(declaration: dict) -> dict:
    """Run-local robot declaration for Gazebo closed-loop runs (hardware config untouched)."""
    import copy
    result = copy.deepcopy(declaration)
    extras = result.setdefault('platform', {}).setdefault('extras', {})
    parameters = extras.get('ros_parameters') or {}
    if 'ekf_node' in parameters:
        raise ValueError('robot declaration already overrides ekf_node')
    extras['ros_parameters'] = {**parameters, 'ekf_node': dict(DRIVE_EKF)}
    return result


def apply_real_time_factor(world: str, factor: float) -> str:
    """Pace a generated world; closed-loop runs slow it so perception keeps up."""
    old = '<real_time_factor>1</real_time_factor>'
    if world.count(old) != 1:
        raise ValueError('generated world must declare exactly one real_time_factor of 1')
    return world.replace(old, f'<real_time_factor>{factor:g}</real_time_factor>')


def run(scenario, duration, output, writer, gui, record=False, scene_profile='baseline', environment=None, render_quality='reference', tuning=None, drive=None):
    """``drive`` (DriveOptions) holds the crowd until a follow stack publishes and records robot truth."""
    from ridgeback_leader_follow.simulation_performance import RuntimeTuning
    tuning=tuning or RuntimeTuning()
    from gz.transport13 import Node
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from ridgeback_leader_follow.crowd_runner import stop_process, write_sample

    if scene_profile == 'realistic':
        from ridgeback_leader_follow.realistic_scene import gazebo_world as realistic_world
        world = realistic_world(scenario, output / 'actors', environment or 'modern_office', render_quality, tuning)
    elif scene_profile == 'f3':
        from ridgeback_leader_follow.f3_scene import gait_speed, gazebo_actor_skins
        world = gazebo_world(scenario, gazebo_actor_skins(scenario, output / 'actors'), gait_speed())
    elif scene_profile == 'baseline':
        world = gazebo_world(scenario)
    else:
        raise ValueError(f'Gazebo does not implement scene profile {scene_profile!r}')
    if drive is not None:
        world = apply_real_time_factor(world, drive.real_time_factor)
    (output / 'scene.sdf').write_text(world)
    monitor = SensorEvidence(output, record=record, odom_topic='platform/odom/filtered')
    transport = Node()
    messages = queue.Queue()
    transport.subscribe(Pose_V, '/leader_follow/ground_truth', messages.put)
    robot_pose = {}
    last_command = [(0., 0., 0.)]
    if drive is not None:
        def on_poses(message):
            for pose in message.pose:
                if pose.name == ROBOT_MODEL:
                    q = pose.orientation
                    robot_pose['latest'] = (pose.position.x, pose.position.y,
                                            math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        transport.subscribe(Pose_V, '/world/leader_follow_scene/pose/info', on_poses)
        from geometry_msgs.msg import TwistStamped
        monitor.node.create_subscription(TwistStamped, 'cmd_vel', lambda m: last_command.__setitem__(
            0, (m.twist.linear.x, m.twist.linear.y, m.twist.angular.z)), 10)
    processes = []
    env = dict(os.environ)
    if not gui:
        env.pop('DISPLAY', None)
        env.pop('WAYLAND_DISPLAY', None)
    with (output / 'simulator.log').open('w') as log:
        try:
            processes.append(subprocess.Popen([
                'ros2', 'launch', 'ridgeback_leader_follow', 'crowd_gazebo.launch.py',
                f'world_file:={output / "scene.sdf"}', f'setup_path:={output / "setup"}/',
                f'camera_width:={tuning.camera_profile().width}',f'camera_height:={tuning.camera_profile().height}'],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            if gui:
                processes.append(subprocess.Popen(['gz', 'sim', '-g'], env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            deadline = time.monotonic() + 120.
            while not monitor.ready:
                if gui and processes[-1].poll() is not None:
                    raise RuntimeError('Gazebo GUI exited during startup; see simulator.log')
                monitor.spin(.02)
                if processes[0].poll() is not None:
                    raise RuntimeError('Gazebo bringup exited; see simulator.log')
                if time.monotonic() > deadline:
                    raise TimeoutError(f'Robot sensors did not become ready: {monitor.stats}')
            deadline = time.monotonic() + 15.
            while '/leader_follow/start' not in transport.service_list():
                if time.monotonic() > deadline:
                    raise TimeoutError('Crowd plugin start service unavailable')
                monitor.spin(.02)
            robot_truth = None
            if drive is not None:
                wait_for_follower(drive, monitor)
                robot_truth = (output / 'robot_truth.csv').open('w')
                robot_truth.write('stamp,scenario_time,x,y,yaw,cmd_vx,cmd_vy,cmd_wz\n')
            # The start service is idempotent; a busy host can time a request out.
            for _ in range(5):
                ok, response = transport.request('/leader_follow/start', Empty(), Empty, Boolean, 10000)
                if ok and response.data:
                    break
                monitor.spin(.2)
            else:
                raise RuntimeError('Crowd start failed after 5 requests')
            by_name = {p.name: p for p in scenario.people}
            last = -1.
            wall_deadline = time.monotonic() + max(120., duration * 10 / (drive.real_time_factor if drive else 1.))
            while last < duration - .001:
                if gui and processes[-1].poll() is not None:
                    raise RuntimeError('Gazebo GUI exited during playback; see simulator.log')
                monitor.spin(.01)
                if time.monotonic() > wall_deadline:
                    raise TimeoutError('Crowd scenario stalled')
                while not messages.empty():
                    message = messages.get_nowait()
                    elapsed = float(next(d.value[0] for d in message.header.data
                                         if d.key == 'scenario_time'))
                    stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
                    monitor.epoch = stamp - elapsed
                    if elapsed <= last:
                        continue
                    if set(p.name for p in message.pose) != set(by_name):
                        raise RuntimeError('Simulator truth is missing a person')
                    for p in message.pose:
                        q = p.orientation
                        yaw = math.atan2(2 * (q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                        write_sample(writer, by_name[p.name], stamp, elapsed, min(elapsed,scenario.duration),
                                     (p.position.x,p.position.y,p.position.z,yaw))
                    if robot_truth is not None:
                        if 'latest' not in robot_pose:
                            raise RuntimeError(f'No {ROBOT_MODEL} pose on the world pose stream')
                        x, y, robot_yaw = robot_pose['latest']
                        robot_truth.write(f'{stamp:.6f},{elapsed:.6f},{x:.6f},{y:.6f},{robot_yaw:.6f},'
                                          + ','.join(f'{v:.6f}' for v in last_command[0]) + '\n')
                    last = elapsed
            if robot_truth is not None:
                robot_truth.close()
            # Dispatch final sensor samples before writing health evidence.
            for _ in range(20):
                monitor.spin(.005)
        finally:
            monitor.close()
            for process in reversed(processes):
                stop_process(process)
