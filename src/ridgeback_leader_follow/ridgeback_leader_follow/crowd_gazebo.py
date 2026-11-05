"""F1 Gazebo backend: native actors, production robot sensors, observed truth."""
import math
import os
import queue
import subprocess
import time

from ridgeback_leader_follow.crowd_geometry import gazebo_world
from ridgeback_leader_follow.crowd_sensors import SensorEvidence


def run(scenario, duration, output, writer, gui, record=False, scene_profile='baseline', environment=None):
    from gz.transport13 import Node
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V
    from ridgeback_leader_follow.crowd_runner import stop_process, write_sample

    if scene_profile == 'realistic':
        from ridgeback_leader_follow.realistic_scene import gazebo_world as realistic_world
        world = realistic_world(scenario, output / 'actors', environment or 'office')
    elif scene_profile == 'f3':
        from ridgeback_leader_follow.f3_scene import gait_speed, gazebo_actor_skins
        world = gazebo_world(scenario, gazebo_actor_skins(scenario, output / 'actors'), gait_speed())
    elif scene_profile == 'baseline':
        world = gazebo_world(scenario)
    else:
        raise ValueError(f'Gazebo does not implement scene profile {scene_profile!r}')
    (output / 'scene.sdf').write_text(world)
    monitor = SensorEvidence(output, record=record, odom_topic='platform/odom/filtered')
    transport = Node()
    messages = queue.Queue()
    transport.subscribe(Pose_V, '/leader_follow/ground_truth', messages.put)
    processes = []
    env = dict(os.environ)
    if not gui:
        env.pop('DISPLAY', None)
        env.pop('WAYLAND_DISPLAY', None)
    with (output / 'simulator.log').open('w') as log:
        try:
            processes.append(subprocess.Popen([
                'ros2', 'launch', 'ridgeback_leader_follow', 'crowd_gazebo.launch.py',
                f'world_file:={output / "scene.sdf"}', f'setup_path:={output / "setup"}/'],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            if gui:
                processes.append(subprocess.Popen(['gz', 'sim', '-g'], env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            deadline = time.monotonic() + 120.
            while not monitor.ready:
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
            ok, response = transport.request('/leader_follow/start', Empty(), Empty, Boolean, 10000)
            if not ok or not response.data:
                raise RuntimeError('Crowd start failed')
            by_name = {p.name: p for p in scenario.people}
            last = -1.
            wall_deadline = time.monotonic() + max(120., duration * 10)
            while last < duration - .001:
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
                    last = elapsed
            # Dispatch final sensor samples before writing health evidence.
            for _ in range(20):
                monitor.spin(.005)
        finally:
            monitor.close()
            for process in reversed(processes):
                stop_process(process)
