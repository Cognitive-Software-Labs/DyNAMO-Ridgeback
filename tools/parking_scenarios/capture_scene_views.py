"""Capture documentation views of a running parking scene, independent of any desktop.

Usage: capture_scene_views.py gz|isaac SCENARIO OUT_DIR [--stage LABEL] [--views a,b]
Run with the workspace sourced, the demo's DDS settings (domain 83, loopback) and,
for Gazebo, its partition. Gazebo views come from temporary camera sensors that the
Gazebo server renders and that are removed again; Isaac views come from the runner's
opt-in view cameras (RIDGEBACK_ISAAC_VIEW_CAMERAS, written by scene_views.py). The
robot's own camera frame is saved as what the robot sees. Every image is checked
(not blank, not flat) and listed in OUT_DIR/manifest.json with its source.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import scene_views  # noqa: E402

GZ_ENV = dict(os.environ, GZ_PARTITION=f'ridgeback-parking-{os.getuid()}-83', GZ_IP='127.0.0.1')


def save_png(path, rgb):
    import cv2
    cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))


def check(rgb):
    """(ok, reason): rejects black, saturated or flat frames."""
    gray = rgb.mean(axis=2)
    mean, spread = float(gray.mean()), float(gray.std())
    if mean < 12:
        return False, f'black frame (mean {mean:.1f})'
    if mean > 250:
        return False, f'white frame (mean {mean:.1f})'
    if spread < 6:
        return False, f'flat frame (std {spread:.1f})'
    return True, f'mean {mean:.0f}, std {spread:.0f}'


def ros_image(topic, timeout_s=60.0, skip=2):
    """One RGB frame from a ROS image topic in the demo namespace."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node('parking_view_capture', namespace='r100_0001')
    frames = []
    node.create_subscription(Image, topic, frames.append, qos_profile_sensor_data)
    deadline = time.monotonic() + timeout_s
    while len(frames) <= skip and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    if len(frames) <= skip:
        raise TimeoutError(f'no image on {topic}')
    msg = frames[-1]
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}[msg.encoding]
    img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    img = img[:, :msg.width * channels].reshape(msg.height, msg.width, channels)[:, :, :3]
    return img[:, :, ::-1].copy() if msg.encoding.startswith('bgr') else img.copy()


def gz_world():
    out = subprocess.run(['gz', 'service', '-l'], env=GZ_ENV, capture_output=True,
                         text=True, timeout=20).stdout
    for line in out.splitlines():
        if line.startswith('/world/') and line.endswith('/create'):
            return line.split('/')[2]
    raise RuntimeError('no Gazebo world found in this partition')


def gz_service(world, name, reqtype, req):
    subprocess.run(['gz', 'service', '-s', f'/world/{world}/{name}', '--reqtype', reqtype,
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req],
                   env=GZ_ENV, capture_output=True, text=True, timeout=20, check=True)


def gz_image(world, view, timeout_s=30.0):
    """Spawn a temporary camera, take one rendered frame, remove the camera."""
    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node
    model, topic = f'parking_view_{view["name"]}', f'/parking_views/{view["name"]}'
    x, y, z, roll, pitch, yaw = scene_views.gz_pose(view)
    sdf = (f'<sdf version="1.9"><model name="{model}"><static>true</static>'
           f'<pose>{x} {y} {z} {roll} {pitch} {yaw}</pose><link name="link">'
           f'<sensor name="camera" type="camera"><always_on>1</always_on>'
           f'<update_rate>2</update_rate><topic>{topic}</topic><camera>'
           f'<horizontal_fov>{math.radians(view["hfov_deg"]):.4f}</horizontal_fov>'
           f'<image><width>{view["width"]}</width><height>{view["height"]}</height>'
           f'<format>R8G8B8</format></image><clip><near>0.05</near><far>120</far></clip>'
           f'</camera></sensor></link></model></sdf>')
    frames, ready = [], threading.Event()

    def on_image(msg):
        frames.append(msg)
        if len(frames) > 2:      # the first frames can precede the scene's lighting
            ready.set()
    os.environ.update(GZ_PARTITION=GZ_ENV['GZ_PARTITION'], GZ_IP=GZ_ENV['GZ_IP'])
    node = Node()
    node.subscribe(Image, topic, on_image)
    gz_service(world, 'create', 'gz.msgs.EntityFactory',
               'sdf: ' + json.dumps(sdf))
    try:
        if not ready.wait(timeout_s):
            raise TimeoutError(f'no rendered frame from {topic}')
    finally:
        gz_service(world, 'remove', 'gz.msgs.Entity', f'name: "{model}" type: MODEL')
    msg = frames[-1]
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3).copy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('backend', choices=['gz', 'isaac'])
    ap.add_argument('scenario')
    ap.add_argument('out_dir')
    ap.add_argument('--stage', default='scene', help='label stored with each image')
    ap.add_argument('--views', default='', help='comma-separated subset of view names')
    ap.add_argument('--no-robot-camera', action='store_true')
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    wanted = set(filter(None, args.views.split(',')))
    selected = [v for v in scene_views.views(args.backend, args.scenario)
                if not wanted or v['name'] in wanted]
    manifest_path = os.path.join(args.out_dir, 'manifest.json')
    manifest = json.load(open(manifest_path)) if os.path.exists(manifest_path) else []
    world = gz_world() if args.backend == 'gz' else None
    jobs = [(v['name'], v) for v in selected]
    if not args.no_robot_camera:
        jobs.append(('robot_camera', None))
    for name, view in jobs:
        try:
            if view is None:
                rgb = ros_image('sensors/camera_0/color/image')
                source = 'robot camera topic sensors/camera_0/color/image'
            elif args.backend == 'gz':
                rgb = gz_image(world, view)
                source = 'Gazebo server camera sensor (temporary, removed after capture)'
            else:
                rgb = ros_image(f'views/{name}/image')
                source = 'Isaac runner view camera (RIDGEBACK_ISAAC_VIEW_CAMERAS)'
        except Exception as exc:    # report and continue with the other views
            print(f'{name}: FAILED {exc}', flush=True)
            manifest.append({'view': name, 'stage': args.stage, 'ok': False, 'reason': str(exc)})
            continue
        ok, reason = check(rgb)
        filename = f'{args.backend}_{args.scenario}_{args.stage}_{name}.png'
        save_png(os.path.join(args.out_dir, filename), rgb)
        manifest.append({'view': name, 'stage': args.stage, 'file': filename, 'ok': ok,
                         'check': reason, 'backend': args.backend, 'scenario': args.scenario,
                         'source': source, 'time': time.time(),
                         'camera': None if view is None else
                         {k: view[k] for k in ('eye', 'target', 'hfov_deg')}})
        print(f'{name}: {"ok" if ok else "REJECTED"} ({reason}) -> {filename}', flush=True)
    with open(manifest_path, 'w') as stream:
        json.dump(manifest, stream, indent=1)


if __name__ == '__main__':
    main()
