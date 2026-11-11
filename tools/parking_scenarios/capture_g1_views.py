"""Record what the robot camera sees of G1 from given robot poses, for offline review.

Usage (inside a running `start_parking_demo.sh gz --plan-only` session, DDS domain 83):
  capture_g1_views.py OUT_DIR NAME:X:Y:YAW_DEG [NAME:X:Y:YAW_DEG ...]

For each pose the robot model is moved there with Gazebo's set_pose service, and once
it has settled the tool saves OUT_DIR/NAME.npz: five aligned depth frames (millimetres,
uint16), the colour frame, the camera intrinsics, the camera's world pose (Gazebo
ground truth of the robot composed with the robot's own static camera transform), the
detector's boxes and scores from the measurement topic, and G1's ground-truth pose.
Ground truth is stored only to evaluate localization offline; nothing here is sent to
the mission. The robot is moved, so the session's SLAM map is not usable afterwards.
"""
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from ridgeback_interfaces.msg import TargetMeasurements

NS = 'r100_0001'
ROBOT_MODEL, G1_MODEL = 'r100_0001/robot', 'bench_parking_demo_g1'
GZ_ENV = dict(os.environ, GZ_PARTITION=f'ridgeback-parking-{os.getuid()}-83', GZ_IP='127.0.0.1')
NUM = r'(-?[\d.]+(?:e-?\d+)?)'


def gz_world():
    out = subprocess.run(['gz', 'service', '-l'], env=GZ_ENV, capture_output=True,
                         text=True, timeout=20).stdout
    return next(line.split('/')[2] for line in out.splitlines()
                if line.startswith('/world/') and line.endswith('/create'))


def gz_pose(model):
    """(x, y, z, roll, pitch, yaw) of a model from `gz model --pose`."""
    txt = subprocess.run(['gz', 'model', '-m', model, '--pose'], env=GZ_ENV,
                         capture_output=True, text=True, timeout=10).stdout
    v = re.findall(r'\[\s*' + r'\s+'.join([NUM] * 3) + r'\s*\]', txt)
    return tuple(float(a) for a in v[0]) + tuple(float(a) for a in v[1])


def set_robot_pose(world, x, y, z, yaw):
    q = (math.cos(yaw / 2.0), math.sin(yaw / 2.0))
    req = (f'name: "{ROBOT_MODEL}" position {{x: {x} y: {y} z: {z}}} '
           f'orientation {{w: {q[0]} z: {q[1]}}}')
    subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req],
                   env=GZ_ENV, capture_output=True, text=True, timeout=20, check=True)


def rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    poses = [(n, float(x), float(y), math.radians(float(yd)))
             for n, x, y, yd in (a.split(':') for a in sys.argv[2:])]
    rclpy.init()
    node = rclpy.create_node('g1_view_probe', namespace=NS, cli_args=[
        '--ros-args', '-r', '/tf:=/r100_0001/tf', '-r', '/tf_static:=/r100_0001/tf_static'])
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    state = {'depth': [], 'color': None, 'info': None, 'meas': []}
    best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    for qos in (10, best_effort):
        node.create_subscription(Image, 'sensors/camera_0/depth/image',
                                 lambda m: state['depth'].append(m), qos)
        node.create_subscription(Image, 'sensors/camera_0/color/image',
                                 lambda m: state.__setitem__('color', m), qos)
        node.create_subscription(CameraInfo, 'sensors/camera_0/color/camera_info',
                                 lambda m: state.__setitem__('info', m), qos)
    node.create_subscription(TargetMeasurements, 'measurements/target/mask',
                             lambda m: state['meas'].append(m), 10)
    world = gz_world()
    deadline = time.time() + 30.0
    camera = None
    while camera is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            camera = buffer.lookup_transform('base_link', 'camera_0_color_optical_frame',
                                             rclpy.time.Time())
        except tf2_ros.TransformException:
            pass
    if camera is None:
        raise RuntimeError('no base_link -> camera_0_color_optical_frame transform')
    base_cam_r = quat_matrix(camera.transform.rotation)
    t = camera.transform.translation
    base_cam_t = np.array([t.x, t.y, t.z])
    g1 = gz_pose(G1_MODEL)
    z0 = gz_pose(ROBOT_MODEL)[2]
    for name, x, y, yaw in poses:
        set_robot_pose(world, x, y, z0, yaw)
        settle = time.time() + 4.0
        while time.time() < settle:
            rclpy.spin_once(node, timeout_sec=0.1)
        state['depth'].clear()
        state['meas'].clear()
        stop = time.time() + 10.0
        while (len(state['depth']) < 5 or len(state['meas']) < 5) and time.time() < stop:
            rclpy.spin_once(node, timeout_sec=0.1)
        robot = gz_pose(ROBOT_MODEL)
        world_base = rotation(*robot[3:])
        cam_r = world_base @ base_cam_r
        cam_t = world_base @ base_cam_t + np.array(robot[:3])
        frames = []
        for msg in state['depth'][-5:]:
            if msg.encoding == '32FC1':
                d = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.width)
            else:
                d = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.width) * 1e-3
            frames.append(np.where(np.isfinite(d) & (d > 0), np.round(d * 1000.0), 0).astype(np.uint16))
        color = state['color']
        rgb = np.frombuffer(bytes(color.data), np.uint8).reshape(color.height, color.width, -1)[:, :, :3]
        if color.encoding.startswith('bgr'):
            rgb = rgb[:, :, ::-1]
        boxes = [list(m.bbox_xyxy) for m in state['meas'][-5:] if m.detected]
        scores = [list(m.scores) for m in state['meas'][-5:] if m.detected]
        k = state['info'].k
        np.savez_compressed(
            os.path.join(out, f'{name}.npz'), depth_mm=np.stack(frames), rgb=rgb,
            intrinsics=np.array([k[0], k[4], k[2], k[5]]), camera_rotation=cam_r,
            camera_translation=cam_t, robot_truth=np.array(robot), g1_truth=np.array(g1),
            boxes=np.array([b[:4] for b in boxes] or np.zeros((0, 4))),
            scores=np.array([s[0] for s in scores] or np.zeros(0)))
        print(f'{name}: robot {robot[0]:.2f},{robot[1]:.2f},{math.degrees(robot[5]):.0f} deg; '
              f'{len(frames)} depth frames; boxes {boxes[-1:] if boxes else "none"} '
              f'scores {[round(s[0], 3) for s in scores]}', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
