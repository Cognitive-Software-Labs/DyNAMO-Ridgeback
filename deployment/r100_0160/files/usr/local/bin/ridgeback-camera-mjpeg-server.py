#!/usr/bin/env python3
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

NS = '/r100_0160'
PORT = 8081
STREAMS = {
    '/color.mjpg': (f'{NS}/camera/color/image_raw', 'color'),
    '/infrared.mjpg': (f'{NS}/camera/infra1/image_rect_raw', 'mono'),
    '/depth.mjpg': (f'{NS}/camera/depth/image_rect_mono8', 'mono'),
}
latest = {path: None for path in STREAMS}
latest_lock = threading.Lock()

class CameraMjpegNode(Node):
    def __init__(self):
        super().__init__('ridgeback_camera_mjpeg_server')
        self.bridge = CvBridge()
        for path, (topic, mode) in STREAMS.items():
            self.create_subscription(Image, topic, self._callback(path, mode), qos_profile_sensor_data)
            self.get_logger().info(f'{path} <- {topic}')

    def _callback(self, path, mode):
        def cb(msg):
            try:
                if mode == 'color':
                    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                else:
                    frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                ok, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    with latest_lock:
                        latest[path] = jpeg.tobytes()
            except Exception as exc:
                self.get_logger().warning(f'failed to encode {path}: {exc}')
        return cb

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path in ('/', '/health'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            self.wfile.write(b'ok\n')
            return
        if self.path not in STREAMS:
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Age', '0')
        self.send_header('Cache-Control', 'no-cache, private')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            with latest_lock:
                frame = latest.get(self.path)
            if frame is None:
                time.sleep(0.05)
                continue
            try:
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
                time.sleep(1.0 / 30.0)
            except (BrokenPipeError, ConnectionResetError):
                break

def main():
    rclpy.init()
    node = CameraMjpegNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    node.get_logger().info(f'MJPEG server listening on 0.0.0.0:{PORT}')
    try:
        server.serve_forever()
    finally:
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
