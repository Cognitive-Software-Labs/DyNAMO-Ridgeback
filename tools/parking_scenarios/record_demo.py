"""Record one parking run as video frames: Isaac view cameras, RViz and the mission state.

Usage (DDS domain 83, alongside run_scenario.sh with ISAAC_VIEWS including the views):
  record_demo.py OUT_DIR RUN_NAME [VIEW ...]        (views default: task isometric)

Twice a second it composes one frame from the latest image of each Isaac view camera
(views/<name>/image, published by the runner's opt-in view cameras), a grab of the X
display (RViz) and a caption with the run, the mission time and the mission state
from parking_mission/state. All parts come from the same run at the same moment;
nothing is reordered or spliced. It stops on SIGINT/SIGTERM or MISSION_COMPLETE /
MISSION_FAILED plus 10 s. make_demo_video.sh turns the frames into an MP4.
"""
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageGrab
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import String

WIDTH, VIEW_H, CAPTION_H, RVIZ_H = 1920, 600, 44, 600
END_STATES = ('MISSION_COMPLETE', 'MISSION_FAILED', 'PARKING_PLANNING_FAILED',
              'MOTION_SETTLEMENT_FAILED', 'PARKING_EXECUTION_FAILED', 'MISSION_CANCELED')


def to_pil(msg):
    channels = 4 if msg.encoding in ('rgba8', 'bgra8') else 3
    data = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    data = data[:, :msg.width * channels].reshape(msg.height, msg.width, channels)[:, :, :3]
    if msg.encoding.startswith('bgr'):
        data = data[:, :, ::-1]
    return Image.fromarray(np.ascontiguousarray(data))


def rviz_window(raise_it=False):
    """(x, y, width, height) of this user's newest visible RViz window, or None."""
    try:
        pid = subprocess.run(['pgrep', '-n', '-u', str(os.getuid()), '-x', 'rviz2'],
                             capture_output=True, text=True, timeout=5).stdout.split()
        if not pid:
            return None
        ids = subprocess.run(['xdotool', 'search', '--onlyvisible', '--pid', pid[0]],
                             capture_output=True, text=True, timeout=5).stdout.split()
        best = None
        for wid in ids:
            geo = dict(line.split('=') for line in subprocess.run(
                ['xdotool', 'getwindowgeometry', '--shell', wid],
                capture_output=True, text=True, timeout=5).stdout.split())
            box = tuple(int(geo[k]) for k in ('X', 'Y', 'WIDTH', 'HEIGHT'))
            if best is None or box[2] * box[3] > best[1][2] * best[1][3]:
                best = (wid, box)
        if best and raise_it:
            subprocess.run(['xdotool', 'windowraise', best[0]], timeout=5)
        return best[1] if best else None
    except (OSError, subprocess.SubprocessError, ValueError, KeyError):
        return None


def main():
    out, run = sys.argv[1], sys.argv[2]
    views = sys.argv[3:] or ['task', 'isometric']
    frames = os.path.join(out, 'frames')
    os.makedirs(frames, exist_ok=True)
    display = os.environ.get('DISPLAY', ':0')
    rclpy.init()
    node = rclpy.create_node('demo_recorder', namespace='r100_0001')
    latest = {v: None for v in views}
    state = {'name': 'WAITING', 'search_start': None, 'end': None}
    for v in views:
        node.create_subscription(RosImage, f'views/{v}/image',
                                 lambda m, v=v: latest.__setitem__(v, m), 2)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_state(msg):
        try:
            name = str(json.loads(msg.data).get('state', '')).upper() or state['name']
        except ValueError:
            return
        if name == 'SEARCH_G1' and state['search_start'] is None:
            state['search_start'] = time.time()
        if name in END_STATES and state['end'] is None:
            state['end'] = time.time()
        state['name'] = name
    node.create_subscription(String, 'parking_mission/state', on_state, latched)
    stop = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__('now', True))
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 24) \
        if os.path.exists('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf') else ImageFont.load_default()
    index, log = 0, open(os.path.join(out, 'frames.csv'), 'a')
    next_t = time.time()
    window, window_checked = None, 0.0
    try:
        while not stop['now'] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() < next_t:
                continue
            next_t += 0.5
            if state['end'] is not None and time.time() - state['end'] > 10.0:
                break
            canvas = Image.new('RGB', (WIDTH, CAPTION_H + VIEW_H + RVIZ_H), (24, 26, 30))
            tile_w = WIDTH // len(views)
            for k, v in enumerate(views):
                if latest[v] is not None:
                    img = to_pil(latest[v])
                    img.thumbnail((tile_w, VIEW_H))
                    canvas.paste(img, (k * tile_w + (tile_w - img.width) // 2, CAPTION_H))
            if index % 120 == 0:
                # Reset the X idle timer (as a keypress would; settings unchanged), so the
                # screen saver does not blank RViz during a long run (2026-11-11).
                subprocess.run(['xset', '-display', display, 's', 'reset'], timeout=5, check=False)
            if time.time() - window_checked > 10.0:
                # Raised once when first found, so it is not hidden behind another window.
                found = rviz_window(raise_it=window is None)
                window, window_checked = found or window, time.time()
            try:
                screen = ImageGrab.grab(xdisplay=display)
                if window is not None:
                    x, y, w, h = window
                    screen = screen.crop((max(0, x), max(0, y), x + w, y + h))
                screen.thumbnail((WIDTH, RVIZ_H))
                canvas.paste(screen, ((WIDTH - screen.width) // 2, CAPTION_H + VIEW_H))
            except OSError:
                pass
            t0 = state['search_start']
            clock = '--:--' if t0 is None else time.strftime('%M:%S', time.gmtime(time.time() - t0))
            ImageDraw.Draw(canvas).text(
                (16, 9), f'{run}  |  Isaac Sim views: {", ".join(views)}  +  RViz  |  mission time '
                         f'{clock}  |  state: {state["name"]}', fill=(235, 235, 235), font=font)
            canvas.save(os.path.join(frames, f'{index:06d}.jpg'), quality=82)
            log.write(f'{index},{time.time():.2f},{state["name"]}\n')
            log.flush()
            index += 1
    finally:
        log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
