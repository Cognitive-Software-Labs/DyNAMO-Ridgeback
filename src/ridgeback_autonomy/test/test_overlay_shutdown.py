"""Exercise benchmark display entry points with real ROS delivery and shutdown signals."""

import os
import select
import signal
import subprocess
import sys
import time

import pytest


CHILD = r'''
import time
from sensor_msgs.msg import Image
import importlib
module = importlib.import_module(MODULE)
base = getattr(module, CLASS)

class ProbeNode(base):
    def __init__(self):
        super().__init__()
        self.seen = False
        self.publisher = self.create_publisher(Image, 'shutdown_probe', 10)
        self.create_subscription(Image, 'shutdown_probe', self.receive, 10)
        self.message = Image(height=1, width=1, encoding='rgb8', step=3, data=[0, 0, 0])
        self.create_timer(.001, lambda: self.publisher.publish(self.message))

    def receive(self, message):
        if not self.seen:
            self.seen = True
            print('READY', flush=True)

    def destroy_node(self):
        print('CLEANUP_CONTEXT', self.context.ok(), flush=True)
        time.sleep(.2)
        return super().destroy_node()

setattr(module, CLASS, ProbeNode)
module.main()
print('DONE', flush=True)
'''


# Every display node a benchmark configuration launches: a signal landing inside
# message deserialization must not surface as an exit-1 traceback.
ENTRY_POINTS = [
    ('ridgeback_localization.overlay_node', 'TargetOverlayNode'),
    ('ridgeback_localization.visualization_node', 'TargetVisualizationNode'),
    ('ridgeback_autonomy.diagnostics.hud_node', 'HudNode'),
]


@pytest.mark.parametrize('module,cls', ENTRY_POINTS, ids=[c for _m, c in ENTRY_POINTS])
@pytest.mark.parametrize('first_signal', [signal.SIGINT, signal.SIGTERM])
def test_display_node_keeps_context_alive_until_signal_shielded_cleanup(tmp_path, first_signal,
                                                                       module, cls):
    env = dict(os.environ, ROS_DOMAIN_ID='189', ROS_LOG_DIR=str(tmp_path))
    code = f'MODULE = {module!r}\nCLASS = {cls!r}\n' + CHILD
    child = subprocess.Popen([sys.executable, '-u', '-c', code],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env)
    output = []

    def expect(prefix):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if not select.select([child.stdout], [], [], .1)[0]:
                continue
            line = child.stdout.readline().decode(errors='replace')
            output.append(line)
            if line.startswith(prefix):
                return line.strip()
            if not line and child.poll() is not None:
                break
        pytest.fail(f'Missing {prefix}: {"".join(output)}')

    try:
        expect('READY')
        child.send_signal(first_signal)
        assert expect('CLEANUP_CONTEXT') == 'CLEANUP_CONTEXT True'
        # Launch/group relays can deliver another signal during destruction.
        child.send_signal(signal.SIGINT)
        remaining, _ = child.communicate(timeout=15)
        output.append(remaining.decode(errors='replace'))
        assert child.returncode == 0, ''.join(output)
        assert 'DONE' in ''.join(output)
        assert 'Traceback' not in ''.join(output)
    finally:
        if child.poll() is None:
            child.kill()
            child.communicate(timeout=5)
