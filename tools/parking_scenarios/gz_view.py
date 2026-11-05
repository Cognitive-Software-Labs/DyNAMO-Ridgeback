"""Point the Gazebo GUI camera obliquely at the scenario's G1 (display only)."""
import math, os, subprocess, sys, yaml
sc = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../src/ridgeback_autonomy/config/parking_scenarios.yaml')))['scenarios'][sys.argv[1]]
# Frame the start (origin) and G1 together, seen steeply from the -y side.
gx, gy = sc['g1']['x'] * 0.6, sc['g1']['y']
px, py, pz = gx, gy - 3.4, 6.0
dx, dy, dz = gx - px, gy - py, 0.3 - pz
yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
cy, sy, cp, sp = math.cos(yaw/2), math.sin(yaw/2), math.cos(pitch/2), math.sin(pitch/2)
q = dict(w=cy*cp, x=-sy*sp, y=cy*sp, z=sy*cp)
req = (f"pose: {{position: {{x: {px}, y: {py}, z: {pz}}}, orientation: {{x: {q['x']}, y: {q['y']}, "
       f"z: {q['z']}, w: {q['w']}}}}}")
env = dict(os.environ, GZ_PARTITION=f'ridgeback-parking-{os.getuid()}-83', GZ_IP='127.0.0.1')
r = subprocess.run(['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
                    '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                   env=env, capture_output=True, text=True, timeout=10)
print('move_to/pose', r.returncode, r.stdout.strip(), r.stderr.strip()[:200])
