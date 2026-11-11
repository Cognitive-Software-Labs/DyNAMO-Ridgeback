"""Play a realistic scenario in the Isaac window without robot sensors or ROS.

For watching only: no camera, depth, lidar, ROS or ground-truth output, so the
window is the only thing rendered each frame. People follow the same routes,
walk and idle animation as recorded runs, paced to wall-clock time and looped.

  OMNI_KIT_ACCEPT_EULA=YES isaac_venv/bin/python \
    src/ridgeback_leader_follow/sim/watch_realistic.py --scenario crowd
"""
import argparse
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument('--environment', default='modern_office')
parser.add_argument('--scenario', default='crowd')
parser.add_argument('--crowd-size', type=int, default=8)
parser.add_argument('--loops', type=int, default=0, help='0 repeats until the window is closed')
parser.add_argument('--window', default='1280x720', help='Isaac window size')
args = parser.parse_args()
sys.argv = [sys.argv[0]]
from isaacsim import SimulationApp  # noqa: E402
width, height = (int(v) for v in args.window.split('x'))
app = SimulationApp({'headless': False, 'width': width, 'height': height, 'window_width': width, 'window_height': height})
try:
    from pathlib import Path
    import omni.usd
    from pxr import Gf, UsdGeom
    from ament_index_python.packages import get_package_share_directory
    from isaacsim.core.utils.viewports import set_camera_view
    from ridgeback_leader_follow.realistic_scene import create_scene, load_scenario
    from ridgeback_leader_follow.crowd_geometry import animate_isaac_person

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    scenario = load_scenario(args.scenario, args.crowd_size, args.environment)
    people = create_scene(stage, scenario, args.environment)
    # The robot is shown as a static model for scale; it has no sensors here.
    adapter = Path(get_package_share_directory('ridgeback_autonomy_isaac')) / 'sim/isaac'
    robot = stage.DefinePrim('/ridgeback', 'Xform')
    robot.GetReferences().AddReference((adapter / 'usd/robots/ridgeback_r100/ridgeback_r100.usda').as_uri())
    UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0, 0, .06))
    set_camera_view(eye=[-3., -2.6, 2.8], target=[5., 0., .9])
    for _ in range(30):
        app.update()
    loop, frames, report = 0, 0, time.perf_counter()
    while app.is_running() and (args.loops == 0 or loop < args.loops):
        start = time.perf_counter()
        while app.is_running():
            t = time.perf_counter() - start
            if t > scenario.duration:
                break
            for person in scenario.people:
                animate_isaac_person(people[person.name], person, t)
            app.update()
            frames += 1
            now = time.perf_counter()
            if now - report >= 5.:
                print(f'watch fps {frames / (now - report):.1f}', flush=True)
                frames, report = 0, now
        loop += 1
except BaseException:
    import traceback
    traceback.print_exc()
    app.close(exit_code=1)
    raise
finally:
    app.close()
