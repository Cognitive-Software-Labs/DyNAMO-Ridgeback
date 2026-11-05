"""F1 Isaac loop using the production Ridgeback USD, rig and sensor bindings."""
import math
import os
import subprocess
import sys
import time
import traceback

from ridgeback_leader_follow.crowd_geometry import create_isaac_scene, animate_isaac_person
from ridgeback_leader_follow.crowd_sensors import SensorEvidence


def run(scenario, duration, output, writer, gui, finish, record=False, scene_profile="baseline", environment=None):
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    from ridgeback_leader_follow.crowd_runner import stop_process, write_sample
    import yaml

    adapter = Path(get_package_share_directory('ridgeback_autonomy_isaac')) / 'sim/isaac'
    sys.path.insert(0, str(adapter))
    # Use the same generated description as the production Isaac backend.
    with (output / 'description.log').open('w') as log:
        subprocess.run(['ros2', 'run', 'clearpath_generator_common', 'generate_description',
                        '-s', str(output / 'setup')], stdout=log, stderr=subprocess.STDOUT,
                       check=True, timeout=30)
        urdf = subprocess.check_output(['xacro', str(output / 'setup/robot.urdf.xacro'),
                                       'is_sim:=true', 'camera_support_deck_z:=0.280'], text=True)
    params = output / 'robot_state_publisher.yaml'
    params.write_text(yaml.safe_dump({'/**': {'ros__parameters': {
        'use_sim_time': True, 'robot_description': urdf}}}))
    rsp_log = (output / 'robot_state_publisher.log').open('w')
    rsp = subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
        '--ros-args', '--params-file', str(params), '-r', '__ns:=/r100_0001',
        '-r', '/tf:=tf', '-r', '/tf_static:=tf_static'],
        stdout=rsp_log, stderr=subprocess.STDOUT, start_new_session=True)
    app = None
    ros = None
    monitor = None
    depth_handle = None
    error = None
    try:
        os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': not gui})
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.extensions import enable_extension
        from omni.kit.loop import _loop as omni_loop
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
        from robot_rig import RidgebackRig
        from ros_io import RosIO
        from sensors import attach_camera, attach_lidars
        from worlds import BASE_LINK_FLOOR_CLEARANCE

        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        if scene_profile == 'realistic':
            from ridgeback_leader_follow.realistic_scene import create_scene
            people = create_scene(stage, scenario, environment or 'office')
        elif scene_profile == 'lab':
            from ridgeback_leader_follow.lab_scene import create_scene
            people = create_scene(stage, scenario)
        elif scene_profile == 'f3':
            from ridgeback_leader_follow.f3_scene import create_isaac_scene as create_f3_scene
            people = create_f3_scene(stage, scenario)
        else:
            people = create_isaac_scene(stage, scenario)
        for person in scenario.people:
            animate_isaac_person(people[person.name], person, 0.)
        robot = stage.DefinePrim('/ridgeback', 'Xform')
        robot.GetReferences().AddReference((adapter / 'usd/robots/ridgeback_r100/ridgeback_r100.usda').as_uri())
        height = BASE_LINK_FLOOR_CLEARANCE
        UsdGeom.XformCommonAPI(robot).SetTranslate(Gf.Vec3d(0, 0, height))
        UsdPhysics.FixedJoint(stage.GetPrimAtPath('/ridgeback/drive_rig/world_fix')).CreateLocalPos0Attr(Gf.Vec3f(0, 0, height))
        root = next(p.GetPath().pathString for p in Usd.PrimRange(robot)
                    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and '/drive_rig/' in str(p.GetPath()))
        physics = UsdPhysics.Scene.Define(stage, '/physicsScene')
        PhysxSchema.PhysxSceneAPI.Apply(physics.GetPrim()).CreateTimeStepsPerSecondAttr(120.)
        enable_extension('isaacsim.ros2.bridge')
        ros = RosIO('r100_0001', odom_tf=True)
        monitor = SensorEvidence(output, record=record)
        attach_lidars(stage, '/ridgeback', 'r100_0001')
        _, depth_handle = attach_camera(stage, '/ridgeback', 'r100_0001',
                                         '640x480', 'ideal', depth_sink=ros.publish_d455_depth)
        rig = RidgebackRig(root, odom_noise=0., seed=0)
        dt = .025
        loop = omni_loop.acquire_loop_interface()
        loop.set_manual_step_size(dt)
        loop.set_manual_mode(True, name='main')
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_time_codes_per_second(40.)
        timeline.set_end_time(1e9)
        timeline.set_looping(False)
        timeline.play()
        for _ in range(240):
            app.update()
            if rig.ready():
                break
        else:
            raise RuntimeError('Ridgeback articulation did not initialize')
        rig.initialize()
        rig.set_planar_pose(0., 0., 0.)
        if gui and scene_profile in ('lab','realistic'):
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=[-3., -5., 3.5] if scene_profile=='realistic' else [-2.5,-2.3,2.4], target=[5.,0.,.9])

        def publish_robot():
            stamp = timeline.get_current_time()
            ros.publish_clock(stamp)
            ros.publish_wheel_joint_states(stamp)
            odom, twist = rig.update_odom()
            ros.publish_odom(stamp, odom, twist)
            ros.publish_imu(stamp, 0., 0., 0.)
            ros.publish_ground_truth(stamp, *rig.ground_truth())
            ros.spin_once()
            for _ in range(12):
                monitor.spin(0.)

        deadline = time.monotonic() + 120.
        while not monitor.ready:
            if time.monotonic() > deadline:
                raise TimeoutError(f'Robot sensors did not become ready: {monitor.stats}')
            rig.step(dt, now=timeline.get_current_time())
            app.update()
            publish_robot()
        origin = timeline.get_current_time()
        monitor.epoch = origin
        for index in range(round(duration / dt)):
            tick_start = time.monotonic()
            if not app.is_running():
                raise RuntimeError('Isaac window closed before scene completion')
            elapsed = (index + 1) * dt
            for person in scenario.people:
                animate_isaac_person(people[person.name], person, elapsed)
            # No command is consumed. The F1 observation robot stays stationary.
            rig.step(dt, now=timeline.get_current_time())
            app.update()
            publish_robot()
            if index % 2 == 1:
                stamp = timeline.get_current_time()
                cache = UsdGeom.XformCache(Usd.TimeCode.Default())
                for person in scenario.people:
                    matrix = cache.GetLocalToWorldTransform(people[person.name][0].GetPrim())
                    position = matrix.ExtractTranslation()
                    direction = matrix.TransformDir(Gf.Vec3d(1, 0, 0))
                    write_sample(writer, person, stamp, stamp-origin, elapsed,
                                 (*position, math.atan2(direction[1], direction[0])))
            if gui:
                time.sleep(max(0., dt - (time.monotonic() - tick_start)))
        stage.GetRootLayer().Export(str(output / 'final_scene.usda'))
    except BaseException as exc:
        error = exc
        traceback.print_exc()
    finally:
        if depth_handle is not None:
            depth_handle.detach()
        if monitor is not None:
            monitor.close()
        if ros is not None:
            ros.shutdown()
        stop_process(rsp)
        rsp_log.close()
        finish(error)
        print('F1 Isaac evidence finalized; shutting down.', flush=True)
        if app is not None:
            app.close(exit_code=0 if error is None else 1)
    if error is not None:
        raise error
