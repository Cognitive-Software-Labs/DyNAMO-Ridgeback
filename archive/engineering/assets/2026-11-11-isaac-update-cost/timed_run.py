"""Run crowd_runner with app.update timing ($TIMING_OUT) and optional variants ($PROF_VARIANT, comma list)."""
import atexit, json, os, sys, time
import isaacsim
V = set(filter(None, os.environ.get('PROF_VARIANT', '').split(',')))
_Orig = isaacsim.SimulationApp
samples = []
class Timed(_Orig):
    def __init__(self, cfg, *a, **k):
        if 'small' in V: cfg = dict(cfg, width=640, height=360, window_width=640, window_height=360)
        super().__init__(cfg, *a, **k)
        import carb
        s = carb.settings.get_settings()
        if 'async' in V: s.set_bool('/app/asyncRendering', True)
        if 'dlss' in V:
            s.set_int('/rtx/post/aa/op', 3); s.set_int('/rtx/post/dlss/execMode', 0)
        if 'dumpset' in V:
            for k in ('/app/runLoops/main/rateLimitEnabled','/app/runLoops/main/rateLimitFrequency','/app/runLoops/main/manualModeEnabled','/app/runLoops/rendering_0/rateLimitEnabled','/app/runLoops/rendering_0/rateLimitFrequency','/app/runLoops/present/rateLimitEnabled','/app/runLoops/present/rateLimitFrequency','/app/vsync','/app/asyncRendering','/rtx/rendermode','/app/hydraEngine/waitIdle','/app/renderer/waitIdle','/exts/isaacsim.core.throttling/enable_async','/persistent/simulation/minFrameRate'):
                print('SETTING',k,s.get(k),flush=True)
        if 'norate' in V:
            for k in ('main','rendering_0','rendering_1','present'):
                s.set_bool(f'/app/runLoops/{k}/rateLimitEnabled', False)
        if 'rtl' in V: s.set_string('/rtx/rendermode', 'RaytracedLighting')
        if V & {'noanim','nomove'}:
            from ridgeback_leader_follow import crowd_isaac, crowd_geometry
            orig = crowd_geometry.animate_isaac_person
            seen = set()
            def anim(handle, person, t, cache=None):
                if person.name not in seen:
                    seen.add(person.name); return orig(handle, person, t, cache)
                if 'nomove' in V: return
                import math
                from pxr import Gf
                x, y, yaw = person.sample(t)
                handle[1].Set(Gf.Vec3d(x, y, 0.)); handle[2].Set(math.degrees(yaw))
            crowd_isaac.animate_isaac_person = anim
        if 'breakdown' in V:
            import collections
            from ridgeback_leader_follow import crowd_isaac, crowd_sensors
            T=collections.defaultdict(float); C=collections.Counter()
            def wrap(obj,name,label):
                f=getattr(obj,name)
                def g(*a,**k):
                    t=time.perf_counter(); r=f(*a,**k); T[label]+=time.perf_counter()-t; C[label]+=1; return r
                setattr(obj,name,g)
            wrap(crowd_isaac,'animate_isaac_person','animate')
            wrap(crowd_isaac,'drain_callbacks','drain') if hasattr(crowd_isaac,'drain_callbacks') else None
            wrap(crowd_isaac,'write_sample','truth') if hasattr(crowd_isaac,'write_sample') else None
            from ament_index_python.packages import get_package_share_directory
            sys.path.insert(0, get_package_share_directory('ridgeback_autonomy_isaac') + '/sim/isaac')
            import ros_io, robot_rig
            for n in ('spin_once','publish_clock','publish_odom','publish_ground_truth','publish_wheel_joint_states','publish_imu'):
                wrap(ros_io.RosIO,n,'ros.'+n)
            wrap(robot_rig.RidgebackRig,'step','rig.step'); wrap(robot_rig.RidgebackRig,'update_odom','rig.update_odom')
            T['_upd']=0
            global BD; BD=lambda: print('BREAKDOWN', {k:(round(v,3),C[k]) for k,v in T.items()}, flush=True)
        if 'novsync' in V: s.set_bool('/app/vsync', False)
        if V & {'nolidar', 'nocam'}:
            from ament_index_python.packages import get_package_share_directory
            sys.path.insert(0, get_package_share_directory('ridgeback_autonomy_isaac') + '/sim/isaac')
            import sensors
            from ridgeback_leader_follow import crowd_sensors
            if 'nolidar' in V: sensors.attach_lidars = lambda *a, **k: []
            if 'nocam' in V: sensors.attach_camera = lambda *a, **k: (None, None)
            n = [0]
            def ready(self):
                n[0] += 1; return n[0] > 30
            crowd_sensors.SensorEvidence.ready = property(ready)
    def update(self):
        if 'novp' in V and not getattr(self,'_vp',0):
            self._vp=1
            from omni.kit.viewport.utility import get_active_viewport
            get_active_viewport().updates_enabled = False
        if 'phys30' in V and not getattr(self,'_ph',0):
            import omni.usd
            from pxr import PhysxSchema
            prim = omni.usd.get_context().get_stage().GetPrimAtPath('/physicsScene')
            if prim:
                self._ph=1; PhysxSchema.PhysxSceneAPI(prim).GetTimeStepsPerSecondAttr().Set(30.)
        if 'cpuphys' in V and not getattr(self,'_cp',0):
            import omni.usd
            from pxr import PhysxSchema
            prim = omni.usd.get_context().get_stage().GetPrimAtPath('/physicsScene')
            if prim:
                self._cp=1; api=PhysxSchema.PhysxSceneAPI(prim)
                api.CreateEnableGPUDynamicsAttr(False); api.CreateBroadphaseTypeAttr('MBP')
        w = time.perf_counter(); c = time.thread_time()
        r = super().update()
        samples.append((time.perf_counter() - w, time.thread_time() - c, time.monotonic()))
        return r
    def close(self, *a, **k):
        dump(); globals().get('BD', lambda: 0)(); return super().close(*a, **k)
def dump():
    if samples: json.dump(samples, open(os.environ['TIMING_OUT'], 'w'))
isaacsim.SimulationApp = Timed
atexit.register(dump)
from ridgeback_leader_follow.crowd_runner import main
sys.exit(main(sys.argv[1:]))
