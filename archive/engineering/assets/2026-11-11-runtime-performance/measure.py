import sys,os,time,json,subprocess
from pathlib import Path
from ridgeback_leader_follow import crowd_runner as runner
from ridgeback_leader_follow.crowd_sensors import SensorEvidence
root=Path(__file__).resolve().parent
backend,variant=sys.argv[1:3]
quality="reference"
variants={"reference":[],"humans":["--human-detail","distant"],"sensors":["--sensor-quality","reduced"],"physics":["--collision-quality","simple"],"balanced":["--performance-profile","balanced"]}
output=root/f'{backend}-{variant}-r3'
frames=[];camera=[];window=[];subscription=None;animation_seconds=[];updates=[];viewport_info={}
original_receive=SensorEvidence.receive
def receive(self,*args,**kwargs):
    if args and args[0]=='rgb':camera.append(time.monotonic())
    return original_receive(self,*args,**kwargs)
SensorEvidence.receive=receive
original_write=runner.write_sample
def write(*args,**kwargs):
    window.append(time.monotonic())
    return original_write(*args,**kwargs)
runner.write_sample=write
if backend=='gz':
    config=Path('/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config').read_text()
    plugin=root/'meter/build/libDirectFps.so'
    cfg=root/f'gui-{variant}.config'
    cfg.write_text(config+f'\n<plugin filename="DirectFps" name="DirectFps"><gz-gui><property type="bool" key="visible">false</property></gz-gui></plugin>\n')
    os.environ['GZ_GUI_PLUGIN_PATH']=str(plugin.parent)+os.pathsep+os.environ.get('GZ_GUI_PLUGIN_PATH','')
    os.environ['DIRECT_FPS_OUTPUT']=str(root/f'gz-{variant}-frames.csv')
    original_popen=subprocess.Popen
    def popen(cmd,*args,**kwargs):
        if cmd[:3]==['gz','sim','-g']:cmd=[*cmd,'--gui-config',str(cfg)]
        return original_popen(cmd,*args,**kwargs)
    subprocess.Popen=popen
else:
    from ridgeback_leader_follow import crowd_isaac
    original_animate=crowd_isaac.animate_isaac_person
    def animate(*args,**kwargs):
        global subscription,viewport_info
        if subscription is None:
            from omni.kit.viewport.utility import get_active_viewport
            viewport=get_active_viewport()
            viewport_info=dict(viewport.frame_info)
            from isaacsim import SimulationApp
            original_update=SimulationApp.update
            def measured_update(self,*a,**kw):
                wall=time.monotonic();cpu=time.thread_time()
                result=original_update(self,*a,**kw)
                updates.append((wall,time.monotonic()-wall,time.thread_time()-cpu))
                return result
            SimulationApp.update=measured_update
            subscription=viewport.subscribe_to_frame_change(lambda *a:frames.append(time.monotonic()))
        start=time.monotonic()
        result=original_animate(*args,**kwargs)
        animation_seconds.append(time.monotonic()-start)
        return result
    crowd_isaac.animate_isaac_person=animate

def save():
    rendered=frames
    file=root/f'gz-{variant}-frames.csv'
    if backend=='gz' and file.exists():rendered=[float(x) for x in file.read_text().splitlines()]
    data={'backend':backend,'quality':quality,'variant':variant,'window':window,'render_timestamps':rendered,'rgb_callback_timestamps':camera,'animation_seconds':animation_seconds,'app_updates':updates,'viewport_info':viewport_info}
    if window:
        start,end=min(window)+2,max(window)
        for name,values in [('viewport',rendered),('rgb_received',camera)]:
            selected=[x for x in values if start<=x<=end]
            data[name]={'count':len(selected),'wall_seconds':end-start,'fps':len(selected)/(end-start) if end>start else None}
    (root/f'{backend}-{variant}-measurement.json').write_text(json.dumps(data,indent=2,default=str))
    print({k:v for k,v in data.items() if k in ('backend','quality','viewport','rgb_received')},flush=True)
if backend=='isaac':
    original=runner.run_isaac
    def run(*args,**kwargs):
        args=list(args);finish=args[5]
        def end(error=None):save();return finish(error)
        args[5]=end
        return original(*args,**kwargs)
    runner.run_isaac=run
try:
    runner.main(['--backend',backend,'--scene-profile','realistic','--environment','modern_office','--scenario','explainer','--gui','--render-quality',quality,'--output-dir',str(output),'--duration','8','--record',*variants[variant]])
finally:
    save()
