"""Render static meshes with a saved-frame Gazebo camera: gzshot.py out_dir cam_x,y,z,pitch,yaw mesh.dae[@x,y,z,scale] ..."""
import subprocess,sys,os,pathlib,shutil
out=pathlib.Path(sys.argv[1]).resolve();shutil.rmtree(out,ignore_errors=True);out.mkdir(parents=True)
cx,cy,cz,pitch,yaw=map(float,sys.argv[2].split(','))
models=''
for i,arg in enumerate(sys.argv[3:]):
    path,_,pose=arg.partition('@');x,y,z,s=(list(map(float,pose.split(',')))+[0,0,0,1])[:4] if pose else (0,0,0,1)
    models+=f'<model name="m{i}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name="l"><visual name="v"><geometry><mesh><uri>file://{pathlib.Path(path).resolve()}</uri><scale>{s} {s} {s}</scale></mesh></geometry></visual></link></model>'
sdf=f'''<?xml version="1.0"?><sdf version="1.9"><world name="shot">
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
<scene><ambient>.45 .45 .45 1</ambient><background>.58 .72 .87 1</background><shadows>true</shadows></scene>
<light name="sun" type="directional"><direction>-0.3 0.4 -1</direction><diffuse>.95 .94 .9 1</diffuse><cast_shadows>true</cast_shadows></light>
<model name="ground"><static>true</static><link name="l"><visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry><material><diffuse>.5 .5 .5 1</diffuse></material></visual></link></model>
{models}
<model name="cam"><static>true</static><pose>{cx} {cy} {cz} 0 {pitch} {yaw}</pose><link name="l"><sensor name="c" type="camera"><update_rate>5</update_rate><always_on>1</always_on>
<camera><horizontal_fov>1.2</horizontal_fov><image><width>960</width><height>640</height></image><clip><near>.05</near><far>100</far></clip><save enabled="true"><path>{out}</path></save></camera></sensor></link></model>
</world></sdf>'''
(out/'shot.sdf').write_text(sdf)
env=dict(os.environ);env.pop('DISPLAY',None);env.pop('WAYLAND_DISPLAY',None)
import time,signal
p=subprocess.Popen(['gz','sim','-s','-r','-v','3',str(out/'shot.sdf')],env=env,stdout=open(out/'log.txt','w'),stderr=subprocess.STDOUT,start_new_session=True)
sub=subprocess.Popen(['gz','topic','-e','-t','/world/shot/model/cam/link/l/sensor/c/image'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
deadline=time.time()+float(os.environ.get('GZSHOT_TIMEOUT','90'))
while time.time()<deadline and len(list(out.glob('*.png')))<int(os.environ.get('GZSHOT_FRAMES','3')):time.sleep(.5)
os.killpg(sub.pid,signal.SIGKILL);os.killpg(p.pid,signal.SIGINT)
try:p.wait(15)
except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL)
frames=sorted(out.glob('*.png'),key=lambda f:f.stat().st_mtime)
if frames:shutil.copy(frames[-1],out/'final.png');print(len(frames),'frames',frames[-1] if frames else None)
