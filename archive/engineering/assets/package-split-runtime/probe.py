import argparse, json, os, signal, subprocess, time, shutil
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from lifecycle_msgs.srv import GetState
import math
from rosgraph_msgs.msg import Clock
from ridgeback_interfaces.msg import TargetDetections, TargetMeasurements, LocalizationHealth
p=argparse.ArgumentParser();p.add_argument('backend');p.add_argument('profile');p.add_argument('enabled');p.add_argument('--timeout',type=float,default=240);args=p.parse_args()
root=Path.cwd();label=f'{args.backend}-{args.profile}-{args.enabled}'+os.environ.get('PROBE_SUFFIX','')
out=root/'artifacts/package-split/runtime'/label;out.mkdir(parents=True,exist_ok=True)
setup=out/'clearpath';setup.mkdir(exist_ok=True);shutil.copyfile(root/'clearpath/robot.yaml',setup/'robot.yaml')
enabled=args.enabled=='true'
cmd=['ros2','launch','ridgeback_autonomy','ridgeback_exploration.launch.py',f'backend:={args.backend}',f'camera_profile:={args.profile}',f'target_localization_enabled:={args.enabled}',f'setup_path:={setup}/','autonomous_motion_enabled:=false','exploration_rviz:=false','coverage_overlay_enabled:=false','headless_rendering:=true','gz_gui:=false','headless:=true','world:=mock_hospital']
env=dict(os.environ,ROS_LOG_DIR=str(out/'ros'),GZ_PARTITION='package-split-'+label)
counts={};frames={};sizes={};stamps={};health=[];detected={};valid={}
rclpy.init();node=Node('package_split_runtime_probe');subscriptions=[]
def observe(key,msg):
 counts[key]=counts.get(key,0)+1
 if hasattr(msg,'detected'):detected[key]=detected.get(key,0)+int(msg.detected)
 if isinstance(msg,TargetMeasurements):
  valid[key]=valid.get(key,0)+sum(math.isfinite(float(v)) for field in ('pointcloud_distance_m','projective_ranging_distance_m','euclidean_reconstruction_distance_m','polar_profiling_distance_m') for v in getattr(msg,field))
 if hasattr(msg,'header'):
  frames[key]=msg.header.frame_id
  stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
  stamps.setdefault(key,set()).add(stamp)
  if len(stamps[key])>200:stamps[key].remove(min(stamps[key]))
 if isinstance(msg,Image): sizes[key]=[msg.width,msg.height]
 if isinstance(msg,LocalizationHealth):
  health.append({'state':msg.state,'detail':msg.detail})
  if len(health)>20:health.pop(0)
for key,typ,topic in [('clock',Clock,'/clock'),('color',Image,'/r100_0001/sensors/camera_0/color/image'),('depth',Image,'/r100_0001/sensors/camera_0/depth/image'),('scan',LaserScan,'/r100_0001/sensors/lidar2d_0/scan'),('odom',Odometry,'/r100_0001/platform/odom/filtered'),('map',OccupancyGrid,'/r100_0001/map'),('detections',TargetDetections,'/r100_0001/detections/target/raw'),('mask',TargetMeasurements,'/r100_0001/measurements/target/mask'),('pointcloud',TargetMeasurements,'/r100_0001/measurements/target/pointcloud'),('health',LocalizationHealth,'/r100_0001/localization/health')]:
 subscriptions.append(node.create_subscription(typ,topic,lambda msg,key=key:observe(key,msg),qos_profile_sensor_data))
log=(out/'console.log').open('w');proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
started=time.monotonic();passed=False
clients={name:node.create_client(GetState,'/r100_0001/'+name+'/get_state') for name in ('controller_server','bt_navigator')}
pending={};states={};last_request=0.0
try:
 while time.monotonic()-started<args.timeout and proc.poll() is None:
  rclpy.spin_once(node,timeout_sec=.1)
  if time.monotonic()-last_request>1:
   for name,client in clients.items():
    if name in pending and pending[name].done():
     states[name]=pending.pop(name).result().current_state.label
    if name not in pending and client.service_is_ready():pending[name]=client.call_async(GetState.Request())
   last_request=time.monotonic()
  required=['clock','color','depth','scan','odom','map']+(['detections','mask','pointcloud'] if enabled else [])
  if all(states.get(name)=='active' for name in clients) and all(counts.get(key,0)>=3 for key in required) and sizes.get('color')==[int(x) for x in args.profile.split('x')]:
   if enabled:
    if len(health)<3 or not all(x['state']=='processing' for x in health[-3:]):continue
    if not (stamps['detections'] & stamps['mask'] & stamps['pointcloud']):continue
   elif any(node.count_publishers(topic) for topic in ['/r100_0001/detections/target/raw','/r100_0001/measurements/target/mask','/r100_0001/localization/health']):continue
   passed=True;break
 result=dict(passed=passed,command=cmd,revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),duration_s=round(time.monotonic()-started,2),counts=counts,nav2_states=states,detected_batches=detected,finite_measurements=valid,frames=frames,sizes=sizes,health=health,launch_exit=proc.poll())
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
finally:
 if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
 try:proc.wait(timeout=20)
 except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
 # Launch can exit before a descendant; kill only this launch's process group.
 try:os.killpg(proc.pid,signal.SIGKILL)
 except ProcessLookupError:pass
 log.close();node.destroy_node();rclpy.shutdown()
raise SystemExit(0 if passed else 1)
