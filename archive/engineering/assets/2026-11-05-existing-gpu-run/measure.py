import json,time,statistics
from pathlib import Path
from ridgeback_leader_follow import crowd_runner
root=Path('artifacts/leader-following/gpu-run-performance');run=root/'full-gui'
original=crowd_runner.run_gazebo
samples=[];truth=[]
def measured(*args,**kwargs):
 from gz.transport13 import Node
 from gz.msgs10.world_stats_pb2 import WorldStatistics
 from gz.msgs10.pose_v_pb2 import Pose_V
 node=Node()
 def stats(m):samples.append({'wall':time.monotonic(),'rtf':m.real_time_factor,'sim':m.sim_time.sec+m.sim_time.nsec*1e-9})
 def people(m):
  for value in m.header.data:
   if value.key=='scenario_time':truth.append((time.monotonic(),float(value.value[0])))
 node.subscribe(WorldStatistics,'/world/leader_follow_scene/stats',stats)
 node.subscribe(Pose_V,'/leader_follow/ground_truth',people)
 try:return original(*args,**kwargs)
 finally:
  result={'gui_fps':'not instrumented; observe GUI separately','statistics':samples,'truth_wall_samples':truth}
  if len(truth)>1:
   wall=truth[-1][0]-truth[0][0];sim=truth[-1][1]-truth[0][1]
   result.update(playback_wall_seconds=wall,playback_sim_seconds=sim,observed_realtime_factor=sim/wall)
   selected=[s['rtf'] for s in samples if truth[0][0]<=s['wall']<=truth[-1][0]]
   if selected:result['median_reported_realtime_factor']=statistics.median(selected)
  (root/'performance.json').write_text(json.dumps(result,indent=2)+'\n')
crowd_runner.run_gazebo=measured
start=time.monotonic()
try:crowd_runner.main(['--backend','gz','--scene-profile','realistic','--environment','modern_office','--scenario','explainer','--gui','--ros-domain','223','--output-dir',str(run)])
finally:(root/'total_wall_seconds.txt').write_text(str(time.monotonic()-start)+'\n')
