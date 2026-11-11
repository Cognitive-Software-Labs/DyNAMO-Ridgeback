"""Full selected-profile ROS replay matrix; corrected TF queue and acquisition ties."""
import json,subprocess,sys,hashlib

def receipt():
 return {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in Path("src/ridgeback_leader_follow").rglob("*.py")}
from pathlib import Path
root=Path('artifacts/leader-following');out=root/'f3-live-validation-continuity-floor';out.mkdir(exist_ok=True)
cases=[]
for prefix in ['f3-appearance','f3-camera-exit']:
 evaluation=root/(prefix+'-evaluation')
 for case in json.loads((evaluation/'corpus.json').read_text())['results']:
  cases.append((Path(case['run']),evaluation/'annotations-stationary-anchor'/(Path(case['run']).name+'.jsonl')))
priority=['isaac-leader_out_of_view-4','gz-leader_out_of_view-4','gz-lookalike_crossing-4','isaac-lookalike_crossing-4','isaac-crossing-4','isaac-crowd-4','gz-leader_stop-4']
cases.sort(key=lambda pair:priority.index(pair[0].name) if pair[0].name in priority else len(priority))
summary={}
source_hashes=receipt()
(out/"source-hashes.json").write_text(json.dumps(source_hashes,indent=2)+"\n")
def run_case(item):
 case_index,(run,annotations)=item
 destination=out/run.name;target=destination/'identity_score.json'
 if not target.exists():
  if destination.exists():raise RuntimeError('Inspect partial case before resuming: '+str(destination))
  assert receipt()==source_hashes, 'Source changed during campaign'
  print('Run',run.name,flush=True)
  subprocess.run([sys.executable,'-m','ridgeback_leader_follow.benchmarking.validate_node','--run',str(run),'--output',str(destination),'--python','perception_venv/bin/python','--model-dir',str(root/'f2-models'),'--ros-domain',str(150+case_index),'--track-maximum-age','3','--observation-timeout','.25','--camera','owlv2','--lidar','lfe-ppn','--appearance','osnet-ain','--appearance-model-dir',str(root/'f3-models'),'--match-threshold','.65','--recovery-margin','.15','--acquire-interval','1','--acquire-distance-tolerance','.25','--acquire-lateral-tolerance','.1','--continuity-min-confidence','.25'],check=True)
  subprocess.run([sys.executable,'-m','ridgeback_leader_follow.benchmarking.ros_identity_score','--run',str(run),'--validation',str(destination),'--annotations',str(annotations),'--output',str(target)],check=True)
 data=json.loads(target.read_text())
 print(run.name,data['status'],'correct',data['correct_locked_frames'],'wrong',data['wrong_locked_frames'],'outside',data['outside_camera_lock_fraction'],flush=True)
 return run.name,data

from concurrent.futures import ThreadPoolExecutor,as_completed
with ThreadPoolExecutor(max_workers=3) as pool:
 for future in as_completed([pool.submit(run_case,item) for item in enumerate(cases)]):
  key,data=future.result();summary[key]=data
  (out/'summary.json').write_text(json.dumps({'status':'running','cases':summary},indent=2)+'\n')
(out/'summary.json').write_text(json.dumps({'status':'passed' if all(d['status']=='passed' for d in summary.values()) else 'failed','cases':summary},indent=2)+'\n')

assert receipt()==source_hashes, "Source changed during campaign"
