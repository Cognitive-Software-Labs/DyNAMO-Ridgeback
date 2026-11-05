import subprocess,json
from pathlib import Path
from ridgeback_leader_follow.crowd_qualify import validate_run
root=Path(__file__).resolve().parent
cases=[('office','leader_alone'),('office','crossing'),('warehouse','lookalike_parallel'),('warehouse','lookalike_crossing'),('courtyard','leader_stop'),('courtyard','crowd')]
results=[]
for backend in ('gz','isaac'):
 for environment,scenario in cases:
  key=f'{backend}-{environment}-{scenario}-8'
  cmd=['python3' if backend=='gz' else 'isaac_venv/bin/python','-m','ridgeback_leader_follow.crowd_runner','--backend',backend,'--scene-profile','realistic','--environment',environment,'--scenario',scenario,'--crowd-size','8','--ros-domain','213' if backend=='gz' else '214','--output-dir',str(root/key)]
  print('Running',key,flush=True)
  with (root/(key+'.log')).open('w') as log:
   process=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,timeout=360)
  try:
   if process.returncode:raise RuntimeError(f'Process exited {process.returncode}')
   result=validate_run(root/key,'realistic')
  except Exception as error:result={'case':key,'status':'failed','error':str(error)}
  results.append(result)
  (root/'summary.json').write_text(json.dumps({'scope':'six full scenario families in both backends, distributed over three worlds; scene/sensor gates only','results':results},indent=2)+'\n')
  print(result,flush=True)
