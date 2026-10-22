#!/usr/bin/env python3
"""Stage/apply reversible IMU selector, battery semantics and platform clock-readiness repair."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import fcntl
import yaml
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/intel_thor'))
from repair_hardware_workflow import snapshot_file,write_file,dump
from install_commissioning_caps import identity,run_probe

LIB=Path('/usr/local/lib/dynamo-hardware')
IMU=Path('/etc/clearpath/platform/config/imu_filter.yaml')
PUMA=Path('/etc/systemd/system/clearpath-platform.service.d/80-dynamo-puma-repair.conf')
DROP=Path('/etc/systemd/system/clearpath-platform.service.d/90-dynamo-auxiliary.conf')
PACKAGE=Path('/opt/ros/jazzy/lib/python3.12/site-packages/clearpath_hardware_interfaces')
PRESERVED=('clearpath-sensors.service','clearpath-scan-merger.service','dynamo-camera.service','ridgeback-vnc.service','ridgeback-vnc-websockify.service','ridgeback-remote-screen.service')


def command(*args):return subprocess.run(args,check=True,capture_output=True,text=True,timeout=60).stdout.strip()
def states():return {u:command('systemctl','show',u,'-p','MainPID','-p','ActiveState') for u in PRESERVED}
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def desired():
    ns=identity();imu=yaml.safe_load(IMU.read_text());assert ns.strip('/') in imu
    parameters=imu[ns.strip('/')]['imu_filter_node']['ros__parameters']
    assert parameters['use_mag'] is False and parameters['publish_tf'] is False
    corrected=yaml.safe_dump({ns+'/imu_filter_madgwick':{'ros__parameters':parameters}},sort_keys=False)
    baseline=PUMA.read_text();old=str(LIB/'commissioning_platform.launch.py')
    if baseline.count(old)!=1 or '/opt/dynamo-puma/' not in baseline:raise RuntimeError('verified Puma overlay required')
    modified=baseline.replace(old,str(LIB/'auxiliary_platform.launch.py'))
    prefix=str(LIB/'aux-python')
    wrapper=f'''from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('PYTHONPATH', {prefix!r} + ':' + os.environ.get('PYTHONPATH','')),
        SetLaunchConfiguration('imu_filter', {str(LIB/'imu_filter_corrected.yaml')!r}),
        IncludeLaunchDescription(PythonLaunchDescriptionSource({old!r})),
    ])
'''
    files={str(DROP):{'content':'[Unit]\nWants=chrony.service\nAfter=chrony.service\n'+modified+'\nExecStartPre=/usr/bin/python3 /usr/local/lib/dynamo-hardware/check_clock_ready.py --timeout 900 --max-correction 0.005\nTimeoutStartSec=930\n','mode':0o644},str(LIB/'auxiliary_platform.launch.py'):{'content':wrapper,'mode':0o644},str(LIB/'imu_filter_corrected.yaml'):{'content':corrected,'mode':0o644},str(LIB/'check_clock_ready.py'):{'content':Path(__file__).with_name('check_clock_ready.py').read_text(),'mode':0o755}}
    package_sources={}
    for p in PACKAGE.rglob('*.py'):
        if p.is_symlink():raise RuntimeError('unexpected package symlink')
        data=p.read_text();package_sources[str(p)]=digest(p)
        if p.name=='battery.py':
            oldline='            self._msg.cell_voltage = [self._msg.voltage / self.series] * self.cell_count'
            if data.count(oldline)!=1:raise RuntimeError('unexpected battery implementation')
            data=data.replace(oldline,'            self._msg.cell_voltage = [nan] * self.cell_count  # No individual measurements.')
        files[str(LIB/'aux-python/clearpath_hardware_interfaces'/p.relative_to(PACKAGE))]={'content':data,'mode':0o644}
    return ns,files,{str(IMU):digest(IMU),str(PUMA):digest(PUMA),**package_sources}


def stage(root):
    ns,files,inputs=desired();root.mkdir(parents=True,exist_ok=False)
    before={p:snapshot_file(Path(p)) for p in files}
    if any(v is not None for v in before.values()):raise RuntimeError('new managed paths already exist; review before stage')
    record={'status':'staged-not-installed','namespace':ns,'files':files,'before':before,'inputs':inputs,'preserved':states(),'intent':[]}
    dump(root/'result.json',record)
    for name,entry in files.items():
        p=root/'staged'/name.lstrip('/');p.parent.mkdir(parents=True,exist_ok=True);p.write_text(entry['content'])
    return record


def load(root):
    record=json.loads((root/'result.json').read_text());ns,files,inputs=desired()
    if record['namespace']!=ns or record['files']!=files or record['inputs']!=inputs or set(record['before'])!=set(files):raise RuntimeError('source/baseline changed; stage again')
    if set(record['preserved'])!=set(PRESERVED):raise RuntimeError('invalid service scope')
    if any(v is not None for v in record['before'].values()):raise RuntimeError('repair owns only previously absent paths')
    if not isinstance(record['intent'],list) or not set(record['intent']).issubset(files) or len(set(record['intent']))!=len(record['intent']):raise RuntimeError('invalid mutation scope')
    return record


def rollback_files(record):
    for name in reversed(record['intent']):
        p=Path(name);before=record['before'][name]
        if before is None:p.unlink(missing_ok=True)
        else:write_file(p,bytes.fromhex(before['content_hex']),before['mode'],before['uid'],before['gid'])


def apply(root,confirmed):
    if not confirmed or os.geteuid()!=0:raise RuntimeError('sudo, operator present, physical e-stop pressed required')
    record=load(root)
    if record['status']!='staged-not-installed':raise RuntimeError('fresh stage required')
    for name,before in record['before'].items():
        if snapshot_file(Path(name))!=before:raise RuntimeError('managed host path changed')
    if states()!=record['preserved']:raise RuntimeError('independent services changed since stage')
    run_probe(record['namespace'],root,'before')
    command('/usr/bin/python3',str(Path(__file__).with_name('check_clock_ready.py')),'--timeout','15')
    command('systemctl','stop','clearpath-platform.service')
    record['status']='applying';dump(root/'result.json',record)
    try:
        for name,entry in record['files'].items():
            record['intent'].append(name);dump(root/'result.json',record)
            write_file(Path(name),entry['content'].encode(),entry['mode'])
        command('systemctl','daemon-reload');command('systemctl','start','clearpath-platform.service')
        run_probe(record['namespace'],root,'after')
        if states()!=record['preserved']:raise RuntimeError('independent service changed')
        record['status']='installed-stationary-awaiting-data-verification';dump(root/'result.json',record)
    except Exception as error:
        command('systemctl','stop','clearpath-platform.service');rollback_files(record)
        command('systemctl','daemon-reload');command('systemctl','start','clearpath-platform.service')
        record.update(status='failed-restored-original',error=str(error));dump(root/'result.json',record);raise
    return record


def probe_auxiliary_data(ns):
    import math,time
    os.environ.update(ROS_DOMAIN_ID='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml',ROS_LOG_DIR='/tmp')
    import rclpy
    from sensor_msgs.msg import BatteryState,Imu
    from rcl_interfaces.srv import GetParameters
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[]);node=rclpy.create_node('auxiliary_install_verifier',namespace=ns,enable_rosout=False,start_parameter_services=False)
    batteries=[];imus=[];parameters={};future=None
    node.create_subscription(BatteryState,'platform/bms/state',lambda m:batteries.append(m),qos_profile_sensor_data)
    node.create_subscription(Imu,'sensors/imu_0/data',lambda m:imus.append(m),qos_profile_sensor_data)
    client=node.create_client(GetParameters,ns+'/imu_filter_madgwick/get_parameters');start=time.monotonic()
    try:
        while time.monotonic()-start<15:
            if future is None and client.service_is_ready():future=client.call_async(GetParameters.Request(names=['use_mag','publish_tf','orientation_stddev']))
            rclpy.spin_once(node,timeout_sec=.05)
            if future is not None and future.done() and not parameters:
                values=future.result().values
                if len(values)==3:parameters={'use_mag':values[0].bool_value,'publish_tf':values[1].bool_value,'orientation_stddev':values[2].double_value,'types':[v.type for v in values]}
            if len(batteries)>=10 and len(imus)>=100 and parameters:break
        checks={'imu_parameters':parameters=={'use_mag':False,'publish_tf':False,'orientation_stddev':0.,'types':[1,1,3]},
                'imu_received':len(imus)>=100,'battery_received':len(batteries)>=10,
                'battery_unknown_cells':bool(batteries) and all(bool(m.cell_voltage) and all(math.isnan(v) for v in m.cell_voltage) for m in batteries),
                'battery_temperature_unmeasured':bool(batteries) and all(math.isnan(m.temperature) for m in batteries),
                'pack_voltage_finite':bool(batteries) and all(math.isfinite(m.voltage) and m.voltage>0 for m in batteries)}
        return {'ok':all(checks.values()),'checks':checks,'imu_parameters':parameters,'battery_samples':len(batteries),'imu_samples':len(imus),'limits':'Verifies installed settings/semantics, not physical IMU calibration, battery accuracy or cross-host synchronization.'}
    finally:node.destroy_node();rclpy.shutdown()


def verify(root,confirmed):
    if not confirmed or os.geteuid()!=0:raise RuntimeError('sudo and operator confirmation required for receipt finalization')
    record=load(root)
    if record['status'] not in ('applying','installed-stationary-awaiting-data-verification','installed-verified-stationary'):raise RuntimeError('no installed repair to verify')
    if set(record['intent'])!=set(record['files']):raise RuntimeError('incomplete installation intent; use restore rather than reapply')
    for name,entry in record['files'].items():
        snapshot=snapshot_file(Path(name))
        if snapshot is None or snapshot['content_hex']!=entry['content'].encode().hex() or snapshot['mode']!=entry['mode'] or snapshot['uid']!=0 or snapshot['gid']!=0:raise RuntimeError('installed file ownership/content mismatch: '+name)
    if states()!=record['preserved']:raise RuntimeError('independent services changed; inspect before finalizing')
    stationary=run_probe(record['namespace'],root,'verification-only')
    clock=json.loads(command('/usr/bin/python3',str(Path(__file__).with_name('check_clock_ready.py')),'--timeout','15'))
    worker=subprocess.run(['/bin/bash','-c','source /opt/ros/jazzy/setup.bash && exec "$@"','auxiliary-verifier','/usr/bin/python3',str(Path(__file__).resolve()),'--data-worker',record['namespace']],capture_output=True,text=True,check=True,timeout=25)
    data=json.loads(worker.stdout)
    dump(root/'data-verification.json',data)
    if not data['ok']:raise RuntimeError('installed auxiliary data gate failed; no services changed; review data-verification.json')
    record.update(status='installed-verified-stationary',verification={'stationary':stationary,'clock':clock,'data':data})
    dump(root/'result.json',record);return record


def restore(root,confirmed):
    if not confirmed or os.geteuid()!=0:raise RuntimeError('sudo and operator confirmation required')
    record=load(root)
    if record['status'] not in ('installed-stationary-awaiting-data-verification','installed-verified-stationary','applying'):raise RuntimeError('no installed repair to restore')
    for name in record['intent']:
        p=Path(name);entry=record['files'][name]
        if snapshot_file(p)!=record['before'][name] and (not p.exists() or p.is_symlink() or p.read_text()!=entry['content'] or p.stat().st_uid!=0):raise RuntimeError('refusing restore over changed file')
    run_probe(record['namespace'],root,'restore-before');command('systemctl','stop','clearpath-platform.service');rollback_files(record)
    command('systemctl','daemon-reload');command('systemctl','start','clearpath-platform.service');record['status']='restored-original';dump(root/'result.json',record);return record


def main():
    parser=argparse.ArgumentParser(description=__doc__);m=parser.add_mutually_exclusive_group(required=True)
    for action in ['stage','apply','restore','verify']:m.add_argument('--'+action,action='store_true')
    parser.add_argument('--operator-confirmed',action='store_true');parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();root=args.run_dir.resolve()
    if not root.is_relative_to(ROOT/'artifacts/hardware') or root==ROOT/'artifacts/hardware':parser.error('fresh artifacts/hardware directory required')
    if args.stage:record=stage(root)
    else:
        if os.geteuid()!=0:parser.error('sudo required')
        fd=os.open('/run/lock/dynamo-auxiliary-repair.lock',os.O_RDWR|os.O_CREAT|os.O_NOFOLLOW,0o600)
        try:
            if os.fstat(fd).st_uid!=0:raise RuntimeError('unexpected lock owner')
            fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            record=verify(root,args.operator_confirmed) if args.verify else restore(root,args.operator_confirmed) if args.restore else apply(root,args.operator_confirmed)
        finally:os.close(fd)
    print(json.dumps({'status':record['status'],'result':str(root/'result.json')}))
if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--data-worker':
        if sys.argv[2]!=identity():raise RuntimeError('worker namespace mismatch')
        print(json.dumps(probe_auxiliary_data(sys.argv[2])))
    else:main()
