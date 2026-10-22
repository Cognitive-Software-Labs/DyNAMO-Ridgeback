#!/usr/bin/env python3
"""Stage/install fixed commissioning caps without modifying Clearpath generated files.

Application restarts only clearpath-platform and the new guard. A physical
stop must be observed before any changes; failed verification leaves the
platform stopped, never automatically returns to the uncapped configuration.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[2]
LIB = Path('/usr/local/lib/dynamo-hardware')
SYSTEM = Path('/etc/systemd/system')
GUARD = 'dynamo-commissioning-speed-guard.service'
PLATFORM = 'clearpath-platform.service'
JOB = Path('/etc/ros/jazzy/clearpath-platform.d/platform-service.launch.py')


def digest(content):
    return hashlib.sha256(content).hexdigest()


def identity():
    import yaml
    from commissioning_speed_guard import namespace
    data = yaml.safe_load(Path('/etc/clearpath/robot.yaml').read_text())
    return namespace(data['system']['ros2']['namespace'])


def desired_files(ns, test_interlock=False, original_manual_speeds=False):
    if test_interlock and original_manual_speeds:
        raise ValueError('bounded test permits require commissioning speed limits')
    from commissioning_speed_guard import namespace
    ns = namespace(ns)
    source = Path(__file__).with_name('commissioning_speed_guard.py').read_bytes()
    wrapper = f'''# Maintained source: tools/hardware/install_commissioning_caps.py
from pathlib import Path
import hashlib
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

def generate_launch_description():
    guard = Path('{LIB}/commissioning_speed_guard.py')
    if hashlib.sha256(guard.read_bytes()).hexdigest() != '{digest(source)}':
        raise RuntimeError('Commissioning guard differs from installed version')
    jobs = [p for p in Path('/etc/ros/jazzy/clearpath-platform.d').iterdir()
            if p.name != '.installed_files']
    if len(jobs) != 1 or jobs[0].name != 'platform-service.launch.py':
        raise RuntimeError('Review changed platform jobs before commissioning launch')
    return LaunchDescription([
        SetRemap(src='platform_velocity_controller/reference', dst='platform/cmd_vel_limited'),
        SetRemap(src='twist_mux:platform/emergency_stop', dst='platform/emergency_stop_mux'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource('{JOB}')),
    ])
'''
    unit = f'''[Unit]
Description=Fixed Ridgeback commissioning speed guard
After=network.target
Before={PLATFORM}
PartOf={PLATFORM}
StartLimitIntervalSec=0

[Service]
Type=simple
User=robot
Group=robot
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Environment=CYCLONEDDS_URI=file:///etc/clearpath/cyclonedds.xml
Environment=ROS_LOG_DIR=/tmp
ExecStart=/bin/bash -c 'source /opt/ros/jazzy/setup.bash && exec python3 {LIB}/commissioning_speed_guard.py --namespace {ns}{" --require-test-permit" if test_interlock else " --original-manual-speeds" if original_manual_speeds else ""}'
Restart=on-failure
RestartSec=1
TimeoutStopSec=10
'''
    dropin = f'''[Unit]
Requires={GUARD}
After={GUARD}

[Service]
Environment=ROS_DOMAIN_ID=0
Environment=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
Environment=CYCLONEDDS_URI=file:///etc/clearpath/cyclonedds.xml
Environment=ROS_LOG_DIR=/tmp
ExecStart=
ExecStart=/bin/bash -c 'source /etc/clearpath/setup.bash && exec ros2 launch {LIB}/commissioning_platform.launch.py'
'''
    return {LIB/'commissioning_speed_guard.py':source,
            LIB/'commissioning_platform.launch.py':wrapper.encode(),
            SYSTEM/GUARD:unit.encode(),
            SYSTEM/(PLATFORM+'.d')/'70-dynamo-commissioning.conf':dropin.encode()}


def systemctl(*args, check=True, timeout=35):
    return subprocess.run(['systemctl',*args],check=check,text=True,capture_output=True,timeout=timeout)


def require_clock_start_margin():
    """Fail before stopping the platform if its clock gate may outlast start."""
    checker=Path(__file__).with_name('check_clock_ready.py')
    command=[sys.executable,str(checker),'--timeout','1','--max-reference-age','60']
    def ready():
        result=subprocess.run(command,capture_output=True,text=True,check=False,timeout=6)
        try:return result.returncode==0 and json.loads(result.stdout)['ready'] is True
        except (ValueError,KeyError,TypeError):return False
    if ready():return
    # Installation already requires sudo. Refresh measurements without stepping
    # the clock, then qualify the selected reference before touching services.
    burst=subprocess.run(['/usr/bin/chronyc','burst','2/4'],capture_output=True,
                         text=True,check=False,timeout=10)
    if burst.returncode==0:
        command[command.index('--timeout')+1]='30'
        if ready():return
    raise RuntimeError('clock startup margin unavailable; request fresh NTP samples before service restart')


def state(unit):
    result=systemctl('show',unit,'-p','ActiveState','-p','MainPID',check=False)
    return dict(line.split('=',1) for line in result.stdout.splitlines() if '=' in line)


def probe(ns, post=False):
    """Read-only ROS worker: stop, raw odometry, graph and controller states."""
    os.environ.update(ROS_DOMAIN_ID='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                      CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml',ROS_LOG_DIR='/tmp')
    import rclpy
    from std_msgs.msg import Bool
    from nav_msgs.msg import Odometry
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[])
    node=rclpy.create_node('commissioning_install_observer',namespace=ns,enable_rosout=False,start_parameter_services=False)
    try:
        stops=[];mux_stops=[];velocities=[];started=time.monotonic()
        node.create_subscription(Bool,'platform/emergency_stop',lambda m:stops.append(bool(m.data)),qos_profile_sensor_data)
        node.create_subscription(Bool,'platform/emergency_stop_mux',lambda m:mux_stops.append(bool(m.data)),qos_profile_sensor_data)
        node.create_subscription(Odometry,'platform/odom',lambda m:velocities.append((m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z)),qos_profile_sensor_data)
        client=node.create_client(ListControllers,ns+'/controller_manager/list_controllers');future=None
        controllers=[];next_request=started
        def nodes(topic,publishers=False):
            method=node.get_publishers_info_by_topic if publishers else node.get_subscriptions_info_by_topic
            return [{'node':e.node_name,'namespace':e.node_namespace,'type':e.topic_type,
                     'reliability':e.qos_profile.reliability.name} for e in method(ns+'/'+topic)]
        def routing():
            raw=nodes('platform/cmd_vel');limited=nodes('platform/cmd_vel_limited');pubs=nodes('platform/cmd_vel_limited',True)
            return (not any(e['node']=='platform_velocity_controller' for e in raw)
                and any(e['node']=='platform_velocity_controller' and e['type']=='geometry_msgs/msg/TwistStamped' for e in limited)
                and len(pubs)==1 and pubs[0]['node']=='commissioning_speed_guard')
        def mux_stop_routing():
            pubs=nodes('platform/emergency_stop_mux',True)
            subs=nodes('platform/emergency_stop_mux')
            return (len(pubs)==1 and pubs[0]['node']=='commissioning_speed_guard'
                and pubs[0]['reliability']=='RELIABLE'
                and any(e['node']=='twist_mux' and e['reliability']=='RELIABLE' for e in subs)
                and not any(e['node']=='twist_mux' for e in nodes('platform/emergency_stop')))
        while time.monotonic()-started<(25 if post else 7):
            if future is not None and future.done():
                controllers=[{'name':c.name,'state':c.state,'type':c.type} for c in future.result().controller]
                future=None;next_request=time.monotonic()+1
            if future is None and time.monotonic()>=next_request and client.service_is_ready():
                future=client.call_async(ListControllers.Request())
            rclpy.spin_once(node,timeout_sec=.05)
            if (post and len(stops)>=3 and all(stops[-3:]) and len(velocities)>=8
                    and any(c['name']=='platform_velocity_controller' and c['state']=='active' for c in controllers)
                    and routing() and mux_stop_routing()
                    and len(mux_stops)>=3 and all(mux_stops[-3:])):break
        report={'stop_samples':len(stops),'stop_engaged':len(stops)>=3 and all(stops[-3:] if post else stops),
                'odom_samples':len(velocities),'stationary':len(velocities)>=8 and all(all(abs(v)<=.001 for v in row) for row in velocities)}
        report['controllers']=controllers
        report['controller_active']=any(c['name']=='platform_velocity_controller' and c['state']=='active' and c['type']=='mecanum_drive_controller/MecanumDriveController' for c in controllers)
        report['raw_subscribers']=nodes('platform/cmd_vel')
        report['limited_subscribers']=nodes('platform/cmd_vel_limited')
        report['limited_publishers']=nodes('platform/cmd_vel_limited',True)
        report['mux_stop_publishers']=nodes('platform/emergency_stop_mux',True)
        report['mux_stop_subscribers']=nodes('platform/emergency_stop_mux')
        report['mux_stop_routing_ok']=mux_stop_routing()
        report['mux_stop_engaged']=len(mux_stops)>=3 and all(mux_stops[-3:])
        report['routing_ok']=(not any(e['node']=='platform_velocity_controller' for e in report['raw_subscribers'])
            and any(e['node']=='platform_velocity_controller' and e['type']=='geometry_msgs/msg/TwistStamped' for e in report['limited_subscribers'])
            and len(report['limited_publishers'])==1
            and report['limited_publishers'][0]['node']=='commissioning_speed_guard')
        report['ok']=report['stop_engaged'] and report['stationary'] and report['controller_active'] and (not post or (report['routing_ok'] and report['mux_stop_routing_ok'] and report['mux_stop_engaged']))
        print(json.dumps(report));return 0 if report['ok'] else 1
    finally:
        node.destroy_node();rclpy.shutdown()


def run_probe(ns, root, phase):
    command=['/bin/bash','-c','source /opt/ros/jazzy/setup.bash && exec python3 "$@"','probe',
             str(Path(__file__).resolve()),'--probe','--namespace',ns]
    if phase=='after':command.append('--post')
    result=subprocess.run(command,capture_output=True,text=True,timeout=35)
    (root/(phase+'-probe.json')).write_text(result.stdout)
    try:report=json.loads(result.stdout)
    except ValueError:raise RuntimeError('read-only ROS probe failed') from None
    if result.returncode or not report['ok']:raise RuntimeError(phase+' stationary/routing gate failed')
    return report


def write_file(path,content,mode=0o644):
    if path.is_symlink():raise RuntimeError('refuse symlink target')
    path.parent.mkdir(parents=True,exist_ok=True)
    fd,name=tempfile.mkstemp(prefix='.'+path.name+'.',dir=path.parent)
    temporary=Path(name)
    try:
        with os.fdopen(fd,'wb') as stream:
            stream.write(content);os.fchmod(stream.fileno(),mode);stream.flush();os.fsync(stream.fileno())
        temporary.replace(path)
    finally:temporary.unlink(missing_ok=True)


def verify_upgrade(ns, files, previous_run):
    """An active guard may only be replaced from its exact prior install record."""
    root=previous_run.resolve()
    if not root.is_relative_to(ROOT/'artifacts/hardware'):
        raise ValueError('previous installation must be under artifacts/hardware')
    record=json.loads((root/'result.json').read_text())
    if (record.get('status')!='installed-verified-stationary'
            or record.get('namespace')!=ns
            or set(record.get('files',{}))!={str(p) for p in files}):
        raise RuntimeError('previous verified installation record does not match owned files/namespace')
    for path in files:
        if (path.is_symlink() or not path.is_file()
                or digest(path.read_bytes())!=record['files'][str(path)]):
            raise RuntimeError('previous installed file differs: '+str(path))
    return root


def install(run_dir,apply,operator,upgrade_from=None,test_interlock=False,original_manual_speeds=False):
    ns=identity()
    jobs=[p for p in JOB.parent.iterdir() if p.name!='.installed_files']
    if len(jobs)!=1 or jobs[0].name!=JOB.name:raise RuntimeError('unexpected platform jobs; review before override')
    if not Path('/etc/clearpath/cyclonedds.xml').is_file():raise RuntimeError('missing DDS profile')
    files=desired_files(ns,test_interlock,original_manual_speeds)
    run_dir=run_dir.resolve()
    if not run_dir.is_relative_to(ROOT/'artifacts/hardware') or run_dir==ROOT/'artifacts/hardware':
        raise ValueError('run directory must be under repository artifacts/hardware')
    if run_dir.exists():raise ValueError('choose a fresh run directory')
    run_dir.mkdir(parents=True)
    for path,content in files.items():
        candidate=run_dir/'staged'/str(path).lstrip('/');candidate.parent.mkdir(parents=True,exist_ok=True);candidate.write_bytes(content)
    record={'status':'staged','namespace':ns,
            'caps':({'planar_axis_m_s':1.2,'yaw_rad_s':1.5} if original_manual_speeds
                    else {'planar_m_s':.05,'yaw_rad_s':.1}),
            'files':{str(path):digest(content) for path,content in files.items()},
            'source':'tools/hardware/install_commissioning_caps.py',
            'test_interlock':test_interlock,
            'original_manual_speeds':original_manual_speeds}
    if upgrade_from is not None:
        record['upgrade_from_run_dir']=str(verify_upgrade(ns,files,upgrade_from))
    report_path=run_dir/'result.json';report_path.write_text(json.dumps(record,indent=2)+'\n')
    if not apply:return record
    if os.geteuid()!=0 or not operator:raise RuntimeError('apply requires sudo and --operator-confirmed')
    # A fresh physical stop is mandatory; this worker never commands the robot.
    run_probe(ns,run_dir,'before')
    require_clock_start_margin()
    before={u:state(u) for u in [PLATFORM,GUARD,'ridgeback-vnc.service','realsense-camera.service','clearpath-sensors.service']}
    if before[PLATFORM].get('ActiveState')!='active':raise RuntimeError('platform must already be active')
    if before[GUARD].get('ActiveState')=='active' and upgrade_from is None:
        raise RuntimeError('guard already active; an exact prior install record is required for upgrade')
    if upgrade_from is not None and before[GUARD].get('ActiveState')!='active':
        raise RuntimeError('upgrade requires the verified prior guard to be active')
    backup=[]
    for path in files:
        if path.is_symlink():raise RuntimeError('refuse existing symlink target')
        backup.append({'path':str(path),'existed':path.exists(),
                       'content_b64':base64.b64encode(path.read_bytes()).decode() if path.exists() else None,
                       'mode':path.stat().st_mode & 0o777 if path.exists() else None,
                       'uid':path.stat().st_uid if path.exists() else None,
                       'gid':path.stat().st_gid if path.exists() else None})
    (run_dir/'backup.json').write_text(json.dumps({'files':backup,'services':before},indent=2)+'\n')
    record['status']='applying';report_path.write_text(json.dumps(record,indent=2)+'\n')
    stopped=False
    try:
        systemctl('stop',PLATFORM);stopped=True
        systemctl('stop',GUARD)
        for path,content in files.items():write_file(path,content)
        systemctl('daemon-reload')
        # Requires starts the guard before the platform. No enable/reset commands.
        systemctl('start',PLATFORM,timeout=90)
        run_probe(ns,run_dir,'after')
        after={u:state(u) for u in before}
        record['service_states']=after
        # Another agent owns perception. Observe independent changes without
        # treating them as a failure of this separately verified control path.
        record['independent_service_changes']=[unit for unit in
            ['ridgeback-vnc.service','realsense-camera.service','clearpath-sensors.service']
            if after[unit]!=before[unit]]
        record['status']='installed-verified-stationary'
    except Exception as error:
        if stopped:systemctl('stop',PLATFORM,check=False)
        record['status']='failed-platform-left-stopped';record['error']=str(error)
        raise
    finally:report_path.write_text(json.dumps(record,indent=2)+'\n')
    return record


def restore(run_dir,operator):
    if os.geteuid()!=0 or not operator:raise RuntimeError('restore requires sudo and operator confirmation of physical stop')
    root=run_dir.resolve()
    if not root.is_relative_to(ROOT/'artifacts/hardware'):raise ValueError('invalid evidence directory')
    record=json.loads((root/'result.json').read_text())
    backup=json.loads((root/'backup.json').read_text())
    allowed=set(desired_files(record['namespace']))
    if {Path(e['path']) for e in backup['files']}!=allowed:raise RuntimeError('backup does not match owned file set')
    for path in allowed:
        if path.is_symlink():raise RuntimeError('refuse symlink target')
        if path.exists() and digest(path.read_bytes())!=record['files'][str(path)]:
            raise RuntimeError('owned file changed; review before restore')
    # If the capped platform is active, also require the observed physical stop.
    # Failed installation may have left it stopped, with no MCU publisher.
    if state(PLATFORM).get('ActiveState')=='active':run_probe(record['namespace'],root,'restore-before')
    systemctl('stop',PLATFORM)
    systemctl('stop',GUARD,check=False)
    for entry in backup['files']:
        path=Path(entry['path'])
        if entry['existed']:
            write_file(path,base64.b64decode(entry['content_b64'],validate=True),entry['mode'])
            os.chown(path,entry['uid'],entry['gid'])
        else:path.unlink(missing_ok=True)
    systemctl('daemon-reload')
    if backup['services'][GUARD].get('ActiveState')=='active':systemctl('start',GUARD)
    if backup['services'][PLATFORM].get('ActiveState')=='active':
        # This service has a clock gate of up to 900 seconds. Queue its restart
        # without killing the start job after the installer's short CLI timeout.
        systemctl('start','--no-block',PLATFORM)
    # The install receipt describes the configuration that was just removed.
    # Keep it for provenance, but do not present its caps/mode as the restored
    # configuration in the final JSON.
    record['replaced_install']={key:record.pop(key) for key in
        ('caps','files','test_interlock','original_manual_speeds','service_states')
        if key in record}
    restored_files={entry['path']:digest(Path(entry['path']).read_bytes())
                    if entry['existed'] else None for entry in backup['files']}
    guard_unit=next((entry for entry in backup['files']
                     if entry['path']==str(SYSTEM/GUARD)),None)
    unit_bytes=base64.b64decode(guard_unit['content_b64'],validate=True) if guard_unit and guard_unit['existed'] else b''
    record['restored_mode']=('original-manual-speeds' if b'--original-manual-speeds' in unit_bytes
                             else 'test-interlock' if b'--require-test-permit' in unit_bytes
                             else 'guarded-default' if unit_bytes else 'guard-absent')
    record['restored_files']=restored_files
    record['restored_service_states']={unit:state(unit) for unit in (GUARD,PLATFORM)}
    base_status=('restored-previous-capped-installation'
                 if backup['services'][GUARD].get('ActiveState')=='active'
                 else 'restored-original-caps-removed')
    record['status']=(base_status if record['restored_service_states'][PLATFORM].get('ActiveState')=='active'
                      else base_status+'-platform-starting')
    (root/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path)
    parser.add_argument('--apply',action='store_true')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--test-interlock',action='store_true',help='temporary test mode: block all driving without fresh bounded test permits')
    mode.add_argument('--original-manual-speeds',action='store_true',help='restore original L1/R1 speed range while retaining stop and command-age protection')
    parser.add_argument('--operator-confirmed',action='store_true')
    parser.add_argument('--restore',action='store_true',help='restore exact owned files; operator must keep physical stop engaged')
    parser.add_argument('--upgrade-from-run-dir',type=Path,
                        help='replace only files matching this prior verified capped installation')
    parser.add_argument('--probe',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--post',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--namespace',help=argparse.SUPPRESS)
    args=parser.parse_args()
    if args.probe:return probe(args.namespace,args.post)
    if not args.run_dir:parser.error('--run-dir is required')
    if args.apply and args.restore:parser.error('--apply and --restore are exclusive')
    if args.restore and args.upgrade_from_run_dir:parser.error('upgrade and restore are exclusive')
    try:result=(restore(args.run_dir,args.operator_confirmed) if args.restore else
                install(args.run_dir,args.apply,args.operator_confirmed,args.upgrade_from_run_dir,args.test_interlock,args.original_manual_speeds))
    except Exception as error:
        print('Commissioning setup failed: '+str(error),file=sys.stderr);return 1
    print(json.dumps(result,indent=2));return 0


if __name__=='__main__':raise SystemExit(main())
