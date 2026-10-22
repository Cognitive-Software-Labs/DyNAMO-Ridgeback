#!/usr/bin/env python3
"""One bounded stationary health batch. No commands, service restarts or Bluetooth actions."""
import argparse
from datetime import datetime,timezone
import json
import math
import os
from pathlib import Path
import subprocess
import time
from uuid import uuid4
ROOT=Path(__file__).resolve().parents[2]


def run(argv):
    result=subprocess.run(argv,capture_output=True,text=True,timeout=12)
    return {'returncode':result.returncode,'stdout':result.stdout,'stderr':result.stderr}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--duration',type=float,default=16);args=p.parse_args()
    if not math.isfinite(args.duration) or not 12<=args.duration<=60:p.error('duration must be 12..60 seconds')
    root=ROOT/'artifacts/hardware'/('stationary-batch-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid4().hex[:6]);root.mkdir(parents=True)
    from install_commissioning_caps import identity
    ns=identity();os.environ.update(ROS_DOMAIN_ID='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml',ROS_LOG_DIR='/tmp')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu,BatteryState,LaserScan
    from std_msgs.msg import Bool
    from nav_msgs.msg import Odometry
    rclpy.init(args=[]);node=rclpy.create_node('fast_stationary_observer',namespace=ns,enable_rosout=False,start_parameter_services=False)
    rows={};start=time.monotonic();checks=[]
    def add(name,passed,detail):checks.append({'name':name,'status':'PASS' if passed else 'FAIL','detail':detail})
    def callback(topic,m):
        entry={'at':time.monotonic()-start}
        if hasattr(m,'header'):entry['stamp']=m.header.stamp.sec*10**9+m.header.stamp.nanosec
        if isinstance(m,Bool):entry['asserted']=bool(m.data)
        if isinstance(m,Odometry):entry['velocity']=[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z]
        if isinstance(m,Imu):entry['values']=[getattr(getattr(m,f),a) for f in ['angular_velocity','linear_acceleration'] for a in ['x','y','z']];entry['orientation_covariance']=list(m.orientation_covariance)
        if isinstance(m,BatteryState):entry.update(voltage=m.voltage,cell_voltage=list(m.cell_voltage),temperature=m.temperature)
        rows.setdefault(topic,[]).append(entry)
    topics={'front':('sensors/lidar2d_0/scan',LaserScan),'rear':('sensors/lidar2d_1/scan',LaserScan),'merged':('sensors/scan',LaserScan),'imu':('sensors/imu_0/data',Imu),'battery':('platform/bms/state',BatteryState),'stop':('platform/emergency_stop',Bool),'relay':('platform/emergency_stop_mux',Bool),'odom':('platform/odom',Odometry)}
    try:
        for label,(topic,kind) in topics.items():node.create_subscription(kind,topic,lambda m,t=label:callback(t,m),qos_profile_sensor_data)
        print('Observing stationary health; keep physical e-stop pressed. No controller actions required.',flush=True)
        while time.monotonic()-start<args.duration:rclpy.spin_once(node,timeout_sec=.02)
        (root/'samples.json').write_text(json.dumps(rows,indent=2)+'\n')
        rates={}
        for label in ['front','rear','merged','imu','battery','odom']:
            values=[r for r in rows.get(label,[]) if r['at']>=4];gaps=[b['at']-a['at'] for a,b in zip(values,values[1:])]
            rate=(len(values)-1)/(values[-1]['at']-values[0]['at']) if len(values)>1 else 0
            maxgap=max(gaps,default=float('inf'));duplicate=any(b.get('stamp',0)<=a.get('stamp',0) for a,b in zip(values,values[1:]))
            threshold={'front':38,'rear':38,'merged':None,'imu':45,'battery':5,'odom':18}[label]
            limit={'front':.25,'rear':.25,'merged':.5,'imu':.1,'battery':.5,'odom':.25}[label]
            rates[label]={'rate_hz':rate,'max_gap_s':maxgap,'samples':len(values),'non_increasing_stamp':duplicate}
            add(label+' continuity',len(values)>1 and (threshold is None or rate>=threshold) and maxgap<=limit and not duplicate,rates[label])
        for label in ['stop','relay']:
            values=rows.get(label,[]);age=time.monotonic()-start-values[-1]['at'] if values else float('inf')
            add(label+' asserted',len(values)>=2 and all(r['asserted'] for r in values) and age<=(2.5 if label=='stop' else .5),{'samples':len(values),'last_age_s':age})
        add('stationary wheels',bool(rows.get('odom')) and all(all(math.isfinite(v) and abs(v)<=.001 for v in r['velocity']) for r in rows.get('odom',[])),'raw odometry only; not independent physical motion measurement')
        add('IMU finite',bool(rows.get('imu')) and all(all(math.isfinite(v) for v in r['values']) for r in rows.get('imu',[])),'zero orientation covariance means unknown uncertainty')
        add('battery semantics',bool(rows.get('battery')) and all(math.isfinite(r['voltage']) and r['voltage']>0 and r['cell_voltage'] and all(math.isnan(v) for v in r['cell_voltage']) and math.isnan(r['temperature']) for r in rows.get('battery',[])),'finite pack voltage; individual voltages and temperature correctly unknown')
        publishers=node.get_publishers_info_by_topic(ns+'/platform/cmd_vel_limited');subs=node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel_limited');raw=node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel')
        add('guarded routing',len(publishers)==1 and publishers[0].node_name=='commissioning_speed_guard' and any(e.node_name=='platform_velocity_controller' for e in subs) and not any(e.node_name=='platform_velocity_controller' for e in raw),'graph ownership, not a motion test')
    finally:node.destroy_node();rclpy.shutdown()
    service_evidence={}
    for unit in ['clearpath-platform','dynamo-commissioning-speed-guard','clearpath-sensors','clearpath-scan-merger','dynamo-camera','ridgeback-vnc','ridgeback-vnc-websockify','ridgeback-remote-screen','controller-autoconnect']:
        query=run(['systemctl','show',unit+'.service','-p','ActiveState','-p','NRestarts']);service_evidence[unit]=query
        add('service '+unit,query['returncode']==0 and 'ActiveState=active' in query['stdout'],query['stdout'].strip())
    clock=run(['/usr/bin/python3',str(Path(__file__).with_name('check_clock_ready.py')),'--timeout','5'])
    try:clock_detail=json.loads(clock['stdout'])
    except (ValueError,TypeError):clock_detail={'error':clock['stderr'] or clock['stdout']}
    add('local clock ready',clock['returncode']==0,clock_detail)
    report={'status':'stationary-health-pass' if all(c['status']=='PASS' for c in checks) else 'stationary-health-failed','checks':checks,'duration_s':time.monotonic()-start,'service_evidence':service_evidence,
            'remaining':[{'item':'D2 deadman and physical-stop isolation; command-loss repetitions','status':'NEEDS SUPERVISED TEST'},
                         {'item':'D3 silent wireless loss','status':'NEEDS TWO-OPERATOR SESSION'},
                         {'item':'D4 handover regression on final installed configuration','status':'NEEDS FINAL REGRESSION'},
                         {'item':'D5 stationary per-wheel freshness deployment','status':'PASSED 2026-10-22'},
                         {'item':'D6/D7 moving feedback and motor-link faults','status':'NEEDS CONTROLLED HARDWARE TEST'},
                         {'item':'D8 measured stopping distance','status':'NEEDS APPROVED MOTION TEST'}],
            'limits':'Current stationary health only; no camera readers, control publishers, Bluetooth transitions or service changes. S1-S10 and P1-P5 are finished for the agreed scope, with their recorded limits; autonomous motion remains disabled.'}
    (root/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'status':report['status'],'failed':[c['name'] for c in checks if c['status']=='FAIL'],'failure_details':[{
        'check':c['name'],
        'reason':[k for k,v in c['detail'].get('checks',{}).items() if not v],
        'measurements':{k:c['detail'][k] for k in ['remaining_correction_s','skew_ppm','estimated_utc_error_bound_s'] if k in c['detail']}
    } for c in checks if c['status']=='FAIL' and isinstance(c['detail'],dict)],'report':str(root/'result.json')},indent=2))
    return 0 if report['status']=='stationary-health-pass' else 1
if __name__=='__main__':raise SystemExit(main())
