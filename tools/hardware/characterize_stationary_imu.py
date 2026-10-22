#!/usr/bin/env python3
"""Stationary IMU bias/noise characterization; never apply calibration or publish commands."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import time
ROOT=Path(__file__).resolve().parents[2]


def summarize(rows):
    result={}
    for field in ['gyro','accel']:
        vectors=[r[field] for r in rows];means=[statistics.mean(v[i] for v in vectors) for i in range(3)]
        covariance=[[sum((v[i]-means[i])*(v[j]-means[j]) for v in vectors)/(len(vectors)-1) for j in range(3)] for i in range(3)]
        halves=[vectors[:len(vectors)//2],vectors[len(vectors)//2:]]
        result[field]={'mean':means,'covariance':covariance,'stddev':[math.sqrt(covariance[i][i]) for i in range(3)],'first_to_second_half_mean_change':[statistics.mean(v[i] for v in halves[1])-statistics.mean(v[i] for v in halves[0]) for i in range(3)]}
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--duration',type=float,default=60);p.add_argument('--run-dir',type=Path,required=True);args=p.parse_args()
    if not math.isfinite(args.duration) or not 30<=args.duration<=600:p.error('duration must be 30..600 seconds')
    root=args.run_dir.resolve()
    if not root.is_relative_to(ROOT/'artifacts/hardware') or root==ROOT/'artifacts/hardware':p.error('fresh artifacts/hardware directory required')
    root.mkdir(parents=True,exist_ok=False)
    from install_commissioning_caps import identity
    ns=identity();os.environ.update(ROS_DOMAIN_ID='0',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI='file:///etc/clearpath/cyclonedds.xml',ROS_LOG_DIR='/tmp')
    import rclpy
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool
    from nav_msgs.msg import Odometry
    from clearpath_platform_msgs.msg import Temperature,Power
    from rclpy.qos import qos_profile_sensor_data
    rclpy.init(args=[]);node=rclpy.create_node('stationary_imu_characterization',namespace=ns,enable_rosout=False,start_parameter_services=False)
    rows=[];temperatures=[];power=[];stops=[];odoms=[];start=time.monotonic();log=(root/'samples.jsonl').open('w');phase='discovery';error=None
    def imu(m):
        now=time.monotonic();values=[[getattr(getattr(m,name),a) for a in ['x','y','z']] for name in ['angular_velocity','linear_acceleration']]
        if not all(math.isfinite(v) for xyz in values for v in xyz):raise RuntimeError('nonfinite IMU data')
        if phase=='measurement':
            row={'elapsed_s':now-start,'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'frame':m.header.frame_id,'gyro':values[0],'accel':values[1]};rows.append(row);log.write(json.dumps(row)+'\n')
    def stop(m):
        stops.append((time.monotonic(),m.data))
        if not m.data:raise RuntimeError('physical e-stop released; characterization stopped')
    def odom(m):
        v=m.twist.twist;values=[v.linear.x,v.linear.y,v.angular.z]
        if not all(math.isfinite(x) and abs(x)<=.001 for x in values):raise RuntimeError('wheel motion reported; characterization stopped')
        odoms.append(time.monotonic())
    node.create_subscription(Imu,'sensors/imu_0/data_raw',imu,qos_profile_sensor_data)
    node.create_subscription(Bool,'platform/emergency_stop',stop,qos_profile_sensor_data)
    node.create_subscription(Odometry,'platform/odom',odom,qos_profile_sensor_data)
    node.create_subscription(Temperature,'platform/mcu/status/temperature',lambda m:temperatures.append({'elapsed_s':time.monotonic()-start,'temperatures':list(m.temperatures)}),qos_profile_sensor_data)
    node.create_subscription(Power,'platform/mcu/status/power',lambda m:power.append({'elapsed_s':time.monotonic()-start,'measured_voltages':list(m.measured_voltages),'measured_currents':list(m.measured_currents)}),qos_profile_sensor_data)
    try:
        while time.monotonic()-start<6:rclpy.spin_once(node,timeout_sec=.05)
        phase='measurement';measurement_start=time.monotonic();print('Stationary characterization started; no controls or services are changed.',flush=True)
        while time.monotonic()-measurement_start<args.duration:
            now=time.monotonic()
            if len(stops)<2 or not stops[-1][1] or now-stops[-1][0]>2.5:raise RuntimeError('asserted-stop observation absent/stale')
            if len(odoms)<20 or now-odoms[-1]>.5:raise RuntimeError('stationary odometry absent/stale')
            rclpy.spin_once(node,timeout_sec=.05)
        if len(rows)<args.duration*45:raise RuntimeError('insufficient IMU samples')
        if any(b['stamp_ns']<=a['stamp_ns'] for a,b in zip(rows,rows[1:])):raise RuntimeError('non-increasing IMU stamps')
        report={'status':'stationary-characterization-complete-not-full-calibration','namespace':ns,'samples':len(rows),'duration_s':args.duration,'statistics':summarize(rows),'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'limits':'Single fixed pose. Gyro mean is a provisional zero-rate observation, not scale/misalignment/temperature calibration. Acceleration bias is confounded with gravity and mounting tilt. MCU/PCB temperature is not battery/IMU die temperature.'}
        (root/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));return 0
    except Exception as e:
        error=str(e);(root/'result.json').write_text(json.dumps({'status':'aborted','error':error})+'\n');raise
    finally:
        (root/'hardware-telemetry.json').write_text(json.dumps({'mcu_pcb_temperature':temperatures,'power':power},indent=2)+'\n');log.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
