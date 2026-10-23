#!/usr/bin/env python3
"""Reproduce MCU filter YAML selector mismatch in loopback-only ROS domain 216."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
ROOT=Path(__file__).resolve().parents[2]


def check(output):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/hardware') or output==ROOT/'artifacts/hardware':raise ValueError('fresh artifacts/hardware output required')
    output.mkdir(parents=True,exist_ok=False)
    cyclone=output/'cyclone.xml';cyclone.write_text('<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
    os.environ.update(ROS_DOMAIN_ID='216',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI=cyclone.as_uri(),ROS_LOG_DIR=str(output/'ros-log'))
    import rclpy
    from rcl_interfaces.srv import GetParameters
    document=yaml.safe_load(Path('/etc/clearpath/platform/config/imu_filter.yaml').read_text())
    original=output/'imu_filter.yaml';original.write_text(yaml.safe_dump(document))
    ns=next(iter(document));selector=next(iter(document[ns]))
    params=document[ns][selector]['ros__parameters'];names=['use_mag','publish_tf','orientation_stddev']
    expected={k:params[k] for k in names};results={}
    corrected=output/'imu_corrected.yaml'
    corrected.write_text(yaml.safe_dump({'/'+ns+'/imu_filter_madgwick':{'ros__parameters':params}}))
    # Namespace unchanged; isolated DDS domain/interface prevents robot discovery.
    rclpy.init();node=rclpy.create_node('selector_check',enable_rosout=False,start_parameter_services=False)
    try:
        for label,name,config in [('original','imu_filter_madgwick',original),('renamed','imu_filter_node',original),('corrected-selector','imu_filter_madgwick',corrected)]:
            with (output/(label+'.log')).open('w') as log:
                child=subprocess.Popen(['/opt/ros/jazzy/lib/imu_filter_madgwick/imu_filter_madgwick_node','--ros-args','-r','__node:='+name,'-r','__ns:=/'+ns,'--params-file',str(config)],stdout=log,stderr=log,start_new_session=True)
                try:
                    client=node.create_client(GetParameters,'/'+ns+'/'+name+'/get_parameters')
                    if not client.wait_for_service(timeout_sec=5):raise RuntimeError('isolated filter discovery failed')
                    future=client.call_async(GetParameters.Request(names=names));end=time.monotonic()+5
                    while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
                    if not future.done():raise RuntimeError('isolated parameter query timed out')
                    values=future.result().values
                    if len(values)!=len(names):raise RuntimeError('incomplete response')
                    results[label]={k:v.bool_value if v.type==1 else v.double_value for k,v in zip(names,values)}
                    node.destroy_client(client)
                finally:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        report={'status':'isolated-selector-repair-verified-not-installed','expected':expected,'observed':results,'domain':216,'interface':'lo','limits':'No IMU samples or hardware; verifies configuration selection, not calibration or filter accuracy.'}
        assert results['original']['use_mag']!=expected['use_mag']
        assert results['renamed']==expected
        assert results['corrected-selector']==expected
        (output/'result.json').write_text(json.dumps(report,indent=2)+'\n');return report
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args();print(json.dumps(check(a.output),indent=2))
