#!/usr/bin/env python3
"""Exercise the guard's bounded permit using loopback-only simulated topics."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    with tempfile.TemporaryDirectory(prefix='test-interlock-') as name:
        root=Path(name)
        profile=root/'cyclone.xml'
        profile.write_text('<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>')
        os.environ.update(ROS_DOMAIN_ID='214',RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',CYCLONEDDS_URI=profile.as_uri(),ROS_LOG_DIR=str(root/'ros'))
        import rclpy
        from std_msgs.msg import Bool
        from geometry_msgs.msg import TwistStamped
        rclpy.init(args=[])
        ns='/isolated_test_permit_'+str(os.getpid())
        node=rclpy.create_node('check_test_permit',namespace=ns,use_global_arguments=False)
        rows=[]
        node.create_subscription(TwistStamped,'platform/cmd_vel_limited',lambda m:rows.append((time.monotonic(),m.twist.linear.x)),1)
        cmd=node.create_publisher(TwistStamped,'platform/cmd_vel',1)
        stop=node.create_publisher(Bool,'platform/emergency_stop',1)
        permit=node.create_publisher(Bool,'platform/commissioning_test_permit',1)
        log=(root/'guard.log').open('w')
        child=subprocess.Popen([sys.executable,str(Path(__file__).with_name('commissioning_speed_guard.py')),'--namespace',ns,'--require-test-permit'],stdout=log,stderr=log)
        checks=[]
        def pump(seconds,pressed=False,allowed=None):
            start=time.monotonic(); offset=len(rows)
            while time.monotonic()-start<seconds:
                if child.poll() is not None:raise RuntimeError('isolated guard exited')
                stop.publish(Bool(data=pressed))
                if allowed is not None:permit.publish(Bool(data=allowed))
                m=TwistStamped();m.header.stamp=node.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=.5;cmd.publish(m)
                rclpy.spin_once(node,timeout_sec=.015)
                time.sleep(.005)
            return rows[offset:]
        def check(label,condition):
            checks.append({'check':label,'passed':bool(condition)})
            if not condition:raise AssertionError(label)
        def zero_tail(samples):return len(samples)>=4 and all(v==0 for _,v in samples[-4:])
        try:
            deadline=time.monotonic()+8
            while not cmd.get_subscription_count() or not stop.get_subscription_count() or not permit.get_subscription_count():
                if time.monotonic()>deadline:raise RuntimeError('isolated discovery timed out')
                rclpy.spin_once(node,timeout_sec=.05)
            check('released physical stop without permit stays blocked',zero_tail(pump(.4)))
            check('fresh permit permits capped command',any(0<v<=.05 for _,v in pump(.25,allowed=True)))
            check('missing heartbeat suppresses ongoing fresh commands',zero_tail(pump(.4)))
            check('heartbeat cannot extend one-second budget',zero_tail(pump(.8,allowed=True)))
            pump(.1,allowed=False)
            check('false then true cannot rearm spent budget',zero_tail(pump(.2,allowed=True)))
            pump(.2,pressed=True,allowed=False)
            check('physical stop release without permit stays blocked',zero_tail(pump(.2)))
            check('new stop cycle permits a fresh bounded pulse',any(0<v<=.05 for _,v in pump(.2,allowed=True)))
            print(json.dumps({'status':'passed','domain':214,'interface':'lo','real_hardware_processes_started':0,'checks':checks},indent=2))
        finally:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait()
            log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
