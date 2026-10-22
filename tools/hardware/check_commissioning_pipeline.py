#!/usr/bin/env python3
"""Check installed controller routing and speed gate with mock wheels on loopback.

No real hardware plugins, robot namespace or deployment domain are used.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--original-manual-speeds',action='store_true',
                        help='verify original joystick boost commands instead of commissioning limits')
    args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='commissioning-pipeline-') as directory:
        root = Path(directory)
        cyclone = root / 'cyclone.xml'
        cyclone.write_text('<CycloneDDS><Domain id="any"><General><Interfaces>'
                           '<NetworkInterface name="lo"/></Interfaces><AllowMulticast>false'
                           '</AllowMulticast></General><Discovery><Peers><Peer Address="127.0.0.1"/>'
                           '</Peers></Discovery></Domain></CycloneDDS>')
        os.environ.update(ROS_DOMAIN_ID='213', RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',
                          CYCLONEDDS_URI=cyclone.as_uri(), ROS_LOG_DIR=str(root / 'ros'))
        import rclpy
        import yaml
        from std_msgs.msg import String, Bool
        from geometry_msgs.msg import TwistStamped
        from controller_manager_msgs.srv import LoadController, ConfigureController, SwitchController
        from control_msgs.msg import MecanumDriveControllerState
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

        ns = '/commissioning_mock_' + str(os.getpid())
        joints = ['fl', 'fr', 'rr', 'rl']
        urdf = '<robot name="commissioning_mock"><link name="base_link"/>'
        for joint in joints:
            urdf += (f'<link name="{joint}"/><joint name="{joint}" type="continuous">'
                     f'<parent link="base_link"/><child link="{joint}"/><axis xyz="0 1 0"/></joint>')
        urdf += '<ros2_control name="MockWheelsOnly" type="system"><hardware><plugin>mock_components/GenericSystem</plugin></hardware>'
        for joint in joints:
            urdf += f'<joint name="{joint}"><command_interface name="velocity"/><state_interface name="position"><param name="initial_value">0</param></state_interface><state_interface name="velocity"><param name="initial_value">0</param></state_interface></joint>'
        urdf += '</ros2_control></robot>'
        controller = {
            'front_left_wheel_command_joint_name':'fl', 'front_right_wheel_command_joint_name':'fr',
            'rear_right_wheel_command_joint_name':'rr', 'rear_left_wheel_command_joint_name':'rl',
            'kinematics.wheels_radius':.0759, 'kinematics.sum_of_robot_center_projection_on_X_Y_axis':.59,
            'reference_timeout':.1, 'enable_odom_tf':False, 'tf_frame_prefix_enable':False,
        }
        config = root / 'controller.yaml'
        config.write_text(yaml.safe_dump({ns:{'controller_manager':{'ros__parameters':{
            'update_rate':20, 'platform_velocity_controller.type':'mecanum_drive_controller/MecanumDriveController',
            }},
            'platform_velocity_controller':{'ros__parameters':controller},
            'twist_mux':{'ros__parameters':{'use_stamped':True,
                'topics':{'external':{'topic':'cmd_vel','timeout':.5,'priority':1}},
                'locks':{'e_stop':{'topic':'platform/emergency_stop','timeout':0.0,'priority':255}}}}}}))
        launch_file=root/'mock_platform.launch.py'
        launch_file.write_text(f'''from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
def generate_launch_description():
    return LaunchDescription([
        SetRemap(src='platform_velocity_controller/reference', dst='platform/cmd_vel_limited'),
        SetRemap(src='twist_mux:platform/emergency_stop', dst='platform/emergency_stop_mux'),
        Node(package='controller_manager', executable='ros2_control_node',
             namespace={ns!r}, parameters=[{str(config)!r}],
             remappings=[('platform_velocity_controller/reference','platform/cmd_vel')]),
        Node(package='twist_mux', executable='twist_mux', namespace={ns!r},
             parameters=[{str(config)!r}], remappings=[('cmd_vel_out','platform/cmd_vel')]),
    ])
''')
        children=[]; node=None; checks=[]; logs=[]
        try:
            rclpy.init(args=[])
            node=rclpy.create_node('commissioning_mock_observer',namespace=ns,enable_rosout=False,start_parameter_services=False)
            def spin(seconds):
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    if any(p.poll() is not None for p in children): raise RuntimeError('mock process exited')
                    rclpy.spin_once(node,timeout_sec=min(.01,max(0,end-time.monotonic())))
            def spawn(command,name):
                handle=(root/name).open('w');logs.append(handle)
                children.append(subprocess.Popen(command,stdout=handle,stderr=handle,start_new_session=True))
            spawn(['ros2','launch',str(launch_file)],'controller.log')
            spawn([sys.executable,str(Path(__file__).with_name('commissioning_speed_guard.py')),'--namespace',ns]
                  + (['--original-manual-speeds'] if args.original_manual_speeds else []),'guard.log')
            description=node.create_publisher(String,ns+'/robot_description',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            description.publish(String(data=urdf))
            def call(service,name,request):
                client=node.create_client(service,ns+'/controller_manager/'+name)
                end=time.monotonic()+6
                while not client.service_is_ready() and time.monotonic()<end: spin(.03)
                if not client.service_is_ready(): raise RuntimeError('mock service discovery timeout')
                future=client.call_async(request)
                while not future.done() and time.monotonic()<end: spin(.03)
                if not future.done():raise RuntimeError('mock service response timeout')
                result=future.result();node.destroy_client(client)
                if not result.ok:raise RuntimeError('mock controller setup failed')
            call(LoadController,'load_controller',LoadController.Request(name='platform_velocity_controller'))
            call(ConfigureController,'configure_controller',ConfigureController.Request(name='platform_velocity_controller'))
            switch=SwitchController.Request();switch.activate_controllers=['platform_velocity_controller'];switch.strictness=2;switch.activate_asap=True;switch.timeout.sec=3
            call(SwitchController,'switch_controller',switch)
            commands=[];states=[];mux_commands=[];mux_stops=[]
            node.create_subscription(TwistStamped,'platform/cmd_vel_limited',lambda msg:commands.append(msg),10)
            node.create_subscription(MecanumDriveControllerState,'platform_velocity_controller/controller_state',lambda msg:states.append(msg),10)
            node.create_subscription(TwistStamped,'platform/cmd_vel',lambda msg:mux_commands.append(msg),10)
            node.create_subscription(Bool,'platform/emergency_stop_mux',lambda msg:mux_stops.append(msg.data),10)
            raw=node.create_publisher(TwistStamped,'cmd_vel',1)
            stop=node.create_publisher(Bool,'platform/emergency_stop',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            spin(.4)
            def check(name,passed):checks.append({'name':name,'passed':bool(passed)})
            raw_subs=node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel')
            limited_subs=node.get_subscriptions_info_by_topic(ns+'/platform/cmd_vel_limited')
            check('controller detached from raw command topic',not any(e.node_name=='platform_velocity_controller' for e in raw_subs))
            check('controller subscribed to capped topic',any(e.node_name=='platform_velocity_controller' for e in limited_subs))
            native_subs=node.get_subscriptions_info_by_topic(ns+'/platform/emergency_stop')
            relay_subs=node.get_subscriptions_info_by_topic(ns+'/platform/emergency_stop_mux')
            check('node-specific remap leaves guard on native firmware stop',
                  any(e.node_name=='commissioning_speed_guard' for e in native_subs)
                  and not any(e.node_name=='twist_mux' for e in native_subs)
                  and any(e.node_name=='twist_mux' for e in relay_subs))
            stop.publish(Bool(data=False));spin(.1)
            sent=[]
            for _ in range(12):
                msg=TwistStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='base_link'
                msg.twist.linear.x=1.2;msg.twist.linear.y=1.2;msg.twist.angular.z=1.5
                sent.append(msg.header.stamp);raw.publish(msg);spin(.035)
            import math
            nonzero=[m for m in commands if m.twist.linear.x or m.twist.linear.y or m.twist.angular.z]
            def reference_matches_limits(twist):
                if args.original_manual_speeds:
                    return (abs(twist.linear.x-1.2)<1e-9 and abs(twist.linear.y-1.2)<1e-9
                            and abs(twist.angular.z-1.5)<1e-9)
                return math.hypot(twist.linear.x,twist.linear.y)<=.050000000001 and abs(twist.angular.z)<=.1
            check('boost-level diagonal matches selected speed profile',
                  bool(nonzero) and all(reference_matches_limits(m.twist) for m in nonzero))
            check('nonzero source timestamps preserved',bool(nonzero) and all(m.header.stamp in sent for m in nonzero))
            moving=[s for s in states if s.reference_velocity.linear.x>0]
            check('mock controller receives selected reference',
                  bool(moving) and all(reference_matches_limits(s.reference_velocity) for s in moving))
            mark=len(commands);spin(.3)
            check('silence yields zero without nonzero replay',bool(commands[mark:]) and all(m.twist.linear.x==0 and m.twist.linear.y==0 and m.twist.angular.z==0 for m in commands[mark:]))
            stop.publish(Bool(data=True));spin(.1)
            mark=len(commands);mux_mark=len(mux_commands)
            msg.header.stamp=node.get_clock().now().to_msg();raw.publish(msg);spin(.1)
            check('asserted stop blocks incoming nonzero command',bool(commands[mark:]) and all(m.twist.linear.x==0 and m.twist.linear.y==0 and m.twist.angular.z==0 for m in commands[mark:]))
            check('best-effort firmware stop reaches reliable mux lock',
                  bool(mux_stops) and mux_stops[-1] and len(mux_commands)==mux_mark)
            stop.publish(Bool(data=False));spin(.1)
            check('releasing mux lock does not replay old input',len(mux_commands)==mux_mark)
            msg.header.stamp=node.get_clock().now().to_msg();raw.publish(msg);spin(.1)
            check('fresh command resumes after stop release',len(mux_commands)>mux_mark)
            stale_started=time.monotonic()
            while time.monotonic()-stale_started<3.2:
                msg.header.stamp=node.get_clock().now().to_msg();raw.publish(msg);spin(.035)
            check('missing firmware stop asserts reliable mux lock',
                  bool(mux_stops) and all(mux_stops[-10:]))
            check('missing firmware stop blocks ongoing fresh commands',
                  len(commands)>=10 and all(m.twist.linear.x==0 and m.twist.linear.y==0 and m.twist.angular.z==0 for m in commands[-10:])
                  and bool(states) and all(s.reference_velocity.linear.x==0 and s.reference_velocity.linear.y==0 and s.reference_velocity.angular.z==0 for s in states[-8:]))
            report={'mode':'loopback_mock_controller_only','original_manual_speeds':args.original_manual_speeds,
                    'domain':213,'namespace':ns,'hardware_plugin':'mock_components/GenericSystem','checks':checks}
            print(json.dumps(report,indent=2));return 0 if all(c['passed'] for c in checks) else 1
        except Exception:
            for path in root.glob('*.log'):
                print(path.name+':\n'+path.read_text()[-4000:],file=sys.stderr)
            raise
        finally:
            if node is not None:node.destroy_node();rclpy.shutdown()
            for p in children:
                if p.poll() is None:
                    os.killpg(p.pid,signal.SIGTERM)
                    try:p.wait(timeout=3)
                    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=3)
            for log in logs:log.close()


if __name__=='__main__':raise SystemExit(main())
