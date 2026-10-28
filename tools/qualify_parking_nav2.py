#!/usr/bin/env python3
"""Configure an isolated Nav2 controller; never activate it or send action goals."""

import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory
import yaml

from ridgeback_autonomy.mission.parking_nav2_profile import (
    GOAL_CHECKER_PARAMETERS, goal_checker_reason, write_profile, parking_tree_identity,
)
from ridgeback_autonomy.mission.parking_planner import GridSnapshot, TargetReference
from ridgeback_autonomy.mission.parking_safety import keepout_mask


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--bt-probe', help='Optional separately built no-tick Nav2 BT probe')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise SystemExit('Run in isolated ROS_DOMAIN_ID=42')
    output = Path(args.output).resolve()
    ns = f'parking_config_probe_{os.getpid()}'
    profile = write_profile(output, ns)
    evidence = dict(namespace=ns, configured=False, action_goals_sent=0)
    rclpy.init()
    node = rclpy.create_node('parking_configuration_probe', namespace=f'/{ns}',
        cli_args=['--ros-args','-r','/tf:=tf','-r','/tf_static:=tf_static'])
    processes = []
    active_costmaps = []

    def call(srv, name, request):
        client = node.create_client(srv, name)
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError(f'service unavailable: {name}')
        f = client.call_async(request)
        rclpy.spin_until_future_complete(node, f, timeout_sec=20.0)
        if not f.done() or f.exception():
            raise RuntimeError(f'service response unavailable: {name}')
        return f.result()

    def parameters(name, names):
        result = call(GetParameters, name, GetParameters.Request(names=names))
        return dict(zip(names, map(parameter_value_to_python, result.values)))

    try:
        with (output/'controller.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'run', 'nav2_controller', 'controller_server',
                '--ros-args', '-r', f'__ns:=/{ns}', '--params-file', str(profile),
                '-p', 'use_sim_time:=false', '-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
                '-r', 'cmd_vel:=qualification_cmd_vel'], stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True)
            processes.append(process)
            request = ChangeState.Request()
            request.transition.id = Transition.TRANSITION_CONFIGURE
            evidence['configured'] = call(ChangeState, f'/{ns}/controller_server/change_state', request).success
            if not evidence['configured']:
                raise RuntimeError('controller configuration failed; inspect controller.log')
            state = call(GetState, f'/{ns}/controller_server/get_state', GetState.Request()).current_state
            evidence['lifecycle_id'], evidence['lifecycle_label'] = state.id, state.label
            evidence['goal_checkers'] = parameters(f'/{ns}/controller_server/get_parameters', list(GOAL_CHECKER_PARAMETERS))
            evidence['goal_checker_reason'] = goal_checker_reason(evidence['goal_checkers'])
            evidence['local_filter'] = parameters(f'/{ns}/local_costmap/local_costmap/get_parameters',
                ['filters', 'parking_g1_keepout.plugin', 'parking_g1_keepout.filter_info_topic'])
            if state.id != 2 or evidence['goal_checker_reason'] is not None:
                raise RuntimeError('expected inactive controller with both configured checkers')
            # Only the test profile's map input changes; no shared map or robot.
            data = yaml.safe_load(profile.read_text())
            data[ns]['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic'] = f'/{ns}/qualification_map'
            profile.write_text(yaml.safe_dump(data,sort_keys=False),encoding='utf-8')
            with (output/'planner.log').open('w') as planner_log:
                planner = subprocess.Popen(['ros2','run','nav2_planner','planner_server','--ros-args',
                    '-r',f'__ns:=/{ns}','--params-file',str(profile),'-p','use_sim_time:=false',
                    '-r','/tf:=tf','-r','/tf_static:=tf_static'],stdout=planner_log,
                    stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(planner)
            evidence['planner_configured'] = call(ChangeState,f'/{ns}/planner_server/change_state',request).success
            if not evidence['planner_configured']:
                raise RuntimeError('planner configuration failed')
            evidence['global_filter'] = parameters(f'/{ns}/global_costmap/global_costmap/get_parameters',
                ['filters','parking_g1_keepout.plugin','parking_g1_keepout.filter_info_topic'])
            tree = Path(get_package_share_directory('ridgeback_autonomy'))/'config/parking_navigate_to_pose.xml'
            evidence['parking_bt_sha256'], reason = parking_tree_identity(tree)
            if reason:
                raise RuntimeError(reason)
            if args.bt_probe:
                probe = subprocess.run([str(Path(args.bt_probe).resolve()),str(tree),f'/{ns}'],
                    capture_output=True,text=True,timeout=15)
                (output/'bt_probe.log').write_text(probe.stdout+probe.stderr,encoding='utf-8')
                if probe.returncode:
                    raise RuntimeError('no-tick BT instantiation failed; inspect bt_probe.log')
                evidence['parking_bt_runtime'] = json.loads(next(line.split('PARKING_BT_PROBE ',1)[1]
                    for line in probe.stdout.splitlines() if line.startswith('PARKING_BT_PROBE ')))
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            mask_pub=node.create_publisher(OccupancyGrid,'parking/g1_keepout_mask',qos)
            info_pub=node.create_publisher(CostmapFilterInfo,'parking/g1_keepout_info',qos)
            map_pub=node.create_publisher(OccupancyGrid,'qualification_map',qos)
            stamp=node.get_clock().now().nanoseconds
            grid=GridSnapshot('map',stamp,.05,400,400,-10.,-10.,0.,np.zeros((400,400),dtype=np.int16))
            target=TargetReference(1.5,0.,'map',stamp,'generic_target_position',False,'synthetic_probe_only')
            mask=keepout_mask(grid,target,.29)
            base_map=OccupancyGrid(header=mask.header)
            base_map.info.resolution=.05; base_map.info.width=base_map.info.height=400
            base_map.info.origin.position.x=base_map.info.origin.position.y=-10.
            base_map.info.origin.orientation.w=1.
            cells=np.zeros((400,400),dtype=np.int16)
            unknown_xy=(target.x+.35,target.y+.35)
            obstacle_xy=(target.x-.35,target.y+.35)
            for xy,cost in [(unknown_xy,-1),(obstacle_xy,100)]:
                col,row=(int((v+10.)/.05) for v in xy)
                cells[row-1:row+2,col-1:col+2]=cost
            base_map.data=cells.ravel().tolist()
            info=CostmapFilterInfo(header=mask.header,type=0,base=0.,multiplier=1.,
                filter_mask_topic=f'/{ns}/parking/g1_keepout_mask')
            broadcaster=StaticTransformBroadcaster(node)
            transforms=[]
            for parent,child in [('map','odom'),('odom','base_link')]:
                tf=TransformStamped(); tf.header.frame_id=parent; tf.child_frame_id=child
                tf.header.stamp=node.get_clock().now().to_msg(); tf.transform.rotation.w=1.
                transforms.append(tf)
            broadcaster.sendTransform(transforms)
            outputs={}
            subscriptions=[]
            for kind in ('local_costmap','global_costmap'):
                subscriptions.append(node.create_subscription(OccupancyGrid,f'{kind}/costmap',
                    lambda msg,k=kind: outputs.update({k:msg}),qos))
            map_pub.publish(base_map)
            for kind in ('local_costmap','global_costmap'):
                activate=ChangeState.Request()
                activate.transition.id=Transition.TRANSITION_ACTIVATE
                if not call(ChangeState,f'/{ns}/{kind}/{kind}/change_state',activate).success:
                    raise RuntimeError(f'{kind} activation failed')
                active_costmaps.append(kind)
            deadline=time.monotonic()+8.
            def sampled_cost(msg,xy=None):
                xy=xy or (target.x,target.y)
                col=int((xy[0]-msg.info.origin.position.x)/msg.info.resolution)
                row=int((xy[1]-msg.info.origin.position.y)/msg.info.resolution)
                if not 0<=col<msg.info.width or not 0<=row<msg.info.height:
                    return None
                return msg.data[row*msg.info.width+col]
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if (len(outputs)==2 and sampled_cost(outputs['global_costmap'],unknown_xy)==-1
                        and sampled_cost(outputs['global_costmap'],obstacle_xy)==100):
                    break
            if len(outputs)!=2:
                raise RuntimeError('no baseline costmaps')
            baseline={kind:sampled_cost(msg) for kind,msg in outputs.items()}
            evidence['underlying_target_costs']=baseline
            if any(cost is None or cost==100 for cost in baseline.values()):
                raise RuntimeError('synthetic target must not already be lethal')
            mask_pub.publish(mask); info_pub.publish(info)
            deadline=time.monotonic()+8.
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if len(outputs)==2 and all(sampled_cost(m)==100 for m in outputs.values()):
                    break
            evidence['synthetic_mask_cells']={}
            for kind in ('local_costmap','global_costmap'):
                if kind not in outputs:
                    raise RuntimeError(f'no {kind} output')
                msg=outputs[kind]
                # The private static test TF is identity; map and odom align here only.
                cost=sampled_cost(msg)
                evidence['synthetic_mask_cells'][kind]=cost
                if cost!=100:
                    raise RuntimeError(f'{kind} did not mark synthetic keepout lethal')
            evidence['controller_final_state']=call(GetState,f'/{ns}/controller_server/get_state',GetState.Request()).current_state.id
            evidence['planner_final_state']=call(GetState,f'/{ns}/planner_server/get_state',GetState.Request()).current_state.id
            if evidence['controller_final_state']!=2 or evidence['planner_final_state']!=2:
                raise RuntimeError('motion-capable servers must remain inactive')
            evidence['global_unknown_preserved']=sampled_cost(outputs['global_costmap'],unknown_xy)
            evidence['global_obstacle_preserved']=sampled_cost(outputs['global_costmap'],obstacle_xy)
            if evidence['global_unknown_preserved']!=-1 or evidence['global_obstacle_preserved']!=100:
                raise RuntimeError('keepout filter changed unknown space or an existing obstacle')
            # Replace one sensor-like reference with a new one. Old mask-owned
            # cells must clear through underlying layers, not by publishing free.
            old_xy=(target.x,target.y)
            target=replace(target,x=-1.5,stamp_ns=node.get_clock().now().nanoseconds)
            mask_pub.publish(keepout_mask(grid,target,.29))
            deadline=time.monotonic()+8.
            while time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                if all(sampled_cost(msg)==100 and sampled_cost(msg,old_xy)==baseline[kind]
                       for kind,msg in outputs.items()):
                    break
            evidence['replacement_mask_cells']={kind:dict(new=sampled_cost(msg),old=sampled_cost(msg,old_xy))
                for kind,msg in outputs.items()}
            if not all(v==dict(new=100,old=baseline[kind])
                       for kind,v in evidence['replacement_mask_cells'].items()):
                raise RuntimeError('replacement mask left a ghost or failed to mark new target')
            if sampled_cost(outputs['global_costmap'],unknown_xy)!=-1 or sampled_cost(outputs['global_costmap'],obstacle_xy)!=100:
                raise RuntimeError('replacement mask damaged unrelated costmap information')
    except Exception as error:
        evidence['error'] = str(error)
    finally:
        for kind in active_costmaps:
            try:
                deactivate=ChangeState.Request()
                deactivate.transition.id=Transition.TRANSITION_DEACTIVATE
                call(ChangeState,f'/{ns}/{kind}/{kind}/change_state',deactivate)
            except Exception as error:
                evidence.setdefault('cleanup_errors',[]).append(str(error))
        node.destroy_node()
        rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        (output/'result.json').write_text(json.dumps(evidence, indent=2), encoding='utf-8')
        print(json.dumps(evidence, indent=2))
    return 1 if 'error' in evidence else 0


if __name__ == '__main__':
    raise SystemExit(main())
