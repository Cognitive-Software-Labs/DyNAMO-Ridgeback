#!/usr/bin/env python3
"""Exercise the real People publisher with isolated recorded sensor playback."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import rclpy
from leader_follow_interfaces.msg import People, LeaderState
from rosbag2_interfaces.srv import Resume
from ridgeback_leader_follow.crowd_runner import stop_process


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('run','output','python','model-dir'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--ros-domain',type=int,required=True)
    p.add_argument('--camera',default='yolo11')
    p.add_argument('--lidar',default='clustering')
    p.add_argument('--appearance',default='none')
    p.add_argument('--track-maximum-age',type=float,default=.75)
    p.add_argument('--observation-timeout',type=float,default=.15)
    p.add_argument('--acquire-interval',type=int,default=8)
    p.add_argument('--acquire-distance-tolerance',type=float,default=0.)
    p.add_argument('--acquire-lateral-tolerance',type=float,default=0.)
    p.add_argument('--match-threshold',type=float,default=.72)
    p.add_argument('--recovery-margin',type=float,default=.05)
    p.add_argument('--appearance-model-dir',type=Path,default=Path('artifacts/leader-following/f3-models'))
    a=p.parse_args();os.environ['ROS_DOMAIN_ID']=str(a.ros_domain)
    a.output.mkdir(parents=True,exist_ok=False)
    qos=a.output/'qos.yaml'
    qos.write_text('/r100_0001/tf_static:\n  reliability: reliable\n  durability: transient_local\n  history: keep_all\n')
    rclpy.init();node=rclpy.create_node('people_validation');messages=[];leaders=[]
    def receive(msg):
        messages.append({'sim_time':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,'frame':msg.header.frame_id,
            'people':[{'track_id':v.id,'x':v.position.x,'y':v.position.y,'vx':v.velocity.x,'vy':v.velocity.y,
                       'confidence':v.confidence,'appearance':list(v.appearance)} for v in msg.people]})
    node.create_subscription(People,'/r100_0001/leader_follow/people',receive,100)
    if a.appearance != 'none':
        node.create_subscription(LeaderState,'/r100_0001/leader_follow/leader',
            lambda m:leaders.append({'sim_time':m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                'frame':m.header.frame_id,'x':m.position.x,'y':m.position.y,
                'state':m.state,'reason':m.reason,'has_leader':m.has_leader,'track_id':m.track_id}),100)
    processes=[]
    with (a.output/'console.log').open('w') as log:
        try:
            from ament_index_python.packages import get_package_prefix
            executable=Path(get_package_prefix('ridgeback_leader_follow'))/'lib/ridgeback_leader_follow/people_tracker'
            tracker=subprocess.Popen([str(a.python),str(executable),'--ros-args','-r','__ns:=/r100_0001',
                '-r','/tf:=tf','-r','/tf_static:=tf_static','-p','use_sim_time:=true',
                '-p',f'appearance_candidate:={a.appearance}','-p',f'track_maximum_age:={a.track_maximum_age}','-p',f'observation_timeout:={a.observation_timeout}',
                '-p',f'appearance_model_dir:={a.appearance_model_dir.resolve()}',
                '-p',f'model_dir:={a.model_dir.resolve()}','-p',f'camera_candidate:={a.camera}','-p',f'lidar_candidate:={a.lidar}'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(tracker)
            if a.appearance != 'none':
                leader=subprocess.Popen(['ros2','run','ridgeback_leader_follow','leader_identity',
                    '--ros-args','-r','__ns:=/r100_0001','-r','/tf:=tf','-r','/tf_static:=tf_static',
                    '-p','use_sim_time:=true','-p','acquire_range:=8.0',
                    '-p',f'acquire_interval:={a.acquire_interval}',
                    '-p',f'acquire_distance_tolerance:={a.acquire_distance_tolerance}',
                    '-p',f'acquire_lateral_tolerance:={a.acquire_lateral_tolerance}',
                    '-p',f'match_threshold:={a.match_threshold}','-p',f'recovery_margin:={a.recovery_margin}'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                processes.append(leader)
            deadline=time.monotonic()+60
            # Start playback only when every node under test is up, so none joins mid-recording.
            while not node.count_publishers('/r100_0001/leader_follow/people') or (
                    a.appearance != 'none' and not node.count_publishers('/r100_0001/leader_follow/leader')):
                rclpy.spin_once(node,timeout_sec=.1)
                if any(process.poll() is not None for process in processes):raise RuntimeError('Node startup failed')
                if time.monotonic() > deadline:raise TimeoutError('Tracker/identity publisher discovery timed out')
            playback=subprocess.Popen(['ros2','bag','play',str(a.run/'sensors'),'--rate','.5','--clock','100',
                '--qos-profile-overrides-path',str(qos),'--disable-keyboard-controls','--start-paused'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(playback)
            resume=node.create_client(Resume,'/rosbag2_player/resume')
            deadline=time.monotonic()+30
            while not (resume.service_is_ready() and node.count_publishers('/r100_0001/sensors/camera_0/color/image')
                       and node.count_publishers('/r100_0001/tf')):
                rclpy.spin_once(node,timeout_sec=.05)
                if playback.poll() is not None:raise RuntimeError('Paused playback exited')
                if time.monotonic()>deadline:raise TimeoutError('Paused sensor publisher discovery timed out')
            future=resume.call_async(Resume.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=10)
            if not future.done() or future.exception():raise RuntimeError('Cannot resume sensor playback')
            node.destroy_client(resume)
            deadline=time.monotonic()+180
            while playback.poll() is None:
                rclpy.spin_once(node,timeout_sec=.02)
                if tracker.poll() is not None:raise RuntimeError('Tracker exited during sensor playback')
                if time.monotonic() > deadline:raise TimeoutError('Sensor playback timed out')
            for _ in range(20):rclpy.spin_once(node,timeout_sec=.05)
            if playback.returncode:raise RuntimeError('Sensor playback failed')
        finally:
            for process in reversed(processes):stop_process(process)
            node.destroy_node();rclpy.shutdown()
    (a.output/'people.jsonl').write_text(''.join(json.dumps(m)+'\n' for m in messages))
    if not messages or not any(m['people'] for m in messages):raise ValueError('No nonempty People messages')
    if any(m['frame'] != 'odom' for m in messages):raise ValueError('Incorrect People frame')
    if a.appearance != 'none':
        (a.output/'leader.jsonl').write_text(''.join(json.dumps(m)+'\n' for m in leaders))
        if not leaders or not any(p['appearance'] for m in messages for p in m['people']):
            raise ValueError('No appearance features or leader-state messages')
    (a.output/'validation.json').write_text(json.dumps({'status':'passed','messages':len(messages),
        'nonempty_messages':sum(bool(m['people']) for m in messages),'domain':a.ros_domain,
        'camera':a.camera,'lidar':a.lidar,'appearance':a.appearance,
        'appearance_observations':sum(bool(p['appearance']) for m in messages for p in m['people']),
        'track_maximum_age':a.track_maximum_age,'observation_timeout':a.observation_timeout,
        'identity_parameters':{'match_threshold':a.match_threshold,'recovery_margin':a.recovery_margin,
            'acquire_interval':a.acquire_interval,'acquire_distance_tolerance':a.acquire_distance_tolerance,
            'acquire_lateral_tolerance':a.acquire_lateral_tolerance},
        'appearance_dimensions':sorted({len(p['appearance']) for m in messages for p in m['people'] if p['appearance']}),
        'leader_messages':len(leaders),'locked_messages':sum(m['state']==LeaderState.LOCKED for m in leaders),'scope':('real ROS feature/LeaderState publication; no identity-accuracy qualification' if a.appearance != 'none' else 'real ROS node, sensors/TF from bag, People publication; no motion command')},indent=2)+'\n')


if __name__ == '__main__':main()
