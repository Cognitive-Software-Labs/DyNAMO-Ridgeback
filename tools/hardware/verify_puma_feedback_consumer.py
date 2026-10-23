#!/usr/bin/env python3
"""Exercise actual hardware-consumer routines with inert feedback and command sinks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from verify_puma_freshness_patch import function
ROOT=Path(__file__).resolve().parents[2]


def verify(source,output):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/hardware'):raise ValueError('save below artifacts/hardware')
    output.mkdir(parents=True,exist_ok=False)
    raw=source.read_text()
    program='''#include <vector>
#include <map>
#include <string>
#include <memory>
#include <cmath>
#include <iostream>
#define RCLCPP_DEBUG(...) ((void)0)
static constexpr double MINIMUM_VELOCITY=0.01f;
namespace rclcpp {struct Time{double value;};struct Duration{double value;};template<class T>void spin_some(T){}}
namespace hardware_interface {enum class return_type {OK,ERROR};}
namespace sensor_msgs {namespace msg {struct JointState{std::vector<std::string> name;std::vector<double> velocity;};}}
struct Wheel{std::string device_name;double speed;double travel;};
struct Feedback{std::vector<Wheel> drivers_feedback;};
struct Node{
 bool fresh=false;Feedback feedback;std::vector<sensor_msgs::msg::JointState> sent;
 bool has_new_feedback(){return fresh;} Feedback get_feedback(){fresh=false;return feedback;}
 void drive_command(sensor_msgs::msg::JointState msg){sent.push_back(msg);}
};
struct PumaHardware{
 std::shared_ptr<Node> node_=std::make_shared<Node>();
 std::map<std::string,int> wheel_joints_{{"fl",0},{"fr",1},{"rl",2},{"rr",3}};
 std::vector<double> hw_commands_{.4,.4,.4,.4},hw_states_velocity_{0,0,0,0},hw_states_position_{0,0,0,0};
 std::string hw_name_="inert";
 void writeCommandsToHardware();void updateJointsFromHardware();
 hardware_interface::return_type read(const rclcpp::Time&,const rclcpp::Duration&);
 hardware_interface::return_type write(const rclcpp::Time&,const rclcpp::Duration&);
};
'''
    for signature in ['void PumaHardware::writeCommandsToHardware()',
                      'void PumaHardware::updateJointsFromHardware()',
                      'hardware_interface::return_type PumaHardware::read(',
                      'hardware_interface::return_type PumaHardware::write(']:
        program+='\n'+function(raw,signature)+'\n'
    program+='''int main(){
 PumaHardware h;h.node_->feedback.drivers_feedback={{"fl",.5,10},{"fr",.5,10},{"rl",.5,10},{"rr",.5,10}};
 h.node_->fresh=true;h.read({0},{.05});
 int success=0;
 for(int i=1;i<=100;i++){
  if(h.read({i*.05},{.05})==hardware_interface::return_type::OK)success++;
  h.write({i*.05},{.05});
 }
 int nonzero=0;for(auto& command:h.node_->sent)for(auto v:command.velocity)if(v!=0)nonzero++;
 bool stale_kept=true;for(auto v:h.hw_states_velocity_)if(v!=.5)stale_kept=false;
 h.node_->feedback.drivers_feedback={{"fl",1,11}};h.node_->fresh=true;h.read({5.05},{.05});
 bool mixed=h.hw_states_velocity_[0]==1&&h.hw_states_velocity_[3]==.5;
 h.node_->feedback.drivers_feedback={{"unknown",2,12}};h.node_->fresh=true;h.read({5.1},{.05});
 bool unknown_changed_first=h.hw_states_velocity_[0]==2;
 h.node_->feedback.drivers_feedback={{"fl",NAN,12}};h.node_->fresh=true;
 bool nan_accepted=h.read({5.15},{.05})==hardware_interface::return_type::OK&&std::isnan(h.hw_states_velocity_[0]);
 std::cout<<"{\\\"missing_feedback_read_successes\\\":"<<success
 <<",\\\"nonzero_command_values_without_feedback\\\":"<<nonzero
 <<",\\\"stale_velocities_kept\\\":"<<stale_kept
 <<",\\\"partial_batch_mixes_old_states\\\":"<<mixed
 <<",\\\"unknown_name_overwrites_first_joint\\\":"<<unknown_changed_first
 <<",\\\"nonfinite_feedback_accepted\\\":"<<nan_accepted<<"}";
}
'''
    cpp=output/'consumer.cpp';cpp.write_text(program)
    subprocess.run(['g++','-std=c++17','-O0',str(cpp),'-o',str(output/'consumer-check')],capture_output=True,text=True,check=True)
    run=subprocess.run([str(output/'consumer-check')],capture_output=True,text=True,check=True)
    results=json.loads(run.stdout)
    expected={'missing_feedback_read_successes':100,'nonzero_command_values_without_feedback':400,
              'stale_velocities_kept':1,'partial_batch_mixes_old_states':1,
              'unknown_name_overwrites_first_joint':1,'nonfinite_feedback_accepted':1}
    if results!=expected:raise RuntimeError('consumer source differs from inspected behavior')
    record={'status':'source-behavior-reproduced-offline','results':results,
            'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'simulated_missing_feedback_seconds':5.,'live_ros_or_hardware_access':False,
            'limits':'Actual method bodies, inert ROS spin/feedback and command sink. No live feed interruption, motor command, physical behavior or safety qualification.'}
    (output/'result.json').write_text(json.dumps(record,indent=2)+'\n');return record


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();print(json.dumps(verify(a.source,a.output),indent=2))
