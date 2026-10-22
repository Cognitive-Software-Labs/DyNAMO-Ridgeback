#!/usr/bin/env python3
"""Execute actual candidate transport/read/write bodies with inert ROS and a test clock."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from verify_puma_freshness_patch import function
ROOT=Path(__file__).resolve().parents[2]


def check(package,output):
    output=output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/hardware'):raise ValueError('save below artifacts/hardware')
    output.mkdir(parents=True,exist_ok=False)
    hardware=(package/'src/puma/hardware.cpp').read_text();transport=(package/'src/puma/hardware_interface.cpp').read_text()
    code='#include "'+str(Path(__file__).with_name('puma_feedback_safety.hpp').resolve())+'"\n'+'''
#include <memory>
#include <chrono>
#include <mutex>
#include <deque>
#include <map>
#include <algorithm>
#include <cassert>
#include <iostream>
#define RCLCPP_DEBUG(...) ((void)0)
using namespace clearpath_hardware_interfaces;
double clock_now=10.;
struct FakeSteady {static std::chrono::steady_clock::time_point now(){return std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(clock_now)));}};
struct Stamp{long long ns=10000000000LL;};
namespace rclcpp {struct Time{Time(){}Time(Stamp s):stamp(s){}Stamp stamp;double seconds()const{return stamp.ns/1.e9;}long long nanoseconds()const{return stamp.ns;}};struct Duration{};template<class T>void spin_some(T){}}
namespace rclcpp_lifecycle {struct State{};}
namespace hardware_interface {enum class return_type{OK,ERROR};enum class CallbackReturn{SUCCESS,ERROR};}
struct FakeSleep{static void sleep_for(std::chrono::milliseconds d){clock_now+=d.count()/1000.;}};
namespace clearpath_motor_msgs {namespace msg {
struct Wheel{std::string device_name;unsigned device_number;double speed,travel,duty_cycle,current,setpoint;};
struct PumaMultiFeedback {using SharedPtr=std::shared_ptr<PumaMultiFeedback>;struct{Stamp stamp;}header;std::vector<Wheel> drivers_feedback;};
}}
namespace sensor_msgs {namespace msg {struct JointState{std::vector<std::string>name;std::vector<double>velocity;};}}
struct Clock {rclcpp::Time now(){rclcpp::Time t;t.stamp.ns=static_cast<long long>(clock_now*1.e9);return t;}};
struct PumaHardwareInterface{
 clearpath_motor_msgs::msg::PumaMultiFeedback feedback_;bool has_feedback_=false,invalid_header_pending_=false;
 std::mutex feedback_mutex_;std::chrono::steady_clock::time_point feedback_receipt_;Clock clock;
 struct PendingFeedback {clearpath_motor_msgs::msg::PumaMultiFeedback msg;std::chrono::steady_clock::time_point receipt;};
 std::deque<PendingFeedback>pending_feedback_;
 std::vector<sensor_msgs::msg::JointState>sent;Clock*get_clock(){return &clock;}
 void drive_command(sensor_msgs::msg::JointState msg){sent.push_back(msg);}
 void feedback_callback(const clearpath_motor_msgs::msg::PumaMultiFeedback::SharedPtr);
 bool take_feedback(clearpath_motor_msgs::msg::PumaMultiFeedback&,std::chrono::steady_clock::time_point&,bool&);
};
static constexpr double MINIMUM_VELOCITY=.01f;
struct PumaHardware{
 std::shared_ptr<PumaHardwareInterface>node_=std::make_shared<PumaHardwareInterface>();
 std::unique_ptr<FeedbackSafety>feedback_safety_;bool feedback_active_=false;
 std::map<std::string,unsigned>wheel_joints_{{"fl",0},{"fr",1},{"rl",2},{"rr",3}};
 std::vector<double>hw_commands_{.4,.4,.4,.4},hw_states_velocity_{0,0,0,0},hw_states_position_{0,0,0,0},hw_states_position_offset_{0,0,0,0};
 hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State&);
 hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State&);
 hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State&);
 void inhibitCommands();void writeCommandsToHardware();void updateJointsFromHardware();
 hardware_interface::return_type read(const rclcpp::Time&,const rclcpp::Duration&);
 hardware_interface::return_type write(const rclcpp::Time&,const rclcpp::Duration&);
};
'''
    for signature in ['double monotonicSeconds(', 'std::vector<FeedbackWheelSample> samples(',
                      'void PumaHardware::inhibitCommands()', 'void PumaHardware::writeCommandsToHardware()',
                      'void PumaHardware::updateJointsFromHardware()',
                      'hardware_interface::CallbackReturn PumaHardware::on_activate(',
                      'hardware_interface::CallbackReturn PumaHardware::on_deactivate(',
                      'hardware_interface::CallbackReturn PumaHardware::on_error(',
                      'hardware_interface::return_type PumaHardware::read(',
                      'hardware_interface::return_type PumaHardware::write(']:
        code+='\n'+function(hardware,signature).replace('std::chrono::steady_clock::now()','FakeSteady::now()').replace('std::this_thread::sleep_for','FakeSleep::sleep_for')+'\n'
    for signature in ['void PumaHardwareInterface::feedback_callback(', 'bool PumaHardwareInterface::take_feedback(']:
        code+='\n'+function(transport,signature).replace('std::chrono::steady_clock::now()','FakeSteady::now()').replace('std::this_thread::sleep_for','FakeSleep::sleep_for')+'\n'
    code+='''
using Return=hardware_interface::return_type;
clearpath_motor_msgs::msg::PumaMultiFeedback::SharedPtr batch(){
 auto m=std::make_shared<clearpath_motor_msgs::msg::PumaMultiFeedback>();m->header.stamp.ns=static_cast<long long>(clock_now*1.e9);
 m->drivers_feedback={{"fl",5,.5,1,0,0,0},{"fr",4,.5,1,0,0,0},{"rl",2,.5,1,0,0,0},{"rr",3,.5,1,0,0,0}};return m;
}
PumaHardware healthy(){clock_now=10.;PumaHardware h;
 h.feedback_active_=true;h.feedback_safety_.reset(new FeedbackSafety({"fl","fr","rl","rr"}));assert(h.feedback_safety_->activate(samples(*batch()),10,10));
 clock_now=10.05;h.node_->feedback_callback(batch());assert(h.read({},{})==Return::OK);return h;
}
bool zero(const PumaHardware& h){if(h.node_->sent.empty())return false;for(double v:h.node_->sent.back().velocity)if(v!=0)return false;return true;}
int main(){int checks=0;auto expect=[&](bool b){assert(b);checks++;};
 auto h=healthy();expect(h.write({},{})==Return::OK);expect(!zero(h));
 clock_now=10.301;expect(h.read({},{})==Return::ERROR);expect(zero(h));expect(h.hw_states_velocity_[0]==.5);
 clock_now=10.31;h.node_->feedback_callback(batch());expect(h.read({},{})==Return::ERROR);
 h.hw_commands_.assign(4,.4);expect(h.write({},{})==Return::ERROR);expect(zero(h));
 for(int i=0;i<7;i++){
  auto bad=healthy();clock_now=10.1;auto m=batch();
  if(i==0)m->drivers_feedback.pop_back();if(i==1)m->drivers_feedback[3].device_name="unknown";
  if(i==2)m->drivers_feedback[3].device_name="fl";if(i==3)m->drivers_feedback[3].device_number=5;
  if(i==4)m->drivers_feedback[3].speed=NAN;if(i==5)m->drivers_feedback[3].travel=INFINITY;
  if(i==6)m->drivers_feedback[3].setpoint=NAN;
  bad.node_->feedback_callback(m);expect(bad.read({},{})==Return::ERROR);expect(zero(bad));
  expect(bad.hw_states_velocity_[0]==.5&&bad.hw_states_velocity_[3]==.5);expect(bad.wheel_joints_.size()==4);
 }
 for(int kind=0;kind<3;kind++){
  auto old=healthy();clock_now=10.1;auto m=batch();
  m->header.stamp.ns=kind==0?0:kind==1?static_cast<long long>(9.*1.e9):static_cast<long long>(11.*1.e9);
  old.node_->feedback_callback(m);old.node_->feedback_callback(batch());
  expect(old.read({},{})==Return::ERROR);expect(zero(old)); // A good packet cannot hide queued invalid timestamp.
 }
 auto masked=healthy();clock_now=10.1;auto malformed=batch();malformed->drivers_feedback.pop_back();
 masked.node_->feedback_callback(malformed);masked.node_->feedback_callback(batch());
 expect(masked.read({},{})==Return::ERROR);expect(zero(masked));
 auto overflow=healthy();clock_now=10.1;for(int i=0;i<33;i++)overflow.node_->feedback_callback(batch());
 expect(overflow.read({},{})==Return::ERROR);expect(zero(overflow));
 auto swapped=healthy();clock_now=10.1;auto swap=batch();std::swap(swap->drivers_feedback[0].device_number,swap->drivers_feedback[1].device_number);
 swapped.node_->feedback_callback(swap);expect(swapped.read({},{})==Return::ERROR);expect(zero(swapped));
 auto queued_valid=healthy();clock_now=10.1;queued_valid.node_->feedback_callback(batch());clock_now=10.15;queued_valid.node_->feedback_callback(batch());
 expect(queued_valid.read({},{})==Return::OK);expect(queued_valid.node_->pending_feedback_.empty());
 auto errored=healthy();expect(errored.on_error({})==hardware_interface::CallbackReturn::SUCCESS);expect(zero(errored));expect(!errored.feedback_active_);
 auto aged=healthy();clock_now=10.15;auto delayed_message=batch();delayed_message->header.stamp.ns=10060000000LL;
 aged.node_->feedback_callback(delayed_message);expect(aged.read({},{})==Return::OK);
 clock_now=10.32;expect(aged.write({},{})==Return::ERROR);expect(zero(aged));
 auto command=healthy();command.hw_commands_[0]=NAN;expect(command.write({},{})==Return::ERROR);expect(zero(command));
 auto stalled=healthy();clock_now=10.4;stalled.node_->feedback_callback(batch());expect(stalled.read({},{})==Return::ERROR);expect(zero(stalled));
 auto normal=healthy();for(int i=1;i<=20;i++){clock_now=10.05+i*.1;normal.node_->feedback_callback(batch());expect(normal.read({},{})==Return::OK);expect(normal.write({},{})==Return::OK);}
 using Life=hardware_interface::CallbackReturn;
 clock_now=20.;PumaHardware absent;expect(absent.on_activate({})==Life::ERROR);expect(zero(absent));expect(clock_now>=25.);
 clock_now=30.;PumaHardware startup;auto initial=batch();for(auto& wheel:initial->drivers_feedback)wheel.speed=0;
 startup.node_->feedback_callback(initial);expect(startup.on_activate({})==Life::SUCCESS);expect(zero(startup));
 expect(startup.hw_states_position_[0]==1);expect(startup.hw_states_velocity_[0]==0);
 startup.hw_commands_.assign(4,.4);expect(startup.write({},{})==Return::OK);
 expect(startup.on_deactivate({})==Life::SUCCESS);expect(zero(startup));expect(startup.write({},{})==Return::OK);expect(zero(startup));
 clock_now=40.;PumaHardware moving;moving.node_->feedback_callback(batch());expect(moving.on_activate({})==Life::ERROR);expect(zero(moving));
 expect(moving.hw_states_position_[0]==0);
 clock_now=50.;PumaHardware invalid;auto empty=batch();empty->drivers_feedback.clear();invalid.node_->feedback_callback(empty);
 expect(invalid.on_activate({})==Life::ERROR);expect(zero(invalid));
 clock_now=60.;PumaHardware queued;auto delayed=batch();for(auto& wheel:delayed->drivers_feedback)wheel.speed=0;
 queued.node_->feedback_callback(delayed);clock_now=60.3;expect(queued.on_activate({})==Life::ERROR);expect(zero(queued));
 std::cout<<checks<<" adapter/lifecycle checks passed";
}
'''
    cpp=output/'adapter.cpp';cpp.write_text(code)
    subprocess.run(['g++','-std=c++14','-O0',str(cpp),'-o',str(output/'adapter-check')],check=True,capture_output=True,text=True)
    result=subprocess.run([str(output/'adapter-check')],check=True,capture_output=True,text=True)
    report={'status':'candidate-adapter-verified-offline-not-installed','stdout':result.stdout,
            'source_hashes':{n:hashlib.sha256((package/n).read_bytes()).hexdigest() for n in ['src/puma/hardware.cpp','src/puma/hardware_interface.cpp']},
            'policy_sha256':hashlib.sha256(Path(__file__).with_name('puma_feedback_safety.hpp').read_bytes()).hexdigest(),
            'live_ros_or_hardware_access':False,
            'limits':'Actual runtime read/write, callback and lifecycle bodies; injected clocks and inert ROS/command sink. Real ROS lifecycle/controller-manager integration and physical stopping unqualified.'}
    (output/'result.json').write_text(json.dumps(report,indent=2)+'\n');return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--package',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(check(a.package,a.output),indent=2))
