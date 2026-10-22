#!/usr/bin/env python3
"""Execute actual publication/diagnostic routines with inert per-wheel caches."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

ROOT=Path(__file__).resolve().parents[2]


def function(source,signature):
    start=source.index(signature);brace=source.index('{',start);depth=0
    for i in range(brace,len(source)):
        depth+=(source[i]=='{')-(source[i]=='}')
        if depth==0:return source[start:i+1]
    raise ValueError('unbalanced routine')


def harness(package):
    node=(package/'src/multi_puma_node.cpp').read_text()
    driver=(package/'src/driver.cpp').read_text()
    code='''#include <array>
#include <memory>
#include <string>
#include <iostream>
#define RCLCPP_DEBUG(...) ((void)0)
namespace FeedbackBit {enum {DutyCycle,Current,Position,Speed,Setpoint,Count};}
namespace StatusBit {enum {BusVoltage,OutVoltage,AnalogInput,Temperature,Mode,Fault,Count};}
struct DiagnosticStatusWrapper {template<class T> void add(const char*,T){};};
namespace diagnostic_updater {using DiagnosticStatusWrapper=::DiagnosticStatusWrapper;}
struct Frequency {void run(DiagnosticStatusWrapper&){};};
struct Driver {
 std::array<bool,5> fb{true,true,true,true,true};
 std::array<bool,6> st{true,true,true,true,true,true};
 std::shared_ptr<Frequency> can_feedback_freq_status_=std::make_shared<Frequency>();
 bool receivedDutyCycle(){return fb[0];} bool receivedCurrent(){return fb[1];}
 bool receivedPosition(){return fb[2];} bool receivedSpeed(){return fb[3];} bool receivedSetpoint(){return fb[4];}
 bool receivedBusVoltage(){return st[0];} bool receivedOutVoltage(){return st[1];}
 bool receivedAnalogInput(){return st[2];} bool receivedTemperature(){return st[3];}
 bool receivedMode(){return st[4];} bool receivedFault(){return st[5];}
 double take(int i){fb[i]=false;return 0;}
 double lastDutyCycle(){return take(0);} double lastCurrent(){return take(1);}
 double lastPosition(){return take(2);} double lastSpeed(){return take(3);} double lastSetpoint(){return take(4);}
 double lastBusVoltage(){st[0]=false;return 0;} double lastOutVoltage(){st[1]=false;return 0;}
 double lastAnalogInput(){st[2]=false;return 0;} double lastTemperature(){st[3]=false;return 0;}
 double lastMode(){st[4]=false;return 0;} double lastFault(){st[5]=false;return 0;}
 int deviceNumber(){return 1;} std::string deviceName(){return "mock";}
 void runFreqStatus(DiagnosticStatusWrapper&);
};
namespace clearpath_motor_msgs {namespace msg {
struct PumaFeedback {int device_number;std::string device_name;double duty_cycle,current,travel,speed,setpoint;};
struct PumaStatus {int device_number;std::string device_name;double bus_voltage,output_voltage,analog_input,temperature,mode,fault;};
}}
struct Header{int stamp=0;};struct Clock{int now(){return 0;}};
struct MultiPumaNode {
 std::array<Driver,4> drivers_;
 struct {std::array<clearpath_motor_msgs::msg::PumaFeedback,4> drivers_feedback;Header header;} feedback_msg_;
 struct {std::array<clearpath_motor_msgs::msg::PumaStatus,4> drivers;Header header;} status_msg_;
 Clock clock;Clock* get_clock(){return &clock;}
 bool getFeedback();bool getStatus();
};
'''
    code+='\n'+function(node,'bool MultiPumaNode::getFeedback()')+'\n'
    code+=function(node,'bool MultiPumaNode::getStatus()')+'\n'
    code+=function(driver,'void Driver::runFreqStatus(')+'\n'
    code+='''int main(){
 int fb_bad=0,st_bad=0;
 for(int wheel=0;wheel<4;wheel++)for(int field=0;field<5;field++){
  MultiPumaNode n;n.drivers_[wheel].fb[field]=false;if(n.getFeedback())fb_bad++;
 }
 for(int wheel=0;wheel<4;wheel++)for(int field: {0,1,3,4,5}){
  MultiPumaNode n;n.drivers_[wheel].st[field]=false;if(n.getStatus())st_bad++;
 }
 MultiPumaNode partial;for(auto& d:partial.drivers_)d.fb.fill(false);
 for(int field=0;field<5;field++)partial.drivers_[field%4].fb[field]=true;
 bool distributed_bad=partial.getFeedback();
 MultiPumaNode diag;DiagnosticStatusWrapper stat;
 for(auto& d:diag.drivers_)d.runFreqStatus(stat);
 bool diagnostics_preserved=diag.getFeedback();
 MultiPumaNode complete;
 if(!complete.getFeedback() || complete.getFeedback())return 2;
 MultiPumaNode status;
 if(!status.getStatus() || status.getStatus())return 3;
 std::cout<<"{\\\"feedback_false_acceptances\\\":"<<fb_bad
 <<",\\\"status_false_acceptances\\\":"<<st_bad
 <<",\\\"distributed_fields_false_acceptance\\\":"<<distributed_bad
 <<",\\\"diagnostics_preserve_feedback\\\":"<<diagnostics_preserved<<"}";
}
'''
    return code


def verify(package,output):
    package=package.resolve();output=output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/hardware'):
        raise ValueError('output must be below artifacts/hardware')
    output.mkdir(parents=True,exist_ok=False)
    tree=output/'patched';target=tree/'clearpath_motor_drivers/puma_motor_driver'
    shutil.copytree(package,target)
    patch=ROOT/'patches/clearpath_puma_freshness.patch'
    from build_puma_repair import apply_patch_checked
    apply_patch_checked(tree,patch)
    results={}
    for label,src in [('baseline',package),('patched',target)]:
        program=output/(label+'.cpp');program.write_text(harness(src));binary=output/(label+'-check')
        subprocess.run(['g++','-std=c++17','-O0',str(program),'-o',str(binary)],check=True,capture_output=True,text=True)
        run=subprocess.run([str(binary)],check=True,capture_output=True,text=True)
        results[label]=json.loads(run.stdout)
    if not (results['baseline']['feedback_false_acceptances']==20 and
            results['baseline']['status_false_acceptances']==15 and
            results['baseline']['distributed_fields_false_acceptance']==1 and
            results['baseline']['diagnostics_preserve_feedback']==0):
        raise RuntimeError('baseline behavior differs from inspected source')
    expected={'feedback_false_acceptances':0,'status_false_acceptances':0,
              'distributed_fields_false_acceptance':0,'diagnostics_preserve_feedback':1}
    if results['patched']!=expected:raise RuntimeError('freshness candidate regression failed')
    record={'status':'verified-extracted-routines-not-installed','results':results,
            'source_hashes':{n:hashlib.sha256((package/n).read_bytes()).hexdigest() for n in ['src/driver.cpp','src/multi_puma_node.cpp']},
            'patch_sha256':hashlib.sha256(patch.read_bytes()).hexdigest(),
            'live_ros_or_can_access':False,
            'limits':'Actual node and diagnostic routine bodies; inert caches model consuming getters. No age bounds, rate qualification or physical synchronization proven.'}
    (output/'result.json').write_text(json.dumps(record,indent=2)+'\n')
    return record


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--package',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();print(json.dumps(verify(args.package,args.output),indent=2))
