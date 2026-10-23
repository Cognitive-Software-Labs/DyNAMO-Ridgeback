#!/usr/bin/env python3
"""Compile/execute the candidate consumer policy with simulated monotonic time."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args();out=a.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/hardware'):p.error('save below artifacts/hardware')
    out.mkdir(parents=True,exist_ok=False)
    header=Path(__file__).with_name('puma_feedback_safety.hpp').resolve()
    program='#include "'+str(header)+'"\n'+'''#include <cassert>
#include <iostream>
#include <limits>
using namespace clearpath_hardware_interfaces;
std::vector<FeedbackWheelSample> good(){return {{"fl",5,.5,1,0,0,0},{"fr",4,.5,1,0,0,0},{"rl",2,.5,1,0,0,0},{"rr",3,.5,1,0,0,0}};}
FeedbackSafety policy(){return FeedbackSafety({"fl","fr","rl","rr"});}
int main(){int checks=0;auto expect=[&](bool result){assert(result);checks++;};
 auto gate=policy();expect(!gate.safe(10));expect(gate.activate(good(),10,10));expect(gate.safe(10.25));
 expect(!gate.safe(10.251));expect(gate.faulted());expect(!gate.accept(good(),10.26,10.26));
 expect(!gate.safe(10.26));expect(gate.activate(good(),10.3,10.3));expect(gate.safe(10.3));
 gate.deactivate();expect(!gate.safe(10.31));expect(!gate.accept(good(),10.31,10.31));
 auto late=policy();expect(late.activate(good(),10,10));expect(!late.accept(good(),10.5,10.5));
 expect(late.faulted()); // Fresh arrival cannot hide the old gap.
 for(int test=0;test<10;test++){
  auto data=good();
  if(test==0)data.pop_back();if(test==1)data.push_back(data[0]);
  if(test==2)data[3].name="unknown";if(test==3)data[3].name="fl";
  if(test==4)data[3].device=5;if(test==5)data[3].device=0;
  if(test==6)data[3].speed=NAN;if(test==7)data[3].travel=INFINITY;
  if(test==8)data[3].current=NAN;if(test==9)data[3].setpoint=NAN;
  auto g=policy();expect(!g.activate(data,10,10));expect(g.activate(good(),10,10));
  expect(!g.accept(data,10.01,10.01));expect(g.faulted());expect(!g.accept(good(),10.02,10.02));
 }
 auto order=policy();expect(order.activate(good(),10,10));expect(!order.accept(good(),9.99,10.01));
 auto reverse=policy();expect(reverse.activate(good(),10,10));expect(!reverse.safe(9.9));
 auto stale=policy();expect(!stale.activate(good(),10,10.251));expect(!stale.activate(good(),NAN,10));
 auto flow=policy();expect(flow.activate(good(),10,10));
 for(int i=1;i<=50;i++){expect(flow.accept(good(),10+i*.1,10+i*.1));expect(flow.safe(10+i*.1));}
 std::cout<<checks<<" policy checks passed";
}
'''
    source=out/'policy-check.cpp';source.write_text(program)
    subprocess.run(['g++','-std=c++14','-O0',str(source),'-o',str(out/'policy-check')],check=True,capture_output=True,text=True)
    result=subprocess.run([str(out/'policy-check')],check=True,capture_output=True,text=True)
    report={'status':'offline-policy-verified-not-installed','stdout':result.stdout,
            'header_sha256':hashlib.sha256(header.read_bytes()).hexdigest(),
            'provisional_timeout_s':.25,'live_ros_or_hardware_access':False,
            'limits':'Policy only; adapter, installed consumer, physical braking and production timeout require separate qualification.'}
    (out/'result.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
