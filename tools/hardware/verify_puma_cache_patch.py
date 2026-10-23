#!/usr/bin/env python3
"""Test the pinned Puma cache repair with sanitizers, without ROS or CAN access.

Compiles the actual patched cache/decoder/message routines in a small harness.
The CAN-frame transport and diagnostic counter are inert substitutes; this is
not a firmware, SocketCAN, concurrency or physical-driver qualification.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
EXPECTED = {
    'include/puma_motor_driver/driver.hpp':
        '568c38600ef0381c2c7f12fad4fae97734028517a6aa257bceb5258887e8cf85',
    'src/driver.cpp':
        '657866879efbc4f4479bb0c880bacee30d6bb43ef848f32999f9047c6d799f15',
}
FAMILIES = ('voltage', 'spd', 'vcomp', 'pos', 'ictrl', 'status', 'cfg')


def block(source, marker):
    start = source.index(marker)
    opening = source.index('{', start)
    depth = 1
    index = opening + 1
    while depth:
        if source[index] == '{':
            depth += 1
        elif source[index] == '}':
            depth -= 1
        index += 1
    return source[start:index]


def harness(header, implementation):
    field = block(header, 'struct Field') + ';'
    arrays = '\n'.join(re.findall(r'  Field \w+_fields_\[[^]]+\];', header))
    constant = re.search(r'  static constexpr uint32_t CACHE_FIELD_COUNT =\s*[^;]+;', header)
    methods = '\n'.join(block(implementation, 'Driver::Field * Driver::'+f+'FieldForMessage')
                        for f in FAMILIES)
    methods += '\n' + block(implementation, 'uint32_t Driver::getApi')
    methods += '\n' + block(implementation, 'uint32_t Driver::getDeviceNumber')
    methods += '\n' + block(implementation, 'void Driver::processMessage')
    declarations = '\n'.join('Field * '+f+'FieldForMessage(uint32_t);' for f in FAMILIES)
    exercise = '\n'.join('for(unsigned i=0;i<16;++i){ auto * f=d.'+family+
                         'FieldForMessage(i<<CAN_MSGID_API_S); f->data[0]=i; '
                         'assert(f->data[0]==i); ++checks; }' for family in FAMILIES)
    return '''#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>
#include "can_proto.hpp"
namespace can_hardware { struct Frame { uint32_t id=0; uint8_t dlc=0;
  std::array<uint8_t,8> data{}; }; }
struct Counter { unsigned ticks=0; void tick(){++ticks;} };
class Driver { public:
''' + field + '\n' + (constant.group(0) if constant else '') + '\n' + arrays + '''
uint8_t device_number_=5;
std::shared_ptr<Counter> can_feedback_freq_status_=std::make_shared<Counter>();
uint32_t getApi(const can_hardware::Frame &);
uint32_t getDeviceNumber(const can_hardware::Frame &);
void processMessage(const can_hardware::Frame &);
''' + declarations + '\n};\n' + methods + '''
int main(int argc,char **argv){
 Driver d{}; unsigned checks=0;
 if(argc>1 && std::strcmp(argv[1],"alignment")==0){
   for(unsigned offset=0;offset<8;++offset){
     alignas(8) unsigned char storage[32]{};
     auto * f=new(storage+offset) Driver::Field{};
     int16_t v8=-128; std::memcpy(f->data,&v8,sizeof(v8));
     assert(f->interpretFixed8x8()==-0.5f); ++checks;
     int32_t v16=-65536; std::memcpy(f->data,&v16,sizeof(v16));
     assert(f->interpretFixed16x16()==-1.0); ++checks;
   }
 }else{
''' + exercise + '''
 const uint32_t classes[]={CAN_API_MC_VOLTAGE,CAN_API_MC_SPD,CAN_API_MC_VCOMP,
   CAN_API_MC_POS,CAN_API_MC_ICTRL,CAN_API_MC_STATUS,CAN_API_MC_CFG};
 for(auto cls:classes)for(unsigned i=0;i<16;++i)for(unsigned n=1;n<=8;++n){
   can_hardware::Frame frame; frame.id=cls|(i<<CAN_MSGID_API_S)|5;
   frame.dlc=n; frame.data.fill(0xa5); d.processMessage(frame);
   Driver::Field * f=nullptr;
   switch(cls){
    case CAN_API_MC_VOLTAGE:f=d.voltageFieldForMessage(frame.id);break;
    case CAN_API_MC_SPD:f=d.spdFieldForMessage(frame.id);break;
    case CAN_API_MC_VCOMP:f=d.vcompFieldForMessage(frame.id);break;
    case CAN_API_MC_POS:f=d.posFieldForMessage(frame.id);break;
    case CAN_API_MC_ICTRL:f=d.ictrlFieldForMessage(frame.id);break;
    case CAN_API_MC_STATUS:f=d.statusFieldForMessage(frame.id);break;
    case CAN_API_MC_CFG:f=d.cfgFieldForMessage(frame.id);break;
   }
   assert(f && f->received);
   for(unsigned b=0;b<4;++b)assert(f->data[b]==(b<n?0xa5:0));
   ++checks;
 }
 // Other-device messages, empty requests, invalid lengths and unsupported
 // classes must leave both the cache and diagnostic counter unchanged.
 for(auto cls:{CAN_API_MC_PSTAT,CAN_API_MC_ACK}){
   Driver empty{}; can_hardware::Frame frame;frame.id=cls|5;frame.dlc=4;
   frame.data.fill(0xff);empty.processMessage(frame);
   assert(empty.can_feedback_freq_status_->ticks==0);
   assert(!empty.ictrl_fields_[0].received);++checks;
 }
 for(unsigned kind=0;kind<3;++kind){
   Driver empty{};can_hardware::Frame frame;frame.id=CAN_API_MC_STATUS|5;
   frame.dlc=4;frame.data.fill(0xff);
   if(kind==0)frame.id=CAN_API_MC_STATUS|4;
   if(kind==1)frame.dlc=0;
   if(kind==2)frame.dlc=9;
   empty.processMessage(frame);assert(!empty.status_fields_[0].received);++checks;
 }
 }
 std::printf("%u checks passed\\n",checks);return 0;
}
'''


def verify(package, output):
    output.mkdir(parents=True, exist_ok=False)
    original = {name: (package/name).read_bytes() for name in EXPECTED}
    for name, digest in EXPECTED.items():
        if hashlib.sha256(original[name]).hexdigest() != digest:
            raise ValueError('source differs from inspected Clearpath 2.9.4: '+name)
    with tempfile.TemporaryDirectory(prefix='puma-cache-check-') as temp:
        tree = Path(temp)
        target = tree/'clearpath_motor_drivers/puma_motor_driver'
        for name, data in original.items():
            (target/name).parent.mkdir(parents=True, exist_ok=True)
            (target/name).write_bytes(data)
        (tree/'can_proto.hpp').write_bytes((package/'include/puma_motor_driver/can_proto.hpp').read_bytes())
        before = harness(original['include/puma_motor_driver/driver.hpp'].decode(),
                         original['src/driver.cpp'].decode())
        patch = ROOT/'patches/clearpath_puma_cache_safety.patch'
        subprocess.run(['git','apply','--check',str(patch)],cwd=tree,check=True)
        subprocess.run(['git','apply',str(patch)],cwd=tree,check=True)
        after = harness((target/'include/puma_motor_driver/driver.hpp').read_text(),
                        (target/'src/driver.cpp').read_text())
        environment = {**os.environ, 'ASAN_OPTIONS':'detect_leaks=0',
                       'UBSAN_OPTIONS':'halt_on_error=1:print_stacktrace=1'}
        results = {}
        for label, program in [('upstream',before),('patched',after)]:
            source = tree/(label+'.cpp'); source.write_text(program)
            (output/source.name).write_text(program)
            binary = tree/label
            # O0 keeps the deliberately unaligned loads from being folded to
            # constants before UBSan can check them in the upstream control.
            subprocess.run(['g++','-std=c++17','-O0','-g','-fno-omit-frame-pointer',
                            '-fsanitize=address,undefined','-fno-sanitize-recover=all',
                            str(source),'-o',str(binary)],check=True,capture_output=True,text=True)
            for case in ('cache','alignment'):
                result = subprocess.run([str(binary),case],env=environment,capture_output=True,text=True)
                (output/(label+'-'+case+'.log')).write_text(result.stdout+result.stderr)
                if label=='upstream' and result.returncode==0:
                    raise RuntimeError('upstream regression was not reproduced: '+case)
                expected_error = 'out of bounds' if case=='cache' else 'misaligned address'
                if label=='upstream' and expected_error not in result.stderr:
                    raise RuntimeError('upstream failed for an unexpected reason: '+case)
                if label=='patched' and result.returncode!=0:
                    raise RuntimeError('patched sanitizer check failed: '+case)
                results[label+'-'+case]={'returncode':result.returncode,'stdout':result.stdout.strip()}
        record={'status':'sanitizer-verified-not-installed','source_hashes':EXPECTED,
                'patch_sha256':hashlib.sha256(patch.read_bytes()).hexdigest(),
                'results':results,'live_ros_or_can_access':False,
                'limits':'Extracted actual routines; inert transport/counter; full package build and live qualification separate.'}
        (output/'result.json').write_text(json.dumps(record,indent=2)+'\n')
        return record


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(json.dumps(verify(args.package,args.output),indent=2))
