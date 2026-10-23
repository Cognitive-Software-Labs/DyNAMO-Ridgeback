#!/usr/bin/env python3
"""Wait for the named, addressed interfaces in the deployed DDS profile.

Reads XML and `ip -json address`; never changes interfaces or the DDS profile.
This is network readiness only, not ROS, controller or physical-stop readiness.
"""
import argparse
import ipaddress
import json
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET


def requirements(profile):
    root = ET.parse(profile).getroot()
    domains = [element for element in root if element.tag.split('}')[-1]=='Domain'
               and element.get('id', 'any') in ('any', '0')]
    if len(domains)!=1:
        raise ValueError('expected one deployed domain-0/any DDS configuration')
    domain=domains[0]
    transport=next((element.text.strip().lower() for element in domain.iter()
                    if element.tag.split('}')[-1]=='Transport'), 'udp')
    if transport not in ('udp','udp4','udp6','tcp','tcp4','tcp6'):
        raise ValueError('unsupported DDS transport for interface readiness: '+transport)
    family='inet6' if transport.endswith('6') else 'inet'
    interfaces=[element for element in domain.iter()
                if element.tag.split('}')[-1]=='NetworkInterface']
    result=[]
    for interface in interfaces:
        name=interface.get('name')
        if not name or interface.get('autodetermine','false')!='false':
            raise ValueError('readiness requires explicit interface names')
        address=interface.get('address')
        if address:
            parsed=ipaddress.ip_address(address)
            if parsed.version != (6 if family=='inet6' else 4):
                raise ValueError('interface address and DDS transport disagree')
        result.append((name,family,address))
    if not result:
        raise ValueError('DDS profile has no explicit named interfaces')
    return result


def missing(required, interfaces):
    current={interface['ifname']:interface for interface in interfaces}
    unavailable=[]
    for name,family,address in required:
        interface=current.get(name)
        addresses=interface.get('addr_info',[]) if interface else []
        if (not interface or 'UP' not in interface.get('flags',[]) or
                not any(value.get('family')==family and
                        (not address or value.get('local')==address)
                        for value in addresses)):
            unavailable.append(name)
    return unavailable


def wait(profile, timeout, monotonic=time.monotonic, sleep=time.sleep, read=None):
    required=requirements(profile)
    deadline=monotonic()+timeout
    if read is None:
        def read():
            result=subprocess.run(['/usr/sbin/ip','-json','address','show'],
                                  capture_output=True,text=True,check=True,timeout=3)
            return json.loads(result.stdout)
    while True:
        unavailable=missing(required,read())
        if not unavailable:
            return {'status':'interfaces-ready','interfaces':[r[0] for r in required]}
        if monotonic()>=deadline:
            raise RuntimeError('DDS interfaces unavailable or unaddressed: '+', '.join(unavailable))
        sleep(min(.2,max(0.,deadline-monotonic())))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=60)
    args=parser.parse_args()
    if not 0<=args.timeout<=60:
        parser.error('timeout must be between zero and 60 seconds')
    try:
        print(json.dumps(wait(args.profile,args.timeout)))
    except Exception as error:
        parser.exit(1,'DDS readiness failed: '+str(error)+'\n')
