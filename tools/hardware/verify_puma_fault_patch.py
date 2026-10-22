#!/usr/bin/env python3
"""Verify the diagnostic-only patch against downloaded Clearpath 2.9.4 source.

No network, ROS participation or installed changes. Needs a C++ compiler.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
EXPECTED_SOURCE_SHA256 = 'd156ed91b28d5526de64c625cad7555b7728f54c7c1e020c9b341bbabccda89b'


def verify(source):
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != EXPECTED_SOURCE_SHA256:
        raise ValueError('source differs from inspected Clearpath 2.9.4 source')
    with tempfile.TemporaryDirectory(prefix='puma-fault-check-') as directory:
        root = Path(directory)
        path = root/'clearpath_motor_drivers/puma_motor_driver/src/multi_puma_node.cpp'
        path.parent.mkdir(parents=True)
        path.write_bytes(raw)
        patch = ROOT/'patches/clearpath_puma_fault_flags.patch'
        subprocess.run(['git', 'apply', '--check', str(patch)], cwd=root, check=True)
        subprocess.run(['git', 'apply', str(patch)], cwd=root, check=True)
        expression = re.search(r'bool flag = (.*);', path.read_text()).group(1)
        expression = expression.replace('status_msg_.drivers[i].fault', 'fault').replace('label.first', 'mask')
        program = root/'check.cpp'
        program.write_text('int main(){for(unsigned fault=0;fault<256;++fault)'
            '{for(unsigned mask: {1u,2u,4u,8u}){bool flag='+expression+';'
            'if(flag != ((fault & mask)!=0))return 1;}}return 0;}\n')
        # initializer_list is the only header required by this extracted expression.
        program.write_text('#include <initializer_list>\n'+program.read_text())
        subprocess.run(['g++', '-std=c++17', str(program), '-o', str(root/'check')], check=True)
        subprocess.run([str(root/'check')], check=True)
        return {'status': 'verified-patch-only-not-installed', 'source_sha256': EXPECTED_SOURCE_SHA256,
                'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest(),
                'fault_mask_cases': 1024, 'changes_control_code': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.source), indent=2))
