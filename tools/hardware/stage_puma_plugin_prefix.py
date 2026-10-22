#!/usr/bin/env python3
"""Stage a Puma-only plugin overlay from the verified offline feedback candidate."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
APT = Path('/opt/ros/jazzy')
EXPECTED = '519cce4380c74aa7346ff8f419faa6a7c84ff9443bba5a1e763d5032343dd157'


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def stage(candidate, output):
    candidate, output = candidate.resolve(), output.resolve()
    if not candidate.is_relative_to(ROOT / 'artifacts/hardware') or not output.is_relative_to(ROOT / 'artifacts/hardware'):
        raise ValueError('use artifacts/hardware paths')
    if output.exists(): raise ValueError('fresh output directory required')
    source = candidate / 'install/clearpath_hardware_interfaces'
    record = json.loads((candidate / 'SOURCE.json').read_text())
    library = source / 'lib/libpuma_hardware.so'
    rel = str(library.relative_to(candidate))
    if (record['status'] != 'built-offline-not-installed' or (candidate / 'DO_NOT_INSTALL.json').exists() or
            record['libraries'].get(rel) != EXPECTED or digest(library) != EXPECTED):
        raise ValueError('candidate is not the verified feedback build')
    if digest(ROOT / 'patches/clearpath_puma_feedback_consumer.patch') != record['patch_sha256'] or digest(ROOT / 'tools/hardware/puma_feedback_safety.hpp') != record['policy_sha256']:
        raise ValueError('maintained candidate inputs changed')
    package = 'clearpath_hardware_interfaces'
    index = f'share/ament_index/resource_index/hardware_interface__pluginlib__plugin/{package}'
    required = ['lib/libpuma_hardware.so', index,
                f'share/ament_index/resource_index/packages/{package}',
                f'share/{package}/package.xml']
    required += (source / index).read_text().splitlines()
    prefix = output / 'prefix'
    for rel in required:
        inp = source / rel
        if not inp.is_file() or inp.is_symlink(): raise ValueError('missing/linked package asset: ' + rel)
        dest = prefix / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(inp, dest)
    companions = {}
    for rel in (source / index).read_text().splitlines():
        if '/puma/' in rel: continue
        name = ET.parse(prefix / rel).getroot().attrib['path']
        apt_lib = APT / 'lib' / ('lib' + name + '.so')
        if not apt_lib.is_file() or apt_lib.is_symlink(): raise ValueError('missing packaged companion: ' + name)
        (prefix / 'lib' / apt_lib.name).symlink_to(apt_lib)
        companions[apt_lib.name] = digest(apt_lib)
    # The platform launch resolves executables through this same package prefix.
    # Keep those six packaged programs available without replacing their bytes.
    apt_libexec = APT / 'lib' / package
    expected_programs = {'battery_state_control', 'battery_state_estimator',
                         'fan_control_node', 'lighting_node', 'pinout_control_node',
                         'sw_low_soc_cutoff_node'}
    if {p.name for p in apt_libexec.iterdir()} != expected_programs:
        raise ValueError('Clearpath package executables changed; review overlay')
    staged_libexec = prefix / 'lib' / package
    staged_libexec.mkdir()
    for name in sorted(expected_programs):
        program = apt_libexec / name
        if not program.is_file() or program.is_symlink():
            raise ValueError('missing packaged executable: ' + name)
        (staged_libexec / name).symlink_to(program)
        companions[f'{package}/{name}'] = digest(program)
    result = {'status': 'staged-plugin-only-not-installed', 'source_candidate': str(candidate),
              'candidate_library_sha256': EXPECTED,
              'files': {rel: digest(prefix / rel) for rel in required},
              'companion_apt_library_hashes': companions,
              'limits': 'Fake-wheel plugin selection must pass; no live deployment or physical braking qualification.'}
    (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(stage(args.candidate, args.output), indent=2))
