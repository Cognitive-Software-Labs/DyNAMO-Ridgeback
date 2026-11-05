#!/usr/bin/env python3
"""Preserve F2 source, dependency versions and model hashes for result review."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import tarfile


def digest(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    workspace=Path.cwd()
    (a.output/'base-revision.txt').write_text(subprocess.check_output(['git','rev-parse','HEAD'],text=True))
    (a.output/'working-tree.patch').write_bytes(subprocess.check_output(['git','diff','--binary']))
    with tarfile.open(a.output/'source.tar.gz','w:gz') as archive:
        for name in ('src/ridgeback_leader_follow','src/leader_follow_interfaces','docs/leader_following','docs/plans/FOLLOWING_leader_in_crowds.md'):
            archive.add(workspace/name,arcname=name,filter=lambda info:None if '__pycache__' in info.name or info.name.endswith('.pyc') else info)
    environment={}
    script='import importlib.metadata as m,json,platform,sys;print(json.dumps({"python":sys.version,"platform":platform.platform(),"packages":sorted({d.metadata["Name"]:d.version for d in m.distributions() if d.metadata["Name"]}.items())}))'
    for name,python in [('vision',workspace/'perception_venv/bin/python'),('tracking',a.root/'f2-venv/bin/python')]:
        environment[name]=json.loads(subprocess.check_output([str(python),'-c',script],text=True))
    files=list((a.root/'f2-models').rglob('*'))
    files+=[a.root/'f2-vendor/petra/petra/model/model_bootstrapping.pb',a.root/'f2-vendor/leg/src/leg_detector/config/trained_leg_detector_res=0.33.yaml']
    hashes={};cache={}
    for path in files:
        if not path.is_file() or path.suffix not in ('.pt','.pth','.onnx','.safetensors','.bin','.pb','.yaml','.json'):continue
        actual=path.resolve()
        if actual not in cache:cache[actual]=digest(path)
        hashes[str(path.relative_to(a.root))]=cache[actual]
    native=a.root/'f2-leg-build/leg_cli'
    if native.exists():hashes[str(native.relative_to(a.root))]=digest(native)
    payload={'base_revision':(a.output/'base-revision.txt').read_text().strip(),
             'source_archive_sha256':digest(a.output/'source.tar.gz'),'patch_sha256':digest(a.output/'working-tree.patch'),
             'environments':environment,'model_and_binary_sha256':hashes,
             'provenance':'partial: campaign included utility changes; per-detector and scene source copies plus this final snapshot are retained',
             'host':platform.platform(),'hardware_performance':'robot/Thor deferred; workstation inference only'}
    (a.output/'manifest.json').write_text(json.dumps(payload,indent=2)+'\n')


if __name__ == '__main__':main()
