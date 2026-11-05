#!/usr/bin/env python3
"""Prepare pinned third-party sources and benchmark checkpoints under artifacts."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import urllib.request


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('artifacts/leader-following'))
    parser.add_argument('--python',type=Path,default=Path('perception_venv/bin/python'))
    a=parser.parse_args();source=Path(__file__).resolve().parent
    if not (source/'candidate-sources.json').exists():
        from ament_index_python.packages import get_package_share_directory
        source=Path(get_package_share_directory('ridgeback_leader_follow'))/'benchmarking'
    vendor=a.root/'f2-vendor';models=a.root/'f2-models';vendor.mkdir(parents=True,exist_ok=True);models.mkdir(parents=True,exist_ok=True)
    pins=json.loads((source/'candidate-sources.json').read_text())
    for name,pin in pins.items():
        repo=vendor/name
        if not repo.exists():
            subprocess.run(['git','clone',pin['url'],str(repo)],check=True)
            subprocess.run(['git','-C',str(repo),'checkout','--detach',pin['revision']],check=True)
        revision=subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()
        if revision != pin['revision']:raise ValueError(f'Unexpected dependency revision: {repo}')
        patch=source/'patches'/(name+'.patch')
        if patch.exists():
            applied=subprocess.run(['git','-C',str(repo),'apply','--reverse','--check',str(patch.resolve())],capture_output=True)
            if applied.returncode:
                subprocess.run(['git','-C',str(repo),'apply','--check',str(patch.resolve())],check=True)
                subprocess.run(['git','-C',str(repo),'apply',str(patch.resolve())],check=True)
    subprocess.run([str(a.python),'-m','pip','install','--no-deps',str(vendor/'drow/dr_spaam')],check=True)
    if not (models/'drow/self_supervised_person_detection/ckpt_jrdb_ann_drow3_e40.pth').exists():
        subprocess.run([str(a.python),'-m','gdown','--folder','https://drive.google.com/drive/folders/1Wl2nC8lJ6s9NI1xtWwmxeAUnuxDiiM4W','-O',str(models/'drow')],check=True)
    if not (models/'osnet_x0_25_msmt17.pt').exists():
        subprocess.run([str(a.python),'-m','gdown','https://drive.google.com/uc?id=1sSwXSUlj4_tHZequ_iZ8w_Jh0VaRQMqF','-O',str(models/'osnet_x0_25_msmt17.pt')],check=True)
    if not (models/'deim-small.pth').exists():
        subprocess.run([str(a.python),'-m','gdown','https://drive.google.com/uc?id=1tB8gVJNrfb6dhFvoHJECKOF5VpkthhfC','-O',str(models/'deim-small.pth')],check=True)
    if not (models/'LFE-PPN.onnx').exists():
        shutil.copyfile(vendor/'lfe/upo_laser_people_detector/models/LFE-PPN.onnx',models/'LFE-PPN.onnx')
    if not (models/'sam2.1_hiera_tiny.pt').exists():
        urllib.request.urlretrieve('https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt',models/'sam2.1_hiera_tiny.pt')
    subprocess.run([str(a.python),'-m','pip','install','--no-deps','--no-build-isolation',str(vendor/'samurai/sam2')],
                   env={**__import__('os').environ,'SAM2_BUILD_CUDA':'0'},check=True)
    native=source.parent/'benchmark_native' if (source.parent/'benchmark_native').exists() else source.parents[1]/'benchmark_native'
    subprocess.run(['cmake','-S',str(native),'-B',str(a.root/'f2-leg-build'),'-DLEG_SOURCE='+str((vendor/'leg/src/leg_detector').resolve())],check=True)
    subprocess.run(['cmake','--build',str(a.root/'f2-leg-build'),'-j4'],check=True)


if __name__ == '__main__':main()
