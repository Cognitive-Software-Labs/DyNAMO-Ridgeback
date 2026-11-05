"""Fetch pinned official F3 sources/checkpoints; refuse dirty source overwrites."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

OSNET_REVISION = 'a5c5cc037c24235cda3b21085b93ad77c9616224'
OSNET_FILE = 'osnet_ain_x1_0_msmt17_256x128_amsgrad_ep50_lr0.0015_coslr_b64_fb10_softmax_labsmth_flip_jitter.pth'
DRIVE = {'clip-reid':'1BVaZo93kOksYLjFNH3Gf7JxIbPlWSkcO',
         'transreid':'1crYsKRrW4eUq6abT4KK8_atMLFsbq56W',
         'solider':'1YrE_r9Fk5uR0uFFQboBv203vxlOpFXU8'}


def prepare(models, sources):
    import gdown
    from huggingface_hub import hf_hub_download
    models = models.resolve()
    vendor = models.parent/'f3-vendor'
    models.mkdir(parents=True,exist_ok=True)
    vendor.mkdir(parents=True,exist_ok=True)
    manifest = json.loads((sources/'f3-sources.json').read_text())
    for name, spec in manifest.items():
        target = vendor/name
        if not target.exists():
            subprocess.run(['git','clone',spec['repository'],str(target)],check=True)
            subprocess.run(['git','-C',str(target),'checkout','--detach',spec['revision']],check=True)
        revision = subprocess.check_output(['git','-C',str(target),'rev-parse','HEAD'],text=True).strip()
        if revision != spec['revision']:
            raise ValueError(f'{target} revision differs; preserve existing checkout and resolve explicitly')
        patch = sources/(name+'-torch2.patch')
        if patch.exists():
            already = subprocess.run(['git','-C',str(target),'apply','--reverse','--check',str(patch.resolve())],
                                     stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode == 0
            if not already:
                subprocess.run(['git','-C',str(target),'apply','--check',str(patch.resolve())],check=True)
                subprocess.run(['git','-C',str(target),'apply',str(patch.resolve())],check=True)
    hf_hub_download('kaiyangzhou/osnet',OSNET_FILE,revision=OSNET_REVISION,local_dir=models/'osnet')
    for name, identifier in DRIVE.items():
        target = models/(name+'.pth')
        if not target.exists():
            temporary = target.with_suffix('.download')
            gdown.download(id=identifier,output=str(temporary),quiet=False)
            if not temporary.exists() or temporary.stat().st_size < 1000000:
                raise ValueError(f'Invalid checkpoint download: {name}')
            temporary.rename(target)
    receipt = {'sources':manifest,'osnet_revision':OSNET_REVISION,'drive_ids':DRIVE,
               'checkpoints':{str(p.relative_to(models)):hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in models.rglob('*.pth')}}
    (models/'preparation.json').write_text(json.dumps(receipt,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir',type=Path,default=Path('artifacts/leader-following/f3-models'))
    parser.add_argument('--sources',type=Path)
    args=parser.parse_args()
    if args.sources is None:
        from ament_index_python.packages import get_package_share_directory
        args.sources=Path(get_package_share_directory('ridgeback_leader_follow'))/'benchmark_native'
    prepare(args.model_dir,args.sources)


if __name__ == '__main__':
    main()
