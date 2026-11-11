#!/usr/bin/env python3
"""Bake shared-texture Rocketbox LODs with Blender; original assets are retained."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from xml.etree import ElementTree as ET


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--blender',type=Path,required=True)
    parser.add_argument('--cache',type=Path,required=True)
    parser.add_argument('--ratio',type=float,default=.5)
    parser.add_argument('--models',nargs='*')
    args=parser.parse_args()
    base=Path(__file__).resolve().parent
    for directory in sorted((base/'assets/realistic/rocketbox').iterdir()):
        if not directory.is_dir() or not (directory/'walk.npz').exists():continue
        if args.models and directory.name not in args.models:continue
        output=directory/'lod1';output.mkdir(exist_ok=True)
        textures=[*directory.glob('*.jpg'),*directory.glob('*.png')]
        for texture in textures:
            target=output/texture.name
            if not target.exists():target.symlink_to(Path('..')/texture.name)
        subprocess.run([str(args.blender.resolve()),'-b','--python-exit-code','1','--threads','2','--python',str(base/'prepare_rocketbox_human.py'),
                        '--',str(args.cache.resolve()),directory.name,str(output),'--lod-ratio',str(args.ratio)],check=True)
        specs=json.loads((output/'materials.json').read_text())
        for spec in specs:
            for key in ('image','normal_image','orm_image','opacity_image'):
                if key in spec:spec[key]='../'+spec[key]
        (output/'materials.json').write_text(json.dumps(specs,indent=2)+'\n')
        ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        ET.register_namespace('',ns['c'])
        for clip in ('walk','idle'):
            path=output/(clip+'.dae');tree=ET.parse(path)
            for image in tree.findall('.//c:library_images/c:image/c:init_from',ns):
                if Path(image.text).name in {t.name for t in textures}:image.text='../'+Path(image.text).name
            tree.write(path,encoding='unicode',xml_declaration=True)
        for texture in textures:(output/texture.name).unlink()
        receipt=json.loads((output/'conversion.json').read_text())
        receipt.update(texture_source='..',source_receipt_sha256=hashlib.sha256((directory/'conversion.json').read_bytes()).hexdigest())
        receipt['sha256']={name:hashlib.sha256((output/name).read_bytes()).hexdigest() for name in receipt['sha256']}
        (output/'conversion.json').write_text(json.dumps(receipt,indent=2)+'\n')


if __name__=='__main__':main()
