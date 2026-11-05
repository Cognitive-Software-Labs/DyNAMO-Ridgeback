"""Download CC0 scene assets from their publishers; preserve URLs and hashes.

Run with system Python. Poly Haven files are 1K to keep the simulator responsive.
The static assets are powered by Poly Haven (https://polyhaven.com).
"""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys
import requests

MODELS = ('modern_arm_chair_01','sofa_03','modern_coffee_table_01','potted_plant_02',
          'metal_office_desk','wooden_crate_01','barrel_03','plastic_crate_01',
          'painted_wooden_bench','street_lamp_01','tree_small_02',
          'binder_notebook','book_encyclopedia_set_01','metal_trash_can')
SURFACES = ('wood_floor','concrete_floor_02','brick_pavement_02','brick_wall_001','grass_ground')
HUMANS = {
    'sam':'relaxed-walk-sam-3d-person-animated-732f1206',
    'jordan':'relaxed-walk-jordan-3d-person-animated-7ab9479d',
    'harper':'relaxed-walk-harper-3d-person-animated-b504cbac',
}
HEADERS={'User-Agent':'DyNAMO-Ridgeback-academic-simulation/1.0 (Powered by Poly Haven)'}


def fetch(url,path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if not path.exists():
        response=requests.get(url,headers=HEADERS,timeout=90);response.raise_for_status()
        path.write_bytes(response.content)
    return {'url':url,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'bytes':path.stat().st_size}


def main():
    output=Path(sys.argv[1]);output.mkdir(parents=True,exist_ok=True)
    receipt={'license':'CC0-1.0','publishers':['https://polyhaven.com/license','https://www.innerscene.com/tools/library'],
             'credit':'Powered by Poly Haven','files':{}}
    def model(key):
        meta=requests.get('https://api.polyhaven.com/files/'+key,headers=HEADERS,timeout=45);meta.raise_for_status()
        spec=meta.json()['gltf']['1k']['gltf'];base=output/'props'/key
        files={str((base/(key+'.gltf')).relative_to(output)):fetch(spec['url'],base/(key+'.gltf'))}
        for relative,data in spec['include'].items():
            files[str((base/relative).relative_to(output))]=fetch(data['url'],base/relative)
        return files
    with ThreadPoolExecutor(max_workers=5) as pool:
        for files in pool.map(model,MODELS):receipt['files'].update(files)
    for key in SURFACES:
        response=requests.get('https://api.polyhaven.com/files/'+key,headers=HEADERS,timeout=45);response.raise_for_status();spec=response.json()
        for channel in ('Diffuse','Rough','nor_gl'):
            item=spec[channel]['1k']['jpg'];path=output/'surfaces'/key/(channel+'.jpg')
            receipt['files'][str(path.relative_to(output))]=fetch(item['url'],path)
    key='kloofendal_48d_partly_cloudy_puresky'
    response=requests.get('https://api.polyhaven.com/files/'+key,headers=HEADERS,timeout=45);response.raise_for_status()
    item=response.json()['hdri']['1k']['hdr'];path=output/'daylight.hdr'
    receipt['files'][str(path.relative_to(output))]=fetch(item['url'],path)
    for name,slug in HUMANS.items():
        url='https://www.innerscene.com/api/library/'+slug+'/download';path=output/'humans'/name/'source.glb'
        receipt['files'][str(path.relative_to(output))]=fetch(url,path)
        receipt['files'][str(path.relative_to(output))]['source_page']='https://www.innerscene.com/tools/library/3d-parts/'+slug
    (output/'sources.json').write_text(json.dumps(receipt,indent=2)+'\n')
    print('Downloaded',len(receipt['files']),'licensed source files',flush=True)


if __name__=='__main__':main()
