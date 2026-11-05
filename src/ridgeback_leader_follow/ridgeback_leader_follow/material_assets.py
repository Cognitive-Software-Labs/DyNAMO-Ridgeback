"""Preserve source PBR channels when rendering the realistic character/prop assets."""
from functools import lru_cache
import json
from pathlib import Path
import struct


@lru_cache(maxsize=32)
def material_specs(directory):
    directory=Path(directory)
    specs=json.loads((directory/'materials.json').read_text())
    binary=(directory/'source.glb').exists()
    if binary:
        with (directory/'source.glb').open('rb') as stream:
            header=stream.read(20)
            if header[:4]!=b'glTF':raise ValueError('GLB source is unavailable; fetch Git LFS assets')
            source=json.loads(stream.read(struct.unpack_from('<I',header,12)[0]))
    else:
        files=[p for p in directory.glob('*.gltf') if not p.name.startswith('.')]
        if not files:return specs
        source=json.loads(files[0].read_text())
    def image(texture):
        index=source['textures'][texture['index']]['source'];spec=source['images'][index]
        return f'image_{index}'+('.png' if spec.get('mimeType')=='image/png' else '.jpg') if binary else spec['uri']
    for spec,original in zip(specs,source['materials'],strict=True):
        normal=original.get('normalTexture')
        if normal:
            spec['normal_image']=image(normal);spec['normal_scale']=normal.get('scale',1.)
        packed=original.get('pbrMetallicRoughness',{}).get('metallicRoughnessTexture')
        if packed:spec['orm_image']=image(packed)
        spec['specular_factor']=original.get('extensions',{}).get('KHR_materials_specular',{}).get('specularFactor',1.)
    masks=json.loads((directory/'opacity_maps.json').read_text()) if (directory/'opacity_maps.json').exists() else {}
    for spec in specs:
        if spec.get('alphaMode') in ('MASK','BLEND') and 'all' in masks:spec['opacity_image']=masks['all']
        for field in ('image','normal_image','orm_image','opacity_image'):
            if field in spec and not (directory/spec[field]).is_file():raise ValueError(f'Missing {field}: {directory/spec[field]}')
    return specs


@lru_cache(maxsize=32)
def texture_tint_scale(image,target):
    """Normalize a dark cloth texture to its assigned colour without replacing its pixels."""
    import numpy as np
    from PIL import Image
    with Image.open(image) as original:
        rgb=np.asarray(original.convert('RGB').resize((128,128)),dtype=np.float32)/255.
    linear=np.where(rgb<=.04045,rgb/12.92,((rgb+.055)/1.055)**2.4)
    median=np.maximum(np.median(linear.reshape(-1,3),axis=0),.025)
    return tuple(float(t/v) for t,v in zip(target,median,strict=True))
