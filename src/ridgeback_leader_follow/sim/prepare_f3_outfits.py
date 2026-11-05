#!/usr/bin/env python3
"""python3 prepare_f3_outfits.py <maya-dir> <maya_f3-dir>: recolour F3 outfit textures.

Each garment keeps the source texture's relative shading (weave, folds) and
takes its albedo from crowd_appearance.OUTFITS. Recolouring is done in linear
light; alpha is copied unchanged. Brown hair is the unmodified source texture.
Run prepare_f3_gazebo_human.py with Blender afterwards; it records its own receipt.
"""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ridgeback_leader_follow.crowd_appearance import OUTFITS  # noqa: E402
from ridgeback_leader_follow.f3_scene import (  # noqa: E402
    HAIR_ALBEDO, SOURCE_IMAGES, hair_directory, outfit_directory)


def to_linear(c):
    return np.where(c <= .04045, c / 12.92, ((c + .055) / 1.055) ** 2.4)


def to_srgb(c):
    c = np.clip(c, 0., 1.)
    return np.where(c <= .0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - .055)


def recolour(source, albedo, output):
    image = Image.open(source)
    pixels = np.asarray(image.convert('RGBA'), dtype=np.float64) / 255.
    linear = to_linear(pixels[..., :3])
    luminance = linear @ np.array([.2126, .7152, .0722])
    covered = pixels[..., 3] >= .5
    relative = luminance / max(float(luminance[covered].mean()), 1e-6)
    rgb = to_srgb(to_linear(np.asarray(albedo, dtype=np.float64)) * relative[..., None])
    result = np.concatenate([rgb, pixels[..., 3:]], axis=-1) if image.mode == 'RGBA' else rgb
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.round(result * 255).astype(np.uint8)).save(output, optimize=True)


def main():
    source, output = (Path(a).resolve() for a in sys.argv[1:3])
    written = []
    for outfit in OUTFITS:
        for garment in ('top', 'trousers'):
            path = outfit_directory(output, outfit) / f'{garment}.png'
            recolour(source / SOURCE_IMAGES[garment], getattr(outfit, garment), path)
            written.append(path)
    for hair, albedo in HAIR_ALBEDO.items():
        if albedo is None:
            continue
        for part in ('hair_roots', 'ponytail'):
            path = hair_directory(output, hair) / f'{part}.png'
            recolour(source / SOURCE_IMAGES[part], albedo, path)
            written.append(path)
    digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()  # noqa: E731
    receipt = {
        'tool': 'prepare_f3_outfits.py (numpy, Pillow)',
        'method': 'linear-light albedo times source relative luminance; alpha unchanged',
        'source_sha256': {name: digest(source / name) for name in sorted(set(SOURCE_IMAGES.values()))},
        'outfits': {o.name: {'top': o.top, 'trousers': o.trousers, 'hair': o.hair} for o in OUTFITS},
        'hair_albedo': HAIR_ALBEDO,
        'sha256': {str(p.relative_to(output)): digest(p) for p in written},
    }
    (output / 'outfits.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(f'Wrote {len(written)} F3 textures')


if __name__ == '__main__':
    main()
