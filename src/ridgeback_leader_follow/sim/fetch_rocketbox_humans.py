"""Download the pinned Microsoft Rocketbox (MIT) avatars and clips; convert textures.

Run with system Python (requests, Pillow):
  python3 fetch_rocketbox_humans.py <source-cache> <assets/realistic/rocketbox>

The cache keeps the publisher's FBX/TGA bytes (large; not committed). The output
directory receives web-sized textures and `sources.json` with URLs and SHA-256
for every source and derived file. Blender conversion is a separate step.
"""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import sys

from PIL import Image
import requests

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ridgeback_leader_follow.rocketbox_cast import COMMIT, IDLE, REPOSITORY, ROSTER, WALK, raw_url  # noqa: E402

# Colour maps keep facial and fabric detail; normal maps are smaller. Specular and
# wrinkle maps are not used by either renderer and are not converted.
SIZES = {'color': 2048, 'normal': 1024, 'opacity': 2048}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fetch(relative, cache):
    path = cache / relative
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        response = requests.get(raw_url(relative), timeout=120)
        response.raise_for_status()
        path.write_bytes(response.content)
    return relative, {'url': raw_url(relative), 'sha256': sha(path), 'bytes': path.stat().st_size}


def tree():
    response = requests.get(f'https://api.github.com/repos/{REPOSITORY}/git/trees/{COMMIT}?recursive=1', timeout=60)
    response.raise_for_status()
    return [item['path'] for item in response.json()['tree'] if item['type'] == 'blob']


def convert(source, target):
    kind = 'opacity' if '_opacity_' in source.name else 'normal' if '_normal' in source.name else 'color'
    image = Image.open(source)
    image = image.convert('RGBA' if kind == 'opacity' else 'RGB')
    size = SIZES[kind]
    if max(image.size) > size:
        image = image.resize((size, size), Image.LANCZOS)
    target.parent.mkdir(parents=True, exist_ok=True)
    if kind == 'opacity':
        image.save(target.with_suffix('.png'), optimize=True)
        return target.with_suffix('.png')
    image.save(target.with_suffix('.jpg'), quality=92, optimize=True)
    return target.with_suffix('.jpg')


def main():
    cache, output = (Path(a).resolve() for a in sys.argv[1:3])
    paths = tree()
    wanted = ['LICENSE.md']
    wanted += [f'Assets/Animations/{clip}.max.fbx' for clip in (*WALK.values(), *IDLE.values())]
    for name, (avatar, _, _) in ROSTER.items():
        prefix = f'Assets/Avatars/{avatar}/'
        wanted.append(f'{prefix}Export/{avatar.split("/")[-1]}.fbx')
        wanted += [p for p in paths if p.startswith(prefix + 'Textures/') and p.endswith('.tga')]
    missing = [p for p in wanted if p not in paths]
    if missing:
        raise RuntimeError(f'Pinned Rocketbox commit lacks {missing}')
    with ThreadPoolExecutor(max_workers=8) as pool:
        sources = dict(pool.map(lambda p: fetch(p, cache), wanted))
    derived = {}
    for name, (avatar, _, _) in ROSTER.items():
        for relative in [p for p in sources if p.startswith(f'Assets/Avatars/{avatar}/Textures/')
                         and not any(word in p for word in ('_specular', '_wrinkle'))]:
            target = convert(cache / relative, output / name / Path(relative).stem)
            derived[str(target.relative_to(output))] = {'source': relative, 'sha256': sha(target)}
    receipt = {'license': 'MIT', 'copyright': 'Copyright (c) 2020 Microsoft',
               'repository': f'https://github.com/{REPOSITORY}', 'commit': COMMIT,
               'texture_sizes': SIZES, 'sources': sources, 'derived_textures': derived}
    output.mkdir(parents=True, exist_ok=True)
    (output / 'LICENSE.md').write_bytes((cache / 'LICENSE.md').read_bytes())
    (output / 'sources.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print('Fetched', len(sources), 'Rocketbox source files;', len(derived), 'textures converted', flush=True)


if __name__ == '__main__':
    main()
