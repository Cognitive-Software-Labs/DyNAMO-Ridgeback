#!/usr/bin/env python3
"""Render docs/exploration/assets/exploration.gif from the explainer page's simulation.

Runs the page's own ``// sim:start`` .. ``// sim:end`` JavaScript block in Node,
so the GIF follows the page, then draws the frames with Pillow. Needs ``node``
and Pillow. Rerun whenever that block changes.
"""
import json
import math
from pathlib import Path
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
PAGE = ROOT / 'docs/exploration/assets/explainer.html'
OUT = ROOT / 'docs/exploration/assets/exploration.gif'
EXPORT = """
const out = []; reset();
const snap = () => { const {clusters} = frontiers(); const fr = []; clusters.forEach(cl => cl.forEach(k => fr.push(k)));
  const k = map.reduce((a, v) => a + (v === FREE ? 1 : 0), 0);
  out.push({map: Array.from(map).join(''), robot, heading, path: path.slice(), goal, fr, seen: !!seen, done,
            cov: Math.min(100, Math.round(k / totalFree * 100)), target: TARGET,
            targetKnown: map[idx(Math.floor(TARGET[0]), Math.floor(TARGET[1]))] !== UNK}); };
let n = 0; snap(); while (!done && n < 5000) { step(); n++; if (n % 3 === 0) snap(); } snap();
process.stdout.write(JSON.stringify({W, H, frames: out}));
"""


def simulate():
    html = PAGE.read_text()
    block = html[html.index('// sim:start'):html.index('// sim:end')]
    block = block.replace("const cv = document.getElementById('map'), ctx = cv.getContext('2d');", '')
    with tempfile.NamedTemporaryFile('w', suffix='.js', delete=False) as script:
        script.write(block + EXPORT)
    return json.loads(subprocess.run(['node', script.name], capture_output=True, text=True, check=True).stdout)


def render(data):
    W, H, C, BAR = data['W'], data['H'], 10, 34
    UNK, FREE, WALL = (201, 209, 217), (251, 252, 253), (38, 52, 67)
    FR, ROBOT, TARGET, OK, INK = (209, 54, 109), (33, 82, 196), (201, 138, 0), (21, 122, 61), (20, 34, 48)
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 14)
    frames = []
    for f in data['frames']:
        im = Image.new('RGB', (W * C, H * C + BAR), (238, 241, 244))
        dr = ImageDraw.Draw(im)
        fr = set(f['fr'])
        for i, ch in enumerate(f['map']):
            x, y = i % W, i // W
            col = WALL if ch == '2' else (FR if i in fr else FREE) if ch == '1' else UNK
            dr.rectangle([x * C, y * C, x * C + C - 1, y * C + C - 1], fill=col)
        if f['path'] and not f['done']:
            dr.line([(f['robot'][0] * C, f['robot'][1] * C)] + [(x * C, y * C) for x, y in f['path']], fill=ROBOT, width=2)
        if f['goal'] is not None and not f['done']:
            gx, gy = (f['goal'] % W + .5) * C, (f['goal'] // W + .5) * C
            dr.ellipse([gx - 7, gy - 7, gx + 7, gy + 7], outline=FR, width=3)
        if f['targetKnown'] or f['seen']:
            tx, ty = f['target'][0] * C, f['target'][1] * C
            dr.ellipse([tx - 6, ty - 6, tx + 6, ty + 6], fill=TARGET)
            if f['seen']:
                dr.rectangle([tx - 11, ty - 13, tx + 11, ty + 13], outline=TARGET, width=2)
        rx, ry, h = f['robot'][0] * C, f['robot'][1] * C, f['heading']
        cone = [(rx, ry)] + [(rx + 120 * math.cos(h + a), ry + 120 * math.sin(h + a)) for a in [i / 10 * 1.4 - .7 for i in range(11)]]
        overlay = Image.new('RGBA', im.size, (0, 0, 0, 0))
        ImageDraw.Draw(overlay).polygon(cone, fill=ROBOT + (30,))
        im.paste(overlay, (0, 0), overlay)
        body = [(-9, -8), (9, -8), (9, 8), (-9, 8)]
        dr.polygon([(rx + x * math.cos(h) - y * math.sin(h), ry + x * math.sin(h) + y * math.cos(h)) for x, y in body], fill=ROBOT)
        y0 = H * C + 9
        dr.text((12, y0), 'EXPLORATION COMPLETE' if f['done'] else 'EXPLORING', fill=OK if f['done'] else ROBOT, font=font)
        dr.text((230, y0), f"explored {f['cov']:3d} %", fill=INK, font=font)
        dr.text((400, y0), 'target found' if f['seen'] else 'searching for target', fill=TARGET if f['seen'] else INK, font=font)
        frames.append(im.convert('P', palette=Image.ADAPTIVE, colors=32))
    frames += [frames[-1]] * 25  # hold the finished map
    frames[0].save(OUT, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=True)
    print(f'wrote {OUT.relative_to(ROOT)}: {len(frames)} frames')


if __name__ == '__main__':
    render(simulate())
