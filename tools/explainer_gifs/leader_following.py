#!/usr/bin/env python3
"""Render docs/leader_following/assets/leader_following.gif from the explainer page's simulation.

Runs the page's own ``// sim:start`` .. ``// sim:end`` JavaScript block in Node,
so the GIF follows the page, then draws every third frame with Pillow. Needs
``node`` and Pillow. Rerun whenever that block changes.
"""
import json
import math
from pathlib import Path
import subprocess
import tempfile

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[2]
PAGE = ROOT / 'docs/leader_following/assets/explainer.html'
OUT = ROOT / 'docs/leader_following/assets/leader_following.gif'
EXPORT = """
process.stdout.write(JSON.stringify({alike: ALIKE, trail: Array.from({length: 49}, (_, k) => leaderAt(k / 2)),
                                     frames: frames.filter((f, i) => i % 3 === 0)}));
"""


def simulate():
    html = PAGE.read_text()
    block = html[html.index('// sim:start'):html.index('// sim:end')]
    with tempfile.NamedTemporaryFile('w', suffix='.js', delete=False) as script:
        script.write('const rad = d => d * Math.PI / 180;\n' + block + EXPORT)
    return json.loads(subprocess.run(['node', script.name], capture_output=True, text=True, check=True).stdout)


def render(d):
    PX = 640 / 22.5; Wd, Hd = 640, int(9 * PX); BAR = 34
    PANEL, GRID, INK, MUTED = (255, 255, 255), (225, 232, 238), (20, 34, 48), (88, 105, 121)
    ROBOT, SCAN, LEADER, PERSON = (33, 82, 196), (11, 143, 128), (224, 87, 15), (141, 154, 166)
    STATE = {'LOCKED': (21, 122, 61), 'UNCERTAIN': (154, 106, 0), 'LOST': (180, 32, 32)}
    WHY = {'locked': 'following at 1.4 m', 'recovered': 'leader re-confirmed, following',
           'hidden': 'view blocked, holding still', 'lost': 'leader unconfirmed, stopped and searching',
           'confirming': 'leader visible again, checking identity'}
    font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 14)
    P = lambda p: (p[0] * PX, p[1] * PX)

    def fan(c, r, a0, a1, n=40):
        return [c] + [(c[0] + r * math.cos(a0 + (a1 - a0) * i / n), c[1] + r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]

    frames = []
    for f in d['frames']:
        im = Image.new('RGB', (Wd, Hd + BAR), (238, 241, 244)); dr = ImageDraw.Draw(im)
        dr.rectangle([0, 0, Wd, Hd], fill=PANEL)
        for g in range(0, Wd, 28): dr.line([(g, 0), (g, Hd)], fill=GRID)
        for g in range(0, Hd, 28): dr.line([(0, g), (Wd, g)], fill=GRID)
        trail = [P(p) for p in d['trail']]
        for a, b in zip(trail[::2], trail[1::2]): dr.line([a, b], fill=(244, 196, 170), width=3)
        R, h = P(f['R']), f['heading']
        ov = Image.new('RGBA', im.size, (0, 0, 0, 0)); od = ImageDraw.Draw(ov)
        od.ellipse([R[0] - 107, R[1] - 107, R[0] + 107, R[1] + 107], fill=SCAN + (18,))
        od.polygon(fan(R, 185, h - math.radians(30), h + math.radians(30)), fill=ROBOT + (28,))
        im.paste(ov, (0, 0), ov)
        col = STATE[f['state']]
        L = P(f['L'])
        dr.line([R, L], fill=col, width=2)
        for k, p in enumerate(f['P']):
            q = P(p)
            if not (0 <= q[0] <= Wd and 0 <= q[1] <= Hd): continue
            dr.ellipse([q[0] - 7, q[1] - 7, q[0] + 7, q[1] + 7], fill=LEADER if k == d['alike'] else PERSON)
            if k == d['alike']: dr.text((q[0] - 34, q[1] - 22), 'look-alike', fill=MUTED, font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 11))
        dr.ellipse([L[0] - 7, L[1] - 7, L[0] + 7, L[1] + 7], fill=LEADER)
        dr.ellipse([L[0] - 12, L[1] - 12, L[0] + 12, L[1] + 12], outline=col, width=3)
        dr.text((L[0] - 20, L[1] + 14), 'leader', fill=INK, font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 11))
        corners = [(-13, -11), (13, -11), (13, 11), (-13, 11)]
        dr.polygon([(R[0] + x * math.cos(h) - y * math.sin(h), R[1] + x * math.sin(h) + y * math.cos(h)) for x, y in corners], fill=ROBOT)
        y0 = Hd + 9
        dr.text((12, y0), f['state'], fill=col, font=font)
        dr.text((130, y0), WHY[f['why']], fill=INK, font=font)
        frames.append(im.convert('P', palette=Image.ADAPTIVE, colors=48))
    frames[0].save(OUT, save_all=True, append_images=frames[1:], duration=100, loop=0, optimize=True)
    print(f'wrote {OUT.relative_to(ROOT)}: {len(frames)} frames')


if __name__ == '__main__':
    render(simulate())
