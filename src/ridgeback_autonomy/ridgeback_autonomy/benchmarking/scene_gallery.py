"""Render a scene_audit run as a self-contained HTML scenario gallery.

One card per scene, grouped by family, showing the audit's camera and
top-down preview at the certified profile, the scenario comment as caption and
the audited numbers underneath. Every number comes from the same
``audit.json`` the certification check reads, so the gallery cannot drift from
the scenario file. Pure; no ROS.
"""

from __future__ import annotations

import base64
from collections.abc import Iterable, Mapping
import html
from pathlib import Path

from ridgeback_autonomy.benchmarking.scene_certifications import (
    is_in_box_scene,
    PRIMARY_VIS,
    scene_comments,
)


# (prefix or predicate key, title, blurb); first match wins.
FAMILIES = (
    ('single_', 'Single target', 'One G1, no objects, stratified by yaw: facing, side-on and back.'),
    ('multi_', 'Multiple targets', 'Two or three G1s, all fully visible.'),
    ('twin_', 'Twin targets', 'Adjacent G1s whose masks can merge (shoulder) or swap (bearing).'),
    ('interocc_', 'Robot-on-robot occlusion',
     'A near G1 partly hides a far one. The far G1 shows through the near '
     "one's limb gaps, so these read above their original bands."),
    ('objocc_', 'Heavy object occlusion', 'An object hides most of the target (vis 0.02-0.20).'),
    ('objpartial_', 'Partial object occlusion', 'An object hides part of the target (vis 0.35-0.65).'),
    ('near_clip_', 'Near limit', 'Targets close enough that the vertical FOV clips the feet.'),
    ('far_', 'Far range', 'Small targets near and beyond the 10 m depth clamp.'),
    ('edge640_', 'Image edge (640x480)',
     'Targets touching, cut by, or barely inside the side edge of the 640x480 frame. '
     'At 1280x720 the wider FOV brings them back into frame.'),
    ('trap_', 'Traps', 'Object placements that attack one stage: masks, scan plane, bbox, depth band.'),
    ('chaos_', 'Chaos', 'Three G1s with mixed objects.'),
    ('in_box', 'Clutter inside the box',
     'IV poles inside the detection box, where the mask admits their beams and range '
     'segmentation must reject them.'),
    ('interfere_', 'Clutter clear of the box', 'Objects near the target but outside its detection box.'),
)


def family_of(scene_id: str) -> str:
    if is_in_box_scene(scene_id):
        return 'in_box'
    return next(key for key, *_ in FAMILIES if key != 'in_box' and scene_id.startswith(key))


def _badges(scene_id: str, comment: str, scene: Mapping, repeats: int | None) -> list[tuple[str, str]]:
    badges = []
    if repeats:
        badges.append(('probe', f'probe x{repeats}'))
    if scene['targets'] > 1:
        badges.append(('', f'{scene["targets"]} targets'))
    if ' the ' in comment and (', above the ' in comment or ', below the ' in comment):
        badges.append(('warn', 'outside band'))
    if 'outside the box' in comment and not scene_id.startswith('interfere_control'):
        badges.append(('warn', 'pole at box edge'))
    if 'intersects the G1' in comment:
        badges.append(('warn', 'pole touches G1'))
    if scene_id.startswith('edge640_'):
        badges.append(('edge', '640x480 only'))
    return badges


def _metrics(scene: Mapping, profile: str) -> str:
    cells = []
    for target in scene['camera'][profile]['targets']:
        vis = target[PRIMARY_VIS]
        cells.append(
            f'<span><i>T{target["index"]}</i> vis <b>{"-" if vis is None else f"{vis:.2f}"}</b>'
            f' · in-frame <b>{target["in_frame_fraction"]:.2f}</b>'
            f' · {target["range_horizontal_m"]:.1f} m</span>')
    beams = [t['beams_total'] for t in scene['lidar']['targets']]
    cells.append(f'<span><i>LiDAR</i> beams on targets <b>{", ".join(map(str, beams))}</b></span>')
    return ''.join(cells)


def render_gallery(
    *,
    audit: Mapping,
    scenario_text: str,
    repeats: Mapping[str, int | None],
    image_dir: Path,
    profile: str,
    scenario_name: str,
) -> str:
    """Return the gallery HTML for one audit run and its scenario file."""

    comments = scene_comments(scenario_text)
    scenes = list(audit['scenes'])
    groups: dict[str, list[Mapping]] = {key: [] for key, *_ in FAMILIES}
    for scene in scenes:
        groups[family_of(scene['id'])].append(scene)

    geometry = audit['sensor_geometry']
    camera = next(i for i in geometry['intrinsics'] if i['name'] == profile)
    optical = geometry['camera_optical']['xyz_in_base_link']
    sections, nav = [], []
    for key, title, blurb in FAMILIES:
        members = groups[key]
        if not members:
            continue
        anchor = key.rstrip('_')
        nav.append(f'<a href="#{anchor}">{html.escape(title)} <span>{len(members)}</span></a>')
        cards = []
        for scene in members:
            sid = scene['id']
            comment = comments.get(sid, '')
            badges = _badges(sid, comment, scene, repeats.get(sid))
            tags = ' '.join(kind for kind, _ in badges if kind)
            badge_html = ''.join(
                f'<span class="badge {kind}">{html.escape(text)}</span>' for kind, text in badges)
            data = base64.b64encode((image_dir / f'{sid}.png').read_bytes()).decode('ascii')
            cards.append(
                f'<figure class="card" data-tags="{tags}">'
                f'<img loading="lazy" alt="{html.escape(sid)}: camera view and top-down view" '
                f'src="data:image/png;base64,{data}">'
                f'<figcaption><div class="cap-head"><code>{html.escape(sid)}</code>{badge_html}</div>'
                f'<p class="note">{html.escape(comment) or "&nbsp;"}</p>'
                f'<div class="metrics">{_metrics(scene, profile)}</div></figcaption></figure>')
        sections.append(
            f'<section id="{anchor}"><div class="fam-head"><h3>{html.escape(title)}</h3>'
            f'<span class="fam-count">{len(members)} scenes</span></div>'
            f'<p class="blurb">{html.escape(blurb)}</p><div class="grid">{"".join(cards)}</div></section>')

    instances = sum(scene['targets'] for scene in scenes)
    trials = sum(repeats.get(scene['id']) or 1 for scene in scenes)
    probes = sum(1 for scene in scenes if repeats.get(scene['id']))
    method = (
        f'Audited by <code>scene_audit</code> on {html.escape(str(audit["created"]))} for '
        f'scenario sha256 <code>{html.escape(audit["scenario"]["sha256"][:12])}</code>: '
        f'ray-cast id buffer of the Gazebo G1 mesh and objects standing on the floor, colour '
        f'optical frame at base_link + ({optical[0]:.4f}, {optical[1]:.4f}, {optical[2]:.4f}), '
        f'fx = {camera["fx"]:g} at {html.escape(profile)}. Visible fraction counts the target '
        f'pixels other geometry leaves visible; in-frame counts the pixels inside the image.')
    return _PAGE.format(
        scenario=html.escape(scenario_name), method=method, scenes=len(scenes),
        instances=instances, trials=trials, probes=probes, nav=''.join(nav),
        sections=''.join(sections), profile=html.escape(profile))


def repeats_by_scene(scenes: Iterable) -> dict[str, int | None]:
    return {scene.id: scene.repeats_override for scene in scenes}


_PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Benchmark scene gallery</title>
<style>
  :root {{
    --ink: #1d2530; --body: #46525f; --faint: #7d8794;
    --paper: #f6f8fb; --card: #ffffff; --line: #dfe4ea;
    --accent: #3b5b8c; --accent-bg: #eef2f8; --glow: rgba(59, 91, 140, .22);
    --warn: #b3512f; --warn-bg: #fdf1ea; --edge: #6b4fa3; --edge-bg: #f3effa;
    --mono: ui-monospace, "SF Mono", "Cascadia Code", Menlo, Consolas, monospace;
  }}
  @media (prefers-color-scheme: dark) {{
    :root {{
      --ink: #e8edf3; --body: #b6c0cc; --faint: #8592a1;
      --paper: #12161c; --card: #1a2028; --line: #2c3540;
      --accent: #8fb0e6; --accent-bg: #1f2a3a; --glow: rgba(143, 176, 230, .25);
      --warn: #f0a07c; --warn-bg: #36231b; --edge: #c3a8f0; --edge-bg: #2a2238;
    }}
  }}
  * {{ box-sizing: border-box; }}
  body {{ margin: 0; background: var(--paper); color: var(--body);
         font: 15px/1.55 -apple-system, "Segoe UI", Roboto, "Helvetica Neue", sans-serif; }}
  .shell {{ max-width: 1280px; margin: 0 auto; padding: 0 16px; }}
  header {{ padding: 2.2rem 0 1.6rem; background: var(--card); border-bottom: 1px solid var(--line); }}
  h1 {{ color: var(--ink); font-size: 1.6rem; margin: 0 0 .4rem; letter-spacing: -.01em; }}
  .sub {{ max-width: 72ch; margin: 0; }}
  code {{ font-family: var(--mono); font-size: .86em; }}
  .method {{ margin: .8rem 0 0; max-width: 80ch; color: var(--faint); font-size: .85rem; }}
  .totals {{ display: flex; flex-wrap: wrap; gap: 1rem 2.2rem; margin-top: 1.1rem; font-size: .85rem; }}
  .totals b {{ display: block; color: var(--ink); font-size: 1.35rem; font-variant-numeric: tabular-nums; }}
  nav {{ position: sticky; top: 0; z-index: 5; background: color-mix(in srgb, var(--paper) 92%, transparent);
        backdrop-filter: blur(6px); border-bottom: 1px solid var(--line); padding: .55rem 0; font-size: .82rem; }}
  nav .shell {{ display: flex; flex-wrap: wrap; gap: .35rem .9rem; align-items: center; }}
  nav a {{ color: var(--accent); text-decoration: none; white-space: nowrap; }}
  nav a span {{ color: var(--faint); font-family: var(--mono); font-size: .75em; }}
  nav a:hover, nav a:focus-visible {{ text-decoration: underline; }}
  .filters {{ margin-left: auto; display: flex; gap: .35rem; }}
  .filters button {{ font: inherit; font-size: .75rem; cursor: pointer; padding: .2rem .65rem;
                    border-radius: 999px; border: 1px solid var(--line); background: var(--card);
                    color: var(--body); transition: all .15s ease; }}
  .filters button[aria-pressed="true"] {{ background: var(--accent); border-color: var(--accent); color: var(--card); }}
  section {{ margin: 2rem 0 2.6rem; scroll-margin-top: 3.2rem; }}
  .fam-head {{ display: flex; flex-wrap: wrap; gap: .2rem 1rem; align-items: baseline; }}
  h3 {{ color: var(--accent); font-size: 1.05rem; margin: 0; }}
  .fam-count {{ font-family: var(--mono); font-size: .72rem; color: var(--faint);
               text-transform: uppercase; letter-spacing: .06em; }}
  .blurb {{ margin: .3rem 0 .9rem; font-size: .86rem; color: var(--faint); max-width: 80ch; }}
  .grid {{ display: grid; gap: 1.1rem; grid-template-columns: repeat(auto-fill, minmax(min(100%, 520px), 1fr)); }}
  .card {{ margin: 0; background: var(--card); border: 1px solid var(--line); border-radius: 8px;
          overflow: hidden; transition: transform .18s ease, box-shadow .18s ease, border-color .18s ease; }}
  .card:hover {{ transform: translateY(-2px); border-color: var(--accent);
                box-shadow: 0 8px 24px var(--glow); }}
  .card img {{ display: block; width: 100%; height: auto; background: #fff; border-bottom: 1px solid var(--line); }}
  figcaption {{ padding: .6rem .85rem .75rem; }}
  .cap-head {{ display: flex; flex-wrap: wrap; align-items: center; gap: .4rem; }}
  .cap-head code {{ color: var(--ink); font-weight: 600; margin-right: .2rem; }}
  .badge {{ font-family: var(--mono); font-size: .64rem; text-transform: uppercase; letter-spacing: .05em;
           padding: .1rem .45rem; border-radius: 999px; background: var(--accent-bg); color: var(--accent); }}
  .badge.probe {{ background: var(--warn-bg); color: var(--warn); }}
  .badge.warn {{ background: var(--warn-bg); color: var(--warn); }}
  .badge.edge {{ background: var(--edge-bg); color: var(--edge); }}
  .note {{ margin: .35rem 0 .4rem; font-size: .84rem; line-height: 1.45; overflow-wrap: anywhere; }}
  .metrics {{ display: flex; flex-wrap: wrap; gap: .2rem .9rem; font-family: var(--mono);
             font-size: .7rem; color: var(--faint); }}
  .metrics i {{ font-style: normal; color: var(--accent); }}
  .metrics b {{ color: var(--ink); font-weight: 600; }}
  .hidden {{ display: none; }}
  footer {{ padding: 1.5rem 0 2.5rem; color: var(--faint); font-size: .8rem; text-align: center; }}
</style>
</head>
<body>
<header><div class="shell">
  <h1>Benchmark scene gallery</h1>
  <p class="sub"><code>{scenario}</code>, scene by scene, at the certified <code>{profile}</code>
  camera profile. Captions are the scenario file's comments; the numbers under them are the audit's.</p>
  <p class="method">{method}</p>
  <div class="totals">
    <div><b>{scenes}</b>scenes</div><div><b>{instances}</b>target instances</div>
    <div><b>{trials}</b>trials at repeats 1</div><div><b>{probes}</b>probe scenes</div>
  </div>
</div></header>
<nav><div class="shell">{nav}
  <div class="filters" role="group" aria-label="Filter scenes">
    <button type="button" data-filter="" aria-pressed="true">all</button>
    <button type="button" data-filter="warn" aria-pressed="false">flagged</button>
    <button type="button" data-filter="edge" aria-pressed="false">edge</button>
    <button type="button" data-filter="probe" aria-pressed="false">probes</button>
  </div>
</div></nav>
<main class="shell">{sections}</main>
<footer>Generated by <code>scene_audit --gallery</code>. Previews are the audit's ray-cast renders, not simulator captures.</footer>
<script>
  const buttons = document.querySelectorAll('.filters button');
  buttons.forEach(button => button.addEventListener('click', () => {{
    const tag = button.dataset.filter;
    buttons.forEach(b => b.setAttribute('aria-pressed', String(b === button)));
    document.querySelectorAll('.card').forEach(card => {{
      card.classList.toggle('hidden', Boolean(tag) && !card.dataset.tags.split(' ').includes(tag));
    }});
    document.querySelectorAll('section').forEach(section => {{
      section.classList.toggle('hidden', !section.querySelector('.card:not(.hidden)'));
    }});
  }}));
</script>
</body>
</html>
"""
