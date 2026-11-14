"""Shared page frame for the parking scenario and results pages (static HTML, no server).

Styled after docs/target_distance_benchmarking/benchmark_scenarios_v2_gallery.html:
CSS tokens with a dark-mode variant, large figures, and a click-to-enlarge viewer.
"""
import html
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '../..'))
DOCS = os.path.join(ROOT, 'docs/exploration')
SCENARIOS_PAGE = 'autonomous_parking_scenarios.html'
RESULTS_PAGE = 'autonomous_parking_results.html'

CSS = """
  :root {
    --ink: #1d2530; --body: #46525f; --faint: #7d8794;
    --paper: #f6f8fb; --card: #ffffff; --line: #dfe4ea;
    --accent: #3b5b8c; --accent-bg: #eef2f8;
    --ok: #2f7d4f; --ok-bg: #e9f5ee; --safe: #8a6a12; --safe-bg: #fbf3dc;
    --bad: #b3512f; --bad-bg: #fdf1ea; --warn: #6b4f9a; --warn-bg: #f1ecf8;
    --mono: ui-monospace, "SF Mono", "Cascadia Code", Menlo, Consolas, monospace;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --ink: #e8edf3; --body: #b6c0cc; --faint: #8592a1;
      --paper: #12161c; --card: #1a2028; --line: #2c3540;
      --accent: #8fb0e6; --accent-bg: #1f2a3a;
      --ok: #7fd3a1; --ok-bg: #18301f; --safe: #e8c66a; --safe-bg: #332a12;
      --bad: #f0a07c; --bad-bg: #36231b; --warn: #c3a8ef; --warn-bg: #2a2238;
    }
  }
  * { box-sizing: border-box; }
  html { scroll-behavior: smooth; }
  body { margin: 0; background: var(--paper); color: var(--body);
         font: 15px/1.55 -apple-system, "Segoe UI", Roboto, "Helvetica Neue", sans-serif; }
  .shell { max-width: 1320px; margin: 0 auto; padding: 0 16px; }
  header.top { padding: 1.8rem 0 1.2rem; background: var(--card); border-bottom: 1px solid var(--line); }
  nav.pages { display: flex; gap: .4rem; margin-bottom: 1rem; flex-wrap: wrap; }
  nav.pages a { padding: .3rem .8rem; border-radius: 999px; text-decoration: none;
                border: 1px solid var(--line); color: var(--accent); font-size: .85rem; }
  nav.pages a.here { background: var(--accent); color: var(--card); border-color: var(--accent); }
  nav.jump { position: sticky; top: 0; z-index: 5; background: var(--paper);
             border-bottom: 1px solid var(--line); padding: .5rem 0; }
  nav.jump .shell { display: flex; flex-wrap: wrap; gap: .3rem .9rem; font-size: .84rem; }
  nav.jump a { color: var(--accent); text-decoration: none; }
  h1 { color: var(--ink); font-size: 1.65rem; margin: 0 0 .4rem; letter-spacing: -.01em; }
  h2 { color: var(--ink); font-size: 1.25rem; margin: 2.2rem 0 .7rem; }
  h3 { color: var(--accent); font-size: 1.1rem; margin: 0; }
  p, ol, ul { max-width: 84ch; }
  code { font-family: var(--mono); font-size: .86em; }
  .note { color: var(--faint); font-size: .86rem; }
  table { border-collapse: collapse; width: 100%; font-size: .86rem; }
  th, td { text-align: left; padding: .32rem .55rem; border-bottom: 1px solid var(--line); vertical-align: top; }
  th { color: var(--ink); font-weight: 600; }
  .wrap { overflow-x: auto; background: var(--card); border: 1px solid var(--line); border-radius: 8px; }
  .card { background: var(--card); border: 1px solid var(--line); border-radius: 10px;
          padding: 1.1rem 1.2rem; margin: 1.3rem 0; scroll-margin-top: 3rem; }
  .card > header { display: flex; flex-wrap: wrap; gap: .4rem .8rem; align-items: baseline; margin-bottom: .4rem; }
  .card .meta { margin-left: auto; font-family: var(--mono); font-size: .76rem; color: var(--faint); }
  .facts { display: grid; gap: .2rem 1.4rem; grid-template-columns: repeat(auto-fill, minmax(min(100%, 300px), 1fr));
           margin: .5rem 0 1rem; font-size: .88rem; }
  .facts div b { color: var(--ink); font-weight: 600; }
  .compare { display: grid; grid-template-columns: 9rem 1fr 1fr; gap: .6rem; align-items: start; }
  .compare .rowlabel { font-size: .82rem; color: var(--faint); padding-top: .3rem; }
  .compare .colhead { font-weight: 600; color: var(--ink); font-size: .9rem; }
  .views { display: grid; gap: .8rem; grid-template-columns: repeat(auto-fill, minmax(min(100%, 460px), 1fr)); }
  .views figure img { cursor: zoom-in; }
  details.secondary { margin-top: .9rem; }
  details.secondary summary { cursor: pointer; color: var(--faint); font-size: .9rem; }
  details.secondary .views { grid-template-columns: repeat(auto-fill, minmax(min(100%, 340px), 1fr)); margin-top: .6rem; }
  .stages { display: grid; gap: .7rem; grid-template-columns: repeat(auto-fill, minmax(min(100%, 400px), 1fr)); }
  @media (max-width: 760px) { .compare { grid-template-columns: 1fr; } .compare .rowlabel { padding-top: .8rem; } }
  figure { margin: 0; }
  figure img { display: block; width: 100%; height: auto; border-radius: 6px; cursor: zoom-in;
               border: 1px solid var(--line); background: #000; }
  figcaption { font-size: .78rem; color: var(--faint); padding: .25rem 0 .1rem; }
  .missing { border: 1px dashed var(--line); border-radius: 6px; padding: 1.2rem; font-size: .84rem;
             color: var(--faint); text-align: center; }
  .badge { font-family: var(--mono); font-size: .68rem; text-transform: uppercase; letter-spacing: .05em;
           padding: .1rem .5rem; border-radius: 999px; white-space: nowrap; }
  .badge.ok { color: var(--ok); background: var(--ok-bg); }
  .badge.safe { color: var(--safe); background: var(--safe-bg); }
  .badge.bad { color: var(--bad); background: var(--bad-bg); }
  .badge.warn { color: var(--warn); background: var(--warn-bg); }
  .badge.old { color: var(--faint); border: 1px solid var(--line); }
  tr.historical td { color: var(--faint); }
  a { color: var(--accent); }
  #viewer { position: fixed; inset: 0; background: rgba(0,0,0,.88); display: none; z-index: 20;
            align-items: center; justify-content: center; flex-direction: column; padding: 2vh 2vw; cursor: zoom-out; }
  #viewer.open { display: flex; }
  #viewer img { max-width: 96vw; max-height: 88vh; border-radius: 4px; background: #000; }
  #viewer p { color: #dfe4ea; font-size: .9rem; margin: .6rem 0 0; max-width: 96vw; text-align: center; }
"""

VIEWER = """<div id="viewer" role="dialog" aria-label="Enlarged image"><img alt=""><p></p></div>
<script>
  const viewer = document.getElementById('viewer');
  document.querySelectorAll('figure img').forEach(img => img.addEventListener('click', () => {
    viewer.querySelector('img').src = img.src;
    viewer.querySelector('p').textContent = img.closest('figure').querySelector('figcaption')?.textContent || '';
    viewer.classList.add('open');
  }));
  viewer.addEventListener('click', () => viewer.classList.remove('open'));
  document.addEventListener('keydown', e => { if (e.key === 'Escape') viewer.classList.remove('open'); });
</script>"""


def esc(text):
    return html.escape(str(text))


def page(title, here, intro_html, body_html, extra_css=''):
    links = ''.join(
        f'<a href="{target}"{" class=\"here\"" if target == here else ""}>{esc(label)}</a>'
        for target, label in ((SCENARIOS_PAGE, 'Scenarios'), (RESULTS_PAGE, 'Experiments and results'),
                              ('parking_scenarios.md', 'Reference (Markdown)')))
    return (f'<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
            f'<meta name="viewport" content="width=device-width, initial-scale=1">\n'
            f'<title>{esc(title)}</title>\n'
            f'<!-- Generated by tools/parking_scenarios/; edit the generator, not this page. -->\n'
            f'<style>{CSS}{extra_css}</style>\n</head>\n<body>\n'
            f'<header class="top"><div class="shell"><nav class="pages">{links}</nav>'
            f'<h1>{esc(title)}</h1>{intro_html}</div></header>\n{body_html}\n{VIEWER}\n</body>\n</html>\n')


def figure(src, caption, alt=None):
    return (f'<figure><img loading="lazy" src="{esc(src)}" alt="{esc(alt or caption)}">'
            f'<figcaption>{esc(caption)}</figcaption></figure>')


def missing(text):
    return f'<div class="missing">{esc(text)}</div>'


def export_jpeg(src, dst, width=1100, crop=None):
    """A web copy of a verified capture; returns its path relative to the pages."""
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    cmd = ['convert', src] + (['-crop', crop, '+repage'] if crop else []) + [
        '-resize', f'{width}x>', '-strip', '-quality', '78', dst]
    subprocess.run(cmd, check=True)
    return os.path.relpath(dst, DOCS)
