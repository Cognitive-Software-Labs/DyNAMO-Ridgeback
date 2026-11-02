#!/usr/bin/env python3
"""Generate a benchmark background's per-scene placements and drop lists.

Evaluates the placement and the drop rule of
``ridgeback_autonomy.benchmarking.background`` for every scene of every
packaged scenario file and writes
``src/ridgeback_autonomy/config/benchmark_backgrounds/<name>.yaml``. Prop and
object footprints come from the Gazebo models the Gazebo asset catalog names,
the geometry's source of truth; the robot's footprint is the navigation
footprint in ``config/nav2_params.yaml``. ``--check`` regenerates in memory
and fails on any difference, so a layout, scenario, model or footprint edit
without regeneration fails. ``--backdrop-gap-m`` evaluates other backdrop
gaps and prints their summaries without writing anything.

    python3 tools/benchmark/build_background.py [--background lab] [--check]
    python3 tools/benchmark/build_background.py --backdrop-gap-m 0.8 1.0 1.5

Needs only PyYAML: no ROS, no simulator.
"""

from __future__ import annotations

import argparse
import difflib
import glob
import importlib.util
import os
from pathlib import Path
import statistics
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / 'src' / 'ridgeback_autonomy'
CONFIG_DIR = PACKAGE_ROOT / 'config'
NAV2_PARAMS = CONFIG_DIR / 'nav2_params.yaml'
GZ_CATALOG = REPO_ROOT / 'src' / 'ridgeback_autonomy_gz' / 'sim' / 'benchmark_assets.yaml'
SCENARIO_GLOB = 'benchmark_scenarios_*.yaml'
# A prop that is the backdrop for more than this share of a file's scenes
# weighs on that share of the results by its colour, shape and height alone.
BACKDROP_SHARE_FLAG = 1.0 / 3.0

sys.path.insert(0, str(PACKAGE_ROOT))
from ridgeback_autonomy.benchmarking import background as bg  # noqa: E402
from ridgeback_autonomy.benchmarking.asset_catalog import load_asset_catalog  # noqa: E402
from ridgeback_autonomy.benchmarking.scenarios import load_scenarios  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    'sdf2usd_background', REPO_ROOT / 'tools' / 'isaac' / 'sdf2usd.py')
sdf2usd = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = sdf2usd
_spec.loader.exec_module(sdf2usd)


def sdf_footprint(resource_path: str) -> bg.Footprint:
    lo, hi = sdf2usd.model_world_aabb(sdf2usd.parse_model(resource_path))
    return bg.Footprint(x0=lo[0], x1=hi[0], y0=lo[1], y1=hi[1])


def robot_footprint() -> tuple[bg.Point, ...]:
    """The navigation footprint, the one both costmaps are configured with."""

    params = yaml.safe_load(NAV2_PARAMS.read_text(encoding='utf-8'))
    polygons = {
        name: yaml.safe_load(params[name][name]['ros__parameters']['footprint'])
        for name in ('local_costmap', 'global_costmap')}
    if polygons['local_costmap'] != polygons['global_costmap']:
        raise bg.BackgroundError(f'{NAV2_PARAMS}: the two costmap footprints differ.')
    return tuple((float(x), float(y)) for x, y in polygons['local_costmap'])


def scenario_files() -> list[Path]:
    return sorted(Path(path) for path in glob.glob(str(CONFIG_DIR / SCENARIO_GLOB)))


def evaluate(name: str, backdrop_gap_m: float | None = None):
    """The layout, footprints, robot footprint, and each file's scenes and placements.

    ``backdrop_gap_m`` overrides the layout's aim, for comparing values.
    """

    layout_path, _drops_path = bg.background_files(str(CONFIG_DIR), name)
    layout = bg.load_background_layout(layout_path)
    catalog = load_asset_catalog(str(GZ_CATALOG))
    selection = catalog.select(layout.world, 'g1', (), layout.models)
    prop_footprints = {
        model: sdf_footprint(resource.uri) for model, resource in selection.background.items()}
    problems = bg.layout_problems(layout, prop_footprints)
    if problems:
        raise bg.BackgroundError(
            f'Layout {layout_path} has problems:\n  ' + '\n  '.join(problems))
    object_footprints = {
        model: sdf_footprint(resource.uri) for model, resource in catalog.objects.items()}
    robot = robot_footprint()

    scenarios = {}
    for path in scenario_files():
        scenes = load_scenarios(str(path))
        placed = {
            scene.id: bg.place_scene(
                layout, prop_footprints, robot, scene, object_footprints,
                backdrop_gap_m=backdrop_gap_m)
            for scene in scenes
        }
        scenarios[path.name] = (bg.sha256_bytes(path.read_bytes()), scenes, placed)
    return layout, prop_footprints, robot, scenarios


def _number(value, digits: int = 4) -> str:
    if isinstance(value, (list, tuple)):
        return '[' + ', '.join(_number(v, digits) for v in value) + ']'
    text = f'{value:.{digits}f}'.rstrip('0').rstrip('.')
    return '0' if text in ('-0', '') else text


def _scene_line(scene_id: str, placement: bg.ScenePlacement) -> str:
    anchor = placement.anchor
    return (
        f'      {scene_id}: {{dx_m: {_number(placement.dx_m)}, '
        f'anchor: {{target_index: {anchor.target_index}, backdrop: {anchor.backdrop}, '
        f'gap_m: {_number(anchor.gap_m)}}}, drops: [{", ".join(placement.drops)}]}}')


def render(name: str, layout, prop_footprints, robot, scenarios) -> str:
    """The drop file, formatted by hand so it diffs one scene per line."""

    lines = [
        f'# Generated by tools/benchmark/build_background.py from',
        f'# {os.path.basename(layout.path)}, the navigation footprint and every packaged',
        '# scenario file. Do not edit: regenerate, and the --check mode keeps this',
        '# file current.',
        '#',
        '# Each scene is captured shifted along world x by dx_m, robot included, so',
        '# its anchor (the target farthest from the camera) stands gap_m in front of',
        '# its backdrop prop (or the wall). It removes the props listed in drops;',
        '# every other prop stays.',
        f'schema: {bg.DROPS_SCHEMA}',
        f'background: {name}',
        f'layout: {{file: {os.path.basename(layout.path)}, sha256: {layout.sha256}}}',
        'rules:',
    ]
    for key, value in bg.RULES.items():
        lines.append(f'  {key}: {_number(value)}')
    lines.append('  robot_footprint: ' + _number([list(point) for point in robot], 6)
                 + '  # nav2_params.yaml')
    lines.append('footprints:  # [x0, x1, y0, y1] in the model frame, from model.sdf')
    for model in sorted(prop_footprints):
        lines.append(f'  {model}: {_number(prop_footprints[model].as_list())}')
    lines.append('scenarios:')
    for file_name, (sha, scenes, placed) in scenarios.items():
        lines.append(f'  {file_name}:')
        lines.append(f'    sha256: {sha}')
        lines.append('    scenes:')
        for scene in scenes:
            lines.append(_scene_line(scene.id, placed[scene.id].placement))
    return '\n'.join(lines) + '\n'


def pass_changes(layout, scenes, placed) -> tuple[int, int]:
    """Removals and restores of one pass over the scenes in file order."""

    present = {prop.id for prop in layout.props}
    removals = restores = 0
    for scene in scenes:
        dropped = set(placed[scene.id].placement.drops)
        removals += len(present & dropped)
        restores += len({prop.id for prop in layout.props} - dropped - present)
        present = {prop.id for prop in layout.props} - dropped
    return removals, restores


def _spread(values) -> str:
    values = sorted(values)
    return (f'min {_number(values[0])} median {_number(statistics.median(values))} '
            f'max {_number(values[-1])}')


def summary(layout, scenarios, backdrop_gap_m: float | None = None) -> str:
    aim = layout.placement.backdrop_gap_m if backdrop_gap_m is None else backdrop_gap_m
    out = []
    for file_name, (_sha, scenes, placed) in scenarios.items():
        placements = [placed[scene.id].placement for scene in scenes]
        count = len(scenes)
        walls = [scene.id for scene, placement in zip(scenes, placements)
                 if placement.anchor.backdrop == bg.WALL]
        prop_gaps = [p.anchor.gap_m for p in placements if p.anchor.backdrop != bg.WALL]
        drops = [len(p.drops) for p in placements]
        removals, restores = pass_changes(layout, scenes, placed)
        out.append(f'{file_name} (backdrop_gap_m {_number(aim)}): {count} scenes, '
                   f'{len(layout.props)} props')
        out.append(f'  dx: {_spread([p.dx_m for p in placements])}')
        if prop_gaps:
            out.append(f'  gap to a prop backdrop ({len(prop_gaps)} scenes): {_spread(prop_gaps)}')
        out.append(f'  wall backdrop: {len(walls)} scenes'
                   + (f': {", ".join(walls)}' if walls else ''))
        out.append(f'  drops per scene: {_spread(drops)}, {sum(d == 0 for d in drops)} drop none; '
                   f'one pass in file order: {removals} removals + {restores} restores '
                   f'= {removals + restores}')
        backdrops: dict[str, int] = {}
        for placement in placements:
            backdrops[placement.anchor.backdrop] = backdrops.get(placement.anchor.backdrop, 0) + 1
        props = sorted((pid for pid in backdrops if pid != bg.WALL),
                       key=lambda pid: (-backdrops[pid], pid))
        out.append(f'  backdrop props: {len(props)} distinct; scenes per backdrop:')
        for pid in props + ([bg.WALL] if bg.WALL in backdrops else []):
            share = backdrops[pid] / count
            flag = '  FLAG: more than a third of the scenes' if share > BACKDROP_SHARE_FLAG else ''
            out.append(f'    {pid:<16} {backdrops[pid]:>3}  {100 * share:5.1f}%{flag}')
    return '\n'.join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--background', default='lab')
    parser.add_argument('--check', action='store_true',
                        help='fail if the committed drop file is not current')
    parser.add_argument('--backdrop-gap-m', type=float, nargs='+', metavar='M',
                        help="evaluate these backdrop gaps instead of the layout's and "
                             'print their summaries; writes nothing')
    args = parser.parse_args()

    if args.backdrop_gap_m:
        if args.check:
            parser.error('--check and --backdrop-gap-m are exclusive')
        for gap in args.backdrop_gap_m:
            layout, _prop_footprints, _robot, scenarios = evaluate(args.background, gap)
            print(summary(layout, scenarios, gap))
        return 0

    layout, prop_footprints, robot, scenarios = evaluate(args.background)
    text = render(args.background, layout, prop_footprints, robot, scenarios)
    _layout_path, drops_path = bg.background_files(str(CONFIG_DIR), args.background)
    if args.check:
        current = Path(drops_path).read_text(encoding='utf-8') if os.path.exists(drops_path) else ''
        if current != text:
            sys.stdout.writelines(difflib.unified_diff(
                current.splitlines(keepends=True), text.splitlines(keepends=True),
                fromfile=drops_path, tofile='regenerated'))
            print(f'{drops_path} is stale; run tools/benchmark/build_background.py',
                  file=sys.stderr)
            return 1
        print(f'{drops_path} is current.')
        print(summary(layout, scenarios))
        return 0
    os.makedirs(os.path.dirname(drops_path), exist_ok=True)
    Path(drops_path).write_text(text, encoding='utf-8')
    print(f'wrote {drops_path}')
    print(summary(layout, scenarios))
    return 0


if __name__ == '__main__':
    sys.exit(main())
