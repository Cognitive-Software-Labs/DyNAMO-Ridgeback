#!/usr/bin/env python3
"""Tabulate gz-vs-isaac exploration A/B results and check the P5 acceptance gate.

Consumes the summary JSONs written by ``tools/benchmark/explore_probe.py`` (one
per run). The gz baseline lives in ``tools/isaac/baseline/gz_mock_hospital/``;
point ``--isaac`` at the directory of fresh Isaac run summaries.

Columns: success, coverage complete%, coverage accuracy%, time-to-complete (wall
s), achieved RTF, genuine aborts, localization error (max m). Fields the run
never recorded show ``—``: the gz baseline predates RTF/accuracy/localization
capture, and gz has no ground-truth pose so localization is Isaac-only.

Run classification is by tag: a summary whose name contains ``rtf0`` is the
unthrottled run (excluded from the "3/3 complete" and RTF-floor checks, and used
only for the "faster wall-clock" check).

Acceptance gate (PORT_PLAN §P5), pure post-processing — no sim required:
  1. all throttled Isaac runs complete, and there are >= 3 of them
  2. Isaac mean coverage-complete >= gz mean - margin (default 10 pts)
  3. Isaac max genuine aborts <= gz max genuine aborts
  4. every throttled Isaac run's achieved RTF >= floor (default 0.8)
  5. the --rtf 0 run completes in less wall-clock than the throttled mean

    python3 tools/isaac/ab_compare.py --isaac <isaac_runs_dir>
    python3 tools/isaac/ab_compare.py --gz <dir> --isaac <dir> --json out.json
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path
from statistics import mean

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GZ = REPO_ROOT / 'tools' / 'isaac' / 'baseline' / 'gz_mock_hospital'
NA = '—'          # em dash for missing values


def load_runs(spec: str) -> list[dict]:
    """Load every *_summary.json under a dir (or matching a glob), tagged by name."""
    path = Path(spec)
    if path.is_dir():
        files = sorted(path.glob('*_summary.json'))
    else:
        files = [Path(p) for p in sorted(glob.glob(spec))]
    runs = []
    for f in files:
        d = json.loads(f.read_text())
        d['run'] = f.name.replace('_summary.json', '')
        runs.append(d)
    return runs


# -- field accessors (tolerant of the older gz baseline schema) ---------------

def success(r):      return r.get('quit_at_s') is not None
def cov(r):          return r.get('coverage_peak_pct')
def acc(r):          return r.get('coverage_accuracy_pct')
def ttc(r):          return r.get('quit_at_s')
def rtf(r):          return r.get('achieved_rtf')
def aborts(r):       return r.get('aborts_genuine')
def loc_max(r):      return r.get('localization_err_max_m')
def is_rtf0(r):      return 'rtf0' in r['run'].lower()


def _f(v, fmt='{:.1f}'):
    return NA if v is None else fmt.format(v)


# -- table --------------------------------------------------------------------

COLS = [
    ('run', 12, lambda r: r['run']),
    ('ok', 4, lambda r: 'yes' if success(r) else 'no'),
    ('cov%', 6, lambda r: _f(cov(r))),
    ('acc%', 6, lambda r: _f(acc(r))),
    ('ttc(s)', 8, lambda r: _f(ttc(r), '{:.0f}')),
    ('RTF', 6, lambda r: _f(rtf(r), '{:.2f}')),
    ('aborts', 7, lambda r: _f(aborts(r), '{:.0f}')),
    ('loc(m)', 7, lambda r: _f(loc_max(r), '{:.2f}')),
]


def render_table(gz: list[dict], isaac: list[dict]) -> str:
    header = '  '.join(f'{name:<{w}}' for name, w, _ in COLS)
    rule = '-' * len(header)
    lines = [header, rule]

    def row(r):
        return '  '.join(f'{fn(r):<{w}}' for _, w, fn in COLS)

    def agg(label, runs, key):
        vals = [key(x) for x in runs if key(x) is not None]
        if not vals:
            return f'{label}: {NA}'
        return (f'{label}: mean {mean(vals):.1f}  min {min(vals):.1f}  '
                f'max {max(vals):.1f}')

    lines.append('# gz baseline')
    lines += [row(r) for r in gz]
    if gz:
        lines.append('  ' + agg('cov%', gz, cov) + '   ' + agg('aborts', gz, aborts))
    lines.append('')
    lines.append('# isaac')
    lines += [row(r) for r in isaac]
    if isaac:
        thr = [r for r in isaac if not is_rtf0(r)]
        if thr:
            lines.append('  ' + agg('cov%', thr, cov) + '   ' + agg('aborts', thr, aborts))
    return '\n'.join(lines)


# -- acceptance gate ----------------------------------------------------------

def check_gate(gz, isaac, margin, rtf_floor):
    """Return a list of (name, passed | None, detail). None = undecidable."""
    out = []
    thr = [r for r in isaac if not is_rtf0(r)]
    rtf0 = [r for r in isaac if is_rtf0(r)]

    gz_cov = [cov(r) for r in gz if cov(r) is not None]
    gz_abrt = [aborts(r) for r in gz if aborts(r) is not None]

    # 1. >= 3 throttled Isaac runs, all complete
    n_ok = sum(success(r) for r in thr)
    out.append((
        f'>=3 throttled runs complete',
        len(thr) >= 3 and n_ok == len(thr),
        f'{n_ok}/{len(thr)} complete'))

    # 2. coverage-complete >= gz mean - margin
    if thr and gz_cov:
        i_cov = [cov(r) for r in thr if cov(r) is not None]
        thr_mean = mean(i_cov) if i_cov else None
        floor = mean(gz_cov) - margin
        out.append((
            'coverage-complete >= gz mean - margin',
            thr_mean is not None and thr_mean >= floor,
            f'isaac mean {thr_mean if thr_mean is None else round(thr_mean,1)} '
            f'>= {round(floor,1)} (gz mean {round(mean(gz_cov),1)} - {margin})'))
    else:
        out.append(('coverage-complete >= gz mean - margin', None,
                    'need gz + isaac coverage'))

    # 3. genuine aborts <= gz max
    if thr and gz_abrt:
        i_abrt = [aborts(r) for r in thr if aborts(r) is not None]
        i_max = max(i_abrt) if i_abrt else None
        out.append((
            'genuine aborts <= gz max',
            i_max is not None and i_max <= max(gz_abrt),
            f'isaac max {i_max} <= gz max {max(gz_abrt)}'))
    else:
        out.append(('genuine aborts <= gz max', None, 'need gz + isaac aborts'))

    # 4. achieved RTF >= floor for throttled runs
    rtfs = [(r['run'], rtf(r)) for r in thr if rtf(r) is not None]
    if rtfs:
        worst = min(v for _, v in rtfs)
        out.append((
            f'throttled RTF >= {rtf_floor}',
            worst >= rtf_floor,
            f'worst {worst:.2f} across {len(rtfs)} run(s)'))
    else:
        out.append((f'throttled RTF >= {rtf_floor}', None, 'no RTF recorded'))

    # 5. rtf0 run faster wall-clock than throttled mean
    thr_ttc = [ttc(r) for r in thr if ttc(r) is not None]
    r0_ttc = [ttc(r) for r in rtf0 if ttc(r) is not None]
    if thr_ttc and r0_ttc:
        best0, thr_avg = min(r0_ttc), mean(thr_ttc)
        out.append((
            '--rtf 0 faster wall-clock',
            best0 < thr_avg,
            f'rtf0 {best0:.0f}s < throttled mean {thr_avg:.0f}s'))
    else:
        out.append(('--rtf 0 faster wall-clock', None,
                    'need a completed rtf0 run + throttled run'))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--gz', default=str(DEFAULT_GZ),
                    help='gz baseline dir or *_summary.json glob')
    ap.add_argument('--isaac', default=None,
                    help='Isaac runs dir or *_summary.json glob')
    ap.add_argument('--margin', type=float, default=10.0,
                    help='coverage-complete slack below gz mean (pts)')
    ap.add_argument('--rtf-floor', type=float, default=0.8)
    ap.add_argument('--json', type=Path, default=None,
                    help='also write machine-readable results here')
    args = ap.parse_args()

    gz = load_runs(args.gz)
    isaac = load_runs(args.isaac) if args.isaac else []

    print(render_table(gz, isaac))

    verdict = None
    if not isaac:
        print('\n(no --isaac runs given; showing gz baseline only)')
    else:
        print('\nACCEPTANCE GATE (PORT_PLAN §P5)')
        gate = check_gate(gz, isaac, args.margin, args.rtf_floor)
        decided = [p for _, p, _ in gate if p is not None]
        for name, passed, detail in gate:
            mark = 'PASS' if passed else ('FAIL' if passed is False else 'n/a ')
            print(f'  [{mark}] {name:<38} {detail}')
        verdict = bool(decided) and all(p for _, p, _ in gate if p is not None)
        undecided = [n for n, p, _ in gate if p is None]
        tail = f' ({len(undecided)} undecided)' if undecided else ''
        print(f'\n  OVERALL: {"PASS" if verdict else "FAIL"}{tail}')

    if args.json:
        args.json.write_text(json.dumps({
            'gz': gz, 'isaac': isaac,
            'gate': [{'name': n, 'passed': p, 'detail': d}
                     for n, p, d in (check_gate(gz, isaac, args.margin,
                                                args.rtf_floor) if isaac else [])],
            'verdict': verdict,
        }, indent=2))
        print(f'\nwrote {args.json}')

    return 0 if (not isaac or verdict) else 1


if __name__ == '__main__':
    raise SystemExit(main())
