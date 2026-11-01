#!/usr/bin/env python3
"""Fail a commit that changes an explainer page's sources without touching the page.

Each people-facing explainer page (an ``.html`` file under ``docs/``) declares the
files its claims come from in ``<meta name="explainer-sources" content="a, b">``.
Policy: docs/project/documentation.md#explainer-pages. Set
``EXPLAINER_SOURCES_UNCHANGED=1`` for a commit whose staged source changes alter
no claim the page shows.
"""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

META = re.compile(r'<meta\s+name="explainer-sources"\s+content="([^"]*)"', re.I)


def explainer_sources(root):
    """Map each explainer page to the repository files it draws on."""
    pages = {}
    for page in sorted((root / 'docs').rglob('*.html')):
        match = META.search(page.read_text(encoding='utf-8', errors='replace'))
        if match:
            pages[page.relative_to(root).as_posix()] = [s.strip() for s in match.group(1).split(',') if s.strip()]
    return pages


def stale_pages(pages, changed):
    """Pages with a changed source that were not changed themselves."""
    changed = set(changed)
    return [(page, [s for s in sources if s in changed]) for page, sources in pages.items()
            if page not in changed and any(s in changed for s in sources)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--files', nargs='*', help='Changed paths; default: the staged files')
    args = parser.parse_args()
    root = args.root.resolve()
    changed = args.files if args.files is not None else subprocess.run(
        ['git', 'diff', '--cached', '--name-only', '--diff-filter=ACMRD'],
        cwd=root, capture_output=True, text=True, check=True).stdout.split()
    pages = explainer_sources(root)
    failures = [f'{page}: listed source {source} does not exist'
                for page, sources in pages.items() for source in sources if not (root / source).exists()]
    if os.environ.get('EXPLAINER_SOURCES_UNCHANGED') != '1':
        failures += [f'{page}: sources changed ({", ".join(hit)}); update the page and any published copy, '
                     'or commit with EXPLAINER_SOURCES_UNCHANGED=1 if no shown claim changed'
                     for page, hit in stale_pages(pages, changed)]
    for failure in failures:
        print(f'[explainer check] {failure}', file=sys.stderr)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
