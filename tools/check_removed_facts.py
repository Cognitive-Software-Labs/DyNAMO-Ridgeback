#!/usr/bin/env python3
"""List distinctive facts deleted from Markdown that no longer appear in the tree.

Run before committing a change that removes or condenses documentation, such as
retiring a plan or handoff, splitting a page or rewriting a backlog item:

    tools/check_removed_facts.py              # working tree against HEAD
    tools/check_removed_facts.py --cached     # staged changes
    tools/check_removed_facts.py HEAD~1 HEAD  # a commit (searches the working tree)
    tools/check_removed_facts.py --path docs/project/dependencies.md

Each reported token (code names, quantities with units, dates, paths, hashes) was
on a deleted line and is not found in any tracked or new text file. Decide for
each whether it was superseded or must move to its owning page. Exits 1 when
anything is reported.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import re
import subprocess

TEXT_SUFFIXES = ('.md', '.py', '.sh', '.yaml', '.yml', '.txt', '.xml', '.json', '.repos',
                 '.cfg', '.toml', '.service', '.conf')
SKIP_PREFIXES = ('graphify-out/', 'build/', 'install/', 'log/', 'artifacts/')
UNIT = r'(?:ms|Hz|s|m/s|rad/s|mm|cm|m|Mbit/s|Gb/s|MiB|GB|min|h|°C|%)'
PATTERNS = (
    re.compile(r'`([^`\n]{4,100})`'),
    re.compile(rf'(?<![\w.])(\d+(?:\.\d+)?\s?{UNIT})(?![\w/])'),
    re.compile(r'\b(20\d\d-\d\d-\d\d(?:T[\w-]*)?)'),
    re.compile(r'((?:artifacts|tools|patches|deployment)/[\w./<>*-]+)'),
    re.compile(r'\b([0-9a-f]{7,40})\b'),
)


def git(root: Path, *args: str) -> str:
    return subprocess.run(['git', '-C', str(root), *args], check=True,
                          capture_output=True, text=True).stdout


def removed_lines(root: Path, diff_args: list[str], paths: list[str]) -> dict[str, list[str]]:
    removed: dict[str, list[str]] = {}
    source = None
    diff = git(root, 'diff', '-U0', '--no-color', *diff_args, '--', *(paths or ['*.md']))
    for line in diff.splitlines():
        if line.startswith('--- '):
            source = line[6:] if line.startswith('--- a/') else None
        elif line.startswith('-') and not line.startswith('---') and source:
            removed.setdefault(source, []).append(line[1:])
    return removed


def tokens(lines: list[str]) -> set[str]:
    found = set()
    for line in lines:
        for pattern in PATTERNS:
            for token in pattern.findall(line):
                token = token.strip().rstrip('.,;:')
                if len(token) >= 4 and re.search(r'[A-Za-z0-9]', token):
                    found.add(token)
    return found


def corpus(root: Path) -> str:
    names = git(root, 'ls-files', '-co', '--exclude-standard').splitlines()
    parts = []
    for name in names:
        if name.endswith(TEXT_SUFFIXES) and not name.startswith(SKIP_PREFIXES):
            path = root / name
            if path.is_file():
                parts.append(path.read_text(encoding='utf-8', errors='ignore'))
    return '\n'.join(parts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('diff_args', nargs='*', help='arguments passed to git diff')
    parser.add_argument('--cached', action='store_true', help='check staged changes')
    parser.add_argument('--path', action='append', default=[],
                        help='limit the diff to this Markdown path (repeatable)')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    diff_args = (['--cached'] if args.cached else []) + (args.diff_args or
                                                           ([] if args.cached else ['HEAD']))
    text = corpus(args.root)
    missing = 0
    for source, lines in sorted(removed_lines(args.root, diff_args, args.path).items()):
        lost = sorted(t for t in tokens(lines) if t not in text)
        if lost:
            print(f'{source}:')
            print(''.join(f'  {token}\n' for token in lost), end='')
            missing += len(lost)
    print(f'Removed facts not found elsewhere: {missing}')
    return bool(missing)


if __name__ == '__main__':
    raise SystemExit(main())
