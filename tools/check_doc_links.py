#!/usr/bin/env python3
"""Offline repository Markdown file/image and heading-link validation."""
from __future__ import annotations

import argparse
import html
from pathlib import Path
import re
import unicodedata
from urllib.parse import unquote, urlsplit

from markdown_it import MarkdownIt

PARSER = MarkdownIt('commonmark')


def heading_ids(text: str) -> set[str]:
    """GitHub-style heading IDs, duplicate suffixes, and explicit HTML anchors."""
    tokens = PARSER.parse(text)
    ids: set[str] = set()
    for index, token in enumerate(tokens):
        if token.type == 'heading_open':
            inline = tokens[index + 1]
            title = ''.join(t.content for t in inline.children or []
                            if t.type in ('text', 'code_inline', 'image'))
            slug = ''.join(c for c in html.unescape(title).lower()
                           if c in ' -_' or unicodedata.category(c)[0] in 'LNM')
            slug = slug.replace(' ', '-')
            candidate, suffix = slug, 0
            while candidate in ids:
                suffix += 1
                candidate = f'{slug}-{suffix}'
            ids.add(candidate)
        if token.type in ('html_block', 'inline'):
            raw = token.content if token.type == 'html_block' else ''.join(
                t.content for t in token.children or [] if t.type == 'html_inline')
            ids.update(re.findall(r'\b(?:id|name)=["\']([^"\']+)["\']', raw))
    return ids


def links(text: str):
    """The parser resolves inline/reference links and excludes code examples."""
    for token in PARSER.parse(text):
        if token.type != 'inline':
            continue
        for child in token.children or []:
            attr = 'href' if child.type == 'link_open' else 'src' if child.type == 'image' else None
            if attr:
                yield (token.map[0] + 1), child.attrGet(attr)


def check(root: Path, sources: list[Path] | None = None) -> list[str]:
    root = root.resolve()
    if sources is None:
        sources = sorted(set(root.glob('*.md')) | set((root / 'docs').rglob('*.md')))
    failures = []
    anchors = {}
    for source in sources:
        for line, target in links(source.read_text(encoding='utf-8')):
            url = urlsplit(target)
            if url.scheme or url.netloc or url.path.startswith(('/', '~')):
                continue
            destination = (source.parent / unquote(url.path)).resolve() if url.path else source.resolve()
            label = f'{source.relative_to(root)}:{line}: {target}'
            if not destination.is_relative_to(root):
                failures.append(f'{label}: target escapes repository')
            elif not destination.exists():
                failures.append(f'{label}: missing target')
            elif url.fragment and destination.suffix.lower() == '.md':
                if destination not in anchors:
                    anchors[destination] = heading_ids(destination.read_text(encoding='utf-8'))
                if unquote(url.fragment) not in anchors[destination]:
                    failures.append(f'{label}: missing heading/anchor')
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    failures = check(args.root)
    for failure in failures:
        print(failure)
    print(f'Documentation links: {len(failures)} failure(s)')
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
