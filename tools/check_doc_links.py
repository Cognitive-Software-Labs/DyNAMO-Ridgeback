#!/usr/bin/env python3
"""Offline Markdown links, archive provenance, and knowledge-boundary checks."""
from __future__ import annotations

import argparse
from datetime import date
import html
from pathlib import Path
import re
import unicodedata
from urllib.parse import unquote, urlsplit

from markdown_it import MarkdownIt

PARSER = MarkdownIt('commonmark')
ARCHIVE = Path('archive/engineering')
REPOSITORY_PATH = '/cognitive-software-labs/dynamo-ridgeback/'


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


def link_contexts(text: str):
    """Resolve Markdown links with their containing level-two section."""
    section = None
    tokens = PARSER.parse(text)
    for index, token in enumerate(tokens):
        if token.type == 'heading_open' and token.tag in ('h1', 'h2'):
            section = tokens[index + 1].content if token.tag == 'h2' else None
        if token.type != 'inline':
            continue
        for child in token.children or []:
            attr = 'href' if child.type == 'link_open' else 'src' if child.type == 'image' else None
            if attr:
                yield token.map[0] + 1, child.attrGet(attr), section, child.type == 'image'


def links(text: str):
    """The parser resolves inline/reference links and excludes code examples."""
    for line, target, _, _ in link_contexts(text):
        yield line, target


def archive_metadata(text: str) -> list[str]:
    """Check declared provenance, without pretending to authenticate a run."""
    # Only the record header counts; an example or a later quote cannot supply it.
    header = '\n'.join(token.content for token in PARSER.parse(text.split('\n## ', 1)[0])
                       if token.type == 'inline' and token.level == 1)
    errors = []
    values = {}
    for field in ('Recorded dates', 'Tested revisions', 'Provenance'):
        matches = re.findall(rf'^{field}: (.+)$', header, re.M)
        if len(matches) != 1:
            errors.append(f'archive header requires exactly one {field} field')
        else:
            values[field] = matches[0]
    dates = values.get('Recorded dates', 'unknown')
    if dates != 'unknown':
        for value in dates.split(', '):
            try:
                if not re.fullmatch(r'\d{4}-\d{2}-\d{2}', value):
                    raise ValueError
                date.fromisoformat(value)
            except ValueError:
                errors.append('Recorded dates must be comma-separated ISO dates or unknown')
                break
    revisions = values.get('Tested revisions', 'unknown')
    if revisions != 'unknown' and not re.fullmatch(r'`[0-9a-f]{40}`(?:, `[0-9a-f]{40}`)*', revisions):
        errors.append('Tested revisions must be full commit hashes in backticks or unknown')
    provenance = values.get('Provenance')
    if provenance is not None and provenance not in ('complete', 'partial', 'unknown'):
        errors.append('Provenance must be complete, partial, or unknown')
    if provenance == 'complete' and (dates == 'unknown' or revisions == 'unknown'):
        errors.append('complete provenance requires recorded dates and tested revisions')
    return errors


def check(root: Path, sources: list[Path] | None = None) -> list[str]:
    root = root.resolve()
    archive = root / ARCHIVE
    if sources is None:
        # Deliberately walk the archive even though normal search ignores it.
        sources = sorted(set(root.glob('*.md')) | set((root / 'docs').rglob('*.md'))
                         | set(archive.rglob('*.md')))
    failures = []
    anchors = {}
    for source in sources:
        source = source.resolve()
        archived = source.is_relative_to(archive)
        text = source.read_text(encoding='utf-8')
        if archived and source != archive / 'README.md':
            failures.extend(f'{source.relative_to(root)}:1: {error}'
                            for error in archive_metadata(text))
        for line, target, section, is_image in link_contexts(text):
            url = urlsplit(target)
            label = f'{source.relative_to(root)}:{line}: {target}'
            repository_path = unquote(url.path)
            if (archived and url.hostname in ('github.com', 'www.github.com')
                    and repository_path.lower().startswith(REPOSITORY_PATH)):
                relative = repository_path[len(REPOSITORY_PATH):]
                if not re.match(r'(?:blob|tree|commit)/[0-9a-f]{40}(?:/|#|$)', relative):
                    failures.append(f'{label}: archive repository links must pin a full commit')
            if (archived and url.hostname == 'raw.githubusercontent.com'
                    and repository_path.lower().startswith(REPOSITORY_PATH)):
                ref = repository_path.split('/')[3]
                if not re.fullmatch(r'[0-9a-f]{40}', ref):
                    failures.append(f'{label}: archive repository links must pin a full commit')
            if url.scheme or url.netloc or url.path.startswith(('/', '~')):
                if archived and not url.scheme and not url.netloc and url.path.startswith(('/', '~')):
                    failures.append(f'{label}: archive links must use preserved relative assets or immutable sources')
                continue
            destination = (source.parent / unquote(url.path)).resolve() if url.path else source.resolve()
            if archived and not destination.is_relative_to(archive):
                failures.append(f'{label}: archive must not link to live working-tree files')
            elif not archived and destination.is_relative_to(archive):
                if is_image:
                    failures.append(f'{label}: dated figures belong in their archive record')
                elif destination != archive / 'README.md' and section != 'Archived evidence':
                    failures.append(f'{label}: archive record links belong under ## Archived evidence')
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
    print(f'Documentation checks: {len(failures)} failure(s)')
    return bool(failures)


if __name__ == '__main__':
    raise SystemExit(main())
