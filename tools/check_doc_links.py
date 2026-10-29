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

# Page-size budgets (documentation.md#page-size-and-splitting). A page over its
# budget is split; only an indivisible page declares a larger one in-file.
PAGE_BUDGET = 600
PLAN_BUDGET = 500
BACKLOG_ITEM_LIMIT = 60
BUDGET_MARKER = re.compile(r'<!--\s*doc-budget:\s*(\d+)\s*lines?\s*;\s*reason:(.*?)-->', re.S)
SPLIT_HINT = ('split a self-contained section into its own page in the same family and leave '
              'a short summary link; only an indivisible page may declare '
              '<!-- doc-budget: N lines; reason: ... --> (documentation.md#page-size-and-splitting)')
MONTHS = ('January|February|March|April|May|June|July|August|September|October|'
          'November|December')
DATED_HEADING = re.compile(rf'\b\d{{4}}-\d{{2}}-\d{{2}}\b|\b(?:{MONTHS})\s+\d{{1,4}}\b')
MERMAID_NODE = re.compile(r'(?<![\w-])([A-Za-z_]\w*)(\[[^\]]*\]|\([^)]*\)|\{[^}]*\})')


def heading_title(inline) -> str:
    return ''.join(t.content for t in inline.children or []
                   if t.type in ('text', 'code_inline', 'image'))


def heading_slug(title: str) -> str:
    slug = ''.join(c for c in html.unescape(title).lower()
                   if c in ' -_' or unicodedata.category(c)[0] in 'LNM')
    return slug.replace(' ', '-')


def heading_ids(text: str) -> set[str]:
    """GitHub-style heading IDs, duplicate suffixes, and explicit HTML anchors."""
    tokens = PARSER.parse(text)
    ids: set[str] = set()
    for index, token in enumerate(tokens):
        if token.type == 'heading_open':
            slug = heading_slug(heading_title(tokens[index + 1]))
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


def page_kind(relative: Path) -> str:
    parts = relative.parts
    if relative.is_relative_to(ARCHIVE):
        return 'archive'
    if parts[:2] == ('docs', 'plans'):
        return 'plan' if len(parts) == 3 else 'handoff'
    if relative.name == 'BACKLOG.md':
        return 'backlog'
    if parts[:2] == ('docs', 'do_not_try_again'):
        return 'decision'
    return 'reference'


def page_structure(relative: Path, text: str) -> list[tuple[int, str]]:
    """Size, chronology and ownership-shape rules for current documentation."""
    kind = page_kind(relative)
    if kind in ('archive', 'handoff'):
        return []
    tokens = PARSER.parse(text)
    errors = []
    lines = text.splitlines()

    markers = []
    for token in tokens:
        if token.type == 'html_block':
            raw = token.content
        elif token.type == 'inline':
            raw = ''.join(t.content for t in token.children or [] if t.type == 'html_inline')
        else:
            continue
        markers += [(token.map[0] + 1, m) for m in BUDGET_MARKER.finditer(raw)]
    budget = PLAN_BUDGET if kind == 'plan' else PAGE_BUDGET
    if len(markers) > 1:
        errors.append((markers[1][0], 'declare at most one doc-budget marker'))
    if markers:
        line, marker = markers[0]
        if not marker.group(2).strip():
            errors.append((line, 'doc-budget marker needs a reason naming why the page cannot be split'))
        budget = int(marker.group(1))
    if len(lines) > budget:
        errors.append((1, f'{len(lines)} lines exceeds the {budget}-line budget; {SPLIT_HINT}'))

    headings = []
    for index, token in enumerate(tokens):
        if token.type == 'heading_open':
            headings.append((token.map[0] + 1, token.tag, heading_title(tokens[index + 1])))
        elif (token.type == 'fence' and token.info.strip() == 'mermaid'
              and token.content.lstrip().startswith(('flowchart', 'graph'))):
            labels = {}
            for node, label in MERMAID_NODE.findall(token.content):
                if labels.setdefault(node, label) != label:
                    errors.append((token.map[0] + 1, f'mermaid node {node} reused with labels '
                                   f'{labels[node]} and {label}; give each node its own ID'))

    slugs: dict[str, list[tuple[int, str]]] = {}
    for line, tag, title in headings:
        slugs.setdefault(heading_slug(title), []).append((line, tag))
        if kind in ('reference', 'backlog') and DATED_HEADING.search(title):
            errors.append((line, 'dated heading in a current page; put the dated run in an '
                           'archive record and keep only the current result here'))
    for slug, found in slugs.items():
        if len(found) > 1 and any(tag in ('h1', 'h2') for _, tag in found):
            errors.append((found[1][0], f'duplicate section heading "{slug}"; merge the sections '
                           'or give each a distinct name'))

    if kind == 'plan' and not any(re.match(r'\**Status', line) for line in lines[:15]):
        errors.append((1, 'plan needs a Status: line within its first 15 lines'))
    if kind == 'backlog':
        starts = [line for line, tag, _ in headings if tag == 'h2'] + [len(lines) + 1]
        titles = [title for _, tag, title in headings if tag == 'h2']
        for title, start, end in zip(titles, starts, starts[1:]):
            if title == 'Archived evidence':
                continue
            body = lines[start - 1:end - 1]
            while body and not body[-1].strip():
                body.pop()
            if not any('**Completion criteria' in line for line in body):
                errors.append((start, 'backlog item needs a **Completion criteria.** paragraph'))
            if len(body) > BACKLOG_ITEM_LIMIT:
                errors.append((start, f'backlog item has {len(body)} lines (limit '
                               f'{BACKLOG_ITEM_LIMIT}); move its procedure or history to a plan '
                               'or reference and link it'))
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
    index = root / 'docs/index.md'
    indexed = {(index.parent / unquote(urlsplit(target).path)).resolve()
               for _, target in links(index.read_text(encoding='utf-8'))} if index.exists() else None
    for source in sources:
        source = source.resolve()
        archived = source.is_relative_to(archive)
        text = source.read_text(encoding='utf-8')
        relative = source.relative_to(root)
        if archived and source != archive / 'README.md':
            failures.extend(f'{relative}:1: {error}' for error in archive_metadata(text))
        failures.extend(f'{relative}:{line}: {error}'
                        for line, error in page_structure(relative, text))
        if indexed is not None and page_kind(relative) == 'plan' and source not in indexed:
            failures.append(f'{relative}:1: plan is not linked from docs/index.md')
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
