"""Offline docs gate plus parser fixtures independent of ROS and simulators."""
import importlib.util
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location('check_doc_links', ROOT / 'tools/check_doc_links.py')
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def test_valid_links_images_references_and_fragments(tmp_path):
    (tmp_path / 'docs').mkdir()
    (tmp_path / 'docs/page name.md').write_text('# Héllo `world`!\n\n## Again\n## Again\n<a id="custom"></a>\n')
    (tmp_path / 'image.png').touch()
    (tmp_path / 'README.md').write_text('''[inline](docs/page%20name.md#héllo-world)
[duplicate](<docs/page name.md#again-1>)
[ref][page]
![image](image.png)
[custom](docs/page%20name.md#custom)

[page]: docs/page%20name.md#again "Title"
''')
    assert CHECKER.check(tmp_path) == []


def test_missing_targets_and_fragments_report_source(tmp_path):
    (tmp_path / 'page.md').write_text('# Real\n')
    (tmp_path / 'README.md').write_text('''[moved](deleted.md)
[heading](page.md#absent)
![missing](lost.png)
[reference][gone]

[gone]: absent.md
''')
    failures = CHECKER.check(tmp_path)
    assert len(failures) == 4
    assert any('README.md:1:' in f and 'deleted.md' in f for f in failures)
    assert any('missing heading/anchor' in f for f in failures)


def test_examples_and_nonrepository_links_are_ignored(tmp_path):
    (tmp_path / 'README.md').write_text('''`[example](missing.md)`

```markdown
[example](missing.md)
```

    [indented](missing.md)

[web](https://example.com/absent)
[mail](mailto:somebody@example.com)
[local](/home/user/absent)
[home](~/absent)
''')
    assert CHECKER.check(tmp_path) == []


def test_heading_slug_formatting_and_duplicates():
    assert {'a-b', 'a-b-1', 'under_score', 'setext'} <= CHECKER.heading_ids(
        '# **A** [B](https://example.com)\n# A B\n# under_score\n\nSetext\n---\n')


def test_cli_fails_for_broken_link(tmp_path):
    (tmp_path / 'README.md').write_text('[broken](missing.md)')
    result = subprocess.run([sys.executable, str(ROOT / 'tools/check_doc_links.py'),
                             '--root', str(tmp_path)], capture_output=True, text=True)
    assert result.returncode == 1
    assert 'README.md:1:' in result.stdout


def test_repository_documentation_links():
    assert CHECKER.check(ROOT) == []


RECORD_HEADER = '''# Recorded experiment

Recorded dates: 2026-09-01
Tested revisions: unknown
Provenance: partial

## Result

The original source did not preserve its tested revision.
'''


def _archive(tmp_path):
    archive = tmp_path / 'archive/engineering'
    archive.mkdir(parents=True)
    (archive / 'README.md').write_text('# Engineering archive\n\n[Record](record.md)\n')
    (archive / 'record.md').write_text(RECORD_HEADER)
    (tmp_path / 'docs').mkdir()
    return archive


def test_archive_opt_in_navigation_and_honest_missing_provenance(tmp_path):
    _archive(tmp_path)
    (tmp_path / 'docs/reference.md').write_text('''# Current contract

This explains the current behavior without historical prerequisites.
[Archive index](../archive/engineering/README.md)

## Archived evidence

### September experiment

[Dated result](../archive/engineering/record.md#result)
''')
    assert CHECKER.check(tmp_path) == []


def test_archive_stays_validated_even_when_search_ignores_it(tmp_path):
    archive = _archive(tmp_path)
    (tmp_path / '.ignore').write_text('/archive/\n')
    (archive / 'record.md').write_text(RECORD_HEADER + '\n![Trace](missing.png)\n')
    failures = CHECKER.check(tmp_path)
    assert len(failures) == 1
    assert 'archive/engineering/record.md' in failures[0]
    assert 'missing target' in failures[0]


@pytest.mark.parametrize('link', [
    '[Record](../archive/engineering/record.md)',
    '[Record][past]\n\n[past]: ../archive/engineering/record.md',
])
def test_current_contract_cannot_inline_archive_links(tmp_path, link):
    _archive(tmp_path)
    (tmp_path / 'docs/reference.md').write_text('# Current contract\n\n' + link)
    failures = CHECKER.check(tmp_path)
    assert len(failures) == 1
    assert 'under ## Archived evidence' in failures[0]


def test_dated_images_stay_with_their_records(tmp_path):
    archive = _archive(tmp_path)
    (archive / 'trace.svg').write_text('<svg/>')
    (tmp_path / 'docs/reference.md').write_text(
        '# Reference\n\n## Archived evidence\n\n![Old trace](../archive/engineering/trace.svg)')
    assert any('dated figures belong' in f for f in CHECKER.check(tmp_path))


def test_archive_cannot_link_back_to_live_contract(tmp_path):
    archive = _archive(tmp_path)
    (tmp_path / 'docs/reference.md').write_text('# Current contract\n')
    (archive / 'record.md').write_text(
        RECORD_HEADER + '\n[Mutable explanation](../../docs/reference.md)\n')
    assert any('live working-tree' in f for f in CHECKER.check(tmp_path))


@pytest.mark.parametrize('url', [
    'https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/main/file.py',
    'http://www.github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/tree/branch',
    'https://raw.githubusercontent.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/main/file.py',
    'https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/%6dain/file.py',
])
def test_archive_repository_source_links_require_full_immutable_revision(tmp_path, url):
    archive = _archive(tmp_path)
    (archive / 'record.md').write_text(RECORD_HEADER + f'\n[Source]({url})\n')
    assert any('pin a full commit' in f for f in CHECKER.check(tmp_path))
    pinned = 'https://github.com/Cognitive-Software-Labs/DyNAMO-Ridgeback/blob/' + 'a' * 40 + '/file.py'
    (archive / 'record.md').write_text(RECORD_HEADER + f'\n[Source]({pinned})\n')
    assert CHECKER.check(tmp_path) == []


@pytest.mark.parametrize('header', [
    RECORD_HEADER.replace('Recorded dates: 2026-09-01\n', ''),
    RECORD_HEADER.replace('2026-09-01', '2026-02-30'),
    RECORD_HEADER.replace('Tested revisions: unknown', 'Tested revisions: `abcdef0`'),
    RECORD_HEADER.replace('Provenance: partial', 'Provenance: verified'),
    RECORD_HEADER.replace('Provenance: partial', 'Provenance: complete'),
    RECORD_HEADER.replace('Provenance: partial', 'Provenance: partial\nProvenance: unknown'),
    '# Example\n\n```text\n' + RECORD_HEADER.split('## Result')[0] + '```\n\n## Result\n',
])
def test_missing_or_misleading_archive_metadata_is_rejected(header):
    assert CHECKER.archive_metadata(header)


def test_complete_and_unknown_archive_metadata():
    assert CHECKER.archive_metadata(RECORD_HEADER.replace(
        'Tested revisions: unknown', 'Tested revisions: `' + 'a' * 40 + '`'
    ).replace('Provenance: partial', 'Provenance: complete')) == []
    assert CHECKER.archive_metadata(RECORD_HEADER.replace(
        'Recorded dates: 2026-09-01', 'Recorded dates: unknown'
    ).replace('Provenance: partial', 'Provenance: unknown')) == []
