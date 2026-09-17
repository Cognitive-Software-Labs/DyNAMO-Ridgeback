"""Offline docs gate plus parser fixtures independent of ROS and simulators."""
import importlib.util
from pathlib import Path
import subprocess
import sys

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
