import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    'check_explainer_sources', Path(__file__).resolve().parents[1] / 'check_explainer_sources.py')
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


def test_sources_are_read_from_the_meta_tag(tmp_path):
    page = tmp_path / 'docs' / 'area' / 'assets' / 'explainer.html'
    page.parent.mkdir(parents=True)
    page.write_text('<title>x</title><meta name="explainer-sources" content="docs/a.md, src/b.yaml">')
    (tmp_path / 'docs' / 'other.html').write_text('<title>no sources</title>')
    assert check.explainer_sources(tmp_path) == {'docs/area/assets/explainer.html': ['docs/a.md', 'src/b.yaml']}


def test_page_is_stale_only_when_a_source_changes_without_it():
    pages = {'docs/p.html': ['docs/a.md', 'src/b.yaml']}
    assert check.stale_pages(pages, ['src/b.yaml']) == [('docs/p.html', ['src/b.yaml'])]
    assert check.stale_pages(pages, ['src/b.yaml', 'docs/p.html']) == []
    assert check.stale_pages(pages, ['docs/unrelated.md']) == []
