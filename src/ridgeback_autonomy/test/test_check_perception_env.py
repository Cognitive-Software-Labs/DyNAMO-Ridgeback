"""The perception GPU gate fails rather than passing an environment without CUDA PyTorch."""
from importlib.machinery import SourceFileLoader
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
loader = SourceFileLoader('check_perception_env', str(ROOT / 'tools/intel_thor/check_perception_env'))
spec = importlib.util.spec_from_loader(loader.name, loader)
TOOL = importlib.util.module_from_spec(spec)
loader.exec_module(TOOL)


def test_missing_torch_fails_the_gate(monkeypatch, tmp_path, capsys):
    monkeypatch.setitem(__import__('sys').modules, 'torch', None)
    output = tmp_path / 'gate.json'
    assert TOOL.main(['--output', str(output)]) == 1
    report = json.loads(output.read_text())
    assert report['passed'] is False
    assert any(item['check'] == 'torch' and not item['passed'] for item in report['checks'])
    assert json.loads(capsys.readouterr().out) == report
