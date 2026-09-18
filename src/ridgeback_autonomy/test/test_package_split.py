"""Dependency closure and import guards for independent deployment."""
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

SRC = Path(__file__).resolve().parents[2]


def test_localization_manifest_closure_excludes_navigation_simulation_and_rviz():
    forbidden = {'ridgeback_autonomy', 'navigation2', 'nav2_bringup', 'nav2_msgs',
                 'rviz2', 'rviz_2d_overlay_plugins', 'ros_gz_sim',
                 'ridgeback_autonomy_gz', 'ridgeback_autonomy_isaac'}
    visited = set()
    def check(package):
        assert package not in forbidden
        if package in visited:
            return
        visited.add(package)
        manifest = SRC / package / 'package.xml'
        if manifest.exists():
            for child in ET.parse(manifest).getroot():
                if child.tag.endswith('depend') and child.tag != 'test_depend':
                    check(child.text)
    check('ridgeback_localization')
    assert {'ridgeback_common', 'ridgeback_interfaces'} <= visited


def test_display_imports_do_not_load_inference_frameworks():
    subprocess.run([sys.executable, '-c', '''
import importlib.abc, sys
class BlockInference(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'torch', 'transformers', 'ultralytics'}:
            raise AssertionError('display tried to load inference: ' + fullname)
sys.meta_path.insert(0, BlockInference())
import ridgeback_localization.overlay_node
import ridgeback_localization.visualization_node
'''], check=True)


def test_workspace_resolution_supports_nested_install_and_explicit_override(tmp_path, monkeypatch):
    from ridgeback_common.paths import workspace_root
    monkeypatch.delenv('RIDGEBACK_WORKSPACE', raising=False)
    (tmp_path / 'src').mkdir()
    (tmp_path / 'clearpath').mkdir()
    (tmp_path / 'clearpath/robot.yaml').touch()
    assert workspace_root(str(tmp_path / 'artifacts/check/install/pkg/share/pkg')) == tmp_path
    monkeypatch.setenv('RIDGEBACK_WORKSPACE', str(tmp_path / 'elsewhere'))
    assert workspace_root() == tmp_path / 'elsewhere'
