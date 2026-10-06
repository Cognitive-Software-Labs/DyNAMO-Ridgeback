"""Republisher commands the Thor decoder starts for a compression candidate."""
from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
loader = SourceFileLoader('thor_decoder', str(ROOT / 'tools/intel_thor/thor_decoder'))
spec = importlib.util.spec_from_loader(loader.name, loader)
TOOL = importlib.util.module_from_spec(spec)
loader.exec_module(TOOL)


def test_each_compressed_stream_gets_one_republisher_on_a_separate_topic():
    color, depth = TOOL.commands('/r100_0160/', 'ffmpeg', 'zstd')
    assert color[0] == TOOL.REPUBLISH
    assert 'in_transport:=ffmpeg' in color and 'out_transport:=raw' in color
    assert 'in/ffmpeg:=/r100_0160/sensors/camera_0/color/image/ffmpeg' in color
    assert 'out:=/r100_0160/sensors/camera_0/thor/color/image' in color
    assert 'in/zstd:=/r100_0160/sensors/camera_0/depth/image/zstd' in depth
    assert 'out:=/r100_0160/sensors/camera_0/thor/depth/image' in depth
    assert '__node:=thor_color_decoder' in color and '__node:=thor_depth_decoder' in depth


def test_raw_streams_need_no_decoder():
    assert [command[command.index('-r') + 1] for command in TOOL.commands('r100_0160', 'raw', 'compressed')] == [
        '__node:=thor_depth_decoder']
    with pytest.raises(SystemExit):
        TOOL.main(['--namespace', 'r100_0160'])


def test_transport_choices_match_the_contract_check():
    contract_loader = SourceFileLoader('camera_contract_check', str(ROOT / 'tools/camera_contract_check'))
    contract_spec = importlib.util.spec_from_loader(contract_loader.name, contract_loader)
    contract = importlib.util.module_from_spec(contract_spec)
    contract_loader.exec_module(contract)
    assert TOOL.COLOR_TRANSPORTS == contract.COLOR_TRANSPORTS
    assert TOOL.DEPTH_TRANSPORTS == contract.DEPTH_TRANSPORTS
