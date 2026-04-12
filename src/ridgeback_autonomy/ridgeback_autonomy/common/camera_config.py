from __future__ import annotations

import json
from pathlib import Path

from ridgeback_autonomy.common.models import CameraConfig


DEFAULT_CAMERA_CONFIG = CameraConfig(
    depth_hfov_deg=87.0,
    depth_vfov_deg=58.0,
    pitch_deg=0.0,
    height_m=0.85,
)


def load_camera_config(config_path: str) -> CameraConfig:
    path = Path(config_path)
    if not path.exists():
        return DEFAULT_CAMERA_CONFIG

    payload = json.loads(path.read_text(encoding='utf-8'))
    camera = payload.get('camera', {})
    return CameraConfig(
        depth_hfov_deg=float(camera.get('depth_hfov_deg', DEFAULT_CAMERA_CONFIG.depth_hfov_deg)),
        depth_vfov_deg=float(camera.get('depth_vfov_deg', DEFAULT_CAMERA_CONFIG.depth_vfov_deg)),
        pitch_deg=float(camera.get('pitch_deg', DEFAULT_CAMERA_CONFIG.pitch_deg)),
        height_m=float(camera.get('height_m', DEFAULT_CAMERA_CONFIG.height_m)),
    )
