import math

import pytest

from ridgeback_autonomy.common.camera_profiles import (
    CAMERA_PROFILE_CHOICES,
    DEFAULT_CAMERA_PROFILE,
    resolve_camera_profile,
)


def test_nominal_d455_profiles_are_pinned() -> None:
    assert DEFAULT_CAMERA_PROFILE == '640x480'
    assert CAMERA_PROFILE_CHOICES == ('640x480', '1280x720')

    vga = resolve_camera_profile('640x480')
    assert (vga.width, vga.height, vga.focal_length_px, vga.cx, vga.cy) == (
        640, 480, 384.0, 320.0, 240.0)
    assert math.degrees(vga.horizontal_fov_rad) == pytest.approx(79.611, abs=0.001)
    assert math.degrees(vga.vertical_fov_rad) == pytest.approx(64.011, abs=0.001)

    hd = resolve_camera_profile('1280x720')
    assert (hd.width, hd.height, hd.focal_length_px, hd.cx, hd.cy) == (
        1280, 720, 640.0, 640.0, 360.0)
    assert math.degrees(hd.horizontal_fov_rad) == pytest.approx(90.0, abs=0.001)
    assert math.degrees(hd.vertical_fov_rad) == pytest.approx(58.716, abs=0.001)


def test_unknown_profile_is_rejected() -> None:
    with pytest.raises(ValueError, match='unknown camera profile'):
        resolve_camera_profile('1920x1080')
