"""Named D455 simulation profiles shared by the Gazebo and Isaac adapters.

The physical driver remains authoritative on hardware.  These profiles are a
reproducible nominal model derived from the D455 RGB datasheet: 1280x800 native
resolution and 90-degree horizontal field of view.  We model square pixels and
centred crops because both simulators need one deterministic pinhole contract.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


DEFAULT_CAMERA_PROFILE = '640x480'
DEFAULT_DEPTH_FIDELITY = 'ideal'
DEPTH_FIDELITY_CHOICES = ('ideal', 'd455')


@dataclass(frozen=True)
class CameraProfile:
    """One centred, square-pixel colour/aligned-depth render profile."""

    name: str
    width: int
    height: int
    focal_length_px: float
    depth_horizontal_fov_deg: float
    minimum_depth_m: float
    fps: float = 30.0

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0

    @property
    def horizontal_fov_rad(self) -> float:
        return 2.0 * math.atan(self.width / (2.0 * self.focal_length_px))

    @property
    def vertical_fov_rad(self) -> float:
        return 2.0 * math.atan(self.height / (2.0 * self.focal_length_px))

    @property
    def depth_focal_length_px(self) -> float:
        """Nominal D455 stereo-imager focal length at this resolution."""
        return self.width / (2.0 * math.tan(
            math.radians(self.depth_horizontal_fov_deg) / 2.0))


# Native D455 RGB nominal: 1280x800, 90 deg horizontal FoV.  That gives a
# 640 px focal length.  The HD profile is a centred 1280x720 vertical crop.
# The VGA profile is a centred 4:3 crop followed by 0.6x scaling, hence 384 px.
CAMERA_PROFILES = {
    # D455 datasheet depth FoV / Min-Z: 75 deg / 0.32 m at 640x480,
    # 87 deg / 0.52 m at 1280x720. Colour/aligned-depth render intrinsics stay
    # on the existing shared profile; these values drive only Isaac's optional
    # stereo-disparity model.
    '640x480': CameraProfile('640x480', 640, 480, 384.0, 75.0, 0.32),
    '1280x720': CameraProfile('1280x720', 1280, 720, 640.0, 87.0, 0.52),
}
CAMERA_PROFILE_CHOICES = tuple(CAMERA_PROFILES)


def resolve_camera_profile(name: str) -> CameraProfile:
    """Return a supported nominal profile or reject the launch value."""
    try:
        return CAMERA_PROFILES[name]
    except KeyError as exc:
        choices = ', '.join(CAMERA_PROFILE_CHOICES)
        raise ValueError(f'unknown camera profile {name!r}; expected one of {choices}') from exc
