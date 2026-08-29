from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class CameraConfig:
    depth_hfov_deg: float
    depth_vfov_deg: float
    pitch_deg: float
    height_m: float


@dataclass
class Detection:
    bbox_xyxy: tuple[int, int, int, int]
    label: str
    score: float
    focus_bbox_xyxy: tuple[int, int, int, int] | None = None
    pointcloud_lateral_m: float | None = None
    pointcloud_forward_m: float | None = None
    pointcloud_distance_m: float | None = None
    projective_ranging_lateral_m: float | None = None
    projective_ranging_forward_m: float | None = None
    projective_ranging_distance_m: float | None = None
    euclidean_reconstruction_lateral_m: float | None = None
    euclidean_reconstruction_forward_m: float | None = None
    euclidean_reconstruction_distance_m: float | None = None
    polar_profiling_lateral_m: float | None = None
    polar_profiling_forward_m: float | None = None
    polar_profiling_distance_m: float | None = None
    projective_ranging_status: int | None = None
    euclidean_reconstruction_status: int | None = None
    polar_profiling_status: int | None = None


@dataclass
class DetectionBatch:
    image_width: int
    image_height: int
    detections: list[Detection] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.detections)

    @property
    def detected(self) -> bool:
        return bool(self.detections)

