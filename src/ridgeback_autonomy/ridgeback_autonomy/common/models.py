from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


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
    rgb_lateral_m: float | None = None
    rgb_forward_m: float | None = None
    rgb_distance_m: float | None = None
    sensor_depth_distance_m: float | None = None
    mono_depth_distance_m: float | None = None
    lidar_lateral_m: float | None = None
    lidar_forward_m: float | None = None
    lidar_distance_m: float | None = None
    pointcloud_lateral_m: float | None = None
    pointcloud_forward_m: float | None = None
    pointcloud_distance_m: float | None = None


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


@dataclass(frozen=True)
class LidarScanPoints:
    forward_m: np.ndarray
    lateral_m: np.ndarray
    planar_distance_m: np.ndarray
    bearing_rad: np.ndarray
    valid: np.ndarray

