"""In-process track types shared by the leader-follow modules (not ROS messages)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

# Mirrors leader_follow_interfaces/LeaderState: only LOCKED may drive motion.
LeaderStateName = Literal["SEARCHING", "LOCKED", "UNCERTAIN", "LOST"]


@dataclass(frozen=True)
class Track:
    """One tracked person as the leader lock sees it."""
    track_id: int
    score: float            # detection confidence
    feature: np.ndarray     # L2-normalised appearance embedding; empty when unseen by the camera


@dataclass(frozen=True)
class CameraTrack(Track):
    """A person tracked in the camera image."""
    bbox: np.ndarray = None  # xyxy pixels, float32
