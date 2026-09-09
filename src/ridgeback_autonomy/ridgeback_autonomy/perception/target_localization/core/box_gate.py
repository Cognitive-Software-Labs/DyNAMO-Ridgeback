"""Pure detector-box acceptance policy shared by live and replay paths."""

from __future__ import annotations


# A detector box covering more than this fraction of the frame is almost always
# a failure (OWLv2 occasionally boxes the whole scene at close range); masking
# with it isolates the background wall and poisons every path.
MAX_BOX_FRAME_FRACTION = 0.60


def box_within_frame_fraction(
    bbox_xyxy,
    image_height: int,
    image_width: int,
    max_fraction: float = MAX_BOX_FRAME_FRACTION,
) -> bool:
    """True if the detector box covers at most ``max_fraction`` of the frame."""

    frame_area = float(image_height) * float(image_width)
    if frame_area <= 0.0:
        return False
    x1, y1, x2, y2 = bbox_xyxy
    box_area = float(max(0, x2 - x1)) * float(max(0, y2 - y1))
    return box_area <= max_fraction * frame_area
