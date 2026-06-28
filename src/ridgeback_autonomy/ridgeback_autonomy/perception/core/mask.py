"""Mask component: the common front-end representation.

Every detector front-end emits into a single representation -- an ``H x W``
boolean mask plus a precision tag -- so downstream paths never branch on which
model produced the region. This module covers the ``rect`` mask: the rasterized
bounding box of an object detector (currently OWLv2). The ``tight`` mask (instance
segmentation) is a future front-end that emits into the identical interface.

See ``object_localization_documentation/mask_component.md``. This component is
deliberately standalone: it has no ROS or OpenCV dependency and is not consumed
by the coordinate paths (depth-image, point-cloud, LiDAR) yet.

Extending to the tight front-end (future): instance segmentation yields a
pixel-precise boolean blob rather than a box. Wrap it with
``mask_from_array(blob, MaskPrecision.TIGHT)`` -- the same ``Mask`` type. Box
rasterization (``rasterize_*``) is inherently rectangular and stays the detection
front-end's producer; segmentation uses ``mask_from_array`` instead. Because
``Mask``, ``masked_rgb``, the overlay panel, and every downstream consumer are
precision-agnostic, adding the tight front-end means adding one producer and
changes nothing that already exists.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

from ridgeback_autonomy.common.models import Detection, DetectionBatch


class MaskPrecision(str, Enum):
    """How faithfully a mask's ``True`` region follows the object silhouette.

    ``rect`` is a lossy over-approximation (a solid rectangle that drags in
    background); ``tight`` is pixel-precise. The tag is the marker that keeps a
    rectangular mask from being mistaken for a segmentation mask.
    """

    TIGHT = 'tight'
    RECT = 'rect'


@dataclass(frozen=True)
class Mask:
    """An ``H x W`` boolean selector over the RGB color image, plus its tag.

    ``data[v, u]`` answers "does color pixel ``(u, v)`` belong to the object?".
    Resolution is implicit in ``data.shape``; the mask is bound to the color
    frame's grid (resolution, intrinsics, timestamp) by convention, but only the
    array and the tag are stored here.
    """

    data: np.ndarray
    precision: MaskPrecision

    def __post_init__(self) -> None:
        # Pin the contract for every producer (rect today, tight later): the
        # selector is always a 2D boolean array. Caught here, not downstream.
        if self.data.ndim != 2:
            raise ValueError(f'mask data must be 2D (H, W); got shape {self.data.shape}')
        if self.data.dtype != np.bool_:
            raise ValueError(f'mask data must be boolean; got dtype {self.data.dtype}')

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])


def mask_from_array(data: np.ndarray, precision: MaskPrecision) -> Mask:
    """Wrap a pre-computed ``H x W`` boolean region into the mask interface.

    The integration hook for any front-end that produces a mask directly instead
    of by rasterizing a box -- in particular the future tight (instance
    segmentation) front-end: ``mask_from_array(blob, MaskPrecision.TIGHT)``.
    Binarize before calling; the contract (enforced by ``Mask``) requires a 2D
    boolean array. Prefer this over constructing ``Mask`` directly so all
    producers share one entry point.
    """

    return Mask(data=data, precision=precision)


def _fill_box(data: np.ndarray, bbox_xyxy: tuple[int, int, int, int]) -> None:
    """Fill the half-open rectangle ``[x1, x2) x [y1, y2)`` with ``True``.

    Coordinates are defensively clamped to the grid bounds and a degenerate or
    inverted box (after clamping) fills nothing. Detector boxes are already
    clamped and non-degenerate; the clamp guards standalone callers.
    """

    image_height, image_width = data.shape
    x1, y1, x2, y2 = bbox_xyxy
    x1 = max(0, min(int(x1), image_width))
    x2 = max(0, min(int(x2), image_width))
    y1 = max(0, min(int(y1), image_height))
    y2 = max(0, min(int(y2), image_height))
    if x2 > x1 and y2 > y1:
        data[y1:y2, x1:x2] = True


def rasterize_bbox(
    bbox_xyxy: tuple[int, int, int, int],
    image_height: int,
    image_width: int,
) -> Mask:
    """Rasterize one ``(x1, y1, x2, y2)`` box into a ``rect`` mask."""

    data = np.zeros((image_height, image_width), dtype=bool)
    _fill_box(data, bbox_xyxy)
    return Mask(data=data, precision=MaskPrecision.RECT)


def rasterize_detection(
    detection: Detection,
    image_height: int,
    image_width: int,
) -> Mask:
    """Rasterize a single detection's box into a ``rect`` mask."""

    return rasterize_bbox(detection.bbox_xyxy, image_height, image_width)


def rasterize_batch(batch: DetectionBatch) -> Mask:
    """Rasterize the union of all detection boxes into one ``rect`` mask.

    Allocated at the batch's color-image resolution. An empty batch yields an
    all-``False`` mask.
    """

    data = np.zeros((batch.image_height, batch.image_width), dtype=bool)
    for detection in batch.detections:
        _fill_box(data, detection.bbox_xyxy)
    return Mask(data=data, precision=MaskPrecision.RECT)


def masked_rgb(rgb: np.ndarray, mask: Mask) -> np.ndarray:
    """Return ``rgb`` where ``mask`` is ``True`` and black everywhere else.

    The masked-RGB view of a ``rect`` mask makes background contamination
    directly visible: floor and neighbours caught inside the box appear right
    next to the target. ``rgb`` and ``mask`` must share ``(H, W)``.
    """

    out = np.zeros_like(rgb)
    out[mask.data] = rgb[mask.data]
    return out
