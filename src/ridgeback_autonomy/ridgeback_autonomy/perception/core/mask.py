"""Mask component: the common front-end representation.

Every detector front-end emits into a single representation -- an ``H x W``
boolean mask plus a precision tag -- so downstream paths never branch on which
model produced the region. Two front-ends exist today, both emitting this same
``Mask``: ``rect`` -- the rasterized bounding box of an open-vocabulary detector
(``rasterize_*``) -- and ``tight`` -- a pixel-precise segmentation blob wrapped by
``mask_from_array`` (the SlimSAM silhouette in ``core/segmentation.py``).

See ``object_localization_documentation/mask_component.md``. This component is
deliberately standalone -- no ROS or OpenCV dependency -- but its output *is*
consumed by all three coordinate paths (depth-image, point-cloud, LiDAR), which
fork on the precision tag: ``localize_projective_ranging``,
``localize_euclidean_reconstruction`` and ``localize_polar_profiling`` each take a
``Mask`` and read ``.precision`` / ``.data``.

Adding a third front-end (hypothetical) needs one producer and nothing else: a
producer that yields a pixel-precise boolean blob wraps it with
``mask_from_array(blob, <precision>)`` -- the same ``Mask`` type; one that
produces boxes rasterizes via ``rasterize_*``. Because ``Mask``, ``masked_rgb``,
the overlay panel, and every downstream consumer are precision-agnostic, the new
front-end changes nothing that already exists.
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

    None vs empty (the two ways "no pixels" arises, distinct by convention): a
    ``None`` entry in a masks list means *no mask for this detection* -- skip it
    (the no-fallback convention: a failed/rejected segmentation drops the trial,
    never patched over). A constructed ``Mask`` is always a real selector but may
    legitimately be all-``False`` (empty batch, degenerate box); the paths drop
    that naturally via their ``min_valid_pixels`` guard. ``is_empty`` tests the
    all-``False`` case.

    Immutable: ``frozen=True`` locks the fields and ``__post_init__`` marks
    ``data`` read-only, so a shared mask cannot be mutated out from under a
    consumer.
    """

    data: np.ndarray
    precision: MaskPrecision

    def __post_init__(self) -> None:
        if self.data.ndim != 2:
            raise ValueError(f'mask data must be 2D (H, W); got shape {self.data.shape}')
        if self.data.dtype != np.bool_:
            raise ValueError(f'mask data must be boolean; got dtype {self.data.dtype}')
        # Make ``frozen=True`` real for the pixels too. No consumer writes
        # into a mask after construction (producers fill a local array first),
        # so this only forbids writes nobody performs. Setting a numpy array
        # read-only is always permitted -- even for a view of a reused tensor
        # buffer -- since only the reverse (read-only -> writable) is restricted.
        self.data.flags.writeable = False

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def is_empty(self) -> bool:
        """True when the mask selects no pixels (all-``False``)."""

        return not bool(self.data.any())


def mask_from_array(data: np.ndarray, precision: MaskPrecision) -> Mask:
    """Wrap a pre-computed ``H x W`` boolean region into the mask interface.

    The semantic hook for a front-end that produces a mask array *directly* --
    the tight (instance segmentation) front-end: ``mask_from_array(blob,
    MaskPrecision.TIGHT)``. Binarize before calling; the contract (enforced by
    ``Mask``) requires a 2D boolean array. The rect producers legitimately
    create-and-tag via ``rasterize_*`` instead; the real shared chokepoint that
    validates every mask is ``Mask.__post_init__``, not this function.
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
