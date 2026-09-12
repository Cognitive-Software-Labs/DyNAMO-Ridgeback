"""Mask component: the common front-end representation.

Every detector front-end emits into a single representation -- a boolean mask
plus a precision tag -- so downstream paths never branch on which model produced
the region. Two front-ends exist today, both emitting a ``MaskRegion``: ``rect``
-- the bounding box of an open-vocabulary detector (``region_from_bbox``) -- and
``tight`` -- a pixel-precise segmentation blob cropped by ``region_from_blob``
(the SlimSAM silhouette in ``core/segmentation.py``).

Two representations, one meaning. ``MaskRegion`` is the production one: a
rectangular *storage window* holding a boolean payload indexed **locally**,
which is what every per-detection measurement stage consumes. ``Mask`` is the
full-grid form, indexed **globally** (``data[v, u]`` over the whole color
image); it remains the representation for whole-frame artifacts -- the box-union
overlay panel, a mask arriving off the wire -- and the published signature of
the standalone helpers. The two never mix silently: ``as_mask_region`` and
``MaskRegion.to_mask`` are the only crossings, and production never converts a
region back to full-grid between stages.

See ``docs/target_localization/mask_representation.md``. This component is
deliberately standalone -- no ROS or OpenCV dependency -- but its output *is*
consumed by all three coordinate paths (depth-image, point-cloud, LiDAR). The
two depth paths fork on the precision tag; polar profiling does not, because
parallax survives even a pixel-precise mask. Each path has a region-native entry
point for production (``localize_prepared_projective_ranging``,
``localize_prepared_euclidean_reconstruction``,
``localize_projected_polar_profiling``) over the same implementation as its
full-grid twin.

Adding a third front-end (hypothetical) needs one producer and nothing else: a
producer that yields a pixel-precise boolean blob wraps it with
``region_from_blob(blob, <precision>)``; one that produces boxes calls
``region_from_bbox``. Because ``MaskRegion``, ``masked_rgb``, the overlay panel,
and every downstream consumer are precision-agnostic, the new front-end changes
nothing that already exists.
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

    The full-grid constructor, kept for callers that genuinely hold a whole-frame
    array -- a mask decoded off the wire, a test fixture. A *producer* wants
    ``region_from_blob`` instead: it crops to the blob's own extent rather than
    carrying the empty remainder of the frame through every stage. Binarize
    before calling; the contract (enforced by ``Mask``) requires a 2D boolean
    array.
    """

    return Mask(data=data, precision=precision)


def clamp_box(
    bbox_xyxy: tuple[int, int, int, int],
    image_height: int,
    image_width: int,
) -> tuple[int, int, int, int]:
    """Detector box -> the half-open pixel bounds ``[x1, x2) x [y1, y2)``.

    Truncates to integers and clamps to the grid; a degenerate or inverted box
    (after clamping) collapses to an empty ``(x, y, x, y)``. Detector boxes are
    already clamped and non-degenerate; the clamp guards standalone callers.

    One owner for this rule, because both mask representations depend on it
    agreeing: the full-grid rasterizers fill exactly these bounds and
    ``region_from_bbox`` sizes its storage window to them.
    """

    x1, y1, x2, y2 = bbox_xyxy
    x1 = max(0, min(int(x1), image_width))
    x2 = max(0, min(int(x2), image_width))
    y1 = max(0, min(int(y1), image_height))
    y2 = max(0, min(int(y2), image_height))
    if x2 <= x1 or y2 <= y1:
        return x1, y1, x1, y1
    return x1, y1, x2, y2


@dataclass(frozen=True)
class MaskRegion:
    """A boolean selector stored in its own rectangular window of the color grid.

    ``data[local_row, local_col]`` answers "does color pixel
    ``(origin_u + local_col, origin_v + local_row)`` belong to the object?".
    Outside the window membership is implicitly ``False``, so the window is
    storage, not meaning: a region and the full-grid ``Mask`` it materializes to
    select exactly the same pixels.

    A window is *not* a downgrade of a silhouette to a rectangle. ``precision``
    is carried, never inferred: a ``TIGHT`` region whose crop happens to be
    all-``True`` (a genuinely rectangular silhouette) is still ``TIGHT``, and the
    estimators still apply the tight foreground policy to it.

    Bounds are half-open and validated against the full grid, so a region can
    always index the frame it was built for. ``image_width`` / ``image_height``
    are the **full grid**; the window's own size is ``roi_shape``. There is
    deliberately no bare ``width`` / ``height`` here -- on ``Mask`` those name the
    whole image, and a consumer that read them off a region would size a
    full-frame buffer to a crop.

    None vs empty (the two ways "no pixels" arises, distinct by convention): a
    ``None`` entry in a masks list means *no mask for this detection* -- skip it
    (the no-fallback convention: a failed/rejected segmentation drops the trial,
    never patched over). A constructed ``MaskRegion`` is always a real selector
    but may legitimately select nothing -- ``empty_region`` (a ``(0, 0)``
    payload, from a degenerate box) or an all-``False`` payload; the paths drop
    those naturally via their ``min_valid_pixels`` guard. ``is_empty`` tests for
    "selects no pixel", covering both.

    Immutable: ``frozen=True`` locks the fields and ``__post_init__`` marks
    ``data`` read-only, so a shared region cannot be mutated out from under a
    consumer.
    """

    data: np.ndarray
    origin_u: int
    origin_v: int
    image_width: int
    image_height: int
    precision: MaskPrecision

    def __post_init__(self) -> None:
        if self.data.ndim != 2:
            raise ValueError(
                f'region data must be 2D (rows, cols); got shape {self.data.shape}')
        if self.data.dtype != np.bool_:
            raise ValueError(f'region data must be boolean; got dtype {self.data.dtype}')
        for name in ('origin_u', 'origin_v', 'image_width', 'image_height'):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, np.integer)):
                raise ValueError(f'{name} must be an integer; got {value!r}')
            object.__setattr__(self, name, int(value))
        if self.origin_u < 0 or self.origin_v < 0:
            raise ValueError(
                f'region origin must be non-negative; got '
                f'({self.origin_u}, {self.origin_v})')
        roi_height, roi_width = self.data.shape
        if (self.origin_u + roi_width > self.image_width
                or self.origin_v + roi_height > self.image_height):
            raise ValueError(
                f'region window ({self.origin_v}:{self.origin_v + roi_height}, '
                f'{self.origin_u}:{self.origin_u + roi_width}) does not fit the '
                f'({self.image_height}, {self.image_width}) grid')
        # Make ``frozen=True`` real for the pixels too: producers fill a local
        # array and hand over ownership, so this only forbids writes nobody
        # performs. Setting a numpy array read-only is always permitted -- even
        # for a view -- since only the reverse is restricted.
        self.data.flags.writeable = False

    @property
    def roi_shape(self) -> tuple[int, int]:
        """``(rows, cols)`` of the stored window."""

        return (int(self.data.shape[0]), int(self.data.shape[1]))

    @property
    def image_shape(self) -> tuple[int, int]:
        """``(rows, cols)`` of the full color grid this region indexes into."""

        return (self.image_height, self.image_width)

    @property
    def covers_full_grid(self) -> bool:
        return self.origin_u == 0 and self.origin_v == 0 and self.roi_shape == self.image_shape

    @property
    def is_empty(self) -> bool:
        """True when the region selects no pixel at all."""

        return not bool(self.data.any())

    def slice_image(self, image: np.ndarray) -> np.ndarray:
        """The window of a full-grid array, aligned with ``data``.

        The array must be on the full grid; a caller handing over an
        already-cropped array would silently offset every subsequent
        coordinate, so the shape is checked rather than trusted. A region that
        covers the whole grid returns the array itself: the full window *is* the
        array, and handing back the original keeps the standalone estimator
        entry points passing a caller's own precomputed array through to a
        custom isolation callable unchanged.
        """

        image = np.asarray(image)
        if image.shape[:2] != self.image_shape:
            raise ValueError(
                f'expected a full-grid array of shape {self.image_shape}; '
                f'got {image.shape[:2]}')
        if self.covers_full_grid:
            return image
        roi_height, roi_width = self.roi_shape
        return image[
            self.origin_v:self.origin_v + roi_height,
            self.origin_u:self.origin_u + roi_width,
        ]

    def global_pixels(
        self,
        local_rows: np.ndarray,
        local_cols: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Window-local ``(rows, cols)`` -> their coordinates on the full grid.

        The one place the origin is added. Deprojection and the representative
        pixel are defined against the color intrinsics, so every coordinate
        leaving the window passes through here exactly once.

        A window at the origin returns its inputs: adding zero would allocate
        two more index arrays as long as the selection to hold the values
        already in hand, which on a full-image mask is the largest allocation in
        the call.
        """

        local_rows = np.asarray(local_rows)
        local_cols = np.asarray(local_cols)
        if self.origin_u == 0 and self.origin_v == 0:
            return local_rows, local_cols
        return local_rows + self.origin_v, local_cols + self.origin_u

    def contains_pixels(self, u_px, v_px) -> np.ndarray:
        """Membership of full-grid pixel coordinates, vectorized.

        Coordinates outside the window are ``False`` rather than an index into
        the wrong pixel: the bounds test runs *before* the local index, so a
        coordinate left of ``origin_u`` cannot wrap around to the far edge of
        the payload.
        """

        local_u = np.asarray(u_px) - self.origin_u
        local_v = np.asarray(v_px) - self.origin_v
        roi_height, roi_width = self.roi_shape
        inside = (
            (local_u >= 0) & (local_u < roi_width)
            & (local_v >= 0) & (local_v < roi_height)
        )
        selected = np.zeros(inside.shape, dtype=bool)
        selected[inside] = self.data[local_v[inside], local_u[inside]]
        return selected

    def blit_into(self, destination: np.ndarray, value) -> None:
        """Write ``value`` into a full-grid array wherever this region selects.

        The composition primitive: several regions blit into one output buffer,
        which is how a per-detection union is built without materializing a
        full-size mask per detection.
        """

        window = self.slice_image(destination)
        window[self.data] = value

    def to_full_array(self) -> np.ndarray:
        """Materialize the selector on the full grid.

        Explicit because it is the expensive direction. For compatibility
        boundaries and whole-frame output only -- production keeps regions
        region-shaped between stages.
        """

        full = np.zeros(self.image_shape, dtype=bool)
        self.blit_into(full, True)
        return full

    def to_mask(self) -> Mask:
        """Materialize this region as a full-grid ``Mask``, precision retained."""

        return Mask(data=self.to_full_array(), precision=self.precision)


def region_from_bbox(
    bbox_xyxy: tuple[int, int, int, int],
    image_height: int,
    image_width: int,
) -> MaskRegion:
    """One ``(x1, y1, x2, y2)`` detector box as an all-``True`` ``rect`` region.

    The box *is* the window, so the payload carries no ``False`` at all. Shares
    ``clamp_box`` with the full-grid rasterizers, so the two agree pixel for
    pixel; a box that clamps to nothing yields the canonical empty region rather
    than a negative-sized window.
    """

    x1, y1, x2, y2 = clamp_box(bbox_xyxy, image_height, image_width)
    if x2 <= x1 or y2 <= y1:
        return empty_region(image_height, image_width, MaskPrecision.RECT)
    return MaskRegion(
        data=np.ones((y2 - y1, x2 - x1), dtype=bool),
        origin_u=x1,
        origin_v=y1,
        image_width=image_width,
        image_height=image_height,
        precision=MaskPrecision.RECT,
    )


def region_from_detection(
    detection: Detection,
    image_height: int,
    image_width: int,
) -> MaskRegion:
    """One detection's box as a ``rect`` region."""

    return region_from_bbox(detection.bbox_xyxy, image_height, image_width)


def region_from_blob(blob: np.ndarray, precision: MaskPrecision) -> MaskRegion:
    """Crop a full-grid boolean blob to its own extent and take ownership.

    The window is the blob's **actual nonzero extent**, not the box that
    prompted it: a padded prompt legitimately returns silhouette pixels outside
    the detector box, and cropping to the box would delete them. The crop is
    copied, so the region does not pin the model's full-frame output alive as
    its numpy base, and no external alias can write through it afterwards.
    Every selected pixel survives, disconnected components and holes included.
    """

    blob = np.asarray(blob)
    if blob.ndim != 2:
        raise ValueError(f'blob must be 2D (H, W); got shape {blob.shape}')
    if blob.dtype != np.bool_:
        raise ValueError(f'blob must be boolean; got dtype {blob.dtype}')

    image_height, image_width = (int(size) for size in blob.shape)
    rows = np.flatnonzero(blob.any(axis=1))
    if rows.size == 0:
        return empty_region(image_height, image_width, precision)
    cols = np.flatnonzero(blob.any(axis=0))
    origin_v, origin_u = int(rows[0]), int(cols[0])
    return MaskRegion(
        data=np.array(blob[origin_v:int(rows[-1]) + 1, origin_u:int(cols[-1]) + 1]),
        origin_u=origin_u,
        origin_v=origin_v,
        image_width=image_width,
        image_height=image_height,
        precision=precision,
    )


def empty_region(
    image_height: int,
    image_width: int,
    precision: MaskPrecision,
) -> MaskRegion:
    """The canonical "selects nothing" region: a ``(0, 0)`` window at the origin.

    A valid selector with zero pixels, **not** ``None`` -- it still names its
    grid and its precision, and the estimators reject it through their ordinary
    minimum-count guards with the ordinary miss reasons.
    """

    return MaskRegion(
        data=np.zeros((0, 0), dtype=bool),
        origin_u=0,
        origin_v=0,
        image_width=image_width,
        image_height=image_height,
        precision=precision,
    )


def as_mask_region(mask) -> MaskRegion:
    """Accept either representation, return a region -- the compatibility crossing.

    A ``MaskRegion`` passes through. A full-grid ``Mask`` is wrapped in a
    full-grid window over the *same* array: no copy, no materialization, and the
    read-only guarantee is already in force. This is what lets one membership
    and one reduction implementation serve both the standalone full-frame entry
    points and the ROI-native production path.
    """

    if isinstance(mask, MaskRegion):
        return mask
    return MaskRegion(
        data=mask.data,
        origin_u=0,
        origin_v=0,
        image_width=mask.width,
        image_height=mask.height,
        precision=mask.precision,
    )


def _fill_box(data: np.ndarray, bbox_xyxy: tuple[int, int, int, int]) -> None:
    """Fill one clamped half-open rectangle of a full-grid array with ``True``."""

    image_height, image_width = data.shape
    x1, y1, x2, y2 = clamp_box(bbox_xyxy, image_height, image_width)
    data[y1:y2, x1:x2] = True


def rasterize_bbox(
    bbox_xyxy: tuple[int, int, int, int],
    image_height: int,
    image_width: int,
) -> Mask:
    """Rasterize one ``(x1, y1, x2, y2)`` box into a full-grid ``rect`` mask."""

    return region_from_bbox(bbox_xyxy, image_height, image_width).to_mask()


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


def masked_rgb(rgb: np.ndarray, mask) -> np.ndarray:
    """Return ``rgb`` where the mask selects and black everywhere else.

    The masked-RGB view of a ``rect`` mask makes background contamination
    directly visible: floor and neighbours caught inside the box appear right
    next to the target. ``rgb`` is on the full color grid; ``mask`` is either
    representation, copied through the same windowed membership so a region
    never has to materialize to be rendered.
    """

    region = as_mask_region(mask)
    out = np.zeros_like(rgb)
    region.slice_image(out)[region.data] = region.slice_image(rgb)[region.data]
    return out
