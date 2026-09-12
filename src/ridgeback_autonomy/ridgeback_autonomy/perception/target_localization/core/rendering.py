from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    DEPTH_PATH_ESTIMATORS,
    ESTIMATOR_LABELS,
    MASK_GATE_SILHOUETTE,
    PUBLIC_ESTIMATOR_ORDER,
)
from ridgeback_autonomy.common.models import DetectionBatch
from ridgeback_autonomy.perception.target_localization.core.pointcloud_ranging import focus_bbox
from ridgeback_autonomy.perception.target_localization.core.mask import (
    MaskPrecision,
    mask_from_array,
    masked_rgb,
    rasterize_batch,
)


# Panel kinds. A run renders only the ones its selected estimators need, so the
# view matches the config instead of a fixed grid.
PANEL_RGB = 'rgb'
PANEL_ALIGNED_DEPTH = 'aligned_depth'
PANEL_BOX_MASK = 'box_mask'
PANEL_SILHOUETTE = 'silhouette'
PANEL_LIDAR = 'lidar'

# Panels per row when packing the grid. Three suits a roughly square window;
# a wide, short target (the RViz strip) wants them all on one row instead.
PANEL_MAX_COLS_DEFAULT = 3


# Scan-point colours, BGR. The first two are the RViz ray colours from
# ``common/markers.py`` (COLOR_USED / COLOR_DROPPED) converted to BGR, so the 2D
# panel and the 3D layers name the same beam the same way and can be compared
# directly. The third is for beams whose state is unknown.
SCAN_COLOR_USED = (0, 166, 255)
SCAN_COLOR_DROPPED = (148, 140, 140)
SCAN_COLOR_UNKNOWN = (90, 90, 90)


@dataclass(frozen=True)
class PanelSpec:
    kind: str
    title: str


@dataclass(frozen=True)
class ScanHighlight:
    """Per-beam booleans over one scan, as the measuring node reduced it.

    Both index the scan array the beams came from, so they mean nothing beside a
    different scan. ``common/messages.py:polar_beam_booleans`` decodes them from
    a ``PolarBeams`` message and refuses when the scan it is handed is not the
    one the indices were recorded against.
    """

    used: np.ndarray
    dropped: np.ndarray


def select_panels(estimators, depth_source: str, mask_gate: str) -> list[PanelSpec]:
    """The panels a run needs, in display order, driven by its estimator set.

    RGB is always the anchor. Each further panel appears only when an estimator
    that consumes its data is selected: the aligned-depth panel and the mask
    (box or silhouette) panel are tied to the depth mask paths, and the LiDAR /
    polar panel to polar profiling. The mask gate picks box vs silhouette.
    """

    selected = set(estimators)
    panels: list[PanelSpec] = [PanelSpec(PANEL_RGB, 'RGB Detection')]

    if selected & DEPTH_PATH_ESTIMATORS:
        panels.append(PanelSpec(PANEL_ALIGNED_DEPTH, f'Aligned Depth ({depth_source})'))
        if mask_gate == MASK_GATE_SILHOUETTE:
            panels.append(PanelSpec(PANEL_SILHOUETTE, 'Silhouette Mask'))
        else:
            panels.append(PanelSpec(PANEL_BOX_MASK, 'Box Mask'))
    if 'polar_profiling' in selected:
        panels.append(PanelSpec(PANEL_LIDAR, ESTIMATOR_LABELS['polar_profiling']))

    return panels


def pack_panels(panels: list[np.ndarray], max_cols: int = PANEL_MAX_COLS_DEFAULT) -> np.ndarray:
    """Tile equal-size panels into a tidy grid, black-padding the last row.

    ``ceil(N / cols)`` rows by ``cols = min(max_cols, N)`` columns, so 1..N
    panels pack without a standing empty cell except the last row's remainder.
    """

    if not panels:
        raise ValueError('pack_panels needs at least one panel')

    count = len(panels)
    cols = min(max_cols, count)
    rows = math.ceil(count / cols)
    blank = np.zeros_like(panels[0])
    cells = list(panels) + [blank] * (rows * cols - count)
    row_images = [np.hstack(cells[row * cols:(row + 1) * cols]) for row in range(rows)]
    return np.vstack(row_images)


def active_label_lines(detection, estimators) -> list[str]:
    """One label line per selected estimator, in ``PUBLIC_ESTIMATOR_ORDER``.

    Every registered estimator reports lateral/forward/distance -- the invariant
    ``ESTIMATOR_POSITION_ATTRS`` carries -- so every line is a position line. A
    missing field reads as ``NA`` (no estimate for this frame).
    """

    selected = set(estimators)
    return [
        format_position_line(
            ESTIMATOR_LABELS[estimator],
            getattr(detection, f'{estimator}_lateral_m', None),
            getattr(detection, f'{estimator}_forward_m', None),
            getattr(detection, f'{estimator}_distance_m', None),
        )
        for estimator in PUBLIC_ESTIMATOR_ORDER
        if estimator in selected
    ]


def format_position_line(
    prefix: str,
    lateral_m: float | None,
    forward_m: float | None,
    distance_m: float | None,
) -> str:
    if lateral_m is None or forward_m is None or distance_m is None:
        return f'{prefix} d=NA'
    return f'{prefix} x={lateral_m:+.2f} z={forward_m:+.2f} d={distance_m:.2f}m'


# Ground-truth reference line: rendered gold so it reads as the benchmark's
# reference, never as one of the estimates.
TRUTH_LABEL_COLOR = (0, 215, 255)


def truth_label_line(truth) -> str:
    """Label line for a ``(lateral_m, forward_m, distance_m)`` ground truth."""

    lateral_m, forward_m, distance_m = truth
    return format_position_line('Truth', lateral_m, forward_m, distance_m)


class RgbdOverlayRenderer:
    """Assembles the per-frame overlay grid for the configured estimator set."""

    def __init__(
        self,
        depth_max_meters: float,
        estimators=PUBLIC_ESTIMATOR_ORDER,
        depth_source: str = 'stereoscopic',
        mask_gate: str = 'box',
        max_cols: int = PANEL_MAX_COLS_DEFAULT,
        rgb_panel_labels: bool = True,
    ) -> None:
        self.depth_max_meters = depth_max_meters
        self.estimators = tuple(estimators)
        self.depth_source = depth_source
        self.max_cols = max_cols
        self.rgb_panel_labels = rgb_panel_labels
        self.panels = select_panels(self.estimators, depth_source, mask_gate)

    def render(
        self,
        frame: np.ndarray,
        batch: DetectionBatch,
        *,
        aligned_depth_meters: np.ndarray | None = None,
        published_mask: np.ndarray | None = None,
        scan_uv: np.ndarray | None = None,
        scan_in_view: np.ndarray | None = None,
        scan_highlight: ScanHighlight | None = None,
        truth: tuple[float, float, float] | None = None,
    ) -> np.ndarray:
        images = [
            self.build_panel(
                spec, frame, batch,
                aligned_depth_meters=aligned_depth_meters,
                published_mask=published_mask,
                scan_uv=scan_uv,
                scan_in_view=scan_in_view,
                scan_highlight=scan_highlight,
                truth=truth,
            )
            for spec in self.panels
        ]
        return pack_panels(images, max_cols=self.max_cols)

    def build_panel(
        self,
        spec: PanelSpec,
        frame: np.ndarray,
        batch: DetectionBatch,
        *,
        aligned_depth_meters,
        published_mask,
        scan_uv,
        scan_in_view,
        scan_highlight,
        truth=None,
    ) -> np.ndarray:
        if spec.kind == PANEL_RGB:
            panel = frame.copy()
            self.annotate_detections(
                panel, batch, draw_labels=self.rgb_panel_labels, truth=truth)
        elif spec.kind == PANEL_ALIGNED_DEPTH:
            panel = self.make_depth_panel(frame.shape[:2], aligned_depth_meters)
            self.annotate_detections(panel, batch, draw_labels=False)
        elif spec.kind == PANEL_BOX_MASK:
            panel = self.make_box_mask_panel(frame, batch)
        elif spec.kind == PANEL_SILHOUETTE:
            panel = self.make_silhouette_mask_panel(frame, published_mask)
        elif spec.kind == PANEL_LIDAR:
            panel = self.make_lidar_panel(frame, batch, scan_uv, scan_in_view, scan_highlight)
        else:
            panel = np.zeros_like(frame)

        self.draw_panel_title(panel, spec.title)
        if not batch.detected and spec.kind in (PANEL_RGB, PANEL_ALIGNED_DEPTH):
            cv2.putText(panel, 'No target detected', (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        return panel

    def annotate_detections(
        self,
        panel: np.ndarray,
        batch: DetectionBatch,
        *,
        draw_labels: bool,
        truth: tuple[float, float, float] | None = None,
    ) -> None:
        if not batch.detected:
            return
        for index, detection in enumerate(batch.detections):
            x1, y1, x2, y2 = detection.bbox_xyxy
            cv2.rectangle(panel, (x1, y1), (x2, y2), (0, 255, 0), 2)
            focus = detection.focus_bbox_xyxy or focus_bbox(detection.bbox_xyxy)
            if focus is not None:
                fx1, fy1, fx2, fy2 = focus
                cv2.rectangle(panel, (fx1, fy1), (fx2, fy2), (0, 200, 255), 2)
            if draw_labels:
                lines = [f'Target #{index + 1} ({detection.score:.0%})']
                lines.extend(active_label_lines(detection, self.estimators))
                line_colors = None
                if truth is not None:
                    line_colors = {len(lines): TRUTH_LABEL_COLOR}
                    lines.append(truth_label_line(truth))
                self.draw_label_block(
                    panel, x1, y1, lines, (0, 255, 0), line_colors=line_colors)

    def make_lidar_panel(
        self,
        frame: np.ndarray,
        batch: DetectionBatch,
        scan_uv: np.ndarray | None,
        scan_in_view: np.ndarray | None,
        scan_highlight: ScanHighlight | None,
    ) -> np.ndarray:
        """The colour frame with the scan drawn on it, as the estimator saw it.

        The panel decides nothing: which beams were used and which were dropped
        arrive already reduced from the node that published the measurement, so
        this cannot show a different band than the run it is labelling.
        """

        panel = frame.copy()
        self.annotate_detections(panel, batch, draw_labels=False)
        draw_scan_points(panel, scan_uv, scan_in_view, scan_highlight)
        return panel

    def stack_panel_grid(self, top_panels, bottom_panels) -> np.ndarray:
        """Retained for callers still passing explicit rows: pack them together."""

        return pack_panels(list(top_panels) + list(bottom_panels), max_cols=len(top_panels))

    def normalize_depth(self, image: np.ndarray) -> np.ndarray:
        valid = np.isfinite(image) & (image > 0.0)
        if self.depth_max_meters > 0.0:
            valid &= image <= self.depth_max_meters
        if not np.any(valid):
            return np.zeros(image.shape, dtype=np.uint8)

        clipped = image.copy()
        clipped[~valid] = 0.0
        max_value = self.depth_max_meters if self.depth_max_meters > 0.0 else float(np.max(clipped[valid]))
        if max_value <= 0.0:
            return np.zeros(image.shape, dtype=np.uint8)

        normalized = np.clip(clipped / max_value, 0.0, 1.0)
        return (normalized * 255.0).astype(np.uint8)

    def make_depth_panel(self, color_shape, depth_meters: np.ndarray | None) -> np.ndarray:
        color_h, color_w = color_shape
        if depth_meters is None:
            return np.zeros((color_h, color_w, 3), dtype=np.uint8)

        normalized = self.normalize_depth(depth_meters)
        panel = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        if panel.shape[:2] != (color_h, color_w):
            panel = cv2.resize(panel, (color_w, color_h), interpolation=cv2.INTER_NEAREST)
        return panel

    def make_box_mask_panel(self, frame: np.ndarray, batch: DetectionBatch) -> np.ndarray:
        mask = rasterize_batch(batch)
        if mask.data.shape == frame.shape[:2]:
            panel = masked_rgb(frame, mask)
        else:
            panel = np.zeros_like(frame)

        if not batch.detected:
            cv2.putText(panel, 'No target detected', (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        return panel

    def make_silhouette_mask_panel(
        self,
        frame: np.ndarray,
        published_mask: np.ndarray | None,
    ) -> np.ndarray:
        if published_mask is not None and published_mask.shape == frame.shape[:2]:
            mask = mask_from_array(published_mask.astype(bool), MaskPrecision.TIGHT)
            panel = masked_rgb(frame, mask)
        else:
            panel = np.zeros_like(frame)
            cv2.putText(panel, 'No silhouette mask', (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
        return panel

    def draw_panel_title(self, image: np.ndarray, title: str) -> None:
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.8
        thickness = 2
        (text_width, text_height), _ = cv2.getTextSize(title, font, scale, thickness)
        left = 10
        top = 10
        right = left + text_width + 20
        bottom = top + text_height + 18
        cv2.rectangle(image, (left, top), (right, bottom), (0, 0, 0), -1)
        cv2.putText(image, title, (left + 10, bottom - 10), font,
                    scale, (255, 255, 255), thickness, cv2.LINE_AA)

    def draw_label_block(self, image: np.ndarray, x: int, y: int,
                         lines: list[str], color,
                         line_colors: dict[int, tuple] | None = None) -> None:
        font = cv2.FONT_HERSHEY_DUPLEX
        scale = 0.68
        thickness = 2
        padding_x = 10
        padding_y = 8
        line_gap = 8

        sizes = [cv2.getTextSize(line, font, scale, thickness)[0] for line in lines]
        block_width = max(size[0] for size in sizes) + padding_x * 2
        line_height = max(size[1] for size in sizes) + line_gap
        block_height = line_height * len(lines) + padding_y * 2

        top_y = y - block_height - 4
        if top_y < 4:
            top_y = min(y + 4, image.shape[0] - block_height - 4)

        left_x = max(4, min(x, image.shape[1] - block_width - 4))
        top_y = max(4, top_y)

        box_top_left = (left_x, top_y)
        box_bottom_right = (left_x + block_width, top_y + block_height)

        overlay = image.copy()
        cv2.rectangle(overlay, box_top_left, box_bottom_right, (12, 12, 12), -1)
        cv2.addWeighted(overlay, 0.86, image, 0.14, 0.0, image)
        cv2.rectangle(image, box_top_left, box_bottom_right, color, 2)

        text_y = top_y + padding_y + line_height - line_gap
        for line_index, line in enumerate(lines):
            text_color = (line_colors or {}).get(
                line_index, color if line_index == 0 else (245, 245, 245))
            origin = (left_x + padding_x, text_y)
            cv2.putText(image, line, origin, font, scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
            cv2.putText(image, line, origin, font, scale, text_color, thickness, cv2.LINE_AA)
            text_y += line_height


def draw_scan_points(
    panel: np.ndarray,
    scan_uv: np.ndarray | None,
    scan_in_view: np.ndarray | None,
    highlight: ScanHighlight | None,
) -> None:
    """Draw the projected scan, coloured by what the estimator did with it.

    The two states are the ones the 3D ray layers draw, in the same colours, so
    the panel and the markers can be read against each other beam for beam: the
    survivors the estimate medians over, and the beams the mask selected but the
    range segmentation discarded. Drawing only the survivors would make a frame
    where the estimator threw the robot away look like a frame where nothing was
    there.

    ``highlight`` of ``None`` means the selection could not be trusted for this
    scan -- the message named a different-sized array -- so the in-view beams are
    drawn plain. That says "the scan is here, which beams were used is unknown",
    which is the honest picture; asserting a state would be a misaligned
    highlight, and drawing nothing would read as a missing scan.
    """

    if scan_uv is None:
        return
    scan_uv = np.asarray(scan_uv, dtype=np.float64)
    if highlight is None:
        if scan_in_view is None:
            return
        layers = ((np.asarray(scan_in_view, dtype=bool), SCAN_COLOR_UNKNOWN, 2),)
    else:
        layers = (
            (np.asarray(highlight.dropped, dtype=bool), SCAN_COLOR_DROPPED, 2),
            (np.asarray(highlight.used, dtype=bool), SCAN_COLOR_USED, 3),
        )

    height, width = panel.shape[:2]
    for beams, color, radius in layers:
        for index in np.flatnonzero(beams):
            u_px = int(round(float(scan_uv[index, 0])))
            v_px = int(round(float(scan_uv[index, 1])))
            if not (0 <= u_px < width and 0 <= v_px < height):
                continue
            cv2.circle(panel, (u_px, v_px), radius, color, -1, cv2.LINE_AA)
