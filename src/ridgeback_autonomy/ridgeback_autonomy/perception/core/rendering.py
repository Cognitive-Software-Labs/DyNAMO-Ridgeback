from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from ridgeback_autonomy.benchmarking.estimators import (
    DEPTH_PATH_ESTIMATORS,
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_LABELS,
    MASK_GATE_SILHOUETTE,
    PUBLIC_ESTIMATOR_ORDER,
)
from ridgeback_autonomy.common.models import DetectionBatch
from ridgeback_autonomy.perception.core.geometry import focus_bbox
from ridgeback_autonomy.perception.core.mask import (
    MaskPrecision,
    mask_from_array,
    masked_rgb,
    rasterize_batch,
)
from ridgeback_autonomy.perception.core.polar_profiling import (
    merge_near_band,
    segment_range_profile,
)


# Panel kinds. A run renders only the ones its selected estimators need, so the
# view matches the config instead of a fixed grid.
PANEL_RGB = 'rgb'
PANEL_SENSOR_DEPTH = 'sensor_depth'
PANEL_MONO_DEPTH = 'depth_anything'
PANEL_ALIGNED_DEPTH = 'aligned_depth'
PANEL_BOX_MASK = 'box_mask'
PANEL_SILHOUETTE = 'silhouette'
PANEL_LIDAR = 'lidar'

# Panels per row when packing the grid. Three suits a roughly square window;
# a wide, short target (the RViz strip) wants them all on one row instead.
PANEL_MAX_COLS_DEFAULT = 3

# Estimators that report a full planar position (lateral/forward/distance); the
# rest report distance only. Field names are ``{estimator}_lateral_m`` etc.,
# except the distance-only pair whose keys live in ``ESTIMATOR_FIELD_KEYS``.
POSITION_ESTIMATORS = frozenset({
    'rgb',
    'pointcloud',
    'lidar',
    'projective_ranging',
    'euclidean_reconstruction',
    'polar_profiling',
})


@dataclass(frozen=True)
class PanelSpec:
    kind: str
    title: str


def select_panels(estimators, depth_source: str, mask_gate: str) -> list[PanelSpec]:
    """The panels a run needs, in display order, driven by its estimator set.

    RGB is always the anchor. Each further panel appears only when an estimator
    that consumes its data is selected: the aligned-depth panel and the mask
    (box or silhouette) panel are tied to the depth mask paths, and the LiDAR /
    polar panel to polar profiling. The mask gate picks box vs silhouette.
    """

    selected = set(estimators)
    panels: list[PanelSpec] = [PanelSpec(PANEL_RGB, 'RGB Detection')]

    if 'sensor_depth' in selected:
        panels.append(PanelSpec(PANEL_SENSOR_DEPTH, 'Sensor Depth'))
    if 'depth_anything' in selected:
        panels.append(PanelSpec(PANEL_MONO_DEPTH, 'Depth-Anything'))
    if selected & DEPTH_PATH_ESTIMATORS:
        panels.append(PanelSpec(PANEL_ALIGNED_DEPTH, f'Aligned Depth ({depth_source})'))
        if mask_gate == MASK_GATE_SILHOUETTE:
            panels.append(PanelSpec(PANEL_SILHOUETTE, 'Silhouette Mask'))
        else:
            panels.append(PanelSpec(PANEL_BOX_MASK, 'Box Mask'))
    if 'polar_profiling' in selected:
        # Name the panel after the selected lidar-family estimator; only when
        # the legacy lidar row is also selected does it carry both names.
        title = 'LiDAR / Polar' if 'lidar' in selected else ESTIMATOR_LABELS['polar_profiling']
        panels.append(PanelSpec(PANEL_LIDAR, title))

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

    Position-capable estimators render lateral/forward/distance; the rest render
    distance only. A missing field reads as ``NA`` (no estimate for this frame).
    """

    selected = set(estimators)
    lines: list[str] = []
    for estimator in PUBLIC_ESTIMATOR_ORDER:
        if estimator not in selected:
            continue
        label = ESTIMATOR_LABELS[estimator]
        if estimator in POSITION_ESTIMATORS:
            lines.append(format_position_line(
                label,
                getattr(detection, f'{estimator}_lateral_m', None),
                getattr(detection, f'{estimator}_forward_m', None),
                getattr(detection, f'{estimator}_distance_m', None),
            ))
        else:
            lines.append(format_distance_line(
                label, getattr(detection, ESTIMATOR_FIELD_KEYS[estimator], None)))
    return lines


def format_position_line(
    prefix: str,
    lateral_m: float | None,
    forward_m: float | None,
    distance_m: float | None,
) -> str:
    if lateral_m is None or forward_m is None or distance_m is None:
        return f'{prefix} d=NA'
    return f'{prefix} x={lateral_m:+.2f} z={forward_m:+.2f} d={distance_m:.2f}m'


def format_distance_line(prefix: str, distance_m: float | None) -> str:
    if distance_m is None:
        return f'{prefix} d=NA'
    return f'{prefix} d={distance_m:.2f}m'


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
        self.mask_gate = mask_gate
        self.max_cols = max_cols
        self.rgb_panel_labels = rgb_panel_labels
        self.panels = select_panels(self.estimators, depth_source, mask_gate)

    def render(
        self,
        frame: np.ndarray,
        batch: DetectionBatch,
        *,
        sensor_depth_meters: np.ndarray | None = None,
        mono_depth_meters: np.ndarray | None = None,
        aligned_depth_meters: np.ndarray | None = None,
        published_mask: np.ndarray | None = None,
        scan_uv: np.ndarray | None = None,
        scan_in_view: np.ndarray | None = None,
        scan_points_optical: np.ndarray | None = None,
        truth: tuple[float, float, float] | None = None,
    ) -> np.ndarray:
        images = [
            self.build_panel(
                spec, frame, batch,
                sensor_depth_meters=sensor_depth_meters,
                mono_depth_meters=mono_depth_meters,
                aligned_depth_meters=aligned_depth_meters,
                published_mask=published_mask,
                scan_uv=scan_uv,
                scan_in_view=scan_in_view,
                scan_points_optical=scan_points_optical,
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
        sensor_depth_meters,
        mono_depth_meters,
        aligned_depth_meters,
        published_mask,
        scan_uv,
        scan_in_view,
        scan_points_optical,
        truth=None,
    ) -> np.ndarray:
        if spec.kind == PANEL_RGB:
            panel = frame.copy()
            self.annotate_detections(
                panel, batch, draw_labels=self.rgb_panel_labels, truth=truth)
        elif spec.kind == PANEL_SENSOR_DEPTH:
            panel = self.make_depth_panel(frame.shape[:2], sensor_depth_meters)
            self.annotate_detections(panel, batch, draw_labels=False)
        elif spec.kind == PANEL_MONO_DEPTH:
            panel = self.make_depth_panel(frame.shape[:2], mono_depth_meters)
            self.annotate_detections(panel, batch, draw_labels=False)
        elif spec.kind == PANEL_ALIGNED_DEPTH:
            panel = self.make_depth_panel(frame.shape[:2], aligned_depth_meters)
            self.annotate_detections(panel, batch, draw_labels=False)
        elif spec.kind == PANEL_BOX_MASK:
            panel = self.make_box_mask_panel(frame, batch)
        elif spec.kind == PANEL_SILHOUETTE:
            panel = self.make_silhouette_mask_panel(frame, published_mask)
        elif spec.kind == PANEL_LIDAR:
            panel = self.make_lidar_panel(
                frame, batch, published_mask, scan_uv, scan_in_view, scan_points_optical)
        else:
            panel = np.zeros_like(frame)

        self.draw_panel_title(panel, spec.title)
        if not batch.detected and spec.kind in (
                PANEL_RGB, PANEL_SENSOR_DEPTH, PANEL_MONO_DEPTH, PANEL_ALIGNED_DEPTH):
            cv2.putText(panel, 'No G1 detected', (20, 100),
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
                lines = [f'G1 #{index + 1} ({detection.score:.0%})']
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
        published_mask: np.ndarray | None,
        scan_uv: np.ndarray | None,
        scan_in_view: np.ndarray | None,
        scan_points_optical: np.ndarray | None,
    ) -> np.ndarray:
        panel = frame.copy()
        self.annotate_detections(panel, batch, draw_labels=False)
        highlight = None
        if scan_uv is not None and scan_points_optical is not None:
            select_mask = self.lidar_select_mask(frame.shape[:2], batch, published_mask)
            highlight = polar_highlight_beams(
                scan_uv, scan_in_view, scan_points_optical, select_mask)
        draw_scan_points(panel, scan_uv, highlight)
        return panel

    def lidar_select_mask(
        self,
        color_shape,
        batch: DetectionBatch,
        published_mask: np.ndarray | None,
    ) -> np.ndarray | None:
        """The boolean selector polar profiling would use: the silhouette union
        when it is available, else the rasterized detection boxes."""

        if (self.mask_gate == MASK_GATE_SILHOUETTE
                and published_mask is not None
                and published_mask.shape == tuple(color_shape)):
            return published_mask.astype(bool)
        if not batch.detected:
            return None
        mask = rasterize_batch(batch)
        if mask.data.shape == tuple(color_shape):
            return mask.data
        return None

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
            cv2.putText(panel, 'No G1 detected', (20, 100),
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


def polar_highlight_beams(
    scan_uv: np.ndarray,
    scan_in_view: np.ndarray,
    scan_points_optical: np.ndarray,
    select_mask: np.ndarray | None,
) -> np.ndarray:
    """Per-beam boolean of the beams polar profiling actually reduces.

    Not merely "inside the mask": the mask select also admits the far
    background (the scan's field of view sees past the object, and by
    perspective those far beams land near the horizon -- torso/arm height in the
    image). Those beams are dropped by the range segmentation, so the highlight
    applies the same nearest-range-band merge the estimator uses and marks only
    the survivors. Assumes one near object across the mask union (the common
    single-robot case); multiple objects at different ranges would keep only the
    nearest band.
    """

    scan_uv = np.asarray(scan_uv, dtype=np.float64)
    highlight = np.zeros(scan_uv.shape[0], dtype=bool)
    if select_mask is None:
        return highlight

    beams = np.flatnonzero(np.asarray(scan_in_view, dtype=bool))
    if beams.size == 0:
        return highlight
    height, width = select_mask.shape[:2]
    u_px = np.rint(scan_uv[beams, 0]).astype(np.intp)
    v_px = np.rint(scan_uv[beams, 1]).astype(np.intp)
    inside_frame = (u_px >= 0) & (u_px < width) & (v_px >= 0) & (v_px < height)
    beams = beams[inside_frame]
    if beams.size == 0:
        return highlight
    in_mask = select_mask[v_px[inside_frame], u_px[inside_frame]]
    mask_beams = beams[in_mask]
    if mask_beams.size == 0:
        return highlight

    points = np.asarray(scan_points_optical, dtype=np.float64)[mask_beams]
    planar_range_m = np.hypot(points[:, 0], points[:, 2])
    runs = segment_range_profile(mask_beams, planar_range_m)
    merged = merge_near_band(runs, planar_range_m)
    highlight[mask_beams[merged]] = True
    return highlight


def draw_scan_points(
    panel: np.ndarray,
    scan_uv: np.ndarray | None,
    highlight: np.ndarray | None,
) -> None:
    """Draw the beams polar profiling reduces to its estimate, and only those.

    The other in-view beams are deliberately not drawn: the panel answers
    "which rays produced this distance", so the background returns the range
    segmentation discards would read as part of the measurement. An empty
    highlight (no mask, or no run survived) therefore draws nothing -- the
    honest picture of a frame that produced no polar estimate.
    """

    if scan_uv is None or highlight is None:
        return
    scan_uv = np.asarray(scan_uv, dtype=np.float64)
    highlight = np.asarray(highlight, dtype=bool)
    height, width = panel.shape[:2]
    for index in np.flatnonzero(highlight):
        u_px = int(round(float(scan_uv[index, 0])))
        v_px = int(round(float(scan_uv[index, 1])))
        if not (0 <= u_px < width and 0 <= v_px < height):
            continue
        cv2.circle(panel, (u_px, v_px), 3, (0, 255, 255), -1, cv2.LINE_AA)
