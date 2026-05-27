from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ridgeback_autonomy.benchmarking.alignment import MeasurementEvent
from ridgeback_autonomy.benchmarking.estimators import (
    ESTIMATOR_LABELS,
    RGB_DEBUG_VIEW_ESTIMATORS,
)
from ridgeback_autonomy.perception.core.rendering import RgbdOverlayRenderer


@dataclass
class PanelContext:
    panel: np.ndarray
    source_label: str
    expected_source_label: str
    preview_available: bool
    matched_stamp_ns: int | None
    match_delta_ms: float | None
    nearest_stamp_ns: int | None
    nearest_delta_ms: float | None


class BenchmarkCollageRenderer:
    def __init__(self, depth_max_meters: float, panel_max_width: int = 320) -> None:
        self.overlay = RgbdOverlayRenderer(depth_max_meters)
        self.panel_max_width = panel_max_width

    def make_color_preview(self, frame_bgr: np.ndarray) -> np.ndarray:
        return self.resize_panel(frame_bgr)

    def make_depth_preview(self, depth_meters: np.ndarray) -> np.ndarray:
        panel = self.overlay.make_depth_panel(depth_meters.shape[:2], depth_meters)
        return self.resize_panel(panel)

    def render_trial_collage(
        self,
        trial_id: str,
        representative_event: MeasurementEvent,
        selected_estimators: tuple[str, ...],
        trial_medians: dict[str, float],
        true_distance_m: float,
    ) -> np.ndarray:
        panels = [
            self.render_estimator_panel(
                estimator,
                trial_id,
                representative_event,
                trial_medians[estimator],
                true_distance_m,
            )
            for estimator in selected_estimators
        ]
        return np.hstack(panels)

    def render_estimator_panel(
        self,
        estimator: str,
        trial_id: str,
        event: MeasurementEvent,
        trial_median_m: float,
        true_distance_m: float,
    ) -> np.ndarray:
        context = self.panel_context_for_estimator(estimator, event)
        panel = context.panel.copy()
        self.overlay.draw_panel_title(panel, ESTIMATOR_LABELS[estimator])
        self.draw_bboxes(panel, event)

        frame_value = event.estimates.get(estimator)
        lines = self.build_panel_lines(
            context,
            trial_id,
            frame_value,
            trial_median_m,
            true_distance_m,
            event.stamp_ns,
        )
        self.draw_value_block(panel, lines)
        return panel

    def panel_context_for_estimator(
        self,
        estimator: str,
        event: MeasurementEvent,
    ) -> PanelContext:
        if estimator == 'sensor_depth' and event.preview.sensor_depth_bgr is not None:
            return self.build_panel_context(
                panel=event.preview.sensor_depth_bgr,
                source_label='Sensor Depth',
                expected_source_label='Sensor Depth',
                preview_available=True,
                matched_stamp_ns=event.preview.sensor_depth_stamp_ns,
                match_delta_ms=event.preview.sensor_depth_delta_ms,
                nearest_stamp_ns=event.preview.sensor_depth_nearest_stamp_ns,
                nearest_delta_ms=event.preview.sensor_depth_nearest_delta_ms,
            )
        if estimator == 'sensor_depth':
            return self.build_panel_context(
                panel=self.make_missing_panel(event),
                source_label='Sensor Depth',
                expected_source_label='Sensor Depth',
                preview_available=False,
                matched_stamp_ns=None,
                match_delta_ms=None,
                nearest_stamp_ns=event.preview.sensor_depth_nearest_stamp_ns,
                nearest_delta_ms=event.preview.sensor_depth_nearest_delta_ms,
            )

        if estimator == 'depth_anything' and event.preview.depth_anything_bgr is not None:
            return self.build_panel_context(
                panel=event.preview.depth_anything_bgr,
                source_label='Depth-Anything',
                expected_source_label='Depth-Anything',
                preview_available=True,
                matched_stamp_ns=event.preview.depth_anything_stamp_ns,
                match_delta_ms=event.preview.depth_anything_delta_ms,
                nearest_stamp_ns=event.preview.depth_anything_nearest_stamp_ns,
                nearest_delta_ms=event.preview.depth_anything_nearest_delta_ms,
            )
        if estimator == 'depth_anything':
            return self.build_panel_context(
                panel=self.make_missing_panel(event),
                source_label='Depth-Anything',
                expected_source_label='Depth-Anything',
                preview_available=False,
                matched_stamp_ns=None,
                match_delta_ms=None,
                nearest_stamp_ns=event.preview.depth_anything_nearest_stamp_ns,
                nearest_delta_ms=event.preview.depth_anything_nearest_delta_ms,
            )

        color_source_label = 'RGB Debug View' if estimator in RGB_DEBUG_VIEW_ESTIMATORS else 'RGB'
        if event.preview.color_bgr is not None:
            return self.build_panel_context(
                panel=event.preview.color_bgr,
                source_label=color_source_label,
                expected_source_label=color_source_label,
                preview_available=True,
                matched_stamp_ns=event.preview.color_stamp_ns,
                match_delta_ms=event.preview.color_delta_ms,
                nearest_stamp_ns=event.preview.color_nearest_stamp_ns,
                nearest_delta_ms=event.preview.color_nearest_delta_ms,
            )
        return self.build_panel_context(
            panel=self.make_missing_panel(event),
            source_label=color_source_label,
            expected_source_label=color_source_label,
            preview_available=False,
            matched_stamp_ns=None,
            match_delta_ms=None,
            nearest_stamp_ns=event.preview.color_nearest_stamp_ns,
            nearest_delta_ms=event.preview.color_nearest_delta_ms,
        )

    def build_panel_context(
        self,
        panel: np.ndarray,
        source_label: str,
        expected_source_label: str,
        preview_available: bool,
        matched_stamp_ns: int | None,
        match_delta_ms: float | None,
        nearest_stamp_ns: int | None,
        nearest_delta_ms: float | None,
    ) -> PanelContext:
        return PanelContext(
            panel=panel,
            source_label=source_label,
            expected_source_label=expected_source_label,
            preview_available=preview_available,
            matched_stamp_ns=matched_stamp_ns,
            match_delta_ms=match_delta_ms,
            nearest_stamp_ns=nearest_stamp_ns,
            nearest_delta_ms=nearest_delta_ms,
        )

    def make_missing_panel(self, event: MeasurementEvent) -> np.ndarray:
        height = max(event.image_height, 180)
        width = max(event.image_width, 320)
        panel = np.full((height, width, 3), 22, dtype=np.uint8)
        cv2.rectangle(panel, (6, 6), (width - 7, height - 7), (0, 0, 255), 2)
        cv2.putText(
            panel,
            'Preview Missing',
            (18, 96),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.95,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            'Benchmark image could not be matched.',
            (18, 132),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (235, 235, 235),
            2,
            cv2.LINE_AA,
        )
        return self.resize_panel(panel)

    def build_panel_lines(
        self,
        context: PanelContext,
        trial_id: str,
        frame_value: float | None,
        trial_median_m: float,
        true_distance_m: float,
        event_stamp_ns: int,
    ) -> list[str]:
        lines = [f'Trial: {trial_id}']
        if context.preview_available:
            lines.append(f'Source: {context.source_label}')
            lines.append(f'Delta: {self.format_delta(context.match_delta_ms)}')
        else:
            lines.append('Preview Missing')
            lines.append(f'Expected: {context.expected_source_label}')
            lines.append(f'Event: {event_stamp_ns}')
            lines.append(f'Nearest: {self.format_delta(context.nearest_delta_ms)}')
        lines.extend([
            f'Frame: {self.format_distance(frame_value)}',
            f'Median: {self.format_distance(trial_median_m)}',
            f'True: {self.format_distance(true_distance_m)}',
        ])
        return lines

    def resize_panel(self, panel: np.ndarray) -> np.ndarray:
        height, width = panel.shape[:2]
        if width <= self.panel_max_width:
            return panel
        scale = self.panel_max_width / float(width)
        return cv2.resize(
            panel,
            (int(round(width * scale)), int(round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )

    def draw_bboxes(self, panel: np.ndarray, event: MeasurementEvent) -> None:
        if not event.bboxes:
            return

        scale_x = panel.shape[1] / max(event.image_width, 1)
        scale_y = panel.shape[0] / max(event.image_height, 1)
        for index, bbox in enumerate(event.bboxes):
            x1, y1, x2, y2 = bbox
            sx1 = int(round(x1 * scale_x))
            sy1 = int(round(y1 * scale_y))
            sx2 = int(round(x2 * scale_x))
            sy2 = int(round(y2 * scale_y))
            cv2.rectangle(panel, (sx1, sy1), (sx2, sy2), (0, 255, 0), 2)
            cv2.putText(
                panel,
                f'G1 #{index + 1}',
                (sx1, max(26, sy1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

    def draw_value_block(self, panel: np.ndarray, lines: list[str]) -> None:
        self.overlay.draw_label_block(
            panel,
            10,
            panel.shape[0] - 12,
            lines,
            (0, 255, 0),
        )

    def format_distance(self, value: float | None) -> str:
        if value is None:
            return 'N/A'
        return f'{value:.3f}m'

    def format_delta(self, value_ms: float | None) -> str:
        if value_ms is None:
            return 'N/A'
        return f'{value_ms:.1f}ms'
