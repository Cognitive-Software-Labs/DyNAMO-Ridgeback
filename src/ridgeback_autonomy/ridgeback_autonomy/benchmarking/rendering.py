from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from ridgeback_autonomy.benchmarking.alignment import MeasurementEvent
from ridgeback_autonomy.perception.estimators import (
    ESTIMATOR_FIELD_KEYS,
    ESTIMATOR_LABELS,
)
from ridgeback_autonomy.perception.core.rendering import RgbdOverlayRenderer


# Every panel is the colour frame with that estimator's boxes and numbers drawn
# on it -- none of the surviving estimators has imagery of its own to show.
COLOR_SOURCE_LABEL = 'RGB Debug View'


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
    def __init__(self, panel_max_width: int = 320) -> None:
        # Borrowed for its title and label-block drawing only: the collage no
        # longer renders any depth imagery, so the renderer's depth range is
        # never consulted.
        self.overlay = RgbdOverlayRenderer(depth_max_meters=0.0)
        self.panel_max_width = panel_max_width

    def make_color_preview(self, frame_bgr: np.ndarray) -> np.ndarray:
        return self.resize_panel(frame_bgr)

    def render_trial_collage(
        self,
        trial_id: str,
        representative_event: MeasurementEvent,
        selected_estimators: tuple[str, ...],
        trial_medians: dict[str, float],
        true_distance_m: float,
        box_annotations: list[dict] | None = None,
        missed_count: int = 0,
        miss_reasons: dict[str, str] | None = None,
    ) -> np.ndarray:
        # trial_medians is partial: an estimator with no usable events has no
        # key, and its panel shows the dominant miss reason instead of a value.
        miss_reasons = miss_reasons or {}
        panels = [
            self.render_estimator_panel(
                estimator,
                trial_id,
                representative_event,
                trial_medians.get(estimator),
                true_distance_m,
                box_annotations,
                missed_count,
                miss_reasons.get(estimator),
            )
            for estimator in selected_estimators
        ]
        return np.hstack(panels)

    def render_estimator_panel(
        self,
        estimator: str,
        trial_id: str,
        event: MeasurementEvent,
        trial_median_m: float | None,
        true_distance_m: float,
        box_annotations: list[dict] | None = None,
        missed_count: int = 0,
        miss_reason: str | None = None,
    ) -> np.ndarray:
        context = self.panel_context_for_event(event)
        panel = context.panel.copy()
        self.overlay.draw_panel_title(panel, ESTIMATOR_LABELS[estimator])
        self.draw_bboxes(panel, event, estimator, box_annotations)
        if missed_count:
            self.draw_missed_note(panel, missed_count)

        frame_value = event.estimates.get(estimator)
        lines = self.build_panel_lines(
            context,
            trial_id,
            frame_value,
            trial_median_m,
            true_distance_m,
            event.stamp_ns,
        )
        if trial_median_m is None and miss_reason:
            lines.append(f'Reason: {miss_reason}')
        self.draw_value_block(panel, lines)
        return panel

    def panel_context_for_event(self, event: MeasurementEvent) -> PanelContext:
        """The colour frame every estimator's panel is drawn on, or a placeholder.

        One context for the whole collage: no estimator renders its own imagery
        any more, so the panels differ in their annotations, not their source.
        """

        if event.preview.color_bgr is not None:
            return self.build_panel_context(
                panel=event.preview.color_bgr,
                source_label=COLOR_SOURCE_LABEL,
                expected_source_label=COLOR_SOURCE_LABEL,
                preview_available=True,
                matched_stamp_ns=event.preview.color_stamp_ns,
                match_delta_ms=event.preview.color_delta_ms,
                nearest_stamp_ns=event.preview.color_nearest_stamp_ns,
                nearest_delta_ms=event.preview.color_nearest_delta_ms,
            )
        return self.build_panel_context(
            panel=self.make_missing_panel(event),
            source_label=COLOR_SOURCE_LABEL,
            expected_source_label=COLOR_SOURCE_LABEL,
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
        trial_median_m: float | None,
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

    def draw_bboxes(
        self,
        panel: np.ndarray,
        event: MeasurementEvent,
        estimator: str | None = None,
        box_annotations: list[dict] | None = None,
    ) -> None:
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
            label, color = self.box_label_and_color(event, index, estimator, box_annotations)
            cv2.rectangle(panel, (sx1, sy1), (sx2, sy2), color, 2)
            cv2.putText(
                panel,
                label,
                (sx1, max(26, sy1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )

    def box_label_and_color(
        self,
        event: MeasurementEvent,
        index: int,
        estimator: str | None,
        box_annotations: list[dict] | None,
    ) -> tuple[str, tuple[int, int, int]]:
        # Without per-instance annotations (single-robot / tests) keep the
        # historical green "G1 #i" label so those collages stay identical.
        if box_annotations is None or estimator is None:
            return f'G1 #{index + 1}', (0, 255, 0)
        annotation = box_annotations[index] if index < len(box_annotations) else None
        if annotation is None:
            # Detection matched no ground truth: an extra / false positive.
            return 'extra', (0, 165, 255)
        value = None
        if index < len(event.detections):
            value = getattr(event.detections[index], ESTIMATOR_FIELD_KEYS[estimator], None)
        value_str = f'{value:.2f}' if value is not None else 'NA'
        label = f"#{annotation['instance_index']} e{value_str}/t{annotation['true_distance_m']:.2f}"
        return label, (0, 255, 0)

    def draw_missed_note(self, panel: np.ndarray, missed_count: int) -> None:
        text = f'missed: {missed_count}'
        (text_width, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        origin = (max(10, panel.shape[1] - text_width - 12), 30)
        cv2.putText(panel, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

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
