from __future__ import annotations

import cv2
import numpy as np

from ridgeback_autonomy.common.models import DetectionBatch
from ridgeback_autonomy.perception.core.geometry import focus_bbox
from ridgeback_autonomy.perception.core.mask import masked_rgb, rasterize_batch


class RgbdOverlayRenderer:
    def __init__(self, depth_max_meters: float) -> None:
        self.depth_max_meters = depth_max_meters

    def render(
        self,
        frame: np.ndarray,
        sensor_depth_meters: np.ndarray | None,
        mono_depth_meters: np.ndarray | None,
        batch: DetectionBatch,
        sensor_depth_warning: str | None,
        mono_depth_warning: str | None,
    ) -> np.ndarray:
        color_panel = frame.copy()
        sensor_depth_panel = self.make_depth_panel(frame.shape[:2], sensor_depth_meters)
        mono_depth_panel = self.make_depth_panel(frame.shape[:2], mono_depth_meters)

        self.draw_panel_title(color_panel, 'RGB Detection')
        self.draw_panel_title(sensor_depth_panel, 'Sensor Depth')
        self.draw_panel_title(mono_depth_panel, 'Depth-Anything')

        # Mask panel is derived from the detection boxes at render time (the mask
        # is not published anywhere yet). Built from the clean RGB frame, not the
        # annotated color_panel, so it shows the real masked content.
        mask_panel = self.make_mask_panel(frame, batch)

        if sensor_depth_warning:
            cv2.putText(sensor_depth_panel, sensor_depth_warning, (20, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
        if mono_depth_warning:
            cv2.putText(mono_depth_panel, mono_depth_warning, (20, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        if not batch.detected:
            for panel in (color_panel, sensor_depth_panel, mono_depth_panel):
                cv2.putText(panel, 'No G1 detected', (20, 100),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
            return np.hstack((color_panel, sensor_depth_panel, mono_depth_panel, mask_panel))

        for index, detection in enumerate(batch.detections):
            x1, y1, x2, y2 = detection.bbox_xyxy
            label_lines = [f'G1 #{index + 1} ({detection.score:.0%})']
            label_lines.append(
                self.format_position_line(
                    'RGB',
                    detection.rgb_lateral_m,
                    detection.rgb_forward_m,
                    detection.rgb_distance_m,
                )
            )
            label_lines.append(self.format_distance_line('Depth', detection.sensor_depth_distance_m))
            label_lines.append(self.format_distance_line('Mono', detection.mono_depth_distance_m))
            label_lines.append(
                self.format_position_line(
                    'Cloud',
                    detection.pointcloud_lateral_m,
                    detection.pointcloud_forward_m,
                    detection.pointcloud_distance_m,
                )
            )
            label_lines.append(
                self.format_position_line(
                    'LiDAR',
                    detection.lidar_lateral_m,
                    detection.lidar_forward_m,
                    detection.lidar_distance_m,
                )
            )

            for panel in (color_panel, sensor_depth_panel, mono_depth_panel):
                cv2.rectangle(panel, (x1, y1), (x2, y2), (0, 255, 0), 2)
                focus = detection.focus_bbox_xyxy or focus_bbox(detection.bbox_xyxy)
                if focus is not None:
                    fx1, fy1, fx2, fy2 = focus
                    cv2.rectangle(panel, (fx1, fy1), (fx2, fy2), (0, 200, 255), 2)
                self.draw_label_block(panel, x1, y1, label_lines, (0, 255, 0))

        return np.hstack((color_panel, sensor_depth_panel, mono_depth_panel, mask_panel))

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

    def make_mask_panel(self, frame: np.ndarray, batch: DetectionBatch) -> np.ndarray:
        # Masked RGB: real pixels inside the rect mask, black outside. For a
        # rect mask this is the RGB rectangle of the box(es) on black, which
        # makes the box's background contamination directly visible.
        mask = rasterize_batch(batch)
        if mask.data.shape == frame.shape[:2]:
            panel = masked_rgb(frame, mask)
        else:
            # Detections came from a differently sized frame; show an empty
            # panel rather than risk an index mismatch.
            panel = np.zeros_like(frame)

        self.draw_panel_title(panel, 'Mask')
        if not batch.detected:
            cv2.putText(panel, 'No G1 detected', (20, 100),
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
                         lines: list[str], color) -> None:
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
            text_color = color if line_index == 0 else (245, 245, 245)
            origin = (left_x + padding_x, text_y)
            cv2.putText(
                image,
                line,
                origin,
                font,
                scale,
                (0, 0, 0),
                thickness + 3,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                line,
                origin,
                font,
                scale,
                text_color,
                thickness,
                cv2.LINE_AA,
            )
            text_y += line_height

    def format_position_line(
        self,
        prefix: str,
        lateral_m: float | None,
        forward_m: float | None,
        distance_m: float | None,
    ) -> str:
        if lateral_m is None or forward_m is None or distance_m is None:
            return f'{prefix:<5} d=NA'
        return (
            f'{prefix:<5} x={lateral_m:+.2f} '
            f'z={forward_m:+.2f} '
            f'd={distance_m:.2f}m'
        )

    def format_distance_line(self, prefix: str, distance_m: float | None) -> str:
        if distance_m is None:
            return f'{prefix:<5} d=NA'
        return f'{prefix:<5} d={distance_m:.2f}m'
