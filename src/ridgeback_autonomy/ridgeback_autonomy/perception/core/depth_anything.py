from __future__ import annotations

import importlib

import cv2
import numpy as np

from ridgeback_autonomy.perception.core.detection import resolve_torch_device


DEPTH_ANYTHING_ENABLED_DEFAULT = False
DEPTH_ANYTHING_MODEL_ID_DEFAULT = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'


class DepthAnythingEstimator:
    def __init__(self, enabled: bool, model_id: str, device: str | None, logger) -> None:
        self.enabled = enabled
        self.model_id = model_id
        self.device = device or resolve_torch_device()
        self.logger = logger
        self._pipeline = None

    def load(self):
        if not self.enabled:
            self.logger.info('Depth-Anything metric branch is disabled.')
            return None

        self.logger.info(f'Loading Depth-Anything model: {self.model_id} on {self.device}')
        try:
            transformers = importlib.import_module('transformers')
            self._pipeline = transformers.pipeline(
                task='depth-estimation',
                model=self.model_id,
                device=self.device,
            )
        except Exception as exc:
            self.logger.warn(f'Cannot load Depth-Anything model ({self.model_id}): {exc}')
            self._pipeline = None
        else:
            self.logger.info('Depth-Anything model loaded.')
        return self._pipeline

    def predict(self, image, target_shape: tuple[int, int]) -> tuple[np.ndarray | None, str | None]:
        if not self.enabled:
            return None, 'Depth-Anything disabled'
        if self._pipeline is None and self.load() is None:
            return None, 'Depth-Anything unavailable'

        try:
            outputs = self._pipeline(image)
        except Exception as exc:
            self.logger.warn(
                f'Depth-Anything inference failed: {exc}. Disabling the metric branch.'
            )
            self._pipeline = None
            return None, 'Depth-Anything unavailable'

        predicted_depth = outputs.get('predicted_depth')
        if predicted_depth is None:
            self.logger.warn('Depth-Anything returned no predicted_depth output.')
            self._pipeline = None
            return None, 'Depth-Anything unavailable'

        if hasattr(predicted_depth, 'detach'):
            depth_meters = predicted_depth.detach().cpu().numpy()
        else:
            depth_meters = np.asarray(predicted_depth)

        depth_meters = np.squeeze(depth_meters).astype(np.float32)
        if depth_meters.ndim != 2:
            self.logger.warn(
                f'Depth-Anything returned unexpected shape {depth_meters.shape}. '
                'Disabling the metric branch.'
            )
            self._pipeline = None
            return None, 'Depth-Anything unavailable'

        target_h, target_w = target_shape
        if depth_meters.shape != (target_h, target_w):
            depth_meters = cv2.resize(
                depth_meters,
                (target_w, target_h),
                interpolation=cv2.INTER_LINEAR,
            )

        return depth_meters, None
