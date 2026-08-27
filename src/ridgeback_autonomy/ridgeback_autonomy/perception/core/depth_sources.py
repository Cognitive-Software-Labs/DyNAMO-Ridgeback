"""Aligned depth frame sources (object_localization_documentation/aligned_depth.md).

Produces the "aligned depth frame" contract: a float32 depth image in meters
on the color camera's pixel grid, stamped with the frame it is aligned to,
with 0/NaN/inf meaning "no depth here". Two interchangeable sources satisfy
the contract behind a single config switch (`depth_source`):

- ``stereoscopic``: converts the camera depth stream to float meters (sim: the
  co-registered gz render; real: the driver's ``aligned_depth_to_color``
  topic -- point ``depth_topic`` at it).
- ``monocular``: predicts metric depth from the RGB stream with
  Depth-Anything V2, aligned by construction.

The sources are pulled by ``g1_mask_measurement_node``, once per detection
batch, on the input frame buffered at the detection stamp -- there is no
producer process and no depth topic in between. Each source declares which
stream it reads through ``input_kind``; nothing downstream branches on depth
semantics.

This module is deliberately independent of the distance-estimator stack
(geometry.py / g1_camera_measurement_node); it shares no code with it.
"""

from __future__ import annotations

import importlib
import time

import numpy as np
from sensor_msgs.msg import Image

from ridgeback_autonomy.perception.core.image_utils import (
    convert_depth_to_meters_message,
    decode_image_message,
)


DEPTH_SOURCE_STEREOSCOPIC = 'stereoscopic'
DEPTH_SOURCE_MONOCULAR = 'monocular'
DEPTH_ANYTHING_MODEL_ID_DEFAULT = 'depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf'
# After a load/inference failure the monocular source waits this long before
# re-attempting, instead of disabling itself for the rest of the run. A single
# transient hiccup (e.g. a CUDA OOM) then self-heals rather than silently
# starving every ``monocular`` benchmark row.
MONOCULAR_RETRY_COOLDOWN_S_DEFAULT = 30.0


def decode_depth_to_meters(msg: Image) -> np.ndarray:
    """Decode a depth Image message into float32 meters, invalid pixels kept
    as 0/NaN/inf per the aligned-depth-frame contract.

    Delegates to the shared decoder, which honours ``msg.step`` (row padding).
    A real camera driver may emit row-aligned buffers (``step > width *
    itemsize``); the previous hand-rolled ``reshape(height, width)`` rejected
    every such frame. This stays the contract's single entry point, so both
    the ``16UC1`` millimetres a real D435 ships and the ``32FC1`` metres sim
    renders decode the same way.
    """

    if msg.encoding not in ('16UC1', 'mono16', '32FC1'):
        raise ValueError(f'unsupported depth encoding "{msg.encoding}"')
    return convert_depth_to_meters_message(msg)


def decode_color_to_rgb(msg: Image) -> np.ndarray:
    """Decode a color Image message into an RGB uint8 array.

    Delegates the row decode to the shared step-aware decoder (honours
    ``msg.step``) -- the same fix as the depth path:
    a real driver may pad rows (``step > width * channels``),
    which the old hand-rolled ``reshape`` rejected, throwing every frame. The
    explicit whitelist stays here to preserve the exact error message and the
    mono rejection. ``produce`` re-contiguous-izes, so returning views is fine.
    """

    if msg.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        raise ValueError(f'unsupported color encoding "{msg.encoding}"')
    image = decode_image_message(msg)
    if msg.encoding == 'rgb8':
        return image
    if msg.encoding == 'bgr8':
        return image[:, :, ::-1]
    if msg.encoding == 'rgba8':
        return image[:, :, :3]
    return image[:, :, :3][:, :, ::-1]  # bgra8


def encode_depth_message(depth_m: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = depth_m.shape
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = msg.width * 4
    msg.data = np.ascontiguousarray(depth_m, dtype=np.float32).tobytes()
    return msg


class StereoDepthSource:
    """camera depth stream -> aligned depth frame.

    In sim the gz render is co-registered with color by construction; on real
    hardware the input topic must already be the driver-aligned
    ``aligned_depth_to_color`` stream (alignment happens in the driver, not
    here). Either way the source only converts units and re-stamps nothing:
    the incoming header already carries the aligned grid's stamp and frame.
    """

    input_kind = 'depth'

    def __init__(self, logger) -> None:
        self.logger = logger

    def produce(self, msg: Image) -> tuple[np.ndarray, object] | None:
        try:
            depth_m = decode_depth_to_meters(msg)
        except ValueError as exc:
            self.logger.warn(f'Stereo source skipped a frame: {exc}')
            return None
        return depth_m, msg.header


class MonocularDepthSource:
    """RGB stream -> Depth-Anything V2 metric depth -> aligned depth frame.

    Aligned by construction: the network input is the color image, so the
    output grid is the color grid. The frame is stamped with the color
    message header it was predicted from.
    """

    input_kind = 'color'

    def __init__(
        self,
        model_id: str,
        device: str,
        logger,
        *,
        now_fn=time.monotonic,
        cooldown_s: float = MONOCULAR_RETRY_COOLDOWN_S_DEFAULT,
    ) -> None:
        self.model_id = model_id
        self.device = device or self.resolve_device()
        self.logger = logger
        self._pipeline = None
        # Retry-with-cooldown instead of a permanent fail latch:
        # ``_retry_after`` is the earliest time (``now_fn`` seconds) a new load
        # is allowed. 0 lets the first load run immediately.
        self._now_fn = now_fn
        self._cooldown_s = float(cooldown_s)
        self._retry_after = 0.0

    @staticmethod
    def resolve_device() -> str:
        try:
            torch = importlib.import_module('torch')
            if torch.cuda.is_available():
                return 'cuda'
        except Exception:
            pass
        return 'cpu'

    def _build_pipeline(self):
        """Construct the Depth-Anything pipeline (seam for tests)."""

        transformers = importlib.import_module('transformers')
        return transformers.pipeline(
            task='depth-estimation',
            model=self.model_id,
            device=self.device,
        )

    def _schedule_retry(self, reason: str) -> None:
        """Drop the pipeline and arm the cooldown, warning on each occurrence."""

        self._pipeline = None
        self._retry_after = self._now_fn() + self._cooldown_s
        self.logger.warn(
            f'Depth-Anything unavailable ({reason}); retrying after '
            f'{self._cooldown_s:.0f} s cooldown.')

    def load(self) -> bool:
        if self._pipeline is not None:
            return True
        if self._now_fn() < self._retry_after:
            return False
        self.logger.info(f'Loading Depth-Anything model {self.model_id} on {self.device}')
        try:
            self._pipeline = self._build_pipeline()
        except Exception as exc:
            self._schedule_retry(f'load failed: {exc}')
            return False
        self.logger.info('Depth-Anything model loaded.')
        return True

    def produce(self, msg: Image) -> tuple[np.ndarray, object] | None:
        if not self.load():
            return None
        try:
            rgb = decode_color_to_rgb(msg)
        except ValueError as exc:
            self.logger.warn(f'Monocular source skipped a frame: {exc}')
            return None

        pil = importlib.import_module('PIL.Image')
        try:
            outputs = self._pipeline(pil.fromarray(np.ascontiguousarray(rgb)))
        except Exception as exc:
            self._schedule_retry(f'inference failed: {exc}')
            return None

        predicted = outputs.get('predicted_depth')
        if predicted is None:
            self.logger.warn('Depth-Anything returned no predicted_depth output.')
            return None
        if hasattr(predicted, 'detach'):
            depth_m = predicted.detach().cpu().numpy()
        else:
            depth_m = np.asarray(predicted)
        depth_m = np.squeeze(depth_m).astype(np.float32)
        if depth_m.ndim != 2:
            self.logger.warn(f'Depth-Anything returned unexpected shape {depth_m.shape}.')
            return None

        if depth_m.shape != (msg.height, msg.width):
            cv2 = importlib.import_module('cv2')
            depth_m = cv2.resize(
                depth_m,
                (msg.width, msg.height),
                interpolation=cv2.INTER_LINEAR,
            )
        return depth_m, msg.header


def build_depth_source(
    name: str,
    logger,
    *,
    model_id: str = DEPTH_ANYTHING_MODEL_ID_DEFAULT,
    device: str = '',
    now_fn=time.monotonic,
):
    """Dispatch the configured depth source by name.

    ``now_fn`` drives the monocular cooldown; the caller passes its node clock
    so the cooldown respects ``use_sim_time``.
    """

    key = str(name).strip().lower()
    if key == DEPTH_SOURCE_STEREOSCOPIC:
        return StereoDepthSource(logger)
    if key == DEPTH_SOURCE_MONOCULAR:
        return MonocularDepthSource(
            model_id, device, logger, now_fn=now_fn)
    raise ValueError(
        f'Unknown depth_source "{name}"; expected '
        f'"{DEPTH_SOURCE_STEREOSCOPIC}" or "{DEPTH_SOURCE_MONOCULAR}".'
    )
