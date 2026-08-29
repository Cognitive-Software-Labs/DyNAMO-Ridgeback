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

Each source also declares ``usable_max_m``: the farthest reading it can
produce that still means something. This is a property of the sensor or the
network, not of the scene, and it is deliberately separate from the working
depth gate the node applies (``depth_max_meters``). The two answer different
questions -- "can this value be believed" versus "how much of the scene do we
want to admit" -- and conflating them is what made a single 10 m constant do
background suppression as a side effect. The node takes the tighter of the
two.

Only the monocular source declares a finite one, and it derives it rather than
naming it: the metric head saturates at a known fraction of the checkpoint's
``max_depth``, so the ceiling is a fact about the loaded weights. Stereo
declares none. There is no honest number to put there -- Intel's D400
datasheet gives the D435 as "0.2 m to over 3 m (varies with lighting
conditions)" while the simulated camera is an exact render out to its 100 m
far clip -- and a made-up ceiling is not a safety net. Out-of-range readings
are already excluded by the frame contract above, which both sources satisfy:
a value past what the source can resolve arrives as 0/NaN/inf, not as a
confident number.

This module is deliberately independent of the pointcloud estimator
(pointcloud_ranging.py); it shares no code with it.
"""

from __future__ import annotations

import importlib
import math
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

# Depth-Anything's metric head is ``sigmoid(x) * config.max_depth``, so the top
# of that range is where the sigmoid saturates rather than where the scene is:
# predictions crowd toward the ceiling instead of resolving against it. Keep
# the fraction of the range where the head still discriminates and treat the
# rest as no-return.
MONOCULAR_USABLE_RANGE_FRACTION = 0.9
# Used only when the loaded config does not expose ``max_depth``; matches the
# Metric Indoor checkpoint this module defaults to.
MONOCULAR_MAX_DEPTH_FALLBACK_M = 20.0


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

    def __init__(
        self,
        logger,
        *,
        usable_max_m: float = math.inf,
    ) -> None:
        self.logger = logger
        # Unbounded by default: see the module docstring on why this source
        # names no ceiling. An operator who knows their sensor can still pass
        # one, and the node's own gate is the other way to bound the scene.
        self.usable_max_m = float(usable_max_m)

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
        # Refined from the checkpoint's own config on load; this standing value
        # only covers the window before the first successful load.
        self.usable_max_m = MONOCULAR_MAX_DEPTH_FALLBACK_M * MONOCULAR_USABLE_RANGE_FRACTION

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

    def _resolve_usable_max(self) -> float:
        """The checkpoint's usable ceiling, and the guard that it is metric at all.

        Depth-Anything ships in two flavours behind identical plumbing: the
        ``relative`` checkpoints emit affine-invariant *inverse* depth (bigger
        means nearer, no units) while the ``metric`` ones emit meters. The
        pipeline hands both back as ``predicted_depth`` and ``produce`` reads
        that as meters, so a relative checkpoint would not error -- it would
        quietly publish unitless disparity as a distance, and enough of it
        would pass the depth gate to look plausible. ``model_id`` is a free
        parameter, so refuse here rather than let that reach the estimators.

        Raises ``ValueError`` when the configured checkpoint is not metric.
        """

        config = getattr(getattr(self._pipeline, 'model', None), 'config', None)
        estimation_type = getattr(config, 'depth_estimation_type', None)
        if estimation_type is not None and str(estimation_type) != 'metric':
            raise ValueError(
                f'model "{self.model_id}" is a "{estimation_type}" depth checkpoint, '
                'which predicts unitless inverse depth rather than meters; the '
                'aligned depth frame contract needs a metric checkpoint '
                f'(e.g. "{DEPTH_ANYTHING_MODEL_ID_DEFAULT}")')
        max_depth = getattr(config, 'max_depth', None)
        if not max_depth:
            self.logger.warn(
                f'Checkpoint "{self.model_id}" exposes no max_depth; assuming '
                f'{MONOCULAR_MAX_DEPTH_FALLBACK_M:.0f} m.')
            max_depth = MONOCULAR_MAX_DEPTH_FALLBACK_M
        return float(max_depth) * MONOCULAR_USABLE_RANGE_FRACTION

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
        try:
            self.usable_max_m = self._resolve_usable_max()
        except ValueError as exc:
            # A misconfigured checkpoint is not transient, but it goes through
            # the same cooldown as any other failure rather than getting its
            # own permanent latch: the model is cached locally by now, so the
            # retry is cheap, and the operator gets the reason repeated instead
            # of once at startup where it scrolls away.
            self._schedule_retry(str(exc))
            return False
        self.logger.info(
            f'Depth-Anything model loaded; usable to {self.usable_max_m:.1f} m.')
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
