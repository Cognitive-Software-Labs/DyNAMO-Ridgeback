"""Segmentation component: the ``tight`` (silhouette) mask front-end.

A box-promptable segmenter (SAM family) prompted with the OWLv2 detection
boxes -- one box prompt, one pixel-precise mask -- keeping the detector's
open-vocabulary property and the 1:1:1 frame -> detection -> mask hierarchy
(``mask_component.md`` Section 6). The consuming node wraps each returned
blob with ``mask_from_array(blob, MaskPrecision.TIGHT)``; this module returns
plain arrays and knows nothing about producers or consumers, same as
``rasterize_*`` on the rect side.

Cost split (``mask_component.md`` Section 6): all boxes of a frame go through
**one** forward pass -- the image encoder runs once per frame, the prompt
decoder once per box.

See ``object_localization_documentation/segmentation_component.md``. Import
of this module is torch-free; the model libraries load lazily in
``SamBoxSegmenter.load()`` so the ``box`` gate never pays for them.
"""

from __future__ import annotations

import contextlib
import importlib

import numpy as np

from ridgeback_autonomy.perception.core.detection import resolve_torch_device


# Pinned by the Phase 0 spike (see segmentation_component.md): silhouette
# fidelity on the G1's legs at benchmark distances, latency, and VRAM.
SEGMENTATION_MODEL_DEFAULT = 'Zigeng/SlimSAM-uniform-50'

# Relative inflation of the prompt box before segmenting: OWLv2 boxes can clip
# the object, and a SAM box prompt truncates hard at its boundary. Padding
# widens only the prompt -- the segmenter still returns the object silhouette
# inside it, so the mask does not inherit the padding.
PROMPT_PADDING_REL_DEFAULT = 0.05

# Floor on SAM's predicted-IoU: masks scoring below it are dropped (-> None ->
# that detection skips, no-fallback). CAVEAT: predicted-IoU rates mask *boundary*
# quality, NOT whether the mask is the robot -- a cleanly-segmented wall can
# score high. So this drops low-quality/uncertain masks (which correlate with
# bad prompt boxes) but will NOT catch a crisp wrong-object mask; full
# correctness would need a depth-consistency or class check (out of scope).
# Spike range for good G1 masks was 0.94-0.99 (segmentation_component.md); the
# audit suggested ~0.7 (S-C6). A conservative 0.5 default leaves headroom for
# real D435 imagery scoring below sim -- a tunable knob, not a magic truth.
SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT = 0.5


def pad_prompt_box(
    bbox_xyxy: tuple[float, float, float, float],
    image_width: int,
    image_height: int,
    pad_rel: float = PROMPT_PADDING_REL_DEFAULT,
) -> tuple[float, float, float, float]:
    """Inflate a prompt box by ``pad_rel`` of its size, clamped to the grid.

    The inflation is symmetric: each side moves out by half of
    ``pad_rel * extent`` along its axis. A zero padding returns the box
    unchanged (modulo the clamp).
    """

    x1, y1, x2, y2 = (float(value) for value in bbox_xyxy)
    dx = (x2 - x1) * pad_rel / 2.0
    dy = (y2 - y1) * pad_rel / 2.0
    return (
        max(0.0, x1 - dx),
        max(0.0, y1 - dy),
        min(float(image_width), x2 + dx),
        min(float(image_height), y2 + dy),
    )


def binarize_mask(mask: np.ndarray) -> np.ndarray:
    """Collapse a segmenter mask (bool or float logits/probabilities) to bool.

    Float masks binarize at 0.5 -- above the sigmoid midpoint counts as
    object. Boolean masks pass through unchanged.
    """

    mask = np.asarray(mask)
    if mask.dtype == np.bool_:
        return mask
    return mask > 0.5


def select_best_masks(
    masks: np.ndarray,
    iou_scores: np.ndarray,
    min_predicted_iou: float = 0.0,
) -> list[np.ndarray | None]:
    """Resolve multimask output to one mask per box: highest predicted IoU.

    ``masks`` is ``(N, K, H, W)`` (N boxes, K mask options each; bool or
    float), ``iou_scores`` is ``(N, K)``. Returns N entries, each an
    ``(H, W)`` boolean array, or ``None`` where the winning option is empty
    **or scores below** ``min_predicted_iou`` -- the caller's signal to skip
    that detection (no-fallback convention: a failed segmentation drops the
    trial, it is never patched over). The floor lives here because this is
    where the score exists (S-9); the default 0.0 keeps the library call
    permissive, the node supplies the production floor. See
    ``SEGMENTATION_MIN_PREDICTED_IOU_DEFAULT`` for the quality-vs-correctness
    caveat.
    """

    masks = np.asarray(masks)
    iou_scores = np.asarray(iou_scores)
    if masks.ndim != 4:
        raise ValueError(f'masks must be (N, K, H, W); got shape {masks.shape}')
    if iou_scores.shape != masks.shape[:2]:
        raise ValueError(
            f'iou_scores shape {iou_scores.shape} does not match '
            f'mask options {masks.shape[:2]}')

    selected: list[np.ndarray | None] = []
    for box_index in range(masks.shape[0]):
        best = int(np.argmax(iou_scores[box_index]))
        score = float(iou_scores[box_index, best])
        mask = binarize_mask(masks[box_index, best])
        keep = mask.any() and score >= min_predicted_iou
        selected.append(mask if keep else None)
    return selected


class SamBoxSegmenter:
    """Box-prompted segmentation model behind one call: boxes in, blobs out.

    Mirrors ``OwlV2Detector``: constructed cheap, the model loads lazily on
    first use, and a missing venv raises one actionable ``RuntimeError``
    instead of crashing the node. Supports both SAM (``facebook/sam-vit-*``,
    SlimSAM) and SAM 2 (``facebook/sam2.1-hiera-*``) checkpoints -- the
    family is dispatched on the checkpoint's ``model_type``.
    """

    def __init__(self, model_name: str, logger) -> None:
        self.model_name = model_name
        self.logger = logger
        self.device = resolve_torch_device()
        self._model = None
        self._processor = None
        # Replaced by torch.inference_mode in load(); a no-op context keeps
        # segment_boxes torch-free for fake-model tests.
        self._inference_mode = contextlib.nullcontext

    def load(self) -> None:
        try:
            transformers = importlib.import_module('transformers')
        except ModuleNotFoundError as exc:
            raise RuntimeError(
                "G1 segmentation needs 'transformers' (and torch). Set up the "
                "perception_venv — see README 'Set up G1 perception venv' — "
                "then relaunch, or run with mask_gate:=box which needs no "
                "model."
            ) from exc
        self.logger.info(f'Loading segmentation model: {self.model_name}')
        self.logger.info(f'Using segmentation device: {self.device}')
        model_class, processor_class = self._model_classes(transformers)
        self._model = model_class.from_pretrained(self.model_name).to(self.device).eval()
        self._processor = processor_class.from_pretrained(self.model_name)
        self._inference_mode = importlib.import_module('torch').inference_mode
        self.logger.info('Segmentation model loaded.')

    def _model_classes(self, transformers) -> tuple[type, type]:
        config = transformers.AutoConfig.from_pretrained(self.model_name)
        model_type = str(getattr(config, 'model_type', ''))
        if model_type.startswith('sam2'):
            return transformers.Sam2Model, transformers.Sam2Processor
        return transformers.SamModel, transformers.SamProcessor

    def segment_boxes(
        self,
        rgb: np.ndarray,
        boxes_xyxy: list[tuple[float, float, float, float]],
        pad_rel: float = PROMPT_PADDING_REL_DEFAULT,
        min_predicted_iou: float = 0.0,
    ) -> list[np.ndarray | None]:
        """Segment one silhouette per box on one RGB frame.

        ``rgb`` is the ``(H, W, 3)`` uint8 color image the boxes live on.
        Returns one entry per box, index-aligned: an ``(H, W)`` boolean blob
        on the same grid, or ``None`` for an empty or low-confidence
        segmentation (below ``min_predicted_iou``). One model forward for the
        whole frame (encoder once, decoder per box).

        Precondition: ``load()`` has been called (the node eager-loads in
        ``__init__``). An empty box list short-circuits before any model use.
        """

        if not boxes_xyxy:
            return []
        # S-4: the model is loaded eagerly by the node; a live None here is a
        # wiring bug, so fail loud rather than the cryptic ``None(**inputs)``.
        # (The detector's detect() carries the identical dead lazy-load, D-6 --
        # out of scope here.)
        if self._model is None:
            raise RuntimeError('segmenter not loaded; call load() first')

        height, width = rgb.shape[:2]
        prompts = [
            list(pad_prompt_box(box, width, height, pad_rel))
            for box in boxes_xyxy
        ]

        inputs = self._processor(
            images=rgb,
            input_boxes=[prompts],
            return_tensors='pt',
        ).to(self.device)
        with self._inference_mode():
            outputs = self._model(**inputs, multimask_output=True)

        masks = self._post_process(inputs, outputs)
        iou_scores = _to_numpy(outputs.iou_scores)
        # Per-image batch of one: (N boxes, K options, H, W) and (N, K).
        return select_best_masks(
            masks, iou_scores.reshape(masks.shape[:2]), min_predicted_iou)

    def _post_process(self, inputs, outputs) -> np.ndarray:
        """Upscale predicted masks back to the original grid, as (N, K, H, W)."""

        kwargs = {}
        # SAM 1 processors also need the pre-padding resized shape; SAM 2
        # processors take original sizes only.
        reshaped = inputs.get('reshaped_input_sizes')
        if reshaped is not None:
            kwargs['reshaped_input_sizes'] = _to_cpu(reshaped)
        masks = self._processor.post_process_masks(
            _to_cpu(outputs.pred_masks),
            _to_cpu(inputs['original_sizes']),
            **kwargs,
        )
        return np.asarray(masks[0])


def _to_cpu(value):
    """Torch tensor -> detached CPU tensor; anything else passes through."""

    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    return value


def _to_numpy(value) -> np.ndarray:
    return np.asarray(_to_cpu(value))
