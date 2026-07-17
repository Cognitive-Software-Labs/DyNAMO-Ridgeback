from __future__ import annotations

import numpy as np
import pytest

from ridgeback_autonomy.perception.core.segmentation import (
    PROMPT_PADDING_REL_DEFAULT,
    SEGMENTATION_MODEL_DEFAULT,
    SamBoxSegmenter,
    binarize_mask,
    pad_prompt_box,
    select_best_masks,
)


class PrintLogger:
    def info(self, message: str) -> None:
        pass

    def warn(self, message: str) -> None:
        pass


def test_pinned_defaults() -> None:
    # The Phase 0 spike pins (see segmentation_component.md); a change here is
    # a deliberate re-pin, not a drive-by.
    assert SEGMENTATION_MODEL_DEFAULT == 'Zigeng/SlimSAM-uniform-50'
    assert PROMPT_PADDING_REL_DEFAULT == 0.05


def test_pad_prompt_box_symmetric_inflation() -> None:
    padded = pad_prompt_box((100, 200, 300, 400), 640, 480, pad_rel=0.10)

    # 10% of the 200x200 extent = 20, split 10 per side.
    assert padded == (90.0, 190.0, 310.0, 410.0)


def test_pad_prompt_box_zero_padding_is_identity() -> None:
    assert pad_prompt_box((10, 20, 30, 40), 640, 480, pad_rel=0.0) == (
        10.0, 20.0, 30.0, 40.0)


def test_pad_prompt_box_clamps_to_grid() -> None:
    padded = pad_prompt_box((0, 0, 640, 480), 640, 480, pad_rel=0.2)

    assert padded == (0.0, 0.0, 640.0, 480.0)


def test_binarize_mask_bool_passthrough() -> None:
    mask = np.array([[True, False]])

    out = binarize_mask(mask)

    assert out.dtype == np.bool_
    assert (out == mask).all()


def test_binarize_mask_float_thresholds_at_half() -> None:
    out = binarize_mask(np.array([[0.4, 0.5, 0.6]]))

    assert out.tolist() == [[False, False, True]]


def test_select_best_masks_picks_highest_scoring_option() -> None:
    # Two boxes, two options each; option 1 wins the first box, option 0 the
    # second.
    masks = np.zeros((2, 2, 4, 4), dtype=bool)
    masks[0, 1, 1, 1] = True
    masks[1, 0, 2, 2] = True
    scores = np.array([[0.2, 0.9], [0.8, 0.3]])

    selected = select_best_masks(masks, scores)

    assert len(selected) == 2
    assert selected[0][1, 1] and selected[0].sum() == 1
    assert selected[1][2, 2] and selected[1].sum() == 1


def test_select_best_masks_empty_winner_is_none() -> None:
    masks = np.zeros((1, 3, 4, 4), dtype=bool)
    scores = np.array([[0.9, 0.1, 0.1]])

    assert select_best_masks(masks, scores) == [None]


def test_select_best_masks_rejects_bad_shapes() -> None:
    with pytest.raises(ValueError, match=r'\(N, K, H, W\)'):
        select_best_masks(np.zeros((2, 4, 4), dtype=bool), np.zeros((2, 1)))
    with pytest.raises(ValueError, match='iou_scores'):
        select_best_masks(np.zeros((2, 3, 4, 4), dtype=bool), np.zeros((2, 2)))


class FakeInputs(dict):
    """The subset of a transformers BatchEncoding segment_boxes touches."""

    def to(self, device):
        return self


class FakeOutputs:
    def __init__(self, pred_masks: np.ndarray, iou_scores: np.ndarray) -> None:
        self.pred_masks = pred_masks
        self.iou_scores = iou_scores


class FakeSamStack:
    """Fake processor + model pair wired for one canned response.

    The processor records the prompts it was called with (the padding
    contract) and post-processes by passing the canned masks through, the way
    a real processor returns per-image lists.
    """

    def __init__(self, pred_masks: np.ndarray, iou_scores: np.ndarray) -> None:
        self.pred_masks = pred_masks
        self.iou_scores = iou_scores
        self.seen_input_boxes = None
        self.seen_multimask = None

    # -- processor interface --
    def __call__(self, images, input_boxes, return_tensors):
        self.seen_input_boxes = input_boxes
        return FakeInputs(original_sizes=np.array([images.shape[:2]]))

    def post_process_masks(self, masks, original_sizes):
        return [masks[0]]

    # -- model interface --
    def model(self, *, original_sizes, multimask_output):
        self.seen_multimask = multimask_output
        return FakeOutputs(self.pred_masks[np.newaxis], self.iou_scores[np.newaxis])


def build_segmenter(pred_masks: np.ndarray, iou_scores: np.ndarray):
    segmenter = SamBoxSegmenter('fake/model', PrintLogger())
    stack = FakeSamStack(pred_masks, iou_scores)
    segmenter._processor = stack
    segmenter._model = stack.model
    return segmenter, stack


def test_segment_boxes_returns_one_aligned_mask_per_box() -> None:
    masks = np.zeros((2, 2, 6, 8), dtype=bool)
    masks[0, 0, 1, 1] = True  # box 0: option 0 wins
    masks[1, 1, 3, 3] = True  # box 1: option 1 wins
    scores = np.array([[0.9, 0.2], [0.1, 0.8]])
    segmenter, stack = build_segmenter(masks, scores)
    rgb = np.zeros((6, 8, 3), dtype=np.uint8)

    out = segmenter.segment_boxes(rgb, [(1, 1, 3, 3), (2, 2, 5, 5)])

    assert len(out) == 2
    assert out[0].shape == (6, 8) and out[0].dtype == np.bool_
    assert out[0][1, 1] and out[1][3, 3]
    assert stack.seen_multimask is True


def test_segment_boxes_prompts_are_padded_and_clamped() -> None:
    masks = np.zeros((1, 1, 480, 640), dtype=bool)
    masks[0, 0, 0, 0] = True
    segmenter, stack = build_segmenter(masks, np.array([[0.9]]))
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)

    segmenter.segment_boxes(rgb, [(100, 200, 300, 400)], pad_rel=0.10)

    assert stack.seen_input_boxes == [[[90.0, 190.0, 310.0, 410.0]]]


def test_segment_boxes_empty_segmentation_is_none() -> None:
    masks = np.zeros((1, 3, 6, 8), dtype=bool)
    segmenter, _ = build_segmenter(masks, np.array([[0.9, 0.5, 0.1]]))

    out = segmenter.segment_boxes(np.zeros((6, 8, 3), dtype=np.uint8), [(1, 1, 3, 3)])

    assert out == [None]


def test_segment_boxes_no_boxes_short_circuits_without_model() -> None:
    segmenter = SamBoxSegmenter('fake/model', PrintLogger())

    # No load() call: an unloaded segmenter must handle the no-detection frame.
    assert segmenter.segment_boxes(np.zeros((6, 8, 3), dtype=np.uint8), []) == []
    assert segmenter._model is None
