import numpy as np
import pytest

from ridgeback_leader_follow.appearance import match_gallery, normalize_embedding


def match(candidates, gallery=((1., 0.),)):
    return match_gallery(gallery, candidates, threshold=.8, minimum_margin=.1)


def test_selects_similar_person_independent_of_feature_scale():
    result = match({7: [10., 0.], 2: [0., 3.]})
    assert result.track_id == 7
    assert result.similarity == pytest.approx(1.)


def test_identical_lookalikes_are_ambiguous_regardless_of_insertion_order():
    for candidates in ({1: [1., 0.], 2: [1., 0.]}, {2: [1., 0.], 1: [1., 0.]}):
        assert match(candidates).track_id is None
        assert match(candidates).reason == 'ambiguous'


def test_missing_invalid_or_wrong_model_features_never_match():
    assert match({1: [], 2: [np.nan, 0], 3: [0, 0], 4: [1, 0, 0]}).track_id is None
    assert normalize_embedding([np.inf, 1]) is None
    assert match({1: [0., 1.]}).reason == 'below_threshold'
    assert match({1: [1., 0.]}, gallery=()).reason == 'empty_gallery'


def test_near_tie_is_rejected_and_mixed_gallery_fails_explicitly():
    assert match({1: [1., 0.], 2: [.99, .1]}).reason == 'ambiguous'
    with pytest.raises(ValueError):
        match({1: [1., 0.]}, gallery=([1., 0.], [1., 0., 0.]))
