"""Numerical parity of the ROI-native paths against pre-migration results.

``mask_region_pre_migration_reference.json`` was captured by running the
estimators on the scenes below **before** masks became ROI-native, and is
checked in unchanged. That is the point of the file: comparing the two current
entry points to each other would pass even if both had drifted together, so the
expectations have to come from outside today's implementation.

Tolerances are tight on purpose. An origin that is added twice, or not at all,
moves a coordinate by tens of pixels; a tolerance loose enough to survive that
would not be testing anything.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from ridgeback_autonomy.common.miss_reason import MissReason
from ridgeback_autonomy.perception.target_localization.core.depth_common import (
    prepare_depth_region,
    valid_depth,
)
from ridgeback_autonomy.perception.target_localization.core.euclidean_reconstruction import (
    localize_euclidean_reconstruction,
    localize_prepared_euclidean_reconstruction,
)
from ridgeback_autonomy.perception.target_localization.core.intrinsics import (
    CameraIntrinsics,
    deproject_masked,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_2d import (
    ISOLATION_2D_RECIPES,
)
from ridgeback_autonomy.perception.target_localization.core.isolation_3d import (
    ISOLATION_3D_DEFAULT,
    ISOLATION_3D_NAMES,
    build_isolation_3d,
)
from ridgeback_autonomy.perception.target_localization.core.mask import (
    Mask,
    MaskPrecision,
    empty_region,
    region_from_bbox,
    region_from_blob,
)
from ridgeback_autonomy.perception.target_localization.core.polar_profiling import (
    localize_polar_profiling,
    project_scan_to_image,
    select_mask_beams,
)
from ridgeback_autonomy.perception.target_localization.core.projective_ranging import (
    localize_prepared_projective_ranging,
    localize_projective_ranging,
)


REFERENCE = json.loads(
    (Path(__file__).parent / 'mask_region_pre_migration_reference.json').read_text())

INTRINSICS = CameraIntrinsics(
    fx=320.0, fy=320.0, cx=160.0, cy=120.0, width=320, height=240)
RECT_BBOX = (195, 65, 280, 135)
# The ceiling the reference file was captured under. Pinned as a literal now
# that no shared default supplies it -- the goldens are only reproducible
# against this number.
REFERENCE_GATE_M = 10.0
TOLERANCE = 1e-9

# The camera pose the 3D recipes are built at: this robot's level mount, camera
# 1.1855 m above the floor. The reference numbers are pose-independent here --
# the deepest row of the rect box against the 6 m wall lands at optical
# Y = 0.2625, an order of magnitude under the 1.1355 m crop plane, so the crop
# keeps every point whatever the mount. A moved golden means that stopped
# holding, not that the file is stale.
CAMERA_HEIGHT_M = 1.1855
LEVEL_DOWN_OPTICAL = (0.0, 1.0, 0.0)


def build_scene(dtype=np.float32) -> np.ndarray:
    """Deterministic 240x320 depth scene: wall at 6 m, subject slab at 2.4 m."""

    depth = np.full((240, 320), 6.0, dtype=dtype)
    ramp = np.linspace(2.30, 2.55, 60, dtype=np.float64)
    depth[70:130, 200:260] = ramp[np.newaxis, :].astype(dtype)
    # Invalid returns scattered through the subject and the wall.
    depth[75, 205] = 0.0
    depth[76, 206] = np.nan
    depth[77, 207] = np.inf
    depth[78, 208] = -1.0
    depth[100, 100] = 0.0
    return depth


def build_rect_mask() -> Mask:
    """The full-grid form of ``RECT_BBOX``: a box that also catches wall."""

    data = np.zeros((240, 320), dtype=bool)
    data[65:135, 195:280] = True
    return Mask(data=data, precision=MaskPrecision.RECT)


def build_tight_blob() -> np.ndarray:
    """A silhouette with a hole and a component outside the detector box."""

    data = np.zeros((240, 320), dtype=bool)
    data[70:130, 200:260] = True
    data[90:100, 220:230] = False
    data[135:140, 262:268] = True
    return data


def build_scan() -> tuple[np.ndarray, np.ndarray]:
    """A planar scan in the camera optical frame, with two invalid beams."""

    count = 200
    angles = -0.35 + np.arange(count) * 0.0035
    ranges = np.full(count, 6.0)
    ranges[80:130] = 2.4 + 0.05 * np.sin(np.arange(50) * 0.4)
    points = np.stack(
        (ranges * np.sin(angles), np.full(count, 0.10), ranges * np.cos(angles)),
        axis=-1)
    valid = np.ones(count, dtype=bool)
    valid[5] = False
    valid[150] = False
    return points, valid


def prepare(region, depth, gate=REFERENCE_GATE_M):
    return prepare_depth_region(region, depth, valid_depth(depth, gate))


def rect_region():
    return region_from_bbox(RECT_BBOX, 240, 320)


def tight_region():
    return region_from_blob(build_tight_blob(), MaskPrecision.TIGHT)


def check_projective(name, result, reason) -> None:
    expected = REFERENCE[name]
    assert int(reason) == expected['reason'], name
    if 'xyz_optical' not in expected:
        assert result is None, name
        return
    assert result is not None, name
    assert result.foreground_pixel_count == expected['foreground_pixel_count'], name
    assert result.depth_m == pytest.approx(expected['depth_m'], abs=TOLERANCE), name
    assert result.representative_uv == pytest.approx(
        expected['representative_uv'], abs=TOLERANCE), name
    assert result.xyz_optical == pytest.approx(
        expected['xyz_optical'], abs=TOLERANCE), name


def check_euclidean(name, result, reason) -> None:
    expected = REFERENCE[name]
    assert int(reason) == expected['reason'], name
    if 'xyz_optical' not in expected:
        assert result is None, name
        return
    assert result is not None, name
    points = result.foreground_points
    assert points.shape[0] == expected['foreground_count'], name
    # First and last pin the ORDER of the point set, the sum pins its content:
    # a reordering that preserved the centroid would still fail here.
    assert points[0] == pytest.approx(expected['foreground_first'], abs=TOLERANCE), name
    assert points[-1] == pytest.approx(expected['foreground_last'], abs=TOLERANCE), name
    assert points.sum(axis=0) == pytest.approx(
        expected['foreground_sum'], rel=1e-12), name
    assert result.xyz_optical == pytest.approx(
        expected['xyz_optical'], abs=TOLERANCE), name


# --- projective ranging ----------------------------------------------------


@pytest.mark.parametrize('label,recipe', [
    ('default', None),
    *((key, ISOLATION_2D_RECIPES[key]) for key in sorted(ISOLATION_2D_RECIPES)),
])
def test_projective_rect_recipes_match_pre_migration(label, recipe) -> None:
    depth = build_scene()
    name = f'projective_rect_{label}'

    check_projective(name, *localize_projective_ranging(
        depth, build_rect_mask(), INTRINSICS, isolation=recipe, depth_max=REFERENCE_GATE_M))
    check_projective(name, *localize_prepared_projective_ranging(
        prepare(rect_region(), depth), INTRINSICS, isolation=recipe))


def test_projective_tight_matches_pre_migration() -> None:
    depth = build_scene()
    tight = Mask(data=build_tight_blob(), precision=MaskPrecision.TIGHT)

    check_projective('projective_tight', *localize_projective_ranging(
        depth, tight, INTRINSICS, depth_max=REFERENCE_GATE_M))
    check_projective('projective_tight', *localize_prepared_projective_ranging(
        prepare(tight_region(), depth), INTRINSICS))


def test_projective_finite_gate_matches_pre_migration() -> None:
    depth = build_scene()
    tight = Mask(data=build_tight_blob(), precision=MaskPrecision.TIGHT)

    check_projective('projective_tight_gated', *localize_projective_ranging(
        depth, tight, INTRINSICS, depth_max=2.45))
    check_projective('projective_tight_gated', *localize_prepared_projective_ranging(
        prepare(tight_region(), depth, 2.45), INTRINSICS))


def test_projective_unlimited_gate_matches_pre_migration() -> None:
    depth = build_scene()

    check_projective('projective_rect_unlimited', *localize_projective_ranging(
        depth, build_rect_mask(), INTRINSICS, depth_max=np.inf))
    check_projective(
        'projective_rect_unlimited', *localize_prepared_projective_ranging(
            prepare(rect_region(), depth, np.inf), INTRINSICS))


def test_projective_float64_depth_matches_pre_migration() -> None:
    depth = build_scene(np.float64)

    check_projective('projective_float64', *localize_projective_ranging(
        depth, build_rect_mask(), INTRINSICS, depth_max=REFERENCE_GATE_M))
    check_projective('projective_float64', *localize_prepared_projective_ranging(
        prepare(rect_region(), depth), INTRINSICS))


def test_projective_shortfall_keeps_too_few_valid_pixels() -> None:
    depth = build_scene()
    tight = Mask(data=build_tight_blob(), precision=MaskPrecision.TIGHT)

    check_projective('projective_tight_starved', *localize_projective_ranging(
        depth, tight, INTRINSICS, min_valid_pixels=100000, depth_max=REFERENCE_GATE_M))
    check_projective(
        'projective_tight_starved', *localize_prepared_projective_ranging(
            prepare(tight_region(), depth), INTRINSICS, min_valid_pixels=100000))


# --- euclidean reconstruction ----------------------------------------------


@pytest.mark.parametrize('label,recipe_name', [
    ('default', ISOLATION_3D_DEFAULT),
    *((key, key) for key in sorted(ISOLATION_3D_NAMES)),
])
def test_euclidean_rect_recipes_match_pre_migration(label, recipe_name) -> None:
    depth = build_scene()
    name = f'euclidean_rect_{label}'
    recipe = build_isolation_3d(recipe_name, CAMERA_HEIGHT_M, LEVEL_DOWN_OPTICAL)

    check_euclidean(name, *localize_euclidean_reconstruction(
        depth, build_rect_mask(), INTRINSICS, isolation=recipe, depth_max=REFERENCE_GATE_M))
    check_euclidean(name, *localize_prepared_euclidean_reconstruction(
        prepare(rect_region(), depth), INTRINSICS, isolation=recipe))


def test_euclidean_tight_matches_pre_migration() -> None:
    depth = build_scene()
    tight = Mask(data=build_tight_blob(), precision=MaskPrecision.TIGHT)

    check_euclidean('euclidean_tight', *localize_euclidean_reconstruction(
        depth, tight, INTRINSICS, depth_max=REFERENCE_GATE_M))
    check_euclidean('euclidean_tight', *localize_prepared_euclidean_reconstruction(
        prepare(tight_region(), depth), INTRINSICS))


def test_euclidean_finite_gate_matches_pre_migration() -> None:
    depth = build_scene()
    tight = Mask(data=build_tight_blob(), precision=MaskPrecision.TIGHT)

    check_euclidean('euclidean_tight_gated', *localize_euclidean_reconstruction(
        depth, tight, INTRINSICS, depth_max=2.45))
    check_euclidean(
        'euclidean_tight_gated', *localize_prepared_euclidean_reconstruction(
            prepare(tight_region(), depth, 2.45), INTRINSICS))


def test_euclidean_float64_depth_matches_pre_migration() -> None:
    depth = build_scene(np.float64)
    recipe = build_isolation_3d(
        ISOLATION_3D_DEFAULT, CAMERA_HEIGHT_M, LEVEL_DOWN_OPTICAL)

    check_euclidean('euclidean_float64', *localize_euclidean_reconstruction(
        depth, build_rect_mask(), INTRINSICS, isolation=recipe, depth_max=REFERENCE_GATE_M))
    check_euclidean('euclidean_float64', *localize_prepared_euclidean_reconstruction(
        prepare(rect_region(), depth), INTRINSICS, isolation=recipe))


def test_euclidean_shortfall_keeps_too_few_valid_points() -> None:
    depth = build_scene()

    check_euclidean('euclidean_rect_starved', *localize_euclidean_reconstruction(
        depth, build_rect_mask(), INTRINSICS, min_valid_points=100000, depth_max=REFERENCE_GATE_M))
    check_euclidean(
        'euclidean_rect_starved', *localize_prepared_euclidean_reconstruction(
            prepare(rect_region(), depth), INTRINSICS, min_valid_points=100000))


def test_isolation_shortfall_keeps_its_own_reason_on_both_paths() -> None:
    """An emptied isolation is TOO_FEW_AFTER_ISOLATION, not a validity shortfall."""

    depth = build_scene()
    empty_2d = (lambda depth_arg, mask_arg, **kwargs: np.zeros_like(mask_arg))
    empty_3d = (lambda points: np.zeros(points.shape[0], dtype=bool))

    _, reason = localize_projective_ranging(
        depth, build_rect_mask(), INTRINSICS, isolation=empty_2d, depth_max=REFERENCE_GATE_M)
    assert reason is MissReason.TOO_FEW_AFTER_ISOLATION
    _, reason = localize_prepared_projective_ranging(
        prepare(rect_region(), depth), INTRINSICS, isolation=empty_2d)
    assert reason is MissReason.TOO_FEW_AFTER_ISOLATION

    _, reason = localize_euclidean_reconstruction(
        depth, build_rect_mask(), INTRINSICS, isolation=empty_3d, depth_max=REFERENCE_GATE_M)
    assert reason is MissReason.TOO_FEW_AFTER_ISOLATION
    _, reason = localize_prepared_euclidean_reconstruction(
        prepare(rect_region(), depth), INTRINSICS, isolation=empty_3d)
    assert reason is MissReason.TOO_FEW_AFTER_ISOLATION


def test_empty_region_is_a_selector_with_zero_pixels_not_a_crash() -> None:
    depth = build_scene()

    for precision in (MaskPrecision.RECT, MaskPrecision.TIGHT):
        prepared = prepare(empty_region(240, 320, precision), depth)
        assert prepared.valid_masked.shape == (0, 0)

        # Both tags, and both paths: an empty region held nothing to isolate.
        result, reason = localize_prepared_projective_ranging(prepared, INTRINSICS)
        assert result is None
        assert reason is MissReason.TOO_FEW_VALID_PIXELS

        result, reason = localize_prepared_euclidean_reconstruction(prepared, INTRINSICS)
        assert result is None
        assert reason is MissReason.TOO_FEW_VALID_POINTS


# --- polar profiling -------------------------------------------------------


@pytest.mark.parametrize('label', ['rect', 'tight'])
def test_polar_beam_parity_across_both_representations(label) -> None:
    points, valid = build_scan()
    projection = project_scan_to_image(points, valid, INTRINSICS)
    if label == 'rect':
        selectors = (build_rect_mask(), rect_region())
    else:
        selectors = (
            Mask(data=build_tight_blob(), precision=MaskPrecision.TIGHT),
            tight_region(),
        )
    expected = REFERENCE[f'polar_{label}']

    for selector in selectors:
        assert select_mask_beams(projection, selector).tolist() == (
            expected['selected_beams'])
        result, reason = localize_polar_profiling(points, valid, selector, INTRINSICS)
        assert int(reason) == expected['reason']
        assert result.selected_beams.tolist() == expected['selected_beams']
        assert result.merged_beams.tolist() == expected['merged_beams']
        assert result.xz_optical == pytest.approx(
            expected['xz_optical'], abs=TOLERANCE)


def test_polar_projection_keeps_every_original_beam_row() -> None:
    """``beam_indices`` addresses the ORIGINAL scan, so invalid beams just drop out."""

    points, valid = build_scan()

    projection = project_scan_to_image(points, valid, INTRINSICS)

    assert projection.points_optical.shape == (points.shape[0], 3)
    assert 5 not in projection.beam_indices
    assert 150 not in projection.beam_indices


def test_polar_selection_is_independent_per_overlapping_region() -> None:
    points, valid = build_scan()
    projection = project_scan_to_image(points, valid, INTRINSICS)
    left = region_from_bbox((150, 60, 240, 140), 240, 320)
    right = region_from_bbox((220, 60, 320, 140), 240, 320)

    selected_left = select_mask_beams(projection, left)
    selected_right = select_mask_beams(projection, right)

    assert selected_left.size and selected_right.size
    assert not np.array_equal(selected_left, selected_right)
    overlap = region_from_bbox((220, 60, 240, 140), 240, 320)
    assert set(select_mask_beams(projection, overlap).tolist()) == (
        set(selected_left.tolist()) & set(selected_right.tolist()))


def test_polar_empty_region_selects_no_beams_and_still_reports() -> None:
    points, valid = build_scan()
    projection = project_scan_to_image(points, valid, INTRINSICS)

    region = empty_region(240, 320, MaskPrecision.TIGHT)

    assert select_mask_beams(projection, region).size == 0
    result, reason = localize_polar_profiling(points, valid, region, INTRINSICS)
    assert result is None
    assert reason is MissReason.TOO_FEW_RAYS_SELECTED


# --- deprojection helper ---------------------------------------------------


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
def test_deproject_masked_gathers_before_widening(dtype) -> None:
    depth = build_scene(dtype)
    rows, cols = np.nonzero(build_tight_blob())

    gathered = deproject_masked(depth, rows, cols, INTRINSICS)
    widened_first = np.asarray(depth, dtype=np.float64)[rows, cols]

    assert gathered.dtype == np.float64
    # Bit-exact, NaN slots included: the selection deliberately covers the
    # scene's 0 / NaN / inf / negative returns.
    assert np.array_equal(gathered[:, 2], widened_first, equal_nan=True)


def test_deproject_masked_handles_strided_input_and_empty_selections() -> None:
    depth = build_scene(np.float32)[:, ::2]
    rows = np.array([10, 20, 30])
    cols = np.array([5, 6, 7])

    points = deproject_masked(depth, rows, cols, INTRINSICS)
    assert np.array_equal(points[:, 2], depth[rows, cols].astype(np.float64))

    empty = deproject_masked(
        depth, np.empty(0, dtype=np.intp), np.empty(0, dtype=np.intp), INTRINSICS)
    assert empty.shape == (0, 3)
