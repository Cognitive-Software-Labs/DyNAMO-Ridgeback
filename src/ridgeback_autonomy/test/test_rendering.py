from __future__ import annotations

import types

import numpy as np
import pytest

from ridgeback_autonomy.perception.target_localization.core.rendering import (
    PANEL_ALIGNED_DEPTH,
    PANEL_BOX_MASK,
    PANEL_LIDAR,
    PANEL_RGB,
    PANEL_SILHOUETTE,
    active_label_lines,
    draw_scan_points,
    pack_panels,
    polar_highlight_beams,
    select_panels,
    truth_label_line,
)


def kinds(panels):
    return [panel.kind for panel in panels]


def test_select_panels_polar_only_is_rgb_plus_lidar() -> None:
    panels = select_panels(('polar_profiling',), 'stereoscopic', 'box')

    assert kinds(panels) == [PANEL_RGB, PANEL_LIDAR]
    lidar = next(panel for panel in panels if panel.kind == PANEL_LIDAR)
    assert lidar.title == 'Polar Profiling'


def test_select_panels_pointcloud_only_is_the_colour_frame_alone() -> None:
    # The pointcloud row measures off the cloud, which has no panel of its own.
    panels = select_panels(('pointcloud',), 'stereoscopic', 'box')

    assert kinds(panels) == [PANEL_RGB]


def test_select_panels_silhouette_proj_polar() -> None:
    panels = select_panels(
        ('projective_ranging', 'polar_profiling'), 'monocular', 'silhouette')

    assert kinds(panels) == [
        PANEL_RGB, PANEL_ALIGNED_DEPTH, PANEL_SILHOUETTE, PANEL_LIDAR]
    aligned = next(panel for panel in panels if panel.kind == PANEL_ALIGNED_DEPTH)
    assert 'monocular' in aligned.title


def test_select_panels_box_gate_uses_box_mask_not_silhouette() -> None:
    panels = select_panels(('euclidean_reconstruction',), 'stereoscopic', 'box')

    assert PANEL_BOX_MASK in kinds(panels)
    assert PANEL_SILHOUETTE not in kinds(panels)


def test_select_panels_full_set_orders_the_mask_panels() -> None:
    panels = select_panels(
        ('pointcloud', 'projective_ranging', 'euclidean_reconstruction',
         'polar_profiling'),
        'stereoscopic', 'box')

    assert kinds(panels) == [
        PANEL_RGB, PANEL_ALIGNED_DEPTH, PANEL_BOX_MASK, PANEL_LIDAR]


def _panel() -> np.ndarray:
    return np.ones((4, 6, 3), dtype=np.uint8)


def test_pack_panels_shapes() -> None:
    assert pack_panels([_panel()]).shape == (4, 6, 3)
    assert pack_panels([_panel()] * 2).shape == (4, 12, 3)
    assert pack_panels([_panel()] * 3).shape == (4, 18, 3)
    # 4 and 5 both pack into a 2x3 grid (blank cells pad the remainder).
    assert pack_panels([_panel()] * 4).shape == (8, 18, 3)
    assert pack_panels([_panel()] * 5).shape == (8, 18, 3)


def test_pack_panels_single_row_fills_width_for_a_wide_target() -> None:
    # Six panels default to 3x2 (aspect ~2:1), which letterboxes badly in a wide
    # short strip; one row is ~8:1 and fills it. Same panels, same pixels.
    tall = pack_panels([_panel()] * 6)
    wide = pack_panels([_panel()] * 6, max_cols=6)

    assert tall.shape == (8, 18, 3)
    assert wide.shape == (4, 36, 3)
    assert wide.shape[1] / wide.shape[0] > tall.shape[1] / tall.shape[0]


def test_pack_panels_single_column_is_allowed() -> None:
    assert pack_panels([_panel()] * 3, max_cols=1).shape == (12, 6, 3)


def test_pack_panels_last_row_padding_is_black() -> None:
    white = np.full((4, 6, 3), 255, dtype=np.uint8)
    grid = pack_panels([white] * 4)  # 2x3: cells 4 and 5 are blank

    # Bottom row: first cell is the real 4th panel, last two are black padding.
    assert grid[4:, :6].sum() > 0
    assert grid[4:, 6:].sum() == 0


def test_pack_panels_empty_raises() -> None:
    with pytest.raises(ValueError):
        pack_panels([])


def test_active_label_lines_only_selected_estimators() -> None:
    detection = types.SimpleNamespace(
        pointcloud_lateral_m=0.2,
        pointcloud_forward_m=2.4,
        pointcloud_distance_m=2.408,
        projective_ranging_lateral_m=-0.1,
        projective_ranging_forward_m=2.0,
        projective_ranging_distance_m=2.003,
    )

    lines = active_label_lines(detection, ('pointcloud', 'projective_ranging'))

    assert len(lines) == 2
    assert lines[0].startswith('Point Cloud') and 'd=2.41m' in lines[0]
    assert lines[1].startswith('Projective Ranging')
    assert 'x=-0.10' in lines[1] and 'z=+2.00' in lines[1] and 'd=2.00m' in lines[1]


def test_active_label_lines_missing_field_reads_na() -> None:
    lines = active_label_lines(types.SimpleNamespace(), ('polar_profiling',))

    assert lines == ['Polar Profiling d=NA']


def test_active_label_lines_follows_public_order() -> None:
    lines = active_label_lines(
        types.SimpleNamespace(), ('polar_profiling', 'pointcloud'))

    assert lines[0].startswith('Point Cloud')
    assert lines[1].startswith('Polar Profiling')


def test_truth_label_line_matches_position_line_format() -> None:
    assert truth_label_line((0.0, 2.5, 2.5)) == 'Truth x=+0.00 z=+2.50 d=2.50m'
    assert truth_label_line((-0.75, 3.5, 3.58)) == 'Truth x=-0.75 z=+3.50 d=3.58m'


def test_polar_highlight_beams_keeps_near_band_only() -> None:
    # 20 beams all projecting into the mask except the last; beams 0-9 are the
    # near object (range 1.5 m), 10-19 the far background (range 4.0 m). Only the
    # near band should highlight -- the far beams are what the estimator drops,
    # and the reason they used to light up torso/arm height.
    n = 20
    uv = np.full((n, 2), 10.0)
    uv[19] = (2.0, 2.0)                 # projects outside the mask
    in_view = np.ones(n, dtype=bool)
    in_view[0] = False                  # beam 0 not in view
    points = np.zeros((n, 3))
    points[:10, 2] = 1.5               # near (Z=1.5 -> planar range 1.5)
    points[10:, 2] = 4.0               # far
    select_mask = np.zeros((20, 20), dtype=bool)
    select_mask[5:15, 5:15] = True     # (10,10) inside, (2,2) outside

    highlight = polar_highlight_beams(uv, in_view, points, select_mask)

    assert highlight.shape == (n,)
    assert not highlight[0]             # out of view
    assert highlight[1:10].all()        # near band, in mask, in view
    assert not highlight[10:19].any()   # far background dropped by the range band
    assert not highlight[19]            # outside the mask


def test_polar_highlight_beams_none_mask_is_empty() -> None:
    highlight = polar_highlight_beams(
        np.full((5, 2), 3.0), np.ones(5, dtype=bool), np.ones((5, 3)), None)

    assert not highlight.any()


def test_draw_scan_points_draws_only_the_highlighted_beams() -> None:
    panel = np.zeros((40, 40, 3), dtype=np.uint8)
    uv = np.array([[10.0, 10.0], [30.0, 30.0]])
    highlight = np.array([True, False])

    draw_scan_points(panel, uv, highlight)

    assert panel[10, 10].tolist() == [0, 255, 255]
    # The dropped beam leaves no trace: a 3 px marker at (30, 30) would tint
    # every pixel within a couple of pixels of it.
    assert not panel[27:34, 27:34].any()


def test_draw_scan_points_empty_highlight_draws_nothing() -> None:
    panel = np.zeros((40, 40, 3), dtype=np.uint8)

    draw_scan_points(panel, np.full((5, 2), 20.0), np.zeros(5, dtype=bool))
    draw_scan_points(panel, np.full((5, 2), 20.0), None)

    assert not panel.any()
