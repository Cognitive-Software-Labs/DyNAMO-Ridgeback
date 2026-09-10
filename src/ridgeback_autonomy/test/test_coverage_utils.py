from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from ridgeback_autonomy.common.coverage_utils import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    pgm_to_grid,
)

# The pixel values the ground-truth generators write (see
# tools/isaac/gt_occupancy.py: PX_FREE / PX_OCC / PX_UNKNOWN).
PX_FREE, PX_OCC, PX_UNKNOWN = 254, 0, 205

TRINARY_YAML = (
    'image: {stem}.pgm\n'
    'mode: trinary\n'
    'resolution: 0.0500\n'
    'origin: [0.0000, 0.0000, 0]\n'
    'negate: 0\n'
    'occupied_thresh: 0.65\n'
    'free_thresh: 0.196\n'
)


def _write_map(tmp_path: Path, pixels: np.ndarray, stem: str = 'm') -> Path:
    pgm = tmp_path / f'{stem}.pgm'
    cv2.imwrite(str(pgm), pixels)
    (tmp_path / f'{stem}.yaml').write_text(TRINARY_YAML.format(stem=stem))
    return pgm


def test_unknown_grey_is_not_read_as_free(tmp_path: Path) -> None:
    """205 is the conventional unknown grey, and it must not land in FREE.

    The classifier was once a hardcoded `px >= 200 -> free`, and 205 >= 200.
    Every unknown cell was then counted into the gt-free denominator that the
    coverage `complete` percentage divides by, understating coverage against
    any map with unknown space -- which is every analytically generated one,
    where everything the flood fill cannot reach is unknown by construction.
    """
    pixels = np.array([[PX_FREE, PX_UNKNOWN, PX_OCC]], dtype=np.uint8)

    grid, meta = pgm_to_grid(_write_map(tmp_path, pixels))

    assert grid.tolist() == [[FREE, UNKNOWN, OCCUPIED]]
    assert meta['resolution'] == pytest.approx(0.05)


def test_classification_round_trips_the_generator_pixel_values(tmp_path: Path) -> None:
    """Counts read back must equal the counts written, per class."""
    rng = np.random.default_rng(0)
    pixels = rng.choice(
        np.array([PX_FREE, PX_UNKNOWN, PX_OCC], dtype=np.uint8),
        size=(40, 40),
    ).astype(np.uint8)

    grid, _ = pgm_to_grid(_write_map(tmp_path, pixels))

    assert int((grid == FREE).sum()) == int((pixels == PX_FREE).sum())
    assert int((grid == UNKNOWN).sum()) == int((pixels == PX_UNKNOWN).sum())
    assert int((grid == OCCUPIED).sum()) == int((pixels == PX_OCC).sum())


def test_thresholds_come_from_the_companion_yaml(tmp_path: Path) -> None:
    """A map declaring different thresholds is classified by those, not by
    hardcoded cutoffs."""
    pgm = _write_map(tmp_path, np.array([[PX_UNKNOWN]], dtype=np.uint8))
    # p(205) = 0.196078; raising free_thresh above it reclassifies the cell.
    pgm.with_suffix('.yaml').write_text(
        TRINARY_YAML.format(stem='m').replace(
            'free_thresh: 0.196', 'free_thresh: 0.25')
    )

    grid, _ = pgm_to_grid(pgm)

    assert grid.tolist() == [[FREE]]


def test_negate_flips_the_occupancy_convention(tmp_path: Path) -> None:
    pgm = _write_map(tmp_path, np.array([[PX_FREE, PX_OCC]], dtype=np.uint8))
    pgm.with_suffix('.yaml').write_text(
        TRINARY_YAML.format(stem='m').replace('negate: 0', 'negate: 1')
    )

    grid, _ = pgm_to_grid(pgm)

    assert grid.tolist() == [[OCCUPIED, FREE]]
