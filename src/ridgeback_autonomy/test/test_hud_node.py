from __future__ import annotations

import pytest

from ridgeback_autonomy.diagnostics.hud_node import (
    PANEL_PADDING_PX,
    PLAIN_LINE_FACTOR,
    PLAIN_LINE_PAD,
    RICH_LINE_FACTOR,
    RICH_LINE_PAD,
    RICH_PANEL_PADDING_PX,
)


TEXT_SIZE = 16.0
# What the rendered overlay actually measured at this font size: nine rows of
# colour-coded distances occupying 212 px, a 27.3 px line pitch.
MEASURED_RICH_PITCH_PX = 27.3
MEASURED_RICH_TEXT_HEIGHT_PX = 216
HUD_ROWS = 9


def panel_height(rows: int, factor: float, pad: float, padding: int) -> int:
    return int(rows * (TEXT_SIZE * factor + pad) + padding)


def test_rich_line_height_matches_the_measured_pitch() -> None:
    # Taken off a screenshot, not guessed: the plain figure over-allocates by
    # about 20% per line, which is what left a third of the panel empty.
    assert TEXT_SIZE * RICH_LINE_FACTOR + RICH_LINE_PAD == pytest.approx(
        MEASURED_RICH_PITCH_PX, abs=0.5)


def test_rich_panel_still_covers_its_text() -> None:
    # Tighter must not mean clipped -- the box has to stay at least as tall as
    # the glyphs it contains, or the bottom rows are cut off.
    assert panel_height(HUD_ROWS, RICH_LINE_FACTOR, RICH_LINE_PAD,
                        RICH_PANEL_PADDING_PX) >= MEASURED_RICH_TEXT_HEIGHT_PX


MEASURED_PLAIN_PITCH_PX_AT_9PT = 14.0   # the parking HUD's rows, off a screenshot
PLAIN_HUD_ROWS = 13                      # velocity and coverage with their blank line


def test_plain_line_height_matches_the_measured_pitch_without_clipping() -> None:
    pitch = 9.0 * PLAIN_LINE_FACTOR + PLAIN_LINE_PAD
    assert MEASURED_PLAIN_PITCH_PX_AT_9PT <= pitch <= MEASURED_PLAIN_PITCH_PX_AT_9PT + 0.6


def test_plain_panel_covers_its_text_and_leaves_room_below() -> None:
    height = int(PLAIN_HUD_ROWS * (9.0 * PLAIN_LINE_FACTOR + PLAIN_LINE_PAD) + PANEL_PADDING_PX)
    text = PLAIN_HUD_ROWS * MEASURED_PLAIN_PITCH_PX_AT_9PT
    assert text <= height <= text + 2 * PANEL_PADDING_PX
