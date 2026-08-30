"""Render target-localization estimator snapshots as RViz rich text."""

from __future__ import annotations

import html

from ridgeback_autonomy.perception.target_localization.estimator_registry import (
    ESTIMATOR_LABELS,
    ESTIMATOR_SHORT_LABELS,
    PUBLIC_ESTIMATOR_ORDER,
)
from ridgeback_autonomy.perception.target_localization.visualization_readings import (
    collect_readings,
)
from ridgeback_autonomy.perception.target_localization.visualization_style import (
    ESTIMATOR_COLOURS,
)


HUD_LAYOUT_ROWS = 'rows'
HUD_LAYOUT_WIDE = 'wide'
HUD_LAYOUTS = (HUD_LAYOUT_ROWS, HUD_LAYOUT_WIDE)
HUD_WIDE_CELL_COLUMNS = 10

HUD_MIN_LUMINANCE = 0.40
HUD_HEADER_COLOUR = (1.0, 1.0, 1.0)
HUD_AGED_LUMINANCE = 0.28
LUMA_WEIGHTS = (0.2126, 0.7152, 0.0722)


def _luminance(colour: tuple[float, float, float]) -> float:
    return sum(weight * channel for weight, channel in zip(LUMA_WEIGHTS, colour))


def colour_at_luminance(
    colour: tuple[float, float, float],
    target: float,
) -> tuple[float, float, float]:
    """Move a colour to a luminance while preserving its hue as far as possible."""

    luminance = _luminance(colour)
    if luminance == target:
        return colour
    if luminance > target:
        scale = target / luminance
        return tuple(channel * scale for channel in colour)
    blend = (target - luminance) / (1.0 - luminance)
    return tuple(channel + (1.0 - channel) * blend for channel in colour)


def hud_text_colour(estimator: str, aged: bool = False) -> tuple[float, float, float]:
    """Return legible HUD text in the matching estimator ring hue."""

    colour = ESTIMATOR_COLOURS[estimator][:3]
    if aged:
        return colour_at_luminance(colour, HUD_AGED_LUMINANCE)
    return colour_at_luminance(colour, max(_luminance(colour), HUD_MIN_LUMINANCE))


def hud_truth_header(truth) -> str:
    if truth is None:
        return 'TARGET DISTANCES'
    header = f'TARGET DISTANCES   truth {truth.distance_m:.3f} m'
    return f'{header}  [{truth.trial_id}]' if truth.trial_id else header


def hud_line(body: str, colour: tuple[float, float, float]) -> str:
    """Escape and colour one HUD row while preserving fixed-width padding."""

    red, green, blue = (int(round(channel * 255)) for channel in colour)
    escaped = html.escape(body, quote=False).replace(' ', '&nbsp;')
    return f'<span style="color: rgb({red}, {green}, {blue})">{escaped}</span>'


def _reading_body(label: str, distance_m: float, truth) -> str:
    error = '' if truth is None else f'  {distance_m - truth.distance_m:+.3f}'
    return f'{label:<24} {distance_m:7.3f}{error}'


def hud_rows(
    same_batch: list,
    aged: list,
    nearest: int,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> list[str]:
    return hud_rows_from_readings(
        collect_readings(same_batch, aged, nearest, estimators), truth, estimators)


def hud_rows_from_readings(
    readings: dict,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> list[str]:
    """Render one fresh, aged, or miss row per selected estimator."""

    rows = []
    for estimator in estimators:
        label = ESTIMATOR_LABELS[estimator]
        reading = readings.get(estimator)
        if reading is None:
            body = f'{label:<24}      --    miss'
            colour = hud_text_colour(estimator)
        elif reading.aged:
            body = (
                f'{_reading_body(label, reading.distance_m, truth)}'
                f'   {reading.age_s:.1f}s'
            )
            colour = hud_text_colour(estimator, aged=True)
        else:
            body = _reading_body(label, reading.distance_m, truth)
            colour = hud_text_colour(estimator)
        rows.append(hud_line(body, colour))
    return rows


def hud_section_text(
    same_batch: list,
    aged: list,
    nearest: int,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> str:
    return hud_section_from_readings(
        collect_readings(same_batch, aged, nearest, estimators),
        truth,
        estimators,
        bool(same_batch or aged),
    )


def hud_section_from_readings(
    readings: dict,
    truth,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
    has_messages: bool = True,
) -> str:
    """Render the benchmark row HUD, or ``''`` when no producer is live."""

    if not has_messages:
        return ''
    lines = [hud_line(hud_truth_header(truth), HUD_HEADER_COLOUR)]
    lines.extend(hud_rows_from_readings(readings, truth, estimators))
    return '<br/>'.join(lines)


def parse_hud_layout(raw_layout: str | None) -> str:
    """Validate the ``rows`` or ``wide`` HUD parameter."""

    layout = (raw_layout or '').strip()
    if not layout:
        return HUD_LAYOUT_ROWS
    if layout not in HUD_LAYOUTS:
        supported = ', '.join(HUD_LAYOUTS)
        raise ValueError(
            f'Unsupported hud_layout "{layout}". Expected one of: {supported}')
    return layout


def _hud_wide_cell(body: str, colour: tuple[float, float, float]) -> str:
    return hud_line(f'{body:>{HUD_WIDE_CELL_COLUMNS}}', colour)


def hud_wide_section_text(
    same_batch: list,
    aged: list,
    nearest: int,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
) -> str:
    return hud_wide_text(
        collect_readings(same_batch, aged, nearest, estimators),
        estimators,
        bool(same_batch or aged),
    )


def hud_wide_text(
    readings: dict,
    estimators: tuple[str, ...] = PUBLIC_ESTIMATOR_ORDER,
    has_messages: bool = True,
) -> str:
    """Render estimator columns for exploration, or ``''`` when empty."""

    if not has_messages:
        return ''

    headers, distances, ages = [], [], []
    for estimator in estimators:
        reading = readings.get(estimator)
        colour = hud_text_colour(
            estimator, aged=reading is not None and reading.aged)
        headers.append(_hud_wide_cell(ESTIMATOR_SHORT_LABELS[estimator], colour))
        if reading is None:
            distances.append(_hud_wide_cell('--', colour))
            ages.append(_hud_wide_cell('', colour))
            continue
        distances.append(_hud_wide_cell(f'{reading.distance_m:.3f}', colour))
        ages.append(_hud_wide_cell(f'{reading.age_s:.1f}s', colour))

    return '<br/>'.join(''.join(cells) for cells in (headers, distances, ages))
