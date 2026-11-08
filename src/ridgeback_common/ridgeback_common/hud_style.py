"""One visual language for the RViz screen overlays (rviz_2d_overlay_plugins).

The mission state-machine panel set it: monospace 9 pt text in light grey on a
near-black, 80 % opaque panel, with muted semantic colours. The exploration HUD,
the target-distance columns and the parking legend use the same tokens, so the
overlays read as one system. Marker colours in the 3D view (the parking donut
rays, estimator rings) are not HUD tokens and stay owned by their renderers.
"""

from __future__ import annotations

FONT_MONO = 'DejaVu Sans Mono'
FONT_SANS = 'DejaVu Sans'      # the legend's prose labels, which monospace would widen
TEXT_SIZE = 9.0                # points
MARGIN_PX = 10                 # distance from the view edge

BACKGROUND = (13, 13, 13)
BACKGROUND_ALPHA = 0.8
TEXT = (235, 235, 235)         # values and titles
MUTED = (125, 125, 125)        # labels, pending or stale entries
ACCENT = (80, 170, 255)        # section titles, the active state
SUCCESS = (90, 200, 120)       # completed states
WARNING = (235, 180, 80)       # degraded but running
FAILURE = (255, 85, 85)        # failed states


def rgba(rgb, alpha=1.0):
    """0-1 floats for a ColorRGBA from a 0-255 token."""
    return tuple(channel / 255.0 for channel in rgb) + (float(alpha),)


def span(text, rgb):
    """Rich-text colour for ``text``; the overlay strips "color: ...;" for its shadow."""
    return f'<span style="color: rgb{tuple(rgb)};">{text}</span>'


def apply_container(msg, *, mono=True, text_size=TEXT_SIZE):
    """Font, colours and margins shared by every overlay panel."""
    msg.font = FONT_MONO if mono else FONT_SANS
    msg.text_size = float(text_size)
    msg.horizontal_distance = msg.vertical_distance = MARGIN_PX
    for target, value in ((msg.bg_color, rgba(BACKGROUND, BACKGROUND_ALPHA)),
                          (msg.fg_color, rgba(TEXT))):
        target.r, target.g, target.b, target.a = value
    return msg
