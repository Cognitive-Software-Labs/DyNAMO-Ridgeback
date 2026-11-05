"""Deterministic F3 outfit assignment; rendering input only, never a perception input."""
from dataclasses import dataclass
import math

SCENE_PROFILES = ('baseline', 'lab', 'f3', 'realistic')
HAIR_COLORS = ('brown', 'black', 'blonde')
# Minimum CIE76 top-garment difference between any two distinct outfits.
MIN_TOP_DELTA_E = 30.
# A look-alike wears exactly the outfit of the person it imitates.
LOOKALIKE_OF = {'lf_lookalike': 'lf_leader'}


@dataclass(frozen=True)
class Outfit:
    name: str
    top: tuple[float, float, float]
    trousers: tuple[float, float, float]
    hair: str

    def __post_init__(self):
        if not self.name.isidentifier() or self.hair not in HAIR_COLORS:
            raise ValueError('Outfit needs an identifier name and a known hair colour')
        if any(len(c) != 3 or not all(0 <= v <= 1 for v in c) for c in (self.top, self.trousers)):
            raise ValueError('Garment colours must be three sRGB values in [0, 1]')


# Albedo in sRGB. Index 0 is always the leader's outfit.
OUTFITS = (
    Outfit('cobalt_khaki', (.10, .26, .70), (.62, .54, .38), 'brown'),
    Outfit('red_navy', (.72, .08, .10), (.10, .13, .26), 'black'),
    Outfit('white_black', (.86, .86, .84), (.06, .06, .07), 'blonde'),
    Outfit('green_grey', (.12, .48, .20), (.45, .46, .47), 'brown'),
    Outfit('mustard_navy', (.85, .66, .12), (.13, .17, .33), 'black'),
    Outfit('plum_charcoal', (.48, .21, .44), (.17, .18, .21), 'brown'),
    Outfit('orange_olive', (.92, .42, .10), (.33, .35, .18), 'blonde'),
    Outfit('teal_denim', (.05, .50, .52), (.32, .43, .62), 'black'),
    Outfit('black_tan', (.07, .07, .08), (.72, .60, .44), 'blonde'),
    Outfit('pink_white', (.93, .55, .68), (.88, .87, .83), 'brown'),
)


def srgb_to_lab(color):
    """CIE L*a*b* (D65) of an sRGB colour in [0, 1]."""
    linear = [c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4 for c in color]
    matrix = ((.4124, .3576, .1805), (.2126, .7152, .0722), (.0193, .1192, .9505))
    white = (.95047, 1., 1.08883)
    xyz = [sum(m * c for m, c in zip(row, linear)) / w for row, w in zip(matrix, white)]
    f = [v ** (1 / 3) if v > (6 / 29) ** 3 else v / (3 * (6 / 29) ** 2) + 4 / 29 for v in xyz]
    return 116 * f[1] - 16, 500 * (f[0] - f[1]), 200 * (f[1] - f[2])


def delta_e(a, b):
    return math.dist(srgb_to_lab(a), srgb_to_lab(b))


def assign_outfits(scenario):
    """Map person name to outfit: leader first, look-alikes copy, others distinct.

    Assignment depends only on scenario order and names, never on randomness,
    so Gazebo and Isaac render the same outfit for the same person.
    """
    leader = next(p.name for p in scenario.people if p.role == 'leader')
    assigned = {leader: OUTFITS[0]}
    following = iter(OUTFITS[1:])
    for person in scenario.people:
        if person.name in assigned:
            continue
        model = LOOKALIKE_OF.get(person.name)
        if model is not None:
            if model not in assigned:
                raise ValueError(f'{person.name} imitates missing person {model}')
            assigned[person.name] = assigned[model]
            continue
        try:
            assigned[person.name] = next(following)
        except StopIteration:
            raise ValueError(f'{scenario.name} needs more than {len(OUTFITS)} distinct outfits') from None
    return {p.name: assigned[p.name] for p in scenario.people}
