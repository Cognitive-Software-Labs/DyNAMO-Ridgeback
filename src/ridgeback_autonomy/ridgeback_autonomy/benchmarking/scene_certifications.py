"""Read the certification claims in a scenario YAML's comments and check them.

The scenario file carries, above each scene, the comment its generator wrote:
visible fractions (``vis 0.23 (band 0.15-0.45)``, ``2 robots all visible:
1.00, 1.00``, ``R0 vis 0.47, R1 vis 0.75``), edge geometry (``in-frame
0.28, center ratio 1.03``), qualitative camera/LiDAR wording (``feet clipped
by VFOV``, ``bed blocks scan plane fully``) and, for the in-box clutter block,
the LiDAR banner criteria. This module turns those comments into claims and
compares each one with a :mod:`scene_audit` result. Pure; no ROS.

``vis`` claims are compared with ``visible_fraction_floor_kept``. Targets
stand on the floor, so it equals the floor-removed ``visible_fraction`` unless
a scene buries part of a model.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import re


# Family bands the scenes were placed into, where a comment states no band.
# Edge scenes are placed for the 640x480 profile only: the frame edge sits at
# +-39.8 deg there and +-45 deg at 1280x720, so no pose is an edge case at both.
FAMILY_BANDS = {
    'edge640_mostly_in': ('in_frame_fraction', 0.99, 1.0),
    'edge640_half_out': ('in_frame_fraction', 0.20, 0.35),
    'edge640_sliver': ('in_frame_fraction', 0.005, 0.05),
}
EDGE_BAND_PROFILE = '640x480'
ALL_VISIBLE_MIN = 0.95
LEG_WIDTH_MIN = 0.60
DEPTH_CLAMP_M = 10.0
PRIMARY_VIS = 'visible_fraction_floor_kept'

_ID = re.compile(r'^\s*-\s+id:\s*(\S+)')
_BAND = re.compile(r'\(band ([\d.]+)-([\d.]+)\)')
_ALL = re.compile(r'(\d+) robots all visible:\s*([\d.,\s]+)')
_RVIS = re.compile(r'R(\d+) vis ([\d.]+)')
_VIS = re.compile(r'\bvis ([\d.]+)')
_EDGE = re.compile(r'in-frame ([\d.]+), center ratio ([\d.]+)')
_POLE = re.compile(
    r'(?:p(\d+) )?at ([\d.]+) m, (\d+) beams, (?:\+(\d+) px inside|(\d+) px outside) the box'
    r'(?:, intersects the G1 by >= ([\d.]+) cm)?')
_LEGS = re.compile(r'legs keep >= (\d+)% width|legs unshadowed')
_INFRONT = re.compile(
    r'at ([\d.]+) m \(([-+][\d.]+) m vs the legs, ~(\d+) beams\), nearest shin (\d+)% visible, '
    r'\+(\d+) px inside the box')


@dataclass(frozen=True)
class Claim:
    scene: str
    claim: str  # metric or wording key
    target: int | None
    value: float | None = None
    band: tuple[float, float] | None = None
    text: str = ''


def scene_comments(text: str) -> dict[str, str]:
    """Scene id to the comment block directly above its ``- id:`` line."""

    comments: dict[str, str] = {}
    block: list[str] = []
    for line in text.splitlines():
        stripped = line.strip()
        match = _ID.match(line)
        if match:
            comments[match.group(1)] = ' '.join(block)
            block = []
        elif stripped.startswith('#'):
            body = stripped.lstrip('#').strip()
            if body.startswith('---'):
                block = []
            else:
                block.append(body)
        elif not stripped:
            block = []
    return comments


def parse_claims(text: str) -> list[Claim]:
    """Every checkable claim in a scenario file's per-scene comments."""

    claims: list[Claim] = []
    for scene, comment in scene_comments(text).items():
        if not comment:
            continue
        add = claims.append
        band = _BAND.search(comment)
        band_t = (float(band.group(1)), float(band.group(2))) if band else None
        if (m := _ALL.search(comment)):
            for i, value in enumerate(v for v in re.split(r'[,\s]+', m.group(2)) if v):
                add(Claim(scene, 'vis', i, float(value), (ALL_VISIBLE_MIN, 1.0), comment))
        elif _RVIS.search(comment):
            for m in _RVIS.finditer(comment):
                add(Claim(scene, 'vis', int(m.group(1)), float(m.group(2)), None, comment))
        elif (m := _EDGE.search(comment)):
            fam = next((f for f in FAMILY_BANDS if scene.startswith(f)), None)
            add(Claim(scene, 'in_frame_fraction', 0, float(m.group(1)),
                      FAMILY_BANDS[fam][1:] if fam else None, comment))
            add(Claim(scene, 'center_ratio', 0, float(m.group(2)), None, comment))
        elif (m := _INFRONT.search(comment)):
            add(Claim(scene, 'pole_range_m', None, float(m.group(1)), None, comment))
            add(Claim(scene, 'pole_minus_leg_range_m', None, float(m.group(2)), None, comment))
            add(Claim(scene, 'pole_beams', None, float(m.group(3)), None, comment))
            add(Claim(scene, 'nearest_leg_width_ratio', 0, float(m.group(4)) / 100.0,
                      (LEG_WIDTH_MIN, 1.0), comment))
            add(Claim(scene, 'pole_inset_px', None, float(m.group(5)), (0.0, float('inf')),
                      comment))
        elif is_in_box_scene(scene) and _POLE.search(comment):
            for i, m in enumerate(_POLE.finditer(comment)):
                k = int(m.group(1)) if m.group(1) is not None else i
                inset = float(m.group(4)) if m.group(4) is not None else -float(m.group(5))
                add(Claim(scene, 'pole_range_m', k, float(m.group(2)), None, comment))
                add(Claim(scene, 'pole_beams', k, float(m.group(3)), None, comment))
                add(Claim(scene, 'pole_inset_px', k, inset, None, comment))
                if m.group(6) is not None:
                    add(Claim(scene, 'pole_target_clearance_m', k,
                              -float(m.group(6)) / 100.0, (-float('inf'), 0.0), comment))
            if (m := _LEGS.search(comment)):
                value = float(m.group(1)) / 100.0 if m.group(1) else 1.0
                add(Claim(scene, 'min_leg_width_ratio', None, value, (LEG_WIDTH_MIN, 1.0),
                          comment))
        elif (m := _VIS.search(comment)):
            target = 'far' if 'far robot' in comment else 0
            if band_t is None and comment.rstrip().endswith('vis 1.00'):
                band_t = (ALL_VISIBLE_MIN, 1.0)
            add(Claim(scene, 'vis', target, float(m.group(1)), band_t, comment))
        if 'feet clipped by VFOV' in comment:
            add(Claim(scene, 'feet_clipped', 0, None, None, comment))
        if 'BEYOND the depth clamp' in comment:
            add(Claim(scene, 'beyond_depth_clamp', 0, None, None, comment))
        if 'just inside the 10 m depth clamp' in comment:
            add(Claim(scene, 'inside_depth_clamp', 0, None, None, comment))
        if 'blocks scan plane fully' in comment:
            add(Claim(scene, 'scan_plane_blocked', 0, None, None, comment))
        if 'scan passes between legs' in comment:
            add(Claim(scene, 'scan_reaches_target', 0, None, None, comment))
        if 'bbox bottom lifted' in comment:
            add(Claim(scene, 'bbox_bottom_lifted', 0, None, None, comment))
    return claims


IN_BOX_PREFIXES = ('interfere_infront_', 'interfere_samerange_', 'interfere_pair_',
                   'interfere_multi_')
IN_BOX_EXTRA = ('interfere_behind_03', 'interfere_behind_04', 'interfere_behind_05',
                'interfere_control_04')


def is_in_box_scene(scene_id: str) -> bool:
    return scene_id.startswith(IN_BOX_PREFIXES) or scene_id in IN_BOX_EXTRA


def _far_index(targets: list[dict]) -> int:
    return max(range(len(targets)), key=lambda i: targets[i]['range_horizontal_m'])


def check_claims(
    claims: list[Claim],
    results: Mapping[str, dict],
    profile: str,
) -> list[dict]:
    """One row per claim: comment value, audited value, diff, band verdict."""

    rows = []
    for claim in claims:
        scene = results.get(claim.scene)
        if scene is None or profile not in scene['camera']:
            continue
        targets = scene['camera'][profile]['targets']
        lidar = scene['lidar']
        index = _far_index(targets) if claim.target == 'far' else claim.target
        target = targets[index] if isinstance(index, int) and index < len(targets) else None
        audited = None
        holds = None
        note = ''
        unreliable = False
        if claim.claim == 'vis':
            audited = target[PRIMARY_VIS]
            note = f'visible_fraction={target["visible_fraction"]}'
        elif claim.claim in ('center_ratio', 'in_frame_fraction'):
            audited = target[claim.claim]
            if target.get('crosses_near_plane') or target.get('extended_canvas_capped'):
                unreliable = True
                note = 'target crosses the near plane or the extended canvas is capped'
        elif claim.claim == 'feet_clipped':
            holds = 'bottom' in target['clipped_edges']
            note = f'clipped_edges={target["clipped_edges"]}'
        elif claim.claim in ('beyond_depth_clamp', 'inside_depth_clamp'):
            audited = target['depth_z_m']
            holds = (audited > DEPTH_CLAMP_M) == (claim.claim == 'beyond_depth_clamp')
            note = 'optical Z of the torso centre'
        elif claim.claim == 'scan_plane_blocked':
            audited = lidar['targets'][index]['beams_total']
            holds = audited == 0
        elif claim.claim == 'scan_reaches_target':
            audited = lidar['targets'][index]['beams_total']
            holds = audited > 0
        elif claim.claim == 'bbox_bottom_lifted':
            box, alone = target['bbox'], target['bbox_alone']
            holds = bool(box and alone and box[3] < alone[3])
            note = f'bbox={box} bbox_alone={alone}'
        elif claim.claim == 'min_leg_width_ratio':
            audited = min((t['min_leg_width_ratio'] for t in lidar['targets']
                           if t['min_leg_width_ratio'] is not None), default=None)
        elif claim.claim.startswith('pole') or claim.claim == 'nearest_leg_width_ratio':
            poles = [o for o in lidar['objects'] if o.get('is_pole')]
            k = claim.target if isinstance(claim.target, int) else 0
            pole = poles[k] if k < len(poles) else None
            lt = lidar['targets'][0]
            if pole is not None:
                if claim.claim == 'pole_range_m':
                    audited = pole['range_from_laser_m']
                elif claim.claim == 'pole_minus_leg_range_m':
                    if pole['beam_range_median_m'] is not None and lt['leg_range_median_m']:
                        audited = round(pole['beam_range_median_m'] - lt['leg_range_median_m'], 4)
                elif claim.claim == 'pole_beams':
                    audited = pole['beams']
                elif claim.claim == 'pole_inset_px':
                    audited = pole['camera'][profile]['inset_in_target_bbox'][0]['inset_px']
                elif claim.claim == 'pole_target_clearance_m':
                    audited = min((c['clearance_m'] for c in pole['clearance_to_targets']
                                   if c['clearance_m'] is not None), default=None)
            if claim.claim == 'nearest_leg_width_ratio':
                # A fully shadowed shin has no range, so it cannot be ranked
                # nearest; the weakest shin is the conservative reading.
                audited = min(leg['width_ratio'] for leg in lt['legs'].values())
        band = claim.band
        if claim.scene.startswith(tuple(FAMILY_BANDS)) and profile != EDGE_BAND_PROFILE:
            band = None
        if audited is not None and band is not None and holds is None and not unreliable:
            holds = band[0] <= audited <= band[1]
        diff = (None if audited is None or claim.value is None
                else round(abs(audited - claim.value), 4))
        rows.append({
            'scene': claim.scene,
            'claim': claim.claim,
            'target': index,
            'comment_value': claim.value,
            'band': list(band) if band else None,
            'audit_value': audited,
            'abs_diff': diff,
            'holds': holds,
            'note': note,
        })
    return rows


def check_in_box_banner(results: Mapping[str, dict], profile: str) -> list[dict]:
    """The in-box clutter banner criteria, per pole of every in-box scene."""

    rows = []
    for scene_id, scene in results.items():
        if not is_in_box_scene(scene_id):
            continue
        lidar = scene['lidar']
        legs_ok = all(t['legs_meet_criterion'] for t in lidar['targets'])
        min_ratio = min((t['min_leg_width_ratio'] for t in lidar['targets']
                         if t['min_leg_width_ratio'] is not None), default=None)
        for pole in (o for o in lidar['objects'] if o.get('is_pole')):
            insets = pole['camera'][profile]['inset_in_target_bbox']
            best = max(insets, key=lambda i: i['inset_px'] if i['inset_px'] is not None
                       else -1e9)
            clear_t = min((c['clearance_m'] for c in pole['clearance_to_targets']
                           if c['clearance_m'] is not None), default=None)
            clear_p = min((c['clearance_m'] for c in pole['clearance_to_other_poles']),
                          default=None)
            failures = []
            if not legs_ok:
                failures.append(f'leg width ratio {min_ratio} < {LEG_WIDTH_MIN}')
            # Control scenes place the pole outside the box on purpose.
            if not scene_id.startswith('interfere_control') and (
                    best['inset_px'] is None or best['inset_px'] <= 0):
                failures.append(f'outside every target bbox (inset {best["inset_px"]} px)')
            if clear_t is not None and clear_t <= 0:
                failures.append(f'touches a target (clearance {clear_t} m)')
            if clear_p is not None and clear_p <= 0:
                failures.append(f'touches another pole (clearance {clear_p} m)')
            if pole.get('clearance_to_sensor_robot_m') is not None and \
                    pole['clearance_to_sensor_robot_m'] <= 0:
                failures.append('touches the sensor robot')
            if pole['shadowed_by']:
                failures.append(f'in shadow of {pole["shadowed_by"]}')
            rows.append({
                'scene': scene_id,
                'pole': pole['index'],
                'pole_beams': pole['beams'],
                'pole_range_m': pole['range_from_laser_m'],
                'min_leg_width_ratio': min_ratio,
                'inset_px': best['inset_px'],
                'inset_target': best['target'],
                'clearance_to_target_m': clear_t,
                'clearance_to_other_poles_m': clear_p,
                'shadowed_by': pole['shadowed_by'],
                'holds': not failures,
                'failures': failures,
            })
    return rows
