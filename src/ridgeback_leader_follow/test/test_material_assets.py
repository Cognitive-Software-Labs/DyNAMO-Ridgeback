"""Source normal/roughness channels must survive conversion to the renderer."""
import pytest
from ridgeback_leader_follow.material_assets import material_specs
from ridgeback_leader_follow.realistic_scene import human_directory,asset_directory


def test_casual_fabric_keeps_source_normal_strength_and_specular():
    specs=material_specs(human_directory('sam'))
    shirt=next(m for m in specs if 'original top' in m['name'])
    assert shirt['normal_image'] and shirt['normal_scale']==pytest.approx(.5)
    assert shirt['specular_factor']==pytest.approx(.5)
    assert (human_directory('sam')/shirt['normal_image']).is_file()


def test_all_realistic_asset_texture_references_resolve():
    directories=[human_directory(m) for m in ('sam','maya','harper','jordan')]
    directories+=list((asset_directory()/'props').iterdir())
    packed=0
    for directory in directories:
        for spec in material_specs(directory):
            for key in ('image','normal_image','orm_image','opacity_image'):
                if key in spec:assert (directory/spec[key]).is_file()
            packed+=int('orm_image' in spec)
    assert packed>0


def test_dark_business_fabric_is_normalized_to_the_cast_colour():
    import numpy as np
    from ridgeback_leader_follow.material_assets import texture_tint_scale
    directory=human_directory('harper')
    fabric=next(m for m in material_specs(directory) if 'business fabric' in m['name'])
    scale=texture_tint_scale(directory/fabric['image'],(.42,.09,.08))
    assert np.isfinite(scale).all() and min(scale)>0
    assert max(scale)>1.  # Dark source cloth must not make every assigned suit black.


def test_vegetation_has_real_opacity_masks():
    for model in ('grass_medium_01','shrub_01'):
        specs=material_specs(asset_directory()/'props'/model)
        assert all(spec.get('opacity_image') for spec in specs)
