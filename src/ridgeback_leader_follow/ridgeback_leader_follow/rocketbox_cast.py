"""Microsoft Rocketbox (MIT) pedestrians used by the realistic profile.

Pure data so the fetch script, Blender bake and scene builders share one roster.
Appearance descriptions are rendering/evaluation metadata, never recognition input.
"""

REPOSITORY = 'microsoft/Microsoft-Rocketbox'
COMMIT = '0943055db6ec570bcef9f2c8b41c9e5467c808f9'
WALK = {'m': 'all_animations_max_motextr_xy/m_walk_neutral_01',
        'f': 'all_animations_max_motextr_xy/f_walk_neutral_01'}
IDLE = {'m': 'all_animations_max_motextr_static/m_idle_neutral_01',
        'f': 'all_animations_max_motextr_static/f_idle_neutral_01'}

# name: (source avatar directory, animation set, short description)
ROSTER = {
    'business_female_01': ('Professions/Business_Female_01', 'f', 'navy trouser suit'),
    'business_male_02': ('Professions/Business_Male_02', 'm', 'navy suit and tie'),
    'business_female_02': ('Professions/Business_Female_02', 'f', 'older, glasses, burgundy blazer'),
    'male_adult_12': ('Adults/Male_Adult_12', 'm', 'denim jacket and jeans'),
    'female_adult_11': ('Adults/Female_Adult_11', 'f', 'brown knit dress and boots'),
    'male_adult_03': ('Adults/Male_Adult_03', 'm', 'older, grey hair, tweed jacket'),
    'business_male_05': ('Professions/Business_Male_05', 'm', 'charcoal suit, violet shirt'),
    'female_adult_05': ('Adults/Female_Adult_05', 'f', 'pink jacket and jeans'),
    'male_adult_17': ('Adults/Male_Adult_17', 'm', 'cap, blue track jacket'),
    'business_male_04': ('Professions/Business_Male_04', 'm', 'older, waistcoat'),
    'female_party_02': ('Adults/Female_Party_02', 'f', 'curly hair, patterned top'),
    'male_adult_08': ('Adults/Male_Adult_08', 'm', 'light blue shirt'),
    'business_female_03': ('Professions/Business_Female_03', 'f', 'grey skirt suit'),
    'male_adult_14': ('Adults/Male_Adult_14', 'm', 'older, grey shirt and chinos'),
}


def raw_url(path):
    return f'https://raw.githubusercontent.com/{REPOSITORY}/{COMMIT}/{path}'
