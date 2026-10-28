"""Generate an opt-in Nav2 qualification profile; never start nodes or drive.

The shared YAML is read-only. Group review is required before using the
generated keep-out filters and explicit checker selection in production.
"""

from copy import deepcopy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


def parking_tree_identity(path, planner_id='GridBased'):
    """Admit only the staged two-action tree; this does not qualify its route."""
    try:
        path = Path(path)
        if not path.is_absolute() or path.stat().st_size > 65536:
            return None, 'parking_bt_path_invalid'
        content = path.read_bytes()
        root = ET.fromstring(content)
    except (OSError, ValueError, TypeError, ET.ParseError):
        return None, 'parking_bt_unavailable_or_invalid'
    expected = ET.fromstring('''<root BTCPP_format="4" main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree"><Sequence>
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"
          error_code_id="{compute_path_error_code}"/>
        <FollowPath path="{path}" controller_id="FollowPath"
          goal_checker_id="parking_goal_checker" error_code_id="{follow_path_error_code}"/>
      </Sequence></BehaviorTree></root>''')
    expected.find('.//ComputePathToPose').set('planner_id', planner_id)
    def signature(element):
        return (element.tag, element.attrib, (element.text or '').strip(),
                tuple(signature(child) for child in element))
    if signature(root) != signature(expected):
        return None, 'parking_bt_contract_mismatch'
    return hashlib.sha256(content).hexdigest(), None


def explicit_general_tree(xml):
    root = ET.fromstring(xml)
    for node in root.iter('FollowPath'):
        node.set('goal_checker_id', 'general_goal_checker')
    return ET.tostring(root, encoding='unicode')


def build_profile(base, namespace, general_tree, through_tree, parking_overlay):
    if not namespace.strip('/') or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_/'
                                       for c in namespace):
        raise ValueError('explicit valid robot namespace required')
    params = deepcopy(base)
    controller = params['controller_server']['ros__parameters']
    if controller['goal_checker_plugins'] != ['general_goal_checker']:
        raise ValueError('unreviewed shared goal-checker layout')
    overlay = parking_overlay['controller_server']['ros__parameters']
    if (set(overlay) != {'goal_checker_plugins', 'parking_goal_checker'}
            or overlay['goal_checker_plugins'] != ['general_goal_checker', 'parking_goal_checker']):
        raise ValueError('unreviewed parking goal-checker overlay')
    controller.update(deepcopy(overlay))
    navigator = params['bt_navigator']['ros__parameters']
    navigator['default_nav_to_pose_bt_xml'] = str(general_tree)
    navigator['default_nav_through_poses_bt_xml'] = str(through_tree)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        costmap.setdefault('filters', []).append('parking_g1_keepout')
        costmap['parking_g1_keepout'] = dict(plugin='nav2_costmap_2d::KeepoutFilter', enabled=True,
            filter_info_topic=f'/{namespace.strip("/")}/parking/g1_keepout_info')
    return {namespace.strip('/'): params}


def goal_checker_reason(values, xy=0.08, yaw=0.12):
    required = {'goal_checker_plugins': ['general_goal_checker', 'parking_goal_checker'],
        'general_goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
        'general_goal_checker.xy_goal_tolerance': 0.40,
        'general_goal_checker.yaw_goal_tolerance': 0.40,
        'general_goal_checker.stateful': True,
        'parking_goal_checker.plugin': 'nav2_controller::SimpleGoalChecker',
        'parking_goal_checker.xy_goal_tolerance': xy,
        'parking_goal_checker.yaw_goal_tolerance': yaw,
        'parking_goal_checker.stateful': False}
    for name, expected in required.items():
        if values.get(name) != expected:
            return f'goal_checker_unavailable_or_mismatched: {name}'
    return None


GOAL_CHECKER_PARAMETERS = ('goal_checker_plugins',
    'general_goal_checker.plugin', 'general_goal_checker.xy_goal_tolerance',
    'general_goal_checker.yaw_goal_tolerance', 'general_goal_checker.stateful',
    'parking_goal_checker.plugin', 'parking_goal_checker.xy_goal_tolerance',
    'parking_goal_checker.yaw_goal_tolerance', 'parking_goal_checker.stateful')


def write_profile(output, namespace):
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('ridgeback_autonomy'))
    trees = Path(get_package_share_directory('nav2_bt_navigator')) / 'behavior_trees'
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    to_pose, through = output/'general_to_pose.xml', output/'general_through_poses.xml'
    for src, dst in [('navigate_to_pose_w_replanning_and_recovery.xml', to_pose),
                     ('navigate_through_poses_w_replanning_and_recovery.xml', through)]:
        dst.write_text(explicit_general_tree((trees/src).read_text()), encoding='utf-8')
    profile = build_profile(yaml.safe_load((share/'config/nav2_params.yaml').read_text()),
        namespace, to_pose, through,
        yaml.safe_load((share/'config/parking_goal_checker.yaml').read_text()))
    result = output/'nav2_parking_qualification.yaml'
    result.write_text(yaml.safe_dump(profile, sort_keys=False), encoding='utf-8')
    return result


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--output', required=True)
    parser.add_argument('--namespace', required=True)
    args = parser.parse_args()
    print(write_profile(args.output, args.namespace))


if __name__ == '__main__':
    main()
