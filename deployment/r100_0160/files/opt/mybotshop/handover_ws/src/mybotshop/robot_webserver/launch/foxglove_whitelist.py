# Builds the foxglove_bridge topic_whitelist from the same topics the web
# viewer subscribes to in robot_info.yaml, so topics are configured in one
# place. Imported by webserver.launch.py.

import re
import yaml


def _pattern(topic):
    """Anchored regex for one viewer topic, matching how it appears on the graph.

    Absolute topics (leading '/') exact-match. Relative topics are namespaced by
    the webserver before the browser subscribes (api.py prepends /<ns>/), so we
    match the suffix under any namespace.
    """
    if topic.startswith('/'):
        return '^' + re.escape(topic) + '$'
    return '^(/[^/]+)*/' + re.escape(topic) + '$'


def build_topic_whitelist(robot_info_path):
    """Return topic_whitelist regexes for the viewer topics: joint_states
    (always), point_cloud and laser_scan (only when enabled).

    Fail-safe: on any read/parse error or empty result, return a deny-all
    whitelist ('$^', matches nothing) so a config slip can never re-expose the
    raw LiDAR firehose through the bridge.
    """
    try:
        with open(robot_info_path) as f:
            info = yaml.safe_load(f) or {}
        viewer = info.get('urdf_viewer', {}) or {}

        topics = []
        joint = viewer.get('joint_states_topic')
        if joint:
            topics.append(joint)
        for key in ('point_cloud', 'laser_scan'):
            section = viewer.get(key, {}) or {}
            if section.get('enabled') and section.get('topic'):
                topics.append(section['topic'])

        patterns = [_pattern(t) for t in topics]
        return patterns if patterns else ['$^']
    except Exception:
        return ['$^']
