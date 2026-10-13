from pathlib import Path

import yaml


PARAMS = Path(__file__).resolve().parents[1] / 'config/nav2_params.yaml'
# The EKF output every backend publishes; Nav2's default "odom" has no publisher.
FILTERED_ODOM = 'platform/odom/filtered'


def test_every_nav2_odometry_reader_uses_the_filtered_platform_odometry():
    params = yaml.safe_load(PARAMS.read_text())
    for server in ('bt_navigator', 'controller_server', 'velocity_smoother'):
        assert params[server]['ros__parameters']['odom_topic'] == FILTERED_ODOM, server
