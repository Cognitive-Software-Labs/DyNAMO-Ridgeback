#!/bin/bash
# Clean up stale processes and launch the office exploration stack with the
# full G1 perception pipeline enabled, including Depth-Anything.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

bash "$SCRIPT_DIR/cleanup.sh"

exec ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    world:=office \
    g1_perception_enabled:=true \
    depth_anything_enabled:=true \
    "$@"
