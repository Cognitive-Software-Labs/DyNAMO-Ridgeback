#!/bin/bash
# Clean up stale processes and launch the exploration stack with the
# full G1 perception pipeline and Depth-Anything enabled.
#
# Usage:
#   bash start_exploration.sh                   # defaults to warehouse
#   bash start_exploration.sh office
#   bash start_exploration.sh hospital

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-warehouse}"
shift 2>/dev/null || true

set +u
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"
set -u

bash "$SCRIPT_DIR/cleanup.sh"

exec ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    world:="$WORLD" \
    depth_anything_enabled:=true \
    "$@"
