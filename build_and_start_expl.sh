#!/bin/bash
# Build the workspace, then launch the exploration stack.
#
# Usage:
#   bash build_and_start_expl.sh                   # defaults to mock_hospital
#   bash build_and_start_expl.sh office
#   bash build_and_start_expl.sh warehouse --ros-args --log-level info

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD="${1:-mock_hospital}"
shift 2>/dev/null || true

set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build --symlink-install --base-paths "$SCRIPT_DIR/src"

exec bash "$SCRIPT_DIR/start_exploration.sh" "$WORLD" "$@"
