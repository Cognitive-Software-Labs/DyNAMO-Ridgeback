#!/bin/bash
# Build the workspace, then launch the exploration stack.
#
# Usage:
#   bash build_and_start_expl.sh                   # defaults to mock_hospital
#   bash build_and_start_expl.sh office
#   bash build_and_start_expl.sh warehouse --ros-args --log-level info
#   bash build_and_start_expl.sh estimators:=polar_profiling
#
# Arguments are forwarded untouched to start_exploration.sh, which owns the
# world / explorer / passthrough split. Splitting the world off here as well
# meant two parsers, and this one consumed a launch argument as the world.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

set +u
source /opt/ros/jazzy/setup.bash
set -u

# An absolute log base also works when this helper is invoked outside the repo.
colcon --log-base "${COLCON_LOG_PATH:-$SCRIPT_DIR/artifacts/colcon}" \
    build --symlink-install --base-paths "$SCRIPT_DIR/src"

exec bash "$SCRIPT_DIR/start_exploration.sh" "$@"
