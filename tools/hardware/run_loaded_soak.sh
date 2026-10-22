#!/bin/bash
# S5 loaded scan/camera recurrence soak. Base e-stopped for the whole run; no
# goal is sent and autonomous motion stays off. Starts SLAM + Nav2 + external
# localization display from an Intel build prefix, then runs one guarded camera
# and scan reader for --hours and records clearpath-sensors LiDAR errors.
# Usage: tools/hardware/run_loaded_soak.sh RUN_DIR HOURS PREFIX_WORKTREE
set -u
RUN="$1"; HOURS="$2"; TREE="$3"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
mkdir -p "$RUN"; RUN="$(cd "$RUN" && pwd)"
SECONDS_TOTAL=$(python3 -c "print(int(float('$HOURS') * 3600))")
{ echo "start $(date -u +%FT%TZ) hours $HOURS"; echo "stack $(git -C "$TREE" rev-parse HEAD)";
  echo "observer $(git -C "$REPO" rev-parse HEAD) dirty $(git -C "$REPO" status --porcelain | wc -l)"; } > "$RUN/meta.txt"
SINCE="$(date '+%F %T')"
(
  set +u; cd "$TREE"
  source /etc/clearpath/setup.bash
  source artifacts/colcon/intel-rviz-install/local_setup.bash
  source src/ridgeback_autonomy_hardware/config/intel_thor/dds_env.sh intel
  export ROS_LOG_DIR="$RUN/ros-log"
  exec setsid env DISPLAY=:1 ros2 launch ridgeback_autonomy ridgeback_exploration.launch.py \
    backend:=hardware localization_mode:=external autonomous_motion_enabled:=false \
    base_frame:=base_link estimators:=projective_ranging,euclidean_reconstruction
) > "$RUN/launch.log" 2>&1 &
STACK=$!
echo "$STACK" > "$RUN/stack.pid"
until grep -qE "Managed nodes are active|Traceback|process has died" "$RUN/launch.log"; do
  sleep 5; kill -0 "$STACK" 2>/dev/null || break
done
grep -m1 -E "Managed nodes are active|Traceback|process has died" "$RUN/launch.log" > "$RUN/stack_ready.txt"
(
  set +u; cd "$REPO"; source /etc/clearpath/setup.bash; export ROS_LOG_DIR=/tmp
  python3 tools/camera_contract_check --backend hardware --guard --warmup 30 \
    --duration "$SECONDS_TOTAL" --output "$RUN/contract.json" > "$RUN/contract.log" 2>&1
)
kill -INT -- -"$STACK" 2>/dev/null; sleep 15; kill -TERM -- -"$STACK" 2>/dev/null
journalctl -u clearpath-sensors --since "$SINCE" --no-pager -o short-iso > "$RUN/clearpath-sensors.journal"
grep -cE "Could not grab|reconnecting|Could not open network Hokuyo" "$RUN/clearpath-sensors.journal" > "$RUN/lidar_error_lines.txt"
echo "end $(date -u +%FT%TZ)" >> "$RUN/meta.txt"
grep -E "CONTRACT|ABORT" "$RUN/contract.log"
