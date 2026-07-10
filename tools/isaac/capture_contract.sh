#!/usr/bin/env bash
# Capture the ROS topic contract from a RUNNING exploration stack.
#
# Records, for every topic Isaac Sim must reproduce, the ground truth the
# port is held to: QoS profiles, message types, publish rates, one
# camera_info sample per camera stream, and the TF tree. Run it while a
# `start_exploration.sh` session is up (any world); results are committed
# under tools/isaac/baseline/contract/ as the P0 contract freeze.
#
# Usage: tools/isaac/capture_contract.sh [namespace] [output_dir]
set -euo pipefail

NS="${1:-r100_0001}"
OUT="${2:-$(git rev-parse --show-toplevel)/tools/isaac/baseline/contract}"
HZ_SECONDS=12

TOPICS=(
  "/$NS/sensors/lidar2d_0/scan"
  "/$NS/sensors/lidar2d_1/scan"
  "/$NS/sensors/camera_0/color/image"
  "/$NS/sensors/camera_0/depth/image"
  "/$NS/sensors/camera_0/points"
  "/$NS/platform/odom/filtered"
  "/$NS/cmd_vel"
  "/clock"
  "/$NS/tf"
  "/$NS/tf_static"
)
CAMERA_INFO_TOPICS=(
  "/$NS/sensors/camera_0/color/camera_info"
  "/$NS/sensors/camera_0/depth/camera_info"
)
HZ_TOPICS=(
  "/$NS/sensors/lidar2d_0/scan"
  "/$NS/sensors/camera_0/color/image"
  "/$NS/platform/odom/filtered"
  "/clock"
)

mkdir -p "$OUT"

echo "== capture_contract: namespace=$NS out=$OUT"
ros2 topic list > "$OUT/topic_list.txt"

: > "$OUT/topic_info.txt"
for t in "${TOPICS[@]}" "${CAMERA_INFO_TOPICS[@]}"; do
  echo "### $t" >> "$OUT/topic_info.txt"
  if ! ros2 topic info -v "$t" >> "$OUT/topic_info.txt" 2>&1; then
    echo "MISSING: $t" | tee -a "$OUT/topic_info.txt"
  fi
  echo >> "$OUT/topic_info.txt"
done

for t in "${CAMERA_INFO_TOPICS[@]}"; do
  name="$(echo "$t" | tr '/' '_' | sed 's/^_//')"
  timeout 15 ros2 topic echo --once "$t" > "$OUT/${name}.yaml" \
    || echo "MISSING: $t" > "$OUT/${name}.yaml"
done

: > "$OUT/topic_hz.txt"
for t in "${HZ_TOPICS[@]}"; do
  echo "### $t" >> "$OUT/topic_hz.txt"
  timeout "$HZ_SECONDS" ros2 topic hz --window 100 "$t" >> "$OUT/topic_hz.txt" 2>&1 || true
  echo >> "$OUT/topic_hz.txt"
done

# view_frames writes frames_<stamp>.{gv,pdf} into CWD; keep only the .gv
( cd "$OUT" \
  && timeout 30 ros2 run tf2_tools view_frames --ros-args -r __ns:="/$NS" \
       -r /tf:=tf -r /tf_static:=tf_static >/dev/null 2>&1 || true
  latest_gv=$(ls -t frames_*.gv 2>/dev/null | head -1 || true)
  if [ -n "$latest_gv" ]; then
    mv "$latest_gv" tf_tree.gv
    rm -f frames_*.gv frames_*.pdf
  else
    echo "tf capture failed" > tf_tree.gv
  fi )

echo "== done. Files:"
ls -l "$OUT"
