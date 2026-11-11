#!/bin/bash
# One parking scenario run: run_scenario.sh gz|isaac SCENARIO OUT_DIR.
# Waits for a free workstation, starts start_parking_demo.sh in its own session, records a small
# bag (with both 2D scans, for parking-clearance review), samples Gazebo ground truth, takes RViz screenshots at mission stages and
# server-rendered top-down views of the scene (capture_scene_views.py) when parked and at the
# end, and stops only that session with SIGTERM. LIMIT (s) bounds the run.
B=$1; SC=$2; D=$(realpath -m "$3"); P=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$P/../.." && pwd); LIMIT=${LIMIT:-900}
: "${RIDGEBACK_PERCEPTION_VENV:?set RIDGEBACK_PERCEPTION_VENV}"; export DISPLAY="${DISPLAY:?a display is needed for screenshots}"
mkdir -p "$D/shots"; cd "$ROOT"
while pgrep -f '(^|/)(isaac_runner\.py|gzserver)( |$)|[/ ]gz sim( |$)' >/dev/null; do timeout 5 tail -n0 -f /dev/null; done
# ...and for the demo's workstation lock, which a stopping demo holds until it exits.
LOCK="${XDG_RUNTIME_DIR:-/tmp}/ridgeback-parking-${UID}.lock"
until flock -n "$LOCK" true 2>/dev/null; do timeout 2 tail -n0 -f /dev/null; done
git rev-parse HEAD > "$D/commit.txt"; git status --short >> "$D/commit.txt"; cp src/ridgeback_autonomy/config/parking_scenarios.yaml "$D/scenarios.yaml"
if [ "$B" = isaac ]; then   # the runner's opt-in top-down cameras for the stage views
  python3 -c "import json,sys; sys.path.insert(0,'$P'); import scene_views as v; json.dump([x for x in v.views('isaac','$SC') if x['name'] in ('overview','workstation')], open('$D/isaac_views.json','w'))" || exit 1
  export RIDGEBACK_ISAAC_VIEW_CAMERAS="$D/isaac_views.json"
fi
setsid bash start_parking_demo.sh $B demo_scenario:=$SC > "$D/console.log" 2>&1 &
LPID=$!
( until [ -s "$D/console.log" ]; do timeout 1 tail -n0 -f /dev/null; done; source install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI="file://$PWD/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
  exec ros2 bag record -s mcap -o "$D/bag" /r100_0001/parking_mission/state /r100_0001/parking/visualization /r100_0001/plan /r100_0001/map /r100_0001/detections/target/raw /r100_0001/measurements/target/mask /r100_0001/sensors/lidar2d_0/scan /r100_0001/sensors/lidar2d_1/scan > "$D/bag_record.log" 2>&1 ) & RPID=$!
[ "$B" = gz ] && { python3 "$P/truth_track.py" "$D" & TPID=$!; }
# FRAMES=1 also keeps camera frames (FRAMES_PERIOD s apart, FRAMES_EXT jpg about 50 KB or lossless png) for detector review.
# The saver records its own PID (frame_saver.pid) so the stop below reaches exactly it.
if [ "${FRAMES:-0}" = 1 ]; then
  ( until [ -s "$D/console.log" ]; do timeout 1 tail -n0 -f /dev/null; done; source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI="file://$PWD/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
    exec python3 "$P/frame_saver.py" "$D/frames" "${FRAMES_PERIOD:-1}" "${FRAMES_EXT:-jpg}" ) &
fi
shot() {  # the RViz window of this session; a desktop capture, checked later for blank frames
  for pid in $(ps -eo pid=,sid=,comm= | awk -v s=$LPID '$2==s && $3=="rviz2" {print $1}'); do
    for w in $(xdotool search --onlyvisible --pid $pid 2>/dev/null); do
      import -window $w "$D/shots/$1_rviz2_$w.png" 2>/dev/null; done; done
  echo "$1 $(date +%s.%N)" >> "$D/shots/stages.txt"; }
simview() {  # server-rendered views of the scene now (Gazebo cameras or Isaac view cameras)
  ( source install/setup.bash
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI="file://$PWD/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
    timeout 90 python3 "$P/capture_scene_views.py" "$B" "$SC" "$D/shots" --stage "$1" --views "$2" --no-robot-camera ) >> "$D/simviews.log" 2>&1; }
declare -A done; end=$((SECONDS + LIMIT)); result=timeout; search_at=
while [ $SECONDS -lt $end ]; do
  # Exploring: 30 s into the search for G1, if it is still searching.
  [ -z "$search_at" ] && grep -q -- "-> SEARCH_G1;" "$D/console.log" 2>/dev/null && search_at=$SECONDS
  [ -n "$search_at" ] && [ -z "${done[EXPLORING]}" ] && [ $SECONDS -ge $((search_at + 30)) ] && {
    done[EXPLORING]=1; grep -q -- "-> STOPPING_EXPLORATION;" "$D/console.log" || shot EXPLORING; }
  for st in G1_LOCALIZED NAVIGATING_TO_PREPARKING NAVIGATING_TO_PARKING PARKED; do
    [ -z "${done[$st]}" ] && grep -q -- "-> $st;" "$D/console.log" 2>/dev/null && { done[$st]=1
      # Parked for 5 s: the scene view first, while the robot still stands at its pose.
      [ $st = PARKED ] && simview parked workstation
      timeout 2 tail -n0 -f /dev/null; shot "$st"; }
  done
  r=$(grep -oE -- "-> (MISSION_COMPLETE|MISSION_FAILED|PARKING_PLANNING_FAILED|PARKING_EXECUTION_FAILED|MOTION_SETTLEMENT_FAILED)" "$D/console.log" 2>/dev/null | tail -1)
  [ -n "$r" ] && { result=${r#-> }; break; }
  kill -0 $LPID 2>/dev/null || { result=launch_exited; break; }
  timeout 2 tail -n0 -f /dev/null
done
timeout 3 tail -n0 -f /dev/null; shot "END_$result"; simview end overview,workstation
kill -TERM $RPID 2>/dev/null; pkill -TERM -P $RPID 2>/dev/null; timeout 30 tail --pid=$RPID -f /dev/null
[ -n "$TPID" ] && kill $TPID 2>/dev/null
if [ -s "$D/frame_saver.pid" ]; then FPID=$(cat "$D/frame_saver.pid")
  kill -TERM "$FPID" 2>/dev/null; timeout 10 tail --pid="$FPID" -f /dev/null; kill -KILL "$FPID" 2>/dev/null; fi
kill -TERM -- -$LPID 2>/dev/null
for k in $(seq 60); do ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . || break; timeout 1 tail -n0 -f /dev/null; done
ps -eo sid= | awk -v g=$LPID '$1==g' | grep -q . && kill -KILL -- -$LPID 2>/dev/null
echo "$SC $B $result $(ls -td artifacts/exploration/parking_*_${B}.* | head -1)" | tee "$D/result.txt"
