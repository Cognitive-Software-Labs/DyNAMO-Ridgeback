#!/bin/bash
# One recorded demonstration run: record_demo_run.sh gz|isaac SCENARIO OUT_DIR
# Runs run_scenario.sh with the Isaac task and angled view cameras and the compact RViz
# layout (Isaac headless, so its window cannot cover RViz), records it with record_demo.py (Isaac views, RViz and the mission state in
# every frame, all from this one run) and writes OUT_DIR/demo.mp4 at 8x speed.
B=$1; SC=$2; D=$(realpath -m "$3"); P=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$P/../.." && pwd)
mkdir -p "$D"; cd "$ROOT"
export ISAAC_VIEWS=${ISAAC_VIEWS:-workstation,task,isometric} DEMO_ARGS=${DEMO_ARGS:-rviz_layout:=compact headless:=true}
LIMIT=${LIMIT:-1800} bash "$P/run_scenario.sh" "$B" "$SC" "$D/run" > "$D/runner.log" 2>&1 & RUN=$!
until [ -s "$D/run/console.log" ] || ! kill -0 $RUN 2>/dev/null; do timeout 2 tail -n0 -f /dev/null; done
( source install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI="file://$PWD/install/ridgeback_autonomy/share/ridgeback_autonomy/config/parking_demo_cyclonedds.xml"
  exec python3 "$P/record_demo.py" "$D" "$B $SC ($(basename "$D"))" task isometric ) > "$D/recorder.log" 2>&1 & REC=$!
wait $RUN
kill -TERM $REC 2>/dev/null; wait $REC 2>/dev/null
ffmpeg -hide_banner -loglevel error -y -framerate 16 -i "$D/frames/%06d.jpg" -c:v libx264 -crf 30 \
  -preset slow -pix_fmt yuv420p -vf "scale=1280:-2" "$D/demo.mp4"
echo "$B $SC $(cat "$D/run/result.txt" 2>/dev/null) -> $D/demo.mp4" | tee -a "$D/demo.txt"
