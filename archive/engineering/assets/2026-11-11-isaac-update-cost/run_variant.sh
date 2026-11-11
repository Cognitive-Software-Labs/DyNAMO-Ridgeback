#!/bin/bash
# usage: run_variant.sh NAME VARIANTS [extra runner args...]
P=$(dirname "$0"); name=$1; var=$2; shift 2
cd /home/pratham/Documents/ChatGPT/DyNAMO-Ridgeback && source /opt/ros/jazzy/setup.bash && source install/setup.bash
PROF_VARIANT=$var TIMING_OUT=$P/$name.json OMNI_KIT_ACCEPT_EULA=YES timeout 600 isaac_venv/bin/python $P/timed_run.py \
  --backend isaac --scene-profile realistic --environment modern_office --scenario explainer \
  --duration 10 --ros-domain 217 --output-dir artifacts/leader-following/fps-probe/$name "$@" > $P/$name.log 2>&1
echo "$name exit=$?"; isaac_venv/bin/python $P/summ.py $P/$name.json
