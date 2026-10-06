#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /opt/mybotshop/install/setup.bash
export MMP_NS=r100_0160
export ROBOT_NS=r100_0160
export ROS_DOMAIN_ID=0
export HOME=/home/robot
export PATH=/home/robot/.local/bin:$PATH
exec ros2 launch robot_webserver webserver.launch.py
