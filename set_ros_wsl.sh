#!/bin/bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://10.31.108.35:11311
export ROS_IP=10.24.78.19
source /home/fish/catkin_ws/devel/setup.bash
echo "WSL ROS env set."
