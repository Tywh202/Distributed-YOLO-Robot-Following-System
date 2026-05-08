#!/bin/bash
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://10.31.108.35:11311
export ROS_IP=10.31.108.35
source /home/spark/catkin_ws/devel/setup.bash
echo "Robot ROS env set."
