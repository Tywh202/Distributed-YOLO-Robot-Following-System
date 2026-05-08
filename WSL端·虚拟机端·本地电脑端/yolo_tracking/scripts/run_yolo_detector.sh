#!/bin/bash
source /home/fish/catkin_ws/devel/setup.bash
exec /usr/local/bin/python3.10 /home/fish/catkin_ws/src/yolo_tracking/scripts/yolo_detector_node.py "$@"
