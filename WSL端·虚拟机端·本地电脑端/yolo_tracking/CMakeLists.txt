cmake_minimum_required(VERSION 3.0.2)
project(yolo_tracking)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

################################################
## Declare ROS messages
################################################

add_message_files(
  FILES
  YoloDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare catkin package
################################################

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)

################################################
## Build
################################################

include_directories(
  ${catkin_INCLUDE_DIRS}
)

################################################
## Install Python scripts
################################################

catkin_install_python(PROGRAMS
  scripts/yolo_detector_sender_ui_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



install(PROGRAMS
  scripts/run_yolo_detector.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)