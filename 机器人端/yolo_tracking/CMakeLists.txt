cmake_minimum_required(VERSION 3.0.2)
project(yolo_tracking)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  YoloDetection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/yolo_detection_bridge_node.py
  scripts/yolo_node_release.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
