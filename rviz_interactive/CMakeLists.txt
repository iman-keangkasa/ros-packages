cmake_minimum_required(VERSION 2.8.3)
project(rviz_interactive)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  rospy
  tf
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS interactive_markers roscpp rospy tf visualization_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

