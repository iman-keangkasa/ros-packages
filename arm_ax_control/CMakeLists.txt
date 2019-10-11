cmake_minimum_required(VERSION 2.8.3)
project(arm_ax_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  trajectory_msgs
  control_msgs
  sensor_msgs
  actionlib
  message_generation
  dynamixel_sdk
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
)


add_service_files(
  FILES
  ax_joints.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_ax_control
  CATKIN_DEPENDS roscpp
  actionlib
  trajectory_msgs
  control_msgs
  sensor_msgs
  DEPENDS 
  moveit_core
  moveit_ros_planning_interface
  message_runtime
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME}_node  
        src/main.cpp)
	
#add_executable(position_controller_ax src/position_controller_ax.cpp
#	src/position_controller_ax.h)
#add_dependencies(position_controller_ax ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(position_controller_ax ${catkin_LIBRARIES})


 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 #install(TARGETS position_controller_ax
 #   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 #)



 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

