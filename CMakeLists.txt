cmake_minimum_required(VERSION 2.8.3)
project(sbpl_demos)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  tf
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  XYZRPY.msg
)
add_service_files(
  FILES 
  PoseUpsample.srv 
  ReturnJointStates.srv
)
add_action_files(
  FILES
  RoconMoveArm.action
  RoconPick.action
  RoconRobotiq.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs   geometry_msgs  moveit_msgs   sensor_msgs  std_msgs
)

###########
## Build ##
###########

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure



add_executable(pr2_rocon_marker_publisher src/pr2_rocon_marker_publisher.cpp)
add_dependencies(pr2_rocon_marker_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pr2_rocon_marker_publisher ${catkin_LIBRARIES} )

add_executable(pose_upsample_node src/pose_upsample_node.cpp)
add_dependencies(pose_upsample_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_upsample_node ${catkin_LIBRARIES} )

add_executable(pose_upsample_node_perchEnabled src/pose_upsample_node_perchEnabled.cpp)
add_dependencies(pose_upsample_node_perchEnabled ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_upsample_node_perchEnabled ${catkin_LIBRARIES} )
